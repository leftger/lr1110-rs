//! Radio Planner for LR1110
//!
//! Coordinates access to the LR1110 radio between different operations
//! (LoRa TX/RX, GNSS scan, WiFi scan, etc.) to prevent conflicts.
//!
//! This is a simplified version of the LoRa Basics Modem radio planner,
//! adapted for use in embedded-hal-async environments.
//!
//! # Features
//!
//! - Task scheduling with priorities
//! - Radio state management
//! - Conflict resolution for concurrent operations
//! - Task abortion support
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::radio_planner::{RadioPlanner, RadioTask, TaskType, TaskPriority};
//!
//! let mut planner = RadioPlanner::new();
//!
//! // Schedule a GNSS scan
//! let task = RadioTask {
//!     task_type: TaskType::GnssScan,
//!     priority: TaskPriority::High,
//!     duration_ms: 20_000,
//!     start_time_ms: planner.get_current_time_ms() + 100,
//! };
//!
//! planner.enqueue_task(task)?;
//! ```

use embassy_time::Instant;

/// Radio task types
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TaskType {
    /// LoRa TX operation
    LoraTx,
    /// LoRa RX operation
    LoraRx,
    /// GNSS scan
    GnssScan,
    /// WiFi scan
    WifiScan,
    /// Ranging operation
    Ranging,
    /// Idle/standby
    Idle,
}

/// Task priority levels
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TaskPriority {
    /// Low priority - can be preempted
    Low = 0,
    /// Medium priority
    Medium = 1,
    /// High priority - preempts lower priority tasks
    High = 2,
    /// Critical priority - cannot be preempted
    Critical = 3,
}

/// Radio task status
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TaskStatus {
    /// Task is scheduled but not started
    Pending,
    /// Task is currently executing
    Running,
    /// Task completed successfully
    Completed,
    /// Task was aborted
    Aborted,
    /// Task failed with error
    Failed,
}

/// Radio task descriptor
#[derive(Clone, Copy, Debug)]
pub struct RadioTask {
    /// Type of radio operation
    pub task_type: TaskType,

    /// Task priority
    pub priority: TaskPriority,

    /// Estimated duration in milliseconds
    pub duration_ms: u32,

    /// Earliest start time (milliseconds since boot)
    /// Set to 0 for immediate execution (ASAP)
    pub start_time_ms: u64,

    /// Task status
    pub status: TaskStatus,

    /// Task ID (assigned by planner)
    pub id: u32,
}

impl RadioTask {
    /// Create a new radio task
    pub fn new(task_type: TaskType, priority: TaskPriority, duration_ms: u32) -> Self {
        Self {
            task_type,
            priority,
            duration_ms,
            start_time_ms: 0, // ASAP
            status: TaskStatus::Pending,
            id: 0,
        }
    }

    /// Set start time for scheduled execution
    pub fn with_start_time(mut self, start_time_ms: u64) -> Self {
        self.start_time_ms = start_time_ms;
        self
    }

    /// Check if task should start now
    pub fn should_start(&self, current_time_ms: u64) -> bool {
        self.start_time_ms == 0 || current_time_ms >= self.start_time_ms
    }
}

/// Error types for radio planner
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PlannerError {
    /// Task queue is full
    QueueFull,
    /// Invalid task parameters
    InvalidTask,
    /// Task not found
    TaskNotFound,
    /// Radio is busy with higher priority task
    RadioBusy,
    /// Task was aborted
    TaskAborted,
}

/// Maximum number of queued tasks
const MAX_TASKS: usize = 8;

/// Radio Planner
///
/// Manages scheduling and execution of radio operations to prevent conflicts.
pub struct RadioPlanner {
    /// Task queue
    tasks: [Option<RadioTask>; MAX_TASKS],

    /// Currently running task ID
    current_task_id: Option<u32>,

    /// Next task ID to assign
    next_task_id: u32,

    /// Planner start time
    start_time: Instant,

    /// Margin delay for ASAP tasks (milliseconds)
    /// Tasks should start within this window or be cancelled
    pub margin_delay_ms: u32,
}

impl RadioPlanner {
    /// Create a new radio planner
    pub fn new() -> Self {
        Self {
            tasks: [None; MAX_TASKS],
            current_task_id: None,
            next_task_id: 1,
            start_time: Instant::now(),
            margin_delay_ms: 100, // 100ms default margin
        }
    }

    /// Get current time in milliseconds since planner creation
    pub fn get_current_time_ms(&self) -> u64 {
        self.start_time.elapsed().as_millis()
    }

    /// Enqueue a new task
    ///
    /// # Returns
    ///
    /// Task ID on success, or `PlannerError` if queue is full or task is invalid
    pub fn enqueue_task(&mut self, mut task: RadioTask) -> Result<u32, PlannerError> {
        // Validate task
        if task.duration_ms == 0 {
            return Err(PlannerError::InvalidTask);
        }

        // Find empty slot
        let slot = self
            .tasks
            .iter_mut()
            .find(|t| t.is_none())
            .ok_or(PlannerError::QueueFull)?;

        // Assign ID and store task
        task.id = self.next_task_id;
        task.status = TaskStatus::Pending;
        self.next_task_id = self.next_task_id.wrapping_add(1);

        *slot = Some(task);
        Ok(task.id)
    }

    /// Get the next task that should run
    ///
    /// Returns the highest priority task that is ready to start.
    pub fn get_next_task(&mut self) -> Option<&mut RadioTask> {
        let current_time_ms = self.get_current_time_ms();

        self.tasks
            .iter_mut()
            .filter_map(|slot| slot.as_mut())
            .filter(|task| task.status == TaskStatus::Pending && task.should_start(current_time_ms))
            .max_by_key(|task| task.priority as u8)
    }

    /// Start a task
    pub fn start_task(&mut self, task_id: u32) -> Result<(), PlannerError> {
        let task = self
            .tasks
            .iter_mut()
            .filter_map(|slot| slot.as_mut())
            .find(|t| t.id == task_id)
            .ok_or(PlannerError::TaskNotFound)?;

        if task.status != TaskStatus::Pending {
            return Err(PlannerError::InvalidTask);
        }

        task.status = TaskStatus::Running;
        self.current_task_id = Some(task_id);
        Ok(())
    }

    /// Complete a task successfully
    pub fn complete_task(&mut self, task_id: u32) -> Result<(), PlannerError> {
        self.set_task_status(task_id, TaskStatus::Completed)
    }

    /// Abort a task
    pub fn abort_task(&mut self, task_id: u32) -> Result<(), PlannerError> {
        self.set_task_status(task_id, TaskStatus::Aborted)
    }

    /// Fail a task
    pub fn fail_task(&mut self, task_id: u32) -> Result<(), PlannerError> {
        self.set_task_status(task_id, TaskStatus::Failed)
    }

    /// Set task status and clean up if final state
    fn set_task_status(&mut self, task_id: u32, status: TaskStatus) -> Result<(), PlannerError> {
        // Find task slot
        let task_slot = self
            .tasks
            .iter_mut()
            .find(|slot| slot.as_ref().map(|t| t.id == task_id).unwrap_or(false))
            .ok_or(PlannerError::TaskNotFound)?;

        if let Some(task) = task_slot.as_mut() {
            task.status = status;

            // Clear current task if this was it
            if self.current_task_id == Some(task_id) {
                self.current_task_id = None;
            }

            // Remove completed/aborted/failed tasks to free slots
            if matches!(
                status,
                TaskStatus::Completed | TaskStatus::Aborted | TaskStatus::Failed
            ) {
                *task_slot = None;
            }
        }

        Ok(())
    }

    /// Get current running task
    pub fn get_current_task(&self) -> Option<&RadioTask> {
        self.current_task_id.and_then(|id| {
            self.tasks
                .iter()
                .filter_map(|slot| slot.as_ref())
                .find(|t| t.id == id)
        })
    }

    /// Check if a specific task type is running
    pub fn is_task_type_running(&self, task_type: TaskType) -> bool {
        self.get_current_task()
            .map(|t| t.task_type == task_type)
            .unwrap_or(false)
    }

    /// Check if radio is available for a given priority
    pub fn is_radio_available(&self, priority: TaskPriority) -> bool {
        match self.get_current_task() {
            None => true,
            Some(current) => current.priority < priority,
        }
    }

    /// Preempt current task if new task has higher priority
    ///
    /// Returns the ID of the preempted task, if any
    pub fn try_preempt(&mut self, new_priority: TaskPriority) -> Option<u32> {
        if let Some(current) = self.get_current_task() {
            if new_priority > current.priority {
                let task_id = current.id;
                let _ = self.abort_task(task_id);
                return Some(task_id);
            }
        }
        None
    }

    /// Get count of pending tasks
    pub fn pending_task_count(&self) -> usize {
        self.tasks
            .iter()
            .filter(|slot| {
                slot.as_ref()
                    .map(|t| t.status == TaskStatus::Pending)
                    .unwrap_or(false)
            })
            .count()
    }

    /// Clear all tasks
    pub fn clear_all_tasks(&mut self) {
        self.tasks = [None; MAX_TASKS];
        self.current_task_id = None;
    }
}

impl Default for RadioPlanner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_planner_creation() {
        let planner = RadioPlanner::new();
        assert_eq!(planner.pending_task_count(), 0);
        assert!(planner.get_current_task().is_none());
    }

    #[test]
    fn test_enqueue_task() {
        let mut planner = RadioPlanner::new();
        let task = RadioTask::new(TaskType::GnssScan, TaskPriority::High, 10_000);

        let task_id = planner.enqueue_task(task).unwrap();
        assert_eq!(planner.pending_task_count(), 1);
        assert_eq!(task_id, 1);
    }

    #[test]
    fn test_task_priority() {
        let mut planner = RadioPlanner::new();

        let low_task = RadioTask::new(TaskType::LoraRx, TaskPriority::Low, 1000);
        let high_task = RadioTask::new(TaskType::GnssScan, TaskPriority::High, 10_000);

        planner.enqueue_task(low_task).unwrap();
        planner.enqueue_task(high_task).unwrap();

        // Should get high priority task first
        let next = planner.get_next_task().unwrap();
        assert_eq!(next.task_type, TaskType::GnssScan);
    }

    #[test]
    fn test_task_lifecycle() {
        let mut planner = RadioPlanner::new();
        let task = RadioTask::new(TaskType::GnssScan, TaskPriority::High, 10_000);

        let task_id = planner.enqueue_task(task).unwrap();
        planner.start_task(task_id).unwrap();
        assert!(planner.get_current_task().is_some());

        planner.complete_task(task_id).unwrap();
        assert!(planner.get_current_task().is_none());
        assert_eq!(planner.pending_task_count(), 0);
    }

    #[test]
    fn test_preemption() {
        let mut planner = RadioPlanner::new();
        let low_task = RadioTask::new(TaskType::LoraRx, TaskPriority::Low, 1000);

        let task_id = planner.enqueue_task(low_task).unwrap();
        planner.start_task(task_id).unwrap();

        // Try to preempt with high priority
        let preempted = planner.try_preempt(TaskPriority::High);
        assert_eq!(preempted, Some(task_id));
        assert!(planner.get_current_task().is_none());
    }
}
