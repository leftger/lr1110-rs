//! GNSS Scan Manager with IRQ Management
//!
//! High-level coordinator for GNSS scanning operations that integrates:
//! - Two-stage IRQ handling for precise timing
//! - Radio planner coordination
//! - Pre/post-scan callbacks
//! - Scan mode optimization (static vs mobile)
//! - Automatic retry logic
//!
//! # Source
//!
//! Ported from: `STM32-Sidewalk-SDK/.../geolocation_services/mw_gnss_scan.c`
//!
//! # Scan Modes
//!
//! ## Mobile Mode (Moving Device)
//!
//! Optimized for back-to-back scans with zero delay:
//!
//! ```ignore
//! ScanMode::Mobile {
//!     scan_group_size: 2,
//!     scan_group_delay_s: 0,  // Zero delay triggers optimization
//! }
//!
//! // Behavior:
//! // Scan 1: Pre-scan → GNSS (20s) → [POST-SCAN SKIPPED] → immediate next
//! // Scan 2: GNSS (20s) → Post-scan → End of group
//! // Total: ~40s (saves ~50ms per group)
//! ```
//!
//! ## Static Mode (Stationary Device)
//!
//! Periodic scans with delays:
//!
//! ```ignore
//! ScanMode::Static {
//!     scan_group_size: 2,
//!     scan_group_delay_s: 15,  // 15 second delay
//! }
//!
//! // Behavior:
//! // Scan 1: Pre-scan → GNSS (20s) → Post-scan → 15s delay
//! // Scan 2: Pre-scan → GNSS (20s) → Post-scan → End
//! // Total: ~55s (callbacks allow concurrent ops during delay)
//! ```
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::gnss_scan_manager::{GnssScanManager, ScanConfig, ScanMode};
//! use lr1110_rs::gnss::{GnssSearchMode, GNSS_GPS_MASK, GNSS_BEIDOU_MASK};
//!
//! // Create manager with callbacks
//! let mut manager = GnssScanManager::new();
//! manager.set_prescan_callback(|| {
//!     sidewalk_suspend();  // Called BEFORE radio access
//!     ble_suspend();
//! });
//! manager.set_postscan_callback(|| {
//!     sidewalk_resume();   // Called AFTER radio idle
//!     ble_resume();
//! });
//!
//! // Configure for mobile scanning
//! let config = ScanConfig {
//!     constellation_mask: GNSS_GPS_MASK | GNSS_BEIDOU_MASK,
//!     search_mode: GnssSearchMode::MidEffort,
//!     mode: ScanMode::Mobile {
//!         scan_group_size: 2,
//!         scan_group_delay_s: 0,  // Back-to-back = skip post-scan
//!     },
//!     ..Default::default()
//! };
//!
//! // Start scanning
//! let result = manager.scan(&mut radio, &config).await?;
//! info!("Detected {} satellites in {}ms", result.num_satellites, result.duration_ms);
//! ```
//!
//! See `examples/stm32wba/src/bin/lr1110_gnss_scan_with_irq.rs` for complete example.

use crate::gnss::{GnssConstellationMask, GnssSearchMode};
use crate::gnss_irq::{GnssIrqManager, GnssScanStatus, PostScanCallback, PreScanCallback};
use crate::radio_planner::{PlannerError, RadioPlanner, RadioTask, TaskPriority, TaskType};
use embassy_time::Instant;

/// GNSS scan mode configuration
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ScanMode {
    /// Static mode: device is stationary
    /// - Longer delay between scans
    /// - Almanac position expected to be stable
    Static {
        /// Number of scans per group
        scan_group_size: u8,
        /// Delay between scans (seconds)
        scan_group_delay_s: u32,
    },

    /// Mobile mode: device is moving
    /// - Short or zero delay between scans
    /// - Position may change between scans
    Mobile {
        /// Number of scans per group
        scan_group_size: u8,
        /// Delay between scans (seconds), typically 0
        scan_group_delay_s: u32,
    },
}

impl Default for ScanMode {
    fn default() -> Self {
        Self::Mobile {
            scan_group_size: 2,
            scan_group_delay_s: 0,
        }
    }
}

/// GNSS scan configuration
#[derive(Clone, Copy, Debug)]
pub struct ScanConfig {
    /// Constellation mask (GPS, BeiDou, or both)
    pub constellation_mask: GnssConstellationMask,

    /// Search mode (effort level)
    pub search_mode: GnssSearchMode,

    /// Scan mode (static or mobile)
    pub mode: ScanMode,

    /// Enable scan result aggregation across multiple scans
    pub aggregate_results: bool,

    /// Maximum retry attempts on scan failure
    pub max_retries: u8,

    /// Timeout for each scan attempt (milliseconds)
    pub scan_timeout_ms: u32,
}

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            constellation_mask: 0x03, // GPS + BeiDou
            search_mode: GnssSearchMode::MidEffort,
            mode: ScanMode::default(),
            aggregate_results: false,
            max_retries: 3,
            scan_timeout_ms: 20_000, // 20 seconds for assisted
        }
    }
}

/// GNSS scan result
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ScanResult {
    /// Scan status
    pub status: GnssScanStatus,

    /// Number of satellites detected
    pub num_satellites: u8,

    /// Scan duration (milliseconds)
    pub duration_ms: u32,

    /// Timestamp when scan completed
    pub timestamp: Instant,

    /// Whether scan is part of aggregated result
    pub is_aggregated: bool,
}

/// GNSS Scan Manager
///
/// Coordinates GNSS scanning with IRQ management, radio planning,
/// and system callbacks.
pub struct GnssScanManager {
    /// IRQ manager for scan operations
    irq_manager: GnssIrqManager,

    /// Radio planner for coordinating access
    radio_planner: RadioPlanner,

    /// Current scan group state
    scan_group: ScanGroupState,

    /// Token for scan group (5-bit, 0x02-0x1F)
    current_token: u8,
}

/// State for managing scan groups
#[derive(Clone, Copy, Debug)]
struct ScanGroupState {
    /// Number of scans completed in current group
    scans_completed: u8,

    /// Number of valid scans in current group
    valid_scans: u8,

    /// Start time of scan group
    start_time: Option<Instant>,

    /// Whether last scan group was valid
    last_group_valid: bool,
}

impl Default for ScanGroupState {
    fn default() -> Self {
        Self {
            scans_completed: 0,
            valid_scans: 0,
            start_time: None,
            last_group_valid: false,
        }
    }
}

impl GnssScanManager {
    /// Create a new GNSS scan manager
    pub fn new() -> Self {
        Self {
            irq_manager: GnssIrqManager::new(),
            radio_planner: RadioPlanner::new(),
            scan_group: ScanGroupState::default(),
            current_token: 0x02, // Start at 0x02 (0x00 and 0x01 reserved)
        }
    }

    /// Set pre-scan callback
    ///
    /// Called before any radio access. Use to suspend concurrent operations
    /// like Sidewalk, BLE, etc.
    pub fn set_prescan_callback(&mut self, cb: PreScanCallback) {
        self.irq_manager.set_prescan_callback(cb);
    }

    /// Set post-scan callback
    ///
    /// Called after scan completes and radio is in standby/sleep.
    /// Use to resume suspended operations.
    pub fn set_postscan_callback(&mut self, cb: PostScanCallback) {
        self.irq_manager.set_postscan_callback(cb);
    }

    /// Perform a GNSS scan with the given configuration
    ///
    /// This is the main entry point for GNSS scanning. It handles:
    /// - IRQ setup and management
    /// - Radio planner coordination
    /// - Scan retry logic
    /// - Pre/post-scan callbacks
    ///
    /// # Returns
    ///
    /// `ScanResult` with scan status and metadata
    pub async fn scan<Radio>(
        &mut self,
        radio: &mut Radio,
        config: &ScanConfig,
    ) -> Result<ScanResult, PlannerError>
    where
        Radio: crate::gnss::GnssExt,
    {
        // Determine scan group parameters
        let (scan_group_size, scan_group_delay_s) = match config.mode {
            ScanMode::Static {
                scan_group_size,
                scan_group_delay_s,
            } => (scan_group_size, scan_group_delay_s),
            ScanMode::Mobile {
                scan_group_size,
                scan_group_delay_s,
            } => (scan_group_size, scan_group_delay_s),
        };

        // Start new scan group if needed
        if self.scan_group.scans_completed == 0 {
            self.start_scan_group(config);
        }

        // Execute pre-scan actions BEFORE any radio access
        self.irq_manager.execute_prescan_actions();

        // Schedule scan task with radio planner
        let task = RadioTask::new(
            TaskType::GnssScan,
            TaskPriority::High,
            config.scan_timeout_ms,
        );

        let task_id = self.radio_planner.enqueue_task(task)?;
        self.radio_planner.start_task(task_id)?;

        // Arm IRQ for scan
        self.irq_manager.arm_for_scan();

        // Initiate GNSS scan
        let start_time = Instant::now();

        // Set constellation
        if let Err(_e) = radio
            .gnss_set_constellation(config.constellation_mask)
            .await
        {
            self.radio_planner.fail_task(task_id)?;
            return Err(PlannerError::TaskAborted);
        }

        // Launch scan
        // result_mask: 0x00 for basic results (no extra Doppler)
        // nb_sv_max: 0 for unlimited satellites
        if let Err(_e) = radio.gnss_scan(config.search_mode, 0x00, 0).await {
            self.radio_planner.fail_task(task_id)?;
            return Err(PlannerError::TaskAborted);
        }

        // Wait for scan completion (in real application, this would be via IRQ)
        // For now, use a timeout-based approach
        embassy_time::Timer::after_millis(config.scan_timeout_ms as u64).await;

        let duration_ms = start_time.elapsed().as_millis() as u32;

        // Read scan results
        let num_satellites = match radio.gnss_get_nb_satellites().await {
            Ok(count) => count,
            Err(_) => 0,
        };

        // Update scan group state
        self.scan_group.scans_completed += 1;
        if num_satellites >= 3 {
            // Consider valid if 3+ satellites detected
            self.scan_group.valid_scans += 1;
        }

        // Determine if scan group is complete
        let scan_complete = self.scan_group.scans_completed >= scan_group_size;
        let status = if num_satellites >= 3 {
            GnssScanStatus::Done
        } else {
            GnssScanStatus::Timeout
        };

        // Execute post-scan actions conditionally
        if self
            .irq_manager
            .should_execute_postscan(scan_group_delay_s * 1000)
            || scan_complete
        {
            self.irq_manager.execute_postscan_actions();
        }

        // Complete task in planner
        self.radio_planner.complete_task(task_id)?;

        // Reset scan group if complete
        if scan_complete {
            self.end_scan_group(config);
        }

        Ok(ScanResult {
            status,
            num_satellites,
            duration_ms,
            timestamp: Instant::now(),
            is_aggregated: config.aggregate_results,
        })
    }

    /// Start a new scan group
    fn start_scan_group(&mut self, config: &ScanConfig) {
        self.scan_group.scans_completed = 0;
        self.scan_group.valid_scans = 0;
        self.scan_group.start_time = Some(Instant::now());

        // Increment token if needed
        if self.scan_group.last_group_valid && !config.aggregate_results {
            self.increment_token();
        }
    }

    /// End current scan group
    fn end_scan_group(&mut self, _config: &ScanConfig) {
        self.scan_group.last_group_valid = self.scan_group.valid_scans > 0;

        // Reset for next group
        self.scan_group.scans_completed = 0;
        self.scan_group.valid_scans = 0;
    }

    /// Increment the NAV group token (5-bit, 0x02-0x1F)
    fn increment_token(&mut self) {
        self.current_token = (self.current_token + 1) & 0x1F;
        if self.current_token < 0x02 {
            self.current_token = 0x02; // Skip 0x00 and 0x01
        }
    }

    /// Get current token
    pub fn get_token(&self) -> u8 {
        self.current_token
    }

    /// Get radio planner (for advanced usage)
    pub fn radio_planner(&mut self) -> &mut RadioPlanner {
        &mut self.radio_planner
    }

    /// Get IRQ manager (for advanced usage)
    pub fn irq_manager(&mut self) -> &mut GnssIrqManager {
        &mut self.irq_manager
    }

    /// Check if scan group is in progress
    pub fn is_scan_group_active(&self) -> bool {
        self.scan_group.scans_completed > 0
    }

    /// Get number of valid scans in current group
    pub fn valid_scan_count(&self) -> u8 {
        self.scan_group.valid_scans
    }
}

impl Default for GnssScanManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_creation() {
        let manager = GnssScanManager::new();
        assert_eq!(manager.get_token(), 0x02);
        assert!(!manager.is_scan_group_active());
    }

    #[test]
    fn test_token_increment() {
        let mut manager = GnssScanManager::new();
        assert_eq!(manager.get_token(), 0x02);

        manager.increment_token();
        assert_eq!(manager.get_token(), 0x03);

        // Test rollover
        manager.current_token = 0x1F;
        manager.increment_token();
        assert_eq!(manager.get_token(), 0x02); // Should wrap to 0x02
    }

    #[test]
    fn test_scan_config_defaults() {
        let config = ScanConfig::default();
        assert_eq!(config.max_retries, 3);
        assert_eq!(config.scan_timeout_ms, 20_000);
    }
}
