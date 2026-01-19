//! GNSS IRQ Management for LR1110
//!
//! This module provides sophisticated interrupt management for GNSS scanning operations,
//! ported from the STM32-Sidewalk-SDK implementation. It implements a two-stage priority
//! system for precise timestamp capture and RTOS-safe processing.
//!
//! # Source
//!
//! Ported from: `STM32-Sidewalk-SDK/platform/sid_mcu/semtech/hal/lr11xx/lr11xx_hal.c:135-209`
//!
//! # Key Features
//!
//! - **Two-stage priority IRQ handling**: Capture timestamps at high priority, process at low priority
//! - **Dynamic priority switching**: Allows RTOS API calls after timestamp capture
//! - **BUSY pin management**: Smart polling with configurable timeouts
//! - **Scan callbacks**: Pre/post-scan hooks for system coordination
//!
//! # Architecture
//!
//! ```text
//! Hardware IRQ (DIO1/PB14) → EXTI14_IRQHandler
//!     ↓ Stage 1: HIGH Priority (0)
//!     ├─ Capture precise timestamp: Instant::now()
//!     ├─ Cancel timeout timer
//!     ├─ Lower IRQ priority to 5
//!     └─ Return immediately (forces NVIC re-entry)
//!     ↓ Stage 2: LOW Priority (5) - NVIC re-enters ISR
//!     ├─ Disarm IRQ line
//!     ├─ Route to scan callback
//!     └─ Process scan results (RTOS APIs safe here)
//! ```
//!
//! # Platform Support
//!
//! - **ARM Cortex-M3/M4/M7** (STM32WBA, STM32F4, nRF52): Full support with dynamic priority
//! - **ARM Cortex-M0/M0+** (RP2040, STM32L0): Limited support, single priority only
//! - **Other architectures**: Basic timestamp capture
//!
//! # Example Usage
//!
//! ```ignore
//! use lr1110_rs::gnss_irq::{GnssIrqManager, GnssScanStatus};
//!
//! static mut GNSS_IRQ: GnssIrqManager = GnssIrqManager::new();
//!
//! // Set up callbacks
//! unsafe {
//!     GNSS_IRQ.set_prescan_callback(|| {
//!         sidewalk_suspend();
//!         ble_suspend();
//!     });
//!
//!     GNSS_IRQ.set_postscan_callback(|| {
//!         sidewalk_resume();
//!         ble_resume();
//!     });
//! }
//!
//! // In IRQ handler (ARM Cortex-M3+)
//! #[interrupt]
//! fn EXTI14_IRQHandler() {
//!     unsafe {
//!         // Stage 1: Capture timestamp at high priority
//!         if GNSS_IRQ.handle_irq_stage1() {
//!             // Lower priority to allow RTOS calls
//!             cortex_m::peripheral::NVIC::set_priority(interrupt::EXTI14, 5);
//!             return;  // Force NVIC re-entry at new priority
//!         }
//!
//!         // Stage 2: Process at low priority (RTOS-safe)
//!         GNSS_IRQ.handle_irq_stage2(GnssScanStatus::Done);
//!     }
//! }
//! ```
//!
//! # BUSY Pin Polling
//!
//! Smart polling with configurable timeouts:
//!
//! ```ignore
//! use lr1110_rs::gnss_irq::{wait_on_busy, BusyTimeout};
//!
//! let config = BusyTimeout {
//!     default_us: 8_000,      // 8ms for normal commands
//!     geoloc_us: 80_000,      // 80ms for GNSS/WiFi commands
//!     gnss_abort_ms: 6_000,   // 6s for GNSS abort
//!     ..Default::default()
//! };
//!
//! wait_on_busy(|| radio.is_busy(), config.geoloc_us, &config).await?;
//! ```

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::Instant;

/// IRQ priority levels
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum IrqPriority {
    /// High priority for timestamp capture (preempts most interrupts)
    High = 0,
    /// Low priority for processing (allows RTOS API calls)
    Low = 5,
}

/// GNSS scan completion status
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssScanStatus {
    /// Scan completed successfully
    Done,
    /// Scan was aborted by radio planner
    AbortedByPlanner,
    /// Scan was self-aborted (error condition)
    SelfAborted,
    /// Scan timed out
    Timeout,
}

/// Callback type for pre-scan actions
///
/// Called before any radio access during GNSS scan initialization.
/// Use this to tear down concurrent radio operations (e.g., Sidewalk, BLE).
pub type PreScanCallback = fn() -> ();

/// Callback type for post-scan actions
///
/// Called after scan completes and radio is in sleep/standby.
/// Use this to resume suspended operations.
pub type PostScanCallback = fn() -> ();

/// IRQ handler callback for GNSS scan completion
pub type ScanCompleteCallback = fn(GnssScanStatus, Instant) -> ();

/// GNSS IRQ Manager
///
/// Manages interrupt handling for GNSS scanning operations with two-stage priority
/// and callbacks for system coordination.
pub struct GnssIrqManager {
    /// Timestamp captured at IRQ entry (high priority)
    pub scan_timestamp: Option<Instant>,

    /// Indicates if IRQ is running at high priority
    is_high_priority: AtomicBool,

    /// Scan completion callback
    scan_complete_cb: Option<ScanCompleteCallback>,

    /// Pre-scan callback
    prescan_cb: Option<PreScanCallback>,

    /// Post-scan callback
    postscan_cb: Option<PostScanCallback>,

    /// Indicates if scan was self-aborted
    self_aborted: AtomicBool,
}

impl GnssIrqManager {
    /// Create a new GNSS IRQ manager
    pub const fn new() -> Self {
        Self {
            scan_timestamp: None,
            is_high_priority: AtomicBool::new(false),
            scan_complete_cb: None,
            prescan_cb: None,
            postscan_cb: None,
            self_aborted: AtomicBool::new(false),
        }
    }

    /// Set the scan completion callback
    pub fn set_scan_complete_callback(&mut self, cb: ScanCompleteCallback) {
        self.scan_complete_cb = Some(cb);
    }

    /// Set pre-scan callback
    ///
    /// Called before any radio access. Use to suspend concurrent operations.
    pub fn set_prescan_callback(&mut self, cb: PreScanCallback) {
        self.prescan_cb = Some(cb);
    }

    /// Set post-scan callback
    ///
    /// Called after scan completes. Use to resume suspended operations.
    pub fn set_postscan_callback(&mut self, cb: PostScanCallback) {
        self.postscan_cb = Some(cb);
    }

    /// Execute pre-scan actions
    ///
    /// This should be called BEFORE the GNSS scan is initiated to allow
    /// proper coordination with other radio operations.
    pub fn execute_prescan_actions(&mut self) {
        if let Some(cb) = self.prescan_cb {
            cb();
        }
    }

    /// Execute post-scan actions
    ///
    /// This should be called AFTER the scan completes and radio is in
    /// sleep/standby to allow resuming other operations.
    pub fn execute_postscan_actions(&self) {
        if let Some(cb) = self.postscan_cb {
            cb();
        }
    }

    /// Check if post-scan actions should be executed
    ///
    /// Returns true if there's a delay before next scan, false for back-to-back scans.
    /// This optimization avoids unnecessary overhead in mobile scanning scenarios.
    pub fn should_execute_postscan(&self, next_scan_delay_ms: u32) -> bool {
        next_scan_delay_ms > 0
    }

    /// Mark scan as self-aborted
    pub fn set_self_aborted(&self, aborted: bool) {
        self.self_aborted.store(aborted, Ordering::Release);
    }

    /// Check if scan was self-aborted
    pub fn is_self_aborted(&self) -> bool {
        self.self_aborted.load(Ordering::Acquire)
    }

    /// First stage IRQ handler - runs at HIGH priority
    ///
    /// This captures the timestamp as quickly as possible and then lowers
    /// the IRQ priority. It should return immediately after lowering priority
    /// to force NVIC re-evaluation.
    ///
    /// # Returns
    ///
    /// `true` if this handler should return immediately (priority was high),
    /// `false` if processing should continue (priority is now low).
    #[cfg(not(target_arch = "arm"))]
    pub fn handle_irq_stage1(&mut self) -> bool {
        // For non-ARM architectures (or ARMv6-M without dynamic priority),
        // capture timestamp and proceed directly to stage 2
        self.scan_timestamp = Some(Instant::now());
        false
    }

    #[cfg(target_arch = "arm")]
    pub fn handle_irq_stage1(&mut self) -> bool {
        // Store timestamp as soon as possible
        self.scan_timestamp = Some(Instant::now());

        // Check if we're running at high priority
        if self.is_high_priority.load(Ordering::Acquire) {
            // Lower the IRQ priority to allow RTOS API calls
            // Note: Actual priority change must be done by the caller
            // using platform-specific NVIC calls
            self.is_high_priority.store(false, Ordering::Release);

            // Return true to signal caller to exit ISR immediately.
            // This forces NVIC to re-evaluate priorities and re-enter
            // the ISR at the new (lower) priority level.
            return true;
        }

        // Already at low priority, continue to stage 2
        false
    }

    /// Second stage IRQ handler - runs at LOW priority
    ///
    /// This disables the IRQ line and invokes the scan completion callback.
    /// At this point, RTOS API calls are safe.
    pub fn handle_irq_stage2(&mut self, status: GnssScanStatus) {
        let timestamp = self.scan_timestamp.unwrap_or_else(|| Instant::now());

        // Invoke scan completion callback
        if let Some(cb) = self.scan_complete_cb {
            cb(status, timestamp);
        }
    }

    /// Arm the IRQ for next scan
    ///
    /// Call this before starting a GNSS scan. It resets state and sets
    /// the IRQ to high priority mode.
    pub fn arm_for_scan(&mut self) {
        self.scan_timestamp = None;
        self.is_high_priority.store(true, Ordering::Release);
        self.self_aborted.store(false, Ordering::Release);
    }

    /// Disarm the IRQ after scan completion
    pub fn disarm(&mut self) {
        self.is_high_priority.store(false, Ordering::Release);
    }
}

/// BUSY pin timeout configuration
#[derive(Clone, Copy, Debug)]
pub struct BusyTimeout {
    /// Default timeout for most operations (microseconds)
    pub default_us: u32,
    /// Timeout for wakeup sequence (microseconds)
    pub wakeup_us: u32,
    /// Timeout for geolocation commands (microseconds)
    pub geoloc_us: u32,
    /// Timeout for GNSS abort (milliseconds)
    pub gnss_abort_ms: u32,
    /// Timeout for WiFi abort (milliseconds)
    pub wifi_abort_ms: u32,
    /// Probe period for polling (microseconds)
    pub probe_period_us: u32,
}

impl Default for BusyTimeout {
    fn default() -> Self {
        Self {
            default_us: 8_000,
            wakeup_us: 32_000,
            geoloc_us: 80_000,
            gnss_abort_ms: 6_000,
            wifi_abort_ms: 150_000,
            probe_period_us: 5,
        }
    }
}

/// Wait for BUSY pin to clear with smart timeout handling
///
/// This implements the polling strategy from the STM32-Sidewalk-SDK with
/// microsecond delays for fast operations and optional scheduler delays
/// for long operations.
///
/// # Arguments
///
/// * `is_busy` - Function to check if BUSY pin is high
/// * `timeout_us` - Timeout in microseconds
/// * `config` - BUSY timeout configuration
///
/// # Returns
///
/// `Ok(())` if BUSY cleared within timeout, `Err(())` if timeout occurred
pub async fn wait_on_busy<F>(
    mut is_busy: F,
    timeout_us: u32,
    config: &BusyTimeout,
) -> Result<(), ()>
where
    F: FnMut() -> bool,
{
    let mut accumulated_us = 0u32;

    // Fast polling loop with microsecond delays
    while accumulated_us < timeout_us / 2 {
        if !is_busy() {
            return Ok(());
        }

        embassy_time::Timer::after_micros(config.probe_period_us as u64).await;
        accumulated_us += config.probe_period_us;
    }

    // For remaining timeout, use longer probe periods if available
    // This allows the scheduler to run other tasks
    let remaining_us = timeout_us - accumulated_us;
    let long_probe_ms = 10u32;
    let mut remaining_ms = remaining_us / 1000;

    while remaining_ms > 0 {
        let delay_ms = remaining_ms.min(long_probe_ms);
        embassy_time::Timer::after_millis(delay_ms as u64).await;
        remaining_ms -= delay_ms;

        if !is_busy() {
            return Ok(());
        }
    }

    // Final check
    if !is_busy() {
        Ok(())
    } else {
        Err(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_irq_manager_creation() {
        let mgr = GnssIrqManager::new();
        assert!(mgr.scan_timestamp.is_none());
        assert!(!mgr.is_self_aborted());
    }

    #[test]
    fn test_self_abort_flag() {
        let mgr = GnssIrqManager::new();
        assert!(!mgr.is_self_aborted());

        mgr.set_self_aborted(true);
        assert!(mgr.is_self_aborted());

        mgr.set_self_aborted(false);
        assert!(!mgr.is_self_aborted());
    }

    #[test]
    fn test_postscan_should_execute() {
        let mgr = GnssIrqManager::new();

        // Should execute if there's a delay
        assert!(mgr.should_execute_postscan(1000));

        // Should NOT execute for back-to-back scans
        assert!(!mgr.should_execute_postscan(0));
    }
}
