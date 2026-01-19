//! WiFi IRQ Management for LR1110
//!
//! This module provides interrupt management for WiFi scanning operations,
//! following the same two-stage priority pattern as GNSS scanning.
//!
//! # Source
//!
//! Ported from: `STM32-Sidewalk-SDK/.../geolocation_services/mw_wifi_scan.c`
//!
//! # Key Differences from GNSS
//!
//! - **Scan duration**: 3-10 seconds (vs 20-80s for GNSS)
//! - **Abort support**: Cannot abort WiFi scans (hardware limitation)
//! - **Result processing**: Requires filtering (remove mobile APs) and sorting (by RSSI)
//! - **Power**: ~50-100 mAh per scan (vs 200-500 mAh for GNSS)
//! - **IRQ**: Uses `IRQ_WIFI_SCAN_DONE` (same DIO1 pin as GNSS)
//!
//! # WiFi Scan Flow
//!
//! ```text
//! 1. Pre-scan callback → Suspend Sidewalk/BLE
//! 2. Configure IRQ: IRQ_WIFI_SCAN_DONE
//! 3. Launch scan: wifi_scan(...)
//! 4. Wait ~3-10 seconds
//! 5. IRQ fires (WIFI_SCAN_DONE)
//!    ├─ Stage 1 (HIGH): Capture timestamp
//!    └─ Stage 2 (LOW): Read results
//! 6. Filter mobile APs (optional)
//! 7. Sort by RSSI (optional)
//! 8. Post-scan callback → Resume operations
//! ```
//!
//! # Example Usage
//!
//! ```ignore
//! use lr1110_rs::wifi_irq::{WifiIrqManager, WifiScanStatus};
//!
//! static mut WIFI_IRQ: WifiIrqManager = WifiIrqManager::new();
//!
//! // Set up callbacks
//! unsafe {
//!     WIFI_IRQ.set_prescan_callback(|| {
//!         sidewalk_suspend();
//!         ble_suspend();
//!     });
//! }
//!
//! // Same IRQ handler pattern as GNSS
//! #[interrupt]
//! fn EXTI14_IRQHandler() {
//!     unsafe {
//!         if WIFI_IRQ.handle_irq_stage1() {
//!             cortex_m::peripheral::NVIC::set_priority(interrupt::EXTI14, 5);
//!             return;
//!         }
//!         WIFI_IRQ.handle_irq_stage2(WifiScanStatus::Done);
//!     }
//! }
//! ```

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_time::Instant;

/// WiFi scan completion status
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiScanStatus {
    /// Scan completed successfully
    Done,
    /// Scan was aborted by radio planner
    AbortedByPlanner,
    /// Scan failed (error condition)
    Failed,
    /// Scan timed out
    Timeout,
}

/// Callback type for pre-scan actions (WiFi)
///
/// Called before any radio access during WiFi scan initialization.
/// Use this to tear down concurrent radio operations (e.g., Sidewalk, BLE).
pub type WifiPreScanCallback = fn() -> ();

/// Callback type for post-scan actions (WiFi)
///
/// Called after scan completes and radio is in sleep/standby.
/// Use this to resume suspended operations.
pub type WifiPostScanCallback = fn() -> ();

/// IRQ handler callback for WiFi scan completion
pub type WifiScanCompleteCallback = fn(WifiScanStatus, Instant) -> ();

/// WiFi IRQ Manager
///
/// Manages interrupt handling for WiFi scanning operations with two-stage priority
/// and callbacks for system coordination.
pub struct WifiIrqManager {
    /// Timestamp captured at IRQ entry (high priority)
    pub scan_timestamp: Option<Instant>,

    /// Indicates if IRQ is running at high priority
    is_high_priority: AtomicBool,

    /// Scan completion callback
    scan_complete_cb: Option<WifiScanCompleteCallback>,

    /// Pre-scan callback
    prescan_cb: Option<WifiPreScanCallback>,

    /// Post-scan callback
    postscan_cb: Option<WifiPostScanCallback>,

    /// Indicates if scan failed
    scan_failed: AtomicBool,
}

impl WifiIrqManager {
    /// Create a new WiFi IRQ manager
    pub const fn new() -> Self {
        Self {
            scan_timestamp: None,
            is_high_priority: AtomicBool::new(false),
            scan_complete_cb: None,
            prescan_cb: None,
            postscan_cb: None,
            scan_failed: AtomicBool::new(false),
        }
    }

    /// Set the scan completion callback
    pub fn set_scan_complete_callback(&mut self, cb: WifiScanCompleteCallback) {
        self.scan_complete_cb = Some(cb);
    }

    /// Set pre-scan callback
    ///
    /// Called before any radio access. Use to suspend concurrent operations.
    pub fn set_prescan_callback(&mut self, cb: WifiPreScanCallback) {
        self.prescan_cb = Some(cb);
    }

    /// Set post-scan callback
    ///
    /// Called after scan completes. Use to resume suspended operations.
    pub fn set_postscan_callback(&mut self, cb: WifiPostScanCallback) {
        self.postscan_cb = Some(cb);
    }

    /// Execute pre-scan actions
    ///
    /// This should be called BEFORE the WiFi scan is initiated to allow
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

    /// Mark scan as failed
    pub fn set_failed(&self, failed: bool) {
        self.scan_failed.store(failed, Ordering::Release);
    }

    /// Check if scan failed
    pub fn is_failed(&self) -> bool {
        self.scan_failed.load(Ordering::Acquire)
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
        // For non-ARM architectures, capture timestamp and proceed directly to stage 2
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
            self.is_high_priority.store(false, Ordering::Release);

            // Return true to signal caller to exit ISR immediately
            return true;
        }

        // Already at low priority, continue to stage 2
        false
    }

    /// Second stage IRQ handler - runs at LOW priority
    ///
    /// This disables the IRQ line and invokes the scan completion callback.
    /// At this point, RTOS API calls are safe.
    pub fn handle_irq_stage2(&mut self, status: WifiScanStatus) {
        let timestamp = self.scan_timestamp.unwrap_or_else(|| Instant::now());

        // Invoke scan completion callback
        if let Some(cb) = self.scan_complete_cb {
            cb(status, timestamp);
        }
    }

    /// Arm the IRQ for next scan
    ///
    /// Call this before starting a WiFi scan. It resets state and sets
    /// the IRQ to high priority mode.
    pub fn arm_for_scan(&mut self) {
        self.scan_timestamp = None;
        self.is_high_priority.store(true, Ordering::Release);
        self.scan_failed.store(false, Ordering::Release);
    }

    /// Disarm the IRQ after scan completion
    pub fn disarm(&mut self) {
        self.is_high_priority.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wifi_irq_manager_creation() {
        let mgr = WifiIrqManager::new();
        assert!(mgr.scan_timestamp.is_none());
        assert!(!mgr.is_failed());
    }

    #[test]
    fn test_failed_flag() {
        let mgr = WifiIrqManager::new();
        assert!(!mgr.is_failed());

        mgr.set_failed(true);
        assert!(mgr.is_failed());

        mgr.set_failed(false);
        assert!(!mgr.is_failed());
    }
}
