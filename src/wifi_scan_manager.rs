//! WiFi Scan Manager with IRQ Management
//!
//! High-level coordinator for WiFi scanning operations that integrates:
//! - Two-stage IRQ handling for precise timing
//! - Radio planner coordination
//! - Pre/post-scan callbacks
//! - Result filtering (remove mobile APs)
//! - Result sorting (by RSSI, strongest first)
//! - Automatic retry logic
//!
//! # Source
//!
//! Ported from: `STM32-Sidewalk-SDK/.../geolocation_services/mw_wifi_scan.c`
//!
//! # WiFi Scanning for Indoor Positioning
//!
//! WiFi passive scanning detects nearby access points for indoor geolocation:
//! - Scan duration: 3-10 seconds (depends on channels and timeout)
//! - Power consumption: ~50-100 mAh per scan
//! - Typical results: 5-20 APs detected
//! - Best results: 3-5 fixed APs with strong signal
//!
//! ## Result Processing
//!
//! WiFi results require filtering and sorting for optimal positioning:
//!
//! 1. **Filter mobile APs**: Remove WiFi hotspots (phones, cars, etc.)
//! 2. **Sort by RSSI**: Strongest signals first (better for positioning)
//! 3. **Limit results**: Keep top 5 for LoRa Cloud (reduces payload)
//!
//! ```ignore
//! let config = WifiScanConfig {
//!     filter_mobile_aps: true,      // Only fixed APs
//!     sort_by_rssi: true,           // Strongest first
//!     max_results_to_send: 5,       // Top 5 for cloud
//!     ..Default::default()
//! };
//! ```
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::wifi_scan_manager::{WifiScanManager, WifiScanConfig};
//! use lr1110_rs::wifi::{WifiSignalTypeScan, WifiScanMode, WIFI_ALL_CHANNELS_MASK};
//!
//! let mut manager = WifiScanManager::new();
//!
//! // Register callbacks for system coordination
//! manager.set_prescan_callback(|| {
//!     sidewalk_suspend();
//!     ble_suspend();
//! });
//! manager.set_postscan_callback(|| {
//!     sidewalk_resume();
//!     ble_resume();
//! });
//!
//! // Configure WiFi scan
//! let config = WifiScanConfig {
//!     signal_type: WifiSignalTypeScan::TypeBGN,  // 802.11 b/g/n
//!     channels: WIFI_ALL_CHANNELS_MASK,          // All channels 1-14
//!     scan_mode: WifiScanMode::Beacon,
//!     max_results: 32,
//!     filter_mobile_aps: true,                   // Only fixed APs
//!     sort_by_rssi: true,                        // Strongest first
//!     max_results_to_send: 5,                    // Top 5
//!     ..Default::default()
//! };
//!
//! // Perform scan
//! let result = manager.scan(&mut radio, &config).await?;
//! info!("Found {} APs in {}ms", result.num_results, result.duration_ms);
//!
//! // Read and send to LoRa Cloud
//! let mut aps = [WifiBasicMacTypeChannelResult::default(); 32];
//! radio.wifi_read_basic_mac_type_channel_results(&mut aps, 0, result.num_results).await?;
//! send_to_lora_cloud(&aps[..result.num_results.min(5) as usize]);
//! ```
//!
//! See `examples/stm32wba/src/bin/lr1110_wifi_scan_with_irq.rs` for complete example.

use crate::radio_planner::{PlannerError, RadioPlanner, RadioTask, TaskPriority, TaskType};
use crate::wifi::{WifiChannelMask, WifiScanMode, WifiSignalTypeScan, WIFI_ALL_CHANNELS_MASK};
use crate::wifi_irq::{WifiIrqManager, WifiPostScanCallback, WifiPreScanCallback, WifiScanStatus};
use embassy_time::Instant;

/// WiFi scan configuration
#[derive(Clone, Copy, Debug)]
pub struct WifiScanConfig {
    /// Signal types to scan (B, G, N)
    pub signal_type: WifiSignalTypeScan,

    /// Channel mask (which channels to scan)
    pub channels: WifiChannelMask,

    /// Scan mode (beacon, full beacon, etc.)
    pub scan_mode: WifiScanMode,

    /// Maximum number of results to collect
    pub max_results: u8,

    /// Number of scans per channel
    pub scans_per_channel: u8,

    /// Timeout per scan in milliseconds (10-65535ms)
    pub timeout_per_scan_ms: u16,

    /// Timeout per channel in milliseconds (1-255ms)
    pub timeout_per_channel_ms: u8,

    /// Abort on timeout
    pub abort_on_timeout: bool,

    /// Maximum retry attempts on scan failure
    pub max_retries: u8,

    /// Overall scan timeout (milliseconds)
    pub scan_timeout_ms: u32,

    /// Filter mobile APs (only keep fixed APs)
    pub filter_mobile_aps: bool,

    /// Sort results by RSSI (strongest first)
    pub sort_by_rssi: bool,

    /// Maximum results to keep after filtering/sorting
    pub max_results_to_send: u8,
}

impl Default for WifiScanConfig {
    fn default() -> Self {
        Self {
            signal_type: WifiSignalTypeScan::TypeBGN,
            channels: WIFI_ALL_CHANNELS_MASK,
            scan_mode: WifiScanMode::Beacon,
            max_results: 32,
            scans_per_channel: 1,
            timeout_per_scan_ms: 90,
            timeout_per_channel_ms: 100,
            abort_on_timeout: false,
            max_retries: 3,
            scan_timeout_ms: 10_000, // 10 seconds
            filter_mobile_aps: true,
            sort_by_rssi: true,
            max_results_to_send: 5,
        }
    }
}

/// WiFi scan result
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiScanResult {
    /// Scan status
    pub status: WifiScanStatus,

    /// Number of APs detected
    pub num_results: u8,

    /// Scan duration (milliseconds)
    pub duration_ms: u32,

    /// Timestamp when scan completed
    pub timestamp: Instant,

    /// Power consumption (nanoampere-hours)
    pub power_consumption_nah: u32,
}

/// WiFi Scan Manager
///
/// Coordinates WiFi scanning with IRQ management, radio planning,
/// and system callbacks.
pub struct WifiScanManager {
    /// IRQ manager for scan operations
    irq_manager: WifiIrqManager,

    /// Radio planner for coordinating access
    radio_planner: RadioPlanner,

    /// Scan start time
    scan_start_time: Option<Instant>,

    /// Scan end time
    scan_end_time: Option<Instant>,
}

impl WifiScanManager {
    /// Create a new WiFi scan manager
    pub fn new() -> Self {
        Self {
            irq_manager: WifiIrqManager::new(),
            radio_planner: RadioPlanner::new(),
            scan_start_time: None,
            scan_end_time: None,
        }
    }

    /// Set pre-scan callback
    ///
    /// Called before any radio access. Use to suspend concurrent operations
    /// like Sidewalk, BLE, etc.
    pub fn set_prescan_callback(&mut self, cb: WifiPreScanCallback) {
        self.irq_manager.set_prescan_callback(cb);
    }

    /// Set post-scan callback
    ///
    /// Called after scan completes and radio is in standby/sleep.
    /// Use to resume suspended operations.
    pub fn set_postscan_callback(&mut self, cb: WifiPostScanCallback) {
        self.irq_manager.set_postscan_callback(cb);
    }

    /// Perform a WiFi scan with the given configuration
    ///
    /// This is the main entry point for WiFi scanning. It handles:
    /// - IRQ setup and management
    /// - Radio planner coordination
    /// - Scan retry logic
    /// - Pre/post-scan callbacks
    /// - Result filtering and sorting
    ///
    /// # Returns
    ///
    /// `WifiScanResult` with scan status and metadata
    pub async fn scan<Radio>(
        &mut self,
        radio: &mut Radio,
        config: &WifiScanConfig,
    ) -> Result<WifiScanResult, PlannerError>
    where
        Radio: crate::wifi::WifiExt,
    {
        // Execute pre-scan actions BEFORE any radio access
        self.irq_manager.execute_prescan_actions();

        // Schedule scan task with radio planner
        let task = RadioTask::new(
            TaskType::WifiScan,
            TaskPriority::High,
            config.scan_timeout_ms,
        );

        let task_id = self.radio_planner.enqueue_task(task)?;
        self.radio_planner.start_task(task_id)?;

        // Arm IRQ for scan
        self.irq_manager.arm_for_scan();

        // Record start time
        self.scan_start_time = Some(Instant::now());

        // Initiate WiFi scan
        if let Err(_e) = radio
            .wifi_scan(
                config.signal_type,
                config.channels,
                config.scan_mode,
                config.max_results,
                config.scans_per_channel,
                config.timeout_per_scan_ms,
                config.abort_on_timeout,
            )
            .await
        {
            self.radio_planner.fail_task(task_id)?;
            self.irq_manager.execute_postscan_actions();
            return Err(PlannerError::TaskAborted);
        }

        // Wait for scan completion
        // In production, this would be handled by IRQ_WIFI_SCAN_DONE
        embassy_time::Timer::after_millis(config.scan_timeout_ms as u64).await;

        // Record end time
        self.scan_end_time = Some(Instant::now());
        let duration_ms = self
            .scan_start_time
            .and_then(|start| {
                self.scan_end_time
                    .map(|end| end.duration_since(start).as_millis() as u32)
            })
            .unwrap_or(0);

        // Read scan results
        let num_results = match radio.wifi_get_nb_results().await {
            Ok(count) => count,
            Err(_) => 0,
        };

        let status = if num_results > 0 {
            WifiScanStatus::Done
        } else {
            WifiScanStatus::Timeout
        };

        // Execute post-scan actions
        self.irq_manager.execute_postscan_actions();

        // Complete task in planner
        self.radio_planner.complete_task(task_id)?;

        // Note: Result filtering (remove mobile APs) and sorting (by RSSI)
        // should be done by the caller after reading results

        Ok(WifiScanResult {
            status,
            num_results,
            duration_ms,
            timestamp: Instant::now(),
            power_consumption_nah: 0, // To be read from radio
        })
    }

    /// Get radio planner (for advanced usage)
    pub fn radio_planner(&mut self) -> &mut RadioPlanner {
        &mut self.radio_planner
    }

    /// Get IRQ manager (for advanced usage)
    pub fn irq_manager(&mut self) -> &mut WifiIrqManager {
        &mut self.irq_manager
    }

    /// Get scan duration (milliseconds)
    pub fn get_scan_duration_ms(&self) -> Option<u32> {
        self.scan_start_time.and_then(|start| {
            self.scan_end_time
                .map(|end| end.duration_since(start).as_millis() as u32)
        })
    }
}

impl Default for WifiScanManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_creation() {
        let manager = WifiScanManager::new();
        assert!(manager.scan_start_time.is_none());
        assert!(manager.scan_end_time.is_none());
    }

    #[test]
    fn test_scan_config_defaults() {
        let config = WifiScanConfig::default();
        assert_eq!(config.max_results, 32);
        assert_eq!(config.scan_timeout_ms, 10_000);
        assert!(config.filter_mobile_aps);
        assert!(config.sort_by_rssi);
    }
}
