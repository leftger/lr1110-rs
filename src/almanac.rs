//! GNSS Almanac Management with Age Tracking
//!
//! This module provides comprehensive almanac management for the LR1110 GNSS functionality,
//! including age tracking, update scheduling, and status monitoring.
//!
//! # Source
//!
//! Ported from: `STM32-Sidewalk-SDK/.../geolocation_services/mw_gnss_almanac.c` and `gnss_helpers.c`
//!
//! # Almanac Basics
//!
//! GPS and BeiDou satellites broadcast almanac data describing satellite orbits.
//! The LR1110 uses this to:
//! - Predict satellite visibility
//! - Accelerate initial position fix (assisted vs autonomous mode)
//! - Reduce scan time (20s vs 80s)
//! - Lower power consumption (200 mAh vs 500 mAh per scan)
//!
//! ## Almanac Age and Performance
//!
//! | Age (days) | Status | Scan Mode | Duration | Power |
//! |------------|--------|-----------|----------|-------|
//! | 0 | Fresh | Assisted | 15-20s | 200 mAh |
//! | 1-7 | Stale | Assisted | 30-40s | 350 mAh |
//! | 8-14 | Very Old | Mixed | 40-60s | 450 mAh |
//! | 15 | No Data | Autonomous | 60-80s | 500 mAh |
//!
//! **Age is stored in bits 3-0 of the first almanac byte:**
//! ```ignore
//! let almanac = radio.gnss_read_almanac_per_satellite(sv_id).await?;
//! let age_days = almanac[0] & 0x0F;  // 0 = fresh, 15 = no data
//! ```
//!
//! ## Update Process
//!
//! GPS broadcasts almanac in **25 subframes** over **12.5 minutes**:
//! - Each subframe: ~30 seconds
//! - Total satellites: 32 (GPS) + 64 (BeiDou)
//! - Update frequency: Every 30 seconds per satellite
//!
//! ## Update Strategies
//!
//! ```ignore
//! pub enum UpdateStrategy {
//!     Full,          // GPS + BeiDou (~15 minutes)
//!     GpsOnly,       // GPS only (~7.5 minutes)
//!     BeiDouOnly,    // BeiDou only (~7.5 minutes)
//!     Incremental,   // Continue existing update
//! }
//! ```
//!
//! # Example Usage
//!
//! ## Check Almanac Status
//!
//! ```ignore
//! use lr1110_rs::almanac::AlmanacManager;
//!
//! let mut almanac_mgr = AlmanacManager::new();
//!
//! // Check comprehensive status (samples all satellites)
//! let status = almanac_mgr.check_status(&mut radio, true).await?;
//!
//! info!("GPS: {}% complete ({} fresh, {} stale, {} no data)",
//!     status.gps.completion_percentage(),
//!     status.gps.fresh_count,
//!     status.gps.stale_count,
//!     status.gps.no_data_count
//! );
//!
//! info!("BeiDou: {}% complete", status.beidou.completion_percentage());
//! info!("Overall: {}%", status.overall_completion());
//! ```
//!
//! ## Individual Satellite Tracking
//!
//! ```ignore
//! let sat = almanac_mgr.get_satellite_status(&mut radio, 5).await?;
//! info!("GPS PRN 6: {} days old ({})", sat.age_days, sat.status_str());
//!
//! if sat.is_fresh() {
//!     info!("Optimal performance!");
//! } else if sat.is_stale() {
//!     warn!("Consider updating almanac");
//! } else if sat.has_no_data() {
//!     error!("Must update for assisted mode!");
//! }
//! ```
//!
//! ## Automatic Updates
//!
//! ```ignore
//! // Check if update needed
//! if almanac_mgr.needs_update(&mut radio).await? {
//!     // Determine best strategy
//!     let strategy = almanac_mgr.determine_update_strategy(&mut radio).await?;
//!     info!("Update strategy: {:?}", strategy);
//!
//!     // Start update
//!     almanac_mgr.start_update(&mut radio, strategy, GnssSearchMode::HighEffort).await?;
//!
//!     // Monitor progress every 30 seconds
//!     for _ in 0..25 {
//!         embassy_time::Timer::after_secs(30).await;
//!         let ctx = radio.gnss_get_context_status().await?;
//!         info!("CRC: 0x{:08X}, GPS: {}, BeiDou: {}",
//!             ctx.global_almanac_crc,
//!             ctx.almanac_update_gps,
//!             ctx.almanac_update_beidou
//!         );
//!     }
//! }
//! ```
//!
//! ## Integration with GNSS Scanning
//!
//! Adjust scan parameters based on almanac age:
//!
//! ```ignore
//! let status = almanac_mgr.check_status(&mut radio, false).await?;
//!
//! let (search_mode, timeout) = if status.gps.has_good_coverage() {
//!     (GnssSearchMode::LowEffort, 15_000)   // Fresh almanac
//! } else if status.overall_completion() > 30 {
//!     (GnssSearchMode::MidEffort, 20_000)   // Some data
//! } else {
//!     (GnssSearchMode::HighEffort, 40_000)  // Poor/no data
//! };
//!
//! let config = ScanConfig { search_mode, scan_timeout_ms: timeout, ..Default::default() };
//! let result = scan_mgr.scan(&mut radio, &config).await?;
//! ```
//!
//! # Update Scheduling
//!
//! Recommended check intervals:
//!
//! ```ignore
//! let interval = almanac_mgr.calculate_next_check_interval(&status);
//! // Returns:
//! // - 8 hours (28800s) if fully updated
//! // - 10 minutes (600s) if partially updated
//! // - 5 minutes (300s) if no time available
//! // - 1 hour (3600s) after bad sky conditions
//! ```
//!
//! See the `lr1110_almanac_manager` example for complete implementation.

use crate::gnss::{
    GnssConstellationMask, GnssExt, GnssSearchMode, GNSS_BEIDOU_MASK, GNSS_GPS_MASK,
    GNSS_SINGLE_ALMANAC_READ_SIZE,
};
use lora_phy::mod_params::RadioError;

// =============================================================================
// Constants
// =============================================================================

/// Maximum age for almanac data before it's considered stale (days)
pub const ALMANAC_MAX_FRESH_AGE_DAYS: u8 = 0;

/// Age value indicating no almanac data available
pub const ALMANAC_AGE_NO_DATA: u8 = 0x0F;

/// Number of GPS satellites (PRN 1-32)
pub const GPS_NUM_SATELLITES: u8 = 32;

/// Number of BeiDou satellites (PRN 1-64)
pub const BEIDOU_NUM_SATELLITES: u8 = 64;

/// Almanac update check period when fully updated (seconds)
pub const ALMANAC_CHECK_PERIOD_DEFAULT_S: u32 = 8 * 60 * 60; // 8 hours

/// Almanac update check period when no time available (seconds)
pub const ALMANAC_CHECK_PERIOD_NO_TIME_S: u32 = 5 * 60; // 5 minutes

/// Almanac update check period after bad sky conditions (seconds)
pub const ALMANAC_CHECK_PERIOD_BAD_SKY_S: u32 = 60 * 60; // 1 hour

/// Almanac update check period when time accuracy is low (seconds)
pub const ALMANAC_CHECK_PERIOD_TIME_ACCURACY_S: u32 = 10 * 60; // 10 minutes

/// Maximum time between almanac read_time calls to maintain time (seconds)
pub const ALMANAC_READ_TIME_THRESHOLD_MAX_S: u32 = 24 * 60 * 60; // 24 hours

/// Duration of almanac update cycle (seconds)
/// GPS broadcasts almanac subframes every ~30 seconds, full update takes ~12.5 minutes
pub const ALMANAC_FULL_UPDATE_DURATION_S: u32 = 15 * 60; // 15 minutes recommended

// =============================================================================
// Types
// =============================================================================

/// Almanac age information for a single satellite
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct SatelliteAlmanacStatus {
    /// Satellite ID (0-31 for GPS, 64-127 for BeiDou)
    pub satellite_id: u8,

    /// Almanac age in days (0 = fresh, 15 = no data)
    pub age_days: u8,

    /// Whether almanac data is valid
    pub is_valid: bool,

    /// Raw almanac data (first 4 bytes for quick inspection)
    pub data_preview: [u8; 4],
}

impl SatelliteAlmanacStatus {
    /// Parse almanac age from almanac data
    ///
    /// The first byte contains the age indicator:
    /// - Bits 7-4: Reserved/status
    /// - Bits 3-0: Age in days (0 = fresh, 15 = no data)
    pub fn parse_age(almanac_data: &[u8; GNSS_SINGLE_ALMANAC_READ_SIZE]) -> u8 {
        almanac_data[0] & 0x0F
    }

    /// Check if almanac is fresh (0 days old)
    pub fn is_fresh(&self) -> bool {
        self.age_days == ALMANAC_MAX_FRESH_AGE_DAYS
    }

    /// Check if almanac is stale (needs update)
    pub fn is_stale(&self) -> bool {
        self.age_days > ALMANAC_MAX_FRESH_AGE_DAYS && self.age_days < ALMANAC_AGE_NO_DATA
    }

    /// Check if no almanac data is available
    pub fn has_no_data(&self) -> bool {
        self.age_days == ALMANAC_AGE_NO_DATA
    }

    /// Get status string
    pub fn status_str(&self) -> &'static str {
        if self.is_fresh() {
            "Fresh"
        } else if self.has_no_data() {
            "No data"
        } else {
            "Stale"
        }
    }
}

/// Overall almanac status for a constellation
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ConstellationAlmanacStatus {
    /// Constellation mask
    pub constellation: GnssConstellationMask,

    /// Number of satellites with fresh almanac
    pub fresh_count: u8,

    /// Number of satellites with stale almanac
    pub stale_count: u8,

    /// Number of satellites with no data
    pub no_data_count: u8,

    /// Total satellites checked
    pub total_count: u8,

    /// Whether this constellation needs update (from context status)
    pub needs_update: bool,

    /// Average age in days
    pub average_age_days: u8,
}

impl ConstellationAlmanacStatus {
    /// Check if constellation has good almanac coverage
    pub fn has_good_coverage(&self) -> bool {
        self.fresh_count >= (self.total_count / 2)
    }

    /// Get completion percentage
    pub fn completion_percentage(&self) -> u8 {
        if self.total_count == 0 {
            0
        } else {
            ((self.fresh_count as u32 * 100) / self.total_count as u32) as u8
        }
    }
}

/// Overall GNSS almanac status
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AlmanacStatus {
    /// GPS constellation status
    pub gps: ConstellationAlmanacStatus,

    /// BeiDou constellation status
    pub beidou: ConstellationAlmanacStatus,

    /// Global almanac CRC (changes when any satellite updates)
    pub global_crc: u32,

    /// Firmware version
    pub firmware_version: u8,

    /// Whether any update is needed
    pub update_needed: bool,
}

impl AlmanacStatus {
    /// Check if almanac is fully updated
    pub fn is_fully_updated(&self) -> bool {
        !self.gps.needs_update && !self.beidou.needs_update
    }

    /// Check if almanac needs update
    pub fn needs_update(&self) -> bool {
        self.gps.needs_update || self.beidou.needs_update
    }

    /// Get overall completion percentage
    pub fn overall_completion(&self) -> u8 {
        let total = self.gps.fresh_count + self.beidou.fresh_count;
        let max = self.gps.total_count + self.beidou.total_count;
        if max == 0 {
            0
        } else {
            ((total as u32 * 100) / max as u32) as u8
        }
    }
}

/// Almanac update strategy
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum UpdateStrategy {
    /// Full update for both constellations
    Full,
    /// Update only GPS
    GpsOnly,
    /// Update only BeiDou
    BeiDouOnly,
    /// Incremental update (continue existing update)
    Incremental,
}

// =============================================================================
// Almanac Manager
// =============================================================================

/// Almanac Manager
///
/// Manages GNSS almanac data including age tracking, update scheduling,
/// and status monitoring.
pub struct AlmanacManager {
    /// Last known almanac CRC
    last_crc: u32,

    /// Last update timestamp (seconds since boot)
    last_update_time_s: Option<u32>,

    /// Update strategy currently in use
    current_strategy: UpdateStrategy,

    /// Number of update attempts
    update_attempts: u32,

    /// Whether an update is in progress
    update_in_progress: bool,
}

impl AlmanacManager {
    /// Create a new almanac manager
    pub const fn new() -> Self {
        Self {
            last_crc: 0,
            last_update_time_s: None,
            current_strategy: UpdateStrategy::Full,
            update_attempts: 0,
            update_in_progress: false,
        }
    }

    /// Check almanac status for all satellites
    ///
    /// This reads the context status and optionally samples individual
    /// satellite almanac data to compute detailed statistics.
    pub async fn check_status<Radio>(
        &mut self,
        radio: &mut Radio,
        sample_satellites: bool,
    ) -> Result<AlmanacStatus, RadioError>
    where
        Radio: GnssExt,
    {
        // Read context status from LR1110
        let context = radio.gnss_get_context_status().await?;

        self.last_crc = context.global_almanac_crc;

        // Build constellation statuses
        let mut gps_status = ConstellationAlmanacStatus {
            constellation: GNSS_GPS_MASK,
            fresh_count: 0,
            stale_count: 0,
            no_data_count: 0,
            total_count: 0,
            needs_update: context.almanac_update_gps,
            average_age_days: 0,
        };

        let mut beidou_status = ConstellationAlmanacStatus {
            constellation: GNSS_BEIDOU_MASK,
            fresh_count: 0,
            stale_count: 0,
            no_data_count: 0,
            total_count: 0,
            needs_update: context.almanac_update_beidou,
            average_age_days: 0,
        };

        // Sample individual satellites if requested
        if sample_satellites {
            // Sample GPS satellites (0-31)
            let mut gps_total_age = 0u32;
            for sv_id in 0..GPS_NUM_SATELLITES {
                if let Ok(almanac) = radio.gnss_read_almanac_per_satellite(sv_id).await {
                    let age = SatelliteAlmanacStatus::parse_age(&almanac);
                    gps_total_age += age as u32;

                    if age == ALMANAC_MAX_FRESH_AGE_DAYS {
                        gps_status.fresh_count += 1;
                    } else if age == ALMANAC_AGE_NO_DATA {
                        gps_status.no_data_count += 1;
                    } else {
                        gps_status.stale_count += 1;
                    }
                }
                gps_status.total_count += 1;
            }
            gps_status.average_age_days = (gps_total_age / GPS_NUM_SATELLITES as u32) as u8;

            // Sample BeiDou satellites (64-127)
            let mut beidou_total_age = 0u32;
            for i in 0..BEIDOU_NUM_SATELLITES {
                let sv_id = 64 + i;
                if let Ok(almanac) = radio.gnss_read_almanac_per_satellite(sv_id).await {
                    let age = SatelliteAlmanacStatus::parse_age(&almanac);
                    beidou_total_age += age as u32;

                    if age == ALMANAC_MAX_FRESH_AGE_DAYS {
                        beidou_status.fresh_count += 1;
                    } else if age == ALMANAC_AGE_NO_DATA {
                        beidou_status.no_data_count += 1;
                    } else {
                        beidou_status.stale_count += 1;
                    }
                }
                beidou_status.total_count += 1;
            }
            beidou_status.average_age_days =
                (beidou_total_age / BEIDOU_NUM_SATELLITES as u32) as u8;
        }

        Ok(AlmanacStatus {
            gps: gps_status,
            beidou: beidou_status,
            global_crc: context.global_almanac_crc,
            firmware_version: context.firmware_version,
            update_needed: context.almanac_update_gps || context.almanac_update_beidou,
        })
    }

    /// Get status for a specific satellite
    pub async fn get_satellite_status<Radio>(
        &self,
        radio: &mut Radio,
        satellite_id: u8,
    ) -> Result<SatelliteAlmanacStatus, RadioError>
    where
        Radio: GnssExt,
    {
        let almanac = radio.gnss_read_almanac_per_satellite(satellite_id).await?;
        let age_days = SatelliteAlmanacStatus::parse_age(&almanac);

        Ok(SatelliteAlmanacStatus {
            satellite_id,
            age_days,
            is_valid: age_days < ALMANAC_AGE_NO_DATA,
            data_preview: [almanac[0], almanac[1], almanac[2], almanac[3]],
        })
    }

    /// Check if almanac update is needed
    pub async fn needs_update<Radio>(&mut self, radio: &mut Radio) -> Result<bool, RadioError>
    where
        Radio: GnssExt,
    {
        let context = radio.gnss_get_context_status().await?;
        Ok(context.almanac_update_gps || context.almanac_update_beidou)
    }

    /// Determine optimal update strategy based on current status
    pub async fn determine_update_strategy<Radio>(
        &mut self,
        radio: &mut Radio,
    ) -> Result<UpdateStrategy, RadioError>
    where
        Radio: GnssExt,
    {
        let context = radio.gnss_get_context_status().await?;

        let strategy = if context.almanac_update_gps && context.almanac_update_beidou {
            UpdateStrategy::Full
        } else if context.almanac_update_gps {
            UpdateStrategy::GpsOnly
        } else if context.almanac_update_beidou {
            UpdateStrategy::BeiDouOnly
        } else {
            UpdateStrategy::Incremental
        };

        self.current_strategy = strategy;
        Ok(strategy)
    }

    /// Start almanac update from satellite
    ///
    /// This launches the almanac demodulation process. The LR1110 will listen
    /// for almanac broadcasts from satellites. This is a long operation:
    /// - Single subframe: ~30 seconds (GPS broadcasts every 30s)
    /// - Full almanac: ~12.5 minutes (25 subframes × 30s)
    ///
    /// # Arguments
    ///
    /// * `radio` - LR1110 radio instance
    /// * `strategy` - Update strategy (full, GPS only, BeiDou only)
    /// * `effort` - Search effort (recommend HighEffort for almanac)
    ///
    /// # Returns
    ///
    /// `Ok(())` if update started successfully
    pub async fn start_update<Radio>(
        &mut self,
        radio: &mut Radio,
        strategy: UpdateStrategy,
        effort: GnssSearchMode,
    ) -> Result<(), RadioError>
    where
        Radio: GnssExt,
    {
        let constellation_mask = match strategy {
            UpdateStrategy::Full => GNSS_GPS_MASK | GNSS_BEIDOU_MASK,
            UpdateStrategy::GpsOnly => GNSS_GPS_MASK,
            UpdateStrategy::BeiDouOnly => GNSS_BEIDOU_MASK,
            UpdateStrategy::Incremental => GNSS_GPS_MASK | GNSS_BEIDOU_MASK,
        };

        self.update_in_progress = true;
        self.current_strategy = strategy;

        radio
            .gnss_almanac_update_from_sat(constellation_mask, effort)
            .await?;

        Ok(())
    }

    /// Complete almanac update and record statistics
    pub fn complete_update(&mut self, new_crc: u32, elapsed_time_s: u32) {
        self.update_in_progress = false;
        self.update_attempts += 1;
        self.last_update_time_s = Some(elapsed_time_s);
        self.last_crc = new_crc;
    }

    /// Check if update is in progress
    pub fn is_update_in_progress(&self) -> bool {
        self.update_in_progress
    }

    /// Get number of update attempts
    pub fn get_update_attempts(&self) -> u32 {
        self.update_attempts
    }

    /// Get last CRC
    pub fn get_last_crc(&self) -> u32 {
        self.last_crc
    }

    /// Get detailed status for sample satellites
    ///
    /// Returns almanac status for a sample of satellites from each constellation.
    /// This is useful for diagnostics and status display without reading all satellites.
    ///
    /// # Arguments
    ///
    /// * `radio` - LR1110 radio instance
    /// * `sample_count` - Number of satellites to sample per constellation
    ///
    /// # Returns
    ///
    /// Array of satellite statuses (GPS samples followed by BeiDou samples)
    pub async fn get_sample_status<Radio>(
        &self,
        radio: &mut Radio,
        sample_count: u8,
    ) -> Result<heapless::Vec<SatelliteAlmanacStatus, 16>, RadioError>
    where
        Radio: GnssExt,
    {
        let mut samples = heapless::Vec::new();

        // Sample GPS satellites
        for sv_id in 0..sample_count.min(GPS_NUM_SATELLITES) {
            if let Ok(status) = self.get_satellite_status(radio, sv_id).await {
                let _ = samples.push(status);
            }
        }

        // Sample BeiDou satellites
        for i in 0..sample_count.min(BEIDOU_NUM_SATELLITES) {
            let sv_id = 64 + i;
            if let Ok(status) = self.get_satellite_status(radio, sv_id).await {
                let _ = samples.push(status);
            }
        }

        Ok(samples)
    }

    /// Get oldest satellite in constellation
    ///
    /// Finds the satellite with the stalest almanac data.
    /// Useful for targeted updates.
    pub async fn get_oldest_satellite<Radio>(
        &self,
        radio: &mut Radio,
        constellation: GnssConstellationMask,
    ) -> Result<SatelliteAlmanacStatus, RadioError>
    where
        Radio: GnssExt,
    {
        let (start_id, count) = if constellation == GNSS_GPS_MASK {
            (0u8, GPS_NUM_SATELLITES)
        } else {
            (64u8, BEIDOU_NUM_SATELLITES)
        };

        let mut oldest = SatelliteAlmanacStatus {
            satellite_id: start_id,
            age_days: 0,
            is_valid: false,
            data_preview: [0; 4],
        };

        for i in 0..count {
            let sv_id = start_id + i;
            if let Ok(status) = self.get_satellite_status(radio, sv_id).await {
                if status.age_days > oldest.age_days {
                    oldest = status;
                }
            }
        }

        Ok(oldest)
    }

    /// Calculate next update check interval based on current status
    ///
    /// Returns recommended delay in seconds before next almanac check.
    pub fn calculate_next_check_interval(&self, status: &AlmanacStatus) -> u32 {
        if status.is_fully_updated() {
            // Fully updated: check every 8 hours
            ALMANAC_CHECK_PERIOD_DEFAULT_S
        } else if status.gps.no_data_count == status.gps.total_count
            && status.beidou.no_data_count == status.beidou.total_count
        {
            // No data at all: check more frequently
            ALMANAC_CHECK_PERIOD_NO_TIME_S
        } else {
            // Partial data: check periodically
            ALMANAC_CHECK_PERIOD_TIME_ACCURACY_S
        }
    }

    /// Reset almanac manager state
    pub fn reset(&mut self) {
        self.last_crc = 0;
        self.last_update_time_s = None;
        self.current_strategy = UpdateStrategy::Full;
        self.update_attempts = 0;
        self.update_in_progress = false;
    }
}

impl Default for AlmanacManager {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Helper Functions
// =============================================================================

/// Convert satellite ID to human-readable PRN
pub fn sv_id_to_prn(sv_id: u8) -> u8 {
    if sv_id < 32 {
        sv_id + 1 // GPS: sv_id 0-31 → PRN 1-32
    } else if sv_id >= 64 && sv_id < 128 {
        sv_id - 64 + 1 // BeiDou: sv_id 64-127 → PRN 1-64
    } else {
        sv_id
    }
}

/// Get constellation name from satellite ID
pub fn constellation_name(sv_id: u8) -> &'static str {
    if sv_id < 32 {
        "GPS"
    } else if sv_id >= 64 && sv_id < 128 {
        "BeiDou"
    } else {
        "Unknown"
    }
}

/// Validate satellite ID
pub fn is_valid_satellite_id(sv_id: u8) -> bool {
    (sv_id < 32) || (sv_id >= 64 && sv_id < 128)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_almanac_age_parsing() {
        let mut almanac = [0u8; GNSS_SINGLE_ALMANAC_READ_SIZE];

        // Fresh almanac
        almanac[0] = 0x00;
        assert_eq!(SatelliteAlmanacStatus::parse_age(&almanac), 0);

        // 5 days old
        almanac[0] = 0x05;
        assert_eq!(SatelliteAlmanacStatus::parse_age(&almanac), 5);

        // No data
        almanac[0] = 0x0F;
        assert_eq!(SatelliteAlmanacStatus::parse_age(&almanac), 15);

        // Mask off upper bits
        almanac[0] = 0xF3; // Should be 3
        assert_eq!(SatelliteAlmanacStatus::parse_age(&almanac), 3);
    }

    #[test]
    fn test_satellite_status() {
        let status = SatelliteAlmanacStatus {
            satellite_id: 5,
            age_days: 0,
            is_valid: true,
            data_preview: [0, 1, 2, 3],
        };

        assert!(status.is_fresh());
        assert!(!status.is_stale());
        assert!(!status.has_no_data());
        assert_eq!(status.status_str(), "Fresh");
    }

    #[test]
    fn test_stale_almanac() {
        let status = SatelliteAlmanacStatus {
            satellite_id: 10,
            age_days: 7,
            is_valid: true,
            data_preview: [0; 4],
        };

        assert!(!status.is_fresh());
        assert!(status.is_stale());
        assert!(!status.has_no_data());
        assert_eq!(status.status_str(), "Stale");
    }

    #[test]
    fn test_no_data_almanac() {
        let status = SatelliteAlmanacStatus {
            satellite_id: 15,
            age_days: ALMANAC_AGE_NO_DATA,
            is_valid: false,
            data_preview: [0xFF; 4],
        };

        assert!(!status.is_fresh());
        assert!(!status.is_stale());
        assert!(status.has_no_data());
        assert_eq!(status.status_str(), "No data");
    }

    #[test]
    fn test_constellation_coverage() {
        let status = ConstellationAlmanacStatus {
            constellation: GNSS_GPS_MASK,
            fresh_count: 20,
            stale_count: 10,
            no_data_count: 2,
            total_count: 32,
            needs_update: false,
            average_age_days: 2,
        };

        // 20/32 = 62.5% > 50%
        assert!(status.has_good_coverage());
        assert_eq!(status.completion_percentage(), 62);
    }

    #[test]
    fn test_sv_id_to_prn() {
        // GPS
        assert_eq!(sv_id_to_prn(0), 1);
        assert_eq!(sv_id_to_prn(31), 32);

        // BeiDou
        assert_eq!(sv_id_to_prn(64), 1);
        assert_eq!(sv_id_to_prn(127), 64);
    }

    #[test]
    fn test_constellation_name() {
        assert_eq!(constellation_name(0), "GPS");
        assert_eq!(constellation_name(31), "GPS");
        assert_eq!(constellation_name(64), "BeiDou");
        assert_eq!(constellation_name(127), "BeiDou");
        assert_eq!(constellation_name(32), "Unknown");
    }

    #[test]
    fn test_update_strategy() {
        let mgr = AlmanacManager::new();
        assert_eq!(mgr.current_strategy, UpdateStrategy::Full);
        assert!(!mgr.is_update_in_progress());
    }

    #[test]
    fn test_almanac_manager_reset() {
        let mut mgr = AlmanacManager::new();
        mgr.last_crc = 0x12345678;
        mgr.update_attempts = 5;

        mgr.reset();
        assert_eq!(mgr.last_crc, 0);
        assert_eq!(mgr.update_attempts, 0);
    }
}
