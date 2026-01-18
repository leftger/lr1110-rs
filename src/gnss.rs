//! GNSS (GPS/BeiDou) scanning functionality for the LR1110
//!
//! This module provides GNSS scanning capabilities for geolocation using
//! GPS and BeiDou satellite constellations.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::gnss::{GnssExt, GnssSearchMode, GNSS_GPS_MASK};
//!
//! // Set up constellations
//! radio.gnss_set_constellation(GNSS_GPS_MASK).await?;
//!
//! // Launch a scan
//! radio.gnss_scan(GnssSearchMode::HighEffort, 0x07, 0).await?;
//!
//! // Wait for GnssScanDone IRQ, then read results
//! let size = radio.gnss_get_result_size().await?;
//! let mut buffer = [0u8; 256];
//! radio.gnss_read_results(&mut buffer[..size as usize]).await?;
//! ```

use lora_phy::mod_params::RadioError;

// =============================================================================
// GNSS Types and Constants
// =============================================================================

/// GNSS OpCodes (16-bit commands)
#[derive(Clone, Copy, PartialEq)]
pub enum GnssOpCode {
    /// Set the constellation to use (0x0400)
    SetConstellation = 0x0400,
    /// Read the used constellations (0x0401)
    ReadConstellation = 0x0401,
    /// Set almanac update configuration (0x0402)
    SetAlmanacUpdate = 0x0402,
    /// Read the almanac update configuration (0x0403)
    ReadAlmanacUpdate = 0x0403,
    /// Set the frequency search space (0x0404)
    SetFreqSearchSpace = 0x0404,
    /// Read the frequency search space (0x0405)
    ReadFreqSearchSpace = 0x0405,
    /// Read the GNSS firmware version (0x0406)
    ReadFwVersion = 0x0406,
    /// Read the supported constellations (0x0407)
    ReadSupportedConstellation = 0x0407,
    /// Define single or double capture mode (0x0408)
    SetScanMode = 0x0408,
    /// Launch the scan (0x040B)
    Scan = 0x040B,
    /// Get the size of the output payload (0x040C)
    GetResultSize = 0x040C,
    /// Read the result byte stream (0x040D)
    ReadResults = 0x040D,
    /// Update the almanac (0x040E)
    AlmanacUpdate = 0x040E,
    /// Read all almanacs (0x040F)
    AlmanacRead = 0x040F,
    /// Set the assistance position (0x0410)
    SetAssistancePosition = 0x0410,
    /// Read the assistance position (0x0411)
    ReadAssistancePosition = 0x0411,
    /// Push messages coming from the solver (0x0414)
    PushSolverMsg = 0x0414,
    /// Push messages coming from the device management (0x0415)
    PushDmMsg = 0x0415,
    /// Read the context (0x0416)
    GetContextStatus = 0x0416,
    /// Get the number of satellites detected during a scan (0x0417)
    GetNbSatellites = 0x0417,
    /// Get the list of satellites detected during a scan (0x0418)
    GetSatellites = 0x0418,
    /// Read the almanac of given satellites (0x041A)
    ReadAlmanacPerSatellite = 0x041A,
    /// Reset the internal time (0x0435)
    ResetTime = 0x0435,
    /// Reset the location and the history Doppler buffer (0x0437)
    ResetPosition = 0x0437,
    /// Launches one scan to download from satellite almanac parameters broadcasted (0x0454)
    AlmanacUpdateFromSat = 0x0454,
}

impl GnssOpCode {
    pub fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// GNSS constellation identifiers
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssConstellation {
    /// GPS constellation
    Gps = 0x01,
    /// BeiDou constellation
    BeiDou = 0x02,
}

impl GnssConstellation {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// Bit mask of constellation configurations
pub type GnssConstellationMask = u8;

/// GPS constellation mask
pub const GNSS_GPS_MASK: GnssConstellationMask = 0x01;
/// BeiDou constellation mask
pub const GNSS_BEIDOU_MASK: GnssConstellationMask = 0x02;

/// Search mode for GNSS scan
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssSearchMode {
    /// Search all requested satellites or fail, scan duration is low
    LowEffort = 0x00,
    /// Add additional search if not all satellites are found, scan duration is standard
    MidEffort = 0x01,
    /// Add additional search if not all satellites are found, scan duration is very high
    HighEffort = 0x02,
}

impl GnssSearchMode {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GNSS response type indicates the destination
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssDestination {
    /// Host MCU
    Host = 0x00,
    /// GNSS Solver (LoRa Cloud)
    Solver = 0x01,
    /// GNSS DMC (Device Management Component)
    Dmc = 0x02,
}

impl GnssDestination {
    pub fn value(self) -> u8 {
        self as u8
    }
}

impl From<u8> for GnssDestination {
    fn from(value: u8) -> Self {
        match value {
            0x00 => GnssDestination::Host,
            0x01 => GnssDestination::Solver,
            0x02 => GnssDestination::Dmc,
            _ => GnssDestination::Host,
        }
    }
}

/// GNSS single or double scan mode
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssScanMode {
    /// Single scan legacy mode - NAV3 format
    SingleScanLegacy = 0x00,
    /// Single scan and 5 fast scans - NAV3 format
    SingleScanAnd5FastScans = 0x03,
}

impl GnssScanMode {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GNSS error codes
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssErrorCode {
    NoError = 0,
    AlmanacTooOld = 1,
    UpdateCrcMismatch = 2,
    UpdateFlashMemoryIntegrity = 3,
    AlmanacUpdateNotAllowed = 4,
}

impl From<u8> for GnssErrorCode {
    fn from(value: u8) -> Self {
        match value {
            0 => GnssErrorCode::NoError,
            1 => GnssErrorCode::AlmanacTooOld,
            2 => GnssErrorCode::UpdateCrcMismatch,
            3 => GnssErrorCode::UpdateFlashMemoryIntegrity,
            4 => GnssErrorCode::AlmanacUpdateNotAllowed,
            _ => GnssErrorCode::NoError,
        }
    }
}

/// GNSS frequency search space
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GnssFreqSearchSpace {
    Hz250 = 0,
    Hz500 = 1,
    Khz1 = 2,
    Khz2 = 3,
}

impl GnssFreqSearchSpace {
    pub fn value(self) -> u8 {
        self as u8
    }
}

impl From<u8> for GnssFreqSearchSpace {
    fn from(value: u8) -> Self {
        match value {
            0 => GnssFreqSearchSpace::Hz250,
            1 => GnssFreqSearchSpace::Hz500,
            2 => GnssFreqSearchSpace::Khz1,
            3 => GnssFreqSearchSpace::Khz2,
            _ => GnssFreqSearchSpace::Hz250,
        }
    }
}

/// Maximal GNSS result buffer size
pub const GNSS_MAX_RESULT_SIZE: usize = 2820;

/// Size of the almanac of a single satellite when reading
pub const GNSS_SINGLE_ALMANAC_READ_SIZE: usize = 22;

/// Size of the almanac of a single satellite when writing
pub const GNSS_SINGLE_ALMANAC_WRITE_SIZE: usize = 20;

/// Size of the GNSS context status buffer
pub const GNSS_CONTEXT_STATUS_LENGTH: usize = 9;

/// SNR to CNR offset conversion
pub const GNSS_SNR_TO_CNR_OFFSET: i8 = 31;

/// Scaling factor for latitude conversion (90 degrees)
pub const GNSS_SCALING_LATITUDE: f32 = 90.0;

/// Scaling factor for longitude conversion (180 degrees)
pub const GNSS_SCALING_LONGITUDE: f32 = 180.0;

/// Assistance position for GNSS
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GnssAssistancePosition {
    /// Latitude in degrees (-90 to +90)
    pub latitude: f32,
    /// Longitude in degrees (-180 to +180)
    pub longitude: f32,
}

/// GNSS firmware version
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GnssVersion {
    /// Version of the firmware
    pub gnss_firmware: u8,
    /// Version of the almanac format
    pub gnss_almanac: u8,
}

/// Detected satellite information
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GnssDetectedSatellite {
    /// Satellite ID
    pub satellite_id: u8,
    /// Carrier-to-noise ratio (C/N) in dB
    pub cnr: i8,
    /// SV doppler in Hz
    pub doppler: i16,
}

/// GNSS context status structure
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GnssContextStatus {
    /// Firmware version
    pub firmware_version: u8,
    /// Global almanac CRC
    pub global_almanac_crc: u32,
    /// Error code
    pub error_code: GnssErrorCode,
    /// Whether GPS almanac needs update
    pub almanac_update_gps: bool,
    /// Whether BeiDou almanac needs update
    pub almanac_update_beidou: bool,
    /// Frequency search space
    pub freq_search_space: GnssFreqSearchSpace,
}

// =============================================================================
// GNSS Extension Trait
// =============================================================================

/// Extension trait that adds GNSS functionality to the LR1110 radio.
///
/// This trait requires that `lora-phy` exposes the `Lr1110Interface` trait
/// which provides access to low-level SPI operations.
#[allow(async_fn_in_trait)]
pub trait GnssExt {
    /// Set the GNSS constellations to use
    async fn gnss_set_constellation(&mut self, constellation_mask: GnssConstellationMask) -> Result<(), RadioError>;

    /// Read the currently configured GNSS constellations
    async fn gnss_read_constellation(&mut self) -> Result<GnssConstellationMask, RadioError>;

    /// Set the GNSS scan mode
    async fn gnss_set_scan_mode(&mut self, scan_mode: GnssScanMode) -> Result<(), RadioError>;

    /// Launch a GNSS scan
    async fn gnss_scan(&mut self, effort_mode: GnssSearchMode, result_mask: u8, nb_sv_max: u8) -> Result<(), RadioError>;

    /// Get the size of the GNSS scan result
    async fn gnss_get_result_size(&mut self) -> Result<u16, RadioError>;

    /// Read the GNSS scan results
    async fn gnss_read_results(&mut self, result_buffer: &mut [u8]) -> Result<(), RadioError>;

    /// Get the number of satellites detected
    async fn gnss_get_nb_satellites(&mut self) -> Result<u8, RadioError>;

    /// Get detected satellite information
    async fn gnss_get_satellites(&mut self, satellites: &mut [GnssDetectedSatellite], nb_satellites: u8) -> Result<u8, RadioError>;

    /// Set the assistance position
    async fn gnss_set_assistance_position(&mut self, position: &GnssAssistancePosition) -> Result<(), RadioError>;

    /// Read the assistance position
    async fn gnss_read_assistance_position(&mut self) -> Result<GnssAssistancePosition, RadioError>;

    /// Read the GNSS firmware version
    async fn gnss_read_firmware_version(&mut self) -> Result<GnssVersion, RadioError>;

    /// Get the GNSS context status
    async fn gnss_get_context_status(&mut self) -> Result<GnssContextStatus, RadioError>;

    /// Set the frequency search space
    async fn gnss_set_freq_search_space(&mut self, freq_search_space: GnssFreqSearchSpace) -> Result<(), RadioError>;

    /// Reset the GNSS time
    async fn gnss_reset_time(&mut self) -> Result<(), RadioError>;

    /// Reset the GNSS position
    async fn gnss_reset_position(&mut self) -> Result<(), RadioError>;

    /// Update almanac from satellite signals
    async fn gnss_almanac_update_from_sat(&mut self, constellation_mask: u8, effort_mode: GnssSearchMode) -> Result<(), RadioError>;

    /// Read almanac data
    async fn gnss_read_almanac(&mut self, sv_id: u8, nb_sv: u8, almanac_buffer: &mut [u8]) -> Result<usize, RadioError>;
}

// NOTE: Implementation requires lora-phy to expose low-level SPI interface.
// This will be implemented once lora-phy adds the Lr1110Interface trait.
//
// For now, users can use the types defined above with their own implementation.
