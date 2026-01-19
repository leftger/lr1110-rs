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
    /// Get the theoretical number of visible satellites (0x041F)
    GetNbVisibleSatellites = 0x041F,
    /// Get visible satellite IDs and Doppler information (0x0420)
    GetVisibleSatellitesDoppler = 0x0420,
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

/// Visible satellite information (theoretical calculation)
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GnssVisibleSatellite {
    /// Satellite ID
    pub satellite_id: u8,
    /// Doppler with 6ppm accuracy
    pub doppler: i16,
}

/// GNSS date type (seconds since January 6, 1980, 00:00:00 with leap seconds)
pub type GnssDate = u32;

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
    async fn gnss_set_constellation(
        &mut self,
        constellation_mask: GnssConstellationMask,
    ) -> Result<(), RadioError>;

    /// Read the currently configured GNSS constellations
    async fn gnss_read_constellation(&mut self) -> Result<GnssConstellationMask, RadioError>;

    /// Read the supported GNSS constellations for this chip
    async fn gnss_read_supported_constellations(
        &mut self,
    ) -> Result<GnssConstellationMask, RadioError>;

    /// Set the GNSS scan mode
    async fn gnss_set_scan_mode(&mut self, scan_mode: GnssScanMode) -> Result<(), RadioError>;

    /// Launch a GNSS scan
    async fn gnss_scan(
        &mut self,
        effort_mode: GnssSearchMode,
        result_mask: u8,
        nb_sv_max: u8,
    ) -> Result<(), RadioError>;

    /// Get the size of the GNSS scan result
    async fn gnss_get_result_size(&mut self) -> Result<u16, RadioError>;

    /// Read the GNSS scan results
    async fn gnss_read_results(&mut self, result_buffer: &mut [u8]) -> Result<(), RadioError>;

    /// Get the number of satellites detected
    async fn gnss_get_nb_satellites(&mut self) -> Result<u8, RadioError>;

    /// Get detected satellite information
    async fn gnss_get_satellites(
        &mut self,
        satellites: &mut [GnssDetectedSatellite],
        nb_satellites: u8,
    ) -> Result<u8, RadioError>;

    /// Set the assistance position
    async fn gnss_set_assistance_position(
        &mut self,
        position: &GnssAssistancePosition,
    ) -> Result<(), RadioError>;

    /// Read the assistance position
    async fn gnss_read_assistance_position(&mut self)
        -> Result<GnssAssistancePosition, RadioError>;

    /// Read the GNSS firmware version
    async fn gnss_read_firmware_version(&mut self) -> Result<GnssVersion, RadioError>;

    /// Get the GNSS context status
    async fn gnss_get_context_status(&mut self) -> Result<GnssContextStatus, RadioError>;

    /// Set the frequency search space
    async fn gnss_set_freq_search_space(
        &mut self,
        freq_search_space: GnssFreqSearchSpace,
    ) -> Result<(), RadioError>;

    /// Read the current frequency search space configuration
    async fn gnss_read_freq_search_space(&mut self) -> Result<GnssFreqSearchSpace, RadioError>;

    /// Reset the GNSS time
    async fn gnss_reset_time(&mut self) -> Result<(), RadioError>;

    /// Reset the GNSS position
    async fn gnss_reset_position(&mut self) -> Result<(), RadioError>;

    /// Update almanac from satellite signals
    async fn gnss_almanac_update_from_sat(
        &mut self,
        constellation_mask: u8,
        effort_mode: GnssSearchMode,
    ) -> Result<(), RadioError>;

    /// Read almanac data
    async fn gnss_read_almanac(
        &mut self,
        sv_id: u8,
        nb_sv: u8,
        almanac_buffer: &mut [u8],
    ) -> Result<usize, RadioError>;

    /// Read almanac data for a single satellite (convenience method)
    async fn gnss_read_almanac_per_satellite(
        &mut self,
        sv_id: u8,
    ) -> Result<[u8; GNSS_SINGLE_ALMANAC_READ_SIZE], RadioError>;

    /// Get the theoretical number of visible satellites
    ///
    /// Calculates the number of satellites that should be visible based on:
    /// - GPS date (seconds since January 6, 1980, 00:00:00 with leap seconds)
    /// - Assistance position (latitude/longitude)
    /// - Constellation mask (GPS, BeiDou, etc.)
    ///
    /// Must be called before `gnss_get_visible_satellites`.
    async fn gnss_get_nb_visible_satellites(
        &mut self,
        date: GnssDate,
        position: &GnssAssistancePosition,
        constellation_mask: GnssConstellationMask,
    ) -> Result<u8, RadioError>;

    /// Get visible satellite IDs and Doppler information
    ///
    /// Returns satellite ID and Doppler for each visible satellite.
    /// Must be called after `gnss_get_nb_visible_satellites`.
    ///
    /// # Arguments
    /// * `satellites` - Buffer to store satellite info (length must match nb_visible from previous call)
    async fn gnss_get_visible_satellites(
        &mut self,
        satellites: &mut [GnssVisibleSatellite],
    ) -> Result<u8, RadioError>;
}

// =============================================================================
// GnssExt trait implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> GnssExt for lora_phy::lr1110::Lr1110<SPI, IV, C>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    IV: lora_phy::mod_traits::InterfaceVariant,
    C: lora_phy::lr1110::variant::Lr1110Variant,
{
    async fn gnss_set_constellation(
        &mut self,
        constellation_mask: GnssConstellationMask,
    ) -> Result<(), RadioError> {
        let opcode = GnssOpCode::SetConstellation.bytes();
        let cmd = [opcode[0], opcode[1], constellation_mask];
        self.execute_command(&cmd).await
    }

    async fn gnss_read_constellation(&mut self) -> Result<GnssConstellationMask, RadioError> {
        let opcode = GnssOpCode::ReadConstellation.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer[0])
    }

    async fn gnss_read_supported_constellations(
        &mut self,
    ) -> Result<GnssConstellationMask, RadioError> {
        let opcode = GnssOpCode::ReadSupportedConstellation.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer[0])
    }

    async fn gnss_set_scan_mode(&mut self, scan_mode: GnssScanMode) -> Result<(), RadioError> {
        let opcode = GnssOpCode::SetScanMode.bytes();
        let cmd = [opcode[0], opcode[1], scan_mode.value()];
        self.execute_command(&cmd).await
    }

    async fn gnss_scan(
        &mut self,
        effort_mode: GnssSearchMode,
        result_mask: u8,
        nb_sv_max: u8,
    ) -> Result<(), RadioError> {
        let opcode = GnssOpCode::Scan.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            effort_mode.value(),
            result_mask,
            nb_sv_max,
        ];
        self.execute_command(&cmd).await
    }

    async fn gnss_get_result_size(&mut self) -> Result<u16, RadioError> {
        let opcode = GnssOpCode::GetResultSize.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16))
    }

    async fn gnss_read_results(&mut self, result_buffer: &mut [u8]) -> Result<(), RadioError> {
        let opcode = GnssOpCode::ReadResults.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command_with_response(&cmd, result_buffer)
            .await
    }

    async fn gnss_get_nb_satellites(&mut self) -> Result<u8, RadioError> {
        let opcode = GnssOpCode::GetNbSatellites.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer[0])
    }

    async fn gnss_get_satellites(
        &mut self,
        satellites: &mut [GnssDetectedSatellite],
        nb_satellites: u8,
    ) -> Result<u8, RadioError> {
        let opcode = GnssOpCode::GetSatellites.bytes();
        let cmd = [opcode[0], opcode[1]];

        // Each satellite entry is 4 bytes: id(1) + cnr(1) + doppler(2)
        let n = (nb_satellites as usize).min(satellites.len()).min(32);
        let mut rbuffer = [0u8; 128]; // 32 * 4 = 128 max
        self.execute_command_with_response(&cmd, &mut rbuffer[..n * 4])
            .await?;

        for (i, satellite) in satellites.iter_mut().enumerate().take(n) {
            let offset = i * 4;
            *satellite = GnssDetectedSatellite {
                satellite_id: rbuffer[offset],
                // CNR = SNR + offset (per SWDR001 lr11xx_gnss.c)
                cnr: (rbuffer[offset + 1] as i8) + GNSS_SNR_TO_CNR_OFFSET,
                doppler: ((rbuffer[offset + 2] as i16) << 8) | (rbuffer[offset + 3] as i16),
            };
        }

        Ok(n as u8)
    }

    async fn gnss_set_assistance_position(
        &mut self,
        position: &GnssAssistancePosition,
    ) -> Result<(), RadioError> {
        let latitude = ((position.latitude * 2048.0) / GNSS_SCALING_LATITUDE) as i16;
        let longitude = ((position.longitude * 2048.0) / GNSS_SCALING_LONGITUDE) as i16;

        let opcode = GnssOpCode::SetAssistancePosition.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (latitude >> 8) as u8,
            (latitude & 0xFF) as u8,
            (longitude >> 8) as u8,
            (longitude & 0xFF) as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn gnss_read_assistance_position(
        &mut self,
    ) -> Result<GnssAssistancePosition, RadioError> {
        let opcode = GnssOpCode::ReadAssistancePosition.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 4];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        let latitude_raw = ((rbuffer[0] as i16) << 8) | (rbuffer[1] as i16);
        let longitude_raw = ((rbuffer[2] as i16) << 8) | (rbuffer[3] as i16);

        Ok(GnssAssistancePosition {
            latitude: (latitude_raw as f32) * GNSS_SCALING_LATITUDE / 2048.0,
            longitude: (longitude_raw as f32) * GNSS_SCALING_LONGITUDE / 2048.0,
        })
    }

    async fn gnss_read_firmware_version(&mut self) -> Result<GnssVersion, RadioError> {
        let opcode = GnssOpCode::ReadFwVersion.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        Ok(GnssVersion {
            gnss_firmware: rbuffer[0],
            gnss_almanac: rbuffer[1],
        })
    }

    async fn gnss_get_context_status(&mut self) -> Result<GnssContextStatus, RadioError> {
        let opcode = GnssOpCode::GetContextStatus.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; GNSS_CONTEXT_STATUS_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        // Parse the context status bytes per SWDR001
        let firmware_version = rbuffer[0];
        let global_almanac_crc = ((rbuffer[1] as u32) << 24)
            | ((rbuffer[2] as u32) << 16)
            | ((rbuffer[3] as u32) << 8)
            | (rbuffer[4] as u32);
        let error_code = GnssErrorCode::from(rbuffer[5] & 0x0F);
        let almanac_update_gps = (rbuffer[6] & 0x02) != 0;
        let almanac_update_beidou = (rbuffer[6] & 0x04) != 0;
        let freq_search_space =
            GnssFreqSearchSpace::from(((rbuffer[6] & 0x01) << 1) | ((rbuffer[7] & 0x80) >> 7));

        Ok(GnssContextStatus {
            firmware_version,
            global_almanac_crc,
            error_code,
            almanac_update_gps,
            almanac_update_beidou,
            freq_search_space,
        })
    }

    async fn gnss_set_freq_search_space(
        &mut self,
        freq_search_space: GnssFreqSearchSpace,
    ) -> Result<(), RadioError> {
        let opcode = GnssOpCode::SetFreqSearchSpace.bytes();
        let cmd = [opcode[0], opcode[1], freq_search_space.value()];
        self.execute_command(&cmd).await
    }

    async fn gnss_reset_time(&mut self) -> Result<(), RadioError> {
        let opcode = GnssOpCode::ResetTime.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn gnss_reset_position(&mut self) -> Result<(), RadioError> {
        let opcode = GnssOpCode::ResetPosition.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn gnss_almanac_update_from_sat(
        &mut self,
        constellation_mask: u8,
        effort_mode: GnssSearchMode,
    ) -> Result<(), RadioError> {
        let opcode = GnssOpCode::AlmanacUpdateFromSat.bytes();
        // Per SWDR001: constellation_mask first, then effort_mode
        let cmd = [
            opcode[0],
            opcode[1],
            constellation_mask,
            effort_mode.value(),
        ];
        self.execute_command(&cmd).await
    }

    async fn gnss_read_almanac(
        &mut self,
        sv_id: u8,
        nb_sv: u8,
        almanac_buffer: &mut [u8],
    ) -> Result<usize, RadioError> {
        let expected_size = (nb_sv as usize) * GNSS_SINGLE_ALMANAC_READ_SIZE;
        if almanac_buffer.len() < expected_size {
            return Err(RadioError::PayloadSizeMismatch(
                expected_size,
                almanac_buffer.len(),
            ));
        }

        let opcode = GnssOpCode::AlmanacRead.bytes();
        let cmd = [opcode[0], opcode[1], sv_id, nb_sv];
        self.execute_command_with_response(&cmd, &mut almanac_buffer[..expected_size])
            .await?;

        Ok(expected_size)
    }

    async fn gnss_read_almanac_per_satellite(
        &mut self,
        sv_id: u8,
    ) -> Result<[u8; GNSS_SINGLE_ALMANAC_READ_SIZE], RadioError> {
        let mut almanac = [0u8; GNSS_SINGLE_ALMANAC_READ_SIZE];
        self.gnss_read_almanac(sv_id, 1, &mut almanac).await?;
        Ok(almanac)
    }

    async fn gnss_read_freq_search_space(&mut self) -> Result<GnssFreqSearchSpace, RadioError> {
        let opcode = GnssOpCode::ReadFreqSearchSpace.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(GnssFreqSearchSpace::from(rbuffer[0]))
    }

    async fn gnss_get_nb_visible_satellites(
        &mut self,
        date: GnssDate,
        position: &GnssAssistancePosition,
        constellation_mask: GnssConstellationMask,
    ) -> Result<u8, RadioError> {
        let opcode = GnssOpCode::GetNbVisibleSatellites.bytes();

        // Pack date (4 bytes, big-endian)
        let date_bytes = date.to_be_bytes();

        // Pack position (latitude: 12 bits, longitude: 12 bits)
        let latitude = ((position.latitude * 2048.0) / GNSS_SCALING_LATITUDE) as i16;
        let longitude = ((position.longitude * 2048.0) / GNSS_SCALING_LONGITUDE) as i16;

        let cmd = [
            opcode[0],
            opcode[1],
            date_bytes[0],
            date_bytes[1],
            date_bytes[2],
            date_bytes[3],
            (latitude >> 8) as u8,
            (latitude & 0xFF) as u8,
            (longitude >> 8) as u8,
            (longitude & 0xFF) as u8,
            constellation_mask,
        ];

        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer[0])
    }

    async fn gnss_get_visible_satellites(
        &mut self,
        satellites: &mut [GnssVisibleSatellite],
    ) -> Result<u8, RadioError> {
        let opcode = GnssOpCode::GetVisibleSatellitesDoppler.bytes();
        let cmd = [opcode[0], opcode[1]];

        // Each satellite is 3 bytes: ID (1) + Doppler (2)
        let result_size = satellites.len() * 3;
        let mut rbuffer = [0u8; 255]; // Max buffer size

        if result_size > rbuffer.len() {
            return Err(RadioError::PayloadSizeMismatch(result_size, rbuffer.len()));
        }

        self.execute_command_with_response(&cmd, &mut rbuffer[..result_size])
            .await?;

        // Parse results
        let count = satellites.len().min(result_size / 3);
        for (i, satellite) in satellites.iter_mut().enumerate().take(count) {
            let offset = i * 3;
            satellite.satellite_id = rbuffer[offset];
            satellite.doppler = ((rbuffer[offset + 1] as i16) << 8) | (rbuffer[offset + 2] as i16);
        }

        Ok(count as u8)
    }
}
