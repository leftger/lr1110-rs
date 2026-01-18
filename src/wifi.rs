//! WiFi passive scanning functionality for the LR1110
//!
//! This module provides WiFi AP scanning capabilities for indoor positioning.
//! The LR1110 can passively scan for WiFi 802.11 b/g/n signals and report
//! the MAC addresses and signal strength of nearby access points.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::wifi::{WifiExt, WifiSignalTypeScan, WifiScanMode, WIFI_ALL_CHANNELS_MASK};
//!
//! // Start a WiFi scan
//! radio.wifi_scan(
//!     WifiSignalTypeScan::TypeBGN,
//!     WIFI_ALL_CHANNELS_MASK,
//!     WifiScanMode::Beacon,
//!     32,  // max results
//!     1,   // scans per channel
//!     100, // timeout per scan (ms)
//!     false,
//! ).await?;
//!
//! // Wait for WifiScanDone IRQ, then read results
//! let nb_results = radio.wifi_get_nb_results().await?;
//! let mut results = [WifiBasicMacTypeChannelResult::default(); 32];
//! radio.wifi_read_basic_mac_type_channel_results(&mut results, 0, nb_results).await?;
//! ```

use lora_phy::mod_params::RadioError;

// =============================================================================
// WiFi Types and Constants (from SWDR001 lr11xx_wifi.c and lr11xx_wifi_types.h)
// =============================================================================

/// WiFi OpCodes (16-bit commands)
#[derive(Clone, Copy, PartialEq)]
pub enum WifiOpCode {
    /// Start WiFi passive scan (0x0300)
    Scan = 0x0300,
    /// Start WiFi passive scan with time limit (0x0301)
    ScanTimeLimit = 0x0301,
    /// Search for country codes (0x0302)
    SearchCountryCode = 0x0302,
    /// Country code with time limit (0x0303)
    CountryCodeTimeLimit = 0x0303,
    /// Get the size of scan results (0x0305)
    GetResultSize = 0x0305,
    /// Read scan results (0x0306)
    ReadResult = 0x0306,
    /// Reset cumulative timing (0x0307)
    ResetCumulTiming = 0x0307,
    /// Read cumulative timing (0x0308)
    ReadCumulTiming = 0x0308,
    /// Get the size of country code results (0x0309)
    GetSizeCountryResult = 0x0309,
    /// Read country codes (0x030A)
    ReadCountryCode = 0x030A,
    /// Configure timestamp for AP phone (0x030B)
    ConfigureTimestampApPhone = 0x030B,
    /// Get WiFi firmware version (0x0320)
    GetVersion = 0x0320,
}

impl WifiOpCode {
    pub fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// WiFi channel mask type (bit mask for channels 1-14)
pub type WifiChannelMask = u16;

/// WiFi MAC address length in bytes
pub const WIFI_MAC_ADDRESS_LENGTH: usize = 6;

/// Maximum number of WiFi results
pub const WIFI_MAX_RESULTS: usize = 32;

/// WiFi SSID length in bytes
pub const WIFI_RESULT_SSID_LENGTH: usize = 32;

/// Maximum number of country codes
pub const WIFI_MAX_COUNTRY_CODE: usize = 32;

/// Country code string size
pub const WIFI_STR_COUNTRY_CODE_SIZE: usize = 2;

/// WiFi basic complete result size in bytes
pub const WIFI_BASIC_COMPLETE_RESULT_SIZE: usize = 22;

/// WiFi basic MAC/type/channel result size in bytes
pub const WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE: usize = 9;

/// WiFi extended complete result size in bytes
pub const WIFI_EXTENDED_COMPLETE_RESULT_SIZE: usize = 79;

/// Maximum results per chunk read
pub const WIFI_N_RESULTS_MAX_PER_CHUNK: u8 = 32;

/// WiFi cumulative timing size in bytes
pub const WIFI_ALL_CUMULATIVE_TIMING_SIZE: usize = 16;

/// WiFi version size in bytes
pub const WIFI_VERSION_SIZE: usize = 2;

/// WiFi channel mask for channel 1 (2.412 GHz)
pub const WIFI_CHANNEL_1_MASK: WifiChannelMask = 0x0001;
/// WiFi channel mask for channel 2 (2.417 GHz)
pub const WIFI_CHANNEL_2_MASK: WifiChannelMask = 0x0002;
/// WiFi channel mask for channel 3 (2.422 GHz)
pub const WIFI_CHANNEL_3_MASK: WifiChannelMask = 0x0004;
/// WiFi channel mask for channel 4 (2.427 GHz)
pub const WIFI_CHANNEL_4_MASK: WifiChannelMask = 0x0008;
/// WiFi channel mask for channel 5 (2.432 GHz)
pub const WIFI_CHANNEL_5_MASK: WifiChannelMask = 0x0010;
/// WiFi channel mask for channel 6 (2.437 GHz)
pub const WIFI_CHANNEL_6_MASK: WifiChannelMask = 0x0020;
/// WiFi channel mask for channel 7 (2.442 GHz)
pub const WIFI_CHANNEL_7_MASK: WifiChannelMask = 0x0040;
/// WiFi channel mask for channel 8 (2.447 GHz)
pub const WIFI_CHANNEL_8_MASK: WifiChannelMask = 0x0080;
/// WiFi channel mask for channel 9 (2.452 GHz)
pub const WIFI_CHANNEL_9_MASK: WifiChannelMask = 0x0100;
/// WiFi channel mask for channel 10 (2.457 GHz)
pub const WIFI_CHANNEL_10_MASK: WifiChannelMask = 0x0200;
/// WiFi channel mask for channel 11 (2.462 GHz)
pub const WIFI_CHANNEL_11_MASK: WifiChannelMask = 0x0400;
/// WiFi channel mask for channel 12 (2.467 GHz)
pub const WIFI_CHANNEL_12_MASK: WifiChannelMask = 0x0800;
/// WiFi channel mask for channel 13 (2.472 GHz)
pub const WIFI_CHANNEL_13_MASK: WifiChannelMask = 0x1000;
/// WiFi channel mask for channel 14 (2.484 GHz)
pub const WIFI_CHANNEL_14_MASK: WifiChannelMask = 0x2000;
/// WiFi channel mask for all channels (1-14)
pub const WIFI_ALL_CHANNELS_MASK: WifiChannelMask = 0x3FFF;

/// WiFi channel index
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiChannel {
    NoChannel = 0x00,
    Channel1 = 0x01,  // 2.412 GHz
    Channel2 = 0x02,  // 2.417 GHz
    Channel3 = 0x03,  // 2.422 GHz
    Channel4 = 0x04,  // 2.427 GHz
    Channel5 = 0x05,  // 2.432 GHz
    Channel6 = 0x06,  // 2.437 GHz
    Channel7 = 0x07,  // 2.442 GHz
    Channel8 = 0x08,  // 2.447 GHz
    Channel9 = 0x09,  // 2.452 GHz
    Channel10 = 0x0A, // 2.457 GHz
    Channel11 = 0x0B, // 2.462 GHz
    Channel12 = 0x0C, // 2.467 GHz
    Channel13 = 0x0D, // 2.472 GHz
    Channel14 = 0x0E, // 2.484 GHz
    AllChannels = 0x0F,
}

impl From<u8> for WifiChannel {
    fn from(value: u8) -> Self {
        match value {
            0x00 => WifiChannel::NoChannel,
            0x01 => WifiChannel::Channel1,
            0x02 => WifiChannel::Channel2,
            0x03 => WifiChannel::Channel3,
            0x04 => WifiChannel::Channel4,
            0x05 => WifiChannel::Channel5,
            0x06 => WifiChannel::Channel6,
            0x07 => WifiChannel::Channel7,
            0x08 => WifiChannel::Channel8,
            0x09 => WifiChannel::Channel9,
            0x0A => WifiChannel::Channel10,
            0x0B => WifiChannel::Channel11,
            0x0C => WifiChannel::Channel12,
            0x0D => WifiChannel::Channel13,
            0x0E => WifiChannel::Channel14,
            0x0F => WifiChannel::AllChannels,
            _ => WifiChannel::NoChannel,
        }
    }
}

/// WiFi signal type for scan configuration
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiSignalTypeScan {
    /// WiFi 802.11b only
    TypeB = 0x01,
    /// WiFi 802.11g only
    TypeG = 0x02,
    /// WiFi 802.11n only (Mixed Mode, not GreenField)
    TypeN = 0x03,
    /// WiFi 802.11b, g, and n
    TypeBGN = 0x04,
}

impl WifiSignalTypeScan {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// WiFi signal type in scan results
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiSignalTypeResult {
    TypeB = 0x01,
    TypeG = 0x02,
    TypeN = 0x03,
}

impl From<u8> for WifiSignalTypeResult {
    fn from(value: u8) -> Self {
        match value {
            0x01 => WifiSignalTypeResult::TypeB,
            0x02 => WifiSignalTypeResult::TypeG,
            0x03 => WifiSignalTypeResult::TypeN,
            _ => WifiSignalTypeResult::TypeB,
        }
    }
}

/// WiFi scan mode
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiScanMode {
    /// Exposes Beacons and Probe Responses until Period Beacon field (Basic result)
    Beacon = 1,
    /// Exposes Management AP frames until Period Beacon field, and other packets until third MAC Address (Basic result)
    BeaconAndPacket = 2,
    /// Exposes Beacons and Probe Responses until FCS field (Extended result). Only WiFi B is scanned.
    FullBeacon = 4,
    /// Exposes Beacons and Probe Responses until end of SSID field (Extended result) - available since firmware 0x0306
    UntilSsid = 5,
}

impl WifiScanMode {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// WiFi result format
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiResultFormat {
    /// Basic complete result (22 bytes)
    BasicComplete,
    /// Basic MAC/type/channel result (9 bytes)
    BasicMacTypeChannel,
    /// Extended full result (79 bytes)
    ExtendedFull,
}

impl WifiResultFormat {
    /// Get the format code for reading results (sent to LR1110)
    pub fn format_code(self) -> u8 {
        match self {
            WifiResultFormat::BasicComplete => 0x01,
            WifiResultFormat::BasicMacTypeChannel => 0x04,
            WifiResultFormat::ExtendedFull => 0x01,
        }
    }

    /// Get the size of a single result in bytes
    pub fn result_size(self) -> usize {
        match self {
            WifiResultFormat::BasicComplete => WIFI_BASIC_COMPLETE_RESULT_SIZE,
            WifiResultFormat::BasicMacTypeChannel => WIFI_BASIC_MAC_TYPE_CHANNEL_RESULT_SIZE,
            WifiResultFormat::ExtendedFull => WIFI_EXTENDED_COMPLETE_RESULT_SIZE,
        }
    }
}

/// WiFi frame type
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiFrameType {
    Management = 0x00,
    Control = 0x01,
    Data = 0x02,
}

impl From<u8> for WifiFrameType {
    fn from(value: u8) -> Self {
        match value {
            0x00 => WifiFrameType::Management,
            0x01 => WifiFrameType::Control,
            0x02 => WifiFrameType::Data,
            _ => WifiFrameType::Management,
        }
    }
}

/// WiFi MAC address origin estimation
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum WifiMacOrigin {
    /// MAC address from a fixed Access Point
    BeaconFixAp = 1,
    /// MAC address from a mobile Access Point
    BeaconMobileAp = 2,
    /// Origin cannot be determined
    Unknown = 3,
}

impl From<u8> for WifiMacOrigin {
    fn from(value: u8) -> Self {
        match value {
            1 => WifiMacOrigin::BeaconFixAp,
            2 => WifiMacOrigin::BeaconMobileAp,
            _ => WifiMacOrigin::Unknown,
        }
    }
}

/// WiFi MAC address type
pub type WifiMacAddress = [u8; WIFI_MAC_ADDRESS_LENGTH];

/// WiFi firmware version
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiVersion {
    pub major: u8,
    pub minor: u8,
}

/// WiFi cumulative timing information
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiCumulativeTimings {
    /// Cumulative time spent during NFE or TOA (microseconds)
    pub rx_detection_us: u32,
    /// Cumulative time spent during preamble detection (microseconds)
    pub rx_correlation_us: u32,
    /// Cumulative time spent during signal acquisition (microseconds)
    pub rx_capture_us: u32,
    /// Cumulative time spent during software demodulation (microseconds)
    pub demodulation_us: u32,
}

/// Basic MAC/type/channel WiFi result (9 bytes)
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiBasicMacTypeChannelResult {
    /// Data rate info byte (contains signal type)
    pub data_rate_info_byte: u8,
    /// Channel info byte (contains channel and RSSI validity)
    pub channel_info_byte: u8,
    /// RSSI in dBm
    pub rssi: i8,
    /// MAC address of the access point
    pub mac_address: WifiMacAddress,
}

impl WifiBasicMacTypeChannelResult {
    /// Extract WiFi signal type from data rate info byte
    pub fn signal_type(&self) -> WifiSignalTypeResult {
        WifiSignalTypeResult::from(self.data_rate_info_byte & 0x03)
    }

    /// Extract channel from channel info byte
    pub fn channel(&self) -> WifiChannel {
        WifiChannel::from(self.channel_info_byte & 0x0F)
    }

    /// Check if RSSI value is valid
    pub fn rssi_valid(&self) -> bool {
        (self.channel_info_byte & 0x40) == 0
    }

    /// Extract MAC origin estimation from channel info byte
    pub fn mac_origin(&self) -> WifiMacOrigin {
        WifiMacOrigin::from((self.channel_info_byte & 0x30) >> 4)
    }
}

/// Basic complete WiFi result (22 bytes)
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiBasicCompleteResult {
    /// Data rate info byte
    pub data_rate_info_byte: u8,
    /// Channel info byte
    pub channel_info_byte: u8,
    /// RSSI in dBm
    pub rssi: i8,
    /// Frame type info byte
    pub frame_type_info_byte: u8,
    /// MAC address of the access point
    pub mac_address: WifiMacAddress,
    /// Phase offset
    pub phi_offset: i16,
    /// Timestamp indicating the up-time of the AP transmitting the Beacon (microseconds)
    pub timestamp_us: u64,
    /// Beacon period in TU (1 TU = 1024 microseconds)
    pub beacon_period_tu: u16,
}

impl WifiBasicCompleteResult {
    /// Extract WiFi signal type from data rate info byte
    pub fn signal_type(&self) -> WifiSignalTypeResult {
        WifiSignalTypeResult::from(self.data_rate_info_byte & 0x03)
    }

    /// Extract channel from channel info byte
    pub fn channel(&self) -> WifiChannel {
        WifiChannel::from(self.channel_info_byte & 0x0F)
    }

    /// Check if RSSI value is valid
    pub fn rssi_valid(&self) -> bool {
        (self.channel_info_byte & 0x40) == 0
    }

    /// Extract MAC origin estimation
    pub fn mac_origin(&self) -> WifiMacOrigin {
        WifiMacOrigin::from((self.channel_info_byte & 0x30) >> 4)
    }

    /// Extract frame type from frame type info byte
    pub fn frame_type(&self) -> WifiFrameType {
        WifiFrameType::from((self.frame_type_info_byte >> 6) & 0x03)
    }
}

/// FCS (Frame Check Sequence) info
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiFcsInfo {
    /// True if FCS was checked
    pub is_fcs_checked: bool,
    /// True if FCS check passed
    pub is_fcs_ok: bool,
}

/// Extended full WiFi result (79 bytes)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct WifiExtendedFullResult {
    /// Data rate info byte
    pub data_rate_info_byte: u8,
    /// Channel info byte
    pub channel_info_byte: u8,
    /// RSSI in dBm
    pub rssi: i8,
    /// Rate index
    pub rate: u8,
    /// Service value
    pub service: u16,
    /// Length of MPDU (microseconds for WiFi B, bytes for WiFi G)
    pub length: u16,
    /// Frame control structure
    pub frame_control: u16,
    /// MAC address 1
    pub mac_address_1: WifiMacAddress,
    /// MAC address 2
    pub mac_address_2: WifiMacAddress,
    /// MAC address 3
    pub mac_address_3: WifiMacAddress,
    /// Timestamp indicating the up-time of the AP (microseconds)
    pub timestamp_us: u64,
    /// Beacon period in TU
    pub beacon_period_tu: u16,
    /// Sequence control value
    pub seq_control: u16,
    /// SSID bytes (Service Set IDentifier)
    pub ssid_bytes: [u8; WIFI_RESULT_SSID_LENGTH],
    /// Current channel indicated in the WiFi frame
    pub current_channel: WifiChannel,
    /// Country code (2 characters)
    pub country_code: [u8; WIFI_STR_COUNTRY_CODE_SIZE],
    /// Input/Output regulation
    pub io_regulation: u8,
    /// FCS check info
    pub fcs_check_byte: WifiFcsInfo,
    /// Phase offset
    pub phi_offset: i16,
}

impl Default for WifiExtendedFullResult {
    fn default() -> Self {
        Self {
            data_rate_info_byte: 0,
            channel_info_byte: 0,
            rssi: 0,
            rate: 0,
            service: 0,
            length: 0,
            frame_control: 0,
            mac_address_1: [0u8; WIFI_MAC_ADDRESS_LENGTH],
            mac_address_2: [0u8; WIFI_MAC_ADDRESS_LENGTH],
            mac_address_3: [0u8; WIFI_MAC_ADDRESS_LENGTH],
            timestamp_us: 0,
            beacon_period_tu: 0,
            seq_control: 0,
            ssid_bytes: [0u8; WIFI_RESULT_SSID_LENGTH],
            current_channel: WifiChannel::NoChannel,
            country_code: [0u8; WIFI_STR_COUNTRY_CODE_SIZE],
            io_regulation: 0,
            fcs_check_byte: WifiFcsInfo::default(),
            phi_offset: 0,
        }
    }
}

impl WifiExtendedFullResult {
    /// Extract WiFi signal type from data rate info byte
    pub fn signal_type(&self) -> WifiSignalTypeResult {
        WifiSignalTypeResult::from(self.data_rate_info_byte & 0x03)
    }

    /// Extract channel from channel info byte
    pub fn channel(&self) -> WifiChannel {
        WifiChannel::from(self.channel_info_byte & 0x0F)
    }

    /// Get SSID as string (if valid UTF-8)
    pub fn ssid_str(&self) -> Option<&str> {
        // Find null terminator
        let len = self
            .ssid_bytes
            .iter()
            .position(|&c| c == 0)
            .unwrap_or(WIFI_RESULT_SSID_LENGTH);
        core::str::from_utf8(&self.ssid_bytes[..len]).ok()
    }
}

// =============================================================================
// WiFi Extension Trait
// =============================================================================

/// Extension trait that adds WiFi functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait WifiExt {
    /// Start a WiFi passive scan
    async fn wifi_scan(
        &mut self,
        signal_type: WifiSignalTypeScan,
        channel_mask: WifiChannelMask,
        scan_mode: WifiScanMode,
        max_results: u8,
        nb_scan_per_channel: u8,
        timeout_per_scan_ms: u16,
        abort_on_timeout: bool,
    ) -> Result<(), RadioError>;

    /// Start a WiFi passive scan with time limit
    async fn wifi_scan_time_limit(
        &mut self,
        signal_type: WifiSignalTypeScan,
        channel_mask: WifiChannelMask,
        scan_mode: WifiScanMode,
        max_results: u8,
        timeout_per_channel_ms: u16,
        timeout_total_ms: u16,
    ) -> Result<(), RadioError>;

    /// Get the number of WiFi results available
    async fn wifi_get_nb_results(&mut self) -> Result<u8, RadioError>;

    /// Read WiFi scan results in basic MAC/type/channel format
    async fn wifi_read_basic_mac_type_channel_results(
        &mut self,
        results: &mut [WifiBasicMacTypeChannelResult],
        start_index: u8,
        nb_results: u8,
    ) -> Result<u8, RadioError>;

    /// Read WiFi scan results in basic complete format
    async fn wifi_read_basic_complete_results(
        &mut self,
        results: &mut [WifiBasicCompleteResult],
        start_index: u8,
        nb_results: u8,
    ) -> Result<u8, RadioError>;

    /// Read cumulative WiFi scan timing information
    async fn wifi_read_cumulative_timing(&mut self) -> Result<WifiCumulativeTimings, RadioError>;

    /// Reset cumulative WiFi scan timing counters
    async fn wifi_reset_cumulative_timing(&mut self) -> Result<(), RadioError>;

    /// Read WiFi firmware version
    async fn wifi_read_version(&mut self) -> Result<WifiVersion, RadioError>;

    /// Configure the timestamp for AP phone scanning
    async fn wifi_cfg_timestamp_ap_phone(&mut self, timestamp_s: u32) -> Result<(), RadioError>;
}

// NOTE: Implementation requires lora-phy to expose low-level SPI interface.
// This will be implemented once lora-phy adds the Lr1110Interface trait.
//
// For now, users can use the types defined above with their own implementation.
