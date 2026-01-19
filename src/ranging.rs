//! RTToF (Round-Trip Time of Flight) ranging functionality for the LR1110
//!
//! This module provides distance measurement capabilities using LoRa-based
//! Round-Trip Time of Flight (RTToF) technology. The LR1110 can measure
//! distances between two devices with high accuracy using radio signal
//! propagation time.
//!
//! # Architecture
//!
//! RTToF ranging operates with two roles:
//! - **Manager**: Initiates ranging requests and measures round-trip time
//! - **Subordinate**: Responds to ranging requests
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::ranging::{RangingExt, RttofResultType, ranging_config};
//! use lora_phy::mod_params::Bandwidth;
//!
//! // Configure RTToF parameters
//! radio.rttof_set_address(ranging_config::DEFAULT_ADDRESS, 4).await?;
//! radio.rttof_set_parameters(ranging_config::RESPONSE_SYMBOLS_COUNT).await?;
//!
//! // After ranging exchange completes, read results
//! let result = radio.rttof_get_distance_result(Bandwidth::_500KHz).await?;
//! println!("Distance: {} m, RSSI: {} dBm", result.distance_m, result.rssi_dbm);
//! ```

use lora_phy::mod_params::{Bandwidth, RadioError};

// =============================================================================
// RTToF (Round-Trip Time of Flight) Types and Constants (from SWDR001 lr11xx_rttof.c/h)
// =============================================================================

/// RTToF OpCodes (16-bit commands)
///
/// Note: RTToF opcodes are in the 0x02XX range (shared with Radio opcodes)
#[derive(Clone, Copy, PartialEq)]
pub enum RttofOpCode {
    /// Set the subordinate device address (0x021C)
    SetAddress = 0x021C,
    /// Set the request address for manager mode (0x021D)
    SetRequestAddress = 0x021D,
    /// Get RTToF result (0x021E)
    GetResult = 0x021E,
    /// Set RX/TX delay indicator for calibration (0x021F)
    SetRxTxDelay = 0x021F,
    /// Set RTToF parameters (0x0228)
    SetParameters = 0x0228,
}

impl RttofOpCode {
    /// Convert opcode to bytes for SPI command
    pub fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// Length of RTToF result in bytes
pub const RTTOF_RESULT_LENGTH: usize = 4;

/// Default RTToF address
pub const RTTOF_DEFAULT_ADDRESS: u32 = 0x00000019;

/// Default number of symbols for RTToF (recommended value)
pub const RTTOF_DEFAULT_NB_SYMBOLS: u8 = 15;

/// RTToF result type
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RttofResultType {
    /// Raw distance result (needs conversion to meters)
    Raw = 0x00,
    /// RSSI result
    Rssi = 0x01,
}

impl RttofResultType {
    /// Get the value for SPI command
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// Type alias for RTToF raw result (4 bytes)
pub type RttofRawResult = [u8; RTTOF_RESULT_LENGTH];

/// RTToF distance result with metadata
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct RttofDistanceResult {
    /// Distance in meters (can be negative for calibration offsets)
    pub distance_m: i32,
    /// RSSI in dBm
    pub rssi_dbm: i8,
}

/// Convert raw RTToF distance result to meters
///
/// # Arguments
/// * `bandwidth` - LoRa bandwidth used during RTToF measurement
/// * `raw_result` - 4-byte raw distance result from device
///
/// # Returns
/// Distance in meters (can be negative)
pub fn rttof_distance_raw_to_meters(bandwidth: Bandwidth, raw_result: &RttofRawResult) -> i32 {
    let bitcnt: u8 = 24;

    // Bandwidth scaling factor
    let bw_scaling: i32 = match bandwidth {
        Bandwidth::_500KHz => 1,
        Bandwidth::_250KHz => 2,
        Bandwidth::_125KHz => 4,
        _ => 1, // Default to 500 kHz scaling for unsupported bandwidths
    };

    // Parse raw distance (big-endian)
    let raw_distance: u32 = ((raw_result[0] as u32) << 24)
        | ((raw_result[1] as u32) << 16)
        | ((raw_result[2] as u32) << 8)
        | (raw_result[3] as u32);

    // Convert to signed value (24-bit two's complement)
    let mut retval = raw_distance as i32;
    if raw_distance >= (1u32 << (bitcnt - 1)) {
        retval -= 1i32 << bitcnt;
    }

    // Calculate distance: 300 * bw_scaling * raw / 4096
    300 * bw_scaling * retval / 4096
}

/// Convert raw RTToF RSSI result to dBm
///
/// # Arguments
/// * `raw_result` - 4-byte raw RSSI result from device
///
/// # Returns
/// RSSI in dBm
pub fn rttof_rssi_raw_to_dbm(raw_result: &RttofRawResult) -> i8 {
    // Only the last byte is meaningful
    -((raw_result[3] >> 1) as i8)
}

// =============================================================================
// RTToF Ranging Constants for Demo Application
// =============================================================================

/// Packet type values for use with set_packet_type()
pub mod packet_type {
    /// LoRa packet type
    pub const LORA: u8 = 0x02;
    /// RTToF (Round-Trip Time of Flight) packet type for ranging
    pub const RTTOF: u8 = 0x05;
}

/// Ranging configuration constants (matching lr11xx_ranging_demo)
pub mod ranging_config {
    /// Default ranging address
    pub const DEFAULT_ADDRESS: u32 = 0x32101222;

    /// Number of address bytes the subordinate checks (1-4)
    pub const SUBORDINATE_CHECK_LENGTH_BYTES: u8 = 4;

    /// Number of symbols in ranging response
    pub const RESPONSE_SYMBOLS_COUNT: u8 = 15;

    /// Payload length for LoRa initialization packets
    pub const INIT_PAYLOAD_LENGTH: usize = 6;

    /// Processing time between ranging channels (ms)
    pub const DONE_PROCESSING_TIME_MS: u32 = 5;

    /// Maximum number of frequency hopping channels
    pub const MAX_HOPPING_CHANNELS: usize = 39;

    /// Minimum successful measurements for valid result
    pub const MIN_HOPPING_CHANNELS: usize = 10;

    /// LoRa sync word for private network
    pub const LORA_SYNC_WORD: u8 = 0x34;

    /// Continuous RX timeout value
    pub const RX_CONTINUOUS: u32 = 0xFFFFFF;
}

/// Frequency hopping channel tables for different regions
pub mod ranging_channels {
    /// ISM 902-928 MHz (US915) - 39 channels
    pub const US915: [u32; 39] = [
        907850000, 902650000, 914350000, 906550000, 905900000, 924750000, 926700000, 918250000,
        921500000, 909150000, 907200000, 924100000, 903950000, 910450000, 917600000, 919550000,
        923450000, 925400000, 909800000, 915000000, 912400000, 904600000, 908500000, 911100000,
        911750000, 916300000, 918900000, 905250000, 913700000, 927350000, 926050000, 916950000,
        913050000, 903300000, 920200000, 922800000, 915650000, 922150000, 920850000,
    ];

    /// ISM 863-870 MHz (EU868) - 39 channels
    pub const EU868: [u32; 39] = [
        863750000, 865100000, 864800000, 868400000, 865250000, 867500000, 865550000, 867650000,
        866150000, 864050000, 867800000, 863300000, 863450000, 867950000, 868550000, 868850000,
        867200000, 867050000, 864650000, 863900000, 864500000, 866450000, 865400000, 868700000,
        863150000, 866750000, 866300000, 864950000, 864350000, 866000000, 866900000, 868250000,
        865850000, 865700000, 867350000, 868100000, 863600000, 866600000, 864200000,
    ];

    /// ISM 490-510 MHz (CN490) - 39 channels
    pub const CN490: [u32; 39] = [
        490810000, 508940000, 496690000, 507470000, 504040000, 508450000, 505020000, 497670000,
        497180000, 500610000, 494240000, 493260000, 495710000, 491300000, 504530000, 501100000,
        502080000, 501590000, 499140000, 494730000, 506980000, 492280000, 509430000, 495220000,
        492770000, 507960000, 493750000, 499630000, 496200000, 498160000, 505510000, 500120000,
        503060000, 506000000, 506490000, 498650000, 491790000, 503550000, 502570000,
    ];

    /// ISM 2.4 GHz - 39 channels
    pub const ISM2G4: [u32; 39] = [
        2450000000, 2402000000, 2476000000, 2436000000, 2430000000, 2468000000, 2458000000,
        2416000000, 2424000000, 2478000000, 2456000000, 2448000000, 2462000000, 2472000000,
        2432000000, 2446000000, 2422000000, 2442000000, 2460000000, 2474000000, 2414000000,
        2464000000, 2454000000, 2444000000, 2404000000, 2434000000, 2410000000, 2408000000,
        2440000000, 2452000000, 2480000000, 2426000000, 2428000000, 2466000000, 2418000000,
        2412000000, 2406000000, 2470000000, 2438000000,
    ];
}

/// LoRa spreading factor values
pub mod lora_sf {
    /// SF5
    pub const SF5: u8 = 0x05;
    /// SF6
    pub const SF6: u8 = 0x06;
    /// SF7
    pub const SF7: u8 = 0x07;
    /// SF8
    pub const SF8: u8 = 0x08;
    /// SF9
    pub const SF9: u8 = 0x09;
    /// SF10
    pub const SF10: u8 = 0x0A;
    /// SF11
    pub const SF11: u8 = 0x0B;
    /// SF12
    pub const SF12: u8 = 0x0C;
}

/// LoRa bandwidth values
pub mod lora_bw {
    /// 125 kHz
    pub const BW_125: u8 = 0x04;
    /// 250 kHz
    pub const BW_250: u8 = 0x05;
    /// 500 kHz
    pub const BW_500: u8 = 0x06;
}

/// LoRa coding rate values
pub mod lora_cr {
    /// CR 4/5
    pub const CR_4_5: u8 = 0x01;
    /// CR 4/6
    pub const CR_4_6: u8 = 0x02;
    /// CR 4/7
    pub const CR_4_7: u8 = 0x03;
    /// CR 4/8
    pub const CR_4_8: u8 = 0x04;
}

/// Calculate single symbol time in milliseconds
///
/// # Arguments
/// * `bw` - Bandwidth value (from lora_bw module)
/// * `sf` - Spreading factor value (from lora_sf module)
///
/// # Returns
/// Symbol time in milliseconds as f32
pub fn calculate_symbol_time_ms(bw: u8, sf: u8) -> f32 {
    let bw_khz: f32 = match bw {
        0x04 => 125.0, // BW_125
        0x05 => 250.0, // BW_250
        0x06 => 500.0, // BW_500
        _ => 500.0,
    };

    let sf_val = sf as u32;
    (1u32 << sf_val) as f32 / bw_khz
}

/// Calculate ranging request delay in milliseconds
///
/// This calculates the time for a complete ranging exchange including:
/// - Preamble
/// - Frequency sync (4.25 symbols, 6.25 for SF5/SF6)
/// - Double header (16 symbols)
/// - Ranging request (15 symbols)
/// - Ranging silence (2 symbols)
/// - Response symbols
///
/// # Arguments
/// * `bw` - Bandwidth value
/// * `sf` - Spreading factor value
/// * `preamble_len` - Preamble length in symbols
/// * `response_symbols` - Number of response symbols
///
/// # Returns
/// Delay in milliseconds
pub fn calculate_ranging_request_delay_ms(
    bw: u8,
    sf: u8,
    preamble_len: u16,
    response_symbols: u8,
) -> u32 {
    let symbol_time_ms = calculate_symbol_time_ms(bw, sf);

    // Extra symbols for SF5/SF6
    let extra_symbols: f32 = if sf == lora_sf::SF5 || sf == lora_sf::SF6 {
        2.0
    } else {
        0.0
    };

    // Total symbols for ranging exchange
    // Preamble + FreqSync(4.25) + DoubleHeader(16) + Request(15) + Silence(2) + Response
    let freq_sync_symbols = 4.25;
    let double_header_symbols = 16.0;
    let request_symbols = 15.0;
    let silence_symbols = 2.0;

    let total_symbols = preamble_len as f32
        + freq_sync_symbols
        + double_header_symbols
        + request_symbols
        + silence_symbols
        + response_symbols as f32
        + extra_symbols;

    // Add PA ramp time (approximately 0.3ms for typical values) and processing time
    let pa_ramp_ms = 0.3;
    let delay_ms = (symbol_time_ms * total_symbols)
        + pa_ramp_ms
        + ranging_config::DONE_PROCESSING_TIME_MS as f32
        + 1.0;

    delay_ms as u32
}

// =============================================================================
// Ranging Extension Trait
// =============================================================================

/// Extension trait that adds RTToF ranging functionality to the LR1110 radio.
///
/// # Note
/// This trait provides only RTToF-specific methods. For general radio control
/// functions like setting frequency, packet type, TX/RX modes, etc., use the
/// `RadioControlExt` trait from the `radio` module.
///
/// # Example
/// ```ignore
/// use lr1110_rs::ranging::RangingExt;
/// use lr1110_rs::radio::RadioControlExt;
///
/// // Configure RTToF parameters
/// radio.rttof_set_address(0x12345678, 4).await?;
/// radio.rttof_set_parameters(15).await?;
///
/// // Use radio control methods for frequency hopping
/// radio.set_packet_type(packet_type::RTTOF).await?;
/// radio.set_rf_frequency(915000000).await?;
/// radio.set_tx(5000).await?;
///
/// // Read ranging result
/// let result = radio.rttof_get_distance_result(Bandwidth::_500KHz).await?;
/// ```
#[allow(async_fn_in_trait)]
pub trait RangingExt {
    /// Set the RTToF address for this subordinate device
    ///
    /// The address is used in subordinate mode when receiving RTToF requests.
    /// The subordinate compares `check_length` bytes (LSB first) of the request
    /// address with its own address. Non-matching packets are discarded.
    async fn rttof_set_address(&mut self, address: u32, check_length: u8)
        -> Result<(), RadioError>;

    /// Set the RTToF request address for manager mode
    ///
    /// The request address is copied into the RTToF request packets sent
    /// when operating as manager.
    async fn rttof_set_request_address(&mut self, request_address: u32) -> Result<(), RadioError>;

    /// Set the RX/TX delay indicator for RTToF calibration
    ///
    /// The transceiver hardware induces a delay depending on the physical layer
    /// configuration (bandwidth, spreading factor). This delay needs to be
    /// compensated by a calibration value for accurate RTToF measurements.
    async fn rttof_set_rx_tx_delay_indicator(
        &mut self,
        delay_indicator: u32,
    ) -> Result<(), RadioError>;

    /// Configure RTToF specific parameters
    ///
    /// The RTToF parameters must be configured in both manager and subordinate devices.
    /// A value of 15 symbols balances RTToF accuracy and power consumption.
    async fn rttof_set_parameters(&mut self, nb_symbols: u8) -> Result<(), RadioError>;

    /// Get the raw RTToF result from the manager device
    ///
    /// Use `rttof_distance_raw_to_meters()` or `rttof_rssi_raw_to_dbm()` to convert.
    async fn rttof_get_raw_result(
        &mut self,
        result_type: RttofResultType,
    ) -> Result<RttofRawResult, RadioError>;

    /// Get complete RTToF distance result with RSSI
    ///
    /// Convenience function that retrieves both distance and RSSI results
    /// and converts them to meaningful units.
    async fn rttof_get_distance_result(
        &mut self,
        bandwidth: Bandwidth,
    ) -> Result<RttofDistanceResult, RadioError>;
}

// =============================================================================
// RangingExt trait implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> RangingExt for lora_phy::lr1110::Lr1110<SPI, IV, C>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    IV: lora_phy::mod_traits::InterfaceVariant,
    C: lora_phy::lr1110::variant::Lr1110Variant,
{
    async fn rttof_set_address(
        &mut self,
        address: u32,
        check_length: u8,
    ) -> Result<(), RadioError> {
        let opcode = RttofOpCode::SetAddress.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
            check_length,
        ];
        self.execute_command(&cmd).await
    }

    async fn rttof_set_request_address(&mut self, request_address: u32) -> Result<(), RadioError> {
        let opcode = RttofOpCode::SetRequestAddress.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (request_address >> 24) as u8,
            (request_address >> 16) as u8,
            (request_address >> 8) as u8,
            request_address as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn rttof_set_rx_tx_delay_indicator(
        &mut self,
        delay_indicator: u32,
    ) -> Result<(), RadioError> {
        let opcode = RttofOpCode::SetRxTxDelay.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (delay_indicator >> 24) as u8,
            (delay_indicator >> 16) as u8,
            (delay_indicator >> 8) as u8,
            delay_indicator as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn rttof_set_parameters(&mut self, nb_symbols: u8) -> Result<(), RadioError> {
        let opcode = RttofOpCode::SetParameters.bytes();
        let cmd = [opcode[0], opcode[1], nb_symbols];
        self.execute_command(&cmd).await
    }

    async fn rttof_get_raw_result(
        &mut self,
        result_type: RttofResultType,
    ) -> Result<RttofRawResult, RadioError> {
        let opcode = RttofOpCode::GetResult.bytes();
        let cmd = [opcode[0], opcode[1], result_type.value()];
        let mut rbuffer = [0u8; RTTOF_RESULT_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer)
    }

    async fn rttof_get_distance_result(
        &mut self,
        bandwidth: Bandwidth,
    ) -> Result<RttofDistanceResult, RadioError> {
        // Get distance result
        let raw_distance = self.rttof_get_raw_result(RttofResultType::Raw).await?;
        let distance_m = rttof_distance_raw_to_meters(bandwidth, &raw_distance);

        // Get RSSI result
        let raw_rssi = self.rttof_get_raw_result(RttofResultType::Rssi).await?;
        let rssi_dbm = rttof_rssi_raw_to_dbm(&raw_rssi);

        Ok(RttofDistanceResult {
            distance_m,
            rssi_dbm,
        })
    }
}
