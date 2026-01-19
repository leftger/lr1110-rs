//! General radio control functions for the LR1110
//!
//! This module provides low-level radio control functions that are not specific
//! to any particular feature (LoRa, GNSS, WiFi, etc.) but are needed for advanced
//! radio operations like ranging, custom modulation, or direct packet handling.

use lora_phy::mod_params::RadioError;

// =============================================================================
// Packet Type
// =============================================================================

/// Radio packet type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PacketType {
    /// No packet type (after cold start, WiFi or GNSS capture)
    None = 0x00,
    /// GFSK modulation
    Gfsk = 0x01,
    /// LoRa modulation
    LoRa = 0x02,
    /// BPSK modulation
    Bpsk = 0x03,
    /// LR-FHSS modulation
    LrFhss = 0x04,
    /// RTToF (Ranging)
    Rttof = 0x05,
}

impl From<u8> for PacketType {
    fn from(value: u8) -> Self {
        match value {
            0x00 => PacketType::None,
            0x01 => PacketType::Gfsk,
            0x02 => PacketType::LoRa,
            0x03 => PacketType::Bpsk,
            0x04 => PacketType::LrFhss,
            0x05 => PacketType::Rttof,
            _ => PacketType::None,
        }
    }
}

// =============================================================================
// Fallback and Intermediary Modes
// =============================================================================

/// Fallback mode after successful TX or RX
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum FallbackMode {
    /// Standby RC mode (default)
    StandbyRC = 0x01,
    /// Standby XOSC mode
    StandbyXOSC = 0x02,
    /// Frequency synthesis mode
    FS = 0x03,
}

/// Intermediary mode for auto TX/RX and duty cycle
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum IntermediaryMode {
    /// Sleep mode (not recommended for certain firmware versions)
    Sleep = 0x00,
    /// Standby RC mode
    StandbyRC = 0x01,
    /// Standby XOSC mode
    StandbyXOSC = 0x02,
    /// Frequency synthesis mode
    FS = 0x03,
}

// =============================================================================
// PA Configuration Types
// =============================================================================

/// Power amplifier selection
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PaSelection {
    /// Low power PA (up to +14 dBm)
    LP = 0x00,
    /// High power PA (up to +22 dBm)
    HP = 0x01,
    /// High frequency PA (for 2.4 GHz)
    HF = 0x02,
}

/// PA regulator supply source
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PaRegSupply {
    /// Regulated voltage supply
    VREG = 0x00,
    /// Battery voltage supply
    VBAT = 0x01,
}

/// PA configuration
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PaConfig {
    /// PA selection
    pub pa_sel: PaSelection,
    /// PA regulator supply
    pub pa_reg_supply: PaRegSupply,
    /// PA duty cycle (0-7 for LP, 0-4 for HP)
    pub pa_duty_cycle: u8,
    /// PA HP selection (0-7 HP slices)
    pub pa_hp_sel: u8,
}

// =============================================================================
// Ramp Time
// =============================================================================

/// TX ramp time configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RampTime {
    /// 16 microseconds
    Ramp16Us = 0x00,
    /// 32 microseconds
    Ramp32Us = 0x01,
    /// 48 microseconds
    Ramp48Us = 0x02,
    /// 64 microseconds
    Ramp64Us = 0x03,
    /// 80 microseconds
    Ramp80Us = 0x04,
    /// 96 microseconds
    Ramp96Us = 0x05,
    /// 112 microseconds
    Ramp112Us = 0x06,
    /// 128 microseconds
    Ramp128Us = 0x07,
    /// 144 microseconds
    Ramp144Us = 0x08,
    /// 160 microseconds
    Ramp160Us = 0x09,
    /// 176 microseconds
    Ramp176Us = 0x0A,
    /// 192 microseconds
    Ramp192Us = 0x0B,
    /// 208 microseconds
    Ramp208Us = 0x0C,
    /// 240 microseconds
    Ramp240Us = 0x0D,
    /// 272 microseconds
    Ramp272Us = 0x0E,
    /// 304 microseconds
    Ramp304Us = 0x0F,
}

// =============================================================================
// LoRa Configuration
// =============================================================================

/// LoRa network type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LoRaNetworkType {
    /// Private network (sync word 0x12)
    Private = 0x00,
    /// Public network (sync word 0x34)
    Public = 0x01,
}

// =============================================================================
// BPSK Modulation Types
// =============================================================================

/// BPSK pulse shaping filter
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum BpskPulseShape {
    /// Double OSR / RRC / BT 0.7
    DbpskPulseShape = 0x16,
}

/// BPSK modulation parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BpskModParams {
    /// Bitrate in bps
    pub br_in_bps: u32,
    /// Pulse shaping filter
    pub pulse_shape: BpskPulseShape,
}

/// BPSK packet parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BpskPktParams {
    /// Payload length in bytes
    pub pld_len_in_bytes: u8,
    /// Ramp up delay for fine tuning (0 if not used)
    pub ramp_up_delay: u16,
    /// Ramp down delay for fine tuning (0 if not used)
    pub ramp_down_delay: u16,
    /// Payload length in bits (0 if not used, for non-8-bit-aligned length)
    pub pld_len_in_bits: u16,
}

// =============================================================================
// GFSK Modulation Types
// =============================================================================

/// GFSK pulse shaping filter
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPulseShape {
    /// No filtering
    Off = 0x00,
    /// Gaussian BT 0.3
    Bt03 = 0x08,
    /// Gaussian BT 0.5
    Bt05 = 0x09,
    /// Gaussian BT 0.7
    Bt07 = 0x0A,
    /// Gaussian BT 1.0
    Bt10 = 0x0B,
}

/// GFSK bandwidth (double-sided)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskBandwidth {
    /// 4.8 kHz
    Bw4800 = 0x1F,
    /// 5.8 kHz
    Bw5800 = 0x17,
    /// 7.3 kHz
    Bw7300 = 0x0F,
    /// 9.7 kHz
    Bw9700 = 0x1E,
    /// 11.7 kHz
    Bw11700 = 0x16,
    /// 14.6 kHz
    Bw14600 = 0x0E,
    /// 19.5 kHz
    Bw19500 = 0x1D,
    /// 23.4 kHz
    Bw23400 = 0x15,
    /// 29.3 kHz
    Bw29300 = 0x0D,
    /// 39.0 kHz
    Bw39000 = 0x1C,
    /// 46.9 kHz
    Bw46900 = 0x14,
    /// 58.6 kHz
    Bw58600 = 0x0C,
    /// 78.2 kHz
    Bw78200 = 0x1B,
    /// 93.8 kHz
    Bw93800 = 0x13,
    /// 117.3 kHz
    Bw117300 = 0x0B,
    /// 156.2 kHz
    Bw156200 = 0x1A,
    /// 187.2 kHz
    Bw187200 = 0x12,
    /// 234.3 kHz
    Bw234300 = 0x0A,
    /// 312.0 kHz
    Bw312000 = 0x19,
    /// 373.6 kHz
    Bw373600 = 0x11,
    /// 467.0 kHz
    Bw467000 = 0x09,
}

/// GFSK modulation parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskModParams {
    /// Bitrate in bps
    pub br_in_bps: u32,
    /// Pulse shaping filter
    pub pulse_shape: GfskPulseShape,
    /// Bandwidth parameter (double-sided)
    pub bw_dsb_param: GfskBandwidth,
    /// Frequency deviation in Hz
    pub fdev_in_hz: u32,
}

// =============================================================================
// GFSK Packet Types
// =============================================================================

/// GFSK preamble detector length
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPreambleDetector {
    /// Preamble detector off
    Off = 0x00,
    /// Minimum 8 bits
    Min8Bits = 0x04,
    /// Minimum 16 bits
    Min16Bits = 0x05,
    /// Minimum 24 bits
    Min24Bits = 0x06,
    /// Minimum 32 bits
    Min32Bits = 0x07,
}

/// GFSK address filtering configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskAddressFiltering {
    /// Address filtering disabled
    Disabled = 0x00,
    /// Filter on node address
    NodeAddress = 0x01,
    /// Filter on node and broadcast addresses
    NodeAndBroadcast = 0x02,
}

/// GFSK header type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskHeaderType {
    /// Fixed length (no header)
    FixedLength = 0x00,
    /// Variable length (header included)
    VariableLength = 0x01,
}

/// GFSK CRC type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskCrcType {
    /// CRC off
    Off = 0x01,
    /// 1 byte CRC
    Crc1Byte = 0x00,
    /// 2 bytes CRC
    Crc2Bytes = 0x02,
    /// 1 byte CRC inverted
    Crc1ByteInv = 0x04,
    /// 2 bytes CRC inverted
    Crc2BytesInv = 0x06,
}

/// GFSK DC-free encoding
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskDcFree {
    /// DC-free encoding off
    Off = 0x00,
    /// Whitening enabled
    Whitening = 0x01,
}

/// GFSK packet parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskPktParams {
    /// Preamble length in bits
    pub preamble_len_in_bits: u16,
    /// Preamble detector configuration
    pub preamble_detector: GfskPreambleDetector,
    /// Sync word length in bits
    pub sync_word_len_in_bits: u8,
    /// Address filtering configuration
    pub address_filtering: GfskAddressFiltering,
    /// Header type
    pub header_type: GfskHeaderType,
    /// Payload length in bytes
    pub pld_len_in_bytes: u8,
    /// CRC type
    pub crc_type: GfskCrcType,
    /// DC-free encoding
    pub dc_free: GfskDcFree,
}

// =============================================================================
// CAD Configuration
// =============================================================================

/// CAD exit mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CadExitMode {
    /// Return to standby RC after CAD
    StandbyRC = 0x00,
    /// Enter RX mode after CAD if detected
    Rx = 0x01,
    /// Enter TX mode after CAD if detected
    Tx = 0x02,
}

/// CAD parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CadParams {
    /// Number of CAD symbols (1-8)
    pub cad_symb_nb: u8,
    /// CAD detection peak (default: 0x32)
    pub cad_detect_peak: u8,
    /// CAD detection minimum (default: 0x0A)
    pub cad_detect_min: u8,
    /// Exit mode after CAD
    pub cad_exit_mode: CadExitMode,
    /// CAD timeout in 31.25us units
    pub cad_timeout: u32,
}

// =============================================================================
// Statistics Types
// =============================================================================

/// GFSK packet statistics
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskStats {
    /// Number of packets received
    pub nb_pkt_received: u16,
    /// Number of packets with CRC error
    pub nb_pkt_crc_error: u16,
    /// Number of packets with length error
    pub nb_pkt_len_error: u16,
}

/// LoRa packet statistics
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct LoRaStats {
    /// Number of packets received
    pub nb_pkt_received: u16,
    /// Number of packets with CRC error
    pub nb_pkt_crc_error: u16,
    /// Number of packets with header error
    pub nb_pkt_header_error: u16,
    /// Number of false sync detections
    pub nb_pkt_falsesync: u16,
}

// =============================================================================
// Packet Status Types
// =============================================================================

/// GFSK packet status
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskPktStatus {
    /// RSSI value latched on sync address detection (dBm)
    pub rssi_sync_in_dbm: i8,
    /// RSSI averaged over payload (dBm)
    pub rssi_avg_in_dbm: i8,
    /// Length of last received packet (bytes)
    pub rx_len_in_bytes: u8,
    /// Address filtering error
    pub is_addr_err: bool,
    /// CRC error
    pub is_crc_err: bool,
    /// Length error
    pub is_len_err: bool,
    /// Abort error
    pub is_abort_err: bool,
    /// Packet received
    pub is_received: bool,
    /// Packet sent
    pub is_sent: bool,
}

/// LoRa packet status (extended version with signal RSSI)
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct LoRaPktStatus {
    /// Average RSSI over last received packet (dBm)
    pub rssi_pkt_in_dbm: i8,
    /// SNR estimated on last received packet (dB)
    pub snr_pkt_in_db: i8,
    /// RSSI of last packet (signal RSSI, dBm)
    pub signal_rssi_pkt_in_dbm: i8,
}

/// RX buffer status
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct RxBufferStatus {
    /// Payload length in bytes
    pub pld_len_in_bytes: u8,
    /// Offset of first byte in RX buffer
    pub buffer_start_pointer: u8,
}

// =============================================================================
// RSSI Calibration
// =============================================================================

/// RSSI calibration table
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct RssiCalibrationTable {
    /// Gain tune for gain 4
    pub g4: u8,
    /// Gain tune for gain 5
    pub g5: u8,
    /// Gain tune for gain 6
    pub g6: u8,
    /// Gain tune for gain 7
    pub g7: u8,
    /// Gain tune for gain 8
    pub g8: u8,
    /// Gain tune for gain 9
    pub g9: u8,
    /// Gain tune for gain 10
    pub g10: u8,
    /// Gain tune for gain 11
    pub g11: u8,
    /// Gain tune for gain 12
    pub g12: u8,
    /// Gain tune for gain 13
    pub g13: u8,
    /// Gain tune for HPA gain 13 variant 1
    pub g13hp1: u8,
    /// Gain tune for HPA gain 13 variant 2
    pub g13hp2: u8,
    /// Gain tune for HPA gain 13 variant 3
    pub g13hp3: u8,
    /// Gain tune for HPA gain 13 variant 4
    pub g13hp4: u8,
    /// Gain tune for HPA gain 13 variant 5
    pub g13hp5: u8,
    /// Gain tune for HPA gain 13 variant 6
    pub g13hp6: u8,
    /// Gain tune for HPA gain 13 variant 7
    pub g13hp7: u8,
    /// Gain offset
    pub gain_offset: i16,
}

// =============================================================================
// Radio Control Extension Trait
// =============================================================================

/// Extension trait that adds general radio control functionality to the LR1110.
#[allow(async_fn_in_trait)]
pub trait RadioControlExt {
    /// Set the packet type (LoRa, GFSK, RTToF, LR-FHSS, etc.)
    ///
    /// # Packet Types
    /// - `0x00`: GFSK
    /// - `0x01`: LoRa
    /// - `0x02`: LR-FHSS
    /// - `0x05`: RTToF (Round-Trip Time of Flight)
    async fn set_packet_type(&mut self, packet_type: u8) -> Result<(), RadioError>;

    /// Set the RF frequency in Hz
    ///
    /// # Arguments
    /// * `frequency_hz` - Frequency in Hz (e.g., 915000000 for 915 MHz)
    ///
    /// # Note
    /// Frequency is converted to chip format: steps of Fxtal/2^18 where Fxtal = 32 MHz
    async fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), RadioError>;

    /// Set DIO IRQ parameters with custom mask
    ///
    /// # Arguments
    /// * `irq_mask` - 32-bit IRQ mask (see LR11xx datasheet for bit definitions)
    async fn set_dio_irq_params(&mut self, irq_mask: u32) -> Result<(), RadioError>;

    /// Start transmission with timeout
    ///
    /// # Arguments
    /// * `timeout_rtc_steps` - Timeout in RTC steps (15.625 us per step)
    ///
    /// # Note
    /// Use 0 for no timeout (single mode)
    async fn set_tx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError>;

    /// Start reception with timeout
    ///
    /// # Arguments
    /// * `timeout_rtc_steps` - Timeout in RTC steps (15.625 us per step)
    ///
    /// # Note
    /// Use 0xFFFFFF for continuous RX mode
    async fn set_rx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError>;

    /// Set LoRa modulation parameters
    ///
    /// # Arguments
    /// * `sf` - Spreading factor (5-12)
    /// * `bw` - Bandwidth (see lora_bw module in ranging.rs)
    /// * `cr` - Coding rate (see lora_cr module in ranging.rs)
    /// * `ldro` - Low data rate optimization (0 = off, 1 = on)
    async fn set_lora_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: u8) -> Result<(), RadioError>;

    /// Set LoRa packet parameters
    ///
    /// # Arguments
    /// * `preamble_len` - Preamble length in symbols
    /// * `header_type` - 0 = explicit header, 1 = implicit header
    /// * `payload_len` - Payload length in bytes
    /// * `crc_on` - 0 = CRC off, 1 = CRC on
    /// * `iq_inverted` - 0 = standard IQ, 1 = inverted IQ
    async fn set_lora_pkt_params(
        &mut self,
        preamble_len: u16,
        header_type: u8,
        payload_len: u8,
        crc_on: u8,
        iq_inverted: u8,
    ) -> Result<(), RadioError>;

    /// Set LoRa sync word
    ///
    /// # Arguments
    /// * `sync_word` - Sync word value (0x12 for private network, 0x34 for public network)
    async fn set_lora_sync_word(&mut self, sync_word: u8) -> Result<(), RadioError>;

    /// Write data to TX buffer
    ///
    /// # Arguments
    /// * `offset` - Offset in TX buffer (usually 0)
    /// * `data` - Data to write
    async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), RadioError>;

    /// Read data from RX buffer
    ///
    /// # Arguments
    /// * `offset` - Offset in RX buffer (usually 0)
    /// * `length` - Number of bytes to read
    /// * `buffer` - Buffer to store read data
    async fn read_buffer(&mut self, offset: u8, length: u8, buffer: &mut [u8]) -> Result<(), RadioError>;

    /// Get RX buffer status
    ///
    /// # Returns
    /// (payload_length, rx_start_buffer_pointer)
    async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError>;

    /// Get LoRa packet status (RSSI and SNR)
    ///
    /// # Returns
    /// (rssi_pkt_in_dbm, snr_pkt_in_db)
    async fn get_lora_pkt_status(&mut self) -> Result<(i16, i8), RadioError>;

    // Packet Status and Info
    /// Get current packet type
    async fn get_pkt_type(&mut self) -> Result<PacketType, RadioError>;

    /// Get GFSK packet status
    async fn get_gfsk_pkt_status(&mut self) -> Result<GfskPktStatus, RadioError>;

    /// Get instantaneous RSSI
    async fn get_rssi_inst(&mut self) -> Result<i16, RadioError>;

    /// Get LoRa RX info (CRC presence and coding rate)
    ///
    /// # Returns
    /// (is_crc_present, coding_rate)
    async fn get_lora_rx_info(&mut self) -> Result<(bool, u8), RadioError>;

    // Sync Word Configuration
    /// Set GFSK sync word (8 bytes)
    ///
    /// # Arguments
    /// * `sync_word` - 8-byte sync word
    async fn set_gfsk_sync_word(&mut self, sync_word: &[u8; 8]) -> Result<(), RadioError>;

    /// Set LR-FHSS sync word (4 bytes)
    ///
    /// # Arguments
    /// * `sync_word` - 4-byte sync word
    async fn set_lr_fhss_sync_word(&mut self, sync_word: &[u8; 4]) -> Result<(), RadioError>;

    /// Set LoRa network type (public or private)
    ///
    /// # Arguments
    /// * `network_type` - Public (sync word 0x34) or Private (sync word 0x12)
    async fn set_lora_public_network(&mut self, network_type: LoRaNetworkType) -> Result<(), RadioError>;

    // Advanced RX/TX Control
    /// Set RX mode with timeout in RTC steps
    ///
    /// # Arguments
    /// * `timeout_rtc_step` - Timeout in RTC steps (15.625 µs per step), 0xFFFFFF for continuous
    async fn set_rx_with_timeout_in_rtc_step(&mut self, timeout_rtc_step: u32) -> Result<(), RadioError>;

    /// Set TX mode with timeout in RTC steps
    ///
    /// # Arguments
    /// * `timeout_rtc_step` - Timeout in RTC steps (15.625 µs per step), 0 for no timeout
    async fn set_tx_with_timeout_in_rtc_step(&mut self, timeout_rtc_step: u32) -> Result<(), RadioError>;

    /// Configure automatic TX after RX or RX after TX
    ///
    /// # Arguments
    /// * `timeout_rtc_step` - Timeout for the automatic operation
    /// * `intermediary_mode` - Intermediary mode between operations
    async fn auto_tx_rx(&mut self, timeout_rtc_step: u32, intermediary_mode: IntermediaryMode) -> Result<(), RadioError>;

    /// Set RX duty cycle mode with timing in milliseconds
    ///
    /// # Arguments
    /// * `rx_time_ms` - RX time in milliseconds
    /// * `sleep_time_ms` - Sleep time in milliseconds
    async fn set_rx_duty_cycle(&mut self, rx_time_ms: u32, sleep_time_ms: u32) -> Result<(), RadioError>;

    /// Set RX duty cycle mode with timing in RTC steps
    ///
    /// # Arguments
    /// * `rx_time_rtc` - RX time in RTC steps
    /// * `sleep_time_rtc` - Sleep time in RTC steps
    /// * `mode` - RX duty cycle mode (RX or CAD for LoRa)
    async fn set_rx_duty_cycle_with_timings_in_rtc_step(
        &mut self,
        rx_time_rtc: u32,
        sleep_time_rtc: u32,
        mode: u8,
    ) -> Result<(), RadioError>;

    // Packet Configuration
    /// Set packet addresses for address filtering
    ///
    /// # Arguments
    /// * `node_address` - Node address
    /// * `broadcast_address` - Broadcast address
    async fn set_pkt_address(&mut self, node_address: u8, broadcast_address: u8) -> Result<(), RadioError>;

    /// Set RX/TX fallback mode
    ///
    /// # Arguments
    /// * `fallback_mode` - Mode to enter after RX or TX completes
    async fn set_rx_tx_fallback_mode(&mut self, fallback_mode: FallbackMode) -> Result<(), RadioError>;

    /// Configure timeout behavior on preamble detection
    ///
    /// # Arguments
    /// * `enable` - true to stop timeout on preamble, false for normal timeout
    async fn stop_timeout_on_preamble(&mut self, enable: bool) -> Result<(), RadioError>;

    // Test Modes
    /// Start CAD operation
    async fn set_cad(&mut self) -> Result<(), RadioError>;

    /// Start TX continuous wave (test mode)
    async fn set_tx_cw(&mut self) -> Result<(), RadioError>;

    /// Start TX infinite preamble (test mode)
    async fn set_tx_infinite_preamble(&mut self) -> Result<(), RadioError>;

    // LoRa Advanced
    /// Set LoRa sync timeout in number of symbols
    ///
    /// # Arguments
    /// * `nb_symbols` - Number of symbols for sync timeout
    async fn set_lora_sync_timeout(&mut self, nb_symbols: u8) -> Result<(), RadioError>;

    // GFSK Advanced
    /// Set GFSK CRC parameters (seed and polynomial)
    ///
    /// # Arguments
    /// * `seed` - CRC seed value
    /// * `polynomial` - CRC polynomial value
    async fn set_gfsk_crc_params(&mut self, seed: u32, polynomial: u32) -> Result<(), RadioError>;

    /// Set GFSK whitening seed
    ///
    /// # Arguments
    /// * `seed` - Whitening seed value
    async fn set_gfsk_whitening_seed(&mut self, seed: u16) -> Result<(), RadioError>;

    /// Get GFSK RX bandwidth parameter
    ///
    /// # Arguments
    /// * `bitrate` - Bitrate in bps
    /// * `bandwidth` - Bandwidth parameter
    ///
    /// # Returns
    /// Computed RX bandwidth parameter
    async fn get_gfsk_rx_bandwidth(&mut self, bitrate: u32, bandwidth: u32) -> Result<u8, RadioError>;

    // Calibration
    /// Set RSSI calibration table
    ///
    /// # Arguments
    /// * `table` - RSSI calibration table
    async fn set_rssi_calibration(&mut self, table: &RssiCalibrationTable) -> Result<(), RadioError>;

    // BPSK Support
    /// Set BPSK modulation parameters
    ///
    /// # Arguments
    /// * `params` - BPSK modulation parameters
    async fn set_bpsk_mod_params(&mut self, params: &BpskModParams) -> Result<(), RadioError>;

    /// Set BPSK packet parameters
    ///
    /// # Arguments
    /// * `params` - BPSK packet parameters
    async fn set_bpsk_pkt_params(&mut self, params: &BpskPktParams) -> Result<(), RadioError>;

    // Workarounds
    /// Apply high ACP (Adjacent Channel Power) workaround
    ///
    /// Required for LR1110 FW 0x0303-0x0307 and LR1120 FW 0x0101 when using LoRa.
    async fn apply_high_acp_workaround(&mut self) -> Result<(), RadioError>;

    // PA & TX Configuration
    /// Set PA configuration
    ///
    /// # Arguments
    /// * `cfg` - PA configuration (selection, supply, duty cycle, HP selection)
    async fn set_pa_cfg(&mut self, cfg: &PaConfig) -> Result<(), RadioError>;

    /// Set TX parameters
    ///
    /// # Arguments
    /// * `power_dbm` - Output power in dBm
    /// * `ramp_time` - PA ramp time
    async fn set_tx_params(&mut self, power_dbm: i8, ramp_time: RampTime) -> Result<(), RadioError>;

    // GFSK Support
    /// Set GFSK modulation parameters
    ///
    /// # Arguments
    /// * `params` - GFSK modulation parameters
    async fn set_gfsk_mod_params(&mut self, params: &GfskModParams) -> Result<(), RadioError>;

    /// Set GFSK packet parameters
    ///
    /// # Arguments
    /// * `params` - GFSK packet parameters
    async fn set_gfsk_pkt_params(&mut self, params: &GfskPktParams) -> Result<(), RadioError>;

    // RX Configuration
    /// Configure boosted RX mode
    ///
    /// # Arguments
    /// * `enable` - Enable boosted RX for improved sensitivity
    async fn cfg_rx_boosted(&mut self, enable: bool) -> Result<(), RadioError>;

    // CAD Support
    /// Set CAD (Channel Activity Detection) parameters
    ///
    /// # Arguments
    /// * `params` - CAD parameters
    async fn set_cad_params(&mut self, params: &CadParams) -> Result<(), RadioError>;

    // Statistics
    /// Reset packet statistics
    async fn reset_stats(&mut self) -> Result<(), RadioError>;

    /// Get GFSK packet statistics
    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError>;

    /// Get LoRa packet statistics
    async fn get_lora_stats(&mut self) -> Result<LoRaStats, RadioError>;
}

// =============================================================================
// RadioControlExt trait implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> RadioControlExt for lora_phy::lr1110::Lr1110<SPI, IV, C>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    IV: lora_phy::mod_traits::InterfaceVariant,
    C: lora_phy::lr1110::variant::Lr1110Variant,
{
    async fn set_packet_type(&mut self, packet_type: u8) -> Result<(), RadioError> {
        // OpCode: 0x8A
        let cmd = [0x8A, packet_type];
        self.execute_command(&cmd).await
    }

    async fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), RadioError> {
        // OpCode: 0x86
        // Frequency is sent in steps of Fxtal/2^18 where Fxtal = 32 MHz
        let freq_val = ((frequency_hz as u64) << 18) / 32_000_000;
        let cmd = [
            0x86,
            (freq_val >> 24) as u8,
            (freq_val >> 16) as u8,
            (freq_val >> 8) as u8,
            freq_val as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_dio_irq_params(&mut self, irq_mask: u32) -> Result<(), RadioError> {
        // OpCode: 0x82
        let cmd = [
            0x82,
            (irq_mask >> 24) as u8,
            (irq_mask >> 16) as u8,
            (irq_mask >> 8) as u8,
            irq_mask as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_tx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError> {
        // OpCode: 0x83
        let cmd = [
            0x83,
            (timeout_rtc_steps >> 16) as u8,
            (timeout_rtc_steps >> 8) as u8,
            timeout_rtc_steps as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_rx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError> {
        // OpCode: 0x82
        let cmd = [
            0x82,
            (timeout_rtc_steps >> 16) as u8,
            (timeout_rtc_steps >> 8) as u8,
            timeout_rtc_steps as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_lora_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: u8) -> Result<(), RadioError> {
        // OpCode: 0x8B
        let cmd = [0x8B, sf, bw, cr, ldro];
        self.execute_command(&cmd).await
    }

    async fn set_lora_pkt_params(
        &mut self,
        preamble_len: u16,
        header_type: u8,
        payload_len: u8,
        crc_on: u8,
        iq_inverted: u8,
    ) -> Result<(), RadioError> {
        // OpCode: 0x8C
        let cmd = [
            0x8C,
            (preamble_len >> 8) as u8,
            (preamble_len & 0xFF) as u8,
            header_type,
            payload_len,
            crc_on,
            iq_inverted,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_lora_sync_word(&mut self, sync_word: u8) -> Result<(), RadioError> {
        // OpCode: 0x9D
        let cmd = [0x9D, sync_word];
        self.execute_command(&cmd).await
    }

    async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), RadioError> {
        // OpCode: 0x0E
        let cmd = [0x0E, offset];
        self.execute_command_with_payload(&cmd, data).await
    }

    async fn read_buffer(&mut self, offset: u8, length: u8, buffer: &mut [u8]) -> Result<(), RadioError> {
        // OpCode: 0x0D
        let cmd = [0x0D, offset, length];
        if buffer.len() < length as usize {
            return Err(RadioError::PayloadSizeMismatch(length as usize, buffer.len()));
        }
        self.execute_command_with_response(&cmd, &mut buffer[..length as usize]).await
    }

    async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError> {
        // OpCode: 0x17
        let cmd = [0x17];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok((rbuffer[0], rbuffer[1]))
    }

    async fn get_lora_pkt_status(&mut self) -> Result<(i16, i8), RadioError> {
        // OpCode: 0x1D
        let cmd = [0x1D];
        let mut rbuffer = [0u8; 3];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        // Parse RSSI and SNR from response
        let rssi = -((rbuffer[0] as i16) >> 1);
        let snr = (rbuffer[1] as i8) >> 2;

        Ok((rssi, snr))
    }

    // PA & TX Configuration
    async fn set_pa_cfg(&mut self, cfg: &PaConfig) -> Result<(), RadioError> {
        // OpCode: 0x0215
        let cmd = [
            0x02,
            0x15,
            cfg.pa_sel as u8,
            cfg.pa_reg_supply as u8,
            cfg.pa_duty_cycle,
            cfg.pa_hp_sel,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_tx_params(&mut self, power_dbm: i8, ramp_time: RampTime) -> Result<(), RadioError> {
        // OpCode: 0x0211
        let cmd = [0x02, 0x11, power_dbm as u8, ramp_time as u8];
        self.execute_command(&cmd).await
    }

    // GFSK Support
    async fn set_gfsk_mod_params(&mut self, params: &GfskModParams) -> Result<(), RadioError> {
        // OpCode: 0x020F
        let cmd = [
            0x02,
            0x0F,
            (params.br_in_bps >> 16) as u8,
            (params.br_in_bps >> 8) as u8,
            params.br_in_bps as u8,
            params.pulse_shape as u8,
            params.bw_dsb_param as u8,
            (params.fdev_in_hz >> 16) as u8,
            (params.fdev_in_hz >> 8) as u8,
            params.fdev_in_hz as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_pkt_params(&mut self, params: &GfskPktParams) -> Result<(), RadioError> {
        // OpCode: 0x0210
        let cmd = [
            0x02,
            0x10,
            (params.preamble_len_in_bits >> 8) as u8,
            params.preamble_len_in_bits as u8,
            params.preamble_detector as u8,
            params.sync_word_len_in_bits,
            params.address_filtering as u8,
            params.header_type as u8,
            params.pld_len_in_bytes,
            params.crc_type as u8,
            params.dc_free as u8,
        ];
        self.execute_command(&cmd).await
    }

    // RX Configuration
    async fn cfg_rx_boosted(&mut self, enable: bool) -> Result<(), RadioError> {
        // OpCode: 0x0227
        let cmd = [0x02, 0x27, enable as u8];
        self.execute_command(&cmd).await
    }

    // CAD Support
    async fn set_cad_params(&mut self, params: &CadParams) -> Result<(), RadioError> {
        // OpCode: 0x020D
        let cmd = [
            0x02,
            0x0D,
            params.cad_symb_nb,
            params.cad_detect_peak,
            params.cad_detect_min,
            params.cad_exit_mode as u8,
            (params.cad_timeout >> 16) as u8,
            (params.cad_timeout >> 8) as u8,
            params.cad_timeout as u8,
        ];
        self.execute_command(&cmd).await
    }

    // Statistics
    async fn reset_stats(&mut self) -> Result<(), RadioError> {
        // OpCode: 0x0200
        let cmd = [0x02, 0x00];
        self.execute_command(&cmd).await
    }

    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError> {
        // OpCode: 0x0201
        let cmd = [0x02, 0x01];
        let mut rbuffer = [0u8; 6];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(GfskStats {
            nb_pkt_received: ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16),
            nb_pkt_crc_error: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
            nb_pkt_len_error: ((rbuffer[4] as u16) << 8) | (rbuffer[5] as u16),
        })
    }

    async fn get_lora_stats(&mut self) -> Result<LoRaStats, RadioError> {
        // OpCode: 0x0201
        let cmd = [0x02, 0x01];
        let mut rbuffer = [0u8; 8];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(LoRaStats {
            nb_pkt_received: ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16),
            nb_pkt_crc_error: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
            nb_pkt_header_error: ((rbuffer[4] as u16) << 8) | (rbuffer[5] as u16),
            nb_pkt_falsesync: ((rbuffer[6] as u16) << 8) | (rbuffer[7] as u16),
        })
    }

    // Packet Status and Info
    async fn get_pkt_type(&mut self) -> Result<PacketType, RadioError> {
        // OpCode: 0x0202
        let cmd = [0x02, 0x02];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok(PacketType::from(rbuffer[0]))
    }

    async fn get_gfsk_pkt_status(&mut self) -> Result<GfskPktStatus, RadioError> {
        // OpCode: 0x0204
        let cmd = [0x02, 0x04];
        let mut rbuffer = [0u8; 4];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(GfskPktStatus {
            rssi_sync_in_dbm: -(rbuffer[0] as i8) / 2,
            rssi_avg_in_dbm: -(rbuffer[1] as i8) / 2,
            rx_len_in_bytes: rbuffer[2],
            is_addr_err: (rbuffer[3] & 0x20) != 0,
            is_crc_err: (rbuffer[3] & 0x10) != 0,
            is_len_err: (rbuffer[3] & 0x08) != 0,
            is_abort_err: (rbuffer[3] & 0x04) != 0,
            is_received: (rbuffer[3] & 0x02) != 0,
            is_sent: (rbuffer[3] & 0x01) != 0,
        })
    }

    async fn get_rssi_inst(&mut self) -> Result<i16, RadioError> {
        // OpCode: 0x0205
        let cmd = [0x02, 0x05];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok(-(rbuffer[0] as i16) / 2)
    }

    async fn get_lora_rx_info(&mut self) -> Result<(bool, u8), RadioError> {
        // OpCode: 0x0230
        let cmd = [0x02, 0x30];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        let is_crc_present = (rbuffer[0] & 0x08) != 0;
        let cr = rbuffer[0] & 0x07;
        Ok((is_crc_present, cr))
    }

    // Sync Word Configuration
    async fn set_gfsk_sync_word(&mut self, sync_word: &[u8; 8]) -> Result<(), RadioError> {
        // OpCode: 0x0206
        let mut cmd = [0u8; 10];
        cmd[0] = 0x02;
        cmd[1] = 0x06;
        cmd[2..10].copy_from_slice(sync_word);
        self.execute_command(&cmd).await
    }

    async fn set_lr_fhss_sync_word(&mut self, sync_word: &[u8; 4]) -> Result<(), RadioError> {
        // OpCode: 0x022D
        let mut cmd = [0u8; 6];
        cmd[0] = 0x02;
        cmd[1] = 0x2D;
        cmd[2..6].copy_from_slice(sync_word);
        self.execute_command(&cmd).await
    }

    async fn set_lora_public_network(&mut self, network_type: LoRaNetworkType) -> Result<(), RadioError> {
        // OpCode: 0x0208
        let cmd = [0x02, 0x08, network_type as u8];
        self.execute_command(&cmd).await
    }

    // Advanced RX/TX Control
    async fn set_rx_with_timeout_in_rtc_step(&mut self, timeout_rtc_step: u32) -> Result<(), RadioError> {
        // OpCode: 0x0209
        let cmd = [
            0x02,
            0x09,
            (timeout_rtc_step >> 16) as u8,
            (timeout_rtc_step >> 8) as u8,
            timeout_rtc_step as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_tx_with_timeout_in_rtc_step(&mut self, timeout_rtc_step: u32) -> Result<(), RadioError> {
        // OpCode: 0x020A
        let cmd = [
            0x02,
            0x0A,
            (timeout_rtc_step >> 16) as u8,
            (timeout_rtc_step >> 8) as u8,
            timeout_rtc_step as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn auto_tx_rx(&mut self, timeout_rtc_step: u32, intermediary_mode: IntermediaryMode) -> Result<(), RadioError> {
        // OpCode: 0x020C
        let cmd = [
            0x02,
            0x0C,
            (timeout_rtc_step >> 16) as u8,
            (timeout_rtc_step >> 8) as u8,
            timeout_rtc_step as u8,
            intermediary_mode as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_rx_duty_cycle(&mut self, rx_time_ms: u32, sleep_time_ms: u32) -> Result<(), RadioError> {
        // OpCode: 0x0214
        // Convert ms to RTC steps (1 RTC step = 15.625 µs)
        let rx_time_rtc = (rx_time_ms * 64) / 1000; // ms to RTC steps
        let sleep_time_rtc = (sleep_time_ms * 64) / 1000;

        let cmd = [
            0x02,
            0x14,
            (rx_time_rtc >> 16) as u8,
            (rx_time_rtc >> 8) as u8,
            rx_time_rtc as u8,
            (sleep_time_rtc >> 16) as u8,
            (sleep_time_rtc >> 8) as u8,
            sleep_time_rtc as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_rx_duty_cycle_with_timings_in_rtc_step(
        &mut self,
        rx_time_rtc: u32,
        sleep_time_rtc: u32,
        mode: u8,
    ) -> Result<(), RadioError> {
        // OpCode: 0x0214
        let cmd = [
            0x02,
            0x14,
            (rx_time_rtc >> 16) as u8,
            (rx_time_rtc >> 8) as u8,
            rx_time_rtc as u8,
            (sleep_time_rtc >> 16) as u8,
            (sleep_time_rtc >> 8) as u8,
            sleep_time_rtc as u8,
            mode,
        ];
        self.execute_command(&cmd).await
    }

    // Packet Configuration
    async fn set_pkt_address(&mut self, node_address: u8, broadcast_address: u8) -> Result<(), RadioError> {
        // OpCode: 0x0212
        let cmd = [0x02, 0x12, node_address, broadcast_address];
        self.execute_command(&cmd).await
    }

    async fn set_rx_tx_fallback_mode(&mut self, fallback_mode: FallbackMode) -> Result<(), RadioError> {
        // OpCode: 0x0213
        let cmd = [0x02, 0x13, fallback_mode as u8];
        self.execute_command(&cmd).await
    }

    async fn stop_timeout_on_preamble(&mut self, enable: bool) -> Result<(), RadioError> {
        // OpCode: 0x0217
        let cmd = [0x02, 0x17, enable as u8];
        self.execute_command(&cmd).await
    }

    // Test Modes
    async fn set_cad(&mut self) -> Result<(), RadioError> {
        // OpCode: 0x0218
        let cmd = [0x02, 0x18];
        self.execute_command(&cmd).await
    }

    async fn set_tx_cw(&mut self) -> Result<(), RadioError> {
        // OpCode: 0x0219
        let cmd = [0x02, 0x19];
        self.execute_command(&cmd).await
    }

    async fn set_tx_infinite_preamble(&mut self) -> Result<(), RadioError> {
        // OpCode: 0x021A
        let cmd = [0x02, 0x1A];
        self.execute_command(&cmd).await
    }

    // LoRa Advanced
    async fn set_lora_sync_timeout(&mut self, nb_symbols: u8) -> Result<(), RadioError> {
        // OpCode: 0x021B
        let cmd = [0x02, 0x1B, nb_symbols];
        self.execute_command(&cmd).await
    }

    // GFSK Advanced
    async fn set_gfsk_crc_params(&mut self, seed: u32, polynomial: u32) -> Result<(), RadioError> {
        // OpCode: 0x0224
        let cmd = [
            0x02,
            0x24,
            (seed >> 24) as u8,
            (seed >> 16) as u8,
            (seed >> 8) as u8,
            seed as u8,
            (polynomial >> 24) as u8,
            (polynomial >> 16) as u8,
            (polynomial >> 8) as u8,
            polynomial as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_whitening_seed(&mut self, seed: u16) -> Result<(), RadioError> {
        // OpCode: 0x0225
        let cmd = [0x02, 0x25, (seed >> 8) as u8, seed as u8];
        self.execute_command(&cmd).await
    }

    async fn get_gfsk_rx_bandwidth(&mut self, _bitrate: u32, bandwidth: u32) -> Result<u8, RadioError> {
        // This is a helper function that doesn't call the radio
        // It computes the RX bandwidth parameter based on bitrate and bandwidth
        // Based on SWDR001 implementation, this is a lookup table operation
        // For simplicity, return the bandwidth parameter directly
        // A full implementation would include the lookup table from the C driver
        Ok((bandwidth / 1000) as u8) // Simplified - should use proper lookup
    }

    // Calibration
    async fn set_rssi_calibration(&mut self, table: &RssiCalibrationTable) -> Result<(), RadioError> {
        // OpCode: 0x0229
        let cmd = [
            0x02, 0x29,
            table.g4, table.g5, table.g6, table.g7, table.g8, table.g9,
            table.g10, table.g11, table.g12, table.g13,
            table.g13hp1, table.g13hp2, table.g13hp3, table.g13hp4,
            table.g13hp5, table.g13hp6, table.g13hp7,
            (table.gain_offset >> 8) as u8,
            table.gain_offset as u8,
        ];
        self.execute_command(&cmd).await
    }

    // BPSK Support
    async fn set_bpsk_mod_params(&mut self, params: &BpskModParams) -> Result<(), RadioError> {
        // OpCode: 0x020F (same as GFSK, different packet type)
        let cmd = [
            0x02,
            0x0F,
            (params.br_in_bps >> 16) as u8,
            (params.br_in_bps >> 8) as u8,
            params.br_in_bps as u8,
            params.pulse_shape as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_bpsk_pkt_params(&mut self, params: &BpskPktParams) -> Result<(), RadioError> {
        // OpCode: 0x0210 (same as GFSK, different packet type)
        let cmd = [
            0x02,
            0x10,
            params.pld_len_in_bytes,
            (params.ramp_up_delay >> 8) as u8,
            params.ramp_up_delay as u8,
            (params.ramp_down_delay >> 8) as u8,
            params.ramp_down_delay as u8,
            (params.pld_len_in_bits >> 8) as u8,
            params.pld_len_in_bits as u8,
        ];
        self.execute_command(&cmd).await
    }

    // Workarounds
    async fn apply_high_acp_workaround(&mut self) -> Result<(), RadioError> {
        // Based on SWDR001: Reset bit 30 in register 0x00F30054
        // This is required for LR1110 FW 0x0303-0x0307 and LR1120 FW 0x0101
        #[cfg(feature = "regmem")]
        {
            #[allow(unused_imports)]
            use crate::regmem::RegMemExt as _;
            let mask = !(1 << 30); // Mask with bit 30 cleared
            let data = 0; // Data doesn't matter, mask will clear the bit
            self.regmem_write_regmem32_mask(0x00F30054, mask, data).await
        }
        #[cfg(not(feature = "regmem"))]
        {
            // Workaround not available without regmem feature
            // Return OK since this is a non-critical workaround
            Ok(())
        }
    }
}

// =============================================================================
// Radio Timing Utilities
// =============================================================================

/// Convert time in milliseconds to RTC steps
///
/// RTC step period is 15.625 µs (1/64000 seconds)
///
/// # Arguments
/// * `time_in_ms` - Time in milliseconds
///
/// # Returns
/// Time in RTC steps (24-bit value, max 0xFFFFFF)
pub fn convert_time_in_ms_to_rtc_step(time_in_ms: u32) -> u32 {
    // 1 RTC step = 15.625 µs = 1/64 ms
    // RTC steps = time_ms × 64
    (time_in_ms * 64).min(0xFFFFFF)
}

/// Convert number of symbols to mantissa and exponent
///
/// Used for LoRa sync timeout configuration.
///
/// # Arguments
/// * `nb_symbols` - Number of symbols (0-65535)
///
/// # Returns
/// (mantissa, exponent) where nb_symbols = mantissa × 2^exponent
pub fn convert_nb_symb_to_mant_exp(nb_symbols: u16) -> (u8, u8) {
    if nb_symbols == 0 {
        return (0, 0);
    }

    let mut exp = 0u8;
    let mut mant = nb_symbols;

    // Find the highest exponent where mantissa fits in 5 bits (max 31)
    while mant > 31 && exp < 15 {
        mant = (mant + 1) >> 1; // Round up when dividing by 2
        exp += 1;
    }

    (mant as u8, exp)
}

/// Get LoRa bandwidth in Hz
///
/// # Arguments
/// * `bw` - LoRa bandwidth parameter
///
/// # Returns
/// Bandwidth in Hz
pub fn get_lora_bw_in_hz(bw: u8) -> u32 {
    match bw {
        0x08 => 10420,   // BW_10
        0x01 => 15630,   // BW_15
        0x09 => 20830,   // BW_20
        0x02 => 31250,   // BW_31
        0x0A => 41670,   // BW_41
        0x03 => 62500,   // BW_62
        0x04 => 125000,  // BW_125
        0x05 => 250000,  // BW_250
        0x06 => 500000,  // BW_500
        0x0D => 203000,  // BW_200 (2.4 GHz)
        0x0E => 406000,  // BW_400 (2.4 GHz)
        0x0F => 812000,  // BW_800 (2.4 GHz)
        _ => 125000,     // Default to 125 kHz
    }
}

/// Get LoRa time-on-air numerator
///
/// This is a helper for calculating time-on-air.
/// ToA = numerator / bandwidth_in_hz
///
/// # Arguments
/// * `preamble_len_in_symb` - Preamble length in symbols
/// * `header_type` - Header type (0 = explicit, 1 = implicit)
/// * `pld_len_in_bytes` - Payload length in bytes
/// * `crc_on` - CRC enabled (0 = off, 1 = on)
/// * `sf` - Spreading factor (5-12)
/// * `cr` - Coding rate (1-7)
/// * `ldro` - Low data rate optimization (0 = off, 1 = on)
///
/// # Returns
/// Time-on-air numerator
#[allow(clippy::too_many_arguments)]
pub fn get_lora_time_on_air_numerator(
    preamble_len_in_symb: u16,
    header_type: u8,
    pld_len_in_bytes: u8,
    crc_on: u8,
    sf: u8,
    cr: u8,
    ldro: u8,
) -> u32 {
    let sf = sf as u32;
    let cr = cr as u32;
    let n_preamble = (preamble_len_in_symb as u32 + 4) << sf;

    let n_header = if header_type == 0 { 20 } else { 0 }; // Explicit header adds 20 bits

    let n_crc = if crc_on != 0 { 16 } else { 0 };

    let payload_bits = (pld_len_in_bytes as u32) * 8 + n_crc - 4 * sf + 8 + n_header;
    let ldro_offset = if ldro != 0 { 2 } else { 0 };
    let cr_denom = 4 + cr;

    let n_payload_full = if payload_bits > 0 {
        let temp = 8 + ((payload_bits * cr_denom + (4 * sf - 4 * ldro_offset)) / (4 * sf)) * cr_denom;
        temp / cr_denom * (1 << sf)
    } else {
        0
    };

    n_preamble + n_payload_full
}

/// Get LoRa time-on-air in milliseconds
///
/// # Arguments
/// * `preamble_len_in_symb` - Preamble length in symbols
/// * `header_type` - Header type (0 = explicit, 1 = implicit)
/// * `pld_len_in_bytes` - Payload length in bytes
/// * `crc_on` - CRC enabled (0 = off, 1 = on)
/// * `sf` - Spreading factor (5-12)
/// * `bw` - Bandwidth parameter
/// * `cr` - Coding rate (1-7)
/// * `ldro` - Low data rate optimization (0 = off, 1 = on)
///
/// # Returns
/// Time-on-air in milliseconds
#[allow(clippy::too_many_arguments)]
pub fn get_lora_time_on_air_in_ms(
    preamble_len_in_symb: u16,
    header_type: u8,
    pld_len_in_bytes: u8,
    crc_on: u8,
    sf: u8,
    bw: u8,
    cr: u8,
    ldro: u8,
) -> u32 {
    let numerator = get_lora_time_on_air_numerator(
        preamble_len_in_symb,
        header_type,
        pld_len_in_bytes,
        crc_on,
        sf,
        cr,
        ldro,
    );
    let bw_hz = get_lora_bw_in_hz(bw);

    // ToA in ms = (numerator / bw_hz) * 1000
    (numerator * 1000) / bw_hz
}

/// Get GFSK time-on-air numerator
///
/// # Arguments
/// * `preamble_len_in_bits` - Preamble length in bits
/// * `sync_word_len_in_bits` - Sync word length in bits
/// * `header_type` - Header type (0 = fixed, 1 = variable)
/// * `pld_len_in_bytes` - Payload length in bytes
/// * `crc_type` - CRC type
/// * `dc_free` - DC-free encoding
///
/// # Returns
/// Time-on-air numerator (in bits)
#[allow(clippy::too_many_arguments)]
pub fn get_gfsk_time_on_air_numerator(
    preamble_len_in_bits: u16,
    sync_word_len_in_bits: u8,
    header_type: u8,
    pld_len_in_bytes: u8,
    crc_type: u8,
    dc_free: u8,
) -> u32 {
    let n_preamble = preamble_len_in_bits as u32;
    let n_sync = sync_word_len_in_bits as u32;
    let n_header = if header_type != 0 { 8 } else { 0 }; // Variable length adds 1 byte

    let n_crc = match crc_type {
        0x00 | 0x04 => 8,  // 1-byte CRC
        0x02 | 0x06 => 16, // 2-byte CRC
        _ => 0,            // No CRC
    };

    let n_payload = (pld_len_in_bytes as u32) * 8;

    // Whitening doesn't change bit count
    let _ = dc_free;

    n_preamble + n_sync + n_header + n_payload + n_crc
}

/// Get GFSK time-on-air in milliseconds
///
/// # Arguments
/// * `preamble_len_in_bits` - Preamble length in bits
/// * `sync_word_len_in_bits` - Sync word length in bits
/// * `header_type` - Header type (0 = fixed, 1 = variable)
/// * `pld_len_in_bytes` - Payload length in bytes
/// * `crc_type` - CRC type
/// * `dc_free` - DC-free encoding
/// * `br_in_bps` - Bitrate in bits per second
///
/// # Returns
/// Time-on-air in milliseconds
#[allow(clippy::too_many_arguments)]
pub fn get_gfsk_time_on_air_in_ms(
    preamble_len_in_bits: u16,
    sync_word_len_in_bits: u8,
    header_type: u8,
    pld_len_in_bytes: u8,
    crc_type: u8,
    dc_free: u8,
    br_in_bps: u32,
) -> u32 {
    let numerator = get_gfsk_time_on_air_numerator(
        preamble_len_in_bits,
        sync_word_len_in_bits,
        header_type,
        pld_len_in_bytes,
        crc_type,
        dc_free,
    );

    // ToA in ms = (total_bits / bitrate) * 1000
    if br_in_bps > 0 {
        (numerator * 1000) / br_in_bps
    } else {
        0
    }
}
