//! GFSK (Gaussian Frequency Shift Keying) functionality for the LR1110
//!
//! This module provides GFSK modulation and packet configuration for the
//! LR11xx family of transceivers.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::gfsk::{GfskExt, GfskModulationParams, GfskPacketParams};
//!
//! // Set up GFSK modulation parameters
//! let mod_params = GfskModulationParams::default();
//! radio.set_gfsk_mod_params(&mod_params).await?;
//!
//! // Set up GFSK packet parameters
//! let pkt_params = GfskPacketParams::default();
//! radio.set_gfsk_pkt_params(&pkt_params).await?;
//!
//! // Set sync word
//! radio.set_gfsk_sync_word(&[0xC1, 0x94, 0xC1, 0x94]).await?;
//! ```

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// GFSK OpCodes
// =============================================================================

/// Radio OpCodes used for GFSK configuration
#[derive(Clone, Copy)]
enum RadioOpCode {
    SetModulationParam = 0x020F,
    SetPktParam = 0x0210,
    SetGfskSyncWord = 0x0206,
    SetGfskCrcParams = 0x0224,
    SetGfskWhiteningParams = 0x0225,
    SetPktAdrs = 0x0212,
    GetPktStatus = 0x0204,
    GetStats = 0x0201,
    ResetStats = 0x0200,
}

impl RadioOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

// =============================================================================
// GFSK Types and Constants
// =============================================================================

/// LR1110 crystal frequency (32 MHz)
const LR1110_XTAL_FREQ: u32 = 32_000_000;

/// GFSK pulse shaping filter
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPulseShape {
    /// No filter
    Off = 0x00,
    /// Gaussian BT=0.3
    Bt03 = 0x08,
    /// Gaussian BT=0.5
    Bt05 = 0x09,
    /// Gaussian BT=0.7
    Bt07 = 0x0A,
    /// Gaussian BT=1
    Bt1 = 0x0B,
}

impl GfskPulseShape {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK bandwidth (receiver bandwidth for RX)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskBandwidth {
    /// 4.8 kHz DSB
    Bw4800 = 0x1F,
    /// 5.8 kHz DSB
    Bw5800 = 0x17,
    /// 7.3 kHz DSB
    Bw7300 = 0x0F,
    /// 9.7 kHz DSB
    Bw9700 = 0x1E,
    /// 11.7 kHz DSB
    Bw11700 = 0x16,
    /// 14.6 kHz DSB
    Bw14600 = 0x0E,
    /// 19.5 kHz DSB
    Bw19500 = 0x1D,
    /// 23.4 kHz DSB
    Bw23400 = 0x15,
    /// 29.3 kHz DSB
    Bw29300 = 0x0D,
    /// 39.0 kHz DSB
    Bw39000 = 0x1C,
    /// 46.9 kHz DSB
    Bw46900 = 0x14,
    /// 58.6 kHz DSB
    Bw58600 = 0x0C,
    /// 78.2 kHz DSB
    Bw78200 = 0x1B,
    /// 93.8 kHz DSB
    Bw93800 = 0x13,
    /// 117.3 kHz DSB
    Bw117300 = 0x0B,
    /// 156.2 kHz DSB
    Bw156200 = 0x1A,
    /// 187.2 kHz DSB
    Bw187200 = 0x12,
    /// 234.3 kHz DSB
    Bw234300 = 0x0A,
    /// 312.0 kHz DSB
    Bw312000 = 0x19,
    /// 373.6 kHz DSB
    Bw373600 = 0x11,
    /// 467.0 kHz DSB
    Bw467000 = 0x09,
}

impl GfskBandwidth {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK preamble detector length
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPreambleDetector {
    /// Preamble detection disabled
    Off = 0x00,
    /// Detect 8 bits preamble
    Bits8 = 0x04,
    /// Detect 16 bits preamble
    Bits16 = 0x05,
    /// Detect 24 bits preamble
    Bits24 = 0x06,
    /// Detect 32 bits preamble
    Bits32 = 0x07,
}

impl GfskPreambleDetector {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK address filtering
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskAddressFiltering {
    /// Address filtering disabled
    Disabled = 0x00,
    /// Filter on node address
    Node = 0x01,
    /// Filter on node and broadcast addresses
    NodeAndBroadcast = 0x02,
}

impl GfskAddressFiltering {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK packet header type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskHeaderType {
    /// Fixed length packet (no length field)
    Fixed = 0x00,
    /// Variable length packet (length in header)
    Variable = 0x01,
}

impl GfskHeaderType {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK CRC type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskCrcType {
    /// CRC disabled
    Off = 0x01,
    /// 1-byte CRC
    Crc1Byte = 0x00,
    /// 2-byte CRC
    Crc2Bytes = 0x02,
    /// 1-byte CRC, inverted
    Crc1ByteInv = 0x04,
    /// 2-byte CRC, inverted
    Crc2BytesInv = 0x06,
}

impl GfskCrcType {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// GFSK whitening configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskDcFree {
    /// Whitening disabled
    Off = 0x00,
    /// Whitening enabled
    Whitening = 0x01,
}

impl GfskDcFree {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// Maximum length of GFSK sync word in bytes
pub const GFSK_SYNC_WORD_MAX_LENGTH: usize = 8;

/// Default GFSK sync word (4 bytes)
pub const GFSK_DEFAULT_SYNC_WORD: [u8; 8] = [0xC1, 0x94, 0xC1, 0x94, 0x00, 0x00, 0x00, 0x00];

/// GFSK modulation parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskModulationParams {
    /// Bitrate in bits per second (600 to 300000)
    pub bitrate_bps: u32,
    /// Pulse shaping filter
    pub pulse_shape: GfskPulseShape,
    /// Receiver bandwidth
    pub bandwidth: GfskBandwidth,
    /// Frequency deviation in Hz
    pub freq_dev_hz: u32,
}

impl Default for GfskModulationParams {
    fn default() -> Self {
        Self {
            bitrate_bps: 50000,
            pulse_shape: GfskPulseShape::Bt1,
            bandwidth: GfskBandwidth::Bw117300,
            freq_dev_hz: 25000,
        }
    }
}

/// GFSK packet parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskPacketParams {
    /// Preamble length in bits (must be multiple of 8)
    pub preamble_length: u16,
    /// Preamble detector length
    pub preamble_detector: GfskPreambleDetector,
    /// Sync word length in bits (0-64, must be multiple of 8)
    pub sync_word_length_bits: u8,
    /// Address filtering mode
    pub address_filtering: GfskAddressFiltering,
    /// Header type (fixed or variable length)
    pub header_type: GfskHeaderType,
    /// Payload length in bytes (for fixed length packets)
    pub payload_length: u8,
    /// CRC type
    pub crc_type: GfskCrcType,
    /// DC-free encoding (whitening)
    pub dc_free: GfskDcFree,
}

impl Default for GfskPacketParams {
    fn default() -> Self {
        Self {
            preamble_length: 32,
            preamble_detector: GfskPreambleDetector::Bits16,
            sync_word_length_bits: 32,
            address_filtering: GfskAddressFiltering::Disabled,
            header_type: GfskHeaderType::Variable,
            payload_length: 255,
            crc_type: GfskCrcType::Crc2Bytes,
            dc_free: GfskDcFree::Whitening,
        }
    }
}

/// GFSK statistics
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskStats {
    /// Number of received packets
    pub nb_pkt_received: u16,
    /// Number of packets received with CRC error
    pub nb_pkt_crc_error: u16,
    /// Number of packets received with length error
    pub nb_pkt_len_error: u16,
}

// =============================================================================
// GFSK Extension Trait
// =============================================================================

/// Extension trait that adds GFSK functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait GfskExt {
    /// Set GFSK modulation parameters
    ///
    /// # Arguments
    /// * `params` - GFSK modulation parameters (bitrate, pulse shape, bandwidth, frequency deviation)
    async fn set_gfsk_mod_params(
        &mut self,
        params: &GfskModulationParams,
    ) -> Result<(), RadioError>;

    /// Set GFSK packet parameters
    ///
    /// # Arguments
    /// * `params` - GFSK packet parameters
    async fn set_gfsk_pkt_params(&mut self, params: &GfskPacketParams) -> Result<(), RadioError>;

    /// Set GFSK sync word
    ///
    /// Sets the sync word used for GFSK packet detection.
    /// The sync word can be up to 8 bytes (64 bits).
    ///
    /// # Arguments
    /// * `sync_word` - Sync word bytes (up to 8 bytes)
    async fn set_gfsk_sync_word(&mut self, sync_word: &[u8]) -> Result<(), RadioError>;

    /// Set GFSK CRC parameters
    ///
    /// # Arguments
    /// * `seed` - CRC seed value
    /// * `polynomial` - CRC polynomial
    async fn set_gfsk_crc_params(&mut self, seed: u32, polynomial: u32) -> Result<(), RadioError>;

    /// Set GFSK whitening parameters
    ///
    /// # Arguments
    /// * `seed` - Whitening seed value (16-bit)
    async fn set_gfsk_whitening_params(&mut self, seed: u16) -> Result<(), RadioError>;

    /// Set node and broadcast addresses for GFSK address filtering
    ///
    /// # Arguments
    /// * `node_address` - Node address
    /// * `broadcast_address` - Broadcast address
    async fn set_gfsk_pkt_address(
        &mut self,
        node_address: u8,
        broadcast_address: u8,
    ) -> Result<(), RadioError>;

    /// Get GFSK packet status
    ///
    /// # Returns
    /// (rx_length, rssi_sync_dbm, rssi_avg_dbm)
    async fn get_gfsk_packet_status(&mut self) -> Result<(u8, i16, i16), RadioError>;

    /// Get GFSK packet statistics
    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError>;

    /// Reset GFSK packet statistics
    async fn reset_gfsk_stats(&mut self) -> Result<(), RadioError>;
}

// =============================================================================
// Implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> GfskExt for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn set_gfsk_mod_params(
        &mut self,
        params: &GfskModulationParams,
    ) -> Result<(), RadioError> {
        // Convert bitrate to chip format: (32 * 32000000) / bitrate
        let br = ((32u64 * LR1110_XTAL_FREQ as u64) / params.bitrate_bps as u64) as u32;
        // Convert frequency deviation to chip format: (fdev * 2^25) / 32000000
        let fdev = ((params.freq_dev_hz as u64) << 25) / (LR1110_XTAL_FREQ as u64);

        let opcode = RadioOpCode::SetModulationParam.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            ((br >> 24) & 0xFF) as u8,
            ((br >> 16) & 0xFF) as u8,
            ((br >> 8) & 0xFF) as u8,
            (br & 0xFF) as u8,
            params.pulse_shape.value(),
            params.bandwidth.value(),
            ((fdev >> 24) & 0xFF) as u8,
            ((fdev >> 16) & 0xFF) as u8,
            ((fdev >> 8) & 0xFF) as u8,
            (fdev & 0xFF) as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_pkt_params(&mut self, params: &GfskPacketParams) -> Result<(), RadioError> {
        let opcode = RadioOpCode::SetPktParam.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            ((params.preamble_length >> 8) & 0xFF) as u8,
            (params.preamble_length & 0xFF) as u8,
            params.preamble_detector.value(),
            params.sync_word_length_bits,
            params.address_filtering.value(),
            params.header_type.value(),
            params.payload_length,
            params.crc_type.value(),
            params.dc_free.value(),
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_sync_word(&mut self, sync_word: &[u8]) -> Result<(), RadioError> {
        if sync_word.len() > GFSK_SYNC_WORD_MAX_LENGTH {
            return Err(RadioError::PayloadSizeMismatch(
                GFSK_SYNC_WORD_MAX_LENGTH,
                sync_word.len(),
            ));
        }

        let opcode = RadioOpCode::SetGfskSyncWord.bytes();
        let mut cmd = [0u8; 10]; // 2 opcode + 8 sync word
        cmd[0] = opcode[0];
        cmd[1] = opcode[1];
        cmd[2..2 + sync_word.len()].copy_from_slice(sync_word);

        self.execute_command(&cmd[..2 + sync_word.len()]).await
    }

    async fn set_gfsk_crc_params(&mut self, seed: u32, polynomial: u32) -> Result<(), RadioError> {
        let opcode = RadioOpCode::SetGfskCrcParams.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            ((seed >> 24) & 0xFF) as u8,
            ((seed >> 16) & 0xFF) as u8,
            ((seed >> 8) & 0xFF) as u8,
            (seed & 0xFF) as u8,
            ((polynomial >> 24) & 0xFF) as u8,
            ((polynomial >> 16) & 0xFF) as u8,
            ((polynomial >> 8) & 0xFF) as u8,
            (polynomial & 0xFF) as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_whitening_params(&mut self, seed: u16) -> Result<(), RadioError> {
        let opcode = RadioOpCode::SetGfskWhiteningParams.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            ((seed >> 8) & 0xFF) as u8,
            (seed & 0xFF) as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_pkt_address(
        &mut self,
        node_address: u8,
        broadcast_address: u8,
    ) -> Result<(), RadioError> {
        let opcode = RadioOpCode::SetPktAdrs.bytes();
        let cmd = [opcode[0], opcode[1], node_address, broadcast_address];
        self.execute_command(&cmd).await
    }

    async fn get_gfsk_packet_status(&mut self) -> Result<(u8, i16, i16), RadioError> {
        let opcode = RadioOpCode::GetPktStatus.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; 4];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        // Parse RSSI values (raw values are unsigned, convert to dBm)
        let rssi_sync_dbm = -((rbuffer[1] as i16) / 2);
        let rssi_avg_dbm = -((rbuffer[2] as i16) / 2);

        Ok((rbuffer[0], rssi_sync_dbm, rssi_avg_dbm))
    }

    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError> {
        let opcode = RadioOpCode::GetStats.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; 6];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        Ok(GfskStats {
            nb_pkt_received: ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16),
            nb_pkt_crc_error: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
            nb_pkt_len_error: ((rbuffer[4] as u16) << 8) | (rbuffer[5] as u16),
        })
    }

    async fn reset_gfsk_stats(&mut self) -> Result<(), RadioError> {
        let opcode = RadioOpCode::ResetStats.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }
}
