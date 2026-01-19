//! LR-FHSS (Long Range Frequency Hopping Spread Spectrum) functionality for the LR1110
//!
//! This module provides LR-FHSS modulation capabilities for the LR11xx family
//! of transceivers, enabling frequency hopping spread spectrum communication.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::lr_fhss::{
//!     LrFhssExt, LrFhssParams, LrFhssV1Params, LrFhssBandwidth, LrFhssCodingRate,
//!     LrFhssGrid, LrFhssModulationType, LR_FHSS_DEFAULT_SYNC_WORD
//! };
//!
//! // Configure LR-FHSS parameters
//! let params = LrFhssParams {
//!     lr_fhss_params: LrFhssV1Params {
//!         sync_word: LR_FHSS_DEFAULT_SYNC_WORD,
//!         modulation_type: LrFhssModulationType::Gmsk488,
//!         coding_rate: LrFhssCodingRate::Cr5_6,
//!         grid: LrFhssGrid::Grid3906Hz,
//!         enable_hopping: true,
//!         bandwidth: LrFhssBandwidth::Bw136719Hz,
//!         header_count: 2,
//!     },
//!     device_offset: 0,
//! };
//!
//! // Initialize LR-FHSS mode
//! radio.lr_fhss_init().await?;
//!
//! // Build and transmit frame
//! let hop_seq_count = lr_fhss_get_hop_sequence_count(&params);
//! radio.lr_fhss_build_frame(&params, 0, &payload).await?;
//! ```

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// LR-FHSS OpCodes
// =============================================================================

/// LR-FHSS specific OpCodes
#[derive(Clone, Copy)]
enum LrFhssOpCode {
    /// Build LR-FHSS frame (0x022C)
    BuildFrame = 0x022C,
}

impl LrFhssOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// Radio OpCodes used for LR-FHSS
#[derive(Clone, Copy)]
enum RadioOpCode {
    SetPktType = 0x020E,
    SetModulationParam = 0x020F,
    SetLrFhssSyncWord = 0x022D,
}

impl RadioOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// Packet type values
#[derive(Clone, Copy)]
enum PacketType {
    LrFhss = 0x04,
}

impl PacketType {
    fn value(self) -> u8 {
        self as u8
    }
}

// =============================================================================
// LR-FHSS Types and Constants
// =============================================================================

/// LR-FHSS modulation type
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LrFhssModulationType {
    Gmsk488 = 0x00,
}

impl LrFhssModulationType {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// LR-FHSS coding rate
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LrFhssCodingRate {
    Cr5_6 = 0x00,
    Cr2_3 = 0x01,
    Cr1_2 = 0x02,
    Cr1_3 = 0x03,
}

impl LrFhssCodingRate {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// LR-FHSS grid spacing
///
/// Note: Values match lr_fhss_v1_grid_t from SWDM001/SWDR001
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LrFhssGrid {
    /// 25.391 kHz grid (coarse)
    Grid25391Hz = 0x00,
    /// 3.906 kHz grid (fine)
    Grid3906Hz = 0x01,
}

impl LrFhssGrid {
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// LR-FHSS bandwidth
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum LrFhssBandwidth {
    Bw39063Hz = 0x00,
    Bw85938Hz = 0x01,
    Bw136719Hz = 0x02,
    Bw183594Hz = 0x03,
    Bw335938Hz = 0x04,
    Bw386719Hz = 0x05,
    Bw722656Hz = 0x06,
    Bw773438Hz = 0x07,
    Bw1523438Hz = 0x08,
    Bw1574219Hz = 0x09,
}

impl LrFhssBandwidth {
    pub fn value(self) -> u8 {
        self as u8
    }

    /// Get the number of hop sequences for this bandwidth and grid
    ///
    /// Values from SWDM001 lr_fhss_v1_base_types.h
    pub fn hop_sequence_count(self, grid: LrFhssGrid) -> u16 {
        match grid {
            LrFhssGrid::Grid25391Hz => match self {
                LrFhssBandwidth::Bw39063Hz => 1,
                LrFhssBandwidth::Bw85938Hz => 1,
                LrFhssBandwidth::Bw136719Hz => 1,
                LrFhssBandwidth::Bw183594Hz => 1,
                LrFhssBandwidth::Bw335938Hz => 44,
                LrFhssBandwidth::Bw386719Hz => 50,
                LrFhssBandwidth::Bw722656Hz => 88,
                LrFhssBandwidth::Bw773438Hz => 94,
                LrFhssBandwidth::Bw1523438Hz => 176,
                LrFhssBandwidth::Bw1574219Hz => 182,
            },
            LrFhssGrid::Grid3906Hz => match self {
                LrFhssBandwidth::Bw39063Hz => 1,
                LrFhssBandwidth::Bw85938Hz => 85,
                LrFhssBandwidth::Bw136719Hz => 170,
                LrFhssBandwidth::Bw183594Hz => 255,
                LrFhssBandwidth::Bw335938Hz => 340,
                LrFhssBandwidth::Bw386719Hz => 383,
                LrFhssBandwidth::Bw722656Hz => 639,
                LrFhssBandwidth::Bw773438Hz => 682,
                LrFhssBandwidth::Bw1523438Hz => 1192,
                LrFhssBandwidth::Bw1574219Hz => 1235,
            },
        }
    }
}

/// LR-FHSS sync word bytes
pub const LR_FHSS_SYNC_WORD_BYTES: usize = 4;

/// Default LR-FHSS sync word from SWDM001: { 0x2C, 0x0F, 0x79, 0x95 }
pub const LR_FHSS_DEFAULT_SYNC_WORD: [u8; LR_FHSS_SYNC_WORD_BYTES] = [0x2C, 0x0F, 0x79, 0x95];

/// LR-FHSS V1 parameters (matching lr_fhss_v1_params_t from SWDM001/SWDR001)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct LrFhssV1Params {
    /// 4-byte sync word (default: 0x2C, 0x0F, 0x79, 0x95)
    pub sync_word: [u8; LR_FHSS_SYNC_WORD_BYTES],
    /// Modulation type (GMSK 488 bps)
    pub modulation_type: LrFhssModulationType,
    /// Coding rate
    pub coding_rate: LrFhssCodingRate,
    /// Grid spacing
    pub grid: LrFhssGrid,
    /// Enable frequency hopping
    pub enable_hopping: bool,
    /// Bandwidth
    pub bandwidth: LrFhssBandwidth,
    /// Number of header blocks
    pub header_count: u8,
}

impl Default for LrFhssV1Params {
    fn default() -> Self {
        Self {
            sync_word: LR_FHSS_DEFAULT_SYNC_WORD,
            modulation_type: LrFhssModulationType::Gmsk488,
            coding_rate: LrFhssCodingRate::Cr5_6,
            grid: LrFhssGrid::Grid3906Hz,
            enable_hopping: true,
            bandwidth: LrFhssBandwidth::Bw136719Hz,
            header_count: 2,
        }
    }
}

/// LR-FHSS parameters (matching lr11xx_lr_fhss_params_t from SWDR001)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct LrFhssParams {
    pub lr_fhss_params: LrFhssV1Params,
    pub device_offset: i8,
}

impl Default for LrFhssParams {
    fn default() -> Self {
        Self {
            lr_fhss_params: LrFhssV1Params::default(),
            device_offset: 0,
        }
    }
}

/// Get the number of hop sequences for given LR-FHSS parameters
pub fn lr_fhss_get_hop_sequence_count(params: &LrFhssParams) -> u16 {
    params
        .lr_fhss_params
        .bandwidth
        .hop_sequence_count(params.lr_fhss_params.grid)
}

// =============================================================================
// LR-FHSS Extension Trait
// =============================================================================

/// Extension trait that adds LR-FHSS functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait LrFhssExt {
    /// Initialize LR-FHSS mode
    ///
    /// This sets the packet type to LR-FHSS and configures modulation parameters.
    /// Must be called before lr_fhss_build_frame().
    ///
    /// Reference: SWDR001 lr11xx_lr_fhss_init()
    async fn lr_fhss_init(&mut self) -> Result<(), RadioError>;

    /// Build and transmit an LR-FHSS frame
    ///
    /// This command configures the LR-FHSS parameters, writes the payload,
    /// and prepares the radio for transmission.
    ///
    /// Reference: SWDR001 lr11xx_lr_fhss_build_frame()
    ///
    /// # Arguments
    /// * `params` - LR-FHSS parameters
    /// * `hop_sequence_id` - Hop sequence identifier
    /// * `payload` - Payload data to transmit
    async fn lr_fhss_build_frame(
        &mut self,
        params: &LrFhssParams,
        hop_sequence_id: u16,
        payload: &[u8],
    ) -> Result<(), RadioError>;

    /// Set LR-FHSS sync word
    ///
    /// # Arguments
    /// * `sync_word` - 4-byte sync word
    async fn lr_fhss_set_sync_word(&mut self, sync_word: &[u8; 4]) -> Result<(), RadioError>;
}

// =============================================================================
// Implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> LrFhssExt for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn lr_fhss_init(&mut self) -> Result<(), RadioError> {
        // Step 1: Set packet type to LR-FHSS (0x04)
        let pkt_type_opcode = RadioOpCode::SetPktType.bytes();
        let pkt_type_cmd = [
            pkt_type_opcode[0],
            pkt_type_opcode[1],
            PacketType::LrFhss.value(),
        ];
        self.execute_command(&pkt_type_cmd).await?;

        // Step 2: Set LR-FHSS modulation parameters (bitrate 488 bps, BT=1 pulse shape)
        // Format: opcode[2] + bitrate[4] + pulse_shape[1]
        // Note: These are special encoded values from SWDR001, NOT the raw bps values!
        let mod_opcode = RadioOpCode::SetModulationParam.bytes();
        let bitrate: u32 = 0x8001E848; // LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS (encoded)
        let pulse_shape: u8 = 0x0B; // LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1
        let mod_cmd = [
            mod_opcode[0],
            mod_opcode[1],
            ((bitrate >> 24) & 0xFF) as u8,
            ((bitrate >> 16) & 0xFF) as u8,
            ((bitrate >> 8) & 0xFF) as u8,
            (bitrate & 0xFF) as u8,
            pulse_shape,
        ];
        self.execute_command(&mod_cmd).await
    }

    async fn lr_fhss_build_frame(
        &mut self,
        params: &LrFhssParams,
        hop_sequence_id: u16,
        payload: &[u8],
    ) -> Result<(), RadioError> {
        // Set LR-FHSS sync word from params (matching SWDR001 behavior)
        self.lr_fhss_set_sync_word(&params.lr_fhss_params.sync_word)
            .await?;

        // Build LR-FHSS frame command
        // Format per SWDR001: opcode[2] + header_count + cr + modulation_type + grid +
        //                     enable_hopping + bw + hop_seq_id[2] + device_offset
        // Total: 11 bytes command, then payload follows
        let opcode = LrFhssOpCode::BuildFrame.bytes();

        // Construct command buffer
        let lr_fhss_params = &params.lr_fhss_params;
        let enable_hopping: u8 = if lr_fhss_params.enable_hopping { 1 } else { 0 };

        let cmd = [
            opcode[0],
            opcode[1],
            lr_fhss_params.header_count,            // [2] header_count
            lr_fhss_params.coding_rate.value(),     // [3] cr
            lr_fhss_params.modulation_type.value(), // [4] modulation_type
            lr_fhss_params.grid.value(),            // [5] grid
            enable_hopping,                         // [6] enable_hopping
            lr_fhss_params.bandwidth.value(),       // [7] bw
            ((hop_sequence_id >> 8) & 0xFF) as u8,  // [8] hop_seq_id MSB
            (hop_sequence_id & 0xFF) as u8,         // [9] hop_seq_id LSB
            params.device_offset as u8,             // [10] device_offset
        ];

        // Write command with payload
        self.execute_command_with_payload(&cmd, payload).await
    }

    async fn lr_fhss_set_sync_word(&mut self, sync_word: &[u8; 4]) -> Result<(), RadioError> {
        let opcode = RadioOpCode::SetLrFhssSyncWord.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            sync_word[0],
            sync_word[1],
            sync_word[2],
            sync_word[3],
        ];
        self.execute_command(&cmd).await
    }
}
