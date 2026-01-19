//! Extended LR1110 functionality beyond core LoRa
//!
//! This crate provides additional features for the Semtech LR1110 transceiver
//! that are beyond the core LoRa functionality provided by `lora-phy`.
//!
//! # Features
//!
//! - **GNSS**: GPS and BeiDou satellite scanning for geolocation
//! - **WiFi**: Passive WiFi AP scanning for indoor positioning
//! - **Crypto**: Hardware AES encryption, CMAC, and key management
//! - **Ranging**: RTToF (Round-Trip Time of Flight) distance measurement
//! - **GFSK**: Gaussian Frequency Shift Keying modulation
//! - **LR-FHSS**: Long Range Frequency Hopping Spread Spectrum
//! - **System**: Temperature, battery voltage, RNG, device identifiers
//! - **Bootloader**: Firmware update and chip identification
//! - **RegMem**: Low-level register and memory access
//!
//! # Usage
//!
//! This crate provides extension traits that add methods to the `Lr1110` struct
//! from `lora-phy`. Import the traits you need:
//!
//! ```ignore
//! use lora_phy::lr1110::Lr1110;
//! use lr1110_rs::gnss::GnssExt;
//! use lr1110_rs::gfsk::GfskExt;
//! use lr1110_rs::system::SystemExt;
//!
//! // Create your Lr1110 instance via lora-phy
//! let mut radio = Lr1110::new(spi, iv, config);
//!
//! // Now you can use extension methods
//! radio.gnss_scan(GnssSearchMode::HighEffort, 0x07, 0).await?;
//!
//! // Read temperature
//! let temp = radio.get_temp().await?;
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

// Re-export lora-phy types for convenience
pub use lora_phy::lr1110::{Config, Lr1110, Lr1110Variant};
pub use lora_phy::mod_params::RadioError;
pub use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// Interface Variant for boards with BUSY pin connected
// =============================================================================

/// InterfaceVariant implementation for LR1110 with BUSY pin support.
///
/// This module provides a generic InterfaceVariant that properly waits for the
/// BUSY pin, which is essential for correct LR1110 operation on boards where
/// the BUSY pin is connected.
///
/// Use this on any platform (STM32, NRF52, RP2040, etc.) where you have the
/// LR1110 BUSY pin wired to a GPIO.
pub mod iv;

// =============================================================================
// Feature-gated modules
// =============================================================================

/// General radio control functions (packet type, frequency, TX/RX, buffers, etc.)
pub mod radio;

#[cfg(feature = "gnss")]
pub mod gnss;

#[cfg(feature = "wifi")]
pub mod wifi;

#[cfg(feature = "crypto")]
pub mod crypto;

#[cfg(feature = "ranging")]
pub mod ranging;

#[cfg(feature = "gfsk")]
pub mod gfsk;

#[cfg(feature = "lr_fhss")]
pub mod lr_fhss;

#[cfg(feature = "system")]
pub mod system;

#[cfg(feature = "bootloader")]
pub mod bootloader;

#[cfg(feature = "regmem")]
pub mod regmem;

// =============================================================================
// Re-export extension traits at crate root for convenience
// =============================================================================

pub use radio::RadioControlExt;

#[cfg(feature = "gnss")]
pub use gnss::GnssExt;

#[cfg(feature = "wifi")]
pub use wifi::WifiExt;

#[cfg(feature = "crypto")]
pub use crypto::CryptoExt;

#[cfg(feature = "ranging")]
pub use ranging::RangingExt;

#[cfg(feature = "gfsk")]
pub use gfsk::GfskExt;

#[cfg(feature = "lr_fhss")]
pub use lr_fhss::LrFhssExt;

#[cfg(feature = "system")]
pub use system::SystemExt;

#[cfg(feature = "bootloader")]
pub use bootloader::BootloaderExt;

#[cfg(feature = "regmem")]
pub use regmem::RegMemExt;

// =============================================================================
// Re-export types from modules
// =============================================================================

// Radio module types
pub use radio::{
    convert_nb_symb_to_mant_exp,
    // Timing utilities
    convert_time_in_ms_to_rtc_step,
    get_gfsk_time_on_air_in_ms,
    get_gfsk_time_on_air_numerator,
    get_lora_bw_in_hz,
    get_lora_time_on_air_in_ms,
    get_lora_time_on_air_numerator,
    // BPSK
    BpskModParams,
    BpskPktParams,
    BpskPulseShape,
    // CAD
    CadExitMode,
    CadParams,
    FallbackMode,
    // GFSK
    GfskAddressFiltering,
    GfskBandwidth,
    GfskCrcType,
    GfskDcFree,
    GfskHeaderType,
    GfskModParams,
    GfskPktParams,
    // Packet Status
    GfskPktStatus,
    GfskPreambleDetector,
    GfskPulseShape,
    // Statistics
    GfskStats,
    IntermediaryMode,
    // LNA Configuration
    LnaMode,
    // LoRa
    LoRaNetworkType,
    LoRaPktStatus,
    LoRaStats,
    // PA and TX
    PaConfig,
    PaRegSupply,
    PaSelection,
    // Packet types and modes
    PacketType,
    RampTime,
    // Calibration
    RssiCalibrationTable,
    RxBufferStatus,
};

// System module types
#[cfg(feature = "system")]
pub use system::{
    // Core types
    CalibrationParam,
    // Status types
    ChipMode,
    ChipType,
    CommandStatus,
    IrqMask,
    ResetStatus,
    RfSwitchConfig,
    SleepConfig,
    StandbyConfig,
    Stat1,
    Stat2,
    SystemErrors,
    TcxoVoltage,
    Version,
    // Calibration constants
    CALIB_ADC_MASK,
    CALIB_ALL,
    CALIB_HF_RC_MASK,
    CALIB_IMG_MASK,
    CALIB_LF_RC_MASK,
    CALIB_PLL_MASK,
    CALIB_PLL_TX_MASK,
    // IRQ constants
    IRQ_ALL,
    IRQ_CAD_DETECTED,
    IRQ_CAD_DONE,
    IRQ_CMD_ERROR,
    IRQ_CRC_ERROR,
    IRQ_EOL,
    IRQ_ERROR,
    IRQ_FSK_ADDR_ERROR,
    IRQ_FSK_LEN_ERROR,
    IRQ_GNSS_SCAN_DONE,
    IRQ_HEADER_ERROR,
    IRQ_LORA_RX_TIMESTAMP,
    IRQ_LR_FHSS_INTRA_PKT_HOP,
    IRQ_NONE,
    IRQ_PREAMBLE_DETECTED,
    IRQ_RANGING_EXCH_VALID,
    IRQ_RANGING_REQ_DISCARDED,
    IRQ_RANGING_REQ_VALID,
    IRQ_RANGING_RESP_DONE,
    IRQ_RANGING_TIMEOUT,
    IRQ_RX_DONE,
    IRQ_SYNC_WORD_HEADER_VALID,
    IRQ_TIMEOUT,
    IRQ_TX_DONE,
    IRQ_WIFI_SCAN_DONE,
    // Constants
    LR11XX_SYSTEM_JOIN_EUI_LENGTH,
    LR11XX_SYSTEM_PIN_LENGTH,
    LR11XX_SYSTEM_UID_LENGTH,
    // RF switch constants
    RFSW0_HIGH,
    RFSW1_HIGH,
    RFSW2_HIGH,
    RFSW3_HIGH,
    RFSW4_HIGH,
};
