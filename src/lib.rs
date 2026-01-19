//! Extended LR1110 functionality beyond core LoRa
//!
//! This crate provides additional features for the Semtech LR1110 transceiver
//! that are beyond the core LoRa functionality provided by `lora-phy`.
//!
//! # Features
//!
//! - **GNSS**: GPS and BeiDou satellite scanning for geolocation
//! - **WiFi**: Passive WiFi AP scanning for indoor positioning
//! - **Almanac**: Age tracking and update management for GNSS almanac data
//! - **IRQ Management**: Two-stage priority IRQ handling for GNSS/WiFi scans
//! - **Radio Planner**: Coordinates concurrent radio operations
//! - **Crypto**: Hardware AES encryption, CMAC, and key management
//! - **Ranging**: RTToF (Round-Trip Time of Flight) distance measurement
//! - **GFSK**: Gaussian Frequency Shift Keying modulation
//! - **LR-FHSS**: Long Range Frequency Hopping Spread Spectrum
//! - **System**: Temperature, battery voltage, RNG, device identifiers
//! - **Bootloader**: Firmware update and chip identification
//! - **RegMem**: Low-level register and memory access
//!
//! # Quick Start
//!
//! ## Basic GNSS Scanning
//!
//! ```ignore
//! use lora_phy::lr1110::Lr1110;
//! use lr1110_rs::gnss::{GnssExt, GnssSearchMode};
//!
//! let mut radio = Lr1110::new(spi, iv, config);
//!
//! // Simple GNSS scan
//! radio.gnss_scan(GnssSearchMode::MidEffort, 0x00, 0).await?;
//! embassy_time::Timer::after_secs(10).await;
//! let num_sats = radio.gnss_get_nb_satellites().await?;
//! ```
//!
//! ## Advanced GNSS with IRQ Management
//!
//! ```ignore
//! use lr1110_rs::gnss_scan_manager::{GnssScanManager, ScanConfig, ScanMode};
//! use lr1110_rs::gnss::{GNSS_GPS_MASK, GNSS_BEIDOU_MASK};
//!
//! let mut manager = GnssScanManager::new();
//! manager.set_prescan_callback(|| { sidewalk_suspend(); });
//! manager.set_postscan_callback(|| { sidewalk_resume(); });
//!
//! let config = ScanConfig {
//!     constellation_mask: GNSS_GPS_MASK | GNSS_BEIDOU_MASK,
//!     mode: ScanMode::Mobile { scan_group_size: 2, scan_group_delay_s: 0 },
//!     ..Default::default()
//! };
//!
//! let result = manager.scan(&mut radio, &config).await?;
//! ```
//!
//! ## Almanac Management
//!
//! ```ignore
//! use lr1110_rs::almanac::AlmanacManager;
//!
//! let mut almanac_mgr = AlmanacManager::new();
//!
//! // Check status with age tracking
//! let status = almanac_mgr.check_status(&mut radio, true).await?;
//! info!("GPS: {}% complete", status.gps.completion_percentage());
//!
//! // Update if needed
//! if status.needs_update() {
//!     let strategy = almanac_mgr.determine_update_strategy(&mut radio).await?;
//!     almanac_mgr.start_update(&mut radio, strategy, GnssSearchMode::HighEffort).await?;
//! }
//! ```
//!
//! ## WiFi Scanning
//!
//! ```ignore
//! use lr1110_rs::wifi::{WifiExt, WifiSignalTypeScan, WifiScanMode, WIFI_ALL_CHANNELS_MASK};
//!
//! radio.wifi_scan(
//!     WifiSignalTypeScan::TypeBGN,
//!     WIFI_ALL_CHANNELS_MASK,
//!     WifiScanMode::Beacon,
//!     32, 1, 90, false
//! ).await?;
//! ```
//!
//! # Feature Flags
//!
//! ```toml
//! [dependencies]
//! lr1110-rs = {
//!     version = "0.1",
//!     features = [
//!         "gnss",        # GNSS scanning
//!         "wifi",        # WiFi scanning
//!         "system",      # System functions
//!         "irq-manager", # Advanced IRQ management (requires embassy-time)
//!         "defmt-03"     # Logging
//!     ]
//! }
//! ```
//!
//! # Examples
//!
//! See `examples/stm32wba/src/bin/` for complete examples:
//! - `lr1110_gnss_scan.rs` - Basic GNSS scanning (simple timeout)
//! - `lr1110_gnss_scan_with_irq.rs` - Advanced GNSS with IRQ management
//! - `lr1110_wifi_scan.rs` - Basic WiFi scanning
//! - `lr1110_wifi_scan_with_irq.rs` - Advanced WiFi with IRQ management
//! - `lr1110_almanac_manager.rs` - Almanac age tracking and updates
//!
//! # Source
//!
//! IRQ management and almanac features ported from:
//! - **STM32-Sidewalk-SDK**: Amazon/STMicroelectronics implementation
//! - **LoRa Basics Modem**: Semtech geolocation services
//!
//! # Platform Support
//!
//! - **Full support** (ARM Cortex-M3/M4/M7): STM32WBA, STM32F4, nRF52, etc.
//! - **Limited support** (ARM Cortex-M0/M0+): RP2040, STM32L0 (single IRQ priority)
//! - **Basic support** (Other): RISC-V, Xtensa (timestamp capture only)

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

#[cfg(all(feature = "gnss", feature = "irq-manager"))]
pub mod gnss_irq;

#[cfg(all(feature = "gnss", feature = "irq-manager"))]
pub mod gnss_scan_manager;

#[cfg(feature = "gnss")]
pub mod almanac;

#[cfg(feature = "irq-manager")]
pub mod radio_planner;

#[cfg(feature = "wifi")]
pub mod wifi;

#[cfg(all(feature = "wifi", feature = "irq-manager"))]
pub mod wifi_irq;

#[cfg(all(feature = "wifi", feature = "irq-manager"))]
pub mod wifi_scan_manager;

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

#[cfg(all(feature = "gnss", feature = "irq-manager"))]
pub use gnss_scan_manager::GnssScanManager;

#[cfg(feature = "gnss")]
pub use almanac::AlmanacManager;

#[cfg(feature = "wifi")]
pub use wifi::WifiExt;

#[cfg(all(feature = "wifi", feature = "irq-manager"))]
pub use wifi_scan_manager::WifiScanManager;

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
    // Helper functions
    convert_temp_to_celsius,
    convert_vbat_to_volts,
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
