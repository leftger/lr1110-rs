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
