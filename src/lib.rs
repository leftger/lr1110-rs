//! Extended LR1110 functionality for GNSS, WiFi, Crypto, and Ranging
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
//!
//! # Usage
//!
//! This crate provides extension traits that add methods to the `Lr1110` struct
//! from `lora-phy`. Import the traits you need:
//!
//! ```ignore
//! use lora_phy::lr1110::Lr1110;
//! use lr1110_rs::gnss::GnssExt;
//! use lr1110_rs::wifi::WifiExt;
//!
//! // Create your Lr1110 instance via lora-phy
//! let mut radio = Lr1110::new(spi, iv, config);
//!
//! // Now you can use GNSS methods
//! radio.gnss_scan(GnssSearchMode::HighEffort, 0x07, 0).await?;
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

// Re-export lora-phy types for convenience
pub use lora_phy::lr1110::{Lr1110, Config, Lr1110Variant};
pub use lora_phy::mod_params::RadioError;

#[cfg(feature = "gnss")]
pub mod gnss;

#[cfg(feature = "wifi")]
pub mod wifi;

#[cfg(feature = "crypto")]
pub mod crypto;

#[cfg(feature = "ranging")]
pub mod ranging;

// Re-export extension traits at crate root for convenience
#[cfg(feature = "gnss")]
pub use gnss::GnssExt;

#[cfg(feature = "wifi")]
pub use wifi::WifiExt;

#[cfg(feature = "crypto")]
pub use crypto::CryptoExt;

#[cfg(feature = "ranging")]
pub use ranging::RangingExt;
