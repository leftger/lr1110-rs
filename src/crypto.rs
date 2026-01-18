//! Crypto Engine functionality for the LR1110
//!
//! This module provides hardware-accelerated cryptographic operations including:
//! - AES encryption/decryption
//! - AES-CMAC (Message Integrity Code) computation and verification
//! - Key management and derivation
//! - LoRaWAN Join Accept message processing
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::crypto::{CryptoExt, CryptoKeyId, CryptoElement};
//!
//! // Select the crypto engine
//! radio.crypto_select(CryptoElement::CryptoEngine).await?;
//!
//! // Set a key in slot GP0
//! let key = [0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
//!            0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C];
//! let status = radio.crypto_set_key(CryptoKeyId::Gp0, &key).await?;
//!
//! // Encrypt some data
//! let plaintext = [0u8; 16];
//! let mut ciphertext = [0u8; 16];
//! radio.crypto_aes_encrypt(CryptoKeyId::Gp0, &plaintext, &mut ciphertext).await?;
//! ```

use lora_phy::mod_params::RadioError;

// =============================================================================
// Crypto Engine Types and Constants (from SWDR001 lr11xx_crypto_engine.c/h)
// =============================================================================

/// Crypto Engine OpCodes (16-bit commands)
#[derive(Clone, Copy, PartialEq)]
pub enum CryptoOpCode {
    /// Select crypto element (internal or secure element) (0x0500)
    Select = 0x0500,
    /// Set a key in the crypto engine (0x0502)
    SetKey = 0x0502,
    /// Derive a key from another key (0x0503)
    DeriveKey = 0x0503,
    /// Process LoRaWAN Join Accept message (0x0504)
    ProcessJoinAccept = 0x0504,
    /// Compute AES-CMAC (0x0505)
    ComputeAesCmac = 0x0505,
    /// Verify AES-CMAC (0x0506)
    VerifyAesCmac = 0x0506,
    /// AES encrypt (legacy, variant 01) (0x0507)
    AesEncrypt01 = 0x0507,
    /// AES encrypt (0x0508)
    AesEncrypt = 0x0508,
    /// AES decrypt (0x0509)
    AesDecrypt = 0x0509,
    /// Store crypto data to flash (0x050A)
    StoreToFlash = 0x050A,
    /// Restore crypto data from flash (0x050B)
    RestoreFromFlash = 0x050B,
    /// Set a crypto parameter (0x050D)
    SetParameter = 0x050D,
    /// Get a crypto parameter (0x050E)
    GetParameter = 0x050E,
    /// Check encrypted firmware image (0x050F)
    CheckEncryptedFwImage = 0x050F,
    /// Get result of encrypted firmware image check (0x0510)
    GetCheckEncryptedFwImageResult = 0x0510,
}

impl CryptoOpCode {
    /// Convert opcode to bytes for SPI command
    pub fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

/// Length of MIC (Message Integrity Code) in bytes
pub const CRYPTO_MIC_LENGTH: usize = 4;

/// Length of AES-CMAC in bytes
pub const CRYPTO_AES_CMAC_LENGTH: usize = 16;

/// Maximum length of data to encrypt/decrypt in bytes
pub const CRYPTO_DATA_MAX_LENGTH: usize = 256;

/// Length of AES key in bytes
pub const CRYPTO_KEY_LENGTH: usize = 16;

/// Length of nonce in bytes
pub const CRYPTO_NONCE_LENGTH: usize = 16;

/// Length of crypto parameter in bytes
pub const CRYPTO_PARAMETER_LENGTH: usize = 4;

/// Crypto element selection
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CryptoElement {
    /// Internal crypto engine (default)
    CryptoEngine = 0x00,
    /// External secure element
    SecureElement = 0x01,
}

impl CryptoElement {
    /// Get the value for SPI command
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// Status returned by crypto operations
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CryptoStatus {
    /// Operation successful
    Success = 0x00,
    /// AES-CMAC invalid or comparison failed
    ErrorFailCmac = 0x01,
    /// Invalid key ID (source or destination)
    ErrorInvalidKeyId = 0x03,
    /// Invalid data buffer size
    ErrorBufferSize = 0x05,
    /// Other error
    Error = 0x06,
}

impl From<u8> for CryptoStatus {
    fn from(value: u8) -> Self {
        match value {
            0x00 => CryptoStatus::Success,
            0x01 => CryptoStatus::ErrorFailCmac,
            0x03 => CryptoStatus::ErrorInvalidKeyId,
            0x05 => CryptoStatus::ErrorBufferSize,
            _ => CryptoStatus::Error,
        }
    }
}

/// LoRaWAN version for crypto operations
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CryptoLorawanVersion {
    /// LoRaWAN 1.0.x
    V1_0x = 0x00,
    /// LoRaWAN 1.1.x
    V1_1x = 0x01,
}

impl CryptoLorawanVersion {
    /// Get the value for SPI command
    pub fn value(self) -> u8 {
        self as u8
    }

    /// Get header length for this LoRaWAN version
    pub fn header_length(self) -> usize {
        match self {
            CryptoLorawanVersion::V1_0x => 1,
            CryptoLorawanVersion::V1_1x => 12,
        }
    }
}

/// Crypto key slot identifiers
///
/// The LR1110 has dedicated key slots for LoRaWAN operations
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum CryptoKeyId {
    /// Mother key (root key for derivation)
    MotherKey = 1,
    /// Network key (NwkKey)
    NwkKey = 2,
    /// Application key (AppKey)
    AppKey = 3,
    /// Join server encryption key (JSEncKey)
    JSEncKey = 4,
    /// Join server integrity key (JSIntKey)
    JSIntKey = 5,
    /// General purpose key encryption key 0
    GpKeKey0 = 6,
    /// General purpose key encryption key 1
    GpKeKey1 = 7,
    /// General purpose key encryption key 2
    GpKeKey2 = 8,
    /// General purpose key encryption key 3
    GpKeKey3 = 9,
    /// General purpose key encryption key 4
    GpKeKey4 = 10,
    /// General purpose key encryption key 5
    GpKeKey5 = 11,
    /// Application session key (AppSKey)
    AppSKey = 12,
    /// Forwarding network session integrity key (FNwkSIntKey)
    FNwkSIntKey = 13,
    /// Serving network session integrity key (SNwkSIntKey)
    SNwkSIntKey = 14,
    /// Network session encryption key (NwkSEncKey)
    NwkSEncKey = 15,
    /// Reserved 0
    Rfu0 = 16,
    /// Reserved 1
    Rfu1 = 17,
    /// Multicast application session key 0
    McAppSKey0 = 18,
    /// Multicast application session key 1
    McAppSKey1 = 19,
    /// Multicast application session key 2
    McAppSKey2 = 20,
    /// Multicast application session key 3
    McAppSKey3 = 21,
    /// Multicast network session key 0
    McNwkSKey0 = 22,
    /// Multicast network session key 1
    McNwkSKey1 = 23,
    /// Multicast network session key 2
    McNwkSKey2 = 24,
    /// Multicast network session key 3
    McNwkSKey3 = 25,
    /// General purpose key 0
    Gp0 = 26,
    /// General purpose key 1
    Gp1 = 27,
}

impl CryptoKeyId {
    /// Get the key ID value
    pub fn value(self) -> u8 {
        self as u8
    }
}

/// Type alias for crypto key (16 bytes)
pub type CryptoKey = [u8; CRYPTO_KEY_LENGTH];

/// Type alias for crypto nonce (16 bytes)
pub type CryptoNonce = [u8; CRYPTO_NONCE_LENGTH];

/// Type alias for MIC (4 bytes)
pub type CryptoMic = [u8; CRYPTO_MIC_LENGTH];

/// Type alias for crypto parameter (4 bytes)
pub type CryptoParam = [u8; CRYPTO_PARAMETER_LENGTH];

// =============================================================================
// Crypto Extension Trait
// =============================================================================

/// Extension trait that adds Crypto Engine functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait CryptoExt {
    /// Select the crypto element to use for subsequent operations
    async fn crypto_select(&mut self, element: CryptoElement) -> Result<(), RadioError>;

    /// Set a key in the specified key slot
    async fn crypto_set_key(&mut self, key_id: CryptoKeyId, key: &CryptoKey) -> Result<CryptoStatus, RadioError>;

    /// Derive a new key from an existing key using a nonce
    async fn crypto_derive_key(
        &mut self,
        src_key_id: CryptoKeyId,
        dest_key_id: CryptoKeyId,
        nonce: &CryptoNonce,
    ) -> Result<CryptoStatus, RadioError>;

    /// Process a LoRaWAN Join Accept message
    async fn crypto_process_join_accept(
        &mut self,
        dec_key_id: CryptoKeyId,
        ver_key_id: CryptoKeyId,
        lorawan_version: CryptoLorawanVersion,
        header: &[u8],
        data_in: &[u8],
        data_out: &mut [u8],
    ) -> Result<CryptoStatus, RadioError>;

    /// Compute AES-CMAC (Message Integrity Code) over data
    async fn crypto_compute_aes_cmac(
        &mut self,
        key_id: CryptoKeyId,
        data: &[u8],
    ) -> Result<(CryptoStatus, CryptoMic), RadioError>;

    /// Verify AES-CMAC (Message Integrity Code) over data
    async fn crypto_verify_aes_cmac(
        &mut self,
        key_id: CryptoKeyId,
        data: &[u8],
        mic: &CryptoMic,
    ) -> Result<CryptoStatus, RadioError>;

    /// AES encrypt data (legacy variant 01)
    async fn crypto_aes_encrypt_01(
        &mut self,
        key_id: CryptoKeyId,
        data: &[u8],
        result: &mut [u8],
    ) -> Result<CryptoStatus, RadioError>;

    /// AES encrypt data
    async fn crypto_aes_encrypt(
        &mut self,
        key_id: CryptoKeyId,
        data: &[u8],
        result: &mut [u8],
    ) -> Result<CryptoStatus, RadioError>;

    /// AES decrypt data
    async fn crypto_aes_decrypt(
        &mut self,
        key_id: CryptoKeyId,
        data: &[u8],
        result: &mut [u8],
    ) -> Result<CryptoStatus, RadioError>;

    /// Store crypto keys and data to flash
    async fn crypto_store_to_flash(&mut self) -> Result<CryptoStatus, RadioError>;

    /// Restore crypto keys and data from flash
    async fn crypto_restore_from_flash(&mut self) -> Result<CryptoStatus, RadioError>;

    /// Set a crypto parameter
    async fn crypto_set_parameter(
        &mut self,
        param_id: u8,
        parameter: &CryptoParam,
    ) -> Result<CryptoStatus, RadioError>;

    /// Get a crypto parameter
    async fn crypto_get_parameter(&mut self, param_id: u8) -> Result<(CryptoStatus, CryptoParam), RadioError>;

    /// Check a portion of an encrypted firmware image
    async fn crypto_check_encrypted_fw_image(&mut self, offset: u32, data: &[u32]) -> Result<(), RadioError>;

    /// Get the result of encrypted firmware image check
    async fn crypto_get_check_encrypted_fw_image_result(&mut self) -> Result<bool, RadioError>;
}

// NOTE: Implementation requires lora-phy to expose low-level SPI interface.
// This will be implemented once lora-phy adds the Lr1110Interface trait.
//
// For now, users can use the types defined above with their own implementation.
