//! System diagnostic and utility functions for the LR1110
//!
//! This module provides access to system-level features of the LR11xx family:
//! - Temperature sensor
//! - Battery voltage measurement
//! - Hardware random number generator
//! - Device identifiers (UID, Join EUI)
//! - Error status
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::system::SystemExt;
//!
//! // Read temperature
//! let temp_raw = radio.get_temp().await?;
//!
//! // Read battery voltage
//! let vbat_raw = radio.get_vbat().await?;
//!
//! // Generate random number
//! let random = radio.get_random_number().await?;
//!
//! // Read unique device ID
//! let uid = radio.read_uid().await?;
//! ```

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// System OpCodes
// =============================================================================

/// System OpCodes (16-bit commands for LR1110)
#[derive(Clone, Copy)]
#[allow(dead_code)]
enum SystemOpCode {
    GetErrors = 0x010D,
    ClearErrors = 0x010E,
    GetVbat = 0x0119,
    GetTemp = 0x011A,
    GetRandom = 0x0120,
    ReadUid = 0x0125,
    ReadJoinEui = 0x0126,
}

impl SystemOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

// =============================================================================
// System Types and Constants
// =============================================================================

/// Length of the LR11XX Unique Identifier in bytes
pub const LR11XX_SYSTEM_UID_LENGTH: usize = 8;

/// Length of the LR11XX Join EUI in bytes
pub const LR11XX_SYSTEM_JOIN_EUI_LENGTH: usize = 8;

/// System error flags bitmask
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct SystemErrors {
    /// Raw error flags
    pub raw: u16,
}

impl SystemErrors {
    /// Check if any error is present
    pub fn has_errors(&self) -> bool {
        self.raw != 0
    }

    /// LF RC calibration error
    pub fn lf_rc_calib_error(&self) -> bool {
        self.raw & 0x01 != 0
    }

    /// HF RC calibration error
    pub fn hf_rc_calib_error(&self) -> bool {
        self.raw & 0x02 != 0
    }

    /// ADC calibration error
    pub fn adc_calib_error(&self) -> bool {
        self.raw & 0x04 != 0
    }

    /// PLL calibration error
    pub fn pll_calib_error(&self) -> bool {
        self.raw & 0x08 != 0
    }

    /// Image calibration error
    pub fn img_calib_error(&self) -> bool {
        self.raw & 0x10 != 0
    }

    /// HF XOSC start error
    pub fn hf_xosc_start_error(&self) -> bool {
        self.raw & 0x20 != 0
    }

    /// LF XOSC start error
    pub fn lf_xosc_start_error(&self) -> bool {
        self.raw & 0x40 != 0
    }

    /// PLL lock error
    pub fn pll_lock_error(&self) -> bool {
        self.raw & 0x80 != 0
    }
}

impl From<u16> for SystemErrors {
    fn from(raw: u16) -> Self {
        Self { raw }
    }
}

// =============================================================================
// System Extension Trait
// =============================================================================

/// Extension trait that adds system diagnostic functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait SystemExt {
    /// Get the chip temperature in degrees Celsius
    ///
    /// Returns the raw temperature value from the internal sensor.
    /// Temperature in Celsius = (raw_value - 273.15) approximately.
    async fn get_temp(&mut self) -> Result<u16, RadioError>;

    /// Get the battery voltage
    ///
    /// Returns a raw ADC value representing battery voltage.
    /// Actual voltage depends on board configuration.
    async fn get_vbat(&mut self) -> Result<u8, RadioError>;

    /// Get a 32-bit random number from the hardware RNG
    ///
    /// The radio must be in receive mode for best entropy.
    async fn get_random_number(&mut self) -> Result<u32, RadioError>;

    /// Read the unique device identifier (8 bytes)
    async fn read_uid(&mut self) -> Result<[u8; LR11XX_SYSTEM_UID_LENGTH], RadioError>;

    /// Read the Join EUI (8 bytes) - for LoRaWAN
    async fn read_join_eui(&mut self) -> Result<[u8; LR11XX_SYSTEM_JOIN_EUI_LENGTH], RadioError>;

    /// Wake the radio up from sleep mode
    ///
    /// Sends a wakeup signal to exit sleep state and return to standby.
    /// This is typically handled at the HAL level by toggling the NSS pin.
    async fn system_wakeup(&mut self) -> Result<(), RadioError>;

    /// Abort a currently executing blocking command
    ///
    /// Sends an abort signal to terminate any ongoing blocking operation.
    /// Useful for canceling long-running scans or ranging operations.
    async fn system_abort_blocking_cmd(&mut self) -> Result<(), RadioError>;
}

// =============================================================================
// Implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> SystemExt for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn get_temp(&mut self) -> Result<u16, RadioError> {
        let opcode = SystemOpCode::GetTemp.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16))
    }

    async fn get_vbat(&mut self) -> Result<u8, RadioError> {
        let opcode = SystemOpCode::GetVbat.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 1];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(rbuffer[0])
    }

    async fn get_random_number(&mut self) -> Result<u32, RadioError> {
        let opcode = SystemOpCode::GetRandom.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 4];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(((rbuffer[0] as u32) << 24)
            | ((rbuffer[1] as u32) << 16)
            | ((rbuffer[2] as u32) << 8)
            | (rbuffer[3] as u32))
    }

    async fn read_uid(&mut self) -> Result<[u8; LR11XX_SYSTEM_UID_LENGTH], RadioError> {
        let opcode = SystemOpCode::ReadUid.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; LR11XX_SYSTEM_UID_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(rbuffer)
    }

    async fn read_join_eui(&mut self) -> Result<[u8; LR11XX_SYSTEM_JOIN_EUI_LENGTH], RadioError> {
        let opcode = SystemOpCode::ReadJoinEui.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; LR11XX_SYSTEM_JOIN_EUI_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(rbuffer)
    }

    async fn system_wakeup(&mut self) -> Result<(), RadioError> {
        // Wakeup is typically handled at the HAL level by toggling NSS
        // The lora-phy layer should handle this via the InterfaceVariant
        // For now, we return Ok as the actual wakeup mechanism is HAL-dependent
        // TODO: Verify if lora-phy has a specific wakeup method or if this needs
        // special NSS toggling sequence as described in SWDR001 lr11xx_hal.c
        Ok(())
    }

    async fn system_abort_blocking_cmd(&mut self) -> Result<(), RadioError> {
        // Abort command requires special HAL handling
        // According to SWDR001, this may require NSS toggling or a specific command sequence
        // The actual implementation depends on the HAL layer capabilities
        // TODO: Consult SWDR001 lr11xx_hal.c for the actual abort sequence
        // and implement proper NSS toggling if needed
        Ok(())
    }
}
