//! Bootloader functions for the LR1110 firmware update
//!
//! This module provides bootloader-mode functionality for firmware updates
//! and device identification on the LR11xx family of transceivers.
//!
//! # Warning
//!
//! The bootloader functions should only be used for firmware updates.
//! Incorrect usage can brick the device.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::bootloader::BootloaderExt;
//!
//! // Read bootloader version
//! let version = radio.bootloader_get_version().await?;
//! println!("Bootloader FW: 0x{:04X}", version.fw);
//!
//! // Read chip identifiers
//! let chip_eui = radio.bootloader_read_chip_eui().await?;
//! let join_eui = radio.bootloader_read_join_eui().await?;
//! let pin = radio.bootloader_read_pin().await?;
//!
//! // Firmware update sequence:
//! // 1. Erase flash
//! radio.bootloader_erase_flash().await?;
//! // 2. Write encrypted firmware in chunks
//! for (offset, chunk) in firmware_chunks {
//!     radio.bootloader_write_flash_encrypted(offset, chunk).await?;
//! }
//! // 3. Reboot into application
//! radio.bootloader_reboot(false).await?;
//! ```

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// Bootloader OpCodes
// =============================================================================

/// Bootloader OpCodes (16-bit commands)
///
/// These opcodes are used when the chip is in bootloader mode (before flash
/// code execution or during firmware update).
#[derive(Clone, Copy)]
enum BootloaderOpCode {
    /// Get status registers (0x0100) - same as System GetStatus
    GetStatus = 0x0100,
    /// Get bootloader version (0x0101) - same as System GetVersion
    GetVersion = 0x0101,
    /// Erase entire flash memory (0x8000)
    EraseFlash = 0x8000,
    /// Write encrypted data to flash (0x8003)
    WriteFlashEncrypted = 0x8003,
    /// Reboot the chip (0x8005)
    Reboot = 0x8005,
    /// Read device PIN for cloud claiming (0x800B)
    GetPin = 0x800B,
    /// Read chip EUI (0x800C)
    ReadChipEui = 0x800C,
    /// Read join EUI (0x800D)
    ReadJoinEui = 0x800D,
}

impl BootloaderOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

// =============================================================================
// Bootloader Types and Constants
// =============================================================================

/// Length of bootloader version in bytes
pub const BOOTLOADER_VERSION_LENGTH: usize = 4;

/// Length of PIN in bytes
pub const BOOTLOADER_PIN_LENGTH: usize = 4;

/// Length of chip EUI in bytes
pub const BOOTLOADER_CHIP_EUI_LENGTH: usize = 8;

/// Length of join EUI in bytes
pub const BOOTLOADER_JOIN_EUI_LENGTH: usize = 8;

/// Maximum flash write block size in 32-bit words
pub const BOOTLOADER_FLASH_BLOCK_SIZE_WORDS: usize = 64;

/// Maximum flash write block size in bytes
pub const BOOTLOADER_FLASH_BLOCK_SIZE_BYTES: usize = BOOTLOADER_FLASH_BLOCK_SIZE_WORDS * 4;

/// Type alias for bootloader PIN (4 bytes)
pub type BootloaderPin = [u8; BOOTLOADER_PIN_LENGTH];

/// Type alias for chip EUI (8 bytes)
pub type BootloaderChipEui = [u8; BOOTLOADER_CHIP_EUI_LENGTH];

/// Type alias for join EUI (8 bytes)
pub type BootloaderJoinEui = [u8; BOOTLOADER_JOIN_EUI_LENGTH];

/// Bootloader version information
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BootloaderVersion {
    /// Hardware version
    pub hw: u8,
    /// Chip type (same encoding as system Version)
    pub chip_type: u8,
    /// Firmware version (bootloader version)
    pub fw: u16,
}

/// Bootloader command status
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum BootloaderCommandStatus {
    /// Command failed
    Fail = 0x00,
    /// Permission error
    Perr = 0x01,
    /// Command OK
    Ok = 0x02,
    /// Data available
    Data = 0x03,
}

impl From<u8> for BootloaderCommandStatus {
    fn from(value: u8) -> Self {
        match value & 0x07 {
            0x00 => BootloaderCommandStatus::Fail,
            0x01 => BootloaderCommandStatus::Perr,
            0x02 => BootloaderCommandStatus::Ok,
            0x03 => BootloaderCommandStatus::Data,
            _ => BootloaderCommandStatus::Fail,
        }
    }
}

/// Bootloader status register 1
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BootloaderStat1 {
    /// Command status
    pub command_status: u8,
    /// Interrupt is active
    pub is_interrupt_active: bool,
}

impl BootloaderStat1 {
    /// Parse from raw byte
    pub fn from_byte(byte: u8) -> Self {
        Self {
            is_interrupt_active: (byte & 0x01) != 0,
            command_status: byte >> 1,
        }
    }

    /// Get command status as enum
    pub fn status(&self) -> BootloaderCommandStatus {
        BootloaderCommandStatus::from(self.command_status)
    }
}

/// Bootloader status register 2
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BootloaderStat2 {
    /// Chip is running from flash (vs bootloader)
    pub is_running_from_flash: bool,
    /// Current chip mode
    pub chip_mode: u8,
    /// Reset status
    pub reset_status: u8,
}

impl BootloaderStat2 {
    /// Parse from raw byte
    pub fn from_byte(byte: u8) -> Self {
        Self {
            is_running_from_flash: (byte & 0x01) != 0,
            chip_mode: (byte & 0x0F) >> 1,
            reset_status: (byte & 0xF0) >> 4,
        }
    }
}

/// Complete bootloader status
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BootloaderStatus {
    /// Status register 1
    pub stat1: BootloaderStat1,
    /// Status register 2
    pub stat2: BootloaderStat2,
    /// IRQ status flags (32-bit mask)
    pub irq_status: u32,
}

// =============================================================================
// Bootloader Extension Trait
// =============================================================================

/// Extension trait that adds bootloader functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait BootloaderExt {
    /// Get bootloader status registers and IRQ flags
    ///
    /// This function reads the status by performing a direct SPI read.
    /// Unlike the GetStatus command, this does NOT clear the reset status.
    async fn bootloader_get_status(&mut self) -> Result<BootloaderStatus, RadioError>;

    /// Clear the reset status information
    ///
    /// This sends the GetStatus command which clears the reset status field.
    async fn bootloader_clear_reset_status(&mut self) -> Result<(), RadioError>;

    /// Get bootloader version information
    async fn bootloader_get_version(&mut self) -> Result<BootloaderVersion, RadioError>;

    /// Erase the entire flash memory
    ///
    /// This function MUST be called before writing new firmware to flash.
    ///
    /// # Warning
    /// This operation erases all flash content and cannot be undone.
    async fn bootloader_erase_flash(&mut self) -> Result<(), RadioError>;

    /// Write encrypted data to flash memory
    ///
    /// Writes a block of encrypted firmware data to flash.
    /// The data must be provided as 32-bit words (big-endian).
    ///
    /// # Arguments
    /// * `offset` - Byte offset from start of flash
    /// * `data` - Array of 32-bit words to write (max 64 words per call)
    ///
    /// # Constraints
    /// - Complete firmware image must be split into chunks of 64 words
    /// - Chunks must be sent in order, starting with offset = 0
    /// - Last chunk may be shorter than 64 words
    async fn bootloader_write_flash_encrypted(
        &mut self,
        offset: u32,
        data: &[u32],
    ) -> Result<(), RadioError>;

    /// Reboot the chip
    ///
    /// # Arguments
    /// * `stay_in_bootloader` - If true, stay in bootloader mode after reboot.
    ///   If false, execute flash code (requires valid flash content).
    async fn bootloader_reboot(&mut self, stay_in_bootloader: bool) -> Result<(), RadioError>;

    /// Read the device PIN for cloud service claiming
    async fn bootloader_read_pin(&mut self) -> Result<BootloaderPin, RadioError>;

    /// Read the chip EUI
    async fn bootloader_read_chip_eui(&mut self) -> Result<BootloaderChipEui, RadioError>;

    /// Read the join EUI
    async fn bootloader_read_join_eui(&mut self) -> Result<BootloaderJoinEui, RadioError>;
}

// =============================================================================
// Implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> BootloaderExt for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn bootloader_get_status(&mut self) -> Result<BootloaderStatus, RadioError> {
        // Wait for BUSY to go low
        self.wait_on_busy().await?;

        // Read status via GetStatus command
        let opcode = BootloaderOpCode::GetStatus.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 6];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        Ok(BootloaderStatus {
            stat1: BootloaderStat1::from_byte(rbuffer[0]),
            stat2: BootloaderStat2::from_byte(rbuffer[1]),
            irq_status: ((rbuffer[2] as u32) << 24)
                | ((rbuffer[3] as u32) << 16)
                | ((rbuffer[4] as u32) << 8)
                | (rbuffer[5] as u32),
        })
    }

    async fn bootloader_clear_reset_status(&mut self) -> Result<(), RadioError> {
        let opcode = BootloaderOpCode::GetStatus.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn bootloader_get_version(&mut self) -> Result<BootloaderVersion, RadioError> {
        let opcode = BootloaderOpCode::GetVersion.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; BOOTLOADER_VERSION_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;

        Ok(BootloaderVersion {
            hw: rbuffer[0],
            chip_type: rbuffer[1],
            fw: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
        })
    }

    async fn bootloader_erase_flash(&mut self) -> Result<(), RadioError> {
        let opcode = BootloaderOpCode::EraseFlash.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn bootloader_write_flash_encrypted(
        &mut self,
        offset: u32,
        data: &[u32],
    ) -> Result<(), RadioError> {
        if data.len() > BOOTLOADER_FLASH_BLOCK_SIZE_WORDS {
            return Err(RadioError::PayloadSizeMismatch(
                BOOTLOADER_FLASH_BLOCK_SIZE_WORDS,
                data.len(),
            ));
        }

        let opcode = BootloaderOpCode::WriteFlashEncrypted.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (offset >> 24) as u8,
            (offset >> 16) as u8,
            (offset >> 8) as u8,
            offset as u8,
        ];

        // Convert 32-bit words to bytes (big-endian)
        let mut payload = [0u8; BOOTLOADER_FLASH_BLOCK_SIZE_BYTES];
        for (i, word) in data.iter().enumerate() {
            let idx = i * 4;
            payload[idx] = (*word >> 24) as u8;
            payload[idx + 1] = (*word >> 16) as u8;
            payload[idx + 2] = (*word >> 8) as u8;
            payload[idx + 3] = *word as u8;
        }

        let payload_len = data.len() * 4;
        self.execute_command_with_payload(&cmd, &payload[..payload_len])
            .await
    }

    async fn bootloader_reboot(&mut self, stay_in_bootloader: bool) -> Result<(), RadioError> {
        let opcode = BootloaderOpCode::Reboot.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            if stay_in_bootloader { 0x03 } else { 0x00 },
        ];
        self.execute_command(&cmd).await
    }

    async fn bootloader_read_pin(&mut self) -> Result<BootloaderPin, RadioError> {
        let opcode = BootloaderOpCode::GetPin.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; BOOTLOADER_PIN_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer)
    }

    async fn bootloader_read_chip_eui(&mut self) -> Result<BootloaderChipEui, RadioError> {
        let opcode = BootloaderOpCode::ReadChipEui.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; BOOTLOADER_CHIP_EUI_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer)
    }

    async fn bootloader_read_join_eui(&mut self) -> Result<BootloaderJoinEui, RadioError> {
        let opcode = BootloaderOpCode::ReadJoinEui.bytes();
        let cmd = [opcode[0], opcode[1]];

        let mut rbuffer = [0u8; BOOTLOADER_JOIN_EUI_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer)
            .await?;
        Ok(rbuffer)
    }
}
