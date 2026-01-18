//! Register and memory access functions for the LR1110
//!
//! This module provides low-level register and memory access for the LR11xx
//! family of transceivers. Use with caution as incorrect register writes
//! can cause unexpected behavior.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::regmem::RegMemExt;
//!
//! // Read 32-bit registers
//! let mut values = [0u32; 4];
//! radio.regmem_read_regmem32(0x00F20000, &mut values).await?;
//!
//! // Write 32-bit registers
//! radio.regmem_write_regmem32(0x00F20000, &[0x12345678]).await?;
//!
//! // Read/write with mask (read-modify-write)
//! radio.regmem_write_regmem32_mask(0x00F30054, 1 << 30, 0).await?;
//! ```

use embedded_hal_async::spi::SpiDevice;
use lora_phy::lr1110::variant::Lr1110Variant;
use lora_phy::lr1110::Lr1110;
use lora_phy::mod_params::RadioError;
use lora_phy::mod_traits::InterfaceVariant;

// =============================================================================
// RegMem OpCodes
// =============================================================================

/// Register/Memory OpCodes (from SWDR001 lr11xx_regmem.c)
#[derive(Clone, Copy)]
enum RegMemOpCode {
    /// Write 32-bit words to register/memory (0x0105)
    WriteRegMem32 = 0x0105,
    /// Read 32-bit words from register/memory (0x0106)
    ReadRegMem32 = 0x0106,
    /// Write bytes to memory (0x0107)
    WriteMem8 = 0x0107,
    /// Read bytes from memory (0x0108)
    ReadMem8 = 0x0108,
    /// Write bytes to TX buffer (0x0109)
    WriteBuffer8 = 0x0109,
    /// Read bytes from RX buffer (0x010A)
    ReadBuffer8 = 0x010A,
    /// Clear RX buffer (0x010B)
    ClearRxBuffer = 0x010B,
    /// Write with mask (read-modify-write) (0x010C)
    WriteRegMem32Mask = 0x010C,
}

impl RegMemOpCode {
    fn bytes(self) -> [u8; 2] {
        let val = self as u16;
        [(val >> 8) as u8, (val & 0xFF) as u8]
    }
}

// =============================================================================
// RegMem Types and Constants
// =============================================================================

/// Maximum number of 32-bit words for single read/write operation
pub const REGMEM_MAX_READ_WRITE_WORDS: usize = 64;

/// Maximum buffer size in bytes
pub const REGMEM_BUFFER_SIZE_MAX: usize = 256;

/// Register address for High ACP workaround (from SWDR001)
pub const HIGH_ACP_WORKAROUND_REG: u32 = 0x00F30054;

// =============================================================================
// RegMem Extension Trait
// =============================================================================

/// Extension trait that adds register/memory access functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait RegMemExt {
    /// Write 32-bit words to register/memory
    ///
    /// # Arguments
    /// * `address` - Starting memory address
    /// * `data` - Array of 32-bit words to write (max 64 words)
    async fn regmem_write_regmem32(&mut self, address: u32, data: &[u32]) -> Result<(), RadioError>;

    /// Read 32-bit words from register/memory
    ///
    /// # Arguments
    /// * `address` - Starting memory address
    /// * `buffer` - Buffer to store read words (max 64 words)
    ///
    /// # Returns
    /// Number of words read
    async fn regmem_read_regmem32(&mut self, address: u32, buffer: &mut [u32]) -> Result<usize, RadioError>;

    /// Write bytes to memory
    ///
    /// # Arguments
    /// * `address` - Starting memory address
    /// * `data` - Bytes to write
    async fn regmem_write_mem8(&mut self, address: u32, data: &[u8]) -> Result<(), RadioError>;

    /// Read bytes from memory
    ///
    /// # Arguments
    /// * `address` - Starting memory address
    /// * `buffer` - Buffer to store read bytes
    ///
    /// # Returns
    /// Number of bytes read
    async fn regmem_read_mem8(&mut self, address: u32, buffer: &mut [u8]) -> Result<usize, RadioError>;

    /// Write bytes to TX buffer
    ///
    /// # Arguments
    /// * `data` - Bytes to write to TX buffer
    async fn regmem_write_buffer8(&mut self, data: &[u8]) -> Result<(), RadioError>;

    /// Read bytes from RX buffer
    ///
    /// # Arguments
    /// * `offset` - Offset within RX buffer
    /// * `buffer` - Buffer to store read bytes
    ///
    /// # Returns
    /// Number of bytes read
    async fn regmem_read_buffer8(&mut self, offset: u8, buffer: &mut [u8]) -> Result<usize, RadioError>;

    /// Clear the RX buffer
    ///
    /// Sets all bytes in the RX buffer to 0x00.
    async fn regmem_clear_rxbuffer(&mut self) -> Result<(), RadioError>;

    /// Read-modify-write a 32-bit register with mask
    ///
    /// Performs: register = (register & ~mask) | (data & mask)
    ///
    /// # Arguments
    /// * `address` - Register address
    /// * `mask` - Bits to modify (1 = modify, 0 = preserve)
    /// * `data` - New data for masked bits
    async fn regmem_write_regmem32_mask(&mut self, address: u32, mask: u32, data: u32) -> Result<(), RadioError>;

    /// Apply the workaround for the High ACP (Adjacent Channel Power) limitation
    ///
    /// This workaround should be called when the chip wakes up from sleep mode
    /// with retention, before any transmission.
    ///
    /// Affected firmware versions:
    /// - LR1110 firmware from 0x0303 to 0x0307
    /// - LR1120 firmware 0x0101
    ///
    /// The workaround resets bit 30 in register 0x00F30054.
    ///
    /// Reference: SWDR001 README.md, "LR11xx firmware known limitations"
    async fn apply_high_acp_workaround(&mut self) -> Result<(), RadioError>;
}

// =============================================================================
// Implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> RegMemExt for Lr1110<SPI, IV, C>
where
    SPI: SpiDevice<u8>,
    IV: InterfaceVariant,
    C: Lr1110Variant,
{
    async fn regmem_write_regmem32(&mut self, address: u32, data: &[u32]) -> Result<(), RadioError> {
        if data.len() > REGMEM_MAX_READ_WRITE_WORDS {
            return Err(RadioError::PayloadSizeMismatch(
                REGMEM_MAX_READ_WRITE_WORDS,
                data.len(),
            ));
        }

        let opcode = RegMemOpCode::WriteRegMem32.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        // Convert 32-bit words to bytes (big-endian)
        let mut payload = [0u8; REGMEM_MAX_READ_WRITE_WORDS * 4];
        for (i, word) in data.iter().enumerate() {
            let idx = i * 4;
            payload[idx] = (*word >> 24) as u8;
            payload[idx + 1] = (*word >> 16) as u8;
            payload[idx + 2] = (*word >> 8) as u8;
            payload[idx + 3] = *word as u8;
        }

        let payload_len = data.len() * 4;
        self.execute_command_with_payload(&cmd, &payload[..payload_len]).await
    }

    async fn regmem_read_regmem32(&mut self, address: u32, buffer: &mut [u32]) -> Result<usize, RadioError> {
        if buffer.len() > REGMEM_MAX_READ_WRITE_WORDS {
            return Err(RadioError::PayloadSizeMismatch(
                REGMEM_MAX_READ_WRITE_WORDS,
                buffer.len(),
            ));
        }

        let opcode = RegMemOpCode::ReadRegMem32.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
            buffer.len() as u8,
        ];

        let mut rbuffer = [0u8; REGMEM_MAX_READ_WRITE_WORDS * 4];
        let read_len = buffer.len() * 4;
        self.execute_command_with_response(&cmd, &mut rbuffer[..read_len]).await?;

        // Convert bytes to 32-bit words (big-endian)
        for (i, word) in buffer.iter_mut().enumerate() {
            let idx = i * 4;
            *word = ((rbuffer[idx] as u32) << 24)
                | ((rbuffer[idx + 1] as u32) << 16)
                | ((rbuffer[idx + 2] as u32) << 8)
                | (rbuffer[idx + 3] as u32);
        }

        Ok(buffer.len())
    }

    async fn regmem_write_mem8(&mut self, address: u32, data: &[u8]) -> Result<(), RadioError> {
        let opcode = RegMemOpCode::WriteMem8.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
        ];

        self.execute_command_with_payload(&cmd, data).await
    }

    async fn regmem_read_mem8(&mut self, address: u32, buffer: &mut [u8]) -> Result<usize, RadioError> {
        let opcode = RegMemOpCode::ReadMem8.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
            buffer.len() as u8,
        ];

        self.execute_command_with_response(&cmd, buffer).await?;
        Ok(buffer.len())
    }

    async fn regmem_write_buffer8(&mut self, data: &[u8]) -> Result<(), RadioError> {
        let opcode = RegMemOpCode::WriteBuffer8.bytes();
        let cmd = [opcode[0], opcode[1]];

        self.execute_command_with_payload(&cmd, data).await
    }

    async fn regmem_read_buffer8(&mut self, offset: u8, buffer: &mut [u8]) -> Result<usize, RadioError> {
        let opcode = RegMemOpCode::ReadBuffer8.bytes();
        let cmd = [opcode[0], opcode[1], offset, buffer.len() as u8];

        self.execute_command_with_response(&cmd, buffer).await?;
        Ok(buffer.len())
    }

    async fn regmem_clear_rxbuffer(&mut self) -> Result<(), RadioError> {
        let opcode = RegMemOpCode::ClearRxBuffer.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn regmem_write_regmem32_mask(&mut self, address: u32, mask: u32, data: u32) -> Result<(), RadioError> {
        let opcode = RegMemOpCode::WriteRegMem32Mask.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 24) as u8,
            (address >> 16) as u8,
            (address >> 8) as u8,
            address as u8,
            (mask >> 24) as u8,
            (mask >> 16) as u8,
            (mask >> 8) as u8,
            mask as u8,
            (data >> 24) as u8,
            (data >> 16) as u8,
            (data >> 8) as u8,
            data as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn apply_high_acp_workaround(&mut self) -> Result<(), RadioError> {
        // Write 32-bit register with mask: clear bit 30 at address 0x00F30054
        self.regmem_write_regmem32_mask(HIGH_ACP_WORKAROUND_REG, 1 << 30, 0).await
    }
}
