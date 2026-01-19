//! General radio control functions for the LR1110
//!
//! This module provides low-level radio control functions that are not specific
//! to any particular feature (LoRa, GNSS, WiFi, etc.) but are needed for advanced
//! radio operations like ranging, custom modulation, or direct packet handling.

use lora_phy::mod_params::RadioError;

// =============================================================================
// Radio Control Extension Trait
// =============================================================================

/// Extension trait that adds general radio control functionality to the LR1110.
#[allow(async_fn_in_trait)]
pub trait RadioControlExt {
    /// Set the packet type (LoRa, GFSK, RTToF, LR-FHSS, etc.)
    ///
    /// # Packet Types
    /// - `0x00`: GFSK
    /// - `0x01`: LoRa
    /// - `0x02`: LR-FHSS
    /// - `0x05`: RTToF (Round-Trip Time of Flight)
    async fn set_packet_type(&mut self, packet_type: u8) -> Result<(), RadioError>;

    /// Set the RF frequency in Hz
    ///
    /// # Arguments
    /// * `frequency_hz` - Frequency in Hz (e.g., 915000000 for 915 MHz)
    ///
    /// # Note
    /// Frequency is converted to chip format: steps of Fxtal/2^18 where Fxtal = 32 MHz
    async fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), RadioError>;

    /// Set DIO IRQ parameters with custom mask
    ///
    /// # Arguments
    /// * `irq_mask` - 32-bit IRQ mask (see LR11xx datasheet for bit definitions)
    async fn set_dio_irq_params(&mut self, irq_mask: u32) -> Result<(), RadioError>;

    /// Start transmission with timeout
    ///
    /// # Arguments
    /// * `timeout_rtc_steps` - Timeout in RTC steps (15.625 us per step)
    ///
    /// # Note
    /// Use 0 for no timeout (single mode)
    async fn set_tx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError>;

    /// Start reception with timeout
    ///
    /// # Arguments
    /// * `timeout_rtc_steps` - Timeout in RTC steps (15.625 us per step)
    ///
    /// # Note
    /// Use 0xFFFFFF for continuous RX mode
    async fn set_rx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError>;

    /// Set LoRa modulation parameters
    ///
    /// # Arguments
    /// * `sf` - Spreading factor (5-12)
    /// * `bw` - Bandwidth (see lora_bw module in ranging.rs)
    /// * `cr` - Coding rate (see lora_cr module in ranging.rs)
    /// * `ldro` - Low data rate optimization (0 = off, 1 = on)
    async fn set_lora_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: u8) -> Result<(), RadioError>;

    /// Set LoRa packet parameters
    ///
    /// # Arguments
    /// * `preamble_len` - Preamble length in symbols
    /// * `header_type` - 0 = explicit header, 1 = implicit header
    /// * `payload_len` - Payload length in bytes
    /// * `crc_on` - 0 = CRC off, 1 = CRC on
    /// * `iq_inverted` - 0 = standard IQ, 1 = inverted IQ
    async fn set_lora_pkt_params(
        &mut self,
        preamble_len: u16,
        header_type: u8,
        payload_len: u8,
        crc_on: u8,
        iq_inverted: u8,
    ) -> Result<(), RadioError>;

    /// Set LoRa sync word
    ///
    /// # Arguments
    /// * `sync_word` - Sync word value (0x12 for private network, 0x34 for public network)
    async fn set_lora_sync_word(&mut self, sync_word: u8) -> Result<(), RadioError>;

    /// Write data to TX buffer
    ///
    /// # Arguments
    /// * `offset` - Offset in TX buffer (usually 0)
    /// * `data` - Data to write
    async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), RadioError>;

    /// Read data from RX buffer
    ///
    /// # Arguments
    /// * `offset` - Offset in RX buffer (usually 0)
    /// * `length` - Number of bytes to read
    /// * `buffer` - Buffer to store read data
    async fn read_buffer(&mut self, offset: u8, length: u8, buffer: &mut [u8]) -> Result<(), RadioError>;

    /// Get RX buffer status
    ///
    /// # Returns
    /// (payload_length, rx_start_buffer_pointer)
    async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError>;

    /// Get LoRa packet status (RSSI and SNR)
    ///
    /// # Returns
    /// (rssi_pkt_in_dbm, snr_pkt_in_db)
    async fn get_lora_pkt_status(&mut self) -> Result<(i16, i8), RadioError>;
}

// =============================================================================
// RadioControlExt trait implementation for Lr1110
// =============================================================================

impl<SPI, IV, C> RadioControlExt for lora_phy::lr1110::Lr1110<SPI, IV, C>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    IV: lora_phy::mod_traits::InterfaceVariant,
    C: lora_phy::lr1110::variant::Lr1110Variant,
{
    async fn set_packet_type(&mut self, packet_type: u8) -> Result<(), RadioError> {
        // OpCode: 0x8A
        let cmd = [0x8A, packet_type];
        self.execute_command(&cmd).await
    }

    async fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), RadioError> {
        // OpCode: 0x86
        // Frequency is sent in steps of Fxtal/2^18 where Fxtal = 32 MHz
        let freq_val = ((frequency_hz as u64) << 18) / 32_000_000;
        let cmd = [
            0x86,
            (freq_val >> 24) as u8,
            (freq_val >> 16) as u8,
            (freq_val >> 8) as u8,
            freq_val as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_dio_irq_params(&mut self, irq_mask: u32) -> Result<(), RadioError> {
        // OpCode: 0x82
        let cmd = [
            0x82,
            (irq_mask >> 24) as u8,
            (irq_mask >> 16) as u8,
            (irq_mask >> 8) as u8,
            irq_mask as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_tx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError> {
        // OpCode: 0x83
        let cmd = [
            0x83,
            (timeout_rtc_steps >> 16) as u8,
            (timeout_rtc_steps >> 8) as u8,
            timeout_rtc_steps as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_rx(&mut self, timeout_rtc_steps: u32) -> Result<(), RadioError> {
        // OpCode: 0x82
        let cmd = [
            0x82,
            (timeout_rtc_steps >> 16) as u8,
            (timeout_rtc_steps >> 8) as u8,
            timeout_rtc_steps as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_lora_mod_params(&mut self, sf: u8, bw: u8, cr: u8, ldro: u8) -> Result<(), RadioError> {
        // OpCode: 0x8B
        let cmd = [0x8B, sf, bw, cr, ldro];
        self.execute_command(&cmd).await
    }

    async fn set_lora_pkt_params(
        &mut self,
        preamble_len: u16,
        header_type: u8,
        payload_len: u8,
        crc_on: u8,
        iq_inverted: u8,
    ) -> Result<(), RadioError> {
        // OpCode: 0x8C
        let cmd = [
            0x8C,
            (preamble_len >> 8) as u8,
            (preamble_len & 0xFF) as u8,
            header_type,
            payload_len,
            crc_on,
            iq_inverted,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_lora_sync_word(&mut self, sync_word: u8) -> Result<(), RadioError> {
        // OpCode: 0x9D
        let cmd = [0x9D, sync_word];
        self.execute_command(&cmd).await
    }

    async fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), RadioError> {
        // OpCode: 0x0E
        let cmd = [0x0E, offset];
        self.execute_command_with_payload(&cmd, data).await
    }

    async fn read_buffer(&mut self, offset: u8, length: u8, buffer: &mut [u8]) -> Result<(), RadioError> {
        // OpCode: 0x0D
        let cmd = [0x0D, offset, length];
        if buffer.len() < length as usize {
            return Err(RadioError::PayloadSizeMismatch(length as usize, buffer.len()));
        }
        self.execute_command_with_response(&cmd, &mut buffer[..length as usize]).await
    }

    async fn get_rx_buffer_status(&mut self) -> Result<(u8, u8), RadioError> {
        // OpCode: 0x17
        let cmd = [0x17];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok((rbuffer[0], rbuffer[1]))
    }

    async fn get_lora_pkt_status(&mut self) -> Result<(i16, i8), RadioError> {
        // OpCode: 0x1D
        let cmd = [0x1D];
        let mut rbuffer = [0u8; 3];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        // Parse RSSI and SNR from response
        let rssi = -((rbuffer[0] as i16) >> 1);
        let snr = (rbuffer[1] as i8) >> 2;

        Ok((rssi, snr))
    }
}
