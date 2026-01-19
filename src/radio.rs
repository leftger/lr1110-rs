//! General radio control functions for the LR1110
//!
//! This module provides low-level radio control functions that are not specific
//! to any particular feature (LoRa, GNSS, WiFi, etc.) but are needed for advanced
//! radio operations like ranging, custom modulation, or direct packet handling.

use lora_phy::mod_params::RadioError;

// =============================================================================
// PA Configuration Types
// =============================================================================

/// Power amplifier selection
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PaSelection {
    /// Low power PA (up to +14 dBm)
    LP = 0x00,
    /// High power PA (up to +22 dBm)
    HP = 0x01,
    /// High frequency PA (for 2.4 GHz)
    HF = 0x02,
}

/// PA regulator supply source
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PaRegSupply {
    /// Regulated voltage supply
    VREG = 0x00,
    /// Battery voltage supply
    VBAT = 0x01,
}

/// PA configuration
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PaConfig {
    /// PA selection
    pub pa_sel: PaSelection,
    /// PA regulator supply
    pub pa_reg_supply: PaRegSupply,
    /// PA duty cycle (0-7 for LP, 0-4 for HP)
    pub pa_duty_cycle: u8,
    /// PA HP selection (0-7 HP slices)
    pub pa_hp_sel: u8,
}

// =============================================================================
// Ramp Time
// =============================================================================

/// TX ramp time configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RampTime {
    /// 16 microseconds
    Ramp16Us = 0x00,
    /// 32 microseconds
    Ramp32Us = 0x01,
    /// 48 microseconds
    Ramp48Us = 0x02,
    /// 64 microseconds
    Ramp64Us = 0x03,
    /// 80 microseconds
    Ramp80Us = 0x04,
    /// 96 microseconds
    Ramp96Us = 0x05,
    /// 112 microseconds
    Ramp112Us = 0x06,
    /// 128 microseconds
    Ramp128Us = 0x07,
    /// 144 microseconds
    Ramp144Us = 0x08,
    /// 160 microseconds
    Ramp160Us = 0x09,
    /// 176 microseconds
    Ramp176Us = 0x0A,
    /// 192 microseconds
    Ramp192Us = 0x0B,
    /// 208 microseconds
    Ramp208Us = 0x0C,
    /// 240 microseconds
    Ramp240Us = 0x0D,
    /// 272 microseconds
    Ramp272Us = 0x0E,
    /// 304 microseconds
    Ramp304Us = 0x0F,
}

// =============================================================================
// GFSK Modulation Types
// =============================================================================

/// GFSK pulse shaping filter
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPulseShape {
    /// No filtering
    Off = 0x00,
    /// Gaussian BT 0.3
    Bt03 = 0x08,
    /// Gaussian BT 0.5
    Bt05 = 0x09,
    /// Gaussian BT 0.7
    Bt07 = 0x0A,
    /// Gaussian BT 1.0
    Bt10 = 0x0B,
}

/// GFSK bandwidth (double-sided)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskBandwidth {
    /// 4.8 kHz
    Bw4800 = 0x1F,
    /// 5.8 kHz
    Bw5800 = 0x17,
    /// 7.3 kHz
    Bw7300 = 0x0F,
    /// 9.7 kHz
    Bw9700 = 0x1E,
    /// 11.7 kHz
    Bw11700 = 0x16,
    /// 14.6 kHz
    Bw14600 = 0x0E,
    /// 19.5 kHz
    Bw19500 = 0x1D,
    /// 23.4 kHz
    Bw23400 = 0x15,
    /// 29.3 kHz
    Bw29300 = 0x0D,
    /// 39.0 kHz
    Bw39000 = 0x1C,
    /// 46.9 kHz
    Bw46900 = 0x14,
    /// 58.6 kHz
    Bw58600 = 0x0C,
    /// 78.2 kHz
    Bw78200 = 0x1B,
    /// 93.8 kHz
    Bw93800 = 0x13,
    /// 117.3 kHz
    Bw117300 = 0x0B,
    /// 156.2 kHz
    Bw156200 = 0x1A,
    /// 187.2 kHz
    Bw187200 = 0x12,
    /// 234.3 kHz
    Bw234300 = 0x0A,
    /// 312.0 kHz
    Bw312000 = 0x19,
    /// 373.6 kHz
    Bw373600 = 0x11,
    /// 467.0 kHz
    Bw467000 = 0x09,
}

/// GFSK modulation parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskModParams {
    /// Bitrate in bps
    pub br_in_bps: u32,
    /// Pulse shaping filter
    pub pulse_shape: GfskPulseShape,
    /// Bandwidth parameter (double-sided)
    pub bw_dsb_param: GfskBandwidth,
    /// Frequency deviation in Hz
    pub fdev_in_hz: u32,
}

// =============================================================================
// GFSK Packet Types
// =============================================================================

/// GFSK preamble detector length
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskPreambleDetector {
    /// Preamble detector off
    Off = 0x00,
    /// Minimum 8 bits
    Min8Bits = 0x04,
    /// Minimum 16 bits
    Min16Bits = 0x05,
    /// Minimum 24 bits
    Min24Bits = 0x06,
    /// Minimum 32 bits
    Min32Bits = 0x07,
}

/// GFSK address filtering configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskAddressFiltering {
    /// Address filtering disabled
    Disabled = 0x00,
    /// Filter on node address
    NodeAddress = 0x01,
    /// Filter on node and broadcast addresses
    NodeAndBroadcast = 0x02,
}

/// GFSK header type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskHeaderType {
    /// Fixed length (no header)
    FixedLength = 0x00,
    /// Variable length (header included)
    VariableLength = 0x01,
}

/// GFSK CRC type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskCrcType {
    /// CRC off
    Off = 0x01,
    /// 1 byte CRC
    Crc1Byte = 0x00,
    /// 2 bytes CRC
    Crc2Bytes = 0x02,
    /// 1 byte CRC inverted
    Crc1ByteInv = 0x04,
    /// 2 bytes CRC inverted
    Crc2BytesInv = 0x06,
}

/// GFSK DC-free encoding
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum GfskDcFree {
    /// DC-free encoding off
    Off = 0x00,
    /// Whitening enabled
    Whitening = 0x01,
}

/// GFSK packet parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskPktParams {
    /// Preamble length in bits
    pub preamble_len_in_bits: u16,
    /// Preamble detector configuration
    pub preamble_detector: GfskPreambleDetector,
    /// Sync word length in bits
    pub sync_word_len_in_bits: u8,
    /// Address filtering configuration
    pub address_filtering: GfskAddressFiltering,
    /// Header type
    pub header_type: GfskHeaderType,
    /// Payload length in bytes
    pub pld_len_in_bytes: u8,
    /// CRC type
    pub crc_type: GfskCrcType,
    /// DC-free encoding
    pub dc_free: GfskDcFree,
}

// =============================================================================
// CAD Configuration
// =============================================================================

/// CAD exit mode
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CadExitMode {
    /// Return to standby RC after CAD
    StandbyRC = 0x00,
    /// Enter RX mode after CAD if detected
    Rx = 0x01,
    /// Enter TX mode after CAD if detected
    Tx = 0x02,
}

/// CAD parameters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct CadParams {
    /// Number of CAD symbols (1-8)
    pub cad_symb_nb: u8,
    /// CAD detection peak (default: 0x32)
    pub cad_detect_peak: u8,
    /// CAD detection minimum (default: 0x0A)
    pub cad_detect_min: u8,
    /// Exit mode after CAD
    pub cad_exit_mode: CadExitMode,
    /// CAD timeout in 31.25us units
    pub cad_timeout: u32,
}

// =============================================================================
// Statistics Types
// =============================================================================

/// GFSK packet statistics
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct GfskStats {
    /// Number of packets received
    pub nb_pkt_received: u16,
    /// Number of packets with CRC error
    pub nb_pkt_crc_error: u16,
    /// Number of packets with length error
    pub nb_pkt_len_error: u16,
}

/// LoRa packet statistics
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct LoRaStats {
    /// Number of packets received
    pub nb_pkt_received: u16,
    /// Number of packets with CRC error
    pub nb_pkt_crc_error: u16,
    /// Number of packets with header error
    pub nb_pkt_header_error: u16,
    /// Number of false sync detections
    pub nb_pkt_falsesync: u16,
}

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

    // PA & TX Configuration
    /// Set PA configuration
    ///
    /// # Arguments
    /// * `cfg` - PA configuration (selection, supply, duty cycle, HP selection)
    async fn set_pa_cfg(&mut self, cfg: &PaConfig) -> Result<(), RadioError>;

    /// Set TX parameters
    ///
    /// # Arguments
    /// * `power_dbm` - Output power in dBm
    /// * `ramp_time` - PA ramp time
    async fn set_tx_params(&mut self, power_dbm: i8, ramp_time: RampTime) -> Result<(), RadioError>;

    // GFSK Support
    /// Set GFSK modulation parameters
    ///
    /// # Arguments
    /// * `params` - GFSK modulation parameters
    async fn set_gfsk_mod_params(&mut self, params: &GfskModParams) -> Result<(), RadioError>;

    /// Set GFSK packet parameters
    ///
    /// # Arguments
    /// * `params` - GFSK packet parameters
    async fn set_gfsk_pkt_params(&mut self, params: &GfskPktParams) -> Result<(), RadioError>;

    // RX Configuration
    /// Configure boosted RX mode
    ///
    /// # Arguments
    /// * `enable` - Enable boosted RX for improved sensitivity
    async fn cfg_rx_boosted(&mut self, enable: bool) -> Result<(), RadioError>;

    // CAD Support
    /// Set CAD (Channel Activity Detection) parameters
    ///
    /// # Arguments
    /// * `params` - CAD parameters
    async fn set_cad_params(&mut self, params: &CadParams) -> Result<(), RadioError>;

    // Statistics
    /// Reset packet statistics
    async fn reset_stats(&mut self) -> Result<(), RadioError>;

    /// Get GFSK packet statistics
    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError>;

    /// Get LoRa packet statistics
    async fn get_lora_stats(&mut self) -> Result<LoRaStats, RadioError>;
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

    // PA & TX Configuration
    async fn set_pa_cfg(&mut self, cfg: &PaConfig) -> Result<(), RadioError> {
        // OpCode: 0x0215
        let cmd = [
            0x02,
            0x15,
            cfg.pa_sel as u8,
            cfg.pa_reg_supply as u8,
            cfg.pa_duty_cycle,
            cfg.pa_hp_sel,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_tx_params(&mut self, power_dbm: i8, ramp_time: RampTime) -> Result<(), RadioError> {
        // OpCode: 0x0211
        let cmd = [0x02, 0x11, power_dbm as u8, ramp_time as u8];
        self.execute_command(&cmd).await
    }

    // GFSK Support
    async fn set_gfsk_mod_params(&mut self, params: &GfskModParams) -> Result<(), RadioError> {
        // OpCode: 0x020F
        let cmd = [
            0x02,
            0x0F,
            (params.br_in_bps >> 16) as u8,
            (params.br_in_bps >> 8) as u8,
            params.br_in_bps as u8,
            params.pulse_shape as u8,
            params.bw_dsb_param as u8,
            (params.fdev_in_hz >> 16) as u8,
            (params.fdev_in_hz >> 8) as u8,
            params.fdev_in_hz as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_gfsk_pkt_params(&mut self, params: &GfskPktParams) -> Result<(), RadioError> {
        // OpCode: 0x0210
        let cmd = [
            0x02,
            0x10,
            (params.preamble_len_in_bits >> 8) as u8,
            params.preamble_len_in_bits as u8,
            params.preamble_detector as u8,
            params.sync_word_len_in_bits,
            params.address_filtering as u8,
            params.header_type as u8,
            params.pld_len_in_bytes,
            params.crc_type as u8,
            params.dc_free as u8,
        ];
        self.execute_command(&cmd).await
    }

    // RX Configuration
    async fn cfg_rx_boosted(&mut self, enable: bool) -> Result<(), RadioError> {
        // OpCode: 0x0227
        let cmd = [0x02, 0x27, enable as u8];
        self.execute_command(&cmd).await
    }

    // CAD Support
    async fn set_cad_params(&mut self, params: &CadParams) -> Result<(), RadioError> {
        // OpCode: 0x020D
        let cmd = [
            0x02,
            0x0D,
            params.cad_symb_nb,
            params.cad_detect_peak,
            params.cad_detect_min,
            params.cad_exit_mode as u8,
            (params.cad_timeout >> 16) as u8,
            (params.cad_timeout >> 8) as u8,
            params.cad_timeout as u8,
        ];
        self.execute_command(&cmd).await
    }

    // Statistics
    async fn reset_stats(&mut self) -> Result<(), RadioError> {
        // OpCode: 0x0200
        let cmd = [0x02, 0x00];
        self.execute_command(&cmd).await
    }

    async fn get_gfsk_stats(&mut self) -> Result<GfskStats, RadioError> {
        // OpCode: 0x0201
        let cmd = [0x02, 0x01];
        let mut rbuffer = [0u8; 6];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(GfskStats {
            nb_pkt_received: ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16),
            nb_pkt_crc_error: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
            nb_pkt_len_error: ((rbuffer[4] as u16) << 8) | (rbuffer[5] as u16),
        })
    }

    async fn get_lora_stats(&mut self) -> Result<LoRaStats, RadioError> {
        // OpCode: 0x0201
        let cmd = [0x02, 0x01];
        let mut rbuffer = [0u8; 8];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(LoRaStats {
            nb_pkt_received: ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16),
            nb_pkt_crc_error: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
            nb_pkt_header_error: ((rbuffer[4] as u16) << 8) | (rbuffer[5] as u16),
            nb_pkt_falsesync: ((rbuffer[6] as u16) << 8) | (rbuffer[7] as u16),
        })
    }
}
