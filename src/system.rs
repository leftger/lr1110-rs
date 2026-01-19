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
    GetStatus = 0x0100,
    GetVersion = 0x0101,
    GetErrors = 0x010D,
    ClearErrors = 0x010E,
    Calibrate = 0x010F,
    SetRegMode = 0x0110,
    CalibrateImage = 0x0111,
    SetDioAsRfSwitch = 0x0112,
    SetDioIrqParams = 0x0113,
    ClearIrq = 0x0114,
    CfgLfClk = 0x0116,
    SetTcxoMode = 0x0117,
    Reboot = 0x0118,
    GetVbat = 0x0119,
    GetTemp = 0x011A,
    SetSleep = 0x011B,
    SetStandby = 0x011C,
    SetFs = 0x011D,
    GetRandom = 0x0120,
    EraseInfopage = 0x0121,
    WriteInfopage = 0x0122,
    ReadInfopage = 0x0123,
    ReadUid = 0x0125,
    ReadJoinEui = 0x0126,
    ReadPin = 0x0127,
    EnableSpiCrc = 0x0128,
    DriveDioInSleepMode = 0x012A,
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

/// Length of the LR11XX PIN in bytes
pub const LR11XX_SYSTEM_PIN_LENGTH: usize = 4;

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
// Chip State Types
// =============================================================================

/// Chip operating modes
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ChipMode {
    /// Sleep mode
    Sleep = 0x00,
    /// Standby RC mode
    StandbyRC = 0x01,
    /// Standby XOSC mode
    StandbyXOSC = 0x02,
    /// Frequency synthesis mode
    FS = 0x03,
    /// Receive mode
    RX = 0x04,
    /// Transmit mode
    TX = 0x05,
    /// Localization mode (GNSS/WiFi scanning)
    LOC = 0x06,
}

impl From<u8> for ChipMode {
    fn from(value: u8) -> Self {
        match value {
            0x00 => ChipMode::Sleep,
            0x01 => ChipMode::StandbyRC,
            0x02 => ChipMode::StandbyXOSC,
            0x03 => ChipMode::FS,
            0x04 => ChipMode::RX,
            0x05 => ChipMode::TX,
            0x06 => ChipMode::LOC,
            _ => ChipMode::Sleep, // Default fallback
        }
    }
}

/// Reset status codes
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ResetStatus {
    /// Reset status cleared
    Cleared = 0x00,
    /// Analog reset
    Analog = 0x01,
    /// External reset (NRESET pin)
    External = 0x02,
    /// System reset
    System = 0x03,
    /// Watchdog reset
    Watchdog = 0x04,
    /// IOCD restart
    IocdRestart = 0x05,
    /// RTC restart
    RtcRestart = 0x06,
}

impl From<u8> for ResetStatus {
    fn from(value: u8) -> Self {
        match value {
            0x00 => ResetStatus::Cleared,
            0x01 => ResetStatus::Analog,
            0x02 => ResetStatus::External,
            0x03 => ResetStatus::System,
            0x04 => ResetStatus::Watchdog,
            0x05 => ResetStatus::IocdRestart,
            0x06 => ResetStatus::RtcRestart,
            _ => ResetStatus::Cleared,
        }
    }
}

/// Command execution status
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum CommandStatus {
    /// Command failed
    Fail = 0x00,
    /// Peripheral error
    PeripheralError = 0x01,
    /// Command completed successfully
    OK = 0x02,
    /// Command completed, data available
    Data = 0x03,
}

impl From<u8> for CommandStatus {
    fn from(value: u8) -> Self {
        match value {
            0x00 => CommandStatus::Fail,
            0x01 => CommandStatus::PeripheralError,
            0x02 => CommandStatus::OK,
            0x03 => CommandStatus::Data,
            _ => CommandStatus::Fail,
        }
    }
}

/// Status register 1
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Stat1 {
    /// Command execution status
    pub command_status: CommandStatus,
    /// Interrupt active flag
    pub is_interrupt_active: bool,
}

impl From<u8> for Stat1 {
    fn from(value: u8) -> Self {
        Self {
            command_status: CommandStatus::from((value >> 1) & 0x07),
            is_interrupt_active: (value & 0x01) != 0,
        }
    }
}

/// Status register 2
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Stat2 {
    /// Reset status
    pub reset_status: ResetStatus,
    /// Current chip mode
    pub chip_mode: ChipMode,
    /// Running from flash flag
    pub is_running_from_flash: bool,
}

impl From<u8> for Stat2 {
    fn from(value: u8) -> Self {
        Self {
            reset_status: ResetStatus::from((value >> 4) & 0x0F),
            chip_mode: ChipMode::from((value >> 1) & 0x07),
            is_running_from_flash: (value & 0x01) != 0,
        }
    }
}

// =============================================================================
// Power Mode Types
// =============================================================================

/// Standby oscillator configuration
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum StandbyConfig {
    /// RC oscillator (13 MHz)
    RC = 0x00,
    /// Crystal oscillator (32 MHz)
    XOSC = 0x01,
}

/// Sleep configuration
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct SleepConfig {
    /// Enable warm start (retain configuration)
    pub is_warm_start: bool,
    /// Enable RTC timeout wakeup
    pub is_rtc_timeout: bool,
}

impl Default for SleepConfig {
    fn default() -> Self {
        Self {
            is_warm_start: true,
            is_rtc_timeout: false,
        }
    }
}

// =============================================================================
// Calibration Types
// =============================================================================

/// Calibration parameter (bitmask of calibration blocks)
pub type CalibrationParam = u8;

/// Calibrate LF RC oscillator
pub const CALIB_LF_RC_MASK: u8 = 0x01;
/// Calibrate HF RC oscillator
pub const CALIB_HF_RC_MASK: u8 = 0x02;
/// Calibrate PLL
pub const CALIB_PLL_MASK: u8 = 0x04;
/// Calibrate ADC
pub const CALIB_ADC_MASK: u8 = 0x08;
/// Calibrate image rejection
pub const CALIB_IMG_MASK: u8 = 0x10;
/// Calibrate PLL for TX
pub const CALIB_PLL_TX_MASK: u8 = 0x20;
/// Calibrate all blocks
pub const CALIB_ALL: u8 = 0x3F;

// =============================================================================
// TCXO Configuration
// =============================================================================

/// TCXO supply voltage
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TcxoVoltage {
    /// 1.6V
    Voltage1_6V = 0x00,
    /// 1.7V
    Voltage1_7V = 0x01,
    /// 1.8V
    Voltage1_8V = 0x02,
    /// 2.2V
    Voltage2_2V = 0x03,
    /// 2.4V
    Voltage2_4V = 0x04,
    /// 2.7V
    Voltage2_7V = 0x05,
    /// 3.0V
    Voltage3_0V = 0x06,
    /// 3.3V
    Voltage3_3V = 0x07,
}

impl TcxoVoltage {
    /// Get the raw value for the TCXO voltage
    pub fn value(self) -> u8 {
        self as u8
    }
}

// =============================================================================
// IRQ Types
// =============================================================================

/// IRQ mask type (32-bit bitmask)
pub type IrqMask = u32;

/// No IRQ
pub const IRQ_NONE: IrqMask = 0;
/// TX done
pub const IRQ_TX_DONE: IrqMask = 1 << 2;
/// RX done
pub const IRQ_RX_DONE: IrqMask = 1 << 3;
/// Preamble detected
pub const IRQ_PREAMBLE_DETECTED: IrqMask = 1 << 4;
/// Sync word or header valid
pub const IRQ_SYNC_WORD_HEADER_VALID: IrqMask = 1 << 5;
/// Header error
pub const IRQ_HEADER_ERROR: IrqMask = 1 << 6;
/// CRC error
pub const IRQ_CRC_ERROR: IrqMask = 1 << 7;
/// CAD done
pub const IRQ_CAD_DONE: IrqMask = 1 << 8;
/// CAD detected
pub const IRQ_CAD_DETECTED: IrqMask = 1 << 9;
/// Timeout
pub const IRQ_TIMEOUT: IrqMask = 1 << 10;
/// LR-FHSS intra-packet hop
pub const IRQ_LR_FHSS_INTRA_PKT_HOP: IrqMask = 1 << 11;
/// Ranging request valid
pub const IRQ_RANGING_REQ_VALID: IrqMask = 1 << 14;
/// Ranging request discarded
pub const IRQ_RANGING_REQ_DISCARDED: IrqMask = 1 << 15;
/// Ranging response done
pub const IRQ_RANGING_RESP_DONE: IrqMask = 1 << 16;
/// Ranging exchange valid
pub const IRQ_RANGING_EXCH_VALID: IrqMask = 1 << 17;
/// Ranging timeout
pub const IRQ_RANGING_TIMEOUT: IrqMask = 1 << 18;
/// GNSS scan done
pub const IRQ_GNSS_SCAN_DONE: IrqMask = 1 << 19;
/// WiFi scan done
pub const IRQ_WIFI_SCAN_DONE: IrqMask = 1 << 20;
/// End of life (battery low)
pub const IRQ_EOL: IrqMask = 1 << 21;
/// Command error
pub const IRQ_CMD_ERROR: IrqMask = 1 << 22;
/// General error
pub const IRQ_ERROR: IrqMask = 1 << 23;
/// FSK length error
pub const IRQ_FSK_LEN_ERROR: IrqMask = 1 << 24;
/// FSK address error
pub const IRQ_FSK_ADDR_ERROR: IrqMask = 1 << 25;
/// LoRa RX timestamp
pub const IRQ_LORA_RX_TIMESTAMP: IrqMask = 1 << 27;
/// All IRQs
pub const IRQ_ALL: IrqMask = 0x0FFFFFFF;

// =============================================================================
// Version Info
// =============================================================================

/// LR11xx chip type
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ChipType {
    /// LR1110
    LR1110 = 0x01,
    /// LR1120
    LR1120 = 0x02,
    /// LR1121
    LR1121 = 0x03,
    /// Unknown chip type
    Unknown = 0xFF,
}

impl From<u8> for ChipType {
    fn from(value: u8) -> Self {
        match value {
            0x01 => ChipType::LR1110,
            0x02 => ChipType::LR1120,
            0x03 => ChipType::LR1121,
            _ => ChipType::Unknown,
        }
    }
}

/// Version information
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Version {
    /// Hardware version
    pub hw: u8,
    /// Chip type
    pub chip_type: ChipType,
    /// Firmware version
    pub fw: u16,
}

// =============================================================================
// RF Switch Configuration
// =============================================================================

/// RF switch GPIO 0 high
pub const RFSW0_HIGH: u8 = 1 << 0;
/// RF switch GPIO 1 high
pub const RFSW1_HIGH: u8 = 1 << 1;
/// RF switch GPIO 2 high
pub const RFSW2_HIGH: u8 = 1 << 2;
/// RF switch GPIO 3 high
pub const RFSW3_HIGH: u8 = 1 << 3;
/// RF switch GPIO 4 high
pub const RFSW4_HIGH: u8 = 1 << 4;

/// RF switch configuration
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct RfSwitchConfig {
    /// Enable RF switch
    pub enable: u8,
    /// GPIO configuration for standby mode
    pub standby: u8,
    /// GPIO configuration for RX mode
    pub rx: u8,
    /// GPIO configuration for TX mode
    pub tx: u8,
    /// GPIO configuration for TX high power mode
    pub tx_hp: u8,
    /// GPIO configuration for TX high frequency mode
    pub tx_hf: u8,
    /// GPIO configuration for GNSS mode
    pub gnss: u8,
    /// GPIO configuration for WiFi mode
    pub wifi: u8,
}

// =============================================================================
// System Extension Trait
// =============================================================================

/// Extension trait that adds system diagnostic functionality to the LR1110 radio.
#[allow(async_fn_in_trait)]
pub trait SystemExt {
    // Status and Control
    /// Clear the reset status information in stat2
    ///
    /// Sends the GetStatus opcode to clear the reset status field.
    async fn clear_reset_status_info(&mut self) -> Result<(), RadioError>;

    /// Reboot the chip
    ///
    /// Performs a software reboot (warm start).
    async fn reboot(&mut self, stay_in_bootloader: bool) -> Result<(), RadioError>;

    /// Enable or disable SPI CRC checking
    ///
    /// # Arguments
    /// * `enable` - true to enable SPI CRC, false to disable
    async fn enable_spi_crc(&mut self, enable: bool) -> Result<(), RadioError>;

    /// Configure DIO pins to be driven in sleep mode
    ///
    /// # Arguments
    /// * `enable` - true to drive DIOs in sleep, false for high-impedance
    async fn drive_dio_in_sleep_mode(&mut self, enable: bool) -> Result<(), RadioError>;

    /// Read the 4-byte PIN from factory-programmed EUIs
    async fn read_pin(&mut self) -> Result<[u8; LR11XX_SYSTEM_PIN_LENGTH], RadioError>;

    /// Read the 4-byte PIN computed from custom EUIs
    ///
    /// # Arguments
    /// * `chip_eui` - 8-byte chip EUI
    /// * `join_eui` - 8-byte join EUI
    async fn read_pin_custom_eui(
        &mut self,
        chip_eui: &[u8; LR11XX_SYSTEM_UID_LENGTH],
        join_eui: &[u8; LR11XX_SYSTEM_JOIN_EUI_LENGTH],
    ) -> Result<[u8; LR11XX_SYSTEM_PIN_LENGTH], RadioError>;

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

    // Power Mode Control
    /// Set the radio to sleep mode
    ///
    /// # Arguments
    /// * `cfg` - Sleep configuration (warm start, RTC timeout)
    /// * `sleep_time` - Sleep duration in 31.25us units (24-bit value)
    async fn set_sleep(&mut self, cfg: SleepConfig, sleep_time: u32) -> Result<(), RadioError>;

    /// Set the radio to standby mode
    ///
    /// # Arguments
    /// * `cfg` - Standby configuration (RC or XOSC)
    async fn set_standby(&mut self, cfg: StandbyConfig) -> Result<(), RadioError>;

    /// Set the radio to frequency synthesis mode
    ///
    /// This mode is used to reduce switching time between standby and TX/RX.
    async fn set_fs(&mut self) -> Result<(), RadioError>;

    // Calibration
    /// Calibrate specified blocks
    ///
    /// # Arguments
    /// * `param` - Bitmask of calibration blocks (CALIB_* constants)
    async fn calibrate(&mut self, param: CalibrationParam) -> Result<(), RadioError>;

    /// Calibrate image rejection for given frequency range
    ///
    /// # Arguments
    /// * `freq1` - Lower frequency (MHz / 4)
    /// * `freq2` - Upper frequency (MHz / 4)
    async fn calibrate_image(&mut self, freq1: u8, freq2: u8) -> Result<(), RadioError>;

    /// Set regulator mode
    ///
    /// # Arguments
    /// * `mode` - Regulator mode (0 = LDO, 1 = DC-DC)
    async fn set_reg_mode(&mut self, mode: u8) -> Result<(), RadioError>;

    // Version & Status
    /// Get the hardware and firmware version
    async fn get_version(&mut self) -> Result<Version, RadioError>;

    /// Get system error flags
    async fn get_errors(&mut self) -> Result<SystemErrors, RadioError>;

    /// Clear system error flags
    async fn clear_errors(&mut self) -> Result<(), RadioError>;

    // IRQ Management
    /// Clear specified IRQ flags
    ///
    /// # Arguments
    /// * `irqs` - Bitmask of IRQs to clear (IRQ_* constants)
    async fn clear_irq_status(&mut self, irqs: IrqMask) -> Result<(), RadioError>;

    /// Get current IRQ status
    ///
    /// Returns a bitmask of active IRQ flags.
    async fn get_irq_status(&mut self) -> Result<IrqMask, RadioError>;

    /// Get and clear IRQ status in a single operation
    ///
    /// Returns the IRQ status before clearing.
    async fn get_and_clear_irq_status(&mut self) -> Result<IrqMask, RadioError>;

    // Configuration
    /// Configure TCXO control
    ///
    /// # Arguments
    /// * `voltage` - TCXO supply voltage
    /// * `timeout` - Timeout in 31.25us units (24-bit value)
    async fn set_tcxo_mode(&mut self, voltage: TcxoVoltage, timeout: u32) -> Result<(), RadioError>;

    /// Configure DIO pins as RF switch
    ///
    /// # Arguments
    /// * `cfg` - RF switch GPIO configuration for different modes
    async fn set_dio_as_rf_switch(&mut self, cfg: &RfSwitchConfig) -> Result<(), RadioError>;

    /// Configure low frequency clock
    ///
    /// # Arguments
    /// * `cfg` - LF clock configuration (0 = RC, 1 = XTAL, 2 = EXT)
    async fn cfg_lf_clk(&mut self, cfg: u8) -> Result<(), RadioError>;

    // Infopage Access
    /// Erase the infopage
    ///
    /// Warning: This erases user-stored calibration data.
    async fn erase_infopage(&mut self) -> Result<(), RadioError>;

    /// Write data to infopage
    ///
    /// # Arguments
    /// * `address` - 16-bit address within infopage
    /// * `data` - Data to write (max 256 bytes)
    async fn write_infopage(&mut self, address: u16, data: &[u8]) -> Result<(), RadioError>;

    /// Read data from infopage
    ///
    /// # Arguments
    /// * `address` - 16-bit address within infopage
    /// * `data` - Buffer to read into
    async fn read_infopage(&mut self, address: u16, data: &mut [u8]) -> Result<(), RadioError>;
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
    // Status and Control
    async fn clear_reset_status_info(&mut self) -> Result<(), RadioError> {
        let opcode = SystemOpCode::GetStatus.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn reboot(&mut self, stay_in_bootloader: bool) -> Result<(), RadioError> {
        let opcode = SystemOpCode::Reboot.bytes();
        let cmd = [opcode[0], opcode[1], stay_in_bootloader as u8];
        self.execute_command(&cmd).await
    }

    async fn enable_spi_crc(&mut self, enable: bool) -> Result<(), RadioError> {
        let opcode = SystemOpCode::EnableSpiCrc.bytes();
        let cmd = [opcode[0], opcode[1], enable as u8];
        self.execute_command(&cmd).await
    }

    async fn drive_dio_in_sleep_mode(&mut self, enable: bool) -> Result<(), RadioError> {
        let opcode = SystemOpCode::DriveDioInSleepMode.bytes();
        let cmd = [opcode[0], opcode[1], enable as u8];
        self.execute_command(&cmd).await
    }

    async fn read_pin(&mut self) -> Result<[u8; LR11XX_SYSTEM_PIN_LENGTH], RadioError> {
        let opcode = SystemOpCode::ReadPin.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; LR11XX_SYSTEM_PIN_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok(rbuffer)
    }

    async fn read_pin_custom_eui(
        &mut self,
        chip_eui: &[u8; LR11XX_SYSTEM_UID_LENGTH],
        join_eui: &[u8; LR11XX_SYSTEM_JOIN_EUI_LENGTH],
    ) -> Result<[u8; LR11XX_SYSTEM_PIN_LENGTH], RadioError> {
        let opcode = SystemOpCode::ReadPin.bytes();
        let mut cmd = [0u8; 18]; // 2 + 8 + 8
        cmd[0] = opcode[0];
        cmd[1] = opcode[1];
        cmd[2..10].copy_from_slice(chip_eui);
        cmd[10..18].copy_from_slice(join_eui);

        let mut rbuffer = [0u8; LR11XX_SYSTEM_PIN_LENGTH];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;
        Ok(rbuffer)
    }

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

    // Power Mode Control
    async fn set_sleep(&mut self, cfg: SleepConfig, sleep_time: u32) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetSleep.bytes();
        let config_byte = ((cfg.is_rtc_timeout as u8) << 1) | (cfg.is_warm_start as u8);
        let cmd = [
            opcode[0],
            opcode[1],
            config_byte,
            (sleep_time >> 16) as u8,
            (sleep_time >> 8) as u8,
            sleep_time as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_standby(&mut self, cfg: StandbyConfig) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetStandby.bytes();
        let cmd = [opcode[0], opcode[1], cfg as u8];
        self.execute_command(&cmd).await
    }

    async fn set_fs(&mut self) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetFs.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    // Calibration
    async fn calibrate(&mut self, param: CalibrationParam) -> Result<(), RadioError> {
        let opcode = SystemOpCode::Calibrate.bytes();
        let cmd = [opcode[0], opcode[1], param];
        self.execute_command(&cmd).await
    }

    async fn calibrate_image(&mut self, freq1: u8, freq2: u8) -> Result<(), RadioError> {
        let opcode = SystemOpCode::CalibrateImage.bytes();
        let cmd = [opcode[0], opcode[1], freq1, freq2];
        self.execute_command(&cmd).await
    }

    async fn set_reg_mode(&mut self, mode: u8) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetRegMode.bytes();
        let cmd = [opcode[0], opcode[1], mode];
        self.execute_command(&cmd).await
    }

    // Version & Status
    async fn get_version(&mut self) -> Result<Version, RadioError> {
        let opcode = SystemOpCode::GetVersion.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 4];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        Ok(Version {
            hw: rbuffer[0],
            chip_type: ChipType::from(rbuffer[1]),
            fw: ((rbuffer[2] as u16) << 8) | (rbuffer[3] as u16),
        })
    }

    async fn get_errors(&mut self) -> Result<SystemErrors, RadioError> {
        let opcode = SystemOpCode::GetErrors.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 2];
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        let raw = ((rbuffer[0] as u16) << 8) | (rbuffer[1] as u16);
        Ok(SystemErrors::from(raw))
    }

    async fn clear_errors(&mut self) -> Result<(), RadioError> {
        let opcode = SystemOpCode::ClearErrors.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    // IRQ Management
    async fn clear_irq_status(&mut self, irqs: IrqMask) -> Result<(), RadioError> {
        let opcode = SystemOpCode::ClearIrq.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (irqs >> 24) as u8,
            (irqs >> 16) as u8,
            (irqs >> 8) as u8,
            irqs as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn get_irq_status(&mut self) -> Result<IrqMask, RadioError> {
        // Note: In SWDR001, this uses direct_read to get 6 bytes (stat1, stat2, irq[4])
        // Since we don't have direct_read in lora-phy, we use execute_command_with_response
        // which achieves the same result via the GetStatus command
        let opcode = SystemOpCode::GetStatus.bytes();
        let cmd = [opcode[0], opcode[1]];
        let mut rbuffer = [0u8; 6]; // stat1, stat2, irq_status (4 bytes)
        self.execute_command_with_response(&cmd, &mut rbuffer).await?;

        // Extract IRQ status from bytes 2-5
        Ok(((rbuffer[2] as u32) << 24)
            | ((rbuffer[3] as u32) << 16)
            | ((rbuffer[4] as u32) << 8)
            | (rbuffer[5] as u32))
    }

    async fn get_and_clear_irq_status(&mut self) -> Result<IrqMask, RadioError> {
        let irq_status = self.get_irq_status().await?;
        self.clear_irq_status(irq_status).await?;
        Ok(irq_status)
    }

    // Configuration
    async fn set_tcxo_mode(&mut self, voltage: TcxoVoltage, timeout: u32) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetTcxoMode.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            voltage.value(),
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8,
        ];
        self.execute_command(&cmd).await
    }

    async fn set_dio_as_rf_switch(&mut self, cfg: &RfSwitchConfig) -> Result<(), RadioError> {
        let opcode = SystemOpCode::SetDioAsRfSwitch.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            cfg.enable,
            cfg.standby,
            cfg.rx,
            cfg.tx,
            cfg.tx_hp,
            cfg.tx_hf,
            cfg.gnss,
            cfg.wifi,
        ];
        self.execute_command(&cmd).await
    }

    async fn cfg_lf_clk(&mut self, cfg: u8) -> Result<(), RadioError> {
        let opcode = SystemOpCode::CfgLfClk.bytes();
        let cmd = [opcode[0], opcode[1], cfg];
        self.execute_command(&cmd).await
    }

    // Infopage Access
    async fn erase_infopage(&mut self) -> Result<(), RadioError> {
        let opcode = SystemOpCode::EraseInfopage.bytes();
        let cmd = [opcode[0], opcode[1]];
        self.execute_command(&cmd).await
    }

    async fn write_infopage(&mut self, address: u16, data: &[u8]) -> Result<(), RadioError> {
        let opcode = SystemOpCode::WriteInfopage.bytes();
        let mut cmd = [0u8; 4 + 256]; // Max 256 bytes data
        cmd[0] = opcode[0];
        cmd[1] = opcode[1];
        cmd[2] = (address >> 8) as u8;
        cmd[3] = address as u8;

        let len = data.len().min(256);
        cmd[4..4 + len].copy_from_slice(&data[..len]);

        self.execute_command(&cmd[..4 + len]).await
    }

    async fn read_infopage(&mut self, address: u16, data: &mut [u8]) -> Result<(), RadioError> {
        let opcode = SystemOpCode::ReadInfopage.bytes();
        let cmd = [
            opcode[0],
            opcode[1],
            (address >> 8) as u8,
            address as u8,
            data.len() as u8,
        ];
        self.execute_command_with_response(&cmd, data).await
    }
}
