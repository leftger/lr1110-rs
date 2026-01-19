//! LR1110 Ranging Demo Example
//!
//! This example demonstrates RTToF (Round-Trip Time of Flight) ranging between
//! two LR1110 devices with frequency hopping across 39 channels.
//!
//! ## Architecture
//!
//! This demo implements a two-phase ranging protocol:
//! 1. **LoRa Initialization Phase**: Devices exchange addresses and synchronize
//! 2. **RTToF Ranging Phase**: Frequency hopping across 39 channels to measure distance
//!
//! ## Modes
//!
//! - **Manager**: Initiates ranging requests and collects distance measurements
//! - **Subordinate**: Responds to ranging requests
//!
//! ## Features
//!
//! - Frequency hopping across 39 preconfigured channels
//! - Two-phase protocol: LoRa initialization + RTToF ranging
//! - Median distance calculation from multi-channel results
//! - Path loss exponent (Gamma) computation
//! - JSON formatted output
//! - Configurable SF/BW combinations
//!
//! ## Hardware Connections
//!
//! For STM32WBA65RI:
//! - SPI2_SCK:  PB10
//! - SPI2_MISO: PA9
//! - SPI2_MOSI: PC3
//! - SPI2_NSS:  PD14 (manual control via GPIO)
//! - LR1110_RESET: PA4 (LR1110_NRESET)
//! - LR1110_BUSY:  PB13 (BUSY signal)
//! - LR1110_DIO1:  PB14 (with EXTI interrupt)
//!
//! ## Usage
//!
//! Manager mode (default):
//! ```bash
//! cargo run --release --bin lr1110_ranging_demo
//! ```
//!
//! Subordinate mode:
//! ```bash
//! cargo run --release --bin lr1110_ranging_demo --features subordinate
//! ```
//!
//! ## Configuration
//!
//! Key parameters can be adjusted in the configuration section:
//! - RANGING_MODE: Manager or Subordinate
//! - LORA_SF: Spreading factor (SF5-SF12)
//! - LORA_BW: Bandwidth (125/250/500 kHz)
//! - TX_POWER: Output power (-17 to +22 dBm)
//! - RANGING_ADDRESS: 4-byte device address
//!
//! ## Output Format
//!
//! Results are output as JSON for easy parsing:
//! ```json
//! {
//!   "SF": "SF8",
//!   "BW": "BW500",
//!   "Address": "0x32101222",
//!   "LoRa RSSI": "-110 dBm",
//!   "RngResult": {
//!     "Num": 39,
//!     "Results": [...],
//!     "DistanceRng": "100.5 m",
//!     "FinalGamma": "2.5",
//!     "PER": "5 %"
//!   }
//! }
//! ```
//!
//! ## Implementation Status
//!
//! NOTE: This is a comprehensive skeleton/template for the ranging demo.
//! Full implementation requires the RangingExt trait to be fully implemented
//! with low-level SPI access from lora-phy. The structure and logic flow
//! are documented here for reference.
//!
//! To complete this example:
//! 1. Implement the RangingExt trait in src/ranging.rs with full SPI access
//! 2. Uncomment and complete the state machine methods below
//! 3. Implement the result processing and statistics calculations
//! 4. Test on hardware with two LR1110 devices

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::rcc::{
    AHB5Prescaler, AHBPrescaler, APBPrescaler, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::lr1110::variant::Lr1110 as Lr1110Chip;
use lora_phy::lr1110::{self as lr1110_module, TcxoCtrlVoltage};
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_traits::RadioKind;
use lr1110_rs::iv::Lr1110InterfaceVariant;
use lr1110_rs::ranging::{
    ranging_channels, ranging_config, lora_bw, lora_sf, lora_cr, packet_type,
    calculate_ranging_request_delay_ms, RangingExt,
};
use {defmt_rtt as _, panic_probe as _};

// Bind EXTI interrupts
bind_interrupts!(struct Irqs {
    EXTI13 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI13>;
    EXTI14 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI14>;
});

// =============================================================================
// Configuration Constants
// =============================================================================

/// Ranging mode: true = Manager, false = Subordinate
#[cfg(not(feature = "subordinate"))]
const IS_MANAGER: bool = true;
#[cfg(feature = "subordinate")]
const IS_MANAGER: bool = false;

/// LoRa spreading factor (SF5-SF12)
const LORA_SF: u8 = lora_sf::SF8;

/// LoRa bandwidth
const LORA_BW: u8 = lora_bw::BW_500;

/// LoRa coding rate
const LORA_CR: u8 = lora_cr::CR_4_5;

/// TX power in dBm
const TX_POWER: i8 = 13;

/// Ranging device address
const RANGING_ADDRESS: u32 = ranging_config::DEFAULT_ADDRESS;

/// Preamble length in symbols
const PREAMBLE_LENGTH: u16 = 12;

/// Use US915 frequency hopping channels (39 channels)
const RANGING_CHANNELS: &[u32] = &ranging_channels::US915;

// =============================================================================
// Ranging State Machine
// =============================================================================

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
enum RangingState {
    Idle,
    LoRaInit,
    LoRaWaitRx,
    LoRaWaitTx,
    RangingStart,
    RangingWaitDone,
    NextChannel,
    Complete,
}

/// Single ranging measurement result
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
struct RangingResult {
    frequency: u32,
    distance_m: i32,
    raw_distance: u32,
    rssi: i8,
    valid: bool,
}

/// Complete ranging session state
struct RangingSession {
    state: RangingState,
    channel_index: usize,
    results: [RangingResult; ranging_config::MAX_HOPPING_CHANNELS],
    result_count: usize,
    manager_rssi: i8,
    subordinate_rssi: i8,
}

impl RangingSession {
    fn new() -> Self {
        Self {
            state: RangingState::Idle,
            channel_index: 0,
            results: [RangingResult::default(); ranging_config::MAX_HOPPING_CHANNELS],
            result_count: 0,
            manager_rssi: 0,
            subordinate_rssi: 0,
        }
    }

    fn reset(&mut self) {
        self.state = RangingState::Idle;
        self.channel_index = 0;
        self.result_count = 0;
        self.manager_rssi = 0;
        self.subordinate_rssi = 0;
        for result in &mut self.results {
            *result = RangingResult::default();
        }
    }

    /// Calculate median distance from valid results
    fn calculate_median_distance(&self) -> Option<i32> {
        // Collect valid distances
        let mut distances: heapless::Vec<i32, 39> = heapless::Vec::new();
        for i in 0..self.result_count {
            if self.results[i].valid {
                let _ = distances.push(self.results[i].distance_m);
            }
        }

        if distances.is_empty() {
            return None;
        }

        // Sort distances
        distances.sort_unstable();

        // Return median
        Some(distances[distances.len() / 2])
    }

    /// Calculate Packet Error Rate (PER)
    fn calculate_per(&self) -> f32 {
        if self.result_count == 0 {
            return 0.0;
        }

        let invalid_count = self.results[..self.result_count]
            .iter()
            .filter(|r| !r.valid)
            .count();

        (invalid_count as f32 / self.result_count as f32) * 100.0
    }

    /// Calculate path loss exponent (Gamma) from ranging results
    ///
    /// This uses the formula from app_pathloss.c:
    /// Gamma = (RSSI_avg - RSSI_1m) / (10 * log10(distance))
    fn calculate_gamma(&self) -> Option<f32> {
        // Collect valid results with distance > 0
        let mut valid_results: heapless::Vec<(i32, i8), 39> = heapless::Vec::new();
        for i in 0..self.result_count {
            if self.results[i].valid && self.results[i].distance_m > 0 {
                let _ = valid_results.push((self.results[i].distance_m, self.results[i].rssi));
            }
        }

        if valid_results.len() < 2 {
            return None;
        }

        // Calculate average RSSI
        let avg_rssi: f32 = valid_results
            .iter()
            .map(|(_, rssi)| *rssi as f32)
            .sum::<f32>()
            / valid_results.len() as f32;

        // Reference RSSI at 1 meter (typical value for LoRa)
        const RSSI_1M: f32 = -50.0;

        // Calculate Gamma using the median distance
        if let Some(median_distance) = self.calculate_median_distance() {
            if median_distance > 0 {
                let distance_f = median_distance as f32;
                let log_distance = libm::log10f(distance_f);
                if log_distance > 0.0 {
                    let gamma = (avg_rssi - RSSI_1M) / (10.0 * log_distance);
                    return Some(gamma);
                }
            }
        }

        None
    }

    /// Output results as JSON
    fn output_results(&self) {
        info!("{{");
        info!("  \"SF\": \"SF{}\",", LORA_SF);
        info!("  \"BW\": \"BW{}\",",
            match LORA_BW {
                lora_bw::BW_125 => 125,
                lora_bw::BW_250 => 250,
                lora_bw::BW_500 => 500,
                _ => 500,
            }
        );
        info!("  \"Address\": \"0x{:08X}\",", RANGING_ADDRESS);
        info!("  \"LoRa RSSI\": \"{} dBm\",", self.manager_rssi);
        info!("  \"RngResult\": {{");
        info!("    \"Num\": {},", self.result_count);
        info!("    \"Results\": [");

        for i in 0..self.result_count {
            let result = &self.results[i];
            let comma = if i < self.result_count - 1 { "," } else { "" };

            if result.valid {
                info!("      {{ \"freq\": {}, \"dist_m\": {}, \"rssi\": {} }}{}",
                    result.frequency, result.distance_m, result.rssi, comma);
            } else {
                info!("      {{ \"freq\": {}, \"error\": true }}{}",
                    result.frequency, comma);
            }
        }

        info!("    ],");

        if let Some(median) = self.calculate_median_distance() {
            info!("    \"DistanceRng\": \"{} m\",", median);
        } else {
            info!("    \"DistanceRng\": \"N/A\",");
        }

        if let Some(gamma) = self.calculate_gamma() {
            // Convert to integer with 2 decimal places for display
            let gamma_int = (gamma * 100.0) as i32;
            info!("    \"FinalGamma\": \"{}.{:02}\",", gamma_int / 100, gamma_int.abs() % 100);
        } else {
            info!("    \"FinalGamma\": \"N/A\",");
        }

        let per = self.calculate_per();
        let per_int = per as i32;
        info!("    \"PER\": \"{} %\"", per_int);

        info!("  }}");
        info!("}}");
    }
}

// =============================================================================
// Main Application
// =============================================================================

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize STM32WBA65RI peripherals
    let mut config = Config::default();

    // Configure PLL1 for 96 MHz system clock
    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL30,
        divr: Some(PllDiv::DIV5),
        divq: None,
        divp: Some(PllDiv::DIV30),
        frac: Some(0),
    });

    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.apb7_pre = APBPrescaler::DIV1;
    config.rcc.ahb5_pre = AHB5Prescaler::DIV4;
    config.rcc.voltage_scale = VoltageScale::RANGE1;
    config.rcc.sys = Sysclk::PLL1_R;

    let p = embassy_stm32::init(config);

    info!("==============================================");
    info!("LR1110 Ranging Demo");
    if IS_MANAGER {
        info!("Mode: MANAGER");
    } else {
        info!("Mode: SUBORDINATE");
    }
    info!("==============================================");

    // Configure SPI2 for LR1110
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(8_000_000);

    let spi = Spi::new(
        p.SPI2,
        p.PB10, // SCK
        p.PC3,  // MOSI
        p.PA9,  // MISO
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        spi_config,
    );

    let nss = Output::new(p.PD14, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    // Configure LR1110 control pins
    let reset = Output::new(p.PA4, Level::High, Speed::Low);
    let busy = ExtiInput::new(p.PB13, p.EXTI13, Pull::None, Irqs);
    let dio1 = ExtiInput::new(p.PB14, p.EXTI14, Pull::Down, Irqs);

    let rf_switch_rx: Option<Output<'_>> = None;
    let rf_switch_tx: Option<Output<'_>> = None;

    let iv = Lr1110InterfaceVariant::new(reset, busy, dio1, rf_switch_rx, rf_switch_tx).unwrap();

    let lr_config = lr1110_module::Config {
        chip: Lr1110Chip::new(),
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl3V0),
        use_dcdc: true,
        rx_boost: false,
    };

    let mut radio = lr1110_module::Lr1110::new(spi_device, iv, lr_config);

    info!("Initializing LR1110...");
    radio.reset(&mut Delay).await.unwrap();
    Timer::after_millis(100).await;

    // Create ranging session
    let mut session = RangingSession::new();

    info!("Configuration:");
    info!("  SF: {}", LORA_SF);
    info!("  BW: {} kHz", match LORA_BW {
        lora_bw::BW_125 => 125,
        lora_bw::BW_250 => 250,
        lora_bw::BW_500 => 500,
        _ => 500,
    });
    info!("  Address: 0x{:08X}", RANGING_ADDRESS);
    info!("  Channels: {}", RANGING_CHANNELS.len());
    info!("==============================================");

    // TODO: Implement ranging state machine
    //
    // The full implementation requires:
    // 1. LoRa initialization phase (exchange addresses)
    // 2. RTToF ranging phase (frequency hopping)
    // 3. Result collection and processing
    //
    // Example state machine flow:
    //
    // loop {
    //     match session.state {
    //         RangingState::Idle => {
    //             // Initialize ranging session
    //             session.state = RangingState::LoRaInit;
    //         }
    //         RangingState::LoRaInit => {
    //             // Phase 1: LoRa initialization
    //             if IS_MANAGER {
    //                 // Manager: TX address packet
    //                 // radio.set_packet_type(packet_type::LORA).await?;
    //                 // radio.set_lora_mod_params(LORA_SF, LORA_BW, LORA_CR, 0).await?;
    //                 // radio.write_tx_buffer(0, &address_payload).await?;
    //                 // radio.set_tx_mode(timeout).await?;
    //                 // session.state = RangingState::LoRaWaitTx;
    //             } else {
    //                 // Subordinate: RX and respond
    //                 // radio.set_packet_type(packet_type::LORA).await?;
    //                 // radio.set_lora_mod_params(LORA_SF, LORA_BW, LORA_CR, 0).await?;
    //                 // radio.set_rx_mode(ranging_config::RX_CONTINUOUS).await?;
    //                 // session.state = RangingState::LoRaWaitRx;
    //             }
    //         }
    //         RangingState::RangingStart => {
    //             // Phase 2: RTToF ranging with frequency hopping
    //             // for each channel in RANGING_CHANNELS {
    //             //     radio.set_packet_type(packet_type::RTTOF).await?;
    //             //     radio.set_rf_frequency(channel).await?;
    //             //     radio.rttof_set_address(RANGING_ADDRESS, 4).await?;
    //             //     radio.rttof_set_parameters(ranging_config::RESPONSE_SYMBOLS_COUNT).await?;
    //             //
    //             //     if IS_MANAGER {
    //             //         radio.set_tx_mode(timeout).await?;
    //             //         // Wait for ranging done IRQ
    //             //         let result = radio.rttof_get_distance_result(Bandwidth::_500KHz).await?;
    //             //         session.results[channel_index] = RangingResult {
    //             //             frequency: channel,
    //             //             distance_m: result.distance_m,
    //             //             rssi: result.rssi_dbm,
    //             //             valid: true,
    //             //             ..Default::default()
    //             //         };
    //             //     } else {
    //             //         radio.set_rx_mode(timeout).await?;
    //             //         // Wait for ranging request and respond
    //             //     }
    //             //
    //             //     Timer::after_millis(ranging_config::DONE_PROCESSING_TIME_MS as u64).await;
    //             // }
    //             // session.state = RangingState::Complete;
    //         }
    //         RangingState::Complete => {
    //             // Output results
    //             session.output_results();
    //             Timer::after_secs(5).await;
    //             session.reset();
    //         }
    //         _ => {
    //             Timer::after_millis(10).await;
    //         }
    //     }
    // }

    warn!("Ranging demo skeleton - full implementation requires completed RangingExt trait");
    warn!("See src/ranging.rs for trait definition and examples/README.md for status");

    // For now, just output a sample result structure
    session.result_count = 3;
    session.results[0] = RangingResult {
        frequency: RANGING_CHANNELS[0],
        distance_m: 100,
        rssi: -85,
        valid: true,
        ..Default::default()
    };
    session.results[1] = RangingResult {
        frequency: RANGING_CHANNELS[1],
        distance_m: 105,
        rssi: -87,
        valid: true,
        ..Default::default()
    };
    session.results[2] = RangingResult {
        frequency: RANGING_CHANNELS[2],
        distance_m: 102,
        rssi: -86,
        valid: true,
        ..Default::default()
    };

    Timer::after_secs(2).await;
    info!("\nSample output format:");
    session.output_results();

    info!("\n==============================================");
    info!("Demo complete - see implementation notes above");
    info!("==============================================");

    loop {
        Timer::after_secs(10).await;
    }
}
