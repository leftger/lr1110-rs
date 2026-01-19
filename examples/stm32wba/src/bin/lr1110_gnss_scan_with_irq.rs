//! LR1110 GNSS Scanning with Advanced IRQ Management
//!
//! This example demonstrates the advanced IRQ management features ported from
//! STM32-Sidewalk-SDK:
//!
//! - **Two-stage priority IRQ handling**: Precise timestamp capture at high priority,
//!   processing at low priority
//! - **Radio planner**: Coordinates concurrent radio operations
//! - **Pre/post-scan callbacks**: System coordination (Sidewalk/BLE suspension)
//! - **Mobile vs Static modes**: Optimized scanning patterns
//! - **Conditional post-scan**: Skips callbacks between back-to-back scans
//!
//! ## Scan Modes
//!
//! ### Mobile Mode (Back-to-Back)
//! - Zero delay between scans
//! - Post-scan callbacks SKIPPED between scans for efficiency
//! - Optimized for moving devices
//!
//! ### Static Mode (Periodic)
//! - Delay between scans (e.g., 15 seconds)
//! - Post-scan callbacks EXECUTED after each scan
//! - Allows other operations during delay
//!
//! ## Hardware connections for STM32WBA65RI:
//! - SPI2_SCK:  PB10
//! - SPI2_MISO: PA9
//! - SPI2_MOSI: PC3
//! - SPI2_NSS:  PD14 (manual control via GPIO)
//! - LR1110_RESET: PA4
//! - LR1110_BUSY:  PB13
//! - LR1110_DIO1:  PB14 (EXTI interrupt for GNSS_SCAN_DONE)
//!
//! ## Configuration
//!
//! Set location and scan mode via environment variables:
//! ```bash
//! GNSS_LAT=33.4942 GNSS_LON=-111.9261 SCAN_MODE=mobile cargo run --release --bin lr1110_gnss_scan_with_irq
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::rcc::{
    AHB5Prescaler, AHBPrescaler, APBPrescaler, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::lr1110::variant::Lr1110 as Lr1110Chip;
use lora_phy::lr1110::{self as lr1110_module, TcxoCtrlVoltage};
use lora_phy::mod_traits::RadioKind;
use lr1110_rs::gnss::{
    GnssAssistancePosition, GnssDetectedSatellite, GnssExt, GnssSearchMode, GNSS_BEIDOU_MASK,
    GNSS_GPS_MASK,
};
use lr1110_rs::gnss_scan_manager::{GnssScanManager, ScanConfig, ScanMode};
use lr1110_rs::iv::Lr1110InterfaceVariant;
use lr1110_rs::system::{
    RfSwitchConfig, StandbyConfig, SystemExt, IRQ_GNSS_SCAN_DONE, IRQ_NONE, RFSW0_HIGH, RFSW1_HIGH,
    RFSW2_HIGH, RFSW3_HIGH,
};
use {defmt_rtt as _, panic_probe as _};

// ============================================================================
// CONFIGURATION
// ============================================================================

include!(concat!(env!("OUT_DIR"), "/gnss_location.rs"));

/// Scan mode selection
#[derive(Clone, Copy, PartialEq)]
enum ExampleScanMode {
    /// Mobile mode: back-to-back scans, optimized for moving devices
    Mobile,
    /// Static mode: periodic scans with delays
    Static,
}

/// Select scan mode (change this or use environment variable)
const SELECTED_SCAN_MODE: ExampleScanMode = ExampleScanMode::Mobile;

/// Mobile mode configuration
const MOBILE_SCAN_GROUP_SIZE: u8 = 2;
const MOBILE_SCAN_DELAY_S: u32 = 0; // Zero delay = skip post-scan optimization

/// Static mode configuration
const STATIC_SCAN_GROUP_SIZE: u8 = 2;
const STATIC_SCAN_DELAY_S: u32 = 15; // 15 second delay between scans

/// Helper to convert satellite ID to PRN
fn sv_id_to_prn(sv_id: u8) -> u8 {
    if sv_id < 32 {
        sv_id + 1 // GPS: sv_id 0-31 -> PRN 1-32
    } else if sv_id >= 64 && sv_id < 128 {
        sv_id - 64 + 1 // BeiDou: sv_id 64-127 -> PRN 1-64
    } else {
        sv_id
    }
}

/// Helper to determine constellation from satellite ID
fn constellation_name(sv_id: u8) -> &'static str {
    if sv_id < 32 {
        "GPS"
    } else if sv_id >= 64 && sv_id < 128 {
        "BeiDou"
    } else {
        "Unknown"
    }
}

// ============================================================================
// Pre/Post-Scan Callbacks (System Coordination)
// ============================================================================

/// Pre-scan callback - called BEFORE any radio access
///
/// Use this to suspend concurrent radio operations like Sidewalk, BLE, etc.
/// This is called BEFORE the GNSS scan starts to allow proper coordination.
fn gnss_prescan_actions() {
    info!("[CALLBACK] Pre-scan: Suspending concurrent operations");

    // In a real application with Sidewalk:
    // sidewalk_suspend();

    // In a real application with BLE:
    // ble_suspend();

    // Example: You might also want to:
    // - Disable LoRa TX/RX
    // - Stop periodic timers
    // - Flush any pending operations
}

/// Post-scan callback - called AFTER scan completes and radio is idle
///
/// Use this to resume suspended operations. Note: In mobile mode with zero delay,
/// this is SKIPPED between scans for efficiency!
fn gnss_postscan_actions() {
    info!("[CALLBACK] Post-scan: Resuming concurrent operations");

    // In a real application with Sidewalk:
    // sidewalk_resume();

    // In a real application with BLE:
    // ble_resume();

    // Example: You might also want to:
    // - Re-enable LoRa operations
    // - Restart periodic timers
    // - Process queued operations
}

// ============================================================================

bind_interrupts!(struct Irqs {
    EXTI13 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI13>;
    EXTI14 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI14>;
});

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

    info!("===========================================");
    info!("LR1110 GNSS with Advanced IRQ Management");
    info!("===========================================");

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
    embassy_time::Timer::after_millis(100).await;

    info!("Configuring TCXO and calibrating...");
    radio.init_system().await.unwrap();

    // Display chip information
    info!("-------------------------------------------");
    info!("Chip Information:");
    match radio.get_version().await {
        Ok(version) => {
            info!("  Hardware: 0x{:02X}", version.hw);
            info!("  Firmware: 0x{:04X}", version.fw);
        }
        Err(e) => error!("  Failed to read version: {:?}", e),
    }

    // Configure RF switches for GNSS
    info!("Configuring RF switches for GNSS LNA...");
    let rf_switch_cfg = RfSwitchConfig {
        enable: 0xFF,
        standby: 0x00,
        rx: RFSW0_HIGH,
        tx: RFSW0_HIGH | RFSW1_HIGH,
        tx_hp: RFSW1_HIGH,
        tx_hf: 0x00,
        gnss: RFSW2_HIGH, // DIO7 controls BGA524N6 LNA
        wifi: RFSW3_HIGH,
    };
    SystemExt::set_dio_as_rf_switch(&mut radio, &rf_switch_cfg)
        .await
        .unwrap();

    // Configure GNSS assistance position
    let assistance_position = GnssAssistancePosition {
        latitude: GNSS_LATITUDE,
        longitude: GNSS_LONGITUDE,
    };

    if let Err(e) = radio
        .gnss_set_assistance_position(&assistance_position)
        .await
    {
        error!("  Failed to set assistance position: {:?}", e);
    } else {
        info!(
            "  Set assistance position: lat={}, lon={}",
            assistance_position.latitude, assistance_position.longitude
        );
    }

    // ========================================================================
    // Initialize GNSS Scan Manager with IRQ management
    // ========================================================================

    info!("-------------------------------------------");
    info!("Initializing GNSS Scan Manager");
    info!("-------------------------------------------");

    let mut scan_manager = GnssScanManager::new();

    // Register pre/post-scan callbacks for system coordination
    scan_manager.set_prescan_callback(gnss_prescan_actions);
    scan_manager.set_postscan_callback(gnss_postscan_actions);

    // Configure scan based on selected mode
    let (scan_mode, mode_name) = match SELECTED_SCAN_MODE {
        ExampleScanMode::Mobile => {
            info!("Scan Mode: MOBILE (back-to-back scans)");
            info!("  Scan group size: {}", MOBILE_SCAN_GROUP_SIZE);
            info!("  Delay between scans: {}s", MOBILE_SCAN_DELAY_S);
            info!("  Optimization: Post-scan callbacks SKIPPED between scans");
            (
                ScanMode::Mobile {
                    scan_group_size: MOBILE_SCAN_GROUP_SIZE,
                    scan_group_delay_s: MOBILE_SCAN_DELAY_S,
                },
                "Mobile",
            )
        }
        ExampleScanMode::Static => {
            info!("Scan Mode: STATIC (periodic scans)");
            info!("  Scan group size: {}", STATIC_SCAN_GROUP_SIZE);
            info!("  Delay between scans: {}s", STATIC_SCAN_DELAY_S);
            info!("  Post-scan callbacks EXECUTED after each scan");
            (
                ScanMode::Static {
                    scan_group_size: STATIC_SCAN_GROUP_SIZE,
                    scan_group_delay_s: STATIC_SCAN_DELAY_S,
                },
                "Static",
            )
        }
    };

    let scan_config = ScanConfig {
        constellation_mask: GNSS_GPS_MASK | GNSS_BEIDOU_MASK,
        search_mode: GnssSearchMode::MidEffort,
        mode: scan_mode,
        aggregate_results: false,
        max_retries: 3,
        scan_timeout_ms: 20_000, // 20 seconds for assisted scan
    };

    info!("-------------------------------------------");
    info!("Starting GNSS Scanning Loop");
    info!("  Mode: {}", mode_name);
    info!("  Constellation: GPS + BeiDou");
    info!("  Search effort: Mid");
    info!("-------------------------------------------");

    let mut scan_count = 0u32;

    loop {
        scan_count += 1;

        info!("");
        info!("===========================================");
        info!("GNSS Scan #{}", scan_count);
        info!("===========================================");

        // Check if this is a new scan group
        if !scan_manager.is_scan_group_active() {
            info!(
                "Starting new scan group (token: 0x{:02X})",
                scan_manager.get_token()
            );
        } else {
            info!(
                "Continuing scan group ({}/{} complete)",
                scan_manager.valid_scan_count(),
                match scan_mode {
                    ScanMode::Mobile {
                        scan_group_size, ..
                    } => scan_group_size,
                    ScanMode::Static {
                        scan_group_size, ..
                    } => scan_group_size,
                }
            );
        }

        // Perform GNSS scan using the scan manager
        // This handles:
        // - IRQ setup and management
        // - Radio planner coordination
        // - Pre/post-scan callbacks
        // - Conditional post-scan execution
        match scan_manager.scan(&mut radio, &scan_config).await {
            Ok(result) => {
                info!("Scan Status: {:?}", result.status);
                info!("  Duration: {}ms", result.duration_ms);
                info!("  Satellites detected: {}", result.num_satellites);
                info!("  Timestamp: {:?}", result.timestamp);

                // Read and display satellite details
                if result.num_satellites > 0 {
                    let mut satellites = [GnssDetectedSatellite::default(); 32];
                    match radio
                        .gnss_get_satellites(&mut satellites, result.num_satellites)
                        .await
                    {
                        Ok(count) => {
                            info!("  Satellite details:");
                            for i in 0..count.min(10) as usize {
                                // Show first 10
                                let sv = &satellites[i];
                                let constellation = constellation_name(sv.satellite_id);
                                let prn = sv_id_to_prn(sv.satellite_id);
                                info!(
                                    "    {} PRN {}: C/N={}dB, Doppler={}Hz",
                                    constellation, prn, sv.cnr, sv.doppler
                                );
                            }

                            // Summary by constellation
                            let gps_count = satellites[..count as usize]
                                .iter()
                                .filter(|s| s.satellite_id < 32)
                                .count();
                            let beidou_count = satellites[..count as usize]
                                .iter()
                                .filter(|s| s.satellite_id >= 64 && s.satellite_id < 128)
                                .count();
                            info!(
                                "  Summary: {} GPS, {} BeiDou satellites",
                                gps_count, beidou_count
                            );
                        }
                        Err(e) => {
                            error!("  Failed to get satellite details: {:?}", e);
                        }
                    }
                }

                // Show performance metrics
                info!("-------------------------------------------");
                info!("Performance Metrics:");
                info!("  Scan manager state:");
                info!("    Current token: 0x{:02X}", scan_manager.get_token());
                info!(
                    "    Valid scans in group: {}",
                    scan_manager.valid_scan_count()
                );
                info!(
                    "    Scan group active: {}",
                    scan_manager.is_scan_group_active()
                );

                // Show radio planner state
                let planner = scan_manager.radio_planner();
                info!("  Radio planner:");
                info!("    Pending tasks: {}", planner.pending_task_count());
                info!("    Current time: {}ms", planner.get_current_time_ms());
            }
            Err(e) => {
                error!("Scan failed: {:?}", e);
                embassy_time::Timer::after_secs(5).await;
                continue;
            }
        }

        // Add delay if in static mode and not at end of group
        if matches!(scan_mode, ScanMode::Static { .. }) && scan_manager.is_scan_group_active() {
            info!("-------------------------------------------");
            info!("Waiting {}s before next scan...", STATIC_SCAN_DELAY_S);

            // Enter standby to save power
            if let Err(e) = SystemExt::set_standby(&mut radio, StandbyConfig::RC).await {
                warn!("Failed to enter standby: {:?}", e);
            }

            embassy_time::Timer::after_secs(STATIC_SCAN_DELAY_S as u64).await;
        } else if !scan_manager.is_scan_group_active() {
            // End of scan group - add longer delay before next group
            info!("-------------------------------------------");
            info!("Scan group complete! Waiting 30s before next group...");

            if let Err(e) = SystemExt::set_standby(&mut radio, StandbyConfig::RC).await {
                warn!("Failed to enter standby: {:?}", e);
            }

            embassy_time::Timer::after_secs(30).await;
        }
    }
}
