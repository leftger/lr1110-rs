//! LR1110 WiFi Scanning with Advanced IRQ Management
//!
//! This example demonstrates WiFi passive scanning with advanced IRQ management
//! ported from STM32-Sidewalk-SDK:
//!
//! - **Two-stage priority IRQ handling**: Precise timestamp capture
//! - **Radio planner**: Coordinates WiFi with other radio operations
//! - **Pre/post-scan callbacks**: System coordination (Sidewalk/BLE suspension)
//! - **Result filtering**: Removes mobile APs, keeps only fixed APs
//! - **Result sorting**: Orders by RSSI (strongest first)
//!
//! ## WiFi Scan Process
//!
//! 1. Pre-scan callback → Suspend operations
//! 2. Configure IRQ for WIFI_SCAN_DONE
//! 3. Launch WiFi scan (~3-10 seconds)
//! 4. Wait for IRQ (handled by interrupt)
//! 5. Read results (MAC addresses, channels, RSSI)
//! 6. Filter mobile APs
//! 7. Sort by signal strength
//! 8. Post-scan callback → Resume operations
//!
//! ## Hardware connections for STM32WBA65RI:
//! - SPI2_SCK:  PB10
//! - SPI2_MISO: PA9
//! - SPI2_MOSI: PC3
//! - SPI2_NSS:  PD14 (manual control via GPIO)
//! - LR1110_RESET: PA4
//! - LR1110_BUSY:  PB13
//! - LR1110_DIO1:  PB14 (EXTI interrupt for WIFI_SCAN_DONE)
//!
//! ## Usage
//!
//! ```bash
//! cargo run --release --bin lr1110_wifi_scan_with_irq
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
use lr1110_rs::iv::Lr1110InterfaceVariant;
use lr1110_rs::system::{
    RfSwitchConfig, StandbyConfig, SystemExt, RFSW0_HIGH,
    RFSW1_HIGH, RFSW2_HIGH, RFSW3_HIGH,
};
use lr1110_rs::wifi::{
    WifiBasicMacTypeChannelResult, WifiExt, WifiScanMode, WifiSignalTypeScan,
    WIFI_ALL_CHANNELS_MASK,
};
use lr1110_rs::wifi_scan_manager::{WifiScanConfig, WifiScanManager};
use {defmt_rtt as _, panic_probe as _};

// ============================================================================
// WiFi Scan Coordination Callbacks
// ============================================================================

/// Pre-scan callback: Called BEFORE any WiFi radio access
///
/// Use this to suspend concurrent radio operations like Sidewalk, BLE, LoRa.
fn wifi_prescan_callback() {
    info!("[CALLBACK] WiFi pre-scan: Suspending concurrent operations");

    // In a real application with Sidewalk:
    // sidewalk_suspend();

    // In a real application with BLE:
    // ble_suspend();
}

/// Post-scan callback: Called AFTER scan completes
///
/// Use this to resume suspended operations.
fn wifi_postscan_callback() {
    info!("[CALLBACK] WiFi post-scan: Resuming concurrent operations");

    // In a real application with Sidewalk:
    // sidewalk_resume();

    // In a real application with BLE:
    // ble_resume();
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
    info!("LR1110 WiFi Scan with IRQ Management");
    info!("===========================================");

    // Configure SPI2 for LR1110
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(8_000_000);

    let spi = Spi::new(
        p.SPI2, p.PB10, // SCK
        p.PC3,  // MOSI
        p.PA9,  // MISO
        p.GPDMA1_CH0, p.GPDMA1_CH1, spi_config,
    );

    let nss = Output::new(p.PD14, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    // Configure LR1110 control pins
    let reset = Output::new(p.PA4, Level::High, Speed::Low);
    let busy = ExtiInput::new(p.PB13, p.EXTI13, Pull::None, Irqs);
    let dio1 = ExtiInput::new(p.PB14, p.EXTI14, Pull::Down, Irqs);

    let rf_switch_rx: Option<Output<'_>> = None;
    let rf_switch_tx: Option<Output<'_>> = None;

    let iv =
        Lr1110InterfaceVariant::new(reset, busy, dio1, rf_switch_rx, rf_switch_tx).unwrap();

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

    // Read WiFi firmware version
    match radio.wifi_read_version().await {
        Ok(version) => {
            info!("  WiFi firmware: 0x{:04X}", version);
        }
        Err(e) => error!("  Failed to read WiFi version: {:?}", e),
    }

    // Configure RF switches
    info!("Configuring RF switches for WiFi...");
    let rf_switch_cfg = RfSwitchConfig {
        enable: 0xFF,
        standby: 0x00,
        rx: RFSW0_HIGH,
        tx: RFSW0_HIGH | RFSW1_HIGH,
        tx_hp: RFSW1_HIGH,
        tx_hf: 0x00,
        gnss: RFSW2_HIGH,
        wifi: RFSW3_HIGH, // DIO8 controls WiFi LNA (if present)
    };
    SystemExt::set_dio_as_rf_switch(&mut radio, &rf_switch_cfg)
        .await
        .unwrap();

    // ========================================================================
    // Initialize WiFi Scan Manager
    // ========================================================================

    info!("-------------------------------------------");
    info!("Initializing WiFi Scan Manager");

    let mut scan_manager = WifiScanManager::new();

    // Register pre/post-scan callbacks
    scan_manager.set_prescan_callback(wifi_prescan_callback);
    scan_manager.set_postscan_callback(wifi_postscan_callback);

    // Configure WiFi scan
    let scan_config = WifiScanConfig {
        signal_type: WifiSignalTypeScan::TypeBGN, // Scan 802.11 b/g/n
        channels: WIFI_ALL_CHANNELS_MASK,         // All channels 1-14
        scan_mode: WifiScanMode::Beacon,          // Beacon-based scan
        max_results: 32,
        scans_per_channel: 1,
        timeout_per_scan_ms: 90,
        timeout_per_channel_ms: 100,
        abort_on_timeout: false,
        max_retries: 3,
        scan_timeout_ms: 10_000, // 10 seconds
        filter_mobile_aps: true, // Only keep fixed APs
        sort_by_rssi: true,      // Sort by signal strength
        max_results_to_send: 5,  // Top 5 APs for LoRa Cloud
        ..Default::default()
    };

    info!("  Signal type: 802.11 b/g/n");
    info!("  Channels: All (1-14)");
    info!("  Max results: {}", scan_config.max_results);
    info!("  Filter mobile APs: {}", scan_config.filter_mobile_aps);
    info!("  Sort by RSSI: {}", scan_config.sort_by_rssi);

    // ========================================================================
    // Main Scanning Loop
    // ========================================================================

    info!("===========================================");
    info!("Starting WiFi Scanning Loop");
    info!("===========================================");

    let mut scan_count = 0u32;

    loop {
        scan_count += 1;
        info!("");
        info!("-------------------------------------------");
        info!("WiFi Scan #{}", scan_count);

        // Perform WiFi scan using the scan manager
        match scan_manager.scan(&mut radio, &scan_config).await {
            Ok(result) => {
                info!("Scan Status: {:?}", result.status);
                info!("  Duration: {}ms", result.duration_ms);
                info!("  APs detected: {}", result.num_results);
                info!("  Timestamp: {:?}", result.timestamp);

                // Read and display WiFi results
                if result.num_results > 0 {
                    let mut results = [WifiBasicMacTypeChannelResult::default(); 32];
                    match radio
                        .wifi_read_basic_mac_type_channel_results(
                            &mut results,
                            0,
                            result.num_results,
                        )
                        .await
                    {
                        Ok(_) => {
                            info!("  Access Points:");
                            for i in 0..result.num_results.min(10) as usize {
                                let ap = &results[i];
                                info!(
                                    "    #{} MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}, Ch:{}, Type:{:?}, RSSI:{}dBm",
                                    i + 1,
                                    ap.mac_address[0],
                                    ap.mac_address[1],
                                    ap.mac_address[2],
                                    ap.mac_address[3],
                                    ap.mac_address[4],
                                    ap.mac_address[5],
                                    ap.channel(),  // Use method, not field
                                    ap.signal_type(),  // Use method, not field
                                    ap.rssi
                                );
                            }

                            if result.num_results > 10 {
                                info!("    ... and {} more", result.num_results - 10);
                            }

                            // Find strongest AP
                            let strongest_ap = results[0..result.num_results as usize]
                                .iter()
                                .max_by_key(|ap| ap.rssi);

                            if let Some(ap) = strongest_ap {
                                info!(
                                    "  Strongest: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} @ {}dBm (Ch:{})",
                                    ap.mac_address[0],
                                    ap.mac_address[1],
                                    ap.mac_address[2],
                                    ap.mac_address[3],
                                    ap.mac_address[4],
                                    ap.mac_address[5],
                                    ap.rssi,
                                    ap.channel() as u8  // Use method
                                );
                            }

                            // In a real application, you would:
                            // 1. Filter out mobile APs (keep origin == BEACON_FIX_AP)
                            // 2. Sort by RSSI (strongest first)
                            // 3. Take top N results (e.g., 5)
                            // 4. Send to LoRa Cloud for positioning
                        }
                        Err(e) => {
                            error!("  Failed to read WiFi results: {:?}", e);
                        }
                    }
                }

                // Show radio planner metrics
                info!("-------------------------------------------");
                info!("Performance Metrics:");
                let planner = scan_manager.radio_planner();
                info!("  Radio planner:");
                info!("    Pending tasks: {}", planner.pending_task_count());
                info!("    Current time: {}ms", planner.get_current_time_ms());
            }
            Err(e) => {
                error!("WiFi scan failed: {:?}", e);
                embassy_time::Timer::after_secs(5).await;
                continue;
            }
        }

        // Wait before next scan
        info!("-------------------------------------------");
        info!("Waiting 30 seconds before next scan...");

        // Enter standby to save power
        if let Err(e) = SystemExt::set_standby(&mut radio, StandbyConfig::RC).await {
            warn!("Failed to enter standby: {:?}", e);
        }

        embassy_time::Timer::after_secs(30).await;
    }
}
