//! LR1110 Almanac Management Example
//!
//! This example demonstrates comprehensive almanac management with age tracking:
//!
//! - **Almanac Status Monitoring**: Check freshness for all satellites
//! - **Age Tracking**: Monitor almanac age per satellite (0-15 days)
//! - **Update Strategy**: Automatic determination (full, GPS only, BeiDou only)
//! - **Scheduled Updates**: Periodic almanac refresh
//! - **Coverage Analysis**: Calculate almanac completion percentage
//! - **Diagnostic Display**: Show sample satellites with age information
//!
//! ## Almanac Basics
//!
//! The LR1110 uses almanac data to predict satellite visibility and accelerate
//! position fixes. Almanac freshness directly impacts:
//! - Scan duration (fresh = faster)
//! - Power consumption (fresh = lower)
//! - Position accuracy (fresh = better)
//!
//! ## Age Categories
//!
//! - **0 days**: Fresh - optimal performance
//! - **1-7 days**: Stale - reduced performance
//! - **8-14 days**: Very old - poor performance
//! - **15**: No data - autonomous mode only
//!
//! ## Update Process
//!
//! GPS broadcasts almanac in 25 subframes, each taking ~30 seconds.
//! Full update requires capturing all subframes over ~12.5 minutes.
//!
//! ## Hardware connections for STM32WBA65RI:
//! - SPI2_SCK:  PB10
//! - SPI2_MISO: PA9
//! - SPI2_MOSI: PC3
//! - SPI2_NSS:  PD14
//! - LR1110_RESET: PA4
//! - LR1110_BUSY:  PB13
//! - LR1110_DIO1:  PB14
//!
//! ## Usage
//!
//! ```bash
//! GNSS_LAT=33.4942 GNSS_LON=-111.9261 cargo run --release --bin lr1110_almanac_manager
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
use embassy_time::{Delay, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::lr1110::variant::Lr1110 as Lr1110Chip;
use lora_phy::lr1110::{self as lr1110_module, TcxoCtrlVoltage};
use lora_phy::mod_traits::RadioKind;
use lr1110_rs::almanac::{AlmanacManager, UpdateStrategy, ALMANAC_FULL_UPDATE_DURATION_S};
use lr1110_rs::gnss::{
    GnssAssistancePosition, GnssExt, GnssSearchMode, GNSS_BEIDOU_MASK, GNSS_GPS_MASK,
};
use lr1110_rs::iv::Lr1110InterfaceVariant;
use lr1110_rs::system::{
    RfSwitchConfig, StandbyConfig, SystemExt, RFSW0_HIGH, RFSW1_HIGH, RFSW2_HIGH, RFSW3_HIGH,
};
use {defmt_rtt as _, panic_probe as _};

// ============================================================================
// Configuration
// ============================================================================

include!(concat!(env!("OUT_DIR"), "/gnss_location.rs"));

/// Enable automatic almanac updates
const AUTO_UPDATE_ENABLED: bool = true;

/// Sample count for status display
const SAMPLE_SATELLITES: u8 = 6;

/// Update check interval when fully updated (seconds)
const CHECK_INTERVAL_UPDATED_S: u64 = 60; // 1 minute for demo (normally 8 hours)

/// Update check interval when needs update (seconds)
const CHECK_INTERVAL_NEEDS_UPDATE_S: u64 = 30; // 30 seconds

// ============================================================================

bind_interrupts!(struct Irqs {
    EXTI13 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI13>;
    EXTI14 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI14>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize STM32WBA65RI peripherals
    let mut config = Config::default();

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
    info!("LR1110 Almanac Management Example");
    info!("===========================================");

    // Configure SPI and pins
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(8_000_000);

    let spi = Spi::new(
        p.SPI2,
        p.PB10,
        p.PC3,
        p.PA9,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        spi_config,
    );

    let nss = Output::new(p.PD14, Level::High, Speed::VeryHigh);
    let spi_device = ExclusiveDevice::new(spi, nss, Delay).unwrap();

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

    // Initialize radio
    info!("Initializing LR1110...");
    radio.reset(&mut Delay).await.unwrap();
    embassy_time::Timer::after_millis(100).await;
    radio.init_system().await.unwrap();

    // Display chip info
    match radio.get_version().await {
        Ok(version) => {
            info!("Chip: HW=0x{:02X}, FW=0x{:04X}", version.hw, version.fw);
        }
        Err(e) => error!("Failed to read version: {:?}", e),
    }

    // Configure RF switches for GNSS
    let rf_switch_cfg = RfSwitchConfig {
        enable: 0xFF,
        standby: 0x00,
        rx: RFSW0_HIGH,
        tx: RFSW0_HIGH | RFSW1_HIGH,
        tx_hp: RFSW1_HIGH,
        tx_hf: 0x00,
        gnss: RFSW2_HIGH,
        wifi: RFSW3_HIGH,
    };
    SystemExt::set_dio_as_rf_switch(&mut radio, &rf_switch_cfg)
        .await
        .unwrap();

    // Set assistance position
    let assistance_position = GnssAssistancePosition {
        latitude: GNSS_LATITUDE,
        longitude: GNSS_LONGITUDE,
    };
    radio
        .gnss_set_assistance_position(&assistance_position)
        .await
        .unwrap();
    info!(
        "Assistance position: lat={}, lon={}",
        GNSS_LATITUDE, GNSS_LONGITUDE
    );

    // ========================================================================
    // Initialize Almanac Manager
    // ========================================================================

    info!("===========================================");
    info!("Almanac Manager");
    info!("===========================================");

    let mut almanac_mgr = AlmanacManager::new();

    // ========================================================================
    // Main Almanac Monitoring Loop
    // ========================================================================

    let mut cycle_count = 0u32;

    loop {
        cycle_count += 1;
        info!("");
        info!("-------------------------------------------");
        info!("Almanac Check Cycle #{}", cycle_count);
        info!("-------------------------------------------");

        // Check comprehensive almanac status
        info!("Checking almanac status (sampling satellites)...");
        let status = match almanac_mgr.check_status(&mut radio, true).await {
            Ok(s) => s,
            Err(e) => {
                error!("Failed to check status: {:?}", e);
                embassy_time::Timer::after_secs(30).await;
                continue;
            }
        };

        // Display overall status
        info!("Overall Status:");
        info!("  Global CRC: 0x{:08X}", status.global_crc);
        info!("  Firmware: 0x{:02X}", status.firmware_version);
        info!("  Update needed: {}", status.needs_update());
        info!("  Overall completion: {}%", status.overall_completion());

        // Display GPS status
        info!("");
        info!("GPS Constellation:");
        info!("  Total satellites: {}", status.gps.total_count);
        info!(
            "  Fresh: {} ({}%)",
            status.gps.fresh_count,
            status.gps.completion_percentage()
        );
        info!("  Stale: {}", status.gps.stale_count);
        info!("  No data: {}", status.gps.no_data_count);
        info!("  Average age: {} days", status.gps.average_age_days);
        info!("  Needs update: {}", status.gps.needs_update);
        info!("  Good coverage: {}", status.gps.has_good_coverage());

        // Display BeiDou status
        info!("");
        info!("BeiDou Constellation:");
        info!("  Total satellites: {}", status.beidou.total_count);
        info!(
            "  Fresh: {} ({}%)",
            status.beidou.fresh_count,
            status.beidou.completion_percentage()
        );
        info!("  Stale: {}", status.beidou.stale_count);
        info!("  No data: {}", status.beidou.no_data_count);
        info!("  Average age: {} days", status.beidou.average_age_days);
        info!("  Needs update: {}", status.beidou.needs_update);
        info!("  Good coverage: {}", status.beidou.has_good_coverage());

        // Display sample satellites
        info!("");
        info!("Sample Satellites:");
        match almanac_mgr
            .get_sample_status(&mut radio, SAMPLE_SATELLITES)
            .await
        {
            Ok(samples) => {
                for sat in samples.iter() {
                    let constellation = lr1110_rs::almanac::constellation_name(sat.satellite_id);
                    let prn = lr1110_rs::almanac::sv_id_to_prn(sat.satellite_id);
                    info!(
                        "  {} PRN {}: Age={} days ({}), Preview={:02X}",
                        constellation,
                        prn,
                        sat.age_days,
                        sat.status_str(),
                        &sat.data_preview
                    );
                }
            }
            Err(e) => error!("Failed to get sample status: {:?}", e),
        }

        // Determine if update is needed
        if AUTO_UPDATE_ENABLED && status.needs_update() {
            info!("");
            info!("===========================================");
            info!("Almanac Update Required");
            info!("===========================================");

            // Determine update strategy
            let strategy = match almanac_mgr.determine_update_strategy(&mut radio).await {
                Ok(s) => s,
                Err(e) => {
                    error!("Failed to determine strategy: {:?}", e);
                    embassy_time::Timer::after_secs(30).await;
                    continue;
                }
            };

            info!("Update strategy: {:?}", strategy);

            // Show what will be updated
            match strategy {
                UpdateStrategy::Full => info!("  Will update: GPS + BeiDou"),
                UpdateStrategy::GpsOnly => info!("  Will update: GPS only"),
                UpdateStrategy::BeiDouOnly => info!("  Will update: BeiDou only"),
                UpdateStrategy::Incremental => info!("  Will update: Continue existing"),
            }

            info!("");
            info!("Starting almanac update...");
            info!("  Duration: ~{} seconds", ALMANAC_FULL_UPDATE_DURATION_S);
            info!("  GPS broadcasts subframes every ~30 seconds");
            info!("  This may take several minutes...");

            let initial_crc = status.global_crc;
            let update_start = Instant::now();

            // Start update
            if let Err(e) = almanac_mgr
                .start_update(&mut radio, strategy, GnssSearchMode::HighEffort)
                .await
            {
                error!("Failed to start update: {:?}", e);
                embassy_time::Timer::after_secs(30).await;
                continue;
            }

            info!("Update in progress...");

            // Monitor update progress
            let mut last_crc = initial_crc;
            let mut subframes_received = 0u32;

            for check_num in 1..=10 {
                embassy_time::Timer::after_secs(30).await; // Check every 30 seconds

                match radio.gnss_get_context_status().await {
                    Ok(ctx) => {
                        let crc_changed = ctx.global_almanac_crc != last_crc;

                        if crc_changed {
                            subframes_received += 1;
                            last_crc = ctx.global_almanac_crc;
                        }

                        let elapsed_s = update_start.elapsed().as_secs();

                        info!(
                            "  [Check #{}, {}s] CRC: 0x{:08X}{}, GPS: {}, BeiDou: {}, Subframes: {}",
                            check_num,
                            elapsed_s,
                            ctx.global_almanac_crc,
                            if crc_changed { " (NEW!)" } else { "" },
                            if ctx.almanac_update_gps {
                                "needs update"
                            } else {
                                "complete"
                            },
                            if ctx.almanac_update_beidou {
                                "needs update"
                            } else {
                                "complete"
                            },
                            subframes_received
                        );

                        // Check if update is complete
                        if !ctx.almanac_update_gps && !ctx.almanac_update_beidou {
                            info!("  Almanac update COMPLETE!");
                            almanac_mgr.complete_update(ctx.global_almanac_crc, elapsed_s as u32);
                            break;
                        }
                    }
                    Err(e) => {
                        warn!("  Status read error: {:?}", e);
                    }
                }
            }

            let total_duration = update_start.elapsed().as_secs();

            info!("");
            info!("Update Summary:");
            info!("  Duration: {}s", total_duration);
            info!("  Subframes received: {}", subframes_received);
            info!("  Total attempts: {}", almanac_mgr.get_update_attempts());
            info!("  Final CRC: 0x{:08X}", almanac_mgr.get_last_crc());

            // Show improvement
            match almanac_mgr.check_status(&mut radio, true).await {
                Ok(new_status) => {
                    info!("");
                    info!("Almanac Improvement:");
                    info!(
                        "  GPS: {}% → {}%",
                        status.gps.completion_percentage(),
                        new_status.gps.completion_percentage()
                    );
                    info!(
                        "  BeiDou: {}% → {}%",
                        status.beidou.completion_percentage(),
                        new_status.beidou.completion_percentage()
                    );
                    info!(
                        "  Overall: {}% → {}%",
                        status.overall_completion(),
                        new_status.overall_completion()
                    );
                }
                Err(e) => {
                    warn!("Failed to check post-update status: {:?}", e);
                }
            }
        }

        // Calculate next check interval
        let next_check_s = if status.is_fully_updated() {
            CHECK_INTERVAL_UPDATED_S
        } else {
            CHECK_INTERVAL_NEEDS_UPDATE_S
        };

        info!("");
        info!("-------------------------------------------");
        info!("Next check in {} seconds...", next_check_s);
        info!("  Almanac manager will monitor for staleness");
        if AUTO_UPDATE_ENABLED {
            info!("  Automatic updates: ENABLED");
        } else {
            info!("  Automatic updates: DISABLED");
        }

        // Enter standby to save power
        if let Err(e) = SystemExt::set_standby(&mut radio, StandbyConfig::RC).await {
            warn!("Failed to enter standby: {:?}", e);
        }

        embassy_time::Timer::after_secs(next_check_s).await;
    }
}
