//! LR1110 WiFi Passive Scanning Example
//!
//! This example demonstrates using the built-in WiFi scanner of the LR1110 radio:
//! - Configure scan parameters (signal type, channels, scan mode)
//! - Perform WiFi passive scan to detect access points
//! - Read detected AP information (MAC address, RSSI, channel)
//!
//! The results can be sent to LoRa Cloud for WiFi-based geolocation.
//!
//! Hardware connections for STM32WBA65RI:
//! - SPI2_SCK:  PB10
//! - SPI2_MISO: PA9
//! - SPI2_MOSI: PC3
//! - SPI2_NSS:  PD14 (manual control via GPIO)
//! - LR1110_RESET: PA4 (LR1110_NRESET)
//! - LR1110_BUSY:  PB13 (BUSY signal, active high)
//! - LR1110_DIO1:  PB14 (with EXTI interrupt)

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
    convert_temp_to_celsius, convert_vbat_to_volts, RfSwitchConfig, StandbyConfig, SystemExt,
    RFSW0_HIGH, RFSW1_HIGH, RFSW2_HIGH, RFSW3_HIGH,
};
use lr1110_rs::wifi::{
    WifiExt, WifiExtendedFullResult, WifiScanMode, WifiSignalTypeScan, WIFI_ALL_CHANNELS_MASK,
    WIFI_MAX_RESULTS,
};
use {defmt_rtt as _, panic_probe as _};

// Bind EXTI interrupts for PB13 (BUSY) and PB14 (DIO1)
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
    info!("LR1110 WiFi Passive Scanning Example");
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

    // Optional RF switch control pins (set to None if not using)
    let rf_switch_rx: Option<Output<'_>> = None;
    let rf_switch_tx: Option<Output<'_>> = None;

    // Create InterfaceVariant
    let iv = Lr1110InterfaceVariant::new(reset, busy, dio1, rf_switch_rx, rf_switch_tx).unwrap();

    // Configure LR1110 chip variant
    let lr_config = lr1110_module::Config {
        chip: Lr1110Chip::new(),
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl3V0),
        use_dcdc: true,
        rx_boost: false,
    };

    // Create radio instance
    let mut radio = lr1110_module::Lr1110::new(spi_device, iv, lr_config);

    info!("Initializing LR1110...");

    // Reset and initialize the radio
    radio.reset(&mut Delay).await.unwrap();
    embassy_time::Timer::after_millis(100).await;

    // Initialize system (TCXO, DC-DC, calibration)
    info!("Configuring TCXO and calibrating...");
    radio.init_system().await.unwrap();

    // Read chip version and system status
    info!("-------------------------------------------");
    info!("Chip Information:");
    match radio.get_version().await {
        Ok(version) => {
            info!("  Hardware: 0x{:02X}", version.hw);
            info!("  Chip type: {:?}", version.chip_type);
            info!("  Firmware: 0x{:04X}", version.fw);
        }
        Err(e) => {
            error!("  Failed to read version: {:?}", e);
        }
    }

    // Display system diagnostics
    match radio.get_temp().await {
        Ok(temp_raw) => {
            let temp_c = convert_temp_to_celsius(temp_raw);
            let temp_int = temp_c as i16;
            let temp_frac = ((temp_c - temp_int as f32) * 10.0) as u8;
            info!(
                "  Temperature: {}.{}°C (raw: 0x{:04X})",
                temp_int, temp_frac, temp_raw
            );
        }
        Err(e) => {
            warn!("  Failed to read temperature: {:?}", e);
        }
    }

    match radio.get_vbat().await {
        Ok(vbat_raw) => {
            let vbat_v = convert_vbat_to_volts(vbat_raw);
            let vbat_int = vbat_v as u16;
            let vbat_frac = ((vbat_v - vbat_int as f32) * 100.0) as u8;
            info!(
                "  Battery voltage: {}.{:02}V (raw: 0x{:02X})",
                vbat_int, vbat_frac, vbat_raw
            );
        }
        Err(e) => {
            warn!("  Failed to read battery voltage: {:?}", e);
        }
    }

    // Check for system errors using SystemExt
    match SystemExt::get_errors(&mut radio).await {
        Ok(errors) => {
            if errors.has_errors() {
                warn!("  System errors detected: 0x{:04X}", errors.raw);
                if errors.lf_rc_calib_error() {
                    warn!("    - LF RC calibration error");
                }
                if errors.hf_rc_calib_error() {
                    warn!("    - HF RC calibration error");
                }
                if errors.pll_calib_error() {
                    warn!("    - PLL calibration error");
                }
                // Clear errors after reporting
                let _ = SystemExt::clear_errors(&mut radio).await;
            } else {
                info!("  System status: OK (no errors)");
            }
        }
        Err(e) => {
            warn!("  Failed to read system errors: {:?}", e);
        }
    }

    // Configure RF switches for E516V02B/E516V03A board using new RfSwitchConfig:
    // - RFSW0 (DIO5): Sub-GHz RX path
    // - RFSW1 (DIO6): Sub-GHz TX path
    // - RFSW2 (DIO7): GNSS LNA enable (BGA524N6)
    // - RFSW3 (DIO8): WiFi LNA enable (BGA524N6)
    info!("Configuring RF switches for WiFi LNA...");
    let rf_switch_cfg = RfSwitchConfig {
        enable: 0xFF,                // Enable all switches
        standby: 0x00,               // Standby: no switches active
        rx: RFSW0_HIGH,              // RX: RFSW0 (DIO5)
        tx: RFSW0_HIGH | RFSW1_HIGH, // TX: RFSW0 + RFSW1
        tx_hp: RFSW1_HIGH,           // TX HP: RFSW1 (DIO6)
        tx_hf: 0x00,                 // TX HF: none
        gnss: RFSW2_HIGH,            // GNSS: RFSW2 (DIO7) - BGA524N6 LNA
        wifi: RFSW3_HIGH,            // WiFi: RFSW3 (DIO8) - BGA524N6 LNA
    };
    SystemExt::set_dio_as_rf_switch(&mut radio, &rf_switch_cfg)
        .await
        .unwrap();

    // Read WiFi firmware version
    info!("-------------------------------------------");
    info!("WiFi Firmware:");
    match radio.wifi_read_version().await {
        Ok(version) => {
            info!("  WiFi Firmware: v{}.{}", version.major, version.minor);
        }
        Err(e) => {
            error!("  Failed to read WiFi version: {:?}", e);
        }
    }

    // Perform WiFi scans in a loop
    info!("===========================================");
    info!("Starting WiFi scanning loop...");
    info!("===========================================");

    let mut scan_count = 0u32;

    loop {
        scan_count += 1;
        info!("-------------------------------------------");
        info!("WiFi Scan #{}", scan_count);

        // Reset cumulative timing before scan
        if let Err(e) = radio.wifi_reset_cumulative_timing().await {
            warn!("  Failed to reset timing: {:?}", e);
        }

        // Launch WiFi scan
        // - TypeBGN: Scan for all WiFi types (802.11 b/g/n)
        // - ALL_CHANNELS_MASK: Scan all 14 channels
        // - UntilSsid: Extended scan mode that demodulates until SSID field (requires firmware 0x0306+)
        // - max_results: 32 (maximum allowed, range is 1-32, 0 is forbidden!)
        // - nb_scan_per_channel: 10 scans per channel
        // - timeout_per_scan_ms: 90ms per scan
        // - abort_on_timeout: false (continue scanning on timeout)
        if let Err(e) = radio
            .wifi_scan(
                WifiSignalTypeScan::TypeBGN,
                WIFI_ALL_CHANNELS_MASK,
                WifiScanMode::UntilSsid,
                32,    // max_results (range 1-32, 0 is forbidden!)
                10,    // nb_scan_per_channel
                90,    // timeout_per_scan_ms
                false, // abort_on_timeout
            )
            .await
        {
            error!("  Failed to start WiFi scan: {:?}", e);
            embassy_time::Timer::after_secs(5).await;
            continue;
        }

        info!("  Scan started, waiting for completion...");

        // Wait for scan to complete
        // In a real application, you would wait for the WifiScanDone IRQ via DIO1
        // For simplicity, we use a fixed timeout here
        // (14 channels * 10 scans * 90ms = ~12.6 seconds max)
        embassy_time::Timer::after_secs(15).await;

        // Get number of results
        let nb_results = match radio.wifi_get_nb_results().await {
            Ok(count) => {
                info!("  Detected {} access points", count);
                count
            }
            Err(e) => {
                error!("  Failed to get result count: {:?}", e);
                embassy_time::Timer::after_secs(5).await;
                continue;
            }
        };

        if nb_results == 0 {
            warn!("  No WiFi access points detected");
            embassy_time::Timer::after_secs(5).await;
            continue;
        }

        // Read results in extended full format
        let mut results = [WifiExtendedFullResult::default(); WIFI_MAX_RESULTS];
        let read_count = match radio
            .wifi_read_extended_full_results(
                &mut results,
                0,
                nb_results.min(WIFI_MAX_RESULTS as u8),
            )
            .await
        {
            Ok(count) => count,
            Err(e) => {
                error!("  Failed to read results: {:?}", e);
                embassy_time::Timer::after_secs(5).await;
                continue;
            }
        };

        // Display extended results
        info!("  Results:");
        for i in 0..read_count as usize {
            let result = &results[i];

            info!("    --- AP {} ---", i + 1);

            // SSID (if available)
            if let Some(ssid) = result.ssid_str() {
                info!("      SSID: \"{}\"", ssid);
            } else {
                info!("      SSID: (not available or invalid UTF-8)");
            }

            // MAC addresses
            let mac1 = result.mac_address_1;
            let mac2 = result.mac_address_2;
            let mac3 = result.mac_address_3;
            info!(
                "      MAC 1: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]
            );
            info!(
                "      MAC 2: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]
            );
            info!(
                "      MAC 3: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]
            );

            // Channel and RSSI
            let channel = result.channel();
            info!("      Channel (scan): {:?}", channel);
            info!("      Channel (frame): {:?}", result.current_channel);
            info!("      RSSI: {} dBm", result.rssi);
            info!("      Signal type: {:?}", result.signal_type());

            // Country code
            if result.country_code[0] != 0 || result.country_code[1] != 0 {
                info!(
                    "      Country code: {}{}",
                    result.country_code[0] as char, result.country_code[1] as char
                );
            }

            // Frame information
            info!("      Frame control: 0x{:04X}", result.frame_control);
            info!("      Sequence control: 0x{:04X}", result.seq_control);
            info!("      Rate: {}", result.rate);
            info!("      Service: 0x{:04X}", result.service);
            info!("      Length: {}", result.length);
            info!("      Beacon period: {} TU", result.beacon_period_tu);
            info!("      Timestamp: {} us", result.timestamp_us);

            // FCS information
            info!(
                "      FCS checked: {}, FCS OK: {}",
                result.fcs_check_byte.is_fcs_checked, result.fcs_check_byte.is_fcs_ok
            );

            // Other fields
            if result.io_regulation != 0 {
                info!("      IO regulation: {}", result.io_regulation);
            }
            info!("      Phi offset: {}", result.phi_offset);
        }

        // Read cumulative timing
        match radio.wifi_read_cumulative_timing().await {
            Ok(timing) => {
                info!("  Timing:");
                info!("    RX detection:   {} us", timing.rx_detection_us);
                info!("    RX correlation: {} us", timing.rx_correlation_us);
                info!("    RX capture:     {} us", timing.rx_capture_us);
                info!("    Demodulation:   {} us", timing.demodulation_us);
            }
            Err(e) => {
                warn!("  Failed to read timing: {:?}", e);
            }
        }

        // In a real application, you would send the MAC addresses and RSSI values
        // to LoRa Cloud for WiFi-based geolocation

        // Wait before next scan
        info!("  Waiting 30 seconds before next scan...");

        // Enter standby mode to save power during wait
        if let Err(e) = SystemExt::set_standby(&mut radio, StandbyConfig::RC).await {
            warn!("  Failed to enter standby: {:?}", e);
        }

        embassy_time::Timer::after_secs(30).await;
    }
}
