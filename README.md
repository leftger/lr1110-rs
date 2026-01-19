# lr1110-rs

Rust library providing high-level access to the Semtech LR1110 transceiver's advanced features.

This crate extends the basic radio functionality provided by [`lora-phy`](https://github.com/lora-rs/lora-rs) with comprehensive access to the LR1110's unique capabilities.

## Features

- **`system`** - System diagnostics (temperature, voltage, RNG, UID)
- **`gnss`** - GNSS scanning for geolocation (GPS/BeiDou)
- **`wifi`** - WiFi passive scanning for indoor positioning
- **`ranging`** - RTToF distance measurement
- **`bootloader`** - Firmware update utilities
- **`lr_fhss`** - LR-FHSS modulation support
- **`regmem`** - Direct register/memory access

## Installation

```toml
[dependencies]
lr1110-rs = { version = "0.1", features = ["system", "gnss", "wifi"] }
lora-phy = { git = "https://github.com/leftger/lora-rs", branch = "feat/lr1110-lora-only" }
```

## Usage

```rust
use lr1110_rs::system::SystemExt;
use lr1110_rs::gnss::{GnssExt, GnssSearchMode, GNSS_GPS_MASK};
use lr1110_rs::wifi::{WifiExt, WifiSignalTypeScan, WIFI_ALL_CHANNELS_MASK};

// System info
let temp = radio.get_temp().await?;
let uid = radio.read_uid().await?;

// GNSS scan
radio.gnss_set_constellation(GNSS_GPS_MASK).await?;
radio.gnss_scan(GnssSearchMode::HighEffort, 0x07, 0).await?;

// WiFi scan
radio.wifi_scan(
    WifiSignalTypeScan::TypeBGN,
    WIFI_ALL_CHANNELS_MASK,
    WifiScanMode::Beacon,
    32, 1, 100, false
).await?;
```

## Examples

See `examples/stm32wba/src/bin/` for complete examples:
- `lr1110_system_info.rs` - System information
- `lr1110_gnss_scan.rs` - GNSS scanning
- `lr1110_wifi_scan.rs` - WiFi scanning
- `lr1110_ranging_demo.rs` - RTToF ranging (skeleton)

## Hardware Support

Tested on STM32WBA65RI with LR1110. Compatible with LR1110, LR1120, and LR1121 variants.

## License

Licensed under either of Apache License 2.0 or MIT license at your option.
