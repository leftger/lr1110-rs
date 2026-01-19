# LR1110 STM32WBA Examples

Complete examples demonstrating LR1110 geolocation features on STM32WBA65.

## Examples

### GNSS Scanning

- **`lr1110_gnss_scan.rs`** - Basic GNSS with timeout
- **`lr1110_gnss_scan_with_irq.rs`** - Advanced GNSS with IRQ management, mobile/static modes
- **`lr1110_almanac_manager.rs`** - Almanac age tracking and updates

### WiFi Scanning

- **`lr1110_wifi_scan.rs`** - Basic WiFi with timeout
- **`lr1110_wifi_scan_with_irq.rs`** - Advanced WiFi with IRQ management, filtering, sorting

### Other Features

- **`lr1110_system_info.rs`** - Chip info, temperature, voltage
- **`lr1110_firmware_update.rs`** - Firmware update over SPI
- **`lr1110_ranging_demo.rs`** - RTToF distance measurement
- **`lr1110_crypto_demo.rs`** - Hardware crypto operations
- **`lr_fhss_ping.rs`** - LR-FHSS communication

## Quick Start

```bash
# Basic GNSS scan
GNSS_LAT=33.4942 GNSS_LON=-111.9261 cargo run --release --bin lr1110_gnss_scan

# Advanced GNSS with IRQ
GNSS_LAT=33.4942 GNSS_LON=-111.9261 cargo run --release --bin lr1110_gnss_scan_with_irq

# WiFi scan
cargo run --release --bin lr1110_wifi_scan_with_irq

# Almanac management
GNSS_LAT=33.4942 GNSS_LON=-111.9261 cargo run --release --bin lr1110_almanac_manager
```

## Hardware Setup

**STM32WBA65 Nucleo + LR1110 Shield:**

```
LR1110       STM32WBA     Function
──────────────────────────────────────
NSS      →   PD14         SPI2_NSS
SCK      →   PB10         SPI2_SCK
MISO     →   PA9          SPI2_MISO
MOSI     →   PC3          SPI2_MOSI
NRESET   →   PA4          GPIO
BUSY     →   PB13         EXTI13
DIO1     →   PB14         EXTI14 (IRQ)
DIO5-8   →   Internal     RF switches
```

## Documentation

All documentation is in the source code:
- Module docs: See `src/*.rs` files
- Examples: See comments in `src/bin/*.rs` files
- API reference: Run `cargo doc --open --features gnss,wifi,irq-manager`
