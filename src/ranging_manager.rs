//! RTToF Ranging Manager with frequency hopping
//!
//! High-level coordinator for RTToF ranging implementing the two-phase protocol from
//! `lr11xx_ranging_demo/app_ranging_hopping.c`:
//!
//! - **Phase 1 (LoRa Init)**: Manager sends address packet; subordinate replies with its RSSI
//! - **Phase 2 (RTToF Hopping)**: 39-channel frequency-hopped RTToF distance measurement
//!
//! # Source
//!
//! Ported from: `lr11xx_ranging_demo/.../ranging_hopping_frequency/app_ranging_hopping.c`
//!
//! # Protocol Roles
//!
//! ## Manager
//!
//! Initiates the session, drives channel hopping, and collects results:
//!
//! ```ignore
//! let result = manager.run(&mut radio, &config, RangingRole::Manager).await?;
//! // result.median_distance_m is the final distance in meters
//! // result.per is the packet error rate (0–100%)
//! ```
//!
//! ## Subordinate
//!
//! Passively responds; hardware handles RTToF replies automatically:
//!
//! ```ignore
//! // Start subordinate first; manager drives timing
//! let result = manager.run(&mut radio, &config, RangingRole::Subordinate).await?;
//! ```
//!
//! # Timing Model
//!
//! Each channel occupies `req_delay + DONE_PROCESSING_TIME_MS` milliseconds.
//! `req_delay` is computed from SF/BW and covers the full RTToF exchange window.
//! Both devices use identical per-channel periods so they remain synchronized after
//! the LoRa init phase.
//!
//! # Example
//!
//! ```ignore
//! use lr1110_rs::ranging_manager::{RangingConfig, RangingManager, RangingRole};
//! use lr1110_rs::ranging::{lora_bw, lora_cr, lora_sf, ranging_channels, ranging_config};
//!
//! let mut manager = RangingManager::new();
//! manager.set_preranging_callback(|| sidewalk_suspend());
//! manager.set_postranging_callback(|| sidewalk_resume());
//!
//! let config = RangingConfig {
//!     sf: lora_sf::SF8,
//!     bw: lora_bw::BW_500,
//!     channel_table: &ranging_channels::US915,
//!     ..Default::default()
//! };
//!
//! let result = manager.run(&mut radio, &config, RangingRole::Manager).await?;
//! defmt::info!("Distance: {} m, PER: {}%", result.median_distance_m as i32, result.per);
//! ```
//!
//! See `examples/stm32wba/src/bin/lr1110_ranging_demo.rs` for a complete example.

use crate::radio::RadioControlExt;
use crate::ranging::{
    calculate_ranging_request_delay_ms, lora_bw, lora_sf, packet_type,
    ranging_config::{
        DONE_PROCESSING_TIME_MS, INIT_PAYLOAD_LENGTH, MAX_HOPPING_CHANNELS, MIN_HOPPING_CHANNELS,
        RESPONSE_SYMBOLS_COUNT, RX_CONTINUOUS, SUBORDINATE_CHECK_LENGTH_BYTES,
    },
    RangingExt,
};
use crate::time::{delay_ms, Instant};
use embedded_hal_async::delay::Delay;
use lora_phy::mod_params::{Bandwidth, RadioError};

pub use crate::ranging::{RangingRole, RangingStatus};

// =============================================================================
// IRQ Masks (from main_ranging_demo.h)
// Bit positions from system.rs constants.
// =============================================================================

/// IRQ mask for the LoRa initialization phase (TX_DONE | RX_DONE | HEADER_ERROR | TIMEOUT | CRC_ERROR)
const LORA_IRQ_MASK: u32 = (1 << 2) | (1 << 3) | (1 << 6) | (1 << 10) | (1 << 7);

/// IRQ mask for the manager during RTToF hopping (EXCH_VALID | RANGING_TIMEOUT)
const RANGING_MANAGER_IRQ_MASK: u32 = (1 << 17) | (1 << 18);

/// IRQ mask for the subordinate during RTToF hopping (REQ_DISCARDED | RESP_DONE | REQ_VALID)
const RANGING_SUBORDINATE_IRQ_MASK: u32 = (1 << 15) | (1 << 16) | (1 << 14);

// =============================================================================
// Public Types
// =============================================================================

/// Ranging session error (distinct from business-logic `RangingStatus`)
#[derive(Debug)]
pub enum RangingError {
    /// SPI / hardware error
    Radio(RadioError),
    /// LoRa init phase: no valid response from peer within `init_timeout_ms`
    InitFailed,
}

impl From<RadioError> for RangingError {
    fn from(e: RadioError) -> Self {
        RangingError::Radio(e)
    }
}

/// Ranging session configuration
#[derive(Clone, Copy, Debug)]
pub struct RangingConfig {
    /// LoRa spreading factor (use `lora_sf::SF*` constants)
    pub sf: u8,

    /// LoRa bandwidth (use `lora_bw::BW_*` constants; BW_500 recommended)
    pub bw: u8,

    /// LoRa coding rate (use `lora_cr::CR_*` constants)
    pub cr: u8,

    /// Preamble length in symbols
    pub preamble_len: u16,

    /// TX output power in dBm
    pub tx_power_dbm: i8,

    /// Base frequency for the LoRa initialization packet (Hz)
    pub base_frequency_hz: u32,

    /// Ranging address (same value must be configured on both devices)
    pub address: u32,

    /// 39-entry frequency hopping table (use `ranging_channels::US915` etc.)
    pub channel_table: &'static [u32],

    /// Optional hardware RX/TX delay calibration value.
    ///
    /// Improves absolute distance accuracy. Look up with
    /// `apps_common_lr11xx_get_ranging_rx_tx_delay` from SWDR001, or leave as
    /// `None` to skip compensation.
    pub rx_tx_delay_indicator: Option<u32>,

    /// Timeout for the LoRa init phase in milliseconds.
    ///
    /// Subordinate should be started before the manager so it is already in RX
    /// when the manager transmits the init packet.
    pub init_timeout_ms: u32,
}

impl Default for RangingConfig {
    fn default() -> Self {
        Self {
            // Defaults match apps_configuration.h in lr11xx_ranging_demo
            sf: lora_sf::SF8,
            bw: lora_bw::BW_500,
            cr: 0x01, // CR_4_5
            preamble_len: 12,
            tx_power_dbm: 13,
            base_frequency_hz: 907_850_000, // US915 first channel
            address: crate::ranging::ranging_config::DEFAULT_ADDRESS,
            channel_table: &crate::ranging::ranging_channels::US915,
            rx_tx_delay_indicator: None,
            init_timeout_ms: 10_000,
        }
    }
}

/// Results from a completed ranging session (manager side)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct RangingResult {
    /// Session outcome
    pub status: RangingStatus,

    /// Median distance across all successful channel measurements (meters)
    pub median_distance_m: f32,

    /// Packet error rate: 100 − (valid / total) × 100
    pub per: u8,

    /// Manager RSSI from the LoRa init exchange (dBm)
    pub rssi_manager_dbm: i8,

    /// Subordinate RSSI echoed back in the LoRa init response (dBm)
    pub rssi_subordinate_dbm: i8,

    /// SNR from the LoRa init exchange (dB)
    pub snr_db: i8,

    /// Number of channels that returned a valid measurement
    pub valid_measurements: usize,

    /// Per-channel distances in meters (index = channel order; 0 for failed channels)
    pub per_channel_distances: [i32; MAX_HOPPING_CHANNELS],

    /// Per-channel RSSI values (dBm; 0 for failed channels)
    pub per_channel_rssi: [i8; MAX_HOPPING_CHANNELS],
}

/// Callback type for pre-ranging actions (e.g., suspending BLE/Sidewalk)
pub type PreRangingCallback = fn();

/// Callback type for post-ranging actions (e.g., resuming BLE/Sidewalk)
pub type PostRangingCallback = fn();

// =============================================================================
// RangingManager
// =============================================================================

/// RTToF Ranging Manager
///
/// Coordinates ranging with pre/post callbacks and radio access sequencing.
/// Create one instance and call [`run`](RangingManager::run) for each session.
pub struct RangingManager {
    preranging_cb: Option<PreRangingCallback>,
    postranging_cb: Option<PostRangingCallback>,
}

impl RangingManager {
    /// Create a new ranging manager with no callbacks
    pub fn new() -> Self {
        Self {
            preranging_cb: None,
            postranging_cb: None,
        }
    }

    /// Set a callback invoked before any radio access begins
    pub fn set_preranging_callback(&mut self, cb: PreRangingCallback) {
        self.preranging_cb = Some(cb);
    }

    /// Set a callback invoked after the session ends and the radio is idle
    pub fn set_postranging_callback(&mut self, cb: PostRangingCallback) {
        self.postranging_cb = Some(cb);
    }

    /// Run a full ranging session
    ///
    /// Executes the two-phase protocol (LoRa init + RTToF hopping) and returns
    /// the aggregated result. The subordinate should enter `run()` before the
    /// manager so it is already listening when the manager sends the init packet.
    ///
    /// # Errors
    ///
    /// - `RangingError::Radio` on SPI/hardware failures
    /// - `RangingError::InitFailed` if no valid init response arrives within `init_timeout_ms`
    pub async fn run<Radio, D>(
        &mut self,
        radio: &mut Radio,
        delay: &mut D,
        config: &RangingConfig,
        role: RangingRole,
    ) -> Result<RangingResult, RangingError>
    where
        Radio: RadioControlExt + RangingExt,
        D: Delay,
    {
        debug_assert!(
            config.channel_table.len() == MAX_HOPPING_CHANNELS,
            "channel_table must have exactly 39 entries"
        );

        if let Some(cb) = self.preranging_cb {
            cb();
        }

        let result = match role {
            RangingRole::Manager => self.run_manager(radio, delay, config).await,
            RangingRole::Subordinate => self.run_subordinate(radio, delay, config).await,
        };

        if let Some(cb) = self.postranging_cb {
            cb();
        }

        result
    }

    // =========================================================================
    // Manager
    // =========================================================================

    async fn run_manager<Radio, D>(
        &mut self,
        radio: &mut Radio,
        delay: &mut D,
        config: &RangingConfig,
    ) -> Result<RangingResult, RangingError>
    where
        Radio: RadioControlExt + RangingExt,
        D: Delay,
    {
        let ldro = compute_ldro(config.bw, config.sf);
        let req_delay_ms = calculate_ranging_request_delay_ms(
            config.bw,
            config.sf,
            config.preamble_len,
            RESPONSE_SYMBOLS_COUNT,
        );
        let lora_toa_ms = lora_time_on_air_ms(config);

        // ── Phase 1: LoRa init ──────────────────────────────────────────────

        self.setup_lora(radio, config, ldro).await?;

        // Build init payload: [addr MSB..LSB, start_channel=0, sub_rssi=0]
        let mut payload = [0u8; INIT_PAYLOAD_LENGTH];
        payload[0] = (config.address >> 24) as u8;
        payload[1] = (config.address >> 16) as u8;
        payload[2] = (config.address >> 8) as u8;
        payload[3] = config.address as u8;
        // payload[4] = 0 (start channel); payload[5] = 0 (placeholder)

        radio.write_buffer(0, &payload).await?;
        radio.set_tx(0).await?;
        delay_ms(delay, lora_toa_ms + 5).await; // wait for TX_DONE

        // Enter RX for subordinate response
        radio.set_rx_ms(lora_toa_ms + 50).await?;
        delay_ms(delay, lora_toa_ms + 50).await; // wait for RX_DONE or timeout

        let (pld_len, buf_ptr) = radio.get_rx_buffer_status().await?;
        if (pld_len as usize) < INIT_PAYLOAD_LENGTH {
            return Err(RangingError::InitFailed);
        }

        let mut rx_buf = [0u8; INIT_PAYLOAD_LENGTH];
        radio
            .read_buffer(buf_ptr, INIT_PAYLOAD_LENGTH as u8, &mut rx_buf)
            .await?;

        if rx_buf[0] != ((config.address >> 24) as u8)
            || rx_buf[1] != ((config.address >> 16) as u8)
            || rx_buf[2] != ((config.address >> 8) as u8)
            || rx_buf[3] != (config.address as u8)
        {
            return Err(RangingError::InitFailed);
        }

        let rssi_subordinate_dbm = rx_buf[5] as i8;
        let (rssi_raw, snr_db) = radio.get_lora_pkt_status().await?;
        let rssi_manager_dbm = rssi_raw as i8;

        // ── Phase 2: RTToF hopping ──────────────────────────────────────────

        self.setup_rttof_manager(radio, config, ldro).await?;

        // Global timeout matches C demo: req_delay * (N+1) + N + 5 ms
        let global_timeout_ms =
            req_delay_ms * (MAX_HOPPING_CHANNELS as u32 + 1) + MAX_HOPPING_CHANNELS as u32 + 5;
        let deadline = Instant::now().add_ms(global_timeout_ms);

        // First channel fires after req_delay (gives subordinate time to switch modes)
        delay_ms(delay, req_delay_ms).await;

        let mut distance_results = [0i32; MAX_HOPPING_CHANNELS];
        let mut rssi_results = [0i8; MAX_HOPPING_CHANNELS];
        let mut valid_count = 0usize;
        let mut timed_out = false;

        for channel in 0..MAX_HOPPING_CHANNELS {
            if Instant::now() > deadline {
                timed_out = true;
                break;
            }

            radio
                .set_rf_frequency(config.channel_table[channel])
                .await?;
            radio.set_tx(0).await?;

            // Wait for the full RTToF exchange window
            delay_ms(delay, req_delay_ms).await;

            // Read result; miss is silent (channel stays at 0 in results arrays)
            if let Ok(r) = radio
                .rttof_get_distance_result(bw_to_bandwidth(config.bw))
                .await
            {
                distance_results[valid_count] = r.distance_m;
                rssi_results[valid_count] = r.rssi_dbm;
                valid_count += 1;
            }

            // Inter-channel processing pause
            delay_ms(delay, DONE_PROCESSING_TIME_MS).await;
        }

        // ── Aggregate results ───────────────────────────────────────────────

        let per =
            100u8.saturating_sub(((valid_count as u32 * 100) / MAX_HOPPING_CHANNELS as u32) as u8);

        let (status, median_distance_m) = if timed_out && valid_count == 0 {
            (RangingStatus::Timeout, 0.0)
        } else if valid_count < MIN_HOPPING_CHANNELS {
            (
                RangingStatus::PerError,
                median_of(&distance_results[..valid_count]),
            )
        } else {
            (
                RangingStatus::Valid,
                median_of(&distance_results[..valid_count]),
            )
        };

        Ok(RangingResult {
            status,
            median_distance_m,
            per,
            rssi_manager_dbm,
            rssi_subordinate_dbm,
            snr_db,
            valid_measurements: valid_count,
            per_channel_distances: distance_results,
            per_channel_rssi: rssi_results,
        })
    }

    // =========================================================================
    // Subordinate
    // =========================================================================

    async fn run_subordinate<Radio, D>(
        &mut self,
        radio: &mut Radio,
        delay: &mut D,
        config: &RangingConfig,
    ) -> Result<RangingResult, RangingError>
    where
        Radio: RadioControlExt + RangingExt,
        D: Delay,
    {
        let ldro = compute_ldro(config.bw, config.sf);
        let req_delay_ms = calculate_ranging_request_delay_ms(
            config.bw,
            config.sf,
            config.preamble_len,
            RESPONSE_SYMBOLS_COUNT,
        );
        let lora_toa_ms = lora_time_on_air_ms(config);

        // ── Phase 1: wait for manager LoRa init packet ──────────────────────

        self.setup_lora(radio, config, ldro).await?;
        radio.set_rx(RX_CONTINUOUS).await?;
        delay_ms(delay, config.init_timeout_ms).await;

        let (pld_len, buf_ptr) = radio.get_rx_buffer_status().await?;
        if (pld_len as usize) < INIT_PAYLOAD_LENGTH {
            return Err(RangingError::InitFailed);
        }

        let mut rx_buf = [0u8; INIT_PAYLOAD_LENGTH];
        radio
            .read_buffer(buf_ptr, INIT_PAYLOAD_LENGTH as u8, &mut rx_buf)
            .await?;

        if rx_buf[0] != ((config.address >> 24) as u8)
            || rx_buf[1] != ((config.address >> 16) as u8)
            || rx_buf[2] != ((config.address >> 8) as u8)
            || rx_buf[3] != (config.address as u8)
        {
            return Err(RangingError::InitFailed);
        }

        let (rssi_raw, _snr) = radio.get_lora_pkt_status().await?;
        let own_rssi = rssi_raw as i8;

        // Echo payload back with own RSSI in byte 5
        rx_buf[5] = own_rssi as u8;
        radio.write_buffer(0, &rx_buf).await?;
        radio.set_tx(0).await?;
        delay_ms(delay, lora_toa_ms + 5).await; // wait for TX_DONE

        // ── Phase 2: RTToF hopping (hardware auto-responds to requests) ──────

        self.setup_rttof_subordinate(radio, config, ldro).await?;

        // First channel: -1ms vs manager to ensure RX window opens before TX
        delay_ms(delay, req_delay_ms.saturating_sub(1)).await;

        for channel in 0..MAX_HOPPING_CHANNELS {
            radio
                .set_rf_frequency(config.channel_table[channel])
                .await?;
            radio.set_rx(0).await?; // single-shot RX; hardware auto-responds

            // Hold the channel window open for the full exchange duration
            delay_ms(delay, req_delay_ms + DONE_PROCESSING_TIME_MS).await;
        }

        // Subordinate has no per-channel results to report
        Ok(RangingResult {
            status: RangingStatus::Valid,
            median_distance_m: 0.0,
            per: 0,
            rssi_manager_dbm: 0,
            rssi_subordinate_dbm: own_rssi,
            snr_db: 0,
            valid_measurements: 0,
            per_channel_distances: [0i32; MAX_HOPPING_CHANNELS],
            per_channel_rssi: [0i8; MAX_HOPPING_CHANNELS],
        })
    }

    // =========================================================================
    // Radio setup helpers
    // =========================================================================

    async fn setup_lora<Radio>(
        &self,
        radio: &mut Radio,
        config: &RangingConfig,
        ldro: u8,
    ) -> Result<(), RangingError>
    where
        Radio: RadioControlExt,
    {
        radio.set_packet_type(packet_type::LORA).await?;
        radio.set_rf_frequency(config.base_frequency_hz).await?;
        radio
            .set_lora_mod_params(config.sf, config.bw, config.cr, ldro)
            .await?;
        radio
            .set_lora_pkt_params(
                config.preamble_len,
                0, // explicit header
                INIT_PAYLOAD_LENGTH as u8,
                1, // CRC on
                0, // standard IQ
            )
            .await?;
        radio
            .set_lora_sync_word(crate::ranging::ranging_config::LORA_SYNC_WORD)
            .await?;
        radio.set_dio_irq_params(LORA_IRQ_MASK).await?;
        Ok(())
    }

    async fn setup_rttof_manager<Radio>(
        &self,
        radio: &mut Radio,
        config: &RangingConfig,
        ldro: u8,
    ) -> Result<(), RangingError>
    where
        Radio: RadioControlExt + RangingExt,
    {
        radio.set_packet_type(packet_type::RTTOF).await?;
        radio
            .set_lora_mod_params(config.sf, config.bw, config.cr, ldro)
            .await?;
        radio
            .set_lora_pkt_params(config.preamble_len, 0, 10, 1, 0)
            .await?;

        if let Some(delay) = config.rx_tx_delay_indicator {
            radio.rttof_set_rx_tx_delay_indicator(delay).await?;
        }

        radio.rttof_set_parameters(RESPONSE_SYMBOLS_COUNT).await?;
        radio.rttof_set_request_address(config.address).await?;
        radio.set_dio_irq_params(RANGING_MANAGER_IRQ_MASK).await?;
        Ok(())
    }

    async fn setup_rttof_subordinate<Radio>(
        &self,
        radio: &mut Radio,
        config: &RangingConfig,
        ldro: u8,
    ) -> Result<(), RangingError>
    where
        Radio: RadioControlExt + RangingExt,
    {
        radio.set_packet_type(packet_type::RTTOF).await?;
        radio
            .set_lora_mod_params(config.sf, config.bw, config.cr, ldro)
            .await?;
        radio
            .set_lora_pkt_params(config.preamble_len, 0, 10, 1, 0)
            .await?;

        if let Some(delay) = config.rx_tx_delay_indicator {
            radio.rttof_set_rx_tx_delay_indicator(delay).await?;
        }

        radio.rttof_set_parameters(RESPONSE_SYMBOLS_COUNT).await?;
        radio
            .rttof_set_address(config.address, SUBORDINATE_CHECK_LENGTH_BYTES)
            .await?;
        radio
            .set_dio_irq_params(RANGING_SUBORDINATE_IRQ_MASK)
            .await?;
        Ok(())
    }
}

impl Default for RangingManager {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Private helpers
// =============================================================================

/// Convert `lora_bw` byte encoding to the `lora-phy` `Bandwidth` enum
fn bw_to_bandwidth(bw: u8) -> Bandwidth {
    match bw {
        lora_bw::BW_125 => Bandwidth::_125KHz,
        lora_bw::BW_250 => Bandwidth::_250KHz,
        lora_bw::BW_500 | _ => Bandwidth::_500KHz,
    }
}

/// Compute LDRO flag: enabled when symbol time ≥ 16 ms
fn compute_ldro(bw: u8, sf: u8) -> u8 {
    let bw_khz: u32 = match bw {
        lora_bw::BW_125 => 125,
        lora_bw::BW_250 => 250,
        lora_bw::BW_500 | _ => 500,
    };
    // symbol_time_ms = 2^sf / bw_khz; LDRO if ≥ 16
    let threshold_count = 16u32 * bw_khz; // 2^sf threshold
    if (1u32 << sf as u32) >= threshold_count {
        1
    } else {
        0
    }
}

/// Estimate LoRa time-on-air for the 6-byte init packet (milliseconds)
///
/// Uses the time-on-air formula from `radio::get_lora_time_on_air_in_ms`.
fn lora_time_on_air_ms(config: &RangingConfig) -> u32 {
    crate::radio::get_lora_time_on_air_in_ms(
        config.preamble_len,
        0, // explicit header
        INIT_PAYLOAD_LENGTH as u8,
        1, // CRC on
        config.sf,
        config.bw,
        config.cr,
        compute_ldro(config.bw, config.sf),
    )
}

/// Compute the median of a slice using an in-place bubble sort on a fixed buffer.
///
/// Matches the C demo's `ranging_handle_distance_result()` logic exactly.
/// Returns 0.0 for empty slices.
fn median_of(values: &[i32]) -> f32 {
    let n = values.len();
    if n == 0 {
        return 0.0;
    }

    let mut buf = [0i32; MAX_HOPPING_CHANNELS];
    buf[..n].copy_from_slice(values);

    // Bubble sort (ascending)
    for i in (1..n).rev() {
        for j in 0..i {
            if buf[j] > buf[j + 1] {
                buf.swap(j, j + 1);
            }
        }
    }

    if n % 2 == 0 {
        (buf[n / 2] + buf[n / 2 - 1]) as f32 / 2.0
    } else {
        buf[n / 2] as f32
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_creation() {
        let m = RangingManager::new();
        assert!(m.preranging_cb.is_none());
        assert!(m.postranging_cb.is_none());
    }

    #[test]
    fn test_median_odd() {
        assert_eq!(median_of(&[30, 10, 20]), 20.0);
    }

    #[test]
    fn test_median_even() {
        assert_eq!(median_of(&[40, 10, 30, 20]), 25.0);
    }

    #[test]
    fn test_median_single() {
        assert_eq!(median_of(&[42]), 42.0);
    }

    #[test]
    fn test_median_empty() {
        assert_eq!(median_of(&[]), 0.0);
    }

    #[test]
    fn test_compute_ldro_sf12_bw125() {
        // SF12, BW_125: symbol_time = 4096/125 ≈ 32.8 ms → LDRO on
        assert_eq!(compute_ldro(lora_bw::BW_125, lora_sf::SF12), 1);
    }

    #[test]
    fn test_compute_ldro_sf7_bw500() {
        // SF7, BW_500: symbol_time = 128/500 ≈ 0.26 ms → LDRO off
        assert_eq!(compute_ldro(lora_bw::BW_500, lora_sf::SF7), 0);
    }

    #[test]
    fn test_default_config() {
        let c = RangingConfig::default();
        assert_eq!(c.sf, lora_sf::SF8);
        assert_eq!(c.bw, lora_bw::BW_500);
        assert_eq!(c.preamble_len, 12);
        assert_eq!(c.channel_table.len(), MAX_HOPPING_CHANNELS);
    }

    #[test]
    fn test_bw_to_bandwidth() {
        assert!(matches!(
            bw_to_bandwidth(lora_bw::BW_125),
            Bandwidth::_125KHz
        ));
        assert!(matches!(
            bw_to_bandwidth(lora_bw::BW_250),
            Bandwidth::_250KHz
        ));
        assert!(matches!(
            bw_to_bandwidth(lora_bw::BW_500),
            Bandwidth::_500KHz
        ));
    }
}
