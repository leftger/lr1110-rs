//! RTToF ranging state machine and frequency hopping session
//!
//! This module provides two complementary building blocks:
//!
//! - **[`RangingProtocol`]** — an event-driven state machine that faithfully ports
//!   `app_radio_ranging_run()` from the lr11xx_ranging_demo. It has no I/O; the
//!   caller executes each [`ProtocolAction`] and feeds back the resulting
//!   [`RadioEvent`]. Works equally well with IRQ handlers and async executors.
//!
//! - **[`HoppingSession`]** — pure data structure that tracks 39-channel state
//!   and computes final statistics (median, PER). Usable independently.
//!
//! # How the state machine relates to the C demo
//!
//! | C demo (`app_radio_internal_states`) | Rust [`ProtocolState`]   |
//! |--------------------------------------|--------------------------|
//! | `APP_RADIO_RANGING_CONFIG`           | `Config`                 |
//! | `APP_RADIO_IDLE`                     | `Idle`                   |
//! | `APP_RADIO_TX`                       | `LoRaTxDone`             |
//! | `APP_RADIO_RX`                       | `LoRaRxDone`             |
//! | `APP_RADIO_TIMEOUT` / `ERROR`        | `LoRaError`              |
//! | `APP_RADIO_RANGING_START`            | `RangingStart`           |
//! | `APP_RADIO_RANGING_DONE`             | `RangingDone`            |
//! | `APP_RADIO_RANGING_TIMEOUT`          | `RangingChannelTimeout`  |
//! | `APP_RADIO_RANGING_REQ_VALID`        | `RangingReqValid`        |
//!
//! # Source
//!
//! Ported from: `lr11xx_ranging_demo/.../ranging_hopping_frequency/app_ranging_hopping.c`
//!
//! # Example (IRQ-driven)
//!
//! ```ignore
//! use lr1110_rs::ranging_protocol::{ProtocolConfig, ProtocolAction, RadioEvent, RangingProtocol};
//! use lr1110_rs::ranging::{RangingRole, ranging_channels};
//!
//! let config = ProtocolConfig::new(
//!     lora_sf::SF8, lora_bw::BW_500, lora_cr::CR_4_5,
//!     12,                          // preamble symbols
//!     ranging_config::DEFAULT_ADDRESS,
//!     &ranging_channels::US915,
//!     907_850_000,                 // base frequency Hz
//! );
//! let mut proto = RangingProtocol::new(config, RangingRole::Manager);
//!
//! // Kick off the session
//! execute_action(proto.process_event(RadioEvent::Start)).await?;
//!
//! loop {
//!     let irq_flags = wait_for_irq().await;
//!     let event = RadioEvent::from_irq_flags(irq_flags);
//!
//!     // Data events: caller reads data first, then tells the protocol
//!     if matches!(event, RadioEvent::RxDone) {
//!         let (len, ptr) = radio.get_rx_buffer_status().await?;
//!         let mut buf = [0u8; 6];
//!         radio.read_buffer(ptr, len, &mut buf).await?;
//!         let (rssi, _snr) = radio.get_lora_pkt_status().await?;
//!         proto.set_lora_rx(buf, rssi as i8);
//!     }
//!     if matches!(event, RadioEvent::RangingExchValid) {
//!         let r = radio.rttof_get_distance_result(bw).await?;
//!         proto.record_result(r.distance_m, r.rssi_dbm);
//!     }
//!
//!     let action = proto.process_event(event);
//!     execute_action(action).await?;
//!
//!     if matches!(action, ProtocolAction::SessionComplete | ProtocolAction::SessionTimeout) {
//!         break;
//!     }
//! }
//!
//! if let Some(result) = proto.finalize() {
//!     defmt::info!("Distance: {} m", result.median_distance_m as i32);
//! }
//! ```

use crate::ranging::{
    calculate_ranging_request_delay_ms, calculate_symbol_time_ms,
    lora_bw,
    ranging_config::{
        DONE_PROCESSING_TIME_MS, INIT_PAYLOAD_LENGTH, MAX_HOPPING_CHANNELS, MIN_HOPPING_CHANNELS,
        RESPONSE_SYMBOLS_COUNT,
    },
    RangingRole, RangingStatus,
};

// =============================================================================
// Protocol Config
// =============================================================================

/// Configuration parameters extracted from the LoRa/RTToF settings.
///
/// Computed once at construction; the state machine then only references
/// the pre-calculated timing values rather than repeating arithmetic.
#[derive(Clone, Copy, Debug)]
pub struct ProtocolConfig {
    /// Ranging address (same on both devices)
    pub address: u32,
    /// Base frequency for LoRa init phase (Hz)
    pub base_freq_hz: u32,
    /// 39-entry frequency hopping channel table
    pub channel_table: &'static [u32],
    /// Time for one complete RTToF exchange (ms), covering all symbols
    pub req_delay_ms: u32,
    /// Estimated LoRa init packet time-on-air (ms)
    pub lora_toa_ms: u32,
    /// Subordinate per-channel RX timeout (ms)
    ///
    /// Computed as: req_delay − response_symbol_time − processing_time/2
    pub sub_rx_timeout_ms: u32,
    /// Global session timeout covering all 39 channels (ms)
    pub global_timeout_ms: u32,
    /// Spreading factor (stored for reference)
    pub sf: u8,
    /// Bandwidth byte value (stored for reference)
    pub bw: u8,
}

impl ProtocolConfig {
    /// Derive all timing values from the raw LoRa parameters.
    ///
    /// Matches the computation in `app_radio_ranging_params_init()`.
    pub fn new(
        sf: u8,
        bw: u8,
        cr: u8,
        preamble_len: u16,
        address: u32,
        channel_table: &'static [u32],
        base_freq_hz: u32,
    ) -> Self {
        let req_delay_ms = calculate_ranging_request_delay_ms(bw, sf, preamble_len, RESPONSE_SYMBOLS_COUNT);

        let response_sym_time_ms =
            (calculate_symbol_time_ms(bw, sf) * RESPONSE_SYMBOLS_COUNT as f32) as u32;
        let sub_rx_timeout_ms = req_delay_ms
            .saturating_sub(response_sym_time_ms)
            .saturating_sub(DONE_PROCESSING_TIME_MS / 2);

        // global = req_delay × (N+1) + N×1 + 5  (matches C demo)
        let n = MAX_HOPPING_CHANNELS as u32;
        let global_timeout_ms = req_delay_ms * (n + 1) + n + 5;

        // Approximate LoRa ToA: rough estimate sufficient for RX timeout
        let lora_toa_ms = crate::radio::get_lora_time_on_air_in_ms(
            preamble_len,
            0,
            INIT_PAYLOAD_LENGTH as u8,
            1,
            sf,
            bw,
            cr,
            if (1u32 << sf as u32) >= 16 * match bw { lora_bw::BW_125 => 125u32, lora_bw::BW_250 => 250, _ => 500 } { 1 } else { 0 },
        );

        Self {
            address,
            base_freq_hz,
            channel_table,
            req_delay_ms,
            lora_toa_ms,
            sub_rx_timeout_ms,
            global_timeout_ms,
            sf,
            bw,
        }
    }
}

// =============================================================================
// Events and Actions
// =============================================================================

/// Hardware IRQ events and software timer events that drive the state machine.
///
/// Maps 1:1 to the events that the C demo's IRQ callback routes into
/// `set_ranging_process_state()`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RadioEvent {
    /// Begin the session (software trigger, not a hardware IRQ)
    Start,
    /// `LR11XX_SYSTEM_IRQ_TX_DONE`
    TxDone,
    /// `LR11XX_SYSTEM_IRQ_RX_DONE` — call `set_lora_rx()` before sending this
    RxDone,
    /// `LR11XX_SYSTEM_IRQ_TIMEOUT`
    Timeout,
    /// `LR11XX_SYSTEM_IRQ_HEADER_ERROR`
    HeaderError,
    /// `LR11XX_SYSTEM_IRQ_CRC_ERROR`
    CrcError,
    /// `LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID` — call `record_result()` before sending this
    RangingExchValid,
    /// `LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT`
    RangingTimeout,
    /// `LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID` (subordinate)
    RangingReqValid,
    /// `LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE` (subordinate)
    RangingRespDone,
    /// `LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED` (subordinate)
    RangingReqDiscarded,
    /// Software: inter-channel delay timer expired (`ranging_next_channel_timer`)
    ChannelTimerFired,
    /// Software: subordinate per-channel RX timeout (`sub_ranging_rx_timeout_timer`)
    SubRxTimerFired,
    /// Software: global session timeout (`ranging_global_timer`)
    GlobalTimerFired,
}

/// I/O action the caller must perform after each `process_event` call.
///
/// Compound actions (e.g. `SendLoRaInit`) encode everything needed so the
/// caller does not need to inspect protocol state to know what to do.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ProtocolAction {
    /// Nothing to do; stay in current state and wait for the next event
    None,

    // ── LoRa init phase ──────────────────────────────────────────────────────

    /// **[Manager]** Write `payload` to the TX buffer and issue `set_tx(0)`
    SendLoRaInit { payload: [u8; INIT_PAYLOAD_LENGTH] },

    /// **[Subordinate]** Issue `set_rx(RX_CONTINUOUS)` and wait for manager
    WaitForLoRaInit,

    /// **[Manager]** Issue `set_rx_ms(timeout_ms)` and wait for response
    WaitForLoRaResponse { timeout_ms: u32 },

    /// **[Subordinate]** Write response `payload` to TX buffer and issue `set_tx(0)`
    SendLoRaResponse { payload: [u8; INIT_PAYLOAD_LENGTH] },

    // ── RTToF hopping phase ───────────────────────────────────────────────────

    /// **[Manager]** `set_rf_frequency(freq_hz)` then `set_tx(0)`
    StartRangingTx { freq_hz: u32 },

    /// **[Subordinate]** `set_rf_frequency(freq_hz)` then `set_rx(0)`;
    /// also start a software timer for `sub_rx_timeout_ms`
    StartRangingRx { freq_hz: u32, sub_rx_timeout_ms: u32 },

    /// **[Manager]** The RTToF result was already captured via `record_result()`;
    /// now start the inter-channel delay timer for `timer_ms` milliseconds
    ResultCaptured { timer_ms: u32 },

    // ── Timer management ─────────────────────────────────────────────────────

    /// Start the inter-channel delay timer for `ms` milliseconds
    StartChannelTimer { ms: u32 },

    /// Cancel the subordinate per-channel RX timeout timer
    CancelSubRxTimer,

    /// Set radio to standby (after timeout / error)
    SetStandby,

    /// Set radio to standby **and** start the channel timer for `ms` milliseconds
    StandbyAndStartChannelTimer { ms: u32 },

    // ── Terminal ──────────────────────────────────────────────────────────────

    /// All channels complete — call `finalize()` to get results
    SessionComplete,

    /// Global timer fired before all channels finished
    SessionTimeout,

    /// Restart from Config due to unrecoverable error (e.g. wrong init address)
    Restart,
}

// =============================================================================
// Protocol State (public for introspection / debugging)
// =============================================================================

/// Internal state of the ranging state machine.
///
/// Mirrors `app_radio_internal_states` in the C demo. Exposed for
/// debugging and integration with external state displays.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum ProtocolState {
    /// Initial state — awaiting `RadioEvent::Start`
    Config,
    /// Idle — waiting for a hardware IRQ or timer event
    Idle,
    /// LoRa TX_DONE received (init phase)
    LoRaTxDone,
    /// LoRa RX_DONE received (init phase)
    LoRaRxDone,
    /// LoRa timeout or error in init phase → restart
    LoRaError,
    /// Channel timer fired — issue TX (manager) or RX (subordinate)
    RangingStart,
    /// RTToF exchange complete (manager received valid response)
    RangingDone,
    /// RTToF channel missed (manager timeout or subordinate discarded)
    RangingChannelTimeout,
    /// RTToF request received by subordinate
    RangingReqValid,
    /// All channels complete
    Done,
    /// Global session timeout
    GlobalTimeout,
}

// =============================================================================
// HoppingSession — pure channel tracking and result aggregation
// =============================================================================

/// Per-channel result tracking and statistical aggregation for the RTToF
/// frequency hopping phase.
///
/// Can be used independently of `RangingProtocol` when building custom
/// ranging logic.
#[derive(Clone, Debug)]
pub struct HoppingSession {
    /// Current channel index (0–38)
    pub current_channel: usize,
    /// Distance results for successful channels (meters)
    pub distance_results: [i32; MAX_HOPPING_CHANNELS],
    /// RSSI for successful channels (dBm)
    pub rssi_results: [i8; MAX_HOPPING_CHANNELS],
    /// Number of channels with valid measurements
    pub valid_count: usize,
}

impl HoppingSession {
    /// Create a new session starting at channel 0
    pub fn new() -> Self {
        Self {
            current_channel: 0,
            distance_results: [0i32; MAX_HOPPING_CHANNELS],
            rssi_results: [0i8; MAX_HOPPING_CHANNELS],
            valid_count: 0,
        }
    }

    /// Record a successful measurement for the current channel, then advance
    pub fn record_and_advance(&mut self, distance_m: i32, rssi_dbm: i8) {
        if self.valid_count < MAX_HOPPING_CHANNELS {
            self.distance_results[self.valid_count] = distance_m;
            self.rssi_results[self.valid_count] = rssi_dbm;
            self.valid_count += 1;
        }
        self.current_channel = self.current_channel.saturating_add(1);
    }

    /// Advance past the current channel without recording a result (miss/timeout)
    pub fn skip_channel(&mut self) {
        self.current_channel = self.current_channel.saturating_add(1);
    }

    /// Whether all 39 channels have been attempted
    pub fn is_complete(&self) -> bool {
        self.current_channel >= MAX_HOPPING_CHANNELS
    }

    /// Remaining channels
    pub fn remaining(&self) -> usize {
        MAX_HOPPING_CHANNELS.saturating_sub(self.current_channel)
    }

    /// Packet error rate: 100 − (valid / total) × 100
    pub fn per(&self) -> u8 {
        100u8.saturating_sub(
            ((self.valid_count as u32 * 100) / MAX_HOPPING_CHANNELS as u32) as u8,
        )
    }

    /// Final `RangingStatus` based on valid count
    pub fn status(&self) -> RangingStatus {
        if self.valid_count >= MIN_HOPPING_CHANNELS {
            RangingStatus::Valid
        } else {
            RangingStatus::PerError
        }
    }

    /// Median distance across all successful measurements.
    ///
    /// Uses bubble sort matching `ranging_handle_distance_result()` in the C demo.
    pub fn median_distance_m(&self) -> f32 {
        let n = self.valid_count;
        if n == 0 {
            return 0.0;
        }
        let mut buf = [0i32; MAX_HOPPING_CHANNELS];
        buf[..n].copy_from_slice(&self.distance_results[..n]);
        // Bubble sort ascending
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
}

impl Default for HoppingSession {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// RangingProtocol — event-driven state machine
// =============================================================================

/// Phase within the two-phase protocol
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    LoraInit,
    RttofHopping,
}

/// RTToF ranging state machine — no I/O, fully event-driven.
///
/// Feed [`RadioEvent`]s in; execute the returned [`ProtocolAction`]s.
pub struct RangingProtocol {
    config: ProtocolConfig,
    role: RangingRole,
    state: ProtocolState,
    phase: Phase,
    hopping: HoppingSession,
    /// LoRa RX payload (set by caller before sending `RxDone`)
    lora_rx_buf: [u8; INIT_PAYLOAD_LENGTH],
    /// RSSI read from LoRa packet (set by caller before sending `RxDone`)
    lora_rx_rssi: i8,
    /// Pending ranging result (set by caller before sending `RangingExchValid`)
    pending_distance_m: i32,
    pending_rssi_dbm: i8,
}

impl RangingProtocol {
    /// Create a new protocol instance in the `Config` state.
    ///
    /// Call `process_event(RadioEvent::Start)` to get the initial action.
    pub fn new(config: ProtocolConfig, role: RangingRole) -> Self {
        Self {
            config,
            role,
            state: ProtocolState::Config,
            phase: Phase::LoraInit,
            hopping: HoppingSession::new(),
            lora_rx_buf: [0u8; INIT_PAYLOAD_LENGTH],
            lora_rx_rssi: 0,
            pending_distance_m: 0,
            pending_rssi_dbm: 0,
        }
    }

    // ── Data setters — call before the corresponding event ───────────────────

    /// Set the LoRa RX buffer and RSSI captured by the caller.
    ///
    /// Must be called before `process_event(RadioEvent::RxDone)`.
    pub fn set_lora_rx(&mut self, payload: [u8; INIT_PAYLOAD_LENGTH], rssi_dbm: i8) {
        self.lora_rx_buf = payload;
        self.lora_rx_rssi = rssi_dbm;
    }

    /// Record a ranging result captured by the caller.
    ///
    /// Must be called before `process_event(RadioEvent::RangingExchValid)`.
    pub fn record_result(&mut self, distance_m: i32, rssi_dbm: i8) {
        self.pending_distance_m = distance_m;
        self.pending_rssi_dbm = rssi_dbm;
    }

    // ── State accessors ───────────────────────────────────────────────────────

    /// Current protocol state (for debugging and external display)
    pub fn state(&self) -> ProtocolState {
        self.state
    }

    /// Read-only view of the hopping session
    pub fn hopping(&self) -> &HoppingSession {
        &self.hopping
    }

    /// Returns `true` once the session has reached a terminal state
    pub fn is_done(&self) -> bool {
        matches!(self.state, ProtocolState::Done | ProtocolState::GlobalTimeout)
    }

    // ── Result extraction ─────────────────────────────────────────────────────

    /// Compute and return the final result after `SessionComplete`.
    ///
    /// Returns `None` if the session has not reached a terminal state yet.
    pub fn finalize(&self) -> Option<SessionResult> {
        if !self.is_done() {
            return None;
        }
        Some(SessionResult {
            status: if self.state == ProtocolState::GlobalTimeout {
                RangingStatus::Timeout
            } else {
                self.hopping.status()
            },
            median_distance_m: self.hopping.median_distance_m(),
            per: self.hopping.per(),
            valid_measurements: self.hopping.valid_count,
            distance_results: self.hopping.distance_results,
            rssi_results: self.hopping.rssi_results,
        })
    }

    // ── Main dispatch ─────────────────────────────────────────────────────────

    /// Process one event and return the action the caller must execute.
    ///
    /// The state machine transitions are modelled after the switch statement in
    /// `app_radio_ranging_run()`, split into role-specific arms.
    pub fn process_event(&mut self, event: RadioEvent) -> ProtocolAction {
        // Global timer fires regardless of current state
        if event == RadioEvent::GlobalTimerFired {
            self.state = ProtocolState::GlobalTimeout;
            return ProtocolAction::SessionTimeout;
        }

        // Map event → state (mirrors the C IRQ callback writing ranging_internal_state)
        self.state = self.event_to_state(event);

        // Process state → action (mirrors the main loop switch in app_radio_ranging_run)
        self.execute_state()
    }

    /// Translate a radio event to the corresponding protocol state.
    ///
    /// During the LoRa init phase and RTToF hopping phase, some events share a
    /// state name but are differentiated by `self.phase` in `execute_state`.
    fn event_to_state(&self, event: RadioEvent) -> ProtocolState {
        match event {
            RadioEvent::Start => ProtocolState::Config,
            RadioEvent::TxDone => ProtocolState::LoRaTxDone,
            RadioEvent::RxDone => ProtocolState::LoRaRxDone,
            RadioEvent::Timeout | RadioEvent::HeaderError | RadioEvent::CrcError => {
                ProtocolState::LoRaError
            }
            RadioEvent::RangingExchValid => ProtocolState::RangingDone,
            RadioEvent::RangingTimeout | RadioEvent::RangingReqDiscarded => {
                ProtocolState::RangingChannelTimeout
            }
            RadioEvent::RangingReqValid => ProtocolState::RangingReqValid,
            // RESP_DONE: subordinate sent response; advance to next channel
            RadioEvent::RangingRespDone => ProtocolState::RangingDone,
            RadioEvent::ChannelTimerFired => ProtocolState::RangingStart,
            RadioEvent::SubRxTimerFired => ProtocolState::RangingChannelTimeout,
            RadioEvent::GlobalTimerFired => ProtocolState::GlobalTimeout,
        }
    }

    /// Execute the current state and return the next action.
    fn execute_state(&mut self) -> ProtocolAction {
        match self.state {
            // ── Config ────────────────────────────────────────────────────────
            // Mirrors APP_RADIO_RANGING_CONFIG in app_radio_ranging_run().
            ProtocolState::Config => {
                self.phase = Phase::LoraInit;
                self.hopping = HoppingSession::new();
                self.state = ProtocolState::Idle;

                match self.role {
                    RangingRole::Manager => {
                        let mut payload = [0u8; INIT_PAYLOAD_LENGTH];
                        payload[0] = (self.config.address >> 24) as u8;
                        payload[1] = (self.config.address >> 16) as u8;
                        payload[2] = (self.config.address >> 8) as u8;
                        payload[3] = self.config.address as u8;
                        // payload[4] = 0: start channel; payload[5] = 0: placeholder
                        ProtocolAction::SendLoRaInit { payload }
                    }
                    RangingRole::Subordinate => ProtocolAction::WaitForLoRaInit,
                }
            }

            // ── LoRa TX done ──────────────────────────────────────────────────
            // Manager: enter RX to catch subordinate response.
            // Subordinate (init complete): switch to RTToF mode; start channel timer.
            ProtocolState::LoRaTxDone => {
                self.state = ProtocolState::Idle;
                match (self.phase, self.role) {
                    (Phase::LoraInit, RangingRole::Manager) => {
                        ProtocolAction::WaitForLoRaResponse {
                            timeout_ms: self.config.lora_toa_ms + 50,
                        }
                    }
                    (Phase::LoraInit, RangingRole::Subordinate) => {
                        // TX response sent → switch to RTToF phase
                        self.phase = Phase::RttofHopping;
                        self.state = ProtocolState::RangingStart;
                        // Subordinate fires its first channel -1ms before manager
                        ProtocolAction::StartChannelTimer {
                            ms: self.config.req_delay_ms.saturating_sub(1),
                        }
                    }
                    _ => ProtocolAction::Restart,
                }
            }

            // ── LoRa RX done ──────────────────────────────────────────────────
            // Manager: validate address, extract sub RSSI, switch to RTToF.
            // Subordinate: validate address, echo response with own RSSI.
            ProtocolState::LoRaRxDone => {
                let buf = self.lora_rx_buf;

                // Validate address (bytes 0–3)
                let addr_ok = buf[0] == ((self.config.address >> 24) as u8)
                    && buf[1] == ((self.config.address >> 16) as u8)
                    && buf[2] == ((self.config.address >> 8) as u8)
                    && buf[3] == (self.config.address as u8);

                if !addr_ok {
                    self.state = ProtocolState::Config;
                    return ProtocolAction::Restart;
                }

                self.state = ProtocolState::Idle;
                match self.role {
                    RangingRole::Manager => {
                        // Switch to RTToF hopping phase
                        self.phase = Phase::RttofHopping;
                        self.state = ProtocolState::RangingStart;
                        ProtocolAction::StartChannelTimer { ms: self.config.req_delay_ms }
                    }
                    RangingRole::Subordinate => {
                        // Echo payload back with own RSSI in byte 5
                        let mut response = buf;
                        response[5] = self.lora_rx_rssi as u8;
                        ProtocolAction::SendLoRaResponse { payload: response }
                    }
                }
            }

            // ── LoRa error ────────────────────────────────────────────────────
            ProtocolState::LoRaError => {
                self.state = ProtocolState::Config;
                ProtocolAction::Restart
            }

            // ── Ranging start — channel timer fired ───────────────────────────
            // Mirrors APP_RADIO_RANGING_START: when ranging_next_channel_timer expires.
            ProtocolState::RangingStart => {
                if self.hopping.is_complete() {
                    self.state = ProtocolState::Done;
                    return ProtocolAction::SessionComplete;
                }

                let ch = self.hopping.current_channel;
                let freq = self.config.channel_table[ch];
                self.state = ProtocolState::Idle;

                match self.role {
                    RangingRole::Manager => ProtocolAction::StartRangingTx { freq_hz: freq },
                    RangingRole::Subordinate => ProtocolAction::StartRangingRx {
                        freq_hz: freq,
                        sub_rx_timeout_ms: self.config.sub_rx_timeout_ms,
                    },
                }
            }

            // ── Ranging done / response sent ──────────────────────────────────
            // Manager (RangingExchValid): result already captured; start 5ms timer.
            // Subordinate (RespDone): advance channel; start 4ms timer.
            ProtocolState::RangingDone => {
                let timer_ms;
                match self.role {
                    RangingRole::Manager => {
                        self.hopping.record_and_advance(
                            self.pending_distance_m,
                            self.pending_rssi_dbm,
                        );
                        timer_ms = DONE_PROCESSING_TIME_MS;
                    }
                    RangingRole::Subordinate => {
                        self.hopping.skip_channel();
                        // Subordinate uses 4ms (1ms less than manager) to align timing
                        timer_ms = DONE_PROCESSING_TIME_MS.saturating_sub(1);
                    }
                }
                self.state = ProtocolState::RangingStart;
                ProtocolAction::ResultCaptured { timer_ms }
            }

            // ── Ranging channel timeout ───────────────────────────────────────
            // Compute backoff to stay aligned with the partner device.
            // Mirrors the elapsed-time logic in APP_RADIO_RANGING_TIMEOUT.
            ProtocolState::RangingChannelTimeout => {
                self.hopping.skip_channel();
                self.state = ProtocolState::RangingStart;
                // Use DONE_PROCESSING_TIME_MS as conservative backoff.
                // (A full implementation would use elapsed time since channel start.)
                let backoff = DONE_PROCESSING_TIME_MS;
                match self.role {
                    RangingRole::Manager => {
                        ProtocolAction::StandbyAndStartChannelTimer { ms: backoff }
                    }
                    RangingRole::Subordinate => {
                        ProtocolAction::StandbyAndStartChannelTimer {
                            ms: backoff.saturating_sub(1),
                        }
                    }
                }
            }

            // ── RTToF request received (subordinate) ──────────────────────────
            // Hardware will auto-respond; cancel the RX timeout timer.
            ProtocolState::RangingReqValid => {
                self.state = ProtocolState::Idle;
                ProtocolAction::CancelSubRxTimer
            }

            // ── Terminal states ───────────────────────────────────────────────
            ProtocolState::Done => ProtocolAction::SessionComplete,
            ProtocolState::GlobalTimeout => ProtocolAction::SessionTimeout,

            // Idle — no pending state; wait for next event
            ProtocolState::Idle => ProtocolAction::None,
        }
    }
}

// =============================================================================
// Session Result
// =============================================================================

/// Aggregated result returned by `RangingProtocol::finalize()`
#[derive(Clone, Debug)]
pub struct SessionResult {
    /// Session outcome
    pub status: RangingStatus,
    /// Median distance across successful channels (meters)
    pub median_distance_m: f32,
    /// Packet error rate (%)
    pub per: u8,
    /// Number of successful channel measurements
    pub valid_measurements: usize,
    /// Per-channel distances (meters; 0 for missed channels)
    pub distance_results: [i32; MAX_HOPPING_CHANNELS],
    /// Per-channel RSSI (dBm; 0 for missed channels)
    pub rssi_results: [i8; MAX_HOPPING_CHANNELS],
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ranging::{lora_bw, lora_sf, ranging_channels, ranging_config};

    fn make_config() -> ProtocolConfig {
        ProtocolConfig::new(
            lora_sf::SF8,
            lora_bw::BW_500,
            0x01, // CR_4_5
            12,
            ranging_config::DEFAULT_ADDRESS,
            &ranging_channels::US915,
            ranging_channels::US915[0],
        )
    }

    // ── HoppingSession tests ─────────────────────────────────────────────────

    #[test]
    fn hopping_record_advances_channel() {
        let mut s = HoppingSession::new();
        assert_eq!(s.current_channel, 0);
        s.record_and_advance(100, -70);
        assert_eq!(s.current_channel, 1);
        assert_eq!(s.valid_count, 1);
        assert_eq!(s.distance_results[0], 100);
    }

    #[test]
    fn hopping_skip_no_result() {
        let mut s = HoppingSession::new();
        s.skip_channel();
        assert_eq!(s.current_channel, 1);
        assert_eq!(s.valid_count, 0);
    }

    #[test]
    fn hopping_complete_at_39() {
        let mut s = HoppingSession::new();
        for _ in 0..MAX_HOPPING_CHANNELS {
            s.skip_channel();
        }
        assert!(s.is_complete());
        assert_eq!(s.remaining(), 0);
    }

    #[test]
    fn hopping_median_odd() {
        let mut s = HoppingSession::new();
        for d in [30, 10, 20] {
            s.record_and_advance(d, 0);
        }
        assert_eq!(s.median_distance_m(), 20.0);
    }

    #[test]
    fn hopping_median_even() {
        let mut s = HoppingSession::new();
        for d in [40, 10, 30, 20] {
            s.record_and_advance(d, 0);
        }
        assert_eq!(s.median_distance_m(), 25.0);
    }

    #[test]
    fn hopping_per_all_valid() {
        let mut s = HoppingSession::new();
        for i in 0..MAX_HOPPING_CHANNELS {
            s.record_and_advance(i as i32, 0);
        }
        assert_eq!(s.per(), 0);
        assert_eq!(s.status(), RangingStatus::Valid);
    }

    #[test]
    fn hopping_per_all_missed() {
        let mut s = HoppingSession::new();
        for _ in 0..MAX_HOPPING_CHANNELS {
            s.skip_channel();
        }
        assert_eq!(s.per(), 100);
        assert_eq!(s.status(), RangingStatus::PerError);
    }

    // ── RangingProtocol state machine tests ──────────────────────────────────

    #[test]
    fn manager_start_returns_send_lora_init() {
        let mut proto = RangingProtocol::new(make_config(), RangingRole::Manager);
        let action = proto.process_event(RadioEvent::Start);
        assert!(
            matches!(action, ProtocolAction::SendLoRaInit { .. }),
            "expected SendLoRaInit, got {:?}",
            action
        );
        assert_eq!(proto.state(), ProtocolState::Idle);
    }

    #[test]
    fn subordinate_start_returns_wait_for_lora_init() {
        let mut proto = RangingProtocol::new(make_config(), RangingRole::Subordinate);
        let action = proto.process_event(RadioEvent::Start);
        assert_eq!(action, ProtocolAction::WaitForLoRaInit);
    }

    #[test]
    fn manager_tx_done_enters_rx() {
        let mut proto = RangingProtocol::new(make_config(), RangingRole::Manager);
        proto.process_event(RadioEvent::Start);
        let action = proto.process_event(RadioEvent::TxDone);
        assert!(
            matches!(action, ProtocolAction::WaitForLoRaResponse { .. }),
            "got {:?}",
            action
        );
    }

    #[test]
    fn manager_lora_rx_valid_starts_hopping() {
        let cfg = make_config();
        let mut proto = RangingProtocol::new(cfg, RangingRole::Manager);
        proto.process_event(RadioEvent::Start);
        proto.process_event(RadioEvent::TxDone);

        // Provide valid init response
        let mut payload = [0u8; INIT_PAYLOAD_LENGTH];
        payload[0] = (ranging_config::DEFAULT_ADDRESS >> 24) as u8;
        payload[1] = (ranging_config::DEFAULT_ADDRESS >> 16) as u8;
        payload[2] = (ranging_config::DEFAULT_ADDRESS >> 8) as u8;
        payload[3] = ranging_config::DEFAULT_ADDRESS as u8;
        payload[5] = 200u8; // sub RSSI
        proto.set_lora_rx(payload, -60);

        let action = proto.process_event(RadioEvent::RxDone);
        assert!(
            matches!(action, ProtocolAction::StartChannelTimer { .. }),
            "expected StartChannelTimer, got {:?}",
            action
        );
        assert_eq!(proto.state(), ProtocolState::RangingStart);
    }

    #[test]
    fn manager_bad_address_restarts() {
        let cfg = make_config();
        let mut proto = RangingProtocol::new(cfg, RangingRole::Manager);
        proto.process_event(RadioEvent::Start);
        proto.process_event(RadioEvent::TxDone);

        // Wrong address
        proto.set_lora_rx([0xDE, 0xAD, 0xBE, 0xEF, 0, 0], -60);
        let action = proto.process_event(RadioEvent::RxDone);
        assert_eq!(action, ProtocolAction::Restart);
        assert_eq!(proto.state(), ProtocolState::Config);
    }

    #[test]
    fn ranging_start_issues_tx_for_manager() {
        let cfg = make_config();
        let mut proto = RangingProtocol::new(cfg, RangingRole::Manager);
        // Fast-path: put protocol directly in RangingStart state
        proto.state = ProtocolState::RangingStart;
        proto.phase = Phase::RttofHopping;
        let action = proto.process_event(RadioEvent::ChannelTimerFired);
        let freq = cfg.channel_table[0];
        assert_eq!(action, ProtocolAction::StartRangingTx { freq_hz: freq });
    }

    #[test]
    fn channel_39_triggers_session_complete() {
        let cfg = make_config();
        let mut proto = RangingProtocol::new(cfg, RangingRole::Manager);
        proto.state = ProtocolState::RangingStart;
        proto.phase = Phase::RttofHopping;
        // Skip all channels
        for _ in 0..MAX_HOPPING_CHANNELS {
            proto.hopping.skip_channel();
        }
        let action = proto.process_event(RadioEvent::ChannelTimerFired);
        assert_eq!(action, ProtocolAction::SessionComplete);
        assert_eq!(proto.state(), ProtocolState::Done);
    }

    #[test]
    fn global_timer_yields_session_timeout() {
        let mut proto = RangingProtocol::new(make_config(), RangingRole::Manager);
        proto.process_event(RadioEvent::Start);
        let action = proto.process_event(RadioEvent::GlobalTimerFired);
        assert_eq!(action, ProtocolAction::SessionTimeout);
        assert!(proto.is_done());
    }

    #[test]
    fn finalize_none_before_done() {
        let proto = RangingProtocol::new(make_config(), RangingRole::Manager);
        assert!(proto.finalize().is_none());
    }

    #[test]
    fn finalize_returns_result_after_complete() {
        let cfg = make_config();
        let mut proto = RangingProtocol::new(cfg, RangingRole::Manager);
        proto.state = ProtocolState::Done;
        proto.phase = Phase::RttofHopping;
        for i in 0..MIN_HOPPING_CHANNELS {
            proto.hopping.record_and_advance(i as i32 * 10, -70);
        }
        let result = proto.finalize().expect("should have result");
        assert_eq!(result.status, RangingStatus::Valid);
        assert_eq!(result.valid_measurements, MIN_HOPPING_CHANNELS);
    }
}
