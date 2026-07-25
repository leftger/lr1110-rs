//! Monotonic time types for `irq-manager` modules.
//!
//! Timestamps come from a user-provided millisecond clock set via [`set_clock`].
//! Async delays use [`embedded_hal_async::delay::Delay`] passed into manager methods.

use core::cell::UnsafeCell;
use core::time::Duration;

use embedded_hal_async::delay::Delay;

fn default_now_ms() -> u64 {
    0
}

static CLOCK: UnsafeCell<fn() -> u64> = UnsafeCell::new(default_now_ms);

/// Install the monotonic millisecond clock used by [`Instant::now`].
///
/// Call once at startup. Embassy applications typically wire this to
/// `|| embassy_time::Instant::now().as_millis()`.
pub fn set_clock(now_ms: fn() -> u64) {
    unsafe {
        *CLOCK.get() = now_ms;
    }
}

fn now_ms() -> u64 {
    unsafe { (*CLOCK.get())() }
}

/// Monotonic timestamp in milliseconds.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Instant(u64);

impl Instant {
    /// Current time from the configured clock.
    pub fn now() -> Self {
        Self(now_ms())
    }

    /// Construct from raw milliseconds.
    pub const fn from_millis(ms: u64) -> Self {
        Self(ms)
    }

    /// Elapsed milliseconds since this instant.
    pub fn elapsed(&self) -> DurationMs {
        Self::now().duration_since(*self)
    }

    /// Duration from `earlier` until `self`.
    pub fn duration_since(&self, earlier: Self) -> DurationMs {
        DurationMs(self.0.saturating_sub(earlier.0))
    }

    /// Raw millisecond value.
    pub const fn as_millis(&self) -> u64 {
        self.0
    }

    /// Deadline at `self + ms`.
    pub const fn add_ms(self, ms: u32) -> Self {
        Self(self.0.saturating_add(ms as u64))
    }
}

/// Elapsed time in milliseconds.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct DurationMs(u64);

impl DurationMs {
    /// Raw millisecond value.
    pub const fn as_millis(&self) -> u64 {
        self.0
    }
}

/// Async sleep helper using [`Delay`].
pub async fn delay_ms<D: Delay>(delay: &mut D, ms: u32) {
    if ms == 0 {
        return;
    }
    let _ = delay.delay(Duration::from_millis(ms as u64)).await;
}

/// Async microsecond sleep helper using [`Delay`].
pub async fn delay_us<D: Delay>(delay: &mut D, us: u32) {
    if us == 0 {
        return;
    }
    let _ = delay.delay(Duration::from_micros(us as u64)).await;
}
