//! Independent watchdog: abstract feed, hardware setup, feed policy.

/// Feed policy: the control task may feed only after a complete cycle with a
/// valid STM32-owned phase, and only while the deadman has not tripped.
pub const fn may_feed(cycle_complete: bool, deadman_tripped: bool) -> bool {
    cycle_complete && !deadman_tripped
}

pub trait Watchdog {
    fn pet(&mut self);
}

pub const WATCHDOG_TIMEOUT_US: u32 = 100_000;

#[cfg(target_os = "none")]
pub type Iwdg<'d> = embassy_stm32::wdg::IndependentWatchdog<'d, embassy_stm32::peripherals::IWDG1>;

/// Configures and starts IWDG. Once unleashed, only a reset stops it.
#[cfg(target_os = "none")]
pub fn setup_iwdg(
    instance: embassy_stm32::Peri<'static, embassy_stm32::peripherals::IWDG1>,
) -> Iwdg<'static> {
    let mut wdg = embassy_stm32::wdg::IndependentWatchdog::new(instance, WATCHDOG_TIMEOUT_US);
    wdg.unleash();
    wdg
}

#[cfg(target_os = "none")]
impl Watchdog for Iwdg<'_> {
    fn pet(&mut self) {
        embassy_stm32::wdg::IndependentWatchdog::pet(self);
    }
}
