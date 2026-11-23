// SPDX-License-Identifier: GPL-3.0-or-later
//! Hardware adapter. Contains no protocol, phase or safety logic: it only
//! wires Embassy to the testable core in `potok_m_firmware`.
//!
//! Pin numbers, AF mappings, DMA streams and servo wiring are intentionally
//! absent. They must be filled from the schematic before the first flash.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

/// The typed pitot/static seam lives beside this adapter. No pin is claimed
/// here: the board implements `Airdata`, the sampler derives Mach from it.
#[cfg(target_os = "none")]
#[path = "shield_sample.rs"]
mod shield_sample;

#[cfg(target_os = "none")]
mod hw {
    use crate::shield_sample::Airdata;
    use embassy_stm32::mode::Async;
    use embassy_stm32::usart::{UartRx, UartTx};
    use potok_m_firmware::{
        board::Board,
        phase::PhaseEvent,
        uart::{DoubleBufferRx, FrameTx, UartError},
        watchdog::Iwdg,
        Command,
    };

    /// TODO(schematic): DMA idle-line receive into two static slots. The
    /// completion callback must publish the filled slot to a fixed-depth
    /// queue; `take` then only copies and re-arms.
    pub struct Rx<'d> {
        _uart: UartRx<'d, Async>,
    }

    impl DoubleBufferRx for Rx<'_> {
        fn take(&mut self, _out: &mut [u8]) -> Result<Option<usize>, UartError> {
            todo!("UART RX DMA: assign pins/streams and wire idle-line double buffering")
        }
    }

    /// TODO(schematic): await the DMA write on the UART TX stream.
    pub struct Tx<'d> {
        _uart: UartTx<'d, Async>,
    }

    impl FrameTx for Tx<'_> {
        fn send(&mut self, _frame: &[u8]) -> Result<(), UartError> {
            todo!("UART TX DMA: assign pins/streams and await the write")
        }
    }

    pub struct HwBoard<'d> {
        rx: Rx<'d>,
        tx: Tx<'d>,
        wd: Iwdg<'d>,
    }

    pub fn build_board(wd: Iwdg<'static>) -> HwBoard<'static> {
        // TODO(schematic): choose USART instance, TX/RX pins and alternate
        // function, the two DMA streams, and the time-driver timer; then
        // `Uart::new(...).split()` and construct HwBoard. Until this is done
        // the firmware cannot receive requests and cannot actuate anything.
        let _ = wd;
        todo!("assign UART pins/DMA streams from the board schematic")
    }

    impl Board for HwBoard<'static> {
        type Rx = Rx<'static>;
        type Tx = Tx<'static>;
        type Wd = Iwdg<'static>;

        fn rx(&mut self) -> &mut Self::Rx {
            &mut self.rx
        }
        fn tx(&mut self) -> &mut Self::Tx {
            &mut self.tx
        }
        fn wd(&mut self) -> &mut Self::Wd {
            &mut self.wd
        }
        fn now_us(&self) -> u32 {
            embassy_time::Instant::now().as_micros() as u32
        }
        fn sensor_flags(&self) -> u8 {
            // TODO(schematic): IMU/baro/mag/pitot health bits from sensor drivers.
            0
        }
        fn motor_armed(&self) -> bool {
            // TODO(schematic): read the motor-armed sense input.
            false
        }
        fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
            // TODO(schematic): derive Burnout/Apogee/Descent/Touchdown from the
            // sensor drivers. Until wired, the FSM stays in BURN and AI is off.
            None
        }
        fn apply_command(&mut self, cmd: Command) {
            // TODO(schematic): servo PWM pins. No actuation until the pin map
            // and the BURN gate have been validated on the bench.
            let _ = cmd;
        }
    }

    impl Airdata for HwBoard<'static> {
        fn pitot_dp_pa(&self) -> Option<f32> {
            // TODO(schematic): pitot differential-pressure driver. No pin is
            // claimed here; until it exists the reading is absent, not zero.
            None
        }
        fn static_pa(&self) -> Option<f32> {
            // TODO(schematic): barometric static-pressure driver. Same rule.
            None
        }
    }
}

#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::error!("firmware panic; resetting");
    cortex_m::peripheral::SCB::sys_reset()
}

#[cfg(target_os = "none")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    use embassy_time::{Duration, Ticker};
    use potok_m_firmware::{
        interlocks::Interlocks,
        scheduler::TICK_HZ,
        step,
        telemetry_log::{Cadence, Emission, LogGate},
        watchdog::{setup_iwdg, WatchdogPolicy},
        LinkState, PhaseFsm, StepResult,
    };

    use crate::shield_sample::ShieldSampler;

    use defmt_rtt as _;

    /// Explicit debug opt-in for every-tick airdata logging. Off in flight:
    /// `Cadence::debug_every_tick()` is the only path to a 500 Hz log and this
    /// flag is the only switch that selects it.
    const DEBUG_EVERY_TICK: bool = false;

    /// Qualitative airdata state used as the gate key: which of the six
    /// reasons are active, never their growing tallies. A tally changes every
    /// tick with no pitot driver and would make every tick a state change.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct AirdataState {
        shield: u8,
        airdata: u8,
    }

    /// A latch warning is immediate on entry into the latched state; the
    /// steady latched ticks are suppressed and counted by the gate.
    fn latch_entered(gate: &mut LogGate<bool>, latched: bool) -> bool {
        latched && matches!(gate.observe_error(latched), Emission::Safety)
    }

    let p = embassy_stm32::init(Default::default());
    let wd = setup_iwdg(p.IWDG1);
    let mut board = hw::build_board(wd);
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();
    // Independent safety gates live in the shipping loop, not just in SIL.
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    let mut sampler = ShieldSampler::new();
    let mut ticker = Ticker::every(Duration::from_millis(2)); // 500 Hz

    // One gate per message: each owns its own dedupe, cadence and suppression
    // count. A cyclic cadence of TICK_HZ is one summary per second.
    let airdata_cadence = if DEBUG_EVERY_TICK {
        Cadence::debug_every_tick()
    } else {
        Cadence::periodic(TICK_HZ)
    };
    let mut airdata_log = LogGate::<AirdataState>::new(airdata_cadence);
    let mut reject_log = LogGate::<()>::new(Cadence::periodic(TICK_HZ));
    let mut applied_log = LogGate::<()>::new(Cadence::periodic(TICK_HZ));
    let mut ai_off_log = LogGate::<bool>::new(Cadence::on_change());
    let mut interlock_off_log = LogGate::<bool>::new(Cadence::on_change());
    let mut deadman_log = LogGate::<bool>::new(Cadence::on_change());

    loop {
        ticker.next().await;
        link.set_shield_sample(sampler.sample(&board));
        match step(
            &mut board,
            &mut fsm,
            &mut link,
            &mut interlock,
            &mut policy,
            true,
        ) {
            StepResult::FrameRejected => {
                if reject_log.observe(()).should_emit() {
                    defmt::warn!(
                        "reply frame rejected (total={} suppressed={})",
                        link.counters.rejected,
                        reject_log.suppressed()
                    );
                }
            }
            StepResult::Accepted(cmd) if cmd.is_zero() => {
                let u = sampler.unavailable_counts();
                let bit = |count: u32| (count > 0) as u8;
                let key = AirdataState {
                    shield: bit(link.shield_reasons.mach_limit)
                        | (bit(link.shield_reasons.pitot_invalid) << 1)
                        | (bit(link.shield_reasons.non_finite_mach) << 2),
                    airdata: bit(u.no_driver)
                        | (bit(u.non_finite) << 1)
                        | (bit(u.non_positive) << 2),
                };
                if airdata_log.observe(key).should_emit() {
                    defmt::info!(
                        "zero command (shield: mach={} pitot={} nonfinite={} airdata: nodrv={} nonfin={} nonpos={} suppressed={})",
                        link.shield_reasons.mach_limit,
                        link.shield_reasons.pitot_invalid,
                        link.shield_reasons.non_finite_mach,
                        u.no_driver,
                        u.non_finite,
                        u.non_positive,
                        airdata_log.suppressed()
                    );
                }
            }
            StepResult::Accepted(_) => {
                if applied_log.observe(()).should_emit() {
                    defmt::info!(
                        "AI command applied (suppressed={})",
                        applied_log.suppressed()
                    );
                }
            }
            StepResult::Idle => {}
        }
        // Safety latches stay immediate on entry; the steady latched ticks
        // are suppressed and counted instead of streamed at 500 Hz.
        if latch_entered(&mut ai_off_log, link.is_disabled() || policy.ai_latched()) {
            defmt::warn!("AI latched off; explicit reset required");
        }
        if latch_entered(&mut interlock_off_log, interlock.latched().is_some()) {
            defmt::warn!("independent interlock latched; explicit reset required");
        }
        if latch_entered(&mut deadman_log, policy.deadman().tripped()) {
            defmt::warn!("control deadline monitor tripped; watchdog feed withheld");
        }
    }
}
