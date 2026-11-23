//! Hardware adapter. Contains no protocol, phase or safety logic: it only
//! wires Embassy to the testable core in `potok_m_firmware`.
//!
//! Pin numbers, AF mappings, DMA streams and servo wiring are intentionally
//! absent. They must be filled from the schematic before the first flash.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(not(target_os = "none"))]
fn main() {}

#[cfg(target_os = "none")]
mod hw {
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
        step,
        watchdog::{setup_iwdg, WatchdogPolicy},
        LinkState, PhaseFsm, StepResult,
    };

    use defmt_rtt as _;
    let p = embassy_stm32::init(Default::default());
    let wd = setup_iwdg(p.IWDG1);
    let mut board = hw::build_board(wd);
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();
    // Independent safety gates live in the shipping loop, not just in SIL.
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    let mut ticker = Ticker::every(Duration::from_millis(2)); // 500 Hz

    loop {
        ticker.next().await;
        match step(
            &mut board,
            &mut fsm,
            &mut link,
            &mut interlock,
            &mut policy,
            true,
        ) {
            StepResult::FrameRejected => defmt::warn!("reply frame rejected"),
            StepResult::Accepted(cmd) if cmd.is_zero() => defmt::info!("zero command"),
            StepResult::Accepted(_) => defmt::info!("AI command applied"),
            StepResult::Idle => {}
        }
        if link.is_disabled() || policy.ai_latched() {
            defmt::warn!("AI latched off; explicit reset required");
        }
        if interlock.latched().is_some() {
            defmt::warn!("independent interlock latched; explicit reset required");
        }
        if policy.deadman().tripped() {
            defmt::warn!("control deadline monitor tripped; watchdog feed withheld");
        }
    }
}
