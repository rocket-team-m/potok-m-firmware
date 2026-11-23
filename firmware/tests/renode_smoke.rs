// SPDX-License-Identifier: GPL-3.0-or-later
//! Host-side contract for the Renode smoke. Renode itself is not available in
//! the host test suite; this pins the two things that make its result word
//! meaningful: (1) the no-panic, typed-`Unavailable` board contract, and
//! (2) the marker protocol shared between the ELF and `smoke.resc`.
//!
//! The bare-metal adapter lives in `emulator/renode/firmware/src/main.rs`
//! because the flight binary's `HwBoard::build_board()` is still a schematic
//! `todo!()`. Mirroring it here keeps a core change from silently breaking the
//! smoke's assumptions.

use core::cell::Cell;
use potok_m_firmware::{
    board::Board,
    interlocks::Interlocks,
    phase::PhaseEvent,
    scheduler::TICK_HZ,
    step,
    uart::{DoubleBufferRx, FrameTx, UartError},
    watchdog::{Watchdog, WatchdogPolicy, CONTROL_PERIOD_US},
    Capability, CapabilityLedger, Command, FlightBoard, FlightPath, LinkState, PhaseFsm,
    Unavailable,
};

const SMOKE_MAIN: &str = include_str!("../../emulator/renode/firmware/src/main.rs");
const SMOKE_RESC: &str = include_str!("../../emulator/renode/smoke.resc");
const SMOKE_MEMORY: &str = include_str!("../../emulator/renode/firmware/memory.x");

struct SmokeRx;
impl DoubleBufferRx for SmokeRx {
    fn take(&mut self, _out: &mut [u8]) -> Result<Option<usize>, UartError> {
        Ok(None)
    }
}

struct SmokeTx;
impl FrameTx for SmokeTx {
    fn send(&mut self, _frame: &[u8]) -> Result<(), UartError> {
        Err(UartError::WouldBlock)
    }
}

struct SmokeWd(u32);
impl Watchdog for SmokeWd {
    fn pet(&mut self) {
        self.0 += 1;
    }
}

/// Same contract as the emulator board: no pin is claimed, every unmapped
/// capability is counted as `Unavailable`, no path reaches a `todo!()`.
struct SmokeBoard {
    rx: SmokeRx,
    tx: SmokeTx,
    wd: SmokeWd,
    now_us: Cell<u32>,
    caps: CapabilityLedger,
}

impl SmokeBoard {
    fn new() -> Self {
        Self {
            rx: SmokeRx,
            tx: SmokeTx,
            wd: SmokeWd(0),
            now_us: Cell::new(0),
            caps: CapabilityLedger::default(),
        }
    }
}

impl Board for SmokeBoard {
    type Rx = SmokeRx;
    type Tx = SmokeTx;
    type Wd = SmokeWd;

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
        self.now_us.get()
    }
    fn sensor_flags(&self) -> u8 {
        0
    }
    fn motor_armed(&self) -> bool {
        self.caps
            .note_unavailable(Capability::MotorArmedSense, Unavailable::NoDriver);
        false
    }
    fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
        self.caps
            .note_unavailable(Capability::PhaseSense, Unavailable::NoDriver);
        None
    }
    fn apply_command(&mut self, _cmd: Command) {
        self.caps
            .note_unavailable(Capability::Actuation, Unavailable::NoDriver);
    }
}

impl FlightBoard for SmokeBoard {}

#[test]
fn no_driver_board_is_counted_and_never_panics() {
    let mut board = SmokeBoard::new();
    let mut fsm = PhaseFsm::new();
    let mut link = LinkState::new();
    let mut interlock = Interlocks::default();
    let mut policy = WatchdogPolicy::new();
    let mut flight = FlightPath::new();

    assert_eq!(TICK_HZ, 500);
    assert_eq!(CONTROL_PERIOD_US, 2_000);

    for _ in 0..8 {
        let now = board.now_us.get();
        flight.tick(&mut board, fsm.phase(), now, CONTROL_PERIOD_US);
        let _ = step(
            &mut board,
            &mut fsm,
            &mut link,
            &mut interlock,
            &mut policy,
            true,
        );
        board.now_us.set(now.wrapping_add(CONTROL_PERIOD_US));
    }

    // The control path fed IWDG on the early ticks before the link deadman
    // latched from the missing replies. Renode reads the same two numbers.
    assert!(board.wd.0 > 0, "control tick must feed the watchdog");
    assert!(
        board.caps.total_unavailable() > 0,
        "unmapped pins must be counted, not silently discarded"
    );
    assert_eq!(
        board.caps.last_reason(Capability::MotorArmedSense),
        Some(Unavailable::NoDriver)
    );
    assert_eq!(
        board.caps.last_reason(Capability::PhaseSense),
        Some(Unavailable::NoDriver)
    );
    assert_eq!(
        board.caps.last_reason(Capability::Actuation),
        Some(Unavailable::NoDriver)
    );
}

#[test]
fn marker_protocol_agrees_between_elf_and_renode_script() {
    // Address and sentinel must match in the linker map, the ELF and the
    // monitor script; a drift here would make Renode read a stale word.
    assert!(SMOKE_MEMORY.contains("0x2001F000"));
    assert!(SMOKE_MAIN.contains("0x2001_F800"));
    assert!(SMOKE_RESC.contains("BASE = 0x2001F800"));
    assert!(SMOKE_MAIN.contains("const SENTINEL: u32 = 0x5245_4E4F;"));
    assert!(SMOKE_RESC.contains("0x52454E4F"));
    assert!(SMOKE_MAIN.contains("const STAGE_PASS: u32 = 0xA5A5_A5A5;"));
    assert!(SMOKE_RESC.contains("0xA5A5A5A5"));
    assert!(SMOKE_MAIN.contains("const TICKS: u32 = 8;"));
    assert!(SMOKE_RESC.contains("ticks == 8"));
    assert!(SMOKE_RESC.contains("panic_line == 0"));
    assert!(SMOKE_RESC.contains("pets > 0"));
    assert!(SMOKE_RESC.contains("unavail > 0"));
}

#[test]
fn emulator_adapter_never_uses_todo_or_panic_reset() {
    for needle in ["todo!", "unimplemented!", "SCB::sys_reset"] {
        assert!(
            !SMOKE_MAIN.contains(needle),
            "emulator firmware must not contain {needle}"
        );
    }
    assert!(SMOKE_MAIN.contains("feature = \"emulator-board\""));
}
