// SPDX-License-Identifier: GPL-3.0-or-later
//! Renode smoke firmware: a no-panic, emulator-only board adapter around the
//! *real* host-testable control core. It is built for `thumbv7em-none-eabihf`
//! and loaded by `emulator/renode/smoke.resc`.
//!
//! This is NOT the flight binary. `firmware/src/main.rs` still contains the
//! schematic-bound unimplemented hardware adapter; this crate deliberately
//! does not touch it. No pin, UART, DMA channel or sensor is claimed here: every
//! unmapped capability is answered with a typed `Unavailable` and a tally.
//!
//! Result protocol (DTCM, little-endian u32 words at 0x2001_F800):
//!   0 stage, 1 panic source line, 2 ticks, 3 watchdog pets,
//!   4 counted Unavailable calls, 5 sentinel 0x52_45_4E_4F ("RENO").
//! Stage 0xA5A5_A5A5 is PASS, 0xDEAD_DEAD is a panic, 0xF011_F011 is a
//! failed in-firmware check. Renode reads word 0 and fails the run unless it
//! is PASS. Nothing here is a claim about real hardware.

#![no_std]
#![no_main]

use core::panic::PanicInfo;

// ---------------------------------------------------------------------------
// Register-window MMIO. Deterministic, no HAL, no interrupt, no clock read.
// ---------------------------------------------------------------------------

const fn mmio(addr: usize) -> *mut u32 {
    addr as *mut u32
}

#[inline(always)]
fn rd(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(mmio(addr)) }
}

#[inline(always)]
fn wr(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(mmio(addr), value) }
}

const RCC: usize = 0x5802_4400;
const RCC_AHB4ENR: usize = RCC + 0xE0;
const RCC_APB2ENR: usize = RCC + 0xF0;
const GPIO_AHB4_BIT: u32 = 1 << 0; // GPIOAEN
const USART1_APB2_BIT: u32 = 1 << 4; // USART1EN

const GPIOA: usize = 0x5802_0000;
const GPIOA_MODER: usize = GPIOA;
const GPIOA_ODR: usize = GPIOA + 0x14;
const GPIOA_PIN5_OUTPUT: u32 = 1 << (5 * 2);

const USART1: usize = 0x4001_1000;
const USART1_CR1: usize = USART1;
const USART1_BRR: usize = USART1 + 0x08;
const USART1_ISR: usize = USART1 + 0x1C;
const USART1_TDR: usize = USART1 + 0x28;
const USART1_TXE: u32 = 1 << 7;
// Mantissa 108 / fraction 0. Renode's USART model ignores the fraction bits;
// using a whole mantissa keeps the smoke log free of unsupported-bit warnings.
const USART1_BRR_115200_AT_200MHZ: u32 = 108 << 4;
const USART1_CR1_UE_TE: u32 = (1 << 0) | (1 << 3);

const IWDG: usize = 0x5800_4800;
const IWDG_KR: usize = IWDG;
const IWDG_PR: usize = IWDG + 0x04;
const IWDG_RLR: usize = IWDG + 0x08;
const IWDG_KEY_UNLOCK: u32 = 0x5555;
const IWDG_KEY_START: u32 = 0xCCCC;
const IWDG_KEY_FEED: u32 = 0xAAAA;

// ---------------------------------------------------------------------------
// Result marker. The linker map keeps the stack below 0x2001_F000.
// ---------------------------------------------------------------------------

const SMK: usize = 0x2001_F800;
const W_STAGE: usize = 0;
const W_PANIC_LINE: usize = 4;
const W_TICKS: usize = 8;
const W_PETS: usize = 12;
const W_UNAVAIL: usize = 16;
const W_SENTINEL: usize = 20;
const SENTINEL: u32 = 0x5245_4E4F;

const STAGE_BOOT: u32 = 1;
const STAGE_CLOCK: u32 = 2;
const STAGE_GPIO: u32 = 3;
const STAGE_UART: u32 = 4;
const STAGE_TICK: u32 = 5;
const STAGE_WDGF: u32 = 6;
const STAGE_PASS: u32 = 0xA5A5_A5A5;
const STAGE_PANIC: u32 = 0xDEAD_DEAD;
const STAGE_CHECK_FAILED: u32 = 0xF011_F011;

#[inline(always)]
fn mark(stage: u32) {
    wr(SMK + W_STAGE, stage);
}

#[inline(always)]
fn record(offset: usize, value: u32) {
    wr(SMK + offset, value);
}

// ---------------------------------------------------------------------------
// Peripheral setup. Each stage is a plain register write sequence; the result
// marker is the proof that the sequence completed without a fault.
// ---------------------------------------------------------------------------

fn init_clocks() {
    wr(RCC_AHB4ENR, GPIO_AHB4_BIT);
    wr(RCC_APB2ENR, USART1_APB2_BIT);
}

fn init_gpio() {
    wr(GPIOA_MODER, GPIOA_PIN5_OUTPUT);
    wr(GPIOA_ODR, 1 << 5);
}

fn init_uart() {
    wr(USART1_BRR, USART1_BRR_115200_AT_200MHZ);
    wr(USART1_CR1, USART1_CR1_UE_TE);
}

fn start_iwdg() {
    wr(IWDG_KR, IWDG_KEY_UNLOCK); // 0x5555: unlock PR/RLR writes
    wr(IWDG_PR, 0x07); // /256
    wr(IWDG_RLR, 0x0FFF); // ~18.7 s at 56 kHz LSI: the smoke never times out.
    wr(IWDG_KR, IWDG_KEY_START); // 0xCCCC: unleash
}

fn feed_iwdg() {
    wr(IWDG_KR, IWDG_KEY_FEED); // 0xAAAA: reload
}

fn uart_puts(bytes: &[u8]) {
    for &byte in bytes {
        let mut spin: u32 = 0;
        while rd(USART1_ISR) & USART1_TXE == 0 && spin < 100_000 {
            spin += 1;
            core::hint::spin_loop();
        }
        wr(USART1_TDR, u32::from(byte));
    }
}

// ---------------------------------------------------------------------------
// Emulator board adapter. Feature-gated: it is not a hardware board and it
// never reaches an unimplemented branch. Unmapped pins/capabilities count as typed
// `Unavailable`, never faked as a successful read or a sent frame.
// ---------------------------------------------------------------------------

#[cfg(feature = "emulator-board")]
mod emulator_board {
    use core::cell::Cell;
    use potok_m_firmware::{
        board::Board,
        phase::PhaseEvent,
        uart::{DoubleBufferRx, FrameTx, UartError},
        watchdog::Watchdog,
        Capability, CapabilityLedger, Command, FlightBoard, Unavailable,
    };

    pub struct EmulatorRx;

    impl DoubleBufferRx for EmulatorRx {
        fn take(&mut self, _out: &mut [u8]) -> Result<Option<usize>, UartError> {
            // No DMA channel is claimed: never invent a frame.
            Ok(None)
        }
    }

    pub struct EmulatorTx;

    impl FrameTx for EmulatorTx {
        fn send(&mut self, _frame: &[u8]) -> Result<(), UartError> {
            // No TX pin is claimed: the caller sees the rejection.
            Err(UartError::WouldBlock)
        }
    }

    pub struct EmulatorWd {
        pets: u32,
    }

    impl EmulatorWd {
        const fn new() -> Self {
            Self { pets: 0 }
        }
    }

    impl Watchdog for EmulatorWd {
        fn pet(&mut self) {
            self.pets += 1;
            super::feed_iwdg();
        }
    }

    pub struct EmulatorBoard {
        rx: EmulatorRx,
        tx: EmulatorTx,
        wd: EmulatorWd,
        now_us: Cell<u32>,
        capabilities: CapabilityLedger,
    }

    impl EmulatorBoard {
        pub fn new() -> Self {
            Self {
                rx: EmulatorRx,
                tx: EmulatorTx,
                wd: EmulatorWd::new(),
                now_us: Cell::new(0),
                capabilities: CapabilityLedger::default(),
            }
        }

        /// Synthetic 500 Hz time: the smoke never reads a device timer, so it
        /// is reproducible across Renode versions and host machines.
        pub fn advance_tick(&self) {
            self.now_us
                .set(self.now_us.get().wrapping_add(super::CONTROL_PERIOD_US));
        }

        pub fn wd_pets(&self) -> u32 {
            self.wd.pets
        }

        pub fn unavailable(&self) -> u32 {
            self.capabilities.total_unavailable()
        }
    }

    impl Board for EmulatorBoard {
        type Rx = EmulatorRx;
        type Tx = EmulatorTx;
        type Wd = EmulatorWd;

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
            // No sensor driver: no health bit is asserted.
            0
        }
        fn motor_armed(&self) -> bool {
            self.capabilities
                .note_unavailable(Capability::MotorArmedSense, Unavailable::NoDriver);
            false
        }
        fn poll_phase_event(&mut self) -> Option<PhaseEvent> {
            self.capabilities
                .note_unavailable(Capability::PhaseSense, Unavailable::NoDriver);
            None
        }
        fn apply_command(&mut self, _cmd: Command) {
            self.capabilities
                .note_unavailable(Capability::Actuation, Unavailable::NoDriver);
        }
    }

    impl FlightBoard for EmulatorBoard {}
}

#[cfg(not(feature = "emulator-board"))]
compile_error!("the Renode smoke binary requires the `emulator-board` feature");

const CONTROL_PERIOD_US: u32 = 2_000;
const TICKS: u32 = 8;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Clear the whole result frame first so a stale Renode run cannot read an
    // old PASS word if this image stops early.
    for off in [
        W_STAGE,
        W_PANIC_LINE,
        W_TICKS,
        W_PETS,
        W_UNAVAIL,
        W_SENTINEL,
    ] {
        record(off, 0);
    }
    mark(STAGE_BOOT);

    init_clocks();
    mark(STAGE_CLOCK);
    init_gpio();
    mark(STAGE_GPIO);
    init_uart();
    start_iwdg();
    mark(STAGE_UART);

    #[cfg(feature = "emulator-board")]
    {
        use crate::emulator_board::EmulatorBoard;
        use potok_m_firmware::{
            board::Board,
            interlocks::Interlocks,
            scheduler::TICK_HZ,
            step,
            watchdog::{WatchdogPolicy, CONTROL_PERIOD_US as CORE_CONTROL_PERIOD_US},
            FlightPath, LinkState, PhaseFsm,
        };

        const _: () = {
            assert!(TICK_HZ == 500);
            assert!(CORE_CONTROL_PERIOD_US == CONTROL_PERIOD_US);
        };

        let mut board = EmulatorBoard::new();
        let mut fsm = PhaseFsm::new();
        let mut link = LinkState::new();
        let mut interlock = Interlocks::default();
        let mut policy = WatchdogPolicy::new();
        let mut flight = FlightPath::new();

        for index in 0..TICKS {
            let now = board.now_us();
            flight.tick(&mut board, fsm.phase(), now, CONTROL_PERIOD_US);
            let _ = step(
                &mut board,
                &mut fsm,
                &mut link,
                &mut interlock,
                &mut policy,
                true,
            );
            record(W_TICKS, index + 1);
            board.advance_tick();
        }
        mark(STAGE_TICK);

        let pets = board.wd_pets();
        let unavailable = board.unavailable();
        record(W_PETS, pets);
        record(W_UNAVAIL, unavailable);
        record(W_SENTINEL, SENTINEL);

        // In-firmware checks: the control path must have fed the watchdog and
        // every unmapped capability must have been counted, not discarded.
        if pets == 0 || unavailable == 0 {
            mark(STAGE_CHECK_FAILED);
            loop {
                cortex_m::asm::wfi();
            }
        }
        mark(STAGE_WDGF);
    }

    uart_puts(b"POTOK_RENODE_SMOKE_PASS\n");
    mark(STAGE_PASS);
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    record(W_PANIC_LINE, info.location().map_or(0, |loc| loc.line()));
    record(W_SENTINEL, SENTINEL);
    mark(STAGE_PANIC);
    // Deliberately no reset: a panic-reset would hide the failure from the
    // marker. Halt so the Renode script reads PANIC and fails loudly.
    loop {
        cortex_m::asm::wfi();
    }
}
