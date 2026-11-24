# Renode smoke (STM32H723)

A deterministic, emulator-only smoke of the Potok-M control core. It builds a
`thumbv7em-none-eabihf` ELF, loads it into an **explicit STM32H723 platform
definition** in Renode 1.17, and checks a result word in DTCM.

**This is not hardware validation.** Nothing here touches a real chip, pin,
clock, sensor, servo or radio. The flight binary still contains the
schematic-bound `HwBoard::build_board()` placeholder; this smoke deliberately
does not modify it. Passing this gate means "the control core boots and runs a
bounded tick sequence on an emulated Cortex-M7 without a fault", nothing more.

The working target part is `stm32h723zg` (single-bank 1 MiB flash, 128 KiB
DTCM); the exact variant/package is `NOT VERIFIED` until the BOM is signed. The
platform's `nvic.systickFrequency: 550_000_000` is the STM32H723 rated-clock
assumption, **NOT VERIFIED** on hardware: the smoke never reads a real clock and
no timing measurement is claimed here.

## Why the platform is not `stm32h743`

Renode 1.17 has **no built-in `stm32h723` platform**. Checked in both the local
1.17.0 install and the 1.17.1 portable tarball pinned by CI: `platforms/cpus/`
contains `stm32h7.repl`, `stm32h743.repl`, `stm32h747.repl` (1.17.0 only) and
`stm32h753.repl` — no H723. Loading the H743 platform would silently model the
wrong part (2 MiB flash, 512 KiB AXI SRAM, H743 SRAM layout), so the smoke
ships its own definition:

`emulator/renode/platform/potok_stm32h723.repl` is a self-contained copy of
Renode 1.17's `platforms/cpus/stm32h7.repl` — the base the built-in H743
platform derives from — with the H723 corrections below applied. It is
**derived from the H743 base, not aliased to it**.

### H723 memory corrections

Addresses and sizes come from ST's RM0468, cross-checked against Zephyr's
`dts/arm/st/h7/stm32h723.dtsi` and Embassy's `examples/stm32h723/memory.x`.
H723 has 564 KiB RAM in total and a single 1 MiB flash bank.

| region | H743 base in Renode 1.17 | H723 (this platform) |
|---|---|---|
| flash | 2 banks × 1 MiB (`0x08000000`, `0x08100000`) | **1 bank × 1 MiB** at `0x08000000`; bank 2 removed |
| ITCM | `0x00000000`, 64 KiB | unchanged |
| DTCM | `0x20000000`, 128 KiB | unchanged (result word `0x2001F800`) |
| AXI SRAM | `0x24000000`, 512 KiB | **`0x24000000`, 320 KiB** (128 + 192 shared) |
| SRAM1 | `0x30000000`, 128 KiB | **`0x30000000`, 16 KiB** |
| SRAM2 | `0x30020000`, 128 KiB | **`0x30004000`, 16 KiB** — base address corrected |
| SRAM3 | `0x30040000`, 32 KiB | **removed** — no SRAM3 on H723; that address is H723 SRAM2 |
| SRAM4 | `0x38000000`, 64 KiB | **`0x38000000`, 16 KiB** |
| backup SRAM | `0x38800000`, 4 KiB | unchanged |
| `FLASH_SIZE` system-flash field | `0x800` (2048 KiB) | **`0x400` (1024 KiB)** |

Peripheral **base addresses used by the smoke are the same as H743 and were
verified against the H723 DTS, not "corrected"**: flash controller
`0x52002000`, RCC `0x58024400`, GPIOA `0x58020000`, USART1 `0x40011000`, IWDG
`0x58004800`, EXTI `0x58000000`, RTC `0x58004000`. Two inherited models were
**dropped** rather than carried over: the two-bank flash controller (its model
requires a phantom second bank) and the two USB OTG models (their type name
differs between 1.17.0 and 1.17.1 and neither is H723-audited).

### Loud approximation label

Every other peripheral below the platform header is inherited verbatim from
Renode's STM32H7/H743 model. Its register behaviour, interrupt routing, clock
gating and even its presence on the H723 die have **not** been audited against
RM0468. The platform header lists these as `APPROXIMATED` in a banner;
`smoke.resc` prints `RENODE_H723_WARNING ... APPROXIMATED ... NOT hardware
validation` on every run, and `run_smoke.sh` fails if that line is absent.
An approximation cannot be silently dropped or forgotten.

## Layout

| path | purpose |
|---|---|
| `emulator/renode/firmware/` | standalone bare-metal smoke binary + its linker map and build script |
| `emulator/renode/platform/potok_stm32h723.repl` | explicit H723 platform (corrected memory map, verified/approximated labels) |
| `emulator/renode/platform/potok_stm32h743.repl` | legacy H743 alias; no longer used by the smoke |
| `emulator/renode/smoke.resc` | loads platform + ELF, verifies the H723 memory map, reads the result word, prints PASS/FAIL |
| `emulator/renode/run_smoke.sh` | platform/ELF gates, builds the ELF, runs Renode; local + CI entry point |
| `firmware/tests/renode_smoke.rs` | host contract: no-panic board, typed `Unavailable`, marker protocol agreement |

The smoke firmware is a **separate crate/workspace**, not a workspace member of
the flight firmware. It depends on `potok-m-firmware` as a path dependency and
calls the same `step()`, `FlightPath`, `PhaseFsm`, `LinkState`, `Interlocks` and
`WatchdogPolicy` code the flight binary calls. Keeping it separate means the
schematic `todo!()` in `firmware/src/main.rs` is never reached and no file
owned by the flight adapter has to change.

## Run

Prerequisites: rustup `stable` with the `thumbv7em-none-eabihf` target,
`readelf` (binutils), and Renode 1.17.x on `PATH`.

```sh
emulator/renode/run_smoke.sh                 # platform + ELF gates, build + run
emulator/renode/run_smoke.sh --skip-build    # re-run with the existing ELF
emulator/renode/run_smoke.sh --check-only    # platform + ELF gates only, no Renode
```

The script pins the rustup stable compiler ahead of any system `rustc` (a
system compiler without the thumb target fails with `E0463: can't find crate
for core`) and disables the machine-local sccache wrapper. Renode's transcript
goes to `/tmp/opencode/renode-smoke.log` by default; override with
`RENODE_SMOKE_LOG`.

### Hard gates before the emulator runs

1. **Platform present** — `potok_stm32h723.repl` must exist or the script exits
   with status 2. There is no fallback to the H743 alias.
2. **ELF layout matches H723** — `readelf -W -l` parses every `LOAD` segment.
   Each must lie wholly inside an H723 region (1 MiB flash, 64 KiB ITCM, 128
   KiB DTCM, 320 KiB AXI, 16 KiB each SRAM1/SRAM2/SRAM4, 4 KiB backup). A
   segment in the H743-only second bank `0x08100000`, past 128 KiB DTCM, or an
   entry point outside flash is a hard failure. The smoke result word
   `0x2001F800` must be inside DTCM. Success prints `H723_ELF_LAYOUT_OK`.
3. **Platform actually loaded** — `smoke.resc` probes twelve addresses through
   `SystemBus.FindMemory` and prints `RENODE_H723_PLATFORM_OK` only if every
   region has the exact H723 bounds. A typo or an accidental H743 load prints
   `RENODE_H723_PLATFORM_FAIL`, which fails the run.
4. **Approximation warning** — must appear in the transcript.

A passing run ends with:

```
RENODE_H723_PLATFORM_OK flash=1MiB itcm=64KiB dtcm=128KiB axi=320KiB sram1=16KiB sram2=16KiB@0x30004000 sram4=16KiB no-bank2=no-sram3
RENODE_H723_WARNING inherited STM32H743 peripheral models are APPROXIMATED for H723; NOT hardware validation
RENODE_SMOKE_RESULT stage=0xA5A5A5A5 panic_line=0 ticks=8 pets=1 unavail=24 sentinel=0x52454E4F
RENODE_SMOKE_PASS
[renode-smoke] PASS — STM32H723 platform, boot, clock/GPIO/UART init, 8 control ticks, watchdog feed, no fault (emulator only, not hardware validation)
```

## Result protocol

DTCM result frame, little-endian u32 words at `0x2001_F800` (the linker map
reserves the top 4 KiB of the 128 KiB DTCM and places the stack below it):

| offset | field | passing value |
|---|---|---|
| `+0x00` | stage | `0xA5A5_A5A5` |
| `+0x04` | panic source line | `0` |
| `+0x08` | control ticks completed | `8` |
| `+0x0C` | watchdog pets from the control path | `> 0` |
| `+0x10` | counted `Unavailable` calls | `> 0` |
| `+0x14` | sentinel | `0x5245_4E4F` ("RENO") |

The firmware clears the frame before stage 1 so a stale word from an earlier
run cannot read as a pass. Stage values are `1` boot, `2` clock, `3` GPIO,
`4` UART, `5` ticks, `6` watchdog fed, then PASS. `0xDEAD_DEAD` is a panic and
`0xF011_F011` is a failed in-firmware check. The panic handler halts on `wfi`
without a system reset, so a panic is observable in the word instead of being
hidden by a reboot.

## What the smoke exercises

1. **Boot** — cortex-m-rt reset handler, vector table from flash at
   `0x0800_0000`, stack at `0x2001_F000`.
2. **Clock init** — RCC enable for GPIOA and USART1 (`AHB4ENR`, `APB2ENR`).
3. **GPIO init** — PA5 configured as output and driven high.
4. **UART init** — USART1 TX enabled; the image emits
   `POTOK_RENODE_SMOKE_PASS` on `usart1` as corroboration (the result word is
   authoritative).
5. **Watchdog init** — IWDG unlocked with `0x5555`, prescaler `/256`, reload
   `0xFFF` (~18.7 s at the model's 56 kHz LSI), started with `0xCCCC`.
6. **Control loop** — 8 deterministic 500 Hz ticks. Each tick runs
   `FlightPath::tick` and the production `step()` with the real interlocks,
   shield and watchdog policy. Time is synthetic (`now_us += 2000`), so no
   device timer is read and the run is bit-reproducible across hosts.
7. **Watchdog feed** — `step()`'s feed path writes `0xAAAA` to `IWDG_KR`.
   The count is recorded; the link deadman latches after three reply timeouts,
   so the observed count is small but non-zero.
8. **No fault** — Renode must not log a watchdog reset or a hard/bus/usage
   fault, and the marker must be PASS.

## Emulator board adapter

The adapter is behind the `emulator-board` Cargo feature (default on). It is
**not** a hardware board:

- `DoubleBufferRx::take` returns `Ok(None)` — no DMA stream is claimed and no
  frame is fabricated.
- `FrameTx::send` returns `UartError::WouldBlock` — no TX pin is claimed.
- `motor_armed`, `poll_phase_event` and `apply_command` record
  `Unavailable::NoDriver` in a `CapabilityLedger` and return the safe value
  (`false`, `None`, discard).
- All seven `FlightBoard` inputs use the trait defaults (`None` /
  `Err(NoDriver)`), so `FlightPath` tallies them as counted unavailability.

There is no `todo!()`, no `unimplemented!()` and no panic-reset anywhere in
this binary. The host test enforces that by scanning the source.

## CI

`.github/workflows/renode.yml` runs these gates:

- **host contract** — `cargo test -p potok-m-firmware --test renode_smoke`.
  Always runs.
- **H723 platform + ELF layout** — requires `potok_stm32h723.repl` on disk,
  builds the ELF, then runs `run_smoke.sh --check-only`. No emulator needed.
- **smoke** — installs the pinned portable Renode 1.17.1 tarball from
  `builds.renode.io` and runs `emulator/renode/run_smoke.sh` on the H723
  platform. If the download is unreachable from GitHub Actions, that step
  fails with the curl error printed; the earlier gates still report whether
  the platform, ELF and host contract are intact.

Renode is not vendored, so the emulator gate depends on `builds.renode.io`
being reachable. That is the only external dependency of this lane. The H723
platform was verified locally against 1.17.0 and the pinned 1.17.1. The older
H743 smoke also ran on 1.16.1; H723 on 1.16.1 is **not** claimed. None of
these results is a hardware measurement.

## Known limitations / blockers

- The H723 platform is a **partial model**, not a chip datasheet. Every
  inherited peripheral outside the VERIFIED list in its header is
  `APPROXIMATED`: its registers, interrupts, clock gating and presence on the
  H723 die are unaudited. The smoke only exercises RCC, GPIOA, USART1 and IWDG.
- Renode's USART model does not implement the BRR baud-divider bits and logs a
  warning for the mantissa write. Older models (1.16.x portable) also warn on
  the RCC `APB2ENR` write. Neither affects execution: UART TX still works and
  the gate does not treat model warnings as faults.
- The IWDG model uses `0x5555` as the PR/RLR unlock key (RM0433/RM0468
  IWDG_KR). Writing `0xAAAA` first leaves PR/RLR write-protected and the
  watchdog resets at its default short timeout; the smoke would then fail.
  The firmware uses the correct order.
- The flash controller model was dropped because it requires two banks; H723
  has one. `LoadELF` writes flash directly, so the smoke is unaffected. A real
  flash-driver test would need a single-bank model.
- No RCC-to-systick link is wired. Renode 1.17.0 accepts only `systick: nvic`
  and 1.17.1 accepts only `nvic` + `dwt`; supplying either spelling to the
  other version is a hard platform-load error. The unused link produces a loud
  `Systick not passed in the RCC constructor` warning. The smoke never changes
  clocks and sets `nvic.systickFrequency` directly, so this is not a silent
  fallback.
- No interrupt, DMA or real-time clock is exercised. The ticker is synthetic
  on purpose: the smoke checks the control logic, not Renode's timer models.
- Passing this gate says nothing about pin maps, sensor drivers, servo wiring,
  EMI, or the flight `HwBoard` adapter. Real-board HIL remains the final
  verification for flight (`docs/HIL_CHECKLIST.md`).
