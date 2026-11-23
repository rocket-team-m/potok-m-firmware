# Renode smoke (STM32H743)

A deterministic, emulator-only smoke of the Potok-M control core. It builds a
`thumbv7em-none-eabihf` ELF, loads it into Renode's built-in STM32H743 model,
and checks a result word in DTCM.

**This is not hardware validation.** Nothing here touches a real chip, pin,
clock, sensor, servo or radio. The flight binary still contains the
schematic-bound `HwBoard::build_board()` placeholder; this smoke deliberately
does not modify it. Passing this gate means "the control core boots and runs a
bounded tick sequence on an emulated Cortex-M7 without a fault", nothing more.

## Layout

| path | purpose |
|---|---|
| `emulator/renode/firmware/` | standalone bare-metal smoke binary + its linker map and build script |
| `emulator/renode/platform/potok_stm32h743.repl` | names Renode's built-in `platforms/cpus/stm32h743.repl` |
| `emulator/renode/smoke.resc` | loads platform + ELF, reads the result word, prints PASS/FAIL |
| `emulator/renode/run_smoke.sh` | builds the ELF, runs Renode, gates on the result (local + CI entry point) |
| `firmware/tests/renode_smoke.rs` | host contract: no-panic board, typed `Unavailable`, marker protocol agreement |

The smoke firmware is a **separate crate/workspace**, not a workspace member of
the flight firmware. It depends on `potok-m-firmware` as a path dependency and
calls the same `step()`, `FlightPath`, `PhaseFsm`, `LinkState`, `Interlocks` and
`WatchdogPolicy` code the flight binary calls. Keeping it separate means the
schematic `todo!()` in `firmware/src/main.rs` is never reached and no file
owned by the flight adapter has to change.

## Run

Prerequisites: rustup `stable` with the `thumbv7em-none-eabihf` target, and
Renode 1.17.0 on `PATH`.

```sh
emulator/renode/run_smoke.sh                 # build + run + gate
emulator/renode/run_smoke.sh --skip-build    # re-run with the existing ELF
```

The script pins the rustup stable compiler ahead of any system `rustc` (a
system compiler without the thumb target fails with `E0463: can't find crate
for core`) and disables the machine-local sccache wrapper. Renode's transcript
goes to `/tmp/opencode/renode-smoke.log` by default; override with
`RENODE_SMOKE_LOG`.

A passing run ends with:

```
RENODE_SMOKE_RESULT stage=0xA5A5A5A5 panic_line=0 ticks=8 pets=1 unavail=24 sentinel=0x52454E4F
RENODE_SMOKE_PASS
[renode-smoke] PASS — boot, clock/GPIO/UART init, 8 control ticks, watchdog feed, no fault
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

`.github/workflows/renode.yml` runs two gates:

- **build**: install the thumb target, build the smoke ELF, run
  `cargo test -p potok-m-firmware --test renode_smoke`. This gate always runs.
- **smoke**: install the pinned portable Renode 1.17.1 tarball from
  `builds.renode.io` and run `emulator/renode/run_smoke.sh`. If the download is
  unreachable from GitHub Actions, that step fails with the curl error
  printed; the build gate still reports whether the ELF and the host contract
  are intact.

Renode is not vendored, so the emulator gate depends on `builds.renode.io`
being reachable. That is the only external dependency of this lane. The
portable tarball was verified locally against both 1.16.1 (`renode-latest`)
and the pinned 1.17.1; both produce the same PASS word.

## Known limitations / blockers

- The built-in `stm32h743.repl` exposes 1 MiB of flash at `0x0800_0000` while
  the linker map declares 2 MiB. The smoke image is ~3.3 KiB of `.text`, so it
  fits; a larger image would need `flashBank1.size` overridden in the platform
  file.
- Renode's USART model does not implement the BRR baud-divider bits and logs a
  warning for the mantissa write. Older models (1.16.x portable) also warn on
  the RCC `APB2ENR` write. Neither affects execution: UART TX still works and
  the gate does not treat model warnings as faults.
- The IWDG model uses `0x5555` as the PR/RLR unlock key (RM0433 IWDG_KR).
  Writing `0xAAAA` first leaves PR/RLR write-protected and the watchdog resets
  at its default short timeout; the smoke would then fail. The firmware uses
  the correct order.
- No interrupt, DMA or real-time clock is exercised. The ticker is synthetic
  on purpose: the smoke checks the control logic, not Renode's timer models.
- Passing this gate says nothing about pin maps, sensor drivers, servo wiring,
  EMI, or the flight `HwBoard` adapter. Real-board HIL remains the final
  verification for flight (`docs/HIL_CHECKLIST.md`).
