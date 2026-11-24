<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Toolchain: pinning the `thumb` check to rustup's rustc

The `thumb no_std` gate in both CI workflows runs one command:

```
RUSTC=$(rustup which rustc --toolchain stable) cargo check --target thumbv7em-none-eabihf
```

`dtolnay/rust-toolchain@stable` installs the `thumbv7em-none-eabihf` core into
the **rustup** stable toolchain. The check must therefore be compiled by that
toolchain's `rustc`.

## Why not `rustup run stable cargo check ...`

The obvious spelling is wrong on hosts where `PATH` resolves `cargo`/`rustc` to
a system install instead of the rustup proxies:

- `rustup run stable cargo` sets `RUSTUP_TOOLCHAIN=stable` but still executes
  whatever `cargo` `PATH` finds first.
- A distro `cargo`/`rustc` is not a rustup proxy and ignores
  `RUSTUP_TOOLCHAIN`. It then reaches for the distro `rustc`, which does not
  carry the rustup-installed `thumbv7em-none-eabihf` core.
- The result is `error[E0463]: can't find crate for core` for every `no_std`
  dependency, even though `rustup target list --installed --toolchain stable`
  lists the target.

This was reproduced on this box: `PATH` resolves `rustc` to Arch Linux's
`/usr/bin/rustc`, while `rustup which rustc --toolchain stable` resolves to
`/home/sehaxe/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc`.
The verbatim `rustup run stable cargo check --target thumbv7em-none-eabihf`
exits 101 with E0463; the `RUSTC=` form above exits 0. The two rustc versions
differ (Arch 1.100.0 vs rustup stable 1.99.0), which is why the target lookup
follows the toolchain binary and not `PATH`.

## Why this form on every platform

On GitHub's `ubuntu-latest`, `cargo`/`rustc` *are* rustup proxies, so the plain
`rustup run stable cargo` form happens to work. Using the explicit `RUSTC=`
form there is harmless: it resolves to the same rustc the runner would pick,
and the `thumb` target is installed by the action. Keeping one command in both
workflows removes a platform-dependent difference between local and CI runs.

## What is unchanged

This is a compiler-path fix, not a scope change. The gate still checks the
same target (`thumbv7em-none-eabihf`) with the same rustup stable toolchain;
nothing is skipped, relaxed or added. No firmware sources are touched. See
[SIL.md](SIL.md) for what the `thumb` gate proves and what it does not.

## What this gate does **not** prove (STM32H723 clock facts)

The `thumb` check is a compile gate. It does not configure RCC, PWR or FLASH,
program option bytes, or observe silicon. In particular it cannot establish any
of the following STM32H723 facts, so none of them is claimed as verified here:

| Item | Datasheet-backed value | Status on this board |
|---|---|---|
| Maximum core clock | 550 MHz, only at VOS0 with the `CPU_FREQ_BOOST` option byte; 520 MHz at VOS0 without it (ST DS13313 table 12, transcribed) | `NOT VERIFIED` |
| AXI clock / flash latency | 275 MHz AXI at 550 MHz CPU; 3 wait states at 275 MHz AXI in VOS0 (RM0468 table 16, transcribed) — the wait-state table is indexed by AXI, not CPU frequency | `NOT VERIFIED` |
| Core voltage | VCORE 1.35 V at VOS0; VDD ≥ 1.7 V with the internal LDO (DS13313 table 13 / §3.7.1, transcribed) | `NOT VERIFIED`; the rail itself is `NOT ASSIGNED` in [POWER_BUDGET.md](POWER_BUDGET.md) |
| Boost side effect | `CPU_FREQ_BOOST` disables ITCM/DTCM ECC (RM0468, transcribed) | option-byte state `NOT ASSIGNED` |

The cycle budget in `firmware/src/perf.rs` uses 550 MHz as a *configured input*
and derives the allowance from it via `cycle_budget_for`; that is arithmetic,
not evidence that the board runs at 550 MHz. `docs/POWER_BUDGET.md` §6 step 12
is the procedure that would turn these `NOT VERIFIED` cells into readings.

One discrepancy is visible from this repo alone and is **not fixed here**:
`firmware/Cargo.toml` still selects the `stm32h743zi` Embassy chip feature
(the `thumb` gate and the `.cargo/config.toml` runner do the same). That is
an H743 part selection, not an H723 one, and is outside this task's file scope.
It must be reconciled by the owner before any target build or flash; this
document records it rather than silently treating the target as correct.

No clock, voltage, wait-state, current or timing value in this document was
measured on hardware, and no target run is claimed.
