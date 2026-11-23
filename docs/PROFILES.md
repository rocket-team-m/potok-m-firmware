<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Typed MCU runtime profiles

**Status:** implemented in `firmware/src/profile.rs`, gated by
`firmware/tests/profile_tests.rs`. **Not wired into `lib.rs` or `main.rs`** —
those files are owned by the parent task. A table that is not selected at boot
is a declaration, not an active configuration; see §6.

## 1. Purpose

Five safety-relevant knobs are currently loose constants or module locals:
PID gains (`control_pid`), shield gates (`interlocks::Limits`), the IWDG window
(`watchdog::WATCHDOG_TIMEOUT_US`), telemetry logging (an inline `defmt` call in
`main.rs`) and the AI correction bound (`MAX_CORRECTION_RAW`). A
`RuntimeProfile` bundles all five into one immutable, typed value so a flight
configuration can be reviewed, hashed and activated as a unit.

The three profiles are ordered from most to least conservative. No profile
raises the `M ≤ 0.6` flight ceiling or the `MAX_CORRECTION_RAW` code veto —
those are requirements, not tuning dimensions.

| Field | SAFE | BALANCED (default) | PERFORMANCE |
|---|---|---|---|
| `control.kp` (per step) | 0.5 | 1.0 | 1.0 |
| `control.ki_dt` | 0.0 | 0.0 | 0.01 |
| `control.kd_dt` | 0.5 | 0.0 | 0.25 |
| `control.out_limit` (raw) | 40 | 100 | 100 |
| `control.i_limit` (raw) | 5 | 10 | 10 |
| `control.deriv_alpha` | 0.5 | 1.0 | 0.75 |
| `shield.max_command` (raw) | 40 | 100 | 100 |
| `shield.max_delta` (raw) | 4 | 10 | 16 |
| `shield.min_interval_us` | 2 000 | 1 000 | 500 |
| `shield.mach_limit` | **0.5** | **0.6** | **0.6** |
| `watchdog_deadline_us` | 50 000 | 100 000 | 100 000 |
| `log_every_ticks` | 1 | 1 | 10 |
| `ai_authority_cap` (raw) | 40 | 100 | 100 |

BALANCED carries `Limits::DEFAULT`, `WATCHDOG_TIMEOUT_US` and
`MAX_CORRECTION_RAW` through unchanged and logs every tick, as `main.rs` does
today. **Adopting this table changes no existing default.** The PID gains are
typed configuration, not a measured flight recipe — no tuning result is claimed
for any profile.

## 2. Changing the active profile requires two gates

```rust
let id = ProfileId::Safe;
let set = ProfileSet::boot(id, profile_hash(id), Activate::explicit())?;
// ... ground checks ...
set.lock(); // one-way; call before the 500 Hz loop starts
```

1. **Hash.** `activate` takes the exact `RuntimeProfile::hash` of the target.
   A stale or hand-edited hash returns
   `ActivationError::HashMismatch { expected, got }` and leaves the active
   profile untouched. `manifest_hash()` covers all three entries, so a change
   to any profile is visible as one table-level value. `profile_hash(id)` is
   the supported way for a caller to obtain the value it must pass.
2. **Explicit intent.** The second argument is an `Activate`, a zero-sized
   token whose only constructor is `Activate::explicit()`. No struct literal
   outside the module can produce one, so a profile cannot be selected by a
   stray assignment.

The hash is FNV-1a over the canonical little-endian field layout; floats are
hashed by IEEE-754 bits. It is deterministic on host and MCU, and needs no
dependency, allocator or `libm`.

## 3. No in-flight switching

`ProfileSet::lock()` is one-way and has no `unlock`. After it, every
`activate` — including one with the correct hash and token — returns
`ActivationError::Locked`; the lock check runs before the hash check. There is
no setter and no path from a decoded UART reply to `ProfileSet`. The firmware
must lock before entering its control loop, so the active profile is fixed for
the whole flight.

## 4. Invalid config is LOUD

`RuntimeProfile::validate` reuses the builders that own each contract:
`Pid::<f32>::new` for gains and `Interlocks::new` for shield thresholds. Their
assertion messages therefore stay identical to the code that will consume the
values. The remaining three fields get their own asserts:

- `watchdog_deadline_us` in `1..=WATCHDOG_TIMEOUT_US` — a software deadline
  past the hardware IWDG window is meaningless, the hardware resets first;
- `log_every_ticks > 0`;
- `ai_authority_cap` and `control.out_limit` within `1..=MAX_CORRECTION_RAW` —
  a profile may lower authority, never raise it.

`ProfileSet::activate` validates before it mutates state. There is no silent
fallback and no "nearest valid profile" adjustment.

## 5. Verification

`firmware/tests/profile_tests.rs` (18 tests, all green) pins the unchanged
BALANCED envelopes, the monotone SAFE → BALANCED → PERFORMANCE ladder, the
fixed 0.6 Mach ceiling and 100-raw code veto, hash/token refusal paths, the
one-way lock, hash stability and sensitivity to a single field change, and one
`#[should_panic]` case per invalid-field class. The profile table cannot drift
from its consumers without failing a test: every config is fed directly to
`Pid::new` and `Interlocks::new` in the suite.

## 6. Integration debt (do not claim more than is true)

`profile.rs` is not declared in `firmware/src/lib.rs`, and the shipping loop in
`firmware/src/main.rs` still reads the loose constants. The host tests include
the module by path with a re-export shim, the same pattern `control_pid.rs` and
`perf.rs` use. Until the parent task declares the module, selects a profile at
boot with hash + `Activate::explicit()`, calls `lock()`, and derives the five
knobs from `active().config()` instead of the constants, **no profile affects
the firmware**. A typed config is not an active config.

The `log_every_ticks` knob is the hook for the unmodelled 500 Hz `defmt` cost
named in `.bulba/night/MCUF01.md` §4.2; PERFORMANCE samples it every tenth tick,
but no on-target cycle measurement of that call exists yet, so no saving is
claimed here.
