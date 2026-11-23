<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# Runtime shield for the Mach ceiling

**Status:** implemented in `firmware/src/interlocks.rs`, gated by
`firmware/tests/interlocks_tests.rs`. **Integration into the flight loop is
owed** — see §7. A shield that is not in the actuation path is not a shield.

## 1. Finding

The `M ≤ 0.6` requirement is currently a **post-flight reward rejection**:
`sim/core/src/reward.rs:9-14,46,260` returns `RewardError::GateRejected` and no
reward when the recorded peak Mach exceeds the limit. A teacher trained against
that gate can still fly `M > 0.6`; it only receives a bad score afterwards. This
is an evaluation knife, not runtime protection (`.bulba/night/WORLD06.md:50`).
CPO / Lagrangian methods constrain an expectation over trajectories and cannot
supply a per-flight guarantee (WORLD06 §1, §2.1).

The fix outside the learner is a deterministic **post-posed shield**: it observes
the same measurements the controller sees and replaces a disallowed AI command
with `Command::ZERO` plus a reason code, on every tick.

## 2. What the shield does

`Interlocks::evaluate_shield(input, pitot_dp, mach_est)` is the shield entry
point. It is pure and deterministic: finite-state phase/flag checks, float
finiteness and ordering comparisons, no RNG, no host round-trip, no `libm`, no
allocation. It composes the existing interlock gate with the new Mach gate and
is safe to call from `no_std`.

**Inputs**

| Input | Origin | Meaning |
|---|---|---|
| `input: Input` | STM32-owned phase code, sensor flags, AI health flag, candidate `Command`, monotonic `u32` clock | Same five fields the existing `Interlocks::evaluate` takes. The shield does not trust phase or health carried in an AI reply. |
| `pitot_dp: i16` | measured pitot differential (dynamic) pressure, Pa, wire field `pitot_dp` | Airspeed evidence. A non-positive reading cannot support a Mach gate. |
| `mach_est: f32` | Mach estimate derived by the caller from `pitot_dp` and static pressure | The value compared to the ceiling. Non-finite values are a fault. |

`Limits::mach_limit` is the configured ceiling; `Limits::DEFAULT.mach_limit` is
**0.6**. A non-positive, NaN or infinite limit is rejected loudly by
`Interlocks::new` — no silent fallback.

## 3. Refusals — zero command, reason code, no tradeoff

Every refusal returns `Decision { command: FAILSAFE, permitted: false,
refusal: Some(code) }`. There is no clamp-to-limit, no "reduce the correction
instead of zeroing it", and no soft band below the ceiling.

| Order | Condition | Reason code | Latch? |
|---|---|---|---|
| 1 | Active latch from an earlier fault | original reason | until `reset()` |
| 2 | Phase code not in the generated `Phase` enum | `UnknownPhase(u8)` | yes |
| 3 | `Phase::BURN` (boost) | `Boost` | no |
| 4 | `MOTOR_ARMED` sensor bit | `MotorArmed` | no |
| 5 | AI declared unhealthy | `AiUnhealthy` | yes |
| 6 | `FAULT` flag | `Fault` | yes |
| 7 | `mach_est` not finite (NaN, ±inf) | `NonFiniteMach` | yes |
| 8 | `pitot_dp <= 0` | `PitotInvalid` | no |
| 9 | `mach_est > mach_limit` | `MachLimit` | no |
| 10 | Command outside the plausible box | `Implausible` | yes |
| — | all gates passed | `None` | rate-limited, not zeroed |

Checks 1–6 are shared verbatim with `Interlocks::evaluate` (factored into
`preflight`), so the shield cannot silently drop a phase or health gate. The
Mach gate (7–9) runs **before** plausibility and rate limiting: a refused tick
never feeds the slew state, so recovery starts from the last *accepted* command
and no `accepted`/`slewed` count is fabricated.

Transient reasons (3, 4, 8, 9) stop the command only while they hold and recover
without a reset. `NonFiniteMach` latches because a non-finite measurement is a
fault, not a condition; only `reset()` clears it. `mach_blocked` counts transient
Mach refusals separately from the latching `trips` counter. Counters are never
cleared by `reset()`.

The `>` comparison is deliberate: a `mach_est` exactly equal to the limit is
accepted. This is pinned by a test.

## 4. Why this is a *command* gate, not a Mach guarantee

The shield guarantees only that **AI output is zero above the ceiling**. It does
not guarantee `M ≤ 0.6`. The airframe's fin corrections cannot, in general,
remove kinetic energy injected by the motor. WORLD06 §2.4 puts the sharp
question: do the ±20 m/s² fin corrections have authority over peak Mach during
boost? `vehicle/DESIGN_INPUT.md:40-45` states the `M = 0.5–0.6` target in the
same line as OpenRocket reaching `M = 0.82`; `POLISH-SIM.md:41` and
`FINAL-CRITIC.md:57` leave the contradiction open.

**Therefore, on boost, no controller — learned, shielded, or CBF-QP — can
enforce `M ≤ 0.6` if thrust drives Mach past the limit.** A CBF-QP with input
bounds returns "no feasible certificate" in exactly this case (WORLD06 §2.4,
§5). The shield zeroes the AI command while that condition holds, but the
physical remedy belongs to the **vehicle design**: change the motor/airframe,
or add a thrust-cutoff interlock with real authority over the state. Claiming
the software shield keeps `M ≤ 0.6` would be a category error.

## 5. What is deliberately not in this change

- No learned one-step model, CBF-QP, or confidence threshold. WORLD06 §2.2/§2.4
  and §5: those give conditional guarantees, depend on model accuracy and QP
  feasibility, and are not needed for the finite-state gates this task covers.
  The production confidence threshold is a separate open defect
  (`docs/src/content/docs/ai/safety.md:59-63` in the parent repo).
- No reward change. `sim/core/src/reward.rs` stays a verifier; the shield is the
  runtime mechanism. Training with the shield in the loop is a follow-up
  (WORLD06 §3 item 4).
- No change to the flight phases or the link/deadman logic. The shield reuses
  the existing interlock gate rather than growing a second policy.

## 6. Verification

`firmware/tests/interlocks_tests.rs` covers the ceiling default, `M` exactly at
the limit, `M` above the limit (zero + reason + no latch + recovery slew),
non-finite estimates (latch until reset), non-positive pitot, ordering against
the phase/health/fault gates, state preservation on refusal, a configured
alternative limit, and loud rejection of bad limits. All firmware tests pass
(`cargo test`, 0 failures). The tests are the instrument for the structural
claim: they are property-style over the finite reason-code table and exact
boundary values, not a statistical flight result.

## 7. Integration debt (do not claim more than is true)

`evaluate_shield` is implemented and tested, but the shipping control loop
still calls `Interlocks::evaluate` (`firmware/src/control.rs:329`) and the SIL
guard likewise (`firmware/src/sil.rs:294`). Neither file is in this task's
scope. Until both are switched to `evaluate_shield` — with `pitot_dp` and a
caller-derived `mach_est` supplied every tick — the Mach gate is **not** in the
actuation path and `M ≤ 0.6` remains a post-flight score. This is the L0
placement gap named in WORLD06 §4 and §1 (the same gap the watchguard has
between the SIL harness and `main.rs`). Closing it is the next task; it must be
verified by a call-graph check that every `board.apply_command` routes through
the shield, not by a test of an uncalled module.
