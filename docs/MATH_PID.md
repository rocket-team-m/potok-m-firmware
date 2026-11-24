<!-- SPDX-License-Identifier: GPL-3.0-or-later -->

# `control_pid.rs` — source-traced PID

**Scope.** One axis, one 500 Hz tick, no I/O. This document traces every term
in `firmware/src/control_pid.rs` to the discrete PID equations printed by
Åström & Hägglund, checks the signs and the per-step scaling, and lists the
closed-form gates in `firmware/tests/pid_math_tests.rs`. **No formula error was
found**: the four original expectations that failed while writing the gates
were wrong test arithmetic (bumpless preload cancelling a P-only setpoint step,
an early integral clamp, and an omitted `K·y` term), not defects in the code.
No gain was changed to make a test pass.

## 1. Sources

Identifiers are resolved, not invented. Formulae were read from the book
chapter PDFs / full text; the publisher records give the edition.

| tag | source | identifier |
|---|---|---|
| **AH95** | Åström & Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed., ISA, 1995, Ch. 3 "PID Control" (§3.3 PID Control, §3.4 Modifications of the PID Algorithm, §3.5 Integrator Windup, §3.6 Digital Implementation, §3.7 Operational Aspects) | ISBN 978-1-556-17516-9 |
| **AH06** | Åström & Hägglund, *Advanced PID Control*, ISA, 2006, Ch. 3 "PID Control" pp. 64–94 (§3.2 The PID Controller, §3.3 Filtering the Derivative, §3.4 Set-Point Weighting, §3.5 Integrator Windup) and Ch. 13 "Implementation" pp. 407–432 (§13.3 Computer Implementations, §13.5 Operational Aspects) | ISBN 978-1-556-17942-6; book DOI 10.1002/9781394442102 (online edition) |
| **IFAC24** | Sundström, Hägglund, Bauer, Eker, Soltesz, *Reference Implementation of the PID Controller*, 4th IFAC Conf. on Advances in PID Control, Almería, June 2024, §3.6 "Bumpless transfer", §3.7 "Integrator anti-windup" | open-access proceedings `PID-2024/0051.pdf` |
| **AH02** | K. J. Åström, *A Practical PID Controller* lecture slides, Oct 2002 — print the full discrete algorithm and windup update | author slides, University of Padova mirror |

Primary chapter files read: ISA/NTNU `Advanced PID Control` chapter 3 and
chapter 13 PDFs; AH95 full text transcription. The 2006 book's chapter map is
also confirmed by the publisher TOC (`Advanced-PID-Control_AstromHagglund_TOC.pdf`).

## 2. Equation map

`y` = measurement, `r` = setpoint, `e = r − y`, `h` = tick period,
`K` = proportional gain, `Ti` = integral time, `Td` = derivative time,
`N` = derivative gain limit. AH95 calls the parallel gains `k, ki, kd`.

| Code | Source equation | Checked |
|---|---|---|
| `error = r − y` | AH95 Eq. (3.9); AH06 Eq. (3.1), text "e = ysp − y" | sign: positive error → positive output |
| `p = kp·error` with `b = 1` | AH95 Eqs. (3.19)–(3.20); AH06 Eqs. (3.17)–(3.18); sampled AH06 Eq. (13.5) | no discretisation needed; `kp = K` |
| `d = −kd_dt·filtered_delta`, `c = 0` | AH95 Eq. (3.22) `Td/N·Ḋ + D = −K·Td·ẏ` and §3.7 program; AH06 Eq. (3.14) `D = −sK Td/(1+sTd/N)·Y`; sampled AH06 Eq. (13.13) | **minus sign on measurement**; `kd_dt = K·Td/h` |
| `filtered_delta += alpha·(Δy − filtered_delta)` | AH06 Eq. (13.13), `alpha = Td/(Td+N·h)` | equivalent recursion (linearity, §3 below) |
| `integral += ki_dt·error` | AH95 §3.7 program `bi = K·h/Ti`; AH06 Eq. (13.7) forward Euler | `ki_dt = (K/Ti)·h` |
| integral stored as output contribution | AH06 §13.5 text and Eq. (13.25) | state is `∫(K/Ti)e dt`, so `K`/`Ti` changes do not bump output |
| `freeze` when saturated and pushing further out | AH95 §3.5 "Conditional Integration" (verbatim rule); AH06 §3.5; IFAC24 §3.7 | unwinds on the next tick when the error reverses |
| setpoint preload `I ← u_prev − p − d` | AH06 Eq. (13.25) (parameter-change form); IFAC24 §3.6 "changing the integrator state to a value that results in a bumpless transfer" | holds `P+I+D` continuous |
| `output = unsat.clamped(−out_limit, out_limit)` | AH95 §3.7 program `u = sat(v, ulow, uhigh)`; AH06 Eq. (13.23) | symmetric, counted by `saturated()` |
| `i_limit` clamp | **firmware choice**, no book equation | separate clamps on I contribution and on output |

## 3. Discrete derivation and sign/scaling verification

### 3.1 Terms

AH06 Eq. (13.5)–(13.13) instantiated at `b = 1`, `c = 0`:

```
P_k  = K (r_k − y_k)                                        (13.5)
e_k  = r_k − y_k
I_{k+1} = I_k + (K h / Ti) e_k                              (13.7, forward Euler)
D_k  = [Td/(Td+N h)] D_{k−1} − [K Td N/(Td+N h)] (y_k − y_{k−1})   (13.13)
v_k  = P_k + I_k + D_k ;  u_k = sat(v_k)                    (AH95 §3.7 program)
```

`PidConfig::from_si` precomputes `ki_dt = Ki·h = (K/Ti)·h` and
`kd_dt = Kd/h = K·Td/h`. The code's `deriv_alpha` is `alpha = Td/(Td+N·h)`, so

```
kd_dt·(1 − alpha) = (K·Td/h)·(N·h/(Td+N·h)) = K·Td·N/(Td+N·h),
```

which is exactly the AH06 Eq. (13.13) measurement coefficient. The firmware
filters the *increment* `f_k = alpha·f_{k−1} + (1−alpha)·Δy_k` and then sets
`D_k = −kd_dt·f_k`; because the map `f ↦ −kd_dt·f` is linear, this is the same
recursion as filtering `D` directly. Gate: `si_gains_match_the_book_discrete_coefficients`.

### 3.2 Signs

* `e = r − y`: measurement below setpoint gives positive error and positive
  output — the AH06 Eq. (3.1) convention.
* `d = −kd_dt·LPF(Δy)`: a **rising measurement lowers the output**. The minus
  is present in AH95 Eq. (3.22), AH06 Eq. (3.14) and AH06 Eq. (13.13). The
  setpoint does not enter `d` (`c = 0`), so a setpoint step cannot cause a
  derivative kick; AH06 §3.4: "c is normally chosen equal to zero to avoid
  large transients ... due to sudden changes in the set point".
* Saturation is symmetric and only flags when `v` is strictly outside the
  limits; `v == limit` is the unsaturated edge.
* Conditional integration freezes only if the candidate update increases the
  saturation (AH95 §3.5: "switched off when the controller is saturated and
  the integrator update is such that it causes the control signal to become
  more saturated"). A reversal therefore unwinds on the next tick.

### 3.3 Bumpless transfer

AH06 Eq. (13.25) changes the state by `I_new = I_old + P_old − P_new` so that
`P+I` is invariant when `K` or `b` changes. The firmware applies the same
invariant to a *setpoint* change, using the already saturated previous output:

```
I_new = u_prev − P_new − D_new    (clamped to ±i_limit),  skip this tick's integration
⇒ v_new = P_new + I_new + D_new = u_prev
```

This is the positional-form trick named in IFAC24 §3.6, not a book equation
for setpoint changes. AH06 §3.4 handles setpoint transients instead with the
weight `b`; this firmware uses `b = 1` plus the preload. Gate:
`setpoint_change_is_bumpless_in_closed_form` and
`bumpless_preload_uses_the_saturated_previous_output`.

## 4. Firmware choices and known limits

* **`b = 1`, `c = 0`.** No setpoint weight on P; derivative on measurement only.
* **Conditional integration, not back-calculation.** AH06 §3.5 presents both;
  the AH95 §3.7 program uses back-calculation `I += (h/Tt)(u − v)`. Choosing
  clamping removes the extra tuning parameter `Tt` (IFAC24 §3.7 makes the same
  choice for its reference implementation).
* **Backward-Euler derivative filter** (AH06 Eq. 13.13) rather than the Tustin
  coefficients `ad = (2Td−Nh)/(2Td+Nh)` in the AH95 §3.7 program. Both are
  named in the sources; only the former is implemented.
* **`i_limit`** is a firmware clamp with no book counterpart; `out_limit` is
  the actuator bound of AH95/AH06.
* **Q16.16 round-half-away-from-zero** (`Fx`) is a firmware rounding policy,
  not sourced; the deterministic `f32` backend is the reference operation
  sequence on the host.
* **First tick primes measurement history to zero.** If the first measurement
  is non-zero, the first derivative sample is non-zero (an initialization
  transient, not a setpoint kick). AH02 likewise leaves `yold` initialization
  to the integrator. Not changed here; prime the controller with the trim
  measurement before arming if that transient matters.
* **P-only setpoint steps are held continuous by the preload.** With `ki_dt =
  kd_dt = 0`, a setpoint step leaves the output unchanged (the preload absorbs
  the whole P change). This follows from applying bumpless transfer to
  setpoints; PI configs resume integration on the following tick. Pinned by
  `pure_p_controller_holds_output_continuous_across_a_setpoint_step`.
* **No hardware validation.** These are compile-time, host-arithmetic gates.
  No board, actuator or sensor run is claimed.

## 5. Closed-form gates

`firmware/tests/pid_math_tests.rs` (18 tests, all green) — expected values are
computed from the equations above, not captured from a run:

| property | test | equation |
|---|---|---|
| P step response, instant and linear | `proportional_step_response_is_instant_linear_and_unsigned_free` | AH06 (13.5) |
| P-only setpoint continuity | `pure_p_controller_holds_output_continuous_across_a_setpoint_step` | AH06 (13.25) |
| I step response = forward Euler + clamp | `integral_step_response_is_forward_euler_then_clamped` | AH06 (13.7) |
| D step response on measurement, negative | `derivative_step_response_is_on_measurement_with_negative_sign` | AH06 (13.13) |
| D filter geometric recursion | `derivative_filter_is_the_geometric_recursion_of_ah13_13` | AH06 (13.13) |
| no derivative kick on setpoint step | `setpoint_step_causes_no_derivative_kick` | AH06 (3.19), (13.13) |
| anti-windup freeze at upper/lower limit | `anti_windup_freezes_at_the_edge_and_unwinds_in_one_tick`, `...lower_limit_too` | AH95 §3.5 / AH06 §3.5 |
| integral limit symmetric; output not saturated | `integral_limit_is_symmetric_and_binds_before_output_saturation` | firmware choice |
| symmetric counted saturation | `output_saturation_is_symmetric_and_counted` | AH95 §3.7 program |
| bumpless setpoint continuity (exact) | `setpoint_change_is_bumpless_in_closed_form` | AH06 (13.25), IFAC24 §3.6 |
| bumpless preload from saturated output | `bumpless_preload_uses_the_saturated_previous_output` | same |
| per-step coefficient scaling | `si_gains_match_the_book_discrete_coefficients` | AH06 (13.7), (13.13) |
| Fx ≡ f32 through saturation + freeze | `fixed_point_and_f32_agree_through_the_full_nonlinear_path` | all of the above |
| reset clears D history | `reset_clears_measurement_history_for_the_derivative` | AH06 (13.13) state |
| loud config refusals | `zero_derivative_alpha_is_rejected_loudly`, `negative_integral_limit_is_rejected_loudly` | firmware boundary |
| command bridge | `command_bridge_is_unchanged` | firmware seam |

The pre-existing `firmware/tests/control_pid_tests.rs` (11 tests) is unchanged
and still green; both files include the module by path so neither depends on
`lib.rs` wiring.
