# Attitude filter — source mapping and verification

**Source of truth.** R. Mahony, T. Hamel, J.-M. Pflimlin, *Nonlinear
Complementary Filters on the Special Orthogonal Group*, IEEE Transactions on
Automatic Control **53**(5), 2008, pp. 1203–1218, **DOI
10.1109/TAC.2008.923738**. The author manuscript (HAL **hal-00488376v1**,
`2007_Mahony.etal_TAC-06-396_v3.pdf`) was opened for this work; equation numbers
below are the published/manuscript numbers as printed there. No hardware or
flight validation is claimed anywhere in this document — the evidence is the
host fixed-point tests listed at the end.

**Implementation.** `firmware/src/attitude.rs` (integer-only, Q30 unit
quaternions/vectors, Q16 rad/s, Q20 s, Q8 gains, Q32 bias accumulator).

## 1. Convention mapping

| paper | firmware | note |
|---|---|---|
| `R = A B R`, body → inertial (Eq. 4) | `R(q)`, body → world | same direction |
| `R̃ = R̂ᵀ R` (Eq. 1) | not materialised | error is formed directly from directions |
| `v_i = Rᵀ v0_i`, `v̂_i = R̂ᵀ v0_i` (Eq. 29) | `accel` / `mag` inputs; `up` / `north` | `v0_a = +e3` (up), `v0_m = +e1` (north) |
| `v_a ≈ −Rᵀ e3` (Eq. 29, inertial `e3` down) | `a/|a| ≈ Rᵀ e3` (inertial `e3` up) | same specific-force vector, opposite world-axis label |
| `Ω_y = Ω + b` (Eq. 11b) | `gyro` input, `b̂ = bias()` | `b` is the bias **added** to the true rate |
| `q̇ = ½ q ⊗ p(Ω)` (Eq. 45) | `q ← q ⊗ [1, Ω_c·dt/2]` | body rate ⇒ right multiplication |

`up` and `north` are rows 3 and 1 of `R(q)`
(`firmware/src/attitude.rs:279`, `:284`): these are `R̂ᵀe3` and `R̂ᵀe1`.

## 2. Equations implemented

| paper equation | expression | code |
|---|---|---|
| Eq. 29–31 | unit-normalised body measurements, estimated references, weights `k_i > 0` | `Vec3::normalized` `:73`; `e_acc`/`e_mag` `:290`, `:297` |
| Eq. 32c | `ω_mes = Σ_i k_i (v_i × v̂_i)` — **measured × estimated** | `cross(a, up)` / `cross(m, north)` `:290`–`:302` |
| Eq. 32a | `Ω_c = Ω_y − b̂ + kP·ω_mes` | bias subtraction `:329`–`:331`, proportional terms `:334`–`:342` |
| Eq. 32b | `ḃ̂ = −kI·ω_mes` | integral updates `:316`–`:323` |
| Eq. 33 | `V̇ = −kP‖Pa(R̃M)‖² ≤ 0` | the two minus signs above are required for this sign |
| Eq. 48b (Eq. 45) | `q̇ = ½ q ⊗ p(Ω_c)` | small-angle `dq` `:343`, right product `mul(q, dq)` `:349` |
| — | discrete projection onto unit quaternions | `.normalized()` `:350` (implementation detail, not a paper equation) |
| Eq. 30/31 | per-reference confidence weights | `Gains { kp, ki, kp_mag, ki_mag }` `:164`–`:189`: `kp/ki = kP·w_a`, `kp_mag/ki_mag = kP·w_m`, `kI·w_a`, `kI·w_m`; default `w_m/w_a = 0.5` |

**Paper-internal factor of two.** Eq. 32c defines `ω_mes` with `k_i`, whereas
Eq. 34 and the quaternion Appendix Eq. 48a print `k_i/2` (since
`Pa(R̃M) = ½ Σ k_i(v̂_i v_iᵀ − v_i v̂_iᵀ)`). The filter definition, Eq. 32, is
implemented verbatim; the factor is absorbed into the positive per-reference
weights. No gain was changed to make a test pass.

## 3. Defects found and fixed

The previous source had the right *attitude* behaviour but the wrong *bias*
convention. The two bias signs were flipped together:

| check | before | paper | action |
|---|---|---|---|
| error sign (Eq. 32c) | `a × up`, `m × north` | `v_i × v̂_i` | **correct already**, kept |
| bias integral sign (Eq. 32b) | `b̂ += kI·e·dt` | `ḃ̂ = −kI·ω_mes` | flipped to `−=` (`:316`–`:323`) |
| bias application (Eq. 32a) | `Ω_c = Ω_y + b̂ + kP·e` | `Ω_c = Ω_y − b̂ + kP·ω_mes` | flipped to `−` (`:329`–`:331`) |
| quaternion order (Eq. 48b) | `q ⊗ dq`, body rate | `q ⊗ p(Ω_c)` | **correct already**, kept |
| normalisation | every step | not in paper; discrete projection | **correct already**, kept |
| gains | `kp`/`ki`, `kp_mag`/`ki_mag` | `kP·w_i`, `kI·w_i` | **structure correct**, values unchanged |

A double sign inversion is self-consistent: the attitude loop still levels
and the bias integral still settles, but at a finite value `b̂ → −b` instead of
`b̂ → b`. That is why a legacy test could pass while asserting the inverted
bias. With the paper signs, `bias()` converges to the bias that was *added* to
the gyro (Eq. 11b) and is subtracted in the update (Eq. 32a); its doc comment
was corrected at `:232`–`:235`. The one stale assertion in
`firmware/tests/attitude_tests.rs` (`bias_estimate_cancels_a_constant_gyro_bias_on_all_axes`)
was changed from `b + truth ≈ 0` to `b − truth ≈ 0` so the existing suite stays
green. This is the only edit outside the three files assigned to this task.

## 4. Fixed-point scales

| quantity | format | conversion |
|---|---|---|
| unit quaternion / unit vector | Q30 (`UNIT = 2³⁰`) | `twice(a,b) = 2ab >> 29` |
| angular rate and `b̂` | Q16 (`RATE = 2¹⁶`) | `b̂` stored Q32, exposed Q16 via `>> 16` |
| time step | Q20 (`DT = 2²⁰`) | `dt_q20_from_us(µs)` |
| gains | Q8 (`GAIN = 2⁸`) | `k·e >> 22` gives Q16 rad/s; `k·e·dt >> 26` gives Q32 rad/s |
| quaternion increment | Q30 half-angle | `(Ω_c[Q16] · dt[Q20]) >> 7` |

These are unchanged by the fix; only signs moved.

## 5. Verification (host only)

`firmware/tests/attitude_math_tests.rs` includes the same source by path and
asserts:

1. `gyro_only_step_matches_quaternion_kinematics_eq45_and_keeps_unit_norm` —
   a known rate produces the known quaternion angle (Eq. 45) with unit norm
   and zero bias.
2. `known_gravity_tilt_reduces_attitude_error_eq32c_eq48b` — a known 0.4 rad
   gravity tilt is acquired: the error shrinks under proportional feedback and
   settles at the known angle. A wrong error sign would drive it to the
   opposite tilt.
3. `constant_gyro_bias_static_gravity_converges_and_does_not_diverge` — with
   a constant bias perpendicular to gravity, `b̂` monotonically approaches
   `+b` at 2 s/8 s/24 s checkpoints and stays bounded; a flipped sign diverges
   and a frozen integral stays at zero. The unobservable yaw component stays
   exactly zero.
4. `constant_gyro_bias_with_gravity_and_north_converges_on_all_axes` — with
   both references, all three bias components converge to the true values.
5. `custom_gains_survive_reset_without_changing_sign_conventions` — positive
   custom gains keep the same convergence direction after reset.
6. `zero_dt_and_degenerate_references_are_loud_without_mutating_state` —
   zero `dt`, zero accel, zero mag (including when the other reference is
   disabled) return the matching `FilterError` and leave quaternion and bias
   untouched.

Commands: `cargo test -p potok-m-firmware`, `cargo clippy --all-targets`,
`cargo fmt --check`, and `cargo check --target thumbv7em-none-eabihf -p
potok-m-firmware --lib` (the last one verifies compilation only).

**Limits.** The step uses a first-order quaternion increment, not an exact
`exp` map; large `dt` or rates are outside the tested regime. Gravity alone
cannot observe yaw bias (Eq. 32c cross product with `e3`), which the tests
assert rather than hide. The per-reference gain weights are design choices,
not values prescribed by the paper. Nothing here establishes flight-worthy
attitude estimation; only the source-convention mapping and host arithmetic
are verified.
