# Firmware math provenance ledger

**Date:** 2026-11-24 · **Repo:** `/home/sehaxe/potok-m-firmware` · **Scope:** every
formula family in `firmware/src` (31 `.rs` files). This is the single ledger;
it consolidates the four per-family documents (`MATH_ATTITUDE.md`,
`MATH_PID.md`, `MATH_AIRDATA.md`, `MATH_VOTING.md`) and the read-only audit
`FWMATH-AUDIT.md` into one table set. No code was changed and no test was run
for this consolidation — test rows below are transcribed from the named test
files and from the audit, not executed here.

**Authority rule.** A source counts as *verified* only when a DOI/ISBN/arXiv ID
or report number was resolved to a live record **and** the relevant equation,
chapter or section was opened. Nothing is filled in from memory. Where no
primary source exists, the row says `NO SOURCE` and names the work needed to
obtain one. An in-tree document is never promoted to an external citation.

## Status and kind legend

| Status | Meaning |
|---|---|
| **SOURCE** | Formula traced to a verified external primary source; identifier and equation/chapter/section named in the row. |
| **IN-TREE** | Authority is a repository-local artifact (module doc, protocol dependency, simulator config, mission doc) or a self-contained algebraic identity. No external primary-source claim. |
| **HEURISTIC** | Shape or numeric value chosen by engineering judgement; no derivation, measurement or primary source. Not a safety argument. |
| **NO SOURCE** | No primary source exists for the claim as written, or the named source was not opened/verified. The "remaining limitation" column says what would be needed. |

| Kind | Meaning |
|---|---|
| **FORMULA** | Mathematical relation that computes a value. |
| **REQUIREMENT** | Mission/design threshold imposed from outside the mathematics (a limit, not a derivation). |
| **CHOICE** | Implementation convention (fixed-point scale, rounding, hash, cadence) chosen locally. |

A row can be `SOURCE` for its formula while its constants remain `HEURISTIC`;
those cases are split into separate rows so a verified equation never lends
authority to an unsourced number. "Closed-form test" means a host test with a
literal or analytically derived expected value — determinism, round-trip and
property-only tests are explicitly labelled as such.

Verified identifiers used below (all resolved, not invented):

- **Mahony** — Mahony, Hamel & Pflimlin, *Nonlinear Complementary Filters on the Special Orthogonal Group*, IEEE TAC **53**(5), 2008, pp. 1203–1218, **DOI 10.1109/TAC.2008.923738**; author manuscript **HAL hal-00488376v1** opened. §III Eq. (32a/b/c), Eq. (33); §IV stability; appendix Eq. (45)/(48b).
- **Solà** — *Quaternion kinematics for the error-state Kalman filter*, **arXiv:1711.02508**, §§1–2, 4.
- **Diebel** — *Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors*, Stanford technical report, 20 Oct 2006, §6.1–6.2 (Hamilton product, scalar-first), §6.4 (quaternion→rotation matrix). No DOI/ISBN exists; do not invent one.
- **AH95** — Åström & Hägglund, *PID Controllers: Theory, Design, and Tuning*, 2nd ed., ISA 1995, **ISBN 978-1-556-17516-9**, Ch. 3 §3.5 (conditional integration), §3.7 (computer program), Eq. (3.9), (3.14), (3.19)–(3.22).
- **AH06** — Åström & Hägglund, *Advanced PID Control*, ISA 2006, **ISBN 978-1-556-17942-6**, book **DOI 10.1002/9781394442102**, Ch. 3 §3.2–§3.5 (Eq. 3.1, 3.14), Ch. 13 §13.3/§13.5 (Eq. 13.5, 13.7, 13.13, 13.23, 13.25).
- **IFAC24** — Sundström et al., *Reference Implementation of the PID Controller*, IFAC PID'24, 2024, `PID-2024/0051.pdf`, §3.6 bumpless transfer, §3.7 anti-windup.
- **Isermann** — *Fault-Diagnosis Systems*, Springer 2006, **DOI 10.1007/3-540-30368-5** (method class only); companion **DOI 10.1016/j.arcontrol.2004.12.002**. Ray & Luck, IEEE CSM 11(2) 1991, **DOI 10.1109/37.67675** (redundancy naming). No page/equation of these books is claimed to be transcribed.
- **Anderson** — *Fundamentals of Aerodynamics*, 5th ed., McGraw-Hill, **ISBN 978-0-07-339810-5**, Ch. 3 §3.4 (incompressible Bernoulli), Ch. 7 (speed of sound), Ch. 8 §8.7 Eq. (8.42) isentropic stagnation, Eq. (8.80) Rayleigh pitot (supersonic, **not** implemented), Example 8.3 (M=1 boundary).
- **US76** — *U.S. Standard Atmosphere, 1976*, **NOAA-S/T-76-1562 / NASA-TM-X-74335 / NTRS 19770009539**: γ=1.4, `p₀=101325 Pa`, `a²=γp/ρ`.

---

## 1. Attitude / quaternion arithmetic — `attitude.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| QAT-1 | Fixed-point scales Q30 unit / Q16 rad/s / Q20 s / Q8 gain / Q32 bias; `dt_q20 = µs·2²⁰/10⁶` | `attitude.rs:52-75` | — | IN-TREE (CHOICE) | `attitude_tests.rs:60` literal 2097 for 2 ms; shifts pinned at `:60-69` | Local convention; no overflow bound for extreme `dt` or rate. |
| QAT-2 | Unit-quaternion constraint `‖q‖=1`, norm² in Q60, normalize | `attitude.rs:92-112`, `:136-158` | Diebel §6.1–6.2 | SOURCE (FORMULA) | `attitude_tests.rs:43-58` literal 0.6/0.8 unit vector | Integer `isqrt` truncation is local; no error bound. |
| QAT-3 | Integer sqrt by Newton iteration `y=(x+n/x)/2` | `attitude.rs:395-407` | — | IN-TREE (CHOICE) | indirect via QAT-2 0.6/0.8 literal | Implementation detail; no worst-case ULP bound. |
| QAT-4 | Hamilton product `q ⊗ dq`, scalar-first `(w,x,y,z)` | `attitude.rs:383-393` | Diebel §6.1–6.2; Solà §1 | SOURCE (FORMULA) | indirect via `attitude_math_tests.rs` gyro-only Eq.45 step | Scalar-first only; `xyzw` tooling (SciPy etc.) is a known mismatch. |
| QAT-5 | First-order increment `q ← q ⊗ [1, Ω_c·dt/2]` | `attitude.rs:343-352` | Mahony Eq. 45 / 48b; Solà §4 | SOURCE (FORMULA) | `attitude_math_tests.rs::gyro_only_step_matches_quaternion_kinematics_eq45_and_keeps_unit_norm` — known angle + unit norm | First order, not exact `exp` map; large `dt`/rate untested. |
| QAT-6 | Per-step renormalization (projection onto unit sphere) | `attitude.rs:351-353` | — | IN-TREE (CHOICE) | unit-norm assertion in QAT-5 test | Not a paper equation; no guarantee at fixed-point overflow. |
| QAT-7 | Estimated up/north = rows 3 and 1 of `R(q)`, i.e. `R̂ᵀe₃`, `R̂ᵀe₁` | `attitude.rs:277-288` | Diebel §6.4 | SOURCE (FORMULA) | `attitude_tests.rs:91-97` aligned refs keep identity attitude | World-axis label convention; see MAH-6 for the unsourced north model. |
| QAT-8 | Cross product of two Q30 unit vectors | `attitude.rs:374-381` | — | IN-TREE (CHOICE) | indirect via 0.4 rad tilt test | Q30 truncation on each component; no bound. |

## 2. Mahony SO(3) complementary filter — `attitude.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| MAH-1 | Innovation `ω_mes = Σ kᵢ (vᵢ × v̂ᵢ)` — measured cross estimated | `attitude.rs:290-303` | Mahony §III Eq. (32c) | SOURCE (FORMULA) | `attitude_math_tests.rs::known_gravity_tilt_reduces_attitude_error_eq32c_eq48b` (0.4 rad) | Gravity alone cannot observe yaw bias; asserted, not hidden. |
| MAH-2 | Bias integral `ḃ̂ = −kI·ω_mes`, shift 26 Q8·Q30·Q20→Q32 | `attitude.rs:308-324` | Mahony Eq. (32b) | SOURCE (FORMULA) | `...constant_gyro_bias_static_gravity_converges_and_does_not_diverge` — `b̂→+b` at 2/8/24 s | Paper is internally inconsistent by 2: Eq. 32c uses `kᵢ`, Eq. 34/48a use `kᵢ/2`; absorbed into weights. |
| MAH-3 | Rate correction `Ω_c = Ω_y − b̂ + kP·ω_mes`, bias `>>16` | `attitude.rs:326-341` | Mahony Eq. (32a) | SOURCE (FORMULA) | `...gravity_and_north_converges_on_all_axes` | Both bias signs were inverted before commit `1fbb273`; accessor doc corrected at `:233-235`. |
| MAH-4 | Lyapunov sign `V̇ = −kP‖Pa(R̃M)‖² ≤ 0` | `attitude.rs:29-33` (doc only) | Mahony Eq. (33) | SOURCE (FORMULA) | none — continuous-time sign, no executed closed-form test | No discrete fixed-point stability proof; names the sign convention only. |
| MAH-5 | Default gains `kp=2.0, ki=0.5, kp_mag=1.0, ki_mag=0.25` and weight ratio `w_m/w_a=0.5` | `attitude.rs:37-40`, `:161-188` | — | HEURISTIC (CHOICE) | none — custom-gain test checks direction, not these values | No tuning reference or calibration datum; tied to Q8 scale. |
| MAH-6 | Fixed world north `+e₁`, no declination, no horizontal projection of the field | `attitude.rs:19-25`, `:284-288` | — | NO SOURCE (FORMULA) | none — tests use ideal `NORTH=(UNIT,0,0)` | Unlevelled/distorted field feeds roll/pitch. Needs a geomagnetic model + field-calibration data to become sourced. |

## 3. PID / Q16.16 — `control_pid.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| PID-1 | `Fx` Q16.16, `i64` saturating product, round half away from zero | `control_pid.rs:110-135` | — | HEURISTIC (CHOICE) | `control_pid_tests.rs:119-140` exact f32≡Fx agreement | Rounding policy is firmware choice; f32 path is the reference operation sequence. |
| PID-2 | Per-step coefficients `ki_dt=Ki·h`, `kd_dt=Kd/h`, `α=Td/(Td+N·h)` | `control_pid.rs:176-220` | AH06 (13.7), (13.13) | SOURCE (FORMULA) | `pid_math_tests.rs::si_gains_match_the_book_discrete_coefficients` | Backward-Euler D filter, not AH95's Tustin `ad`. |
| PID-3 | `e = r − y` | `control_pid.rs:290-291` | AH95 (3.9); AH06 (3.1) | SOURCE (FORMULA) | `proportional_step_response_is_instant_linear_and_unsigned_free` | Setpoint weight `b=1` (no P pre-filter). |
| PID-4 | `P = kp·e` (b=1) | `control_pid.rs:292-293` | AH06 (13.5), (13.17)–(13.18) | SOURCE (FORMULA) | same P step test | Instantaneous; no discretisation. |
| PID-5 | `D = −kd_dt·LPF(Δy)`; filtered increment `f += α(Δy−f)`, `c=0` | `control_pid.rs:279-288` | AH95 (3.22); AH06 (3.14), (13.13) | SOURCE (FORMULA) | `derivative_step_response_is_on_measurement_with_negative_sign`, `derivative_filter_is_the_geometric_recursion_of_ah13_13`, `setpoint_step_causes_no_derivative_kick` | First-tick measurement history primed to zero → transient if first measurement nonzero. |
| PID-6 | `I += ki_dt·e` forward Euler | `control_pid.rs:307-310` | AH06 (13.7) | SOURCE (FORMULA) | `integral_step_response_is_forward_euler_then_clamped` | Euler stability at the chosen `h` not analysed. |
| PID-7 | Separate `i_limit` clamp on the integral contribution | `control_pid.rs:296-297`, `:310` | — | HEURISTIC (CHOICE) | `integral_limit_is_symmetric_and_binds_before_output_saturation` | No book counterpart; value needs a tuning datum. |
| PID-8 | Bumpless preload `I ← u_prev − P − D`, skip this tick's integration | `control_pid.rs:299-305`, `:323-324` | AH06 (13.25); IFAC24 §3.6 | SOURCE (FORMULA) | `setpoint_change_is_bumpless_in_closed_form`, `bumpless_preload_uses_the_saturated_previous_output` | Book form is parameter-change; applying it to setpoints makes P-only steps output-continuous. |
| PID-9 | `u = sat(P+I+D, ±out_limit)`, counted strict-outside flag | `control_pid.rs:313-315` | AH95 §3.7 program; AH06 (13.23) | SOURCE (FORMULA) | `output_saturation_is_symmetric_and_counted` | `out_limit=100` raw is a model scale, not a paper value (see PID-11). |
| PID-10 | Conditional integration: freeze only if saturated **and** update pushes further out | `control_pid.rs:317-325` | AH95 §3.5; AH06 §3.5; IFAC24 §3.7 | SOURCE (FORMULA) | `anti_windup_freezes_at_the_edge_and_unwinds_in_one_tick` (+lower limit) | No back-calculation / `T_t`; reversal unwinds in one tick by design. |
| PID-11 | `out_limit = MAX_CORRECTION_RAW = 100` raw = 0.1 rad at 0.001 rad/count | `control_pid.rs:52-55`; `lib.rs:44` | protocol/model scale | IN-TREE (REQUIREMENT) | `control_pid_tests.rs:154-171`; `interlocks_tests.rs:32-45` | Model-scale assumption, not a derived constant. |

## 4. Redundant-sensor voting — `sensor_voting.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| VOT-1 | Dimensionless residual `r=\|a−b\|/(abs+rel·max(\|a\|,\|b\|))`, fault iff `r>1` | `sensor_voting.rs:83-105`, `:256-258` | Isermann DOI 10.1007/3-540-30368-5; Ray & Luck DOI 10.1109/37.67675 | SOURCE (FORMULA) | `voting_math_tests.rs::gyro_only_fault_is_detected_when_accel_is_quiet`, `accel_only_fault_is_detected_when_gyro_is_quiet` | Method-class attribution only — no page/equation transcribed; threshold numbers not sourced. |
| VOT-2 | Per-channel limits: gyro rad/s, accel m/s², independent relative terms | `sensor_voting.rs:158-201`, `:239-253` | — | IN-TREE (FORMULA) | `voting_math_tests.rs::single_mixed_threshold_cannot_pass` — literal `0.10/(0.05+0.02·9.81)=0.406 < 1` for the retired expression | Fix is dimensional, not calibrated; real noise floors unmeasured. |
| VOT-3 | 1 reading → value; 2 → midpoint; 3 → medoid minimizing `Σr` | `sensor_voting.rs:430-452` | — | IN-TREE (FORMULA) | `sensor_voting_tests.rs:27-59`, `:176-210` literal medoid/midpoint | Textbook median/medoid; no primary source opened in the supplied inputs. |
| VOT-4 | Stuck quarantine: bit-identical ≥ `stuck_samples` ticks **and** a moved peer; peers from pre-tick state | `sensor_voting.rs:352-391` | — | IN-TREE (FORMULA) | `sensor_voting_tests.rs:86-116`, `:118-131`, `:134-145` | Common-mode failure is undetectable by construction. |
| VOT-5 | `stuck_samples = 8` | `sensor_voting.rs:261-269` | — | HEURISTIC (CHOICE) | none — tests pin counter semantics only | No noise-floor or flight datum. |
| VOT-6 | `confidence = base·(1 − 0.8·clamp01(worst_r−1))`, bases 0.35/0.75/1.0 | `sensor_voting.rs:37-46`, `:462-481` | — | HEURISTIC (CHOICE) | `sensor_voting_tests.rs:27-40` pins 0.2; `:42-59` pins 0.75 | No paper, calibration or flight datum; explicitly not a safety argument. The `n/3` alternative is *not* implemented. |
| VOT-7 | `confidence_u8 = round(clamp01(c)·255)` | `sensor_voting.rs:291-294` | protocol byte scale | IN-TREE (CHOICE) | `sensor_voting_tests.rs:27-40` pins 51 | Quantization only; does not validate the confidence. |
| VOT-8 | Production tolerances: IMU `new(0.05, 0.02)`, baro `new(0.5, 0.02)` | `lib.rs:352-353` | — | HEURISTIC (REQUIREMENT) | none — tests use their own constructed tolerances | Semantics fixed by VOT-2, numbers unchanged; replace with `Tolerance::imu` from measured floors. |
| VOT-9 | Outlier = third slot when it exceeds tolerance against the selected value | `sensor_voting.rs:454-460` | — | IN-TREE (FORMULA) | `sensor_voting_tests.rs:110-115` | Only identifiable with three channels; dual vote names no outlier. |

## 5. Mach / pitot / runtime shield — `shield_sample.rs`, `interlocks.rs`, `replay.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| MCH-1 | `a² = γp/ρ`, `γ=1.4` for air | `shield_sample.rs:17-21` | Anderson Ch. 7; US76 | SOURCE (FORMULA) | none direct | Dry-air assumption; comment's "same value as simulator" not verified here. |
| MCH-2 | Subsonic isentropic pitot `q_c/p = (1+0.2M²)^3.5 − 1`, `q_c = p_t−p = dp` | `shield_sample.rs:214-228` | Anderson 5th ed. ISBN 978-0-07-339810-5, Ch. 8 §8.7 Eq. (8.42) | SOURCE (FORMULA) | `mach_math_tests.rs` trusted table M=0.1/0.3/0.6/0.9 (dp 711/6528/27915/70046 Pa) | Subsonic only. Must state whether the driver reports impact pressure `q_c` or simulator `q=½ρV²`; supersonic Eq. (8.80) not implemented. |
| MCH-3 | Squared solver `(1+0.2x)⁷=(1+dp/p)²`, fixed 32-step bisection on `x=M²` | `shield_sample.rs:194-212`, `:236-251` | — | IN-TREE (CHOICE) | table from MCH-2 plus monotonicity/bit-reproducibility tests (property, not closed-form) | No formal error bound; 32 steps fixed; assumes f32 `pow7` stays monotone. |
| MCH-4 | `sqrt` seed `from_bits((bits>>1)+0x1fc0_0000)` + 4 Newton steps | `shield_sample.rs:259-271` | — | IN-TREE (CHOICE) | `shield_sample_tests.rs:225` — dp 1..=35 000 Pa sweep vs `f32::sqrt`, worst **1 ULP** at dp=1 | Representative sweep, not all f32 bit patterns; subnormals/extremes untested. Old "well below one ULP" claim retracted in `FWMATH-PITOT.md`. |
| MCH-5 | M=1 boundary `dp/p = 1.2^3.5−1 = 0.892929`; above → `AboveSubsonicLimit` | `shield_sample.rs:206-207`, `:238-240` | Anderson Example 8.3 | SOURCE (FORMULA) | `mach_math_tests.rs` loud refusal at/above boundary, no clamped M=1 sample | `above_subsonic` counter is not included in the `sil.rs` `no_driver+non_finite+non_positive` sum (telemetry blind spot). |
| MCH-6 | `pitot_pa_to_i16`: clamp to `i16::MIN..=MAX` + saturation flag | `shield_sample.rs:175-184`, `:153-166` | wire field width | IN-TREE (CHOICE) | `shield_sample_tests.rs:147` (40 000 Pa→`i16::MAX`), `:186` (exactly `i16::MAX` not counted) | Telemetry field cannot hold the ~90 kPa at M=1; sample stays valid, only the field saturates. |
| MCH-7 | Replay `pitot_i16(pa) = finite ? pa as i16 : 0` | `replay.rs:159-165` | — | NO SOURCE (FORMULA) | none — indirect corruption tests only | Non-finite maps to a value indistinguishable from a valid zero; `0` is not an invalid-reading sentinel. Needs a typed `Unavailable` seam like `shield_sample.rs`. |
| MCH-8 | Shield order: finite check → `dp<=0` → `mach>limit`, strict `>` | `interlocks.rs:260-275` | — | IN-TREE (FORMULA) | `interlocks_tests.rs:273-437` incl. M=0.6 accepted | Unavailable → `PitotInvalid` veto; boundary value convention is a design choice. |
| MCH-9 | `mach_limit = 0.6` | `interlocks.rs:127-135` | `docs/RUNTIME_SHIELD.md`, `vehicle/DESIGN_INPUT.md` | IN-TREE (REQUIREMENT) | `interlocks_tests.rs:274-277` boundary | Mission requirement, **not** a mathematically derived constant; `Safe` profile uses 0.5 as a separate design choice. |

## 6. Calibration interpolation — `calibration_table.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| CAL-1 | Q8 temp, Q12 bias/scale, `FRAC_SHIFT=16`, `OUT_DEN=2¹²·2¹²` | `calibration_table.rs:12-18`, `:29-46` | — | IN-TREE (CHOICE) | `calibration_table_tests.rs:60-69` | Local fixed-point convention; no standard claimed. |
| CAL-2 | Table validation: ascending points, `0<scale≤16`, raw ≤ `2²⁰` | `calibration_table.rs:86-127` | — | IN-TREE (CHOICE) | `calibration_table_tests.rs:127-158`, `:160-177` loud errors | Limits are design choices, not physics. |
| CAL-3 | Segment search + blend `(T−t0)/(t1−t0)` in Q16; endpoint clamp | `calibration_table.rs:139-157` | — | IN-TREE (FORMULA) | `calibration_table_tests.rs:76-91` [9,6,3], `:93-109` [8,0,−8] | Linear interpolation identity; no primary source opened. Truncating blend. |
| CAL-4 | `out = round(((raw·2¹² − bias)·scale)/(2¹²·2¹²))`, half away from zero | `calibration_table.rs:160-200` | — | IN-TREE (FORMULA) | `calibration_table_tests.rs:49-74` exact [20,10,5] | **No firmware caller** (`lib.rs:15` declares the module only): no sensor is actually calibrated. |
| CAL-5 | `lerp_q12(a,b,f) = a + (b−a)·f/2¹⁶` (truncating) | `calibration_table.rs:202-204` | — | IN-TREE (CHOICE) | indirect via `:76-91` | Truncation bias uncharacterised. |

## 7. Deployment / separation — `deployment.rs`, `separation.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| DEP-1 | Defaults `30/2/5/15/−1500/3/3` m,m,m,m,cm/s,samples,m, under a "Verified thresholds" header | `deployment.rs:28-59` | — | NO SOURCE (REQUIREMENT) | `deployment_tests.rs:54-70` pins literals (implementation, not the choice) | No flight-data analysis, standard or paper named. Needs flight-log ground truth or a range/airworthiness standard. |
| DEP-2 | Config validation inequalities (floor < low ≤ apogee min; drop>0; climb>0; confirm>0; rate<0) | `deployment.rs:62-104` | — | IN-TREE (CHOICE) | `deployment_tests.rs:54-70` | Consistency constraints only; say nothing about threshold validity. |
| DEP-3 | Peak tracking; saturating descent counter; reset on non-negative speed | `deployment.rs:216-241` | — | IN-TREE (FORMULA) | `deployment_tests.rs:72-103`, `:228-265` | State-machine logic, no external source claim. |
| DEP-4 | Three triggers: apogee descent (peak≥5, alt≥30, alt≤peak−2, 3 desc ticks); fast descent (v≤−1500, alt≥3); low altitude (alt≤15, descent confirm) | `deployment.rs:243-260` | — | NO SOURCE (REQUIREMENT) | `deployment_tests.rs:154-226` exact trigger per case | Tests validate implementation of the decision table, not the choice of numbers. Same evidence gap as DEP-1. |
| DEP-5 | One-shot release; window by wire phase `COAST..=DESCENT` | `deployment.rs:301-306` | — | IN-TREE (CHOICE) | `deployment_tests.rs:267-316` | Assumes wire phase ordering is meaningful for a release window. |
| SEP-1 | `spread = \|a−b\|` widened i64→u32 | `separation.rs:53-60` | — | IN-TREE (FORMULA) | `separation_tests.rs:358-370` full-range `u32::MAX` literal | Widening avoids `i32::MIN/MAX` overflow; no other limit. |
| SEP-2 | Two arm channels OR-latched; both required before leaving Idle | `separation.rs:190-200` | — | IN-TREE (CHOICE) | `separation_tests.rs:55-111` | Design redundancy policy; no source claimed. |
| SEP-3 | Monotonic `now_us`; sensor spread ≤ tolerance on every active tick | `separation.rs:205-226` | — | IN-TREE (FORMULA) | `separation_tests.rs:161-190`, `:247-261` | Numeric tolerance is a design value (see threshold table). |
| SEP-4 | Conservative epoch = later stamp; skew tolerance; future-stamp abort; `now−entered ≥ delay_us` | `separation.rs:249-296` | — | IN-TREE (CHOICE) | `separation_tests.rs:113-159`, `:192-245` | "Never earlier than either sensor says" is a design convention, not a sourced criterion. |
| SEP-5 | Separation confirmed only when both sensors ≥ level; ignition latched | `separation.rs:231-244` | — | HEURISTIC (REQUIREMENT) | `separation_tests.rs:263-308` | Level not derived from a sensor specification or flight datum. |

## 8. Scheduler / jitter / watchdog — `scheduler.rs`, `watchdog.rs`, `main.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| SCH-1 | 500 Hz, `TICK_PERIOD_US = 10⁶/500 = 2000` | `scheduler.rs:18-20` | — | IN-TREE (REQUIREMENT) | `scheduler_tests.rs:14-30` literal 2000 | Mission cadence, not a derivation. |
| SCH-2 | Jitter min/max/last/Σ\|e\|; `mean_abs_us = Σ\|e\|/n` (integer trunc) | `scheduler.rs:40-75` | — | IN-TREE (FORMULA) | `scheduler_tests.rs:83-102` — 67/3 → 22 | Truncation; module not wired into the shipping loop. |
| SCH-3 | `deadline_for(i) = origin + i·period` (wrapping) | `scheduler.rs:168-181` | — | IN-TREE (FORMULA) | `scheduler_tests.rs:14-30`, `:214-236` u32 rollover | Not wired: `main.rs:202,220` uses `Ticker::every(2 ms)`. |
| SCH-4 | `delta = now − deadline`; `missed = delta/period`; rejoin by `steps·period` | `scheduler.rs:185-214` | — | IN-TREE (FORMULA) | `scheduler_tests.rs:104-145`, `:214-236` | Not wired; no flight jitter measurement exists. |
| SCH-5 | On-time iff `Δnow ≤ 2000+250 µs`; 3 consecutive misses trip | `watchdog.rs:42-49`, `:83-100` | — | IN-TREE (REQUIREMENT) | `watchdog_tests.rs`, `watchguard_tests.rs` | 250 µs slack is an unsourced allowance, not a measured margin. |
| SCH-6 | `may_feed = cycle_complete && !deadman_tripped`; `must_zero_output` | `watchdog.rs:8-12`, `:170-194` | — | IN-TREE (CHOICE) | `watchdog_tests.rs:17` | Feed policy is local safety design. |
| SCH-7 | Shipping tick is Embassy `Ticker::every(Duration::from_millis(2))` | `main.rs:202`, `:220` | — | IN-TREE (CHOICE) | none | SCH-2..SCH-4 never execute in flight; no measured jitter/deadline claim is supported. |

## 9. Phase FSM / replay timing — `phase.rs`, `phase_fsm.rs`, `replay.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| PH-1 | Forward-only 5-edge table LiftOff/Burnout/Apogee/Descent/Touchdown | `phase.rs:37-50` | — | IN-TREE (CHOICE) | `phase_fsm_tests.rs` (shared tests) | No timestamps or transition log in the shipping FSM. |
| PH-2 | `ai_allowed = phase != BURN && !(flags & MOTOR_ARMED)` | `phase.rs:52-56` | — | IN-TREE (REQUIREMENT) | `phase_fsm_tests.rs` | Two independent gates by design; no external safety standard cited. |
| PH-3 | Timestamped FSM: `now_us ≥ entered_at_us`, transition log, rejection counter | `phase_fsm.rs:174-202` | — | IN-TREE (FORMULA) | `phase_fsm_tests.rs` (9 tests) | **Unwired** (`lib.rs:25` vs `control.rs:11`, `protocol_link.rs:15`, `main.rs:163`): tests exercise dead production code. |
| PH-4 | `last_transition` = highest reached phase slot ("forward-only ⇒ phase order is time order") | `phase_fsm.rs:151-170` | — | IN-TREE (CHOICE) | `phase_fsm_tests.rs` | Same unwired limitation as PH-3. |
| PH-5 | Phase reconstruction: `height<=0 && t>0.5`; `\|climb\|<2 m/s && height>10 m`; 5 m / 10 m; `burn_end=9.95 s` | `replay.rs:51`, `:194-212` | simulator `phase_at` behaviour, module disclaims bit-exactness | NO SOURCE (FORMULA) | `replay_tests.rs:130-175` four literal cases | Ordering calls a stationary pad (`height<=0`, `t>0.5`) `Landed` before the boost check; vertical speed is a proxy for horizontal speed. Needs the simulator source and a ground-truth phase column. |
| PH-6 | 50 Hz→500 Hz, `TICKS_PER_SIM_FRAME = 500/50 = 10` | `replay.rs:26-33` | — | IN-TREE (CHOICE) | `replay_tests.rs:77-104` | Assumes exact divisibility of the source rate. |
| PH-7 | Confidence byte `round(clamp(conf,0,1)·255)` | `replay.rs:558-564` | protocol byte scale | IN-TREE (CHOICE) | `replay_tests.rs:106-128` pins 128 for 0.5 | Quantization only. |
| PH-8 | Sim `[roll,pitch]` → wire `[pitch,yaw]` via `pp::reply_*_from_rad` | `replay.rs:565-576` | protocol-owned conversion | IN-TREE (CHOICE) | `replay_tests.rs:106-128` pins −20/−20 | External codec; no independent local vector. |

## 10. Framing / CRC / protocol — `protocol.rs`, `protocol_link.rs`, `telemetry_log.rs`, `self_test.rs`, `uart.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| CRC-1 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) for frames and telemetry | `protocol.rs:49-53`; `telemetry_log.rs:14`, `:112`, `:131`; `firmware/Cargo.toml:17` (rev `fdc485d`) | algorithm named; implementation external and not opened here | NO SOURCE (FORMULA) | none for frame/telemetry — encode→decode round-trip uses the same external function | No independent check vector in this repo. Need a published CCITT-FALSE test vector and a local assertion of polynomial/init. |
| CRC-2 | Golden config CRC `0xABF7` over bytes `[1,0,0x50,0x4D,100,0,80,0]` | `self_test.rs:41-51` | — | IN-TREE (CHOICE) | `self_test_tests.rs:86-90` pins CRC and layout; audit recomputed independently → matches | Locally computed, not an external vector; only the 8-byte config image is covered. |
| CRC-3 | `SeqGate`: `issue = last.wrapping_add(1)`; strict equality on reply | `protocol.rs:55-71` | — | IN-TREE (FORMULA) | `safety.rs`, `fault_inject_tests.rs:196-208` | One outstanding request assumption. |
| CRC-4 | Reply assembler: magic `u16::from_le_bytes`, resync keeping newest byte as magic start | `protocol.rs:99-142` | — | IN-TREE (CHOICE) | `protocol_link_tests.rs` | Bytes before magic discarded without counting a miss (documented). |
| CRC-5 | Double-buffer `back = 1−active`; swap `active = back`; overrun counted | `protocol_link.rs:20-69`; `uart.rs:36-72` | — | IN-TREE (CHOICE) | `protocol_link_tests.rs:110-135` | Buffer count is fixed at two. |
| CRC-6 | Request template: patch seq/time/phase/flags at fixed offsets, recompute `pp::crc16` over `buf[..CRC_OFF]` | `protocol_link.rs:132-166` | protocol-owned offsets/CRC | IN-TREE (CHOICE) | `protocol_link_tests.rs` | Offsets owned by external crate; depends on CRC-1. |
| CRC-7 | Locked-length assembler: after magic match, one slice copy of `REPLY_SIZE−len` | `protocol_link.rs:175-243` | — | IN-TREE (CHOICE) | `protocol_link_tests.rs` | Assumes the first two bytes cannot change once matched. |
| CRC-8 | Telemetry record `HEADER_LEN=13`, `CRC_LEN=2`, `MIN_RECORD_SIZE=15`, LE fields, CRC trailing LE | `telemetry_log.rs:20-24`, `:78-115`, `:119-150` | protocol-owned CRC | IN-TREE (CHOICE) | `telemetry_log_tests.rs` round-trip only — **no** closed-form CRC value | Correctness of polynomial/init untested locally (CRC-1). |
| CRC-9 | Ring `write=(write+1)%N`; full → `head=write`; flush `head=(head+count)%N` | `telemetry_log.rs:297-352` | — | IN-TREE (CHOICE) | `telemetry_log_tests.rs` | Oldest-frame overwrite policy; no source claim. |
| CRC-10 | Replay continuity `expected = last.wrapping_add(1)`; stop at first fault | `telemetry_log.rs:409-453` | — | IN-TREE (CHOICE) | `telemetry_log_tests.rs` | Sequence semantics local. |
| CRC-11 | `LogGate`: state-change > debug > periodic cadence; `since_emit.saturating_add(1)` | `telemetry_log.rs:543-579` | — | IN-TREE (CHOICE) | `telemetry_log_tests.rs` | Logging policy, not a mathematical formula. |

## 11. Profiles / cycle budget — `profile.rs`, `perf.rs`, `timing_budget.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| PRF-1 | Profile ladder: Safe/Balanced/Performance gains, mach 0.5/0.6, `log_every_ticks` 1/1/10 | `profile.rs:56-64`, `:113-158` | — | HEURISTIC (CHOICE) | `profile_tests.rs:41-53`, `:55-77` pin relationships only | No derivation for gain or mach values; Balanced reproduces existing envelopes by assertion. |
| PRF-2 | FNV-1a 64 (offset `0xcbf29ce484222325`, prime `0x100000001b3`) over LE fields, floats by bits; manifest = hash of 3 profile hashes | `profile.rs:187-217`, `:322-364` | named algorithm; no primary source opened | IN-TREE (CHOICE) | `profile_tests.rs:136-159` stability/distinctness — **no golden digest** | Activation gate, not a security boundary. A published FNV test vector would be needed for a golden. |
| PRF-3 | `CORE_HZ=550e6`, 3 flash wait states — "as transcribed" from ST RM0468 table 16, **NOT VERIFIED on board** | `perf.rs:42-51` | ST RM0468 table 16 (unverified) | NO SOURCE (REQUIREMENT) | `perf.rs:317-348` pins 24 000/27 500 against the assumption | Needs a board clock measurement and an opened datasheet copy before the budget can be claimed for hardware. |
| PRF-4 | `cycles_per_us=hz/10⁶`; `budget=us·cycles_per_us`; `tick_cycles=hz/tick_hz`; 50 µs → 27 500 cycles at 550 MHz | `perf.rs:55-71` | — | IN-TREE (CHOICE) | `perf.rs:333-335`; `timing_budget_tests.rs:26-37` | Design allowance, not a measurement. |
| PRF-5 | Per-op constants: RX 6/B, CRC 4/B, quant 12/field, gate 12, axis 4, overhead 200, shield 32, watchdog 8 | `perf.rs:76-96` | — | HEURISTIC (CHOICE) | self-consistent pinned tests only | Explicitly "conservative upper bounds", not timings of the host. |
| PRF-6 | Branchless `ai_gate` over 256-entry phase LUT | `perf.rs:98-125` | — | IN-TREE (CHOICE) | `perf_tests.rs:40-79` all 256 codes | Test/bench-only; not called by `control::step`. |
| PRF-7 | Q15 `from_ratio = num·32768/den` cap ±2.0; `apply = round_half_away(q15·i16/2¹⁵)` with i16 clamp | `perf.rs:127-161` | — | IN-TREE (CHOICE) | `perf_tests.rs` gain/scaling tests | Test/bench-only. |
| PRF-8 | `CycleEstimate.cycles = rx+crc+quantize+gate+fixed_math+overhead`, assert ≤ 27 500 | `perf.rs:181-225` | — | IN-TREE (CHOICE) | `timing_budget_tests.rs` | Legacy SIL reservation: "charges work no shipped function performs". |
| PRF-9 | `StepCycles.cycles = rx+crc+phase_gate+shield+watchdog+clamp+overhead` | `perf.rs:228-297` | — | IN-TREE (CHOICE) | `timing_budget_tests.rs` | Authoritative model of `control::step`, but not called by it. |
| PRF-10 | Derived `CYCLES_PER_US/TICK_US/TICK_CYCLES`; compile-time asserts; worst case over 4 SIL scenarios; LOUD panic on overrun | `timing_budget.rs:29-41`, `:138-143`, `:161-191`, `:204-219` | — | IN-TREE (CHOICE) | `timing_budget_tests.rs:26-37`, `:221-244` — 891 ppm @480 MHz, 778 ppm @550 MHz, margin 26 644 cycles / 48 µs | Model arithmetic only; no hardware timing measurement. |

## 12. Other numeric families — `fault_inject.rs`, `search_beacon.rs`, `buzzer.rs`

| ID | Formula | file:line | Source | Status | Closed-form test | Remaining limitation |
|---|---|---|---|---|---|---|
| OTH-1 | xorshift64* shifts 12/25/27, multiplier `0x2545F4914F6CDD1D`, zero seed → `0x9E3779B97F4A7C15` | `fault_inject.rs:207-230` | PRNG family named; no paper pin opened | IN-TREE (CHOICE) | `fault_inject_tests.rs:167-174` determinism only — no golden stream | Deterministic, not a validated random source. Needs Marsaglia's note if a citation is required. |
| OTH-2 | Fisher–Yates `j = rng.next_u64() % i`, swap `i−1, j` | `fault_inject.rs:232-251` | algorithm named; no citation | IN-TREE (CHOICE) | `fault_inject_tests.rs:131-165` reproducibility/uniqueness, **no golden permutation** | Modulo bias from `% i`; acceptable for fault ordering only. |
| OTH-3 | Battery bands 3300/3600/3900 mV → radio 0/5/10/20 %, buzzer 0/25/50 % | `search_beacon.rs:29-40`, `:154-165` | — | HEURISTIC (REQUIREMENT) | `search_beacon_tests.rs` pins bands/duty | No pack datasheet or flight datum fixes the boundaries. |
| OTH-4 | `slots = duty%·20/100`; `in_slot = tick%20 < slots` | `search_beacon.rs:25-26`, `:273-291` | — | IN-TREE (FORMULA) | `search_beacon_tests.rs` | Duty is a hint, not a measured regulatory compliance. |
| OTH-5 | 24-tick (2.4 s) search pattern; step advances at note end | `search_beacon.rs:43`, `:308-322` | — | IN-TREE (CHOICE) | `search_beacon_tests.rs` pattern tests | Local cadence design. |
| OTH-6 | Equal-temperament notes A4=440 Hz, rounded to whole Hz | `buzzer.rs:20-36` | equal temperament named; no primary source opened | IN-TREE (CHOICE) | `buzzer_tests.rs:160-163` pins 880/1047 | Rounding error uncharacterised; a tuning standard would be needed for a citation. |
| OTH-7 | Note advance by tick count; `freq_hz==0` → silence | `buzzer.rs:163-188` | — | IN-TREE (CHOICE) | `buzzer_tests.rs` pattern tests | None material. |

---

## Mission / requirement thresholds — explicitly NOT mathematical formulas

These are imposed limits, design values or operational constants. A closed-form
test pins their *implementation*; it never validates the *choice*. None is a
derived equation, and none should be presented as one in a safety argument.

| Threshold | file:line | Where the requirement comes from | Status |
|---|---|---|---|
| Mach limit 0.6 (Safe profile 0.5) | `interlocks.rs:135`; `profile.rs:113`, `:155` | `docs/RUNTIME_SHIELD.md`, `vehicle/DESIGN_INPUT.md`; Safe value is a design choice | IN-TREE (REQUIREMENT) |
| `max_command=100` raw (=0.1 rad), `max_delta=10` raw, `min_interval=1000 µs` | `interlocks.rs:132-134` | model scale / design choice | IN-TREE (REQUIREMENT) |
| Deployment `30/2/5/15/−1500/3/3` | `deployment.rs:51-58` | none — header says "Verified" without evidence | **NO SOURCE (REQUIREMENT)** |
| Separation delay/skew/sensor tolerance/separation level | `separation.rs:69-76` | design values | HEURISTIC (REQUIREMENT) |
| IMU tolerance `0.05/0.02`, baro `0.5/0.02`; `stuck_samples=8` | `lib.rs:352-353`; `sensor_voting.rs:269` | design values; noise floors unmeasured | HEURISTIC (REQUIREMENT) |
| Voting confidence `0.35/0.75/1.0`, penalty `0.8` | `sensor_voting.rs:41-46` | design values | HEURISTIC (CHOICE) |
| Mahony gains `2/0.5/1/0.25` | `attitude.rs:173-178` | design values | HEURISTIC (CHOICE) |
| Watchdog on-time `2000+250 µs`, 3 misses | `watchdog.rs:44-49` | design allowance | IN-TREE (REQUIREMENT) |
| Tick 500 Hz; cycle budget 50 µs / 27 500 cycles | `scheduler.rs:18-20`; `perf.rs:42-71` | design budget on an unverified 550 MHz assumption | IN-TREE (REQUIREMENT) / NO SOURCE for the clock |
| Battery bands 3300/3600/3900 mV and duty caps | `search_beacon.rs:29-40` | design values | HEURISTIC (REQUIREMENT) |
| Replay burn end `9.95 s` | `replay.rs:51` | simulator `config/default.toml` default (in-tree) | IN-TREE (REQUIREMENT) |
| `MAX_CONSECUTIVE_MISSES=3` | `lib.rs:42` | design policy | IN-TREE (REQUIREMENT) |

## Status counts

Row counts over the 12 tables above (each row counted once by its status and
once by its kind):

| Status | Rows (12 formula tables) |
|---|---|
| SOURCE | 20 |
| IN-TREE | 64 |
| HEURISTIC | 10 |
| NO SOURCE | 7 |
| **Total** | **101** |

The separate mission/requirement table adds 12 threshold rows (5 IN-TREE, 4
HEURISTIC, 1 NO SOURCE, 1 mixed IN-TREE/NO-SOURCE for the unverified clock), so
**113 rows total** in this ledger. The largest verified block is control
(Mahony Eq. 32, PID Eq. 3.x/13.x, Anderson Eq. 8.42). The largest unverified
block is the set of mission thresholds and design constants that sit *next to*
those sourced formulas: `HEURISTIC + NO SOURCE = 17` of 101 formula rows, plus
5 of the 12 threshold rows.

## Top three remaining risks

1. **Flight-critical thresholds with no source, one of them labelled "Verified".** `deployment.rs:28-59` / `:243-260` carries the six parachute triggers under a "Verified thresholds" header with no flight-data analysis, standard or paper; separation levels (`separation.rs:69-76`) and the voting tolerances/confidences (`lib.rs:352-353`, `sensor_voting.rs:37-46`) are likewise unmeasured. Their tests pin arithmetic, not the choice. Obtain flight-log ground truth, a range/airworthiness standard and measured sensor noise floors, then re-derive each number; until then none of these is a safety argument.

2. **Tested math that is not in the flying tick, on an unverified clock assumption.** Scheduler jitter/deadline math (`scheduler.rs:40-214`), calibration interpolation (`calibration_table.rs:139-200`), the timestamped FSM (`phase_fsm.rs:174-202`), and `perf`/`timing_budget` models have closed-form tests but no production caller (`main.rs:202,220` uses `Ticker::every(2 ms)`; `control::step` calls none of the Q15/gate/cycle helpers). `CORE_HZ=550e6` and the per-op cycle constants are assumptions, not measurements. No timing, jitter or calibration claim about the shipped tick is currently supported. Measure on the target board and wire the intended module before such a claim.

3. **Airdata and phase reconstruction blind spots on the veto path.** The Mach solver is subsonic-only and its correctness depends on whether the driver reports impact pressure `q_c` or simulator `q=½ρV²` (MCH-2); `replay.rs:159-165` maps non-finite pitot to `0`, indistinguishable from a valid reading; `phase_code` (`replay.rs:194-212`) can call a stationary pad `Landed`; `above_subsonic` is absent from the `sil.rs` tally; and frame/telemetry CRC has no independent local vector (CRC-1). State the pressure quantity, add typed unavailable/out-of-domain reporting to replay and SIL, fix the phase ordering, and pin a published CRC test vector.

## Consolidation note

This ledger supersedes the per-family source sections of `docs/MATH_ATTITUDE.md`,
`docs/MATH_PID.md`, `docs/MATH_AIRDATA.md` and `docs/MATH_VOTING.md` as the
single provenance index; those documents remain as the detailed derivations and
gate lists. The audit inputs were `.bulba/night/FWMATH-{AUDIT,SOURCES,MAHONY,
PID,MACH,PITOT,VOTING}.md` (in `/home/sehaxe/potok-m`). Line numbers were
re-pinned against the current `firmware/src` at commit `1d25e7f`; the audit's
older line numbers describe pre-fix state and were not copied blindly.
