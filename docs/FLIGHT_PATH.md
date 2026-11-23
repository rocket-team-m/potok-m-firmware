# Flight path wiring

FWCRIT found that `control_pid`, `attitude` (Mahony), `sensor_voting`,
`deployment`, `separation`, `self_test` and `search_beacon` had no call sites in
the shipping binary, that `HwBoard::apply_command` discarded every command, and
that `motor_armed` / `poll_phase_event` were silent stubs pinning the FSM in
BURN. This document records the seam that gives those modules one production
caller each without claiming hardware the repository does not have.

## The seam

`FlightPath::tick` in `firmware/src/lib.rs` is called once per 500 Hz tick from
the loop in `firmware/src/main.rs`. It makes one bounded pass over every module:

| Capability | Module | Input gate | Output gate |
|---|---|---|---|
| `SensorVoting` | `sensor_voting::VotingChannel` | `imu_readings`, `height_readings` | — |
| `Mahony` | `attitude::Mahony` | voted IMU + tick period | — |
| `Pid` | `control_pid::Pid` | `stabilization` (setpoint/measurement/gains) | `apply_stabilization` |
| `Deployment` | `deployment::DeploymentController` | `deployment` (arm + sample) | `apply_springs` |
| `Separation` | `separation::Sequencer` | `separation` (plan, arm, phase pair, sensor pair) | `apply_separation` |
| `SelfTest` | `self_test::SelfTest` | `self_test_input` | report |
| `Radio` | `search_beacon::SearchBeacon` | `signal_inputs` | `apply_radio` |

`FlightBoard` is the typed capability gate. Every method is optional and
**defaults to absent**. A board supplies only what it actually owns; no pin
number, bus, peripheral, radio channel or actuator mapping appears in the
seam. The outputs are typed commands, not hardware writes.

## Absence is counted, never silent

`CapabilityLedger` keeps `used`, `unavailable` and `last_reason` per capability.
`Unavailable` has four reasons: `NoDriver`, `NoSample`, `Refused`,
`OutputFailed`. If a module's input is absent, the module does not run and the
ledger records why. If a module refuses a sample (degenerate data, wrong phase,
incomplete arm, latched fault) that refusal is counted too. If a board claims
an output but rejects the write, the failure is counted as `OutputFailed`.

`Cell<u32>` backs the counters so the `&self` stubs on `HwBoard` (`motor_armed`,
`apply_command`) are countable without a mutex or a global. The shipping loop
emits one summary per second through the existing typed `LogGate`/`Cadence`
policy; suppression itself is counted by that gate.

`HwBoard` implements `FlightBoard` without overriding a single method. All
seven module capabilities are therefore absent on the target today, and the
summary reports that explicitly. Its stubs now count `Actuation`,
`PhaseSense` and `MotorArmedSense` as `NoDriver` instead of discarding the
command or pretending a disarmed, quiescent world. Keep `false` / `None` as
the *safe* fallback values — the count is what makes the stub visible.

## Safety invariants

- `step()` in `control.rs` is unchanged and remains the only gate in front of
  `Board::apply_command`. `FlightPath::tick` runs before it and never calls
  `apply_command`.
- PID output goes to the separate `apply_stabilization` capability, not around
  the interlock. A board may only wire that output after its own gate placement
  is validated. With no such board today, no fin command is emitted.
- All existing refusals still produce `Command::ZERO`: the three-miss latch,
  the phase/motor/health/fault gates, the Mach shield and the deadline
  monitor. A zero PID input produces `Command::ZERO`.
- Deployment keeps its one-shot, dual-arm and phase-window rules; separation
  keeps its independent-arm, sensor-pair and abort latch. Both are still
  decision logic — no squib or spring hardware is claimed.
- Self-test advances exactly one bounded slice per tick and never spins.

## What is still open

No sensor, PWM, pyro or radio driver exists, so the target reports every
capability unavailable. The host tests supply typed inputs and prove each
module actually runs when a board provides its capability. Wiring real drivers
and deciding where the PID output joins the actuation path remain schematic
and HIL decisions, not software claims made here.
