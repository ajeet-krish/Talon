# Talon FlightLab: UAV Autopilot Simulator

Talon FlightLab is a drone autopilot simulator you can fly and inspect in
the browser. It pairs a validated guidance, navigation, and control (GNC)
core with a web tool that presents the results: a replayable 3D flight,
live telemetry channels, the automated validation report, the recorded
flight videos, and a live run that executes the autopilot itself in the
browser.

The core is a complete autopilot for a 1.5 kg Crazyflie-class quadrotor: a
15-state multiplicative error-state Kalman filter, NLGL line-of-sight
guidance, a cascaded PD controller, an 8-state flight-phase state machine
with FDIR, and a UDP MAVLink software-in-the-loop (SIL) runner, all closed
loop against a C++17 rigid-body truth plant. The computational kernel
(dynamics, Dryden wind, controller, EKF) is header-only C++17, exposed
through pybind11 for the recorded runs and through Emscripten/embind for
the web app; the mission, guidance, estimation glue, SIL, and analysis
tooling are Python, with the browser-side subset ported to TypeScript.
Talon is the single-agent companion to SwarmGNC: where SwarmGNC studies
multi-agent formation flight, Talon proves one vehicle done honestly,
estimator in the loop, with every acceptance number checked by an
automated report.

## Run the app

The web app is the repository root (Vite + React).

```bash
npm install
npm run dev      # dev server
npm run build    # typecheck + production build into dist/
npm test         # vitest unit tests
```

The app is served at the Pages root once deployed
(`https://<user>.github.io/<repo>/`).

The Run section needs the WebAssembly kernel. Build it once with
`./wasm/build.sh` (Emscripten); without it every other section still works
and Run disables itself with a clear message.

## Sections

One entry point, five sections (the replay and telemetry sections share the
playback clock; the left rail always carries the scenario list):

- **Replay**: a full-bleed 3D scene. Scrub a recorded flight and read the autopilot working: EKF state estimation with a 3-sigma covariance ellipsoid, the flight-phase state machine (phase-colored trail and phase-segmented timeline), NLGL guidance, and detour avoidance around spherical keep-out zones with a live clearance readout.
- **Telemetry**: the recorded channels as time-series (altitude, ground speed, EKF error, wind, thrust, 3-sigma, obstacle clearance).
- **Metrics**: a run summary derived from the active scenario plus the validation driver's own acceptance table.
- **Videos**: the recorded Matplotlib flight exports, with an "open in replay" jump for the runs that have a scenario.
- **Run**: fly a scenario in the browser. The C++ kernel compiled to WebAssembly is driven by the TypeScript mission, guidance, motor, sensor, and state-machine glue at 200 Hz plant / 100 Hz control / 10 Hz GPS; the finished run appears in the rail and drops into Replay, Telemetry, and Metrics like a recorded flight.

## Highlights

- **15-state MEKF**: position, velocity, attitude (multiplicative error), gyro bias, accel bias; chi2-gated 10 Hz GPS updates, covariance-based convergence and divergence detection.
- **Cascaded PD autopilot** at 100 Hz with zero-order hold: altitude / velocity outer loop, attitude / rate inner loop, saturating mixer-aware commands, 200 Hz RK4 truth with first-order motor lag.
- **NLGL guidance** (Park, Deyst and How 2004, ArduPilot clipped form): look-ahead line-of-sight steering with ground-velocity Nu2, behind-A recovery, and per-leg cross-track diagnostics.
- **Flight-phase state machine**: IDLE, ARM, TAKEOFF, CRUISE, WAYPOINT, RTL, LAND, EMERGENCY with guarded transitions, an event audit log, and automatic critical-fault response to EKF divergence.
- **UDP MAVLink SIL**: real MAVLink v2 frames (pymavlink codec) for telemetry (HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED, GPS_RAW_INT, BATTERY_STATUS) and COMMAND_LONG events on loopback ports 14550/14551, plus fault injection (GPS outage, EKF corruption, battery drain).
- **Deterministic obstacle avoidance**: a pure-Python leg-detour planner inserts waypoints around spherical keep-out zones, and the schema 1.1 telemetry export carries the zones so the web scene can render them.
- **7 closed-loop scenarios** with printed acceptance metrics, plus a validation driver that runs everything, generates seven publication plots and five flight videos, exports schema 1.1 telemetry JSON, and writes an automated validation report plus a machine-readable copy.
- **In-browser WASM run**: the same C++ kernel (dynamics, Dryden wind, cascaded PD, 15-state MEKF) built to WebAssembly with Emscripten + embind and driven by a faithful TypeScript port of the Python mission, guidance, motor, sensor, and state-machine glue, so hover, waypoints, obstacle field, and wind fly live in the browser with no server round trip.
- **Test suites**: 186 pytest, 34 Google Test, and 80 vitest, all passing.

## Architecture

Data flow of one 200 Hz SIL step (sensors and estimator at their own rates, controller at 100 Hz):

```mermaid
flowchart LR
    W["Dryden wind<br/>(C++)"] --> T["Truth dynamics<br/>RK4, 200 Hz (C++)"]
    T --> S["Sensors<br/>IMU 200 Hz, GPS 10 Hz (Python)"]
    S --> E["EKF<br/>15-state MEKF (C++)"]
    E --> SM["Flight phase machine<br/>+ FDIR (Python)"]
    SM --> G["NLGL guidance<br/>(Python)"]
    G --> C["Cascaded PD<br/>100 Hz (C++)"]
    C --> M["Motor lag +<br/>X mixer (Python)"]
    M --> T
    E -. "MAVLink telemetry" .- O["UDP SIL runner / operator"]
    O -. "COMMAND_LONG" .- SM
```

Module ownership (single source of truth per layer):

| Layer | C++17 kernel (`core/include/talon/`) | Python (`python/talon/`) |
|-------|--------------------------------------|--------------------------|
| Dynamics / RK4 | `dynamics.hpp` (`propagate`) | re-export only |
| Wind | `wind.hpp` (`DrydenWind`) | re-export only |
| Controller | `controller.hpp` (cascaded PD) | `controller.py` binding |
| Estimator | `ekf.hpp` (15-state MEKF) | `ekf.py` binding |
| Motors | constants in `constants.hpp` | `motors.py` (lag + mix) |
| Sensors | constants | `sensors.py` (truth-side IMU/GPS) |
| Guidance / mission | - | `guidance.py` (NLGL), `mission.py` |
| Obstacle avoidance | - | `avoidance.py` (detour planner) |
| State machine / FDIR | - | `state_machine.py` |
| SIL + MAVLink | - | `mavlink_sil.py`, `mavlink_codec.py` |
| Battery | - | `battery.py` |
| Plots / report / video | - | `plotting.py`, `export_json.py`, `animation.py` |

The web app consumes the exported JSON and the validation report for the
recorded sections. The Run section adds a second, browser-side path: the
same C++ kernel built to WebAssembly (`wasm/exports.cpp`, an embind
`Kernel` class) plus a TypeScript port of the Python glue under `src/live/`
(mission, NLGL guidance, motor mixer and lag, truth-side sensors, the
flight-phase machine, and a line-for-line mirror of the SIL loop in
`sim.ts`). The live-run configs are exported from the Python scenario
definitions by `python/sims/export_live_configs.py`, so the browser can
never drift from the recorder. The estimator is not reimplemented: the
15-state MEKF runs in WASM, since a second implementation of the most
safety-critical math would be a second source of truth.

## Build and test

The native extension `talon._core` is built by CMake (not by the Python build backend). From the repository root:

```bash
# Build the C++ kernel, GTest binary, and the pybind11 module
# (drops _core*.so into python/talon/). Run under `uv run` or with the
# virtualenv active so CMake picks the same interpreter as the tests:
uv run cmake -B build -DCMAKE_BUILD_TYPE=Release
uv run cmake --build build -j

./build/talon_tests                  # 34 C++ unit tests (Google Test)

uv sync                              # Python environment (.venv)
uv run pytest python/tests/ -q       # 186 Python tests

uv run python python/sims/run_all.py # 7 scenarios + plots + videos + reports

# WebAssembly kernel for the in-browser live run (needs Emscripten)
./wasm/build.sh                      # writes public/wasm/talon.js + .wasm
```

Individual scenarios also run standalone, for example
`uv run python python/sims/scenario_waypoints.py`. The validation driver writes `plots/*.png`, `plots/validation_report.md`, and `plots/json/validation.json` (the plots/ directory is gitignored; regenerate
on demand).

To refresh the web app's telemetry and recordings after a driver run:

```bash
scripts/copy_data.sh   # copies plots/json/*.json and docs/videos/*.mp4 into public/
                       # and regenerates public/data/live/configs.json
```

## Scenarios

| Scenario | Purpose | Key metric |
|----------|---------|------------|
| `scenario_hover` | Cascaded PD closed-loop hover with motor lag, calm air | steady-state drift < 0.05 m (achieved < 0.001 m) |
| `scenario_waypoints` | NLGL 40x30 m rectangle at 10 m under Dryden wind, full EKF-in-the-loop stack | mean cross-track 0.65 m, EKF RMSE 0.10 m |
| `scenario_rtl` | Low-battery fault injection, return-to-launch, gentle touchdown | 0.40 m/s touchdown, 0.07 m from home |
| `scenario_wind` | 60 s hover hold in 4-6 m/s airflow versus a calm baseline, SIL state machine | windy RMS 0.09 m inside the 0.25 m bound |
| `scenario_gps_outage` | 4 s mid-mission GPS blackout: dead-reckon, reconverge, no EMERGENCY | 0.69 m error growth, 0.01 s reconvergence |
| `scenario_emergency` | EKF position corruption at hover: FDIR detection and controlled descent | detection 0.00 s, soft settled landing |
| `scenario_obstacle_field` | 40x30 m circuit at 10 m through three spherical keep-out zones, detour planner + full EKF loop | 8/8 waypoints at 2.07 m minimum 3-D clearance |

## Validation summary

Headline numbers from `uv run python python/sims/run_all.py` (the full 25-row table lands in `plots/validation_report.md` on every run, and the same rows land in `plots/json/validation.json` for the web report):

| Metric | Target | Achieved |
|--------|--------|----------|
| Hover steady-state drift | < 0.05 m | 0.005 m |
| Position step rise time / overshoot | < 2.5 s / < 10 % | 1.29 s / 8.2 % |
| EKF position RMSE (waypoint circuit) | < 2.0 m | 0.102 m |
| Gyro bias estimate (max axis) | < 0.01 rad/s | 0.00177 rad/s |
| NLGL mean / max cross-track (after 15 s) | < 1.0 m / < 3.0 m | 0.650 m / 1.967 m |
| Waypoints reached | 5 of 5 | 5 of 5 |
| RTL touchdown: horizontal / vertical speed | < 3 m / < 0.5 m/s | 0.070 m / 0.398 m/s |
| Wind hover RMS (vs baseline-scaled bound) | < max(0.25, ...) m | 0.094 m |
| GPS outage error growth / reconvergence | < 5 m / < 5 s | 0.686 m / 0.01 s |
| Obstacle field min 3-D surface clearance | > 0 m | 2.072 m |
| Emergency detection / final altitude | < 1 s / < 0.5 m | 0.000 s / settled |
| State machine phases covered (SIL union) | 8 of 8 | 8 of 8 |
| Test suites | 139+ pytest, 34 GTest | 186 pytest, 34 GTest |

Two example plots (generated by the validation driver into the gitignored `plots/` directory; regenerate with `uv run python python/sims/run_all.py`):

![3D truth trajectory of the GPS-outage circuit: climb, two-waypoint
out-and-back, RTL descent to home](plots/trajectory_3d.png)

*3D NED truth trajectory with waypoint and home markers: takeoff climb, two 40 m legs at 10 m, then the RTL climb-over-home and descent. The cross-track samples for both legs come from the NLGL diagnostics.*

![Flight phase Gantt of the emergency scenario: idle, arm, takeoff,
cruise, emergency, land, idle with accepted event markers](plots/state_machine.png)

*Flight-phase Gantt from the emergency scenario: the corrupted EKF estimate trips `critical_fault` at 20 s, the machine owns a controlled descent through EMERGENCY and LAND, and the settled vehicle returns to IDLE. Dotted lines are accepted guarded events; down-ticks are rejected attempts in the audit log.*

Note: `plots/` is gitignored, so the two images embedded above do not render on GitHub. Generate them locally with `uv run python python/sims/run_all.py` and open `plots/*.png` on disk.

## Frames and conventions

NED world frame, aerospace body frame, Hamilton scalar-first quaternions, thrust along -body-z, gravity [0, 0, +g] in NED. The full single source of truth for frames, signs, units, and the motor mixer is [docs/conventions.md](docs/conventions.md); every C++ header and Python module is checked against it.

## Guidance note

Talon's guidance law is Nonlinear Linear guidance (NLGL) in the form of
Park, Deyst and How (2004), implemented in the scalar ArduPilot
AP_L1_Control style: look-ahead distance L1 = zeta * T * V / pi, a
clipped cross-track capture angle Nu1, and a course error Nu2 measured
from the EKF ground-velocity vector (mandatory for wind immunity: the
crab angle cancels exactly in Nu2). The commanded course is
chi_d = chi_q + Nu with Nu clipped to +/-90 deg, the ArduPilot form that
keeps L1 globally convergent; the unclipped Nu1 + Nu2 sum has a
wrong-way equilibrium at corners. Altitude rides a separate P/PD channel
on the active waypoint's NED z. Waypoint sequencing, acceptance radii,
and the first-leg start-position rule live in `talon.mission`; the law
itself is a pure function with per-call diagnostics (`e_ct`, `nu1`,
`nu2`, `l1`, `chi_d`).

## Talon versus SwarmGNC

| Aspect | Talon | SwarmGNC |
|--------|-------|----------|
| Scope | Single vehicle, full onboard GNC | Multi-agent swarm, coordination research |
| Navigation | 15-state MEKF (GPS + IMU, bias estimation) | Truth-state (formation study isolates control) |
| Control | Cascaded PD with motor lag and mixer saturation | LQR consensus controllers |
| Guidance | NLGL line-of-sight path following | DCM-based formation keeping |
| Fault handling | FDIR state machine, emergency landing | Not in scope |
| Presentation | FlightLab web simulator (replay, telemetry, metrics) | Static 3D viewer |

Talon and SwarmGNC are complementary portfolio projects: Talon is the
flagship flight-dynamics project proving estimation and control honesty
on one airframe; SwarmGNC remains the active project for multi-agent
work.

## Portfolio narrative

Talon is the flagship of the flight-dynamics portfolio: a single-vehicle
autopilot where the estimator is in the loop, the wind is modeled, the
faults are injected, and every headline number is produced by an
automated report rather than by hand. FlightLab is the presentation layer:
the same validated runs, replayable and inspectable in the browser.
SwarmGNC remains the active project for multi-agent formation flight;
Talon is what makes the single-agent layer underneath that research
trustworthy.

## References

1. Beard, R. W. and McLain, T. W. *Small Unmanned Aircraft: Theory and
   Practice*. Princeton University Press, 2012. (Cascaded autopilot
   design, path following, Dryden wind model.)
2. Trawny, N. and Roumeliotis, S. I. "Indirect Kalman Filter for 3D
   Attitude Estimation." University of Minnesota, GRASP Laboratory,
   2005. (MEKF formulation used by `ekf.hpp`.)
3. Solà, J. "Quaternion kinematics for the error-state Kalman filter."
   arXiv:1711.02508, 2017. (Multiplicative error-state conventions.)
4. Park, S., Deyst, J. and How, J. P. "Performance and Lyapunov
   stability of a nonlinear path-following guidance law." *AIAA Journal
   of Guidance, Control, and Dynamics*, 2007 (with the 2004 conference
   precursor). (NLGL guidance law.)
5. Nelson, D. B. *Flight Stability and Automatic Control*. McGraw-Hill,
   1998. (Aerospace frame and rigid-body context.)
6. Groves, P. D. *Principles of GNSS, Inertial, and Multisensor
   Integrated Navigation Systems*. Artech House, 2013. (GPS measurement
   modeling and sensor fusion practice.)
7. MIL-F-8785C. *Military Specification: Flying Qualities of Piloted
   Airplanes*. (Turbulence severity context for the Dryden spectrum.)
8. Quan, Q. *Introduction to Multicopter Design and Control*. Springer,
   2017. (Quadrotor modeling, mixer, and motor dynamics.)
9. Mellinger, D. and Kumar, V. "Minimum snap trajectory generation and
   control for quadrotors." *IEEE ICRA*, 2011. (Reference for
   trajectory-tracking performance expectations on small quadrotors.)
10. Lee, T., Leoky, M. and McClamroch, N. H. "Geometric tracking
    control of a quadrotor UAV on SE(3)." *IEEE CDC*, 2010.
    (Geometric-control contrast to the cascaded PD used here.)

## Layout

```
Talon/
├── index.html               # web app entry (Vite)
├── package.json             # web deps and scripts (dev/build/test)
├── vite.config.ts           # base '/', '@' alias to src/
├── src/                     # web app: React 19 + react-three-fiber
│   ├── views/               #   Replay, Telemetry, Metrics, Videos, Run
│   ├── components/scene/    #   R3F layers + FlightScene
│   ├── components/charts/   #   SVG time-series chart
│   ├── components/ui/       #   rail, tabs, HUD, legend, timeline
│   ├── live/                #   TS glue for the WASM run (guidance, mission,
│   │                        #   motors, sensors, state machine, SIL loop)
│   ├── utils/               #   interpolation, clearance, chart, telemetry (pure)
│   ├── store/               #   scenario cache, clock, toggles, active section
│   └── data/                #   scenario, recording, and live-run metadata
├── wasm/                    # Emscripten build of the kernel (embind exports)
├── public/data/             # exported telemetry JSON + live configs (committed)
├── public/wasm/             # compiled kernel glue + wasm (committed)
├── public/videos/           # recorded MP4s (committed)
├── fixtures/                # Python-generated parity fixtures (committed)
├── scripts/copy_data.sh     # stage JSON + recordings into public/
├── core/                    # header-only C++17 kernel + Google Tests
│   ├── include/talon/       #   dynamics, wind, controller, ekf, types, constants
│   └── tests/               #   34 Google Tests
├── bindings/pytalon.cpp     # pybind11 module talon._core
├── python/                  # Python package, simulations, and tests
│   ├── talon/               #   package (_core*.so built here by CMake)
│   ├── sims/                #   7 scenarios, run_all.py, export_live_configs.py
│   └── tests/               #   186 pytest tests
├── docs/                    # conventions, architecture, decisions, research,
│                            #   reviews, tests, videos, portfolio
├── CMakeLists.txt           # kernel + GTest + pybind11 build
├── pyproject.toml           # uv-managed Python deps
└── .github/workflows/       # ci.yml (native + web), deploy.yml (Pages)
```

## License

Talon is a portfolio project. It is shared for evaluation and learning;
no open-source license is granted by default. Reach out if you want to
reuse part of it.
