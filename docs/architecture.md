# Talon Architecture

Last updated: 2026-10-18

## Current-state snapshot (FlightLab Phase 4)

Talon is a single-vehicle UAV GNC stack (C++17 header-only kernel via
pybind11, Python mission/SIL/analysis glue) plus a TypeScript flight tool.
Phase 1 added deterministic obstacle avoidance and the schema 1.1 obstacle
contract; Phase 2 added the interactive 3D obstacle scene; Phase 3 turned the
replay app into Talon FlightLab, a single entry point carrying the replay,
telemetry, metrics, and video sections; Phase 4 added a live run that
executes the same C++ kernel in the browser as WebAssembly, driven by a
TypeScript port of the Python glue, behind a fifth Run section.

### Repository layout

The web app owns the repository root. Hosting publishes the built `dist/`
from that root as a GitHub Pages *project* site, served from a subpath
(`https://<owner>.github.io/Talon/`), so Vite's `base` is `/Talon/` and every
emitted asset URL carries that prefix (D21).

    /                      web app (Vite + React 19 + R3F)
      index.html  package.json  vite.config.ts  tsconfig*.json
      src/                 views, components, utils, store, data, hooks
      src/live/            TS glue for the WASM run (see below)
      public/data/         exported telemetry JSON + live configs (committed)
      public/wasm/         compiled kernel glue + wasm (committed)
      public/videos/       recorded MP4s (committed)
      scripts/             copy_data.sh
    wasm/                  Emscripten build of the kernel (exports.cpp)
    fixtures/              Python-generated parity fixtures (committed)
    core/                  header-only C++17 kernel + Google Tests
      include/talon/       dynamics, wind, controller, ekf, types, constants
      tests/
    bindings/pytalon.cpp   pybind11 module talon._core
    python/                Python package, simulations, and tests
      talon/               package (CMake drops _core*.so here)
      sims/                scenarios, run_all.py, export_live_configs.py
      tests/
    docs/                  conventions, architecture, decisions, reviews, tests
    CMakeLists.txt  pyproject.toml  uv.lock

Import path note: `python/` is on the pytest and driver `sys.path`, so
`import talon` and `import sims` resolve without an install step; the
editable install additionally maps the `talon` package to `python/talon`.

### Data flow (one 200 Hz SIL step)

    dynamics (truth, C++ RK4) -> motors -> wind -> sensors -> EKF -> state
    machine -> phase setpoints -> NLGL guidance -> cascaded PD -> motors

### Obstacle avoidance pipeline

    CIRCUIT_WAYPOINTS (5-leg rectangle)
        |
        v
    plan_avoidance_mission (talon.avoidance, pure Python)
        |  inserts detour waypoints around SphericalObstacle keep-out zones
        v
    augmented waypoint list (8 Waypoints)
        |
        v
    SILConfig.waypoints -> SILRunner closed loop (EKF + NLGL + cascaded PD)
        |
        v
    SILResult (truth_position, phase_timeline, obstacles, ...)
        |
        v
    export_sil_result_json (schema 1.1, obstacles serialized as top-level key)
        |
        v
    src/ web app (R3F scene renders ObstacleField from the JSON)

### Module ownership

| Layer | C++17 kernel (`core/include/talon/`) | Python (`python/talon/`) |
|-------|--------------------------------------|------------------------|
| Dynamics / RK4 | `dynamics.hpp` (`propagate`) | re-export only |
| Wind | `wind.hpp` (`DrydenWind`) | re-export only |
| Controller | `controller.hpp` (cascaded PD) | `controller.py` binding |
| Estimator | `ekf.hpp` (15-state MEKF) | `ekf.py` binding |
| Motors | constants in `constants.hpp` | `motors.py` (lag + mix) |
| Sensors | constants | `sensors.py` (truth-side IMU/GPS) |
| Guidance / mission | - | `guidance.py` (NLGL), `mission.py` |
| Obstacle avoidance | - | `avoidance.py` (detour planner, clearance) |
| State machine / FDIR | - | `state_machine.py` |
| SIL + MAVLink | - | `mavlink_sil.py`, `mavlink_codec.py` |
| Battery | - | `battery.py` |
| Plots / report / video | - | `plotting.py`, `export_json.py`, `animation.py` |

### Schema 1.1 JSON contract

The exporter (`talon.export_json`) writes a versioned JSON document with
`meta.schema_version = "1.1"`. The new `obstacles` top-level key is a
list of `{id, center_ned, radius_m}` objects. Old 1.0 documents (without
the `obstacles` key) remain valid; the validator checks the block only
when present.

### Web app rendering layers (Phase 2)

The R3F scene (`src/components/scene/FlightScene.tsx`) composes
static and per-frame layers over the loaded `FlightData`, plus a single
postprocessing pass:

| Layer | File | Driven by |
|-------|------|-----------|
| Ground grid | `GroundPlane.tsx` | static |
| Truth trail | `TrajectoryRibbon.tsx` | per-sample, phase-colored |
| Drone marker + camera | `DroneMarker.tsx` | truth position/velocity |
| EKF covariance | `CovarianceEllipsoid.tsx` | `position_var` |
| Wind field | `WindParticles.tsx` | `wind_ned` |
| Waypoints | `WaypointMarkers.tsx` | `waypoints_ned` |
| Keep-out zones | `ObstacleField.tsx` | `obstacles` + live clearance |
| Bloom | `FlightScene.tsx` | `@react-three/postprocessing` |

Clearance math lives in one pure module, `src/utils/clearance.ts`
(`surfaceClearance`, `minSurfaceClearance`, `clearanceBand`), shared by
`ObstacleField` (scene tint), the HUD (live readout), and the legend (flight
minimum). The 2.07 m flight-minimum the legend prints is the same quantity
the Python CI asserts, so the web readout and the validation report cannot
drift.

### Web app FlightLab shell (Phase 3)

The app is one entry point with four sections, switched by
`useFlightStore.activeView` and rendered by `ViewTabs` in the header:

| Section | File | Backed by |
|---------|------|-----------|
| Replay | `src/views/ReplayView.tsx` | R3F scene + HUD + legend |
| Telemetry | `src/views/TelemetryView.tsx` | `deriveTelemetry(data)` time-series |
| Metrics | `src/views/MetricsView.tsx` | `summarizeRun(data)` + `validation.json` |
| Videos | `src/views/VideosView.tsx` | `public/videos/*.mp4` (`RECORDINGS`) |

The replay and telemetry sections share the playback clock, so the transport
bar and phase scrubber stay pinned to the bottom for both; metrics and videos
drop them. The left rail always carries the scenario list and adds the
scene-layer toggles only for the time-based sections.

Two data paths feed the screens, both derived rather than hand-maintained:

    recorded samples (public/data/*.json)
        -> deriveTelemetry  -> Telemetry charts (altitude, speed, EKF error,
                               wind, thrust, 3-sigma, clearance)
        -> summarizeRun     -> Metrics run summary (duration, EKF RMSE,
                               maxima, distance, min clearance)

    python/sims/run_all.py -> plots/json/validation.json -> Metrics gates table

Chart helpers live in one pure module, `src/utils/chart.ts` (min/max bucket
decimation, domain, scales, path builder), and the series derivation in
`src/utils/telemetry.ts`. Both are unit-tested in the node vitest
environment; no charting dependency is added.

### Recording pipeline

    python/sims/run_all.py -> docs/videos/*.mp4 (five recordings)
        -> scripts/copy_data.sh
        -> public/videos/*.mp4 (committed, served statically)

### Live in-browser run (Phase 4)

The Run section executes the autopilot in the browser. One 200 Hz step:

    configs.json (Python-exported)  -> LiveSim (src/live/sim.ts)
      sensors: truth-side IMU 200 Hz, GPS 10 Hz (src/live/sensors.ts)
      estimator: 15-state MEKF (WASM kernel)
      derived flags: convergence / divergence (mirrors mavlink_sil)
      state machine + FDIR (src/live/stateMachine.ts)
      guidance: NLGL (src/live/guidance.ts) + mission (src/live/mission.ts)
      controller: cascaded PD, 100 Hz ZOH (WASM kernel)
      motors: X mixer + first-order lag (src/live/motors.ts)
      plant: RK4 + Dryden wind (WASM kernel), ground clamp
      history at 100 Hz -> FlightData (schema 1.1)

The module boundary is deliberate (decisions.md D17): the numerically heavy
and safety-critical pieces (dynamics, wind, controller, MEKF) run in the
compiled kernel (`wasm/exports.cpp`, embind `Kernel` class, scalar API so a
step costs a handful of cheap calls). Everything with no native counterpart
is TypeScript. The WASM glue is loaded once via a script tag
(`src/live/kernel.ts`) and reached through a `KernelHandle` interface, so
tests inject a node-loaded kernel and the UI only ever depends on the
interface.

Config flow and fixtures, all generated by
`python/sims/export_live_configs.py` from the Python scenario definitions:

    scenario_obstacle_field (waypoints + obstacles + detour planner)
      -> public/data/live/configs.json    (hover, waypoints, obstacle_field, wind)
      -> fixtures/live_fixtures.json      (guidance / mission / motors / kernel)
      -> fixtures/sil_reference.json      (optional, full Python SIL trajectories)

The live run emits the same `FlightData` document a recorded scenario does,
so the scene, chart, and metrics layers are unchanged and a completed run
becomes a rail entry ("LIVE RUN") alongside the recorded scenarios. The
store keys it under the string `live` (`ScenarioKey`), which is never
fetched.

Deliberate scope limits (decisions.md D18, D20): the noise stream is a
seeded mulberry32 generator, so a live run is statistically equivalent to,
not bit-identical with, the recorder; fault injection (GPS outage,
emergency, low battery) is recorded-only and the battery state of charge is
held at 1.0; the obstacle waypoints are pre-planned offline. When the WASM
module is unavailable, Run disables itself with a visible message and every
other section still works (decisions.md D19).

## Phase roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| 2 | Interactive obstacle scene (R3F + bloom + labels) | Complete |
| 3 | Unified entry point (videos + scene + metrics) | Complete |
| 4 | In-browser WASM run | Complete |
