# Talon Architecture

Last updated: 2026-10-18

## Current-state snapshot (FlightLab Phase 5b, in progress)

Talon is a single-vehicle UAV GNC stack whose autopilot is one Rust library,
`talon-core`, plus a Python analysis harness and a TypeScript flight tool.
Phase 1 added deterministic obstacle avoidance and the schema 1.1 obstacle
contract; Phase 2 added the interactive 3D obstacle scene; Phase 3 turned the
replay app into Talon FlightLab, a single entry point carrying the replay,
telemetry, metrics, and video sections; Phase 4 added a live run that executed
the then-C++ kernel in the browser as WebAssembly, driven by a TypeScript port
of the Python glue. Phase 5 moves the whole autopilot into Rust (D22, D23) and
puts the browser run on it: the wasm-bindgen session in D24 replaces the
TypeScript loop, and the schema 1.1 document is now assembled in Rust (D25) so
the scene, charts, and metrics are unchanged.

The Python and C++ stacks are still present while the port is finished; the
recorded telemetry, the validation report, and the five videos are frozen as
produced by the Python stack and are not regenerated.

### Repository layout

The web app owns the repository root. Hosting publishes the built `dist/`
from that root as a GitHub Pages *project* site, served from a subpath
(`https://<owner>.github.io/Talon/`), so Vite's `base` is `/Talon/` and every
emitted asset URL carries that prefix (D21).

    /                      web app (Vite + React 19 + R3F) + Cargo workspace
      index.html  package.json  vite.config.ts  tsconfig*.json
      Cargo.toml           Cargo workspace (members: core, wasm)
      src/                 views, components, utils, store, data, hooks
      src/components/live/ the live ground-station scene and instrument rail
      src/live/            engine loader, recorder, pacer, display geometry
      public/data/         exported telemetry JSON + live configs (committed)
      public/wasm-rust/    talon-core via wasm-bindgen, web target (committed)
      public/wasm/         the retiring Emscripten kernel (still committed)
      public/videos/       recorded MP4s (committed)
      scripts/             copy_data.sh, smoke_live_wasm.mjs
    core/                  talon-core Rust GNC library (the autopilot)
      src/                 math, constants, rng, types, dynamics, wind,
                           controller, ekf, guidance, mission, state_machine,
                           motors, battery, sensors, sim, scenario, export
      tests/               GTest suites for the retiring C++ kernel
      include/talon/       the retiring C++17 header-only kernel
    wasm/                  wasm-bindgen crate (plus the retiring Emscripten build)
      src/lib.rs           the Session the browser drives
    fixtures/              Python-generated parity fixtures (committed)
    bindings/pytalon.cpp   pybind11 module talon._core (to become a PyO3 crate)
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
| Run | `src/views/RunView.tsx` | live GCS: native session + recorder + pacer |

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

### Live in-browser run (Phase 5b)

The Run section executes the autopilot in the browser. The core is the Rust
library compiled to wasm32; the page only steps it:

    configs.json (Python-exported)  -> LiveSim (src/live/sim.ts)
      -> talon_core Session (public/wasm-rust/, wasm-bindgen, D24)
           sensors: truth-side IMU 200 Hz, GPS 10 Hz
           estimator: 15-state MEKF
           derived flags: convergence / divergence (mirrors mavlink_sil)
           state machine + FDIR
           guidance: NLGL + waypoint mission
           controller: cascaded PD, 100 Hz ZOH
           motors: X mixer + first-order lag
           plant: RK4 + Dryden wind, ground clamp
           history at 100 Hz
      -> document_json  -> FlightData (schema 1.1, assembled in Rust, D25)
      -> summary_json   -> mission outcome + fix accounting for the run tiles

The module boundary is now the whole core: physics, estimation, guidance,
mission, FDIR, and the SIL loop itself are native, and the frame loop makes
about one call per frame instead of a scalar read per control tick (D22, D24).
`src/live/engine.ts` loads the wasm-pack web output at runtime from the Vite
base URL and declares the exported surface by hand, so a signature drift fails
the typecheck. `LiveSim` is the seam: construct a session from the scenario
JSON, step it, drain what it recorded, and turn the finished session into the
document the scene and charts already consume.

The session streams its recorded history as flat `f64` blocks, one per
control-rate sample (`SAMPLE_STRIDE` doubles). `src/live/stream.ts` decodes them
into a recorder (`LiveBuffer`) that keeps the scene-space trail, six derived
scalar series, and the phase in effect per sample, so a frame reads what it needs
instead of recomputing anything. The decoder mirrors the Rust field offsets by
hand, and the run view asserts `sample_stride()` from the loaded build against
the local layout, so a stale wasm build fails with a message instead of showing
mislabelled numbers.

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

Deliberate scope limits: the core's own noise generator replaces numpy's, so a
live run is statistically equivalent to, not bit-identical with, the recorder
(D18, and the recorded files stay frozen). Fault injection (GPS
outage, emergency, low battery) is still recorded-only (D20). When the WASM
module is unavailable, Run
disables itself with a visible message and every other section still works
(D19).

### Live ground station and pacing (Phase 6)

The live run is revealed at a pace you can watch, and the Run section is laid
out as a ground station: a scene with the planned mission and the flight as it
happens, next to an instrument rail (D26).

    RunView (src/views/RunView.tsx)
      frame loop  -> Pacer.tick(dtMs)  -> target sim time (1x / 4x / 16x / MAX)
                  -> session.advance(round((target - time) / DT))
                  -> session.samples_from(cursor)  -> LiveBuffer.append
                  -> buffer.clock = session.time
      completion  -> session.document_json + summary_json -> setLiveRun('live')

Pacing is a display concern and lives entirely in `src/live/pacing.ts`. The core
has no idea it is being paced, so a watched run and a batch run of the same
scenario are the same flight; only the reveal differs. The step count comes from
the difference between the pacer's target and the session's own clock, so
per-frame rounding never accumulates into a drift, and a hidden tab's giant frame
delta is clamped rather than teleported.

Layers, all reading the same recorder:

| Layer | File | Driven by |
|-------|------|-----------|
| Ground grid | `components/scene/GroundPlane.tsx` | static, shared with the replay |
| Planned route | `components/live/LiveField.tsx` | `config.waypoints` |
| Waypoints | `components/live/LiveField.tsx` | `config.waypoints` |
| Keep-out zones | `components/live/LiveField.tsx` | `config.obstacles` + live clearance |
| Trail | `components/live/LiveTrail.tsx` | recorded samples, phase-colored |
| Vehicle | `components/live/LiveAirframe.tsx` | recorded attitude, body axes |
| Bloom | `components/live/LiveScene.tsx` | `@react-three/postprocessing` |
| Numbers, health, captions | `components/live/TelemetryRail.tsx` | `readReadout` at ~10 Hz |
| Strip charts | `components/live/LiveSpark.tsx` | recorded series |

The scene animates from the recorder's `clock` field, which the frame loop
writes, so a frame that only advances the clock costs no React work. The rail
re-renders at about 10 Hz: text has no need to keep up with 100 Hz, and the
render budget stays with the scene. `src/components/live/captions.ts` supplies
per-phase teaching text, and it names the technique rather than describing the
motion.

Two facts about presentation worth recording. The camera is framed from the
mission (start, waypoints, keep-out zones, takeoff column), not from the
trajectory, because a live trajectory is a single point on the first frame and
would drag the camera out to the launch pad. And the vehicle is drawn in body
axes with the truth attitude applied, so the nose visibly drops in cruise and
leans in a gust, with the roll and pitch in the rail agreeing with the picture.

`scripts/smoke_live_wasm.mjs` (`npm run smoke:wasm`) flies every live config
through the built wasm the way the page does and asserts the outcomes, since no
browser runs in CI: sample accounting, capacity headroom, schema 1.1 on a
stepped document, per-scenario acceptance (a hold keeps altitude, a mission
lands), and that a paced 1x run tracks the wall clock without drift.

## Phase roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| 2 | Interactive obstacle scene (R3F + bloom + labels) | Complete |
| 3 | Unified entry point (videos + scene + metrics) | Complete |
| 4 | In-browser WASM run | Complete |
| 5a | Rust `talon-core` library + native tests | Complete |
| 5b | WASM + PyO3 bindings, generated type mirrors, retire the C++ stack | In Progress |
| 6 | GCS shell with live telemetry and teaching captions | Complete |
| 7 | Altitude legibility in the scene | In Progress |
| 8 | Mission authoring (waypoints, wind, faults) | Not started |
| 9 | Reactive avoidance in Rust, with validation | Not started |
