# Talon Architecture

Last updated: 2026-10-17

## Current-state snapshot (FlightLab Phase 3)

Talon is a single-vehicle UAV GNC stack (C++17 header-only kernel via
pybind11, Python mission/SIL/analysis glue) plus a TypeScript flight tool.
Phase 1 added deterministic obstacle avoidance and the schema 1.1 obstacle
contract; Phase 2 added the interactive 3D obstacle scene; Phase 3 turns the
replay app into Talon FlightLab, a single entry point carrying four
sections: the 3D replay, the telemetry charts, the metrics report, and the
recorded flight videos.

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
    talon-replay (R3F scene renders ObstacleField from the JSON)

### Module ownership

| Layer | C++17 kernel (`cpp/include/talon/`) | Python (`src/talon/`) |
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

### talon-replay rendering layers (Phase 2)

The R3F scene (`talon-replay/src/components/scene/FlightScene.tsx`) composes
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

Clearance math lives in one pure module, `talon-replay/src/utils/clearance.ts`
(`surfaceClearance`, `minSurfaceClearance`, `clearanceBand`), shared by
`ObstacleField` (scene tint), the HUD (live readout), and the legend (flight
minimum). The 2.07 m flight-minimum the legend prints is the same quantity
the Python CI asserts, so the web readout and the validation report cannot
drift.

### talon-replay FlightLab shell (Phase 3)

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

    sims/run_all.py -> plots/json/validation.json -> Metrics gates table

Chart helpers live in one pure module, `src/utils/chart.ts` (min/max bucket
decimation, domain, scales, path builder), and the series derivation in
`src/utils/telemetry.ts`. Both are unit-tested in the node vitest
environment; no charting dependency is added.

### Recording pipeline

    sims/run_all.py -> docs/videos/*.mp4 (five recordings)
        -> scripts/copy_data.sh
        -> talon-replay/public/videos/*.mp4 (committed, served statically)

### talon-replay data-source abstraction (Phase 4-ready)

The replay app consumes schema 1.1 JSON via `useFlightStore.loadScenario`.
The renderer layers (`FlightScene`, `TrajectoryRibbon`, `DroneMarker`,
`CovarianceEllipsoid`, `WindParticles`, `WaypointMarkers`) read from a
`FlightData` object with whole-history arrays and binary-search
interpolation. For Phase 4 (in-browser WASM run), the same components
will accept a live frame stream via a `FlightDataSource` interface:

    protocol FlightDataSource:
        duration_s -> float
        bounds -> tuple[Vec3, Vec3]
        sample_at(t: float, field: str) -> Vec3 | float

The recorded implementation wraps the whole-history arrays; the live
implementation wraps a ring buffer fed by the WASM frame stream. The
scene components are agnostic to which source is active.

## Phase roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Obstacle scenario + schema 1.1 (offline detour avoidance) | Complete |
| 2 | Interactive obstacle scene (R3F + bloom + labels) | Complete |
| 3 | Unified entry point (videos + scene + metrics) | Complete |
| 4 | In-browser WASM run | Pending |
