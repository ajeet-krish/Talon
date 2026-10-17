# Talon Architecture

Last updated: 2026-10-17

## Current-state snapshot (FlightLab Phase 1)

Talon is a single-vehicle UAV GNC stack (C++17 header-only kernel via
pybind11, Python mission/SIL/analysis glue) plus a TypeScript 3D flight
replay web app. Phase 1 adds deterministic obstacle avoidance to the
simulation pipeline and exports the obstacle field in the schema 1.1
JSON contract.

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
| 2 | Interactive obstacle scene (R3F + bloom + labels) | Pending |
| 3 | Unified entry point (videos + scene + metrics) | Pending |
| 4 | In-browser WASM run | Pending |
