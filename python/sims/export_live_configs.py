"""Live-run config and parity-fixture exporter (FlightLab Phase 4).

Writes two committed artifacts from the Python reference implementation so
the in-browser WASM run has a single source of truth:

1. ``public/data/live/configs.json``: the LiveConfig for every runnable
   scenario (hover, waypoints, obstacle_field, wind), built from the same
   scenario constants and the same avoidance planner the recorded runs use.
   The web Run view loads this; it never hardcodes scenario knobs.

2. ``fixtures/live_fixtures.json``: deterministic input/output pairs for the
   TypeScript glue port (NLGL guidance, the waypoint manager, the motor
   mixer/lag) and for the WASM kernel (propagate, EKF predict/update,
   cascaded controller). The vitest suites assert the TS/WASM side matches
   these within tolerance.

Optionally (``--with-sil``) it also writes ``fixtures/sil_reference.json``:
a full Python SIL reference trajectory for two live configs, used by the
browser-run integration test to check the live trajectory tracks the Python
one (statistically, since the noise streams differ by design).

Run from the repository root:

    uv run python python/sims/export_live_configs.py
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1]))  # python/

from talon import (  # noqa: E402
    DT,
    DT_CONTROL,
    CascadedController,
    ControlGains,
    EKFConfig,
    Ekf,
    NLGLGuidance,
    Quaternion,
    SILConfig,
    SILRunner,
    State,
    VehicleParams,
    Waypoint,
    WaypointMission,
    plan_avoidance_mission,
    propagate,
)
from talon.motors import MotorModel, forward_mix, motor_mixing  # noqa: E402
from sims import scenario_obstacle_field as sof  # noqa: E402

REPO_ROOT = _HERE.parents[2]
CONFIG_OUT = REPO_ROOT / "public" / "data" / "live" / "configs.json"
FIXTURE_OUT = REPO_ROOT / "fixtures" / "live_fixtures.json"
SIL_REF_OUT = REPO_ROOT / "fixtures" / "sil_reference.json"

SEED = 20261012
GPS_POSITION_NOISE_M = 0.3
GPS_POSITION_NOISE_VERTICAL_M = 0.5
GPS_VELOCITY_NOISE_MPS = 0.05
CRUISE_HOLD_S = 2.0


def _round(value: float) -> float:
    return round(float(value), 12)


def _vec(values) -> list[float]:
    return [_round(v) for v in np.asarray(values, dtype=np.float64).reshape(-1)]


def _wp(position, speed: float, heading: float, radius: float) -> dict:
    return {
        "position": _vec(position),
        "speed": _round(speed),
        "heading": _round(heading),
        "radius": _round(radius),
    }


def _obstacle(obstacle) -> dict:
    return {
        "id": obstacle.id,
        "center_ned": _vec(obstacle.center_ned),
        "radius_m": _round(obstacle.radius_m),
    }


def _base(id_: str, label: str, blurb: str, duration_s: float) -> dict:
    return {
        "id": id_,
        "label": label,
        "blurb": blurb,
        "duration_s": duration_s,
        "takeoff_altitude": 10.0,
        "start_position_ned": [0.0, 0.0, 0.0],
        "waypoints": [],
        "obstacles": [],
        "wind_sigma": [0.0, 0.0, 0.0],
        "wind_steady_ned": [0.0, 0.0, 0.0],
        "seed": SEED,
        "gps_position_noise": GPS_POSITION_NOISE_M,
        "gps_position_noise_vertical": GPS_POSITION_NOISE_VERTICAL_M,
        "gps_velocity_noise": GPS_VELOCITY_NOISE_MPS,
        "cruise_hold_s": CRUISE_HOLD_S,
        "auto_arm": True,
    }


def build_configs() -> list[dict]:
    """The runnable live scenarios, from the Python scenario definitions."""
    circuit = [
        _wp(w.position, w.speed, w.heading, w.radius) for w in sof.CIRCUIT_WAYPOINTS
    ]
    augmented = plan_avoidance_mission(
        sof.CIRCUIT_WAYPOINTS, sof.OBSTACLES, sof.CLEARANCE_MARGIN_M
    )
    augmented_cfg = [
        _wp(w.position, w.speed, w.heading, w.radius) for w in augmented
    ]
    obstacles = [_obstacle(o) for o in sof.OBSTACLES]

    hover = _base(
        "hover",
        "Hover",
        "Climb to 10 m and hold station for 60 s in calm air.",
        60.0,
    )
    hover["wind_sigma"] = [0.0, 0.0, 0.0]

    waypoints = _base(
        "waypoints",
        "Waypoint circuit",
        "40 m x 30 m rectangle at 10 m, 5 m/s legs, light turbulence.",
        120.0,
    )
    waypoints["waypoints"] = circuit
    waypoints["wind_sigma"] = [2.0, 2.0, 1.0]

    obstacle = _base(
        "obstacle_field",
        "Obstacle field",
        "The waypoint circuit flown around three keep-out spheres via the "
        "planned detours.",
        70.0,
    )
    obstacle["waypoints"] = augmented_cfg
    obstacle["obstacles"] = obstacles

    wind = _base(
        "wind",
        "Hold in wind",
        "Hover at 10 m against 2 m/s north wind with strong Dryden turbulence.",
        60.0,
    )
    wind["wind_sigma"] = [3.0, 3.0, 1.5]
    wind["wind_steady_ned"] = [2.0, 0.0, 0.0]

    return [hover, waypoints, obstacle, wind]


def build_fixtures() -> dict:
    """Deterministic Python reference values for the TS/WASM parity tests."""
    guidance = NLGLGuidance()
    guidance_cases = []
    for name, position, velocity, a, b in [
        ("mid_leg", [10.0, 1.5, -10.0], [5.0, 0.0, 0.0], [0.0, 0.0, -10.0], [40.0, 0.0, -10.0]),
        ("zero_length_leg", [0.0, 0.0, -10.0], [0.0, 0.0, 0.0], [0.0, 0.0, -10.0], [0.0, 0.0, -10.0]),
        ("behind_a_recovery", [-40.0, 0.0, -10.0], [0.0, 0.0, 0.0], [0.0, 0.0, -10.0], [40.0, 0.0, -10.0]),
        ("low_speed", [10.0, 0.2, -10.0], [0.1, 0.05, 0.0], [0.0, 0.0, -10.0], [40.0, 0.0, -10.0]),
        ("crab_wind", [20.0, 2.0, -10.0], [4.7, 1.1, 0.0], [0.0, 0.0, -10.0], [40.0, 0.0, -10.0]),
    ]:
        v_des, d = guidance.compute_leg(position, velocity, a, b)
        guidance_cases.append(
            {
                "name": name,
                "position": _vec(position),
                "velocity": _vec(velocity),
                "wp_a": _vec(a),
                "wp_b": _vec(b),
                "v_des": _vec(v_des),
                "e_ct": _round(d.e_ct),
                "nu1": _round(d.nu1),
                "nu2": _round(d.nu2),
                "nu": _round(d.nu),
                "l1": _round(d.l1),
                "chi_d": _round(d.chi_d),
                "a_lat": _round(d.a_lat),
                "behind_a_recovery": bool(d.behind_a_recovery),
            }
        )

    mission_wps = [
        Waypoint(np.array([0.0, 0.0, -10.0]), speed=5.0, radius=2.0),
        Waypoint(np.array([10.0, 0.0, -10.0]), speed=5.0, radius=2.0),
        Waypoint(np.array([10.0, 10.0, -10.0]), speed=5.0, radius=2.0),
    ]
    mission = WaypointMission(mission_wps)
    mission_records = []
    for position in [
        [0.0, 0.0, -10.0],
        [5.0, 0.0, -10.0],
        [10.0, 0.0, -10.0],
        [10.0, 5.0, -10.0],
        [10.0, 10.0, -10.0],
    ]:
        p = np.asarray(position, dtype=np.float64)
        complete = mission.update(p)
        mission_records.append(
            {
                "position": _vec(p),
                "active_index": int(mission.active_index),
                "complete": bool(complete),
                "distance_remaining": _round(mission.distance_remaining(p)),
            }
        )

    mixing = []
    for thrust, moment in [
        (0.0, [0.0, 0.0, 0.0]),
        (14.715, [0.0, 0.0, 0.0]),
        (20.0, [0.05, -0.03, 0.01]),
    ]:
        forces = motor_mixing(thrust, np.asarray(moment, dtype=np.float64))
        mixing.append(
            {
                "thrust": _round(thrust),
                "moment": _vec(moment),
                "forces": _vec(forces),
            }
        )
    fwd = []
    for forces in [[3.67875, 3.67875, 3.67875, 3.67875], [5.0, 4.0, 4.5, 3.5]]:
        thrust, moment = forward_mix(np.asarray(forces, dtype=np.float64))
        fwd.append(
            {
                "forces": _vec(forces),
                "thrust": _round(thrust),
                "moment": _vec(moment),
            }
        )
    lag_model = MotorModel()
    lag = []
    for k in range(5):
        forces = lag_model.step(14.715, np.zeros(3), DT)
        lag.append({"step": k, "forces": _vec(forces)})

    # Kernel: propagate.
    state0 = State()
    state0.position = np.array([1.0, 2.0, -3.0])
    state0.velocity = np.array([0.5, -0.2, 0.1])
    state0.orientation = Quaternion()
    state0.angular_rate = np.zeros(3)
    k_thrust, k_moment, k_wind = 15.0, np.array([0.01, -0.02, 0.003]), np.array([0.3, -0.4, 0.05])
    final = propagate(state0, k_thrust, k_moment, k_wind, DT, VehicleParams())
    propagate_fixture = {
        "initial_position": _vec(state0.position),
        "initial_velocity": _vec(state0.velocity),
        "thrust": _round(k_thrust),
        "moment": _vec(k_moment),
        "wind": _vec(k_wind),
        "dt": _round(DT),
        "final_position": _vec(final.position),
        "final_velocity": _vec(final.velocity),
        "final_quat": [final.orientation.w, final.orientation.x, final.orientation.y, final.orientation.z],
    }

    # Kernel: EKF predict + update.
    ekf = Ekf(EKFConfig())
    ekf.initialize_level(np.zeros(3), np.zeros(3))
    accel = np.array([0.02, -0.01, -9.83])
    gyro = np.array([0.001, -0.002, 0.0015])
    gps_pos = np.array([0.1, -0.2, 0.05])
    gps_vel = np.array([0.01, -0.02, 0.03])
    ekf.predict(accel, gyro, DT)
    accepted = bool(ekf.update(gps_pos, gps_vel))
    nom = ekf.nominal()
    ekf_fixture = {
        "init_position": [0.0, 0.0, 0.0],
        "init_velocity": [0.0, 0.0, 0.0],
        "init_yaw": 0.0,
        "accel": _vec(accel),
        "gyro": _vec(gyro),
        "dt": _round(DT),
        "gps_position": _vec(gps_pos),
        "gps_velocity": _vec(gps_vel),
        "accepted": accepted,
        "position": _vec(nom.position),
        "velocity": _vec(nom.velocity),
    }

    # Kernel: cascaded controller (level EKF state, zero gyro).
    ctrl_ekf = Ekf(EKFConfig())
    ctrl_pos = np.array([0.0, 0.0, -5.0])
    ctrl_vel = np.array([0.4, -0.1, 0.2])
    ctrl_ekf.initialize_level(ctrl_pos, ctrl_vel)
    ctrl_nom = ctrl_ekf.nominal()
    est = State()
    est.position = ctrl_pos
    est.velocity = ctrl_vel
    est.orientation = ctrl_nom.attitude
    est.angular_rate = ctrl_ekf.estimated_body_rates(np.zeros(3))
    controller = CascadedController(ControlGains.defaults(), VehicleParams())
    ref = np.array([0.0, 0.0, -10.0])
    v_cmd = np.array([2.0, 0.0, -0.5])
    yaw = 0.3
    cmd = controller.step(est, ref, v_cmd, yaw, DT_CONTROL)
    controller_fixture = {
        "init_position": _vec(ctrl_pos),
        "init_velocity": _vec(ctrl_vel),
        "init_yaw": 0.0,
        "reference": _vec(ref),
        "v_cmd": _vec(v_cmd),
        "yaw": _round(yaw),
        "dt": _round(DT_CONTROL),
        "gyro": [0.0, 0.0, 0.0],
        "thrust": _round(max(0.0, cmd.collective_thrust)),
        "moment": _vec(cmd.moment),
    }

    return {
        "guidance": guidance_cases,
        "mission": {"waypoints": mission_wps and [_wp(w.position, w.speed, w.heading, w.radius) for w in mission_wps], "records": mission_records},
        "motors": {"mixing": mixing, "forward": fwd, "lag": lag, "lag_dt": _round(DT), "lag_thrust": 14.715},
        "kernel": {
            "propagate": propagate_fixture,
            "ekf": ekf_fixture,
            "controller": controller_fixture,
        },
    }


def _downsample(history: np.ndarray, every: int) -> list[list[float]]:
    return [_vec(row) for row in np.asarray(history)[::every]]


def build_sil_reference() -> dict:
    """Python SIL reference trajectories for two live configs (10 Hz samples)."""
    out: dict[str, dict] = {}
    for cfg in build_configs():
        if cfg["id"] not in ("hover", "obstacle_field"):
            continue
        sil_config = SILConfig(
            waypoints=[
                Waypoint(np.asarray(w["position"], dtype=np.float64), speed=w["speed"],
                         heading=w["heading"], radius=w["radius"])
                for w in cfg["waypoints"]
            ],
            takeoff_altitude=cfg["takeoff_altitude"],
            start_position_ned=np.asarray(cfg["start_position_ned"], dtype=np.float64),
            wind_sigma=tuple(cfg["wind_sigma"]),
            wind_steady_ned=tuple(cfg["wind_steady_ned"]),
            gps_position_noise=cfg["gps_position_noise"],
            gps_position_noise_vertical=cfg["gps_position_noise_vertical"],
            gps_velocity_noise=cfg["gps_velocity_noise"],
            cruise_hold_s=cfg["cruise_hold_s"],
            enable_udp=False,
            seed=cfg["seed"],
            obstacles=[],
        )
        runner = SILRunner(sil_config)
        runner.request_event("arm")
        runner.request_event("takeoff")
        result = runner.run(cfg["duration_s"])
        every = max(1, round(0.1 / 0.01))  # 10 Hz
        phases = [[_round(t), phase.value] for t, phase in result.phase_timeline]
        out[cfg["id"]] = {
            "sample_dt": 0.1,
            "truth_position": _downsample(result.truth_position, every),
            "phase_timeline": phases,
            "metrics": {k: float(v) for k, v in result.metrics.items() if isinstance(v, (int, float))},
        }
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-sil",
        action="store_true",
        help="also regenerate the Python SIL reference trajectories (slow)",
    )
    args = parser.parse_args()

    configs = build_configs()
    CONFIG_OUT.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_OUT.write_text(
        json.dumps(
            {
                "generated_utc": datetime.now(timezone.utc)
                .isoformat(timespec="seconds")
                .replace("+00:00", "Z"),
                "configs": configs,
            },
            indent=2,
        )
        + "\n"
    )
    print(f"wrote {CONFIG_OUT.relative_to(REPO_ROOT)} ({len(configs)} configs)")

    fixtures = build_fixtures()
    FIXTURE_OUT.parent.mkdir(parents=True, exist_ok=True)
    FIXTURE_OUT.write_text(json.dumps(fixtures, indent=2) + "\n")
    print(f"wrote {FIXTURE_OUT.relative_to(REPO_ROOT)}")

    if args.with_sil:
        reference = build_sil_reference()
        SIL_REF_OUT.parent.mkdir(parents=True, exist_ok=True)
        SIL_REF_OUT.write_text(json.dumps(reference, indent=2) + "\n")
        print(f"wrote {SIL_REF_OUT.relative_to(REPO_ROOT)} ({len(reference)} runs)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
