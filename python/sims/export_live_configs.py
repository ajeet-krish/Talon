"""Live-run config exporter (FlightLab Phase 4).

Writes one committed artifact from the Python reference implementation so
the in-browser WASM run has a single source of truth:

``public/data/live/configs.json``: the LiveConfig for every runnable
scenario (hover, waypoints, obstacle_field, avoidance, wind), built from
the same scenario constants and the same avoidance planner the recorded
runs use. The web Run view loads this; it never hardcodes scenario knobs.

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

from talon import plan_avoidance_mission  # noqa: E402
from sims import scenario_obstacle_field as sof  # noqa: E402

REPO_ROOT = _HERE.parents[2]
CONFIG_OUT = REPO_ROOT / "public" / "data" / "live" / "configs.json"

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

    # The same circuit and the same zones, but with no detours planned: the
    # autopilot flies the straight legs and deviates around each zone itself,
    # in the loop, through the reactive law in talon-core.
    avoidance = _base(
        "avoidance",
        "Reactive avoidance",
        "The plain circuit straight through the same three keep-out spheres: "
        "the autopilot sees each zone, bypasses it, and rejoins the leg.",
        70.0,
    )
    avoidance["waypoints"] = circuit
    avoidance["obstacles"] = obstacles
    # The margin is stated rather than inherited, because it is the number the
    # smoke check and the operator read the achieved clearance against.
    avoidance["avoidance"] = {
        "enabled": True,
        "clearance_margin_m": _round(sof.CLEARANCE_MARGIN_M),
    }

    wind = _base(
        "wind",
        "Hold in wind",
        "Hover at 10 m against 2 m/s north wind with strong Dryden turbulence.",
        60.0,
    )
    wind["wind_sigma"] = [3.0, 3.0, 1.5]
    wind["wind_steady_ned"] = [2.0, 0.0, 0.0]

    return [hover, waypoints, obstacle, avoidance, wind]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
