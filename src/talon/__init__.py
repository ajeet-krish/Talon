from __future__ import annotations

try:
    from ._core import (
        AIR_DENSITY,
        ARM_LENGTH,
        DRAG_COEFF,
        DT,
        DT_CONTROL,
        DT_GPS,
        DT_IMU,
        GRAVITY,
        IXX,
        IYY,
        IZZ,
        M_MAX,
        MASS,
        MOTOR_KF,
        MOTOR_KM,
        MOTOR_TIME_CONST,
        VEHICLE_AREA,
        DrydenWind,
        Quaternion,
        State,
        VehicleParams,
        hover_thrust,
        propagate,
        rotation_matrix,
    )
except ImportError as exc:  # pragma: no cover - exercised only on unbuilt checkouts
    raise RuntimeError(
        "Talon native extension 'talon._core' is not built. Build it from the "
        "repository root with: cmake -B build && cmake --build build -j "
        "(run under `uv run` or with the virtualenv active so CMake picks the "
        "same Python interpreter)."
    ) from exc

from .battery import BatteryConfig, BatteryModel, BatteryState
from .controller import CascadedController, ControlCommand, ControlGains
from .ekf import EKFConfig, EKFNominalState, Ekf
from .guidance import GuidanceDiagnostics, NLGLConfig, NLGLGuidance
from .mavlink_codec import MavlinkCodec
from .mavlink_sil import SILConfig, SILResult, SILRunner
from .mission import WaypointMission
from .sensors import (
    GPSConfig,
    IMUConfig,
    SensorSuite,
    gps_to_ned,
    gps_velocity_ned,
    specific_force_from_velocity,
)
from .state_machine import FlightPhaseMachine, PhaseTransition, Telemetry
from .types import FlightPhase, Waypoint

__all__ = [
    "AIR_DENSITY",
    "ARM_LENGTH",
    "BatteryConfig",
    "BatteryModel",
    "BatteryState",
    "DRAG_COEFF",
    "DT",
    "DT_CONTROL",
    "DT_GPS",
    "DT_IMU",
    "EKFConfig",
    "EKFNominalState",
    "CascadedController",
    "ControlCommand",
    "ControlGains",
    "DrydenWind",
    "Ekf",
    "FlightPhase",
    "FlightPhaseMachine",
    "GPSConfig",
    "GRAVITY",
    "GuidanceDiagnostics",
    "IXX",
    "IYY",
    "IZZ",
    "IMUConfig",
    "M_MAX",
    "MASS",
    "MOTOR_KF",
    "MOTOR_KM",
    "MOTOR_TIME_CONST",
    "MavlinkCodec",
    "NLGLConfig",
    "NLGLGuidance",
    "PhaseTransition",
    "Quaternion",
    "SILConfig",
    "SILResult",
    "SILRunner",
    "SensorSuite",
    "State",
    "Telemetry",
    "VEHICLE_AREA",
    "VehicleParams",
    "Waypoint",
    "WaypointMission",
    "gps_to_ned",
    "gps_velocity_ned",
    "hover_thrust",
    "propagate",
    "rotation_matrix",
    "specific_force_from_velocity",
]

__version__ = "0.1.0"
