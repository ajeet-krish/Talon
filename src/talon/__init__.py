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

from .controller import CascadedController, ControlCommand, ControlGains
from .ekf import EKFConfig, EKFNominalState, Ekf
from .sensors import (
    GPSConfig,
    IMUConfig,
    SensorSuite,
    gps_to_ned,
    gps_velocity_ned,
    specific_force_from_velocity,
)

__all__ = [
    "AIR_DENSITY",
    "ARM_LENGTH",
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
    "GPSConfig",
    "GRAVITY",
    "IXX",
    "IYY",
    "IZZ",
    "IMUConfig",
    "M_MAX",
    "MASS",
    "MOTOR_KF",
    "MOTOR_KM",
    "MOTOR_TIME_CONST",
    "Quaternion",
    "SensorSuite",
    "State",
    "VEHICLE_AREA",
    "VehicleParams",
    "gps_to_ned",
    "gps_velocity_ned",
    "hover_thrust",
    "propagate",
    "rotation_matrix",
    "specific_force_from_velocity",
]

__version__ = "0.1.0"
