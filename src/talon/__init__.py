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

__all__ = [
    "AIR_DENSITY",
    "ARM_LENGTH",
    "DRAG_COEFF",
    "DT",
    "DT_CONTROL",
    "DT_GPS",
    "DT_IMU",
    "DrydenWind",
    "GRAVITY",
    "IXX",
    "IYY",
    "IZZ",
    "M_MAX",
    "MASS",
    "MOTOR_KF",
    "MOTOR_KM",
    "MOTOR_TIME_CONST",
    "Quaternion",
    "State",
    "VEHICLE_AREA",
    "VehicleParams",
    "hover_thrust",
    "propagate",
    "rotation_matrix",
]

__version__ = "0.1.0"
