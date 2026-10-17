from __future__ import annotations

from talon._core import (
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
)

# Alias for readability at motor-modelling call sites.
MOTOR_LAG = MOTOR_TIME_CONST

__all__ = [
    "AIR_DENSITY",
    "ARM_LENGTH",
    "DRAG_COEFF",
    "DT",
    "DT_CONTROL",
    "DT_GPS",
    "DT_IMU",
    "GRAVITY",
    "IXX",
    "IYY",
    "IZZ",
    "M_MAX",
    "MASS",
    "MOTOR_KF",
    "MOTOR_KM",
    "MOTOR_LAG",
    "MOTOR_TIME_CONST",
    "VEHICLE_AREA",
]
