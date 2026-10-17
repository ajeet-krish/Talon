from __future__ import annotations

import enum
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


class FlightPhase(enum.Enum):
    """High-level autopilot mission phase (drives Phase 2+ logic)."""

    IDLE = "idle"
    ARM = "arm"
    TAKEOFF = "takeoff"
    CRUISE = "cruise"
    WAYPOINT = "waypoint"
    RTL = "rtl"
    LAND = "land"
    EMERGENCY = "emergency"


@dataclass(slots=True)
class IMUMeasurement:
    """One IMU sample.

    accel is the specific force in body frame (m/s^2): in hover it reads
    [0, 0, -g] under the Talon NED convention (see docs/conventions.md).
    gyro is the body angular rate (rad/s).
    """

    timestamp: float
    accel: NDArray[np.float64]
    gyro: NDArray[np.float64]

    def __post_init__(self) -> None:
        self.accel = np.asarray(self.accel, dtype=np.float64)
        self.gyro = np.asarray(self.gyro, dtype=np.float64)
        if self.accel.shape != (3,):
            raise ValueError(f"accel must have shape (3,), got {self.accel.shape}")
        if self.gyro.shape != (3,):
            raise ValueError(f"gyro must have shape (3,), got {self.gyro.shape}")


@dataclass(slots=True)
class GPSMeasurement:
    """One GPS fix: lat/lon in degrees, alt in m, NED velocity in m/s."""

    timestamp: float
    lat: float
    lon: float
    alt: float
    vn: float
    ve: float
    vd: float


@dataclass(slots=True)
class Waypoint:
    """Navigation target in NED (m).

    radius is the horizontal acceptance radius (m): a mission advances to
    the next waypoint when the vehicle is within radius of ``position``
    (horizontal plane only; altitude is handled by the guidance channel).
    """

    position: NDArray[np.float64]
    speed: float = 5.0
    heading: float = 0.0
    radius: float = 2.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64)
        if self.position.shape != (3,):
            raise ValueError(
                f"position must have shape (3,), got {self.position.shape}"
            )


@dataclass(slots=True)
class SphericalObstacle:
    """Spherical keep-out zone in NED (m).

    center_ned is the 3-D sphere centre in the NED frame; radius_m is the
    keep-out radius. A position is inside the obstacle when its 3-D Euclidean
    distance to centre is less than radius_m.
    """

    id: str
    center_ned: NDArray[np.float64]
    radius_m: float

    def __post_init__(self) -> None:
        self.center_ned = np.asarray(self.center_ned, dtype=np.float64)
        if self.center_ned.shape != (3,):
            raise ValueError(
                f"center_ned must have shape (3,), got {self.center_ned.shape}"
            )
        if self.radius_m <= 0.0:
            raise ValueError(
                f"radius_m must be positive, got {self.radius_m}"
            )


__all__ = [
    "FlightPhase",
    "GPSMeasurement",
    "IMUMeasurement",
    "SphericalObstacle",
    "Waypoint",
]
