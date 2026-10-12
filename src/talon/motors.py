from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from talon.constants import ARM_LENGTH, DT, MOTOR_KF, MOTOR_KM, MOTOR_TIME_CONST

# X-configuration motor mixing.
#
#   [T ]   [  1   1   1   1 ] [f1]
#   [Mx] = [ -l  -l   l   l ] [f2]
#   [My]   [ -l   l  -l   l ] [f3]
#   [Mz]   [ -c   c   c  -c ] [f4]
#
# with l = arm_length / sqrt(2) and c = MOTOR_KM / MOTOR_KF (the yaw moment
# per unit thrust implied by the motor coefficients).
_MIX_L = ARM_LENGTH / math.sqrt(2.0)
_MIX_C = MOTOR_KM / MOTOR_KF

MIX_MATRIX: NDArray[np.float64] = np.array(
    [
        [1.0, 1.0, 1.0, 1.0],
        [-_MIX_L, -_MIX_L, _MIX_L, _MIX_L],
        [-_MIX_L, _MIX_L, -_MIX_L, _MIX_L],
        [-_MIX_C, _MIX_C, _MIX_C, -_MIX_C],
    ],
    dtype=np.float64,
)
_MIX_INVERSE: NDArray[np.float64] = np.linalg.inv(MIX_MATRIX)


def motor_mixing(thrust: float, moment: ArrayLike) -> NDArray[np.float64]:
    """Solve the inverse mix: (collective thrust, body moment) -> 4 motor forces.

    May return negative forces for aggressive moments; MotorModel clamps those
    to zero (motors cannot produce negative thrust).
    """
    moment_arr = np.asarray(moment, dtype=np.float64)
    if moment_arr.shape != (3,):
        raise ValueError(f"moment must have shape (3,), got {moment_arr.shape}")
    wrench = np.array([thrust, moment_arr[0], moment_arr[1], moment_arr[2]])
    return _MIX_INVERSE @ wrench


def forward_mix(motor_forces: ArrayLike) -> tuple[float, NDArray[np.float64]]:
    """Forward mix: 4 motor forces -> (collective thrust, body moment)."""
    forces = np.asarray(motor_forces, dtype=np.float64)
    if forces.shape != (4,):
        raise ValueError(f"motor_forces must have shape (4,), got {forces.shape}")
    wrench = MIX_MATRIX @ forces
    return float(wrench[0]), wrench[1:4]


class MotorModel:
    """Four first-order lag motor responses.

    Each motor force relaxes toward its commanded value with the exact
    exponential update over dt:
        f += (f_cmd - f) * (1 - exp(-dt / tau))
    and is clamped at zero (motors cannot produce negative thrust).
    """

    def __init__(self, time_const: float = MOTOR_TIME_CONST) -> None:
        if time_const <= 0.0:
            raise ValueError(f"time_const must be positive, got {time_const}")
        self._time_const = time_const
        self._forces = np.zeros(4, dtype=np.float64)

    def reset(self) -> None:
        self._forces[:] = 0.0

    def step(
        self,
        thrust: float,
        moment: ArrayLike,
        dt: float = DT,
    ) -> NDArray[np.float64]:
        """Advance one step; returns the actual (lagged) motor forces."""
        if dt < 0.0:
            raise ValueError(f"dt must be non-negative, got {dt}")
        commanded = motor_mixing(thrust, moment)
        alpha = 1.0 - math.exp(-dt / self._time_const)
        self._forces += (commanded - self._forces) * alpha
        np.clip(self._forces, 0.0, None, out=self._forces)
        return self._forces.copy()

    @property
    def forces(self) -> NDArray[np.float64]:
        """Current actual motor forces (N)."""
        return self._forces.copy()


__all__ = ["MIX_MATRIX", "MotorModel", "forward_mix", "motor_mixing"]
