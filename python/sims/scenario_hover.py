"""Phase 2 hover scenario: 60 s hover at 10 m altitude.

Architecture (documented choices):
- C++ dynamics (talon.propagate, RK4 at DT = 0.005 s / 200 Hz).
- C++ cascaded PD controller (talon.CascadedController) running at
  DT_CONTROL = 0.01 s (100 Hz) with zero-order hold between control ticks,
  matching docs/conventions.md. Every second dynamics step triggers a new
  controller step (2 RK4 steps per control tick).
- Phase 1 Python MotorModel in the loop: the commanded (T, M) goes through
  inverse mixing, first-order lag (tau = 0.02 s), clamping, and forward
  mixing before reaching the plant. Motors start at rest, so the vehicle
  briefly free-falls before thrust spins up, as in real life.
- No wind (Phase 6 adds the Dryden model to scenarios); calm-air hover keeps
  the steady-state check deterministic.

Acceptance: after the 40 s mark, position error stays below 0.05 m in each
axis; final position error is printed and asserted.
"""

from __future__ import annotations

import sys

import numpy as np

from talon import (
    DT,
    DT_CONTROL,
    MOTOR_TIME_CONST,
    CascadedController,
    ControlGains,
    State,
    VehicleParams,
    propagate,
)
from talon.motors import MotorModel, forward_mix

SIM_DURATION_S = 60.0
ALTITUDE_M = 10.0  # climb to 10 m (NED z = -10)
ERROR_TOLERANCE_M = 0.05
CHECK_WINDOW_S = 20.0  # steady-state window at the end of the run


def main() -> int:
    params = VehicleParams()
    gains = ControlGains.defaults()
    controller = CascadedController(gains, params)
    motors = MotorModel()

    state = State()
    pos_desired = np.array([0.0, 0.0, -ALTITUDE_M])
    vel_desired = np.zeros(3)
    yaw_desired = 0.0

    steps_per_control = max(1, round(DT_CONTROL / DT))
    n_steps = round(SIM_DURATION_S / DT)
    check_start = round((SIM_DURATION_S - CHECK_WINDOW_S) / DT)

    command_thrust = params.mass * params.gravity
    command_moment = np.zeros(3)

    max_deviation = 0.0
    for i in range(n_steps):
        if i % steps_per_control == 0:
            cmd = controller.step(
                state, pos_desired, vel_desired, yaw_desired, DT_CONTROL
            )
            command_thrust = cmd.collective_thrust
            command_moment = np.asarray(cmd.moment)

        forces = motors.step(command_thrust, command_moment, dt=DT)
        thrust_actual, moment_actual = forward_mix(forces)
        state = propagate(
            state, thrust_actual, moment_actual, np.zeros(3), DT, params
        )

        if i >= check_start:
            error = pos_desired - np.asarray(state.position)
            max_deviation = max(max_deviation, float(np.max(np.abs(error))))

    final_error = pos_desired - np.asarray(state.position)
    final_error_norm = float(np.linalg.norm(final_error))
    print(f"t = {SIM_DURATION_S:.1f} s hover at {ALTITUDE_M:.1f} m (NED)")
    print(
        f"control rate: {1.0 / DT_CONTROL:.0f} Hz (ZOH), "
        f"dynamics: {1.0 / DT:.0f} Hz, motor lag: {MOTOR_TIME_CONST * 1e3:.0f} ms"
    )
    print(f"final position error [n, e, d] (m): "
          f"[{final_error[0]:+.6f}, {final_error[1]:+.6f}, {final_error[2]:+.6f}]")
    print(f"final position error norm (m): {final_error_norm:.6f}")
    print(f"max deviation over last {CHECK_WINDOW_S:.0f} s (m): {max_deviation:.6f}")
    print(f"final velocity norm (m/s): "
          f"{np.linalg.norm(np.asarray(state.velocity)):.6f}")

    ok = True
    for axis, name in enumerate("ned"):
        if abs(final_error[axis]) >= ERROR_TOLERANCE_M:
            print(f"FAIL: final {name} error "
                  f"{final_error[axis]:+.6f} m >= {ERROR_TOLERANCE_M} m")
            ok = False
    if max_deviation >= ERROR_TOLERANCE_M:
        print(f"FAIL: window deviation {max_deviation:.6f} m "
              f">= {ERROR_TOLERANCE_M} m")
        ok = False
    if not ok:
        return 1
    print(f"PASS: steady-state error within {ERROR_TOLERANCE_M} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
