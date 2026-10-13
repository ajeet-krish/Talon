"""Phase 5 emergency scenario: EKF divergence at hover, controlled descent.

Full EKF-in-the-loop stack on the SILRunner: arm, takeoff, CRUISE hold at
10 m, then at t = 20 s inject_ekf_divergence("corrupt_bias"): a one-shot
+25 m offset on the nominal NORTH position. The innovation gate rejects
every subsequent fix (chi2 ~ 270 vs the gate of 25), the EKF-vs-truth
error exceeds the 20 m divergence threshold on the same control step,
and the state machine fires critical_fault -> EMERGENCY within one step
(well under the 1 s acceptance).

EMERGENCY descends at 1.0 m/s with a zero lateral velocity command
(through the velocity reference, so the PD produces ~0.7 mg of thrust,
never negative). At altitude < 0.5 m the emergency_landing event moves
the machine to LAND (guard: emergency_descent_active, latched on the
first EMERGENCY step), which finishes the touchdown at 0.5 m/s with the
near-field flare; the landed guard (EKF altitude < 0.1 m and |vz| <
0.1 m/s) drops to IDLE.

The horizontal position at touchdown is intentionally NOT part of the
acceptance: with a 25 m-corrupted horizontal estimate the vehicle holds
wherever the corrupted solution says it is, and LAND refuses to chase a
large horizontal error (documented in talon.mavlink_sil). What must
still hold is the VERTICAL channel, which the corruption does not touch:
a controlled descent and a soft, settled stop.

Acceptance:

- EMERGENCY entered within 1 s of the injection
- LAND reached from EMERGENCY (emergency_landing accepted)
- IDLE reached from LAND (landed accepted)
- final truth altitude < 0.5 m, final |vz| < 1.0 m/s
- descent is monotone-ish: no upward excursion > 1 m after EMERGENCY entry
"""

from __future__ import annotations

import sys

import numpy as np

from talon.mavlink_sil import SILConfig, SILResult, SILRunner
from talon.types import FlightPhase

DURATION_S = 50.0
INJECT_S = 20.0
ALTITUDE_M = 10.0

DETECT_MAX_DELAY_S = 1.0
FINAL_ALTITUDE_MAX_M = 0.5
FINAL_VZ_MAX_MPS = 1.0
ASCENT_TOLERANCE_M = 1.0


def default_config() -> SILConfig:
    """Hover-only SIL config (calm air: the failure under test is the EKF)."""
    return SILConfig()


def run_scenario(
    config: SILConfig | None = None, duration_s: float = DURATION_S
) -> SILResult:
    """Hover at 10 m, corrupt the EKF at t = 20 s, land under emergency."""
    runner = SILRunner(config if config is not None else default_config())
    runner.request_event("arm")
    runner.request_event("takeoff")
    runner.schedule(
        INJECT_S, lambda: runner.inject_ekf_divergence("corrupt_bias")
    )
    return runner.run(duration_s)


def main() -> int:
    result = run_scenario()

    timeline = result.phase_timeline
    emergency_times = [t for t, p in timeline if p is FlightPhase.EMERGENCY]
    land_times = [t for t, p in timeline if p is FlightPhase.LAND]
    detect_delay = (
        emergency_times[0] - INJECT_S if emergency_times else float("inf")
    )

    accepted = [e for e in result.event_log if e.guard_ok]
    accepted_names = [e.event for e in accepted]

    time_s = result.time_s
    altitude = -result.truth_position[:, 2]  # above home, positive up
    if emergency_times:
        after_emergency = time_s >= emergency_times[0]
        max_ascent = (
            float(np.max(altitude[after_emergency] - altitude[after_emergency][0]))
            if np.any(after_emergency)
            else 0.0
        )
    else:
        max_ascent = float("inf")

    final_altitude = float(-result.truth_position[-1, 2])
    final_vz = float(abs(result.truth_velocity[-1, 2]))

    print("=== Phase 5 emergency (EKF corrupt_bias at t = 20 s) ===")
    print(f"duration: {time_s[-1]:.1f} s")
    print("phase timeline:")
    for t, phase in timeline:
        print(f"  t = {t:6.2f} s  -> {phase.value}")
    print(
        f"emergency detected {detect_delay:.3f} s after injection "
        f"(limit {DETECT_MAX_DELAY_S:.1f} s)"
    )
    print(
        "accepted events: "
        + ", ".join(f"{e.event}@{e.time:.2f}s" for e in accepted)
    )
    print(f"max ascent after EMERGENCY entry: {max_ascent:.3f} m")
    print(
        f"final truth: altitude {final_altitude:.3f} m, "
        f"|vz| {final_vz:.3f} m/s, "
        f"phase {result.final_phase.value}"
    )

    ok = True
    if detect_delay > DETECT_MAX_DELAY_S:
        print(f"FAIL: detection delay {detect_delay:.3f} s > 1.0 s")
        ok = False
    if FlightPhase.LAND not in result.phases_visited():
        print("FAIL: never reached LAND")
        ok = False
    if FlightPhase.IDLE not in result.phases_visited():
        print("FAIL: never reached IDLE (landed)")
        ok = False
    if "emergency_landing" not in accepted_names:
        print("FAIL: emergency_landing never accepted")
        ok = False
    if "landed" not in accepted_names:
        print("FAIL: landed never accepted")
        ok = False
    if final_altitude >= FINAL_ALTITUDE_MAX_M:
        print(
            f"FAIL: final altitude {final_altitude:.3f} m "
            f">= {FINAL_ALTITUDE_MAX_M} m"
        )
        ok = False
    if final_vz >= FINAL_VZ_MAX_MPS:
        print(
            f"FAIL: final |vz| {final_vz:.3f} m/s >= {FINAL_VZ_MAX_MPS} m/s"
        )
        ok = False
    if max_ascent > ASCENT_TOLERANCE_M:
        print(f"FAIL: ascent of {max_ascent:.3f} m during the emergency")
        ok = False
    if not ok:
        return 1
    print("PASS: emergency detected, controlled descent, settled")
    return 0


if __name__ == "__main__":
    sys.exit(main())
