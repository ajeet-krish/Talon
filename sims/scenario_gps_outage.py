"""Phase 5 GPS outage scenario: 4 s mid-mission GPS blackout.

Full EKF-in-the-loop stack on the SILRunner: arm, takeoff, a 2-waypoint
circuit at 10 m, with a 4 s GPS outage injected at t = 20 s. 4 s is
deliberately UNDER the 5 s gps_lost guard of the state machine, so this
scenario demonstrates the boundary: the EKF must dead-reckon on IMU
alone through the outage and reconverge when the fixes resume, and the
state machine must stay out of EMERGENCY the whole time.

The gps_lost guard is evaluated from WAYPOINT ONLY (the transition
table has no gps_lost entry from any other phase). An outage longer
than 5 s while still in WAYPOINT would be an EMERGENCY by design; but
if the mission completes during the outage, mission_complete is
checked first, the vehicle transitions to RTL, and it flies home on
the dead-reckoned estimate by design (no EMERGENCY). See
test_state_machine.test_gps_lost_waypoint_only_documents_phase_dependence.

Documented expectation: with production IMU noise densities (0.004
m/s^2/sqrt(Hz)) and bias estimation already converged, 4 s of pure
dead reckoning grows the position error by well under 1 m, far inside
the 5 m acceptance. The interesting failure modes (bias corruption,
adversarial GPS) are exercised by scenario_emergency.

Acceptance:

- EKF position error at the END of the outage < 5 m (dead-reckon bound)
- EKF reconverges to < 1.0 m within 5 s of GPS resuming
- the state machine NEVER enters EMERGENCY
- the mission still completes (the outage does not strand guidance)
"""

from __future__ import annotations

import sys

import numpy as np

from talon.mavlink_sil import SILConfig, SILResult, SILRunner
from talon.types import FlightPhase, Waypoint

DURATION_S = 70.0
OUTAGE_START_S = 20.0
OUTAGE_DURATION_S = 4.0
ALTITUDE_M = 10.0
WP_RADIUS_M = 5.0

OUTAGE_END_ERROR_MAX_M = 5.0
RECONVERGE_TIME_MAX_S = 5.0
RECONVERGE_ERROR_MAX_M = 1.0

WAYPOINT_POSITIONS = [
    (40.0, 0.0),
    (40.0, 40.0),
]


def default_config() -> SILConfig:
    """2-waypoint circuit config."""
    return SILConfig(
        waypoints=[
            Waypoint(
                np.array([north, east, -ALTITUDE_M]), radius=WP_RADIUS_M
            )
            for north, east in WAYPOINT_POSITIONS
        ]
    )


def run_scenario(
    config: SILConfig | None = None, duration_s: float = DURATION_S
) -> SILResult:
    """Fly the circuit with a 4 s GPS outage at t = 20 s."""
    runner = SILRunner(config if config is not None else default_config())
    runner.request_event("arm")
    runner.request_event("takeoff")
    runner.schedule(
        OUTAGE_START_S,
        lambda: runner.inject_gps_outage(OUTAGE_DURATION_S),
    )
    return runner.run(duration_s)


def main() -> int:
    result = run_scenario()

    time_s = result.time_s
    error = np.linalg.norm(result.ekf_position - result.truth_position, axis=1)
    outage_end = OUTAGE_START_S + OUTAGE_DURATION_S

    i_start = int(np.searchsorted(time_s, OUTAGE_START_S, side="right") - 1)
    i_end = int(np.searchsorted(time_s, outage_end, side="right") - 1)
    err_start = float(error[i_start])
    err_end = float(error[i_end])
    growth = err_end - err_start

    after = time_s >= outage_end
    post_times = time_s[after]
    post_errors = error[after]
    below = np.nonzero(post_errors < RECONVERGE_ERROR_MAX_M)[0]
    reconverge_s = (
        float(post_times[below[0]] - outage_end) if below.size else float("inf")
    )
    err_at_reconverge_deadline = (
        float(post_errors[post_times <= outage_end + RECONVERGE_TIME_MAX_S][-1])
        if np.any(post_times <= outage_end + RECONVERGE_TIME_MAX_S)
        else float("inf")
    )

    emergency = FlightPhase.EMERGENCY in result.phases_visited()

    print(
        "=== Phase 5 GPS outage "
        f"({OUTAGE_DURATION_S:.0f} s blackout at t = {OUTAGE_START_S:.0f} s, "
        "2-waypoint circuit) ==="
    )
    print(f"duration: {time_s[-1]:.1f} s")
    print(
        f"EKF position error: {err_start:.3f} m at outage start, "
        f"{err_end:.3f} m at outage end (growth {growth:+.3f} m)"
    )
    print(
        f"reconvergence after resume: {reconverge_s:.2f} s to "
        f"< {RECONVERGE_ERROR_MAX_M:.1f} m "
        f"(error at +{RECONVERGE_TIME_MAX_S:.0f} s: "
        f"{err_at_reconverge_deadline:.3f} m)"
    )
    print(
        f"mission: complete={result.metrics['mission_complete'] == 1.0}, "
        f"waypoints reached: "
        f"{result.metrics['waypoints_reached']}/{len(WAYPOINT_POSITIONS)}"
    )
    print(
        "phases visited: "
        + ", ".join(sorted(p.value for p in result.phases_visited()))
    )
    print(
        f"GPS updates: accepted {result.metrics['gps_accepted']}, "
        f"gated {result.gps_gated}, skipped(outage) {result.gps_skipped}"
    )

    ok = True
    if err_end >= OUTAGE_END_ERROR_MAX_M:
        print(
            f"FAIL: outage-end error {err_end:.3f} m "
            f">= {OUTAGE_END_ERROR_MAX_M} m"
        )
        ok = False
    if reconverge_s > RECONVERGE_TIME_MAX_S:
        print(
            f"FAIL: reconvergence took {reconverge_s:.2f} s "
            f"> {RECONVERGE_TIME_MAX_S} s"
        )
        ok = False
    if err_at_reconverge_deadline >= RECONVERGE_ERROR_MAX_M:
        print(
            "FAIL: error still "
            f"{err_at_reconverge_deadline:.3f} m at the deadline"
        )
        ok = False
    if emergency:
        print("FAIL: state machine entered EMERGENCY during a 4 s outage")
        ok = False
    if result.metrics["mission_complete"] != 1.0:
        print("FAIL: mission did not complete")
        ok = False
    if not ok:
        return 1
    print("PASS: outage survived, no EMERGENCY, mission complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
