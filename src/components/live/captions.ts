/**
 * Teaching captions for the live run.
 *
 * The point of the Run section is to show the autopilot working, so every flight
 * phase states what the controller, the estimator, or the mission manager is
 * doing during it. The text names the actual technique rather than describing the
 * motion, because "the vehicle climbs" is visible and "the altitude loop is
 * driving collective thrust" is the part worth reading.
 */

import type { FlightPhase } from '@/types/flight'

export interface PhaseCaption {
  /** One-word state, as the state machine spells it. */
  title: string
  /** What the autopilot is doing, in the autopilot's terms. */
  body: string
}

export const PHASE_CAPTIONS: Record<FlightPhase, PhaseCaption> = {
  idle: {
    title: 'IDLE',
    body: 'Disarmed. The flight computer is up and the 15-state estimator is converging on the ground before the clock starts, which is why the estimate error falls here and the vehicle does not move.',
  },
  arm: {
    title: 'ARM',
    body: 'Armed. The motor mixer is live but the collective sits below hover, so the vehicle stays on the pad. The state machine waits here for the takeoff command.',
  },
  takeoff: {
    title: 'TAKEOFF',
    body: 'The altitude loop is driving collective thrust up the vertical channel toward the target height, with the attitude loop holding level. Horizontal guidance is disabled, so a wind gust shows up as a lean rather than a drift.',
  },
  cruise: {
    title: 'CRUISE',
    body: 'NLGL guidance is steering for a point a lookahead distance down the leg to the active waypoint, and the attitude loop is trading pitch for forward speed. The vehicle noses down here on purpose: that is how a quadrotor accelerates.',
  },
  waypoint: {
    title: 'WAYPOINT',
    body: 'Inside the acceptance radius. The mission manager advances the active waypoint and the guidance law re-targets the next leg. If this is the last one, the mission hands over to the return-to-launch phase.',
  },
  rtl: {
    title: 'RTL',
    body: 'Return to launch. The mission is over, so the vehicle flies the home leg with the same guidance law and then descends onto the launch point rather than landing wherever it finished.',
  },
  land: {
    title: 'LAND',
    body: 'Descent onto the home position, thrust ramped down through ground effect until the contact detector reports the vehicle is down. The altitude loop stays closed all the way in.',
  },
  emergency: {
    title: 'EMERGENCY',
    body: 'Flight-dynamics intervention: the estimator or the sensor health monitor failed a check, so the state machine has left the mission and is flying a controlled descent instead of continuing.',
  },
}

/** Short notes under the metric groups, so the numbers are readable as engineering. */
export const READOUT_NOTES = {
  attitude:
    'Roll, pitch, and heading come from the truth attitude streamed by the core. The autopilot never sees these; it sees the estimate.',
  estimate:
    'Estimate error is the distance between the truth state and what the 15-state filter believes. It never reaches zero: a GPS fix is a noisy measurement, and the filter trades that noise against the inertial drift.',
  gps: 'GPS fixes are accepted, gated, or skipped. Gated means the fix arrived but failed the innovation test, so it was rejected rather than trusted.',
  wind: 'Wind is the Dryden turbulence the plant is flying through, plus any steady component the scenario declares.',
  clearance:
    'Clearance is the distance from the vehicle to the nearest keep-out surface. Negative means the shell was penetrated.',
} as const
