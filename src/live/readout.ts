/**
 * One frame's worth of numbers for the live rail.
 *
 * The rail is text, so it wants a plain object it can render; the scene is
 * geometry, so it reads the recorder directly. Splitting them this way means the
 * numbers are produced by one function that a test can drive with a fake
 * session, rather than by JSX that a test can not.
 *
 * Every value here is read from the newest recorded sample rather than from the
 * session's live getters, so the readout and the trail agree: what the numbers
 * say and what the map draws are the same instant of the flight.
 */

import type { FlightPhase } from '@/types/flight'
import { surfaceClearance } from '@/utils/clearance'
import type { LiveSim } from './sim'
import { emptyFrame, LiveBuffer } from './stream'
import type { LiveConfig } from './types'
import { rollPitchYaw } from './view'

/** Numeric state of a running scenario, for the rail. */
export interface LiveReadout {
  phase: FlightPhase
  /** Sim time at the newest sample (s). */
  time: number
  /** Fraction of the run completed, in [0, 1]. */
  progress: number
  /** Altitude above the ground plane (m, positive up). */
  altitude: number
  /** Climb rate (m/s, positive up). */
  verticalSpeed: number
  /** Horizontal speed (m/s). */
  groundSpeed: number
  /** Roll (rad). */
  roll: number
  /** Pitch (rad). */
  pitch: number
  /** Heading from north toward east (rad). */
  heading: number
  /** Distance between the truth and the estimate (m). */
  estimateError: number
  /** Wind magnitude (m/s). */
  windSpeed: number
  /** Delivered collective thrust (N). */
  thrust: number
  /** Battery state of charge in [0, 1]. */
  battery: number
  /** GPS fixes: accepted, gated, skipped. */
  gps: [number, number, number]
  /** Estimator flags: converged, diverged. */
  ekf: [boolean, boolean]
  /** Surface clearance to the nearest keep-out zone (m), or Infinity. */
  clearance: number
}

/** Scratch frame, reused so a 10 Hz readout allocates nothing. */
const scratch = emptyFrame()

/**
 * Read the current numbers out of a running session and its recorder.
 *
 * Returns null before the first sample has arrived, which is the honest answer
 * rather than a row of zeros that looks like a vehicle sitting on the pad.
 */
export function readReadout(
  sim: LiveSim,
  buffer: LiveBuffer,
  cfg: LiveConfig,
): LiveReadout | null {
  if (!buffer.latest(scratch)) return null

  const [pn, pe, pd] = scratch.truthPosition
  const [, , vd] = scratch.truthVelocity
  const attitude = rollPitchYaw(scratch.quaternion)
  const [en, ee, ed] = scratch.ekfPosition
  const [wn, we, wd] = scratch.wind
  const [accepted, gated, skipped] = sim.gpsCounts()
  const [converged, diverged] = sim.ekfFlags()

  return {
    phase: sim.phase,
    time: scratch.time,
    progress: sim.progress,
    altitude: -pd,
    verticalSpeed: -vd,
    groundSpeed: Math.hypot(scratch.truthVelocity[0], scratch.truthVelocity[1]),
    roll: attitude.roll,
    pitch: attitude.pitch,
    heading: attitude.yaw,
    estimateError: Math.hypot(pn - en, pe - ee, pd - ed),
    windSpeed: Math.hypot(wn, we, wd),
    thrust: scratch.thrust,
    battery: sim.batterySoc,
    gps: [accepted, gated, skipped],
    ekf: [converged, diverged],
    clearance: surfaceClearance(scratch.truthPosition, cfg.obstacles),
  }
}

/** How many samples a run of `durationS` will record, plus headroom. */
export function bufferCapacity(cfg: LiveConfig, controlDtS: number): number {
  return Math.ceil(cfg.duration_s / controlDtS) + 64
}
