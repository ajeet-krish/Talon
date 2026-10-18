/**
 * Display geometry for the live run.
 *
 * Everything here is pure: framing for the scene camera, and attitude angles
 * for the rail. It is separate from `stream.ts` because it is about presenting
 * a run, not about decoding one, and it is testable without a canvas.
 *
 * The live run can not be framed from its own trajectory the way a recorded one
 * is: at the start the trajectory is a single point, so the camera would zoom to
 * the launch pad and then need to pull back as the vehicle flew away. Instead the
 * frame comes from the mission itself, which is known before the run starts.
 */

import type { Vec3 } from '@/types/flight'
import type { LiveConfig } from './types'
import { nedToScene } from '@/utils/interpolation'

/** Scene-space bounding sphere. */
export interface SceneBounds {
  center: [number, number, number]
  radius: number
}

/** Minimum framing radius (m): a hover still needs a readable volume. */
const MIN_RADIUS = 14

/** How much of the mission extent to add as margin. */
const MARGIN = 1.18

/**
 * Bounding sphere of the mission: start, waypoints, keep-out zones, and the
 * takeoff column, in scene space.
 *
 * Used for the initial camera framing so the whole flight is in view from the
 * first frame, whatever the vehicle does later. The ground is included on
 * purpose: a scene that frames only the flight volume loses the sense of height.
 */
export function missionBounds(cfg: LiveConfig): SceneBounds {
  let minX = Infinity
  let minY = Infinity
  let minZ = Infinity
  let maxX = -Infinity
  let maxY = -Infinity
  let maxZ = -Infinity

  const include = (ned: Vec3, pad = 0): void => {
    const [x, y, z] = nedToScene(ned)
    minX = Math.min(minX, x - pad)
    maxX = Math.max(maxX, x + pad)
    minY = Math.min(minY, y - pad)
    maxY = Math.max(maxY, y + pad)
    minZ = Math.min(minZ, z - pad)
    maxZ = Math.max(maxZ, z + pad)
  }

  include(cfg.start_position_ned)
  for (const wp of cfg.waypoints) include(wp.position)
  for (const ob of cfg.obstacles) include(ob.center_ned, ob.radius_m)

  // The ground plane and the takeoff column, so height reads against something.
  const ground: Vec3 = [cfg.start_position_ned[0], cfg.start_position_ned[1], 0]
  include(ground, 0)
  const top: Vec3 = [cfg.start_position_ned[0], cfg.start_position_ned[1], -cfg.takeoff_altitude]
  include(top, cfg.takeoff_altitude * 0.25)

  if (!Number.isFinite(minX)) {
    return { center: [0, 8, 0], radius: MIN_RADIUS }
  }

  const center: [number, number, number] = [
    (minX + maxX) / 2,
    (minY + maxY) / 2,
    (minZ + maxZ) / 2,
  ]
  const diagonal = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ)
  const radius = Math.max((diagonal / 2) * MARGIN, MIN_RADIUS)
  return { center, radius }
}

/** One rung of the pad-side altitude ladder. */
export interface AltitudeTick {
  /** Height above the ground plane (m). */
  altitude: number
  /** True for the rung at an altitude the mission actually flies. */
  flightLevel: boolean
}

/** Measurement intervals the ladder may use (m), ascending. */
const NICE_STEPS = [0.5, 1, 2, 2.5, 5, 10, 20, 25, 50, 100]

/** The largest measurement interval at or below `value` (m), so the ladder
 *  never uses a rung spacing nobody measures in. Falls back to the finest
 *  interval when `value` is below all of them. */
export function niceStep(value: number): number {
  let step = NICE_STEPS[0] ?? 1
  for (const candidate of NICE_STEPS) {
    if (candidate <= value) step = candidate
  }
  return step
}

/** Altitudes the mission actually uses (m above the ground plane): the takeoff
 *  column and every waypoint. Ground level is excluded, it is the datum. */
export function missionAltitudes(cfg: LiveConfig): number[] {
  const altitudes = [cfg.takeoff_altitude, ...cfg.waypoints.map((w) => -w.position[2])]
  return altitudes.filter((a) => Number.isFinite(a) && a > 0)
}

/**
 * The graduated altitude ladder drawn beside the pad.
 *
 * The rungs are a regular measurement grid, sized from the mission's own
 * scale, and the rung at an altitude the mission flies is flagged: the ladder
 * states the plan, not just the grid. The top rung is always the highest
 * altitude flown, so the ladder reaches the mission's ceiling even when the
 * grid runs out below it (at most `maxTicks` grid rungs, plus that ceiling).
 */
export function altitudeLadder(cfg: LiveConfig, maxTicks = 6): AltitudeTick[] {
  const flown = missionAltitudes(cfg)
  if (flown.length === 0) return []
  const top = Math.max(...flown)

  let step = niceStep(top / maxTicks)
  while (top / step > maxTicks) step *= 2

  const ticks: AltitudeTick[] = []
  const rungs = Math.floor(top / step + 1e-9)
  for (let i = 1; i <= rungs; i += 1) {
    const altitude = i * step
    ticks.push({
      altitude,
      flightLevel: flown.some((f) => Math.abs(f - altitude) <= step * 0.1),
    })
  }

  const highest = ticks.length > 0 ? (ticks[ticks.length - 1]?.altitude ?? 0) : 0
  if (top - highest > step * 0.1) {
    ticks.push({ altitude: top, flightLevel: true })
  }
  return ticks
}

/** Roll, pitch, and heading in radians, from a body-to-NED quaternion. */
export interface Attitude {
  roll: number
  pitch: number
  /** Heading, measured from north toward east. */
  yaw: number
}

/**
 * Euler angles for a Hamilton quaternion `[w, x, y, z]` rotating body into NED.
 *
 * Standard aerospace ZYX sequence: roll about the body x axis (forward), pitch
 * about the right wing, heading from north. These are the angles the rail shows,
 * computed on this side of the boundary: attitude is display-only, so it is not
 * in the schema, and the core streams the quaternion instead of choosing an
 * Euler convention on the autopilot's behalf.
 */
export function rollPitchYaw(q: [number, number, number, number]): Attitude {
  const [w, x, y, z] = q
  const norm = Math.hypot(w, x, y, z)
  if (norm === 0) return { roll: 0, pitch: 0, yaw: 0 }
  const [qw, qx, qy, qz] = [w / norm, x / norm, y / norm, z / norm]
  const roll = Math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
  const sinPitch = 2 * (qw * qy - qz * qx)
  const pitch = Math.asin(Math.min(Math.max(sinPitch, -1), 1))
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  return { roll, pitch, yaw }
}

/**
 * Quaternion mapping NED into the scene frame.
 *
 * The scene transform is a rotation of +90 degrees about x: north stays x, down
 * becomes -y, east becomes z. Applying it after the body-to-NED rotation gives
 * body-to-scene, which is what an Object3D needs.
 */
export const SCENE_FROM_NED: [number, number, number, number] = [
  Math.SQRT1_2,
  Math.SQRT1_2,
  0,
  0,
]
