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
