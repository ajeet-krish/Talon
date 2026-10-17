import type { Obstacle, Vec3 } from '@/types/flight'

/**
 * Obstacle clearance math (schema 1.1). Keep-out zones are spheres in the
 * NED frame; clearance is the 3-D Euclidean distance from a position to the
 * nearest sphere *surface*. Positive means outside every zone, zero means on
 * a surface, negative means the position is inside a zone.
 *
 * Distance is frame-independent (the NED-to-scene map is a rotation), so
 * these helpers run on raw NED triples with no scene conversion.
 */

/** Surface clearance (m) from a NED position to one obstacle. */
export function obstacleClearance(posNed: Vec3, obstacle: Obstacle): number {
  const [n, e, d] = posNed
  const [cn, ce, cd] = obstacle.center_ned
  return Math.hypot(n - cn, e - ce, d - cd) - obstacle.radius_m
}

/**
 * Minimum surface clearance (m) from a NED position to the nearest keep-out
 * zone. Returns +Infinity when there are no obstacles, so callers can render
 * an "unbounded" state without special casing.
 */
export function surfaceClearance(
  posNed: Vec3,
  obstacles: readonly Obstacle[],
): number {
  let min = Infinity
  for (const obstacle of obstacles) {
    min = Math.min(min, obstacleClearance(posNed, obstacle))
  }
  return min
}

/** Id of the nearest keep-out zone, or null when there are none. */
export function nearestObstacleId(
  posNed: Vec3,
  obstacles: readonly Obstacle[],
): string | null {
  let bestId: string | null = null
  let best = Infinity
  for (const obstacle of obstacles) {
    const clearance = obstacleClearance(posNed, obstacle)
    if (clearance < best) {
      best = clearance
      bestId = obstacle.id
    }
  }
  return bestId
}

/**
 * Minimum surface clearance (m) over a whole trajectory, the headline number
 * the obstacle field is graded on. Returns +Infinity when either input is
 * empty.
 */
export function minSurfaceClearance(
  positions: readonly Vec3[],
  obstacles: readonly Obstacle[],
): number {
  if (positions.length === 0 || obstacles.length === 0) return Infinity
  let min = Infinity
  for (const position of positions) {
    min = Math.min(min, surfaceClearance(position, obstacles))
    if (min === -Infinity) break
  }
  return min
}

/** Clearance band used to tint the scene and the HUD readout. */
export type ClearanceBand = 'breach' | 'caution' | 'clear'

/**
 * Default clearance margin (m) at which clearance shifts from clear to
 * caution. Matches the planner's stated margin in
 * sims/scenario_obstacle_field.py so the visual band tracks the engineering
 * constraint rather than an arbitrary threshold.
 */
export const CLEARANCE_CAUTION_M = 2.0

/**
 * Keep-out palette, deliberately outside the Wong flight-phase palette.
 * Shared by ObstacleField, the HUD readout, and the legend so the two
 * readings of clearance never disagree.
 */
export const CLEARANCE_COLORS = {
  clear: '#d9862b',
  caution: '#f2c14e',
  breach: '#e2506a',
} as const

/**
 * Classify a clearance into visual bands. `caution` covers the approach
 * corridor (one obstacle radius' worth of standoff is not available here, so
 * the caller passes an absolute margin in meters).
 */
export function clearanceBand(
  clearance: number,
  cautionMarginM: number,
): ClearanceBand {
  if (clearance < 0) return 'breach'
  if (clearance < cautionMarginM) return 'caution'
  return 'clear'
}
