/**
 * Display geometry tests.
 *
 * Two things here are easy to get silently wrong and expensive to notice: the
 * camera frame (a wrong frame leaves the vehicle off screen for most of a run)
 * and the sign of the attitude angles (a wrong pitch sign shows a climb as a
 * dive, which an educational tool can not afford).
 */

import { describe, expect, it } from 'vitest'
import { missionBounds, rollPitchYaw, SCENE_FROM_NED } from './view'
import { nedToScene } from '@/utils/interpolation'
import type { LiveConfig } from './types'

function config(overrides: Partial<LiveConfig> = {}): LiveConfig {
  return {
    id: 'test',
    label: 'Test',
    blurb: '',
    duration_s: 60,
    takeoff_altitude: 10,
    start_position_ned: [0, 0, 0],
    waypoints: [],
    obstacles: [],
    wind_sigma: [0, 0, 0],
    wind_steady_ned: [0, 0, 0],
    seed: 1,
    gps_position_noise: 0,
    gps_position_noise_vertical: 0,
    gps_velocity_noise: 0,
    cruise_hold_s: 0,
    auto_arm: true,
    ...overrides,
  }
}

describe('missionBounds', () => {
  it('includes the takeoff column, so height reads against the ground', () => {
    const bounds = missionBounds(config({ takeoff_altitude: 10 }))
    // The top of the climb is 10 m up; the center sits above the ground.
    expect(bounds.center[1]).toBeGreaterThan(0)
    expect(bounds.radius).toBeGreaterThanOrEqual(14)
  })

  it('covers every waypoint and keep-out zone', () => {
    const bounds = missionBounds(
      config({
        waypoints: [
          { position: [40, 0, -10], speed: 8, heading: 0, radius: 3 },
          { position: [40, 30, -10], speed: 8, heading: 0, radius: 3 },
        ],
        obstacles: [{ id: 'o', center_ned: [50, 30, -10], radius_m: 4 }],
      }),
    )
    // The contract: everything the mission defines is inside the sphere, so the
    // camera never loses the vehicle or a keep-out zone.
    const inside = (ned: [number, number, number]): boolean => {
      const [x, y, z] = nedToScene(ned)
      return Math.hypot(x - bounds.center[0], y - bounds.center[1], z - bounds.center[2]) <= bounds.radius
    }
    expect(inside([0, 0, 0])).toBe(true)
    expect(inside([40, 0, -10])).toBe(true)
    expect(inside([40, 30, -10])).toBe(true)
    // The far edge of the keep-out shell, not just its center.
    expect(inside([54, 30, -10])).toBe(true)
    expect(inside([50, 34, -10])).toBe(true)
    expect(inside([50, 30, -14])).toBe(true)
    // The top of the takeoff column.
    expect(inside([0, 0, -10])).toBe(true)
    // And it is a tight frame, not a huge one: the radius is the mission's scale.
    expect(bounds.radius).toBeLessThan(60)
  })

  it('falls back to a readable volume for a degenerate config', () => {
    const bounds = missionBounds(config({ takeoff_altitude: 0 }))
    expect(Number.isFinite(bounds.center[0])).toBe(true)
    expect(bounds.radius).toBeGreaterThanOrEqual(14)
  })
})

describe('rollPitchYaw', () => {
  it('reads level flight as zero', () => {
    const { roll, pitch, yaw } = rollPitchYaw([1, 0, 0, 0])
    expect(roll).toBeCloseTo(0, 9)
    expect(pitch).toBeCloseTo(0, 9)
    expect(yaw).toBeCloseTo(0, 9)
  })

  it('reads nose-up pitch as positive', () => {
    // A rotation of +10 degrees about the body y axis (right wing) pitches the
    // nose up in the NED convention.
    const half = (10 * Math.PI) / 180 / 2
    const { pitch } = rollPitchYaw([Math.cos(half), 0, Math.sin(half), 0])
    expect(pitch).toBeCloseTo((10 * Math.PI) / 180, 9)
  })

  it('reads a right roll as positive and a right turn as positive heading', () => {
    const rollHalf = (20 * Math.PI) / 180 / 2
    expect(rollPitchYaw([Math.cos(rollHalf), Math.sin(rollHalf), 0, 0]).roll).toBeCloseTo(
      (20 * Math.PI) / 180,
      9,
    )
    const yawHalf = (30 * Math.PI) / 180 / 2
    expect(rollPitchYaw([Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)]).yaw).toBeCloseTo(
      (30 * Math.PI) / 180,
      9,
    )
  })

  it('normalises a non-unit quaternion instead of scaling the angles', () => {
    const half = (15 * Math.PI) / 180 / 2
    const { pitch } = rollPitchYaw([2 * Math.cos(half), 0, 2 * Math.sin(half), 0])
    expect(pitch).toBeCloseTo((15 * Math.PI) / 180, 9)
  })
})

describe('SCENE_FROM_NED', () => {
  it('is the quarter turn about x that the scene transform implies', () => {
    const [w, x, y, z] = SCENE_FROM_NED
    expect(w).toBeCloseTo(Math.SQRT1_2, 12)
    expect(x).toBeCloseTo(Math.SQRT1_2, 12)
    expect(y).toBeCloseTo(0, 12)
    expect(z).toBeCloseTo(0, 12)
  })
})
