import { describe, expect, it } from 'vitest'
import {
  clearanceBand,
  minSurfaceClearance,
  nearestObstacleId,
  obstacleClearance,
  surfaceClearance,
} from './clearance'
import type { Obstacle, Vec3 } from '@/types/flight'

function obstacle(id: string, center: Vec3, radius: number): Obstacle {
  return { id, center_ned: center, radius_m: radius }
}

const OBS_A = obstacle('obs_north', [20, 3, -10], 4)
const OBS_B = obstacle('obs_east', [37, 15, -10], 3.5)

describe('obstacleClearance', () => {
  it('is positive outside the zone', () => {
    // (20, 3, 0) is 10 m straight up from the centre, minus a 4 m radius.
    expect(obstacleClearance([20, 3, 0], OBS_A)).toBeCloseTo(6, 10)
  })

  it('is zero on the surface', () => {
    expect(obstacleClearance([24, 3, -10], OBS_A)).toBeCloseTo(0, 10)
  })

  it('is negative inside the zone', () => {
    expect(obstacleClearance([22, 3, -10], OBS_A)).toBeCloseTo(-2, 10)
  })

  it('measures the full 3-D distance, not a ground projection', () => {
    // Directly above the centre by exactly the radius: clearance is zero even
    // though the horizontal offset is zero.
    expect(obstacleClearance([20, 3, -6], OBS_A)).toBeCloseTo(0, 10)
  })
})

describe('surfaceClearance', () => {
  it('returns +Infinity with no obstacles', () => {
    expect(surfaceClearance([0, 0, 0], [])).toBe(Infinity)
  })

  it('returns the nearest zone clearance', () => {
    // Near obs_north (clearance 1) and far from obs_east.
    expect(surfaceClearance([20, 3, -5], [OBS_A, OBS_B])).toBeCloseTo(1, 10)
  })
})

describe('nearestObstacleId', () => {
  it('names the closest zone', () => {
    expect(nearestObstacleId([36, 15, -10], [OBS_A, OBS_B])).toBe('obs_east')
  })

  it('is null with no obstacles', () => {
    expect(nearestObstacleId([0, 0, 0], [])).toBeNull()
  })
})

describe('minSurfaceClearance', () => {
  it('returns +Infinity with no obstacles', () => {
    expect(minSurfaceClearance([[0, 0, 0]], [])).toBe(Infinity)
  })

  it('returns +Infinity with no samples', () => {
    expect(minSurfaceClearance([], [OBS_A])).toBe(Infinity)
  })

  it('finds the worst clearance along a trajectory', () => {
    const path: Vec3[] = [
      [10, 0, 0],
      [20, 3, -6], // closest approach: 0 m to the surface
      [30, 0, 0],
    ]
    expect(minSurfaceClearance(path, [OBS_A])).toBeCloseTo(0, 10)
  })

  it('reports penetration as negative', () => {
    expect(minSurfaceClearance([[21, 3, -10]], [OBS_A])).toBeCloseTo(-3, 10)
  })
})

describe('clearanceBand', () => {
  it('bands negative clearance as breach', () => {
    expect(clearanceBand(-0.1, 2)).toBe('breach')
  })

  it('bands sub-margin clearance as caution', () => {
    expect(clearanceBand(0, 2)).toBe('caution')
    expect(clearanceBand(1.9, 2)).toBe('caution')
  })

  it('bands margin-and-above as clear', () => {
    expect(clearanceBand(2, 2)).toBe('clear')
    expect(clearanceBand(9, 2)).toBe('clear')
  })
})
