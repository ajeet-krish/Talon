/**
 * Preview framing tests (FlightLab Phase 8d-D): the pure maths the 3D
 * course preview frames its camera and ground grid with. Three claims the
 * source contract cannot see: the bounds actually hold every waypoint and
 * zone (counting its sphere) padded by the chart's own margin, the pose
 * guarantees a fit at the panel aspect with sane clamps, and the grid always
 * covers the bounds with its centre snapped to a round-meter cell line.
 */

import { describe, expect, it } from 'vitest'
import { CHART_EMPTY_SPAN_M, CHART_PADDING_M, gridStepM } from '@/components/editor/chart'
import type { Obstacle, Waypoint } from '@/types/generated/talon_core'
import { emptyMission, type MissionConfig } from '@/utils/editor'
import {
  PREVIEW_FOV_DEG,
  missionSceneBounds,
  previewCameraPose,
  previewGrid,
} from './previewFraming'

function leg(north: number, east: number, altitude = 10): Waypoint {
  return { position: [north, east, -altitude], speed: 5, heading: 0, radius: 2 }
}

function zone(north: number, east: number, radiusM: number): Obstacle {
  return { id: 'zone_1', center_ned: [north, east, -5], radius_m: radiusM }
}

function withGeometry(waypoints: Waypoint[], obstacles: Obstacle[] = []): MissionConfig {
  const base = emptyMission('preview_test')
  return { ...base, start_position_ned: [0, 0, 0], waypoints, obstacles }
}

/** Euclidean distance between the pose position and its target. */
function poseDistance(bounds: ReturnType<typeof missionSceneBounds>, aspect: number): number {
  const pose = previewCameraPose(bounds, aspect)
  return Math.hypot(
    pose.position[0] - pose.target[0],
    pose.position[1] - pose.target[1],
    pose.position[2] - pose.target[2],
  )
}

describe('missionSceneBounds', () => {
  it('frames an empty mission as the chart 60 m field on the start', () => {
    const bounds = missionSceneBounds(withGeometry([]))
    expect(bounds.max[0] - bounds.min[0]).toBeCloseTo(CHART_EMPTY_SPAN_M)
    expect(bounds.max[2] - bounds.min[2]).toBeCloseTo(CHART_EMPTY_SPAN_M)
    // Centred on the start (the origin here), never a degenerate point.
    expect(bounds.center[0]).toBeCloseTo(0)
    expect(bounds.center[2]).toBeCloseTo(0)
    expect(bounds.radius).toBeGreaterThan(8)
  })

  it('holds every waypoint and zone sphere, padded by the chart margin', () => {
    const bounds = missionSceneBounds(
      withGeometry([leg(100, 50, 20), leg(0, 0, 10)], [zone(200, 0, 15)]),
    )
    // Start and low waypoint set the minimum side...
    expect(bounds.min[0]).toBeLessThanOrEqual(0 - CHART_PADDING_M)
    expect(bounds.min[2]).toBeLessThanOrEqual(0 - CHART_PADDING_M)
    // ...the waypoint's altitude lifts the top, with padding.
    expect(bounds.max[1]).toBeGreaterThanOrEqual(20 + CHART_PADDING_M)
    // ...and the zone counts as its whole sphere, radius and all.
    expect(bounds.max[0]).toBeGreaterThanOrEqual(200 + 15 + CHART_PADDING_M)
    expect(bounds.max[2]).toBeGreaterThanOrEqual(50 + CHART_PADDING_M)
  })

  it('sizes the radius to half the box diagonal, with a floor', () => {
    const bounds = missionSceneBounds(withGeometry([leg(100, 0), leg(0, 500)]))
    const diagonal = Math.hypot(
      bounds.max[0] - bounds.min[0],
      bounds.max[1] - bounds.min[1],
      bounds.max[2] - bounds.min[2],
    )
    expect(bounds.radius).toBeCloseTo(diagonal / 2)
    expect(bounds.radius).toBeGreaterThanOrEqual(8)
  })

  it('floors the radius for a degenerate route', () => {
    const bounds = missionSceneBounds(withGeometry([leg(0, 0), leg(1, 0)]))
    expect(bounds.radius).toBeGreaterThanOrEqual(8)
  })

  it('never mutates the config it frames', () => {
    const config = withGeometry([leg(10, 20)], [zone(30, 40, 5)])
    const before = JSON.stringify(config)
    missionSceneBounds(config)
    expect(JSON.stringify(config)).toBe(before)
  })
})

describe('previewCameraPose', () => {
  const bounds = missionSceneBounds(withGeometry([leg(120, 60, 30), leg(0, 0)], [zone(40, 90, 12)]))

  it('stands along the replay view direction at a guaranteed fit distance', () => {
    const pose = previewCameraPose(bounds, 1)
    const offset: [number, number, number] = [
      pose.position[0] - pose.target[0],
      pose.position[1] - pose.target[1],
      pose.position[2] - pose.target[2],
    ]
    // Direction ratios match the replay CameraRig's framing.
    expect(offset[0] / offset[1]).toBeCloseTo(1.25 / 0.85, 5)
    expect(offset[2] / offset[1]).toBeCloseTo(1.15 / 0.85, 5)
    // At aspect 1 the tighter axis is the vertical fov: a sphere of the
    // bounds radius must fit inside it, with the 8% margin.
    const halfV = ((PREVIEW_FOV_DEG * Math.PI) / 180) / 2
    const distance = Math.hypot(offset[0], offset[1], offset[2])
    expect(distance).toBeCloseTo((bounds.radius / Math.sin(halfV)) * 1.08, 6)
    expect(Math.asin(bounds.radius / distance)).toBeLessThan(halfV)
  })

  it('pulls back for a narrower panel so a tall split still fits', () => {
    expect(poseDistance(bounds, 0.6)).toBeGreaterThan(poseDistance(bounds, 1.6))
    expect(poseDistance(bounds, 0.4)).toBeCloseTo(poseDistance(bounds, 0.5)) // aspect floor
    expect(poseDistance(bounds, Number.NaN)).toBeGreaterThan(0) // junk reads as square
  })

  it('never targets below the ground and keeps near/far inside the clamps', () => {
    // A zone sunk below the surface drags the box centre under y = 0, so
    // the target has to clamp up to the ground plane.
    const low = missionSceneBounds(
      withGeometry([], [{ id: 'deep', center_ned: [0, 0, 30], radius_m: 40 }]),
    )
    expect(low.center[1]).toBeLessThan(0)
    const pose = previewCameraPose(low, 1.2)
    expect(pose.target[1]).toBeGreaterThanOrEqual(0)
    expect(pose.near).toBeGreaterThan(0)
    expect(pose.near).toBeLessThan(pose.minDistance)
    expect(pose.minDistance).toBeLessThan(pose.maxDistance)
    // Everything the controls allow must be inside the far plane.
    expect(pose.far).toBeGreaterThan(pose.maxDistance + low.radius)
    // And the fit distance itself sits inside the zoom range.
    const fit = poseDistance(low, 1.2)
    expect(fit).toBeGreaterThan(pose.minDistance)
    expect(fit).toBeLessThan(pose.maxDistance)
  })
})

describe('previewGrid', () => {
  it('always covers the bounds with its centre snapped to a cell line', () => {
    const cases = [
      withGeometry([]),
      withGeometry([leg(1000, 500), leg(0, 0)]),
      withGeometry([leg(-37, 12), leg(8, -90)], [zone(240, -180, 30)]),
    ]
    for (const config of cases) {
      const bounds = missionSceneBounds(config)
      const grid = previewGrid(bounds)
      const step = grid.size / grid.divisions
      expect(grid.divisions).toBe(10)
      expect(Number.isInteger(step)).toBe(true)
      expect(grid.position[0] % step).toBeCloseTo(0, 9)
      expect(grid.position[2] % step).toBeCloseTo(0, 9)
      expect(grid.position[1]).toBe(0)
      expect(grid.position[0] - grid.size / 2).toBeLessThanOrEqual(bounds.min[0])
      expect(grid.position[0] + grid.size / 2).toBeGreaterThanOrEqual(bounds.max[0])
      expect(grid.position[2] - grid.size / 2).toBeLessThanOrEqual(bounds.min[2])
      expect(grid.position[2] + grid.size / 2).toBeGreaterThanOrEqual(bounds.max[2])
    }
  })

  it('takes its step from the chart ladder, with headroom for the snap', () => {
    const bounds = missionSceneBounds(withGeometry([leg(0, 0), leg(400, 0)]))
    const grid = previewGrid(bounds)
    const extent = Math.max(bounds.max[0] - bounds.min[0], bounds.max[2] - bounds.min[2])
    // The same round-meter rule the chart's grid uses, over the box the
    // chart would frame, so the two grids read as one instrument.
    expect(grid.size / grid.divisions).toBe(gridStepM(extent * 1.2))
  })

  it('grows the cells with the mission', () => {
    const small = previewGrid(missionSceneBounds(withGeometry([leg(0, 0), leg(10, 0)])))
    const large = previewGrid(missionSceneBounds(withGeometry([leg(0, 0), leg(4000, 0)])))
    expect(large.size).toBeGreaterThan(small.size)
  })

  it('covers an extreme mission even where the round ladder runs out', () => {
    const bounds = missionSceneBounds(withGeometry([leg(50000, 0), leg(-50000, 0)]))
    const grid = previewGrid(bounds)
    const step = grid.size / grid.divisions
    // Past the ladder's ceiling the extent/9 floor takes over: coverage
    // first, round cells where round cells exist.
    expect(step).toBeGreaterThan(10000)
    expect(grid.position[0] - grid.size / 2).toBeLessThanOrEqual(bounds.min[0])
    expect(grid.position[0] + grid.size / 2).toBeGreaterThanOrEqual(bounds.max[0])
    expect(grid.position[2] - grid.size / 2).toBeLessThanOrEqual(bounds.min[2])
    expect(grid.position[2] + grid.size / 2).toBeGreaterThanOrEqual(bounds.max[2])
  })
})
