/**
 * Display-helper tests for the mission editor chart and readouts (FlightLab
 * Phase 8c). Pure functions only: bounds framing, round grid/label/scale
 * steps, wind direction, fault labels, and the flight-time format.
 */

import { describe, expect, it } from 'vitest'
import { emptyMission, type MissionConfig } from '@/utils/editor'
import type { FaultSpec, Obstacle, Waypoint } from '@/types/generated/talon_core'
import {
  CHART_EMPTY_SPAN_M,
  CHART_PADDING_M,
  boundsHeight,
  boundsViewBox,
  boundsWidth,
  chartBounds,
  compassPoint,
  faultLabel,
  formatMinutesSeconds,
  formatWindSummary,
  gridStepM,
  labelStepM,
  scaleBarLengthM,
  windFromDirectionDeg,
  windSpeedMps,
} from './chart'

function leg(north: number, east: number, altitude = 10): Waypoint {
  return { position: [north, east, -altitude], speed: 5, heading: 0, radius: 2 }
}

function zone(north: number, east: number, radiusM: number): Obstacle {
  return { id: 'zone_1', center_ned: [north, east, -10], radius_m: radiusM }
}

function withGeometry(waypoints: Waypoint[], obstacles: Obstacle[] = []): MissionConfig {
  const base = emptyMission('chart_test')
  return { ...base, start_position_ned: [0, 0, 0], waypoints, obstacles }
}

describe('chartBounds', () => {
  it('frames an empty mission as a 60 m field centred on the start', () => {
    const bounds = chartBounds(withGeometry([]))
    expect(boundsWidth(bounds)).toBeCloseTo(CHART_EMPTY_SPAN_M)
    expect(boundsHeight(bounds)).toBeCloseTo(CHART_EMPTY_SPAN_M)
    // Centred on the origin: symmetric around zero in both axes.
    expect(bounds.minX).toBeCloseTo(-CHART_EMPTY_SPAN_M / 2)
    expect(bounds.maxX).toBeCloseTo(CHART_EMPTY_SPAN_M / 2)
    expect(bounds.minY).toBeCloseTo(-CHART_EMPTY_SPAN_M / 2)
    expect(bounds.maxY).toBeCloseTo(CHART_EMPTY_SPAN_M / 2)
  })

  it('frames the route with padding, in chart space (y is negative north)', () => {
    const bounds = chartBounds(withGeometry([leg(0, 0), leg(40, 30)]))
    expect(bounds.minX).toBeCloseTo(0 - CHART_PADDING_M)
    expect(bounds.maxX).toBeCloseTo(30 + CHART_PADDING_M)
    // North 40 m maps to chart y = -40, the top of the frame.
    expect(bounds.minY).toBeCloseTo(-40 - CHART_PADDING_M)
    expect(bounds.maxY).toBeCloseTo(0 + CHART_PADDING_M)
  })

  it('includes zone radius and the start point in the frame', () => {
    const bounds = chartBounds(withGeometry([leg(10, 10)], [zone(0, -6, 5)]))
    // The zone reaches east -11 m, west of the start, and that widens the frame.
    expect(bounds.minX).toBeCloseTo(-6 - 5 - CHART_PADDING_M)
    expect(bounds.maxX).toBeCloseTo(10 + CHART_PADDING_M)
    expect(bounds.minY).toBeCloseTo(-10 - CHART_PADDING_M)
    // The zone reaches north -5 m, below the start, and that sets the bottom.
    expect(bounds.maxY).toBeCloseTo(5 + CHART_PADDING_M)
  })

  it('centres an empty frame on a start point away from the origin', () => {
    const base = withGeometry([])
    const moved: MissionConfig = { ...base, start_position_ned: [100, -50, 0] }
    const bounds = chartBounds(moved)
    expect((bounds.minX + bounds.maxX) / 2).toBeCloseTo(-50)
    expect((bounds.minY + bounds.maxY) / 2).toBeCloseTo(-100)
  })

  it('renders a viewBox string from the frame', () => {
    const bounds = chartBounds(withGeometry([]))
    expect(boundsViewBox(bounds)).toBe(
      `${bounds.minX} ${bounds.minY} ${CHART_EMPTY_SPAN_M} ${CHART_EMPTY_SPAN_M}`,
    )
  })
})

describe('grid and label steps', () => {
  it('grids a 60 m frame every 10 m and labels every 50 m', () => {
    expect(gridStepM(60)).toBe(10)
    expect(labelStepM(60)).toBe(50)
  })

  it('scales the steps up for large spans and down for small ones', () => {
    // A 200 m span still fits ten 20 m cells; 4000 m needs 500 m cells.
    expect(gridStepM(200)).toBe(20)
    expect(gridStepM(4000)).toBe(500)
    expect(gridStepM(24)).toBe(5)
    expect(labelStepM(4000)).toBe(2500)
  })

  it('always labels on a grid line', () => {
    for (const span of [16, 60, 120, 340, 900, 2500, 9000]) {
      expect(labelStepM(span) % gridStepM(span)).toBe(0)
    }
  })
})

describe('scaleBarLengthM', () => {
  it('picks the longest round length that fits a quarter of the frame', () => {
    // 60 m span: a quarter is 15 m, so the bar rounds down to 10 m.
    expect(scaleBarLengthM(60)).toBe(10)
    // 400 m span: a quarter is exactly 100 m, a round number itself.
    expect(scaleBarLengthM(400)).toBe(100)
    expect(scaleBarLengthM(100)).toBe(25)
  })

  it('never returns zero or exceeds the frame', () => {
    for (const span of [8, 30, 77, 500, 3000]) {
      const bar = scaleBarLengthM(span)
      expect(bar).toBeGreaterThan(0)
      expect(bar).toBeLessThanOrEqual(span)
    }
  })
})

describe('wind direction', () => {
  it('reports speed from the north and east components', () => {
    expect(windSpeedMps(3, 4)).toBeCloseTo(5)
    expect(windSpeedMps(0, 0)).toBe(0)
  })

  it('gives the meteorological source bearing, clockwise from north', () => {
    // A wind blowing toward the south comes FROM the north.
    expect(windFromDirectionDeg(-2, 0)).toBeCloseTo(0)
    // A wind blowing toward the east comes FROM the west.
    expect(windFromDirectionDeg(0, 2)).toBeCloseTo(270)
    // A wind blowing toward the north comes FROM the south.
    expect(windFromDirectionDeg(2, 0)).toBeCloseTo(180)
  })

  it('is null in calm air and formats a summary line', () => {
    expect(windFromDirectionDeg(0, 0)).toBeNull()
    expect(formatWindSummary(0, 0)).toBe('CALM')
    expect(formatWindSummary(-3, 0)).toBe('3.0 m/s FROM 0° (N)')
    expect(formatWindSummary(0, 3)).toBe('3.0 m/s FROM 270° (W)')
  })

  it('labels the nearest compass point', () => {
    expect(compassPoint(0)).toBe('N')
    expect(compassPoint(22.5)).toBe('NNE')
    expect(compassPoint(350)).toBe('N')
    expect(compassPoint(180)).toBe('S')
  })
})

describe('faultLabel', () => {
  it('names each fault kind with its parameters', () => {
    const cases: [FaultSpec, string][] = [
      [{ kind: 'gps_outage', at_s: 20, duration_s: 4 }, 'GPS outage at 20.0 s for 4.0 s'],
      [
        { kind: 'ekf_divergence', at_s: 12.5, mode: 'corrupt_bias' },
        'EKF divergence at 12.5 s (corrupt_bias)',
      ],
      [{ kind: 'battery_soc', at_s: 30, soc: 0.4 }, 'Battery forced to 0.40 at 30.0 s'],
      [{ kind: 'motor_fault', at_s: 8 }, 'Motor fault at 8.0 s'],
      [{ kind: 'emergency', at_s: 5.25 }, 'Emergency at 5.3 s'],
      [{ kind: 'rtl', at_s: 45 }, 'RTL at 45.0 s'],
    ]
    for (const [fault, label] of cases) {
      expect(faultLabel(fault)).toBe(label)
    }
  })
})

describe('formatMinutesSeconds', () => {
  it('formats seconds as m:ss', () => {
    expect(formatMinutesSeconds(0)).toBe('0:00')
    expect(formatMinutesSeconds(65)).toBe('1:05')
    expect(formatMinutesSeconds(125.4)).toBe('2:05')
    expect(formatMinutesSeconds(3599)).toBe('59:59')
  })

  it('floors negative input at zero rather than printing minus minutes', () => {
    expect(formatMinutesSeconds(-10)).toBe('0:00')
  })
})
