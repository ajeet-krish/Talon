/**
 * Display-helper tests for the mission editor chart and readouts (FlightLab
 * Phase 8c, viewport Phase 8d-C). Pure functions only: bounds framing,
 * round grid/label/scale steps, the zoom and pan viewport (clamps, anchor
 * maths, fit, grid coverage over the visible rectangle), wind direction,
 * fault labels, and the flight-time format.
 */

import { describe, expect, it } from 'vitest'
import { emptyMission, type MissionConfig } from '@/utils/editor'
import type { FaultSpec, Obstacle, Waypoint } from '@/types/generated/talon_core'
import {
  CHART_EMPTY_SPAN_M,
  CHART_PADDING_M,
  MAX_GRID_LINES,
  SPAN_MAX_M,
  SPAN_MIN_M,
  boundsHeight,
  boundsViewBox,
  boundsWidth,
  canZoomIn,
  canZoomOut,
  chartBounds,
  clampSpan,
  compassPoint,
  faultLabel,
  formatMinutesSeconds,
  formatWindSummary,
  gridStepM,
  labelStepM,
  linesOver,
  panBy,
  scaleBarLengthM,
  viewBounds,
  viewFit,
  windFromDirectionDeg,
  windSpeedMps,
  zoomAt,
  type ChartView,
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

// ---------------------------------------------------------------------------
// Viewport (zoom and pan)
// ---------------------------------------------------------------------------

describe('clampSpan', () => {
  it('holds the zoom range and floors junk input', () => {
    expect(clampSpan(60)).toBe(60)
    expect(clampSpan(SPAN_MIN_M)).toBe(SPAN_MIN_M)
    expect(clampSpan(1)).toBe(SPAN_MIN_M)
    expect(clampSpan(-40)).toBe(SPAN_MIN_M)
    expect(clampSpan(1e9)).toBe(SPAN_MAX_M)
    expect(clampSpan(Number.NaN)).toBe(SPAN_MIN_M)
    expect(clampSpan(Number.POSITIVE_INFINITY)).toBe(SPAN_MAX_M)
    expect(clampSpan(Number.NEGATIVE_INFINITY)).toBe(SPAN_MIN_M)
  })
})

describe('viewBounds', () => {
  it('centres the visible rectangle on the view at a square aspect', () => {
    expect(viewBounds({ centerX: 40, centerY: -10, spanM: 100 }, 1)).toEqual({
      minX: -10,
      maxX: 90,
      minY: -60,
      maxY: 40,
    })
  })

  it('makes spanM the larger side at every aspect', () => {
    const wide = viewBounds({ centerX: 0, centerY: 0, spanM: 200 }, 2)
    expect(boundsWidth(wide)).toBe(200)
    expect(boundsHeight(wide)).toBe(100)
    const tall = viewBounds({ centerX: 0, centerY: 0, spanM: 200 }, 0.5)
    expect(boundsWidth(tall)).toBe(100)
    expect(boundsHeight(tall)).toBe(200)
  })

  it('never draws a degenerate or NaN frame, whatever state the view is in', () => {
    for (const aspect of [0, -2, Number.NaN, 0.3, 1, 4]) {
      for (const spanM of [SPAN_MIN_M, 60, SPAN_MAX_M, 1e9, Number.NaN, -7]) {
        const bounds = viewBounds({ centerX: 1000, centerY: -1000, spanM }, aspect)
        for (const edge of [bounds.minX, bounds.maxX, bounds.minY, bounds.maxY]) {
          expect(Number.isFinite(edge)).toBe(true)
        }
        expect(boundsWidth(bounds)).toBeGreaterThan(0)
        expect(boundsHeight(bounds)).toBeGreaterThan(0)
        expect(boundsWidth(bounds)).toBeLessThanOrEqual(SPAN_MAX_M)
        expect(boundsHeight(bounds)).toBeLessThanOrEqual(SPAN_MAX_M)
      }
      const junkCentre = viewBounds(
        { centerX: Number.NaN, centerY: Number.NaN, spanM: 60 },
        aspect,
      )
      expect(Number.isFinite(junkCentre.minX)).toBe(true)
      expect(Number.isFinite(junkCentre.minY)).toBe(true)
    }
  })
})

describe('viewFit', () => {
  const mission = chartBounds(withGeometry([leg(0, 0), leg(40, 30)]))

  it('frames the whole mission at every aspect, without clipping', () => {
    for (const aspect of [0.5, 1, 1.6, 2, 4]) {
      const shown = viewBounds(viewFit(mission, aspect), aspect)
      const slack = 1e-6
      expect(shown.minX).toBeLessThanOrEqual(mission.minX + slack)
      expect(shown.maxX).toBeGreaterThanOrEqual(mission.maxX - slack)
      expect(shown.minY).toBeLessThanOrEqual(mission.minY + slack)
      expect(shown.maxY).toBeGreaterThanOrEqual(mission.maxY - slack)
    }
  })

  it('centres on the mission and keeps an empty mission a planning field', () => {
    const view = viewFit(chartBounds(withGeometry([])), 1)
    expect(view.centerX).toBeCloseTo(0)
    expect(view.centerY).toBeCloseTo(0)
    expect(view.spanM).toBeCloseTo(CHART_EMPTY_SPAN_M)
  })

  it('clamps a mission wider than the zoom ceiling', () => {
    const huge = { minX: -200_000, maxX: 200_000, minY: -50, maxY: 50 }
    expect(viewFit(huge, 1).spanM).toBe(SPAN_MAX_M)
  })

  it('never returns a span below the floor', () => {
    const tiny = { minX: 0, maxX: 1, minY: 0, maxY: 1 }
    expect(viewFit(tiny, 1).spanM).toBe(SPAN_MIN_M)
  })
})

describe('zoomAt', () => {
  const view: ChartView = { centerX: 100, centerY: -50, spanM: 400 }

  it('shrinks the span when zooming in and grows it when zooming out', () => {
    expect(zoomAt(view, { x: 100, y: -50 }, 2).spanM).toBe(200)
    expect(zoomAt(view, { x: 100, y: -50 }, 0.5).spanM).toBe(800)
  })

  it('keeps the anchor fixed: the point under the cursor does not move', () => {
    const anchor = { x: 180, y: -90 }
    for (const factor of [1.3, 2, 0.8]) {
      const zoomed = zoomAt(view, anchor, factor)
      const ratio = zoomed.spanM / view.spanM
      expect(anchor.x - zoomed.centerX).toBeCloseTo((anchor.x - view.centerX) * ratio)
      expect(anchor.y - zoomed.centerY).toBeCloseTo((anchor.y - view.centerY) * ratio)
    }
  })

  it('lands exactly on the clamp and stops there', () => {
    const floor: ChartView = { centerX: 12, centerY: -4, spanM: SPAN_MIN_M }
    expect(zoomAt(floor, { x: 0, y: 0 }, 1.3)).toEqual(floor)
    const ceiling: ChartView = { centerX: 12, centerY: -4, spanM: SPAN_MAX_M }
    expect(zoomAt(ceiling, { x: 0, y: 0 }, 1 / 1.3)).toEqual(ceiling)
    // Repeated presses walk into the clamp and stay finite.
    let zoomed: ChartView = { centerX: 0, centerY: 0, spanM: SPAN_MIN_M * 4 }
    for (let i = 0; i < 50; i++) {
      zoomed = zoomAt(zoomed, { x: 0, y: 0 }, 1.3)
    }
    expect(zoomed.spanM).toBe(SPAN_MIN_M)
    expect(Number.isFinite(zoomed.centerX)).toBe(true)
  })

  it('ignores junk instead of drawing a NaN frame', () => {
    expect(zoomAt(view, { x: 0, y: 0 }, 0)).toBe(view)
    expect(zoomAt(view, { x: 0, y: 0 }, -2)).toBe(view)
    expect(zoomAt(view, { x: 0, y: 0 }, Number.NaN)).toBe(view)
    expect(zoomAt(view, { x: Number.NaN, y: 0 }, 2)).toBe(view)
    const junk: ChartView = { centerX: 0, centerY: 0, spanM: Number.NaN }
    expect(zoomAt(junk, { x: 0, y: 0 }, 2)).toBe(junk)
  })
})

describe('panBy', () => {
  it('shifts the centre and leaves the zoom alone', () => {
    const view: ChartView = { centerX: 10, centerY: -20, spanM: 60 }
    expect(panBy(view, 5, -15)).toEqual({ centerX: 15, centerY: -35, spanM: 60 })
  })

  it('ignores a delta that would break the frame', () => {
    const view: ChartView = { centerX: 0, centerY: 0, spanM: 60 }
    expect(panBy(view, Number.NaN, 5)).toBe(view)
    expect(panBy(view, 5, Number.POSITIVE_INFINITY)).toBe(view)
    const junk: ChartView = { centerX: Number.NaN, centerY: 0, spanM: 60 }
    expect(panBy(junk, 5, 5)).toBe(junk)
  })

  it('preserves the span exactly across repeated pans, even at the clamps', () => {
    for (const spanM of [SPAN_MIN_M, 60, SPAN_MAX_M]) {
      const start: ChartView = { centerX: 1234.5, centerY: -678.25, spanM }
      let view = start
      for (let i = 0; i < 50; i++) view = panBy(view, 7.25, -3.5)
      expect(view.spanM).toBe(spanM) // panning is pure translation
      expect(view.centerX).not.toBe(start.centerX)
      expect(view.centerY).not.toBe(start.centerY)
    }
  })
})

describe('zoom controls', () => {
  it('enables each control until its clamp', () => {
    expect(canZoomIn({ centerX: 0, centerY: 0, spanM: SPAN_MIN_M })).toBe(false)
    expect(canZoomOut({ centerX: 0, centerY: 0, spanM: SPAN_MIN_M })).toBe(true)
    expect(canZoomIn({ centerX: 0, centerY: 0, spanM: SPAN_MAX_M })).toBe(true)
    expect(canZoomOut({ centerX: 0, centerY: 0, spanM: SPAN_MAX_M })).toBe(false)
    expect(canZoomIn({ centerX: 0, centerY: 0, spanM: 60 })).toBe(true)
    expect(canZoomOut({ centerX: 0, centerY: 0, spanM: 60 })).toBe(true)
  })
})

describe('linesOver', () => {
  it('returns every multiple of the step inside the visible range', () => {
    expect(linesOver(-15, 25, 10)).toEqual([-10, 0, 10, 20])
    expect(linesOver(0, 30, 10)).toEqual([0, 10, 20, 30])
    expect(linesOver(3, 7, 10)).toEqual([])
  })

  it('covers a panned frame from edge to edge', () => {
    // A view panned far from the mission: the grid still spans the frame.
    const frame = viewBounds({ centerX: 98765, centerY: -54321, spanM: 1000 }, 2)
    const step = gridStepM(1000)
    const lines = linesOver(frame.minX, frame.maxX, step)
    expect(lines.length).toBeGreaterThan(1)
    expect(lines[0]).toBeGreaterThanOrEqual(frame.minX)
    expect(lines[lines.length - 1]).toBeLessThanOrEqual(frame.maxX)
    expect(lines[0] ?? Number.POSITIVE_INFINITY).toBeLessThan(frame.minX + step)
    expect(lines[lines.length - 1] ?? Number.NEGATIVE_INFINITY).toBeGreaterThan(
      frame.maxX - step,
    )
  })

  it('refuses junk and caps a runaway step', () => {
    expect(linesOver(0, 100, 0)).toEqual([])
    expect(linesOver(0, 100, -5)).toEqual([])
    expect(linesOver(10, 0, 1)).toEqual([])
    expect(linesOver(0, 100, Number.NaN)).toEqual([])
    expect(linesOver(Number.NaN, 100, 1)).toEqual([])
    expect(linesOver(0, 1e9, 1)).toHaveLength(MAX_GRID_LINES)
  })
})

describe('grid density', () => {
  it('keeps every zoom level under about fifty cells across', () => {
    for (let i = 0; i <= 40; i++) {
      const spanM = SPAN_MIN_M * Math.pow(SPAN_MAX_M / SPAN_MIN_M, i / 40)
      const step = gridStepM(spanM)
      expect(Math.ceil(spanM / step)).toBeLessThanOrEqual(50)
      expect(linesOver(0, spanM, step).length).toBeLessThanOrEqual(MAX_GRID_LINES)
    }
  })
})

describe('grid steps follow the view', () => {
  it('derives grid, label, and scale steps from the view span, not the mission', () => {
    const mission = chartBounds(withGeometry([leg(0, 0), leg(40, 30)]))
    // The mission frame alone is 56 m at its largest side: 10 m cells.
    expect(gridStepM(Math.max(boundsWidth(mission), boundsHeight(mission)))).toBe(10)
    const stepFor = (view: ChartView): number => {
      const frame = viewBounds(view, 1)
      return gridStepM(Math.max(boundsWidth(frame), boundsHeight(frame)))
    }
    // The same mission zoomed in: 5 m cells follow the view, not the bounds.
    expect(stepFor({ centerX: 15, centerY: -20, spanM: 40 })).toBe(5)
    // Panned far away at a wide span: 5 km cells.
    expect(stepFor({ centerX: 4000, centerY: -4000, spanM: 40000 })).toBe(5000)
    // The label and scale steps hang off the same view span.
    expect(labelStepM(40)).toBe(25)
    expect(scaleBarLengthM(40)).toBe(10)
  })
})
