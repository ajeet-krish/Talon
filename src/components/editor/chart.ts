/**
 * Pure display helpers for the mission editor (FlightLab Phase 8c). Chart
 * geometry and the small readouts the inspector and the chart share: no
 * React, no DOM, so every rule here is unit tested in `chart.test.ts`.
 *
 * The chart is a north-up planning grid in "chart space": x is east, y is
 * negative north, so north is up on screen and east is right, the way a
 * ground-station map reads. `chartBounds` frames the start, the route, and
 * the keep-out zones; the grid, label, and scale-bar steps are round meters
 * chosen from the span, so the chart never labels a sub-meter tick or draws
 * thousands of lines on a kilometer-scale route.
 */

import type { FaultSpec } from '@/types/generated/talon_core'
import type { MissionConfig } from '@/utils/editor'

// ---------------------------------------------------------------------------
// Chart bounds
// ---------------------------------------------------------------------------

/** Padding (m) added around the framed geometry so nodes never touch the edge. */
export const CHART_PADDING_M = 8

/** Framed span (m) when the mission has no route and no zones: a planning field. */
export const CHART_EMPTY_SPAN_M = 60

/** Chart-space frame: x is east, y is negative north, meters. */
export interface ChartBounds {
  minX: number
  maxX: number
  minY: number
  maxY: number
}

/**
 * The chart frame for a mission: every waypoint, every zone (counting its
 * radius), and the start point, padded by `CHART_PADDING_M`. With no route
 * and no zones the frame is a `CHART_EMPTY_SPAN_M` square centred on the
 * start, so an empty mission reads as a field to plan on rather than a
 * degenerate line. Returns chart-space coordinates (y is negative north).
 */
export function chartBounds(config: MissionConfig): ChartBounds {
  const [startNorth, startEast] = config.start_position_ned
  let minNorth = startNorth
  let maxNorth = startNorth
  let minEast = startEast
  let maxEast = startEast
  let hasGeometry = false

  for (const wp of config.waypoints) {
    minNorth = Math.min(minNorth, wp.position[0])
    maxNorth = Math.max(maxNorth, wp.position[0])
    minEast = Math.min(minEast, wp.position[1])
    maxEast = Math.max(maxEast, wp.position[1])
    hasGeometry = true
  }
  for (const ob of config.obstacles) {
    minNorth = Math.min(minNorth, ob.center_ned[0] - ob.radius_m)
    maxNorth = Math.max(maxNorth, ob.center_ned[0] + ob.radius_m)
    minEast = Math.min(minEast, ob.center_ned[1] - ob.radius_m)
    maxEast = Math.max(maxEast, ob.center_ned[1] + ob.radius_m)
    hasGeometry = true
  }

  if (!hasGeometry) {
    // Frame a 60 m square on the start, padding included.
    const half = CHART_EMPTY_SPAN_M / 2 - CHART_PADDING_M
    minNorth = startNorth - half
    maxNorth = startNorth + half
    minEast = startEast - half
    maxEast = startEast + half
  }

  minNorth -= CHART_PADDING_M
  maxNorth += CHART_PADDING_M
  minEast -= CHART_PADDING_M
  maxEast += CHART_PADDING_M

  // Chart space flips north: y = -north.
  return { minX: minEast, maxX: maxEast, minY: -maxNorth, maxY: -minNorth }
}

/** Width of a chart frame (m). */
export function boundsWidth(bounds: ChartBounds): number {
  return bounds.maxX - bounds.minX
}

/** Height of a chart frame (m). */
export function boundsHeight(bounds: ChartBounds): number {
  return bounds.maxY - bounds.minY
}

/** The `viewBox` string for a frame, for `preserveAspectRatio="xMidYMid meet"`. */
export function boundsViewBox(bounds: ChartBounds): string {
  return `${bounds.minX} ${bounds.minY} ${boundsWidth(bounds)} ${boundsHeight(bounds)}`
}

// ---------------------------------------------------------------------------
// Round steps (grid, labels, scale bar)
// ---------------------------------------------------------------------------

const GRID_CANDIDATES = [5, 10, 20, 25, 50, 100, 200, 250, 500, 1000, 2000, 5000] as const

/**
 * Grid spacing (m) for a span: the smallest round step that keeps the chart
 * to about ten lines per axis. A 60 m frame grids every 10 m; a 2 km frame
 * grids every 200 m.
 */
export function gridStepM(span: number): number {
  for (const step of GRID_CANDIDATES) {
    if (span / step <= 10) return step
  }
  return GRID_CANDIDATES[GRID_CANDIDATES.length - 1] ?? 5000
}

/**
 * Label spacing (m) for a span: five grid steps, so a 10 m grid labels every
 * 50 m and a 200 m grid labels every kilometer. Labels always land on a grid
 * line.
 */
export function labelStepM(span: number): number {
  return gridStepM(span) * 5
}

const SCALE_CANDIDATES = [1, 2, 5, 10, 20, 25, 50, 100, 200, 250, 500, 1000, 2000, 5000] as const

/**
 * Scale-bar length (m) for a span: the longest round length that fits in a
 * quarter of the frame, so the bar is readable without crowding the chart.
 */
export function scaleBarLengthM(span: number): number {
  const target = span / 4
  let chosen: number = SCALE_CANDIDATES[0] ?? 1
  for (const candidate of SCALE_CANDIDATES) {
    if (candidate <= target) chosen = candidate
  }
  return chosen
}

// ---------------------------------------------------------------------------
// Wind readouts
// ---------------------------------------------------------------------------

/** Wind speed (m/s) from its steady NED north and east components. */
export function windSpeedMps(north: number, east: number): number {
  return Math.hypot(north, east)
}

/**
 * Meteorological direction (deg, clockwise from north) the wind blows FROM,
 * or null in calm air. The steady wind is stored as a velocity NED triple,
 * so the source bearing is the direction opposite the vector.
 */
export function windFromDirectionDeg(north: number, east: number): number | null {
  if (windSpeedMps(north, east) < 1e-6) return null
  const deg = (Math.atan2(-east, -north) * 180) / Math.PI
  return (deg + 360) % 360
}

const COMPASS_POINTS = [
  'N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
  'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW',
] as const

/** Nearest 16-point compass label for a bearing in degrees. */
export function compassPoint(bearingDeg: number): string {
  const index = Math.round((((bearingDeg % 360) + 360) % 360) / 22.5) % 16
  return COMPASS_POINTS[index] ?? 'N'
}

/**
 * The steady wind as a readout line: speed and source bearing, or "CALM" in
 * still air. Example: `3.0 m/s FROM 180° (S)`.
 */
export function formatWindSummary(north: number, east: number): string {
  const bearing = windFromDirectionDeg(north, east)
  if (bearing === null) return 'CALM'
  return `${windSpeedMps(north, east).toFixed(1)} m/s FROM ${Math.round(bearing)}° (${compassPoint(bearing)})`
}

// ---------------------------------------------------------------------------
// Fault and time labels
// ---------------------------------------------------------------------------

/** One fault as a readable line, used by the timeline strip and the list. */
export function faultLabel(fault: FaultSpec): string {
  switch (fault.kind) {
    case 'gps_outage':
      return `GPS outage at ${fault.at_s.toFixed(1)} s for ${fault.duration_s.toFixed(1)} s`
    case 'ekf_divergence':
      return `EKF divergence at ${fault.at_s.toFixed(1)} s (${fault.mode})`
    case 'battery_soc':
      return `Battery forced to ${fault.soc.toFixed(2)} at ${fault.at_s.toFixed(1)} s`
    case 'motor_fault':
      return `Motor fault at ${fault.at_s.toFixed(1)} s`
    case 'emergency':
      return `Emergency at ${fault.at_s.toFixed(1)} s`
    case 'rtl':
      return `RTL at ${fault.at_s.toFixed(1)} s`
    default:
      // Unreachable for the generated kinds; keep the function total anyway.
      return 'Fault'
  }
}

/** Seconds as `m:ss`, the flight-time estimate format (65 s -> `1:05`). */
export function formatMinutesSeconds(seconds: number): string {
  const total = Math.max(0, Math.round(seconds))
  const minutes = Math.floor(total / 60)
  const rest = total % 60
  return `${minutes}:${String(rest).padStart(2, '0')}`
}
