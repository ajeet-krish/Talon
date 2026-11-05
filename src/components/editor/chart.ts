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
 *
 * Phase 8d-C adds the viewport: `ChartView` is the user's zoom and pan,
 * held outside the undo history, `viewBounds` turns it into the visible
 * rectangle the grid and chrome are drawn over, and `viewFit` is the only
 * path back to framing the whole mission (mount, document identity change,
 * the FIT button).
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
// Viewport (zoom and pan)
// ---------------------------------------------------------------------------

/**
 * The chart viewport: where the view looks and how much it shows. `spanM`
 * is the visible rectangle's larger side in meters, so one number is the
 * zoom level and the centre pair is the pan. The viewport lives in
 * component state outside the undo history: editing, undo, and redo never
 * touch it, which is what keeps the frame steady while a mission is
 * reworked.
 */
export interface ChartView {
  /** Chart-space x (east) of the view centre, meters. */
  centerX: number
  /** Chart-space y (negative north) of the view centre, meters. */
  centerY: number
  /** Visible span along the larger screen side, meters. */
  spanM: number
}

/** A chart-space point: the anchor a zoom keeps in place under the cursor. */
export interface ChartPoint {
  x: number
  y: number
}

/** Smallest visible span (m): fully zoomed in, still never degenerate. */
export const SPAN_MIN_M = 10

/**
 * Largest visible span (m): the zoom-out bound. It sits at the top of the
 * round-meter grid's ladder (a 50 km frame still grids every 5 km, ten
 * cells across), so a frame never needs a step the ladder lacks; a mission
 * wider than this centres at the cap and panning reaches the rest.
 */
export const SPAN_MAX_M = 50000

/**
 * Clamp a span into the zoom range. NaN input (a corrupted view) falls back
 * to the minimum; infinities clamp naturally, so no caller can draw an
 * unbounded frame.
 */
export function clampSpan(spanM: number): number {
  if (Number.isNaN(spanM)) return SPAN_MIN_M
  return Math.min(Math.max(spanM, SPAN_MIN_M), SPAN_MAX_M)
}

/**
 * The visible rectangle for a view at an element aspect (width / height).
 * `spanM` is the larger side, so a landscape element shows `spanM` wide and
 * a portrait element `spanM` tall, with the centre fixed. The span is
 * clamped and the centre falls back to the origin when it is not finite, so
 * the rectangle (and the `viewBox` drawn from it) is never zero, negative,
 * or NaN, whatever state a view arrives in.
 */
export function viewBounds(view: ChartView, aspect: number): ChartBounds {
  const spanM = clampSpan(view.spanM)
  const safeAspect = Number.isFinite(aspect) && aspect > 0 ? aspect : 1
  const spanX = safeAspect >= 1 ? spanM : spanM * safeAspect
  const spanY = safeAspect >= 1 ? spanM / safeAspect : spanM
  const centerX = Number.isFinite(view.centerX) ? view.centerX : 0
  const centerY = Number.isFinite(view.centerY) ? view.centerY : 0
  return {
    minX: centerX - spanX / 2,
    maxX: centerX + spanX / 2,
    minY: centerY - spanY / 2,
    maxY: centerY + spanY / 2,
  }
}

/**
 * The view that frames a bounds rectangle at an element aspect: centred on
 * it, at the smallest span that shows all of it on both axes (so nothing is
 * clipped), clamped into the zoom range. This is the FIT target, reached
 * only on mount, on a document identity change (template load, import,
 * CLEAR), and from the FIT button, never from an edit.
 */
export function viewFit(bounds: ChartBounds, aspect: number): ChartView {
  const safeAspect = Number.isFinite(aspect) && aspect > 0 ? aspect : 1
  const width = boundsWidth(bounds)
  const height = boundsHeight(bounds)
  const needed =
    safeAspect >= 1 ? Math.max(width, height * safeAspect) : Math.max(width / safeAspect, height)
  const centerX = (bounds.minX + bounds.maxX) / 2
  const centerY = (bounds.minY + bounds.maxY) / 2
  return {
    centerX: Number.isFinite(centerX) ? centerX : 0,
    centerY: Number.isFinite(centerY) ? centerY : 0,
    spanM: clampSpan(needed),
  }
}

/**
 * Zoom about an anchor point: the span shrinks by `factor` (above 1 zooms
 * in, below 1 zooms out) while the anchor stays fixed, which is what puts
 * the world point under the cursor in place on a wheel tick. A zoom that
 * would pass a span clamp lands exactly on it, so at a bound this is a
 * no-op (the controls also disable there); junk factors, anchors, and views
 * are ignored rather than allowed to draw a NaN frame.
 */
export function zoomAt(view: ChartView, anchor: ChartPoint, factor: number): ChartView {
  if (!Number.isFinite(factor) || factor <= 0) return view
  if (!Number.isFinite(view.centerX) || !Number.isFinite(view.centerY)) return view
  if (!Number.isFinite(view.spanM) || view.spanM <= 0) return view
  if (!Number.isFinite(anchor.x) || !Number.isFinite(anchor.y)) return view
  const current = clampSpan(view.spanM)
  const spanM = clampSpan(current / factor)
  const scale = spanM / current
  const centerX = anchor.x + (view.centerX - anchor.x) * scale
  const centerY = anchor.y + (view.centerY - anchor.y) * scale
  if (!Number.isFinite(centerX) || !Number.isFinite(centerY)) return view
  return { centerX, centerY, spanM }
}

/**
 * Shift the view centre by a world-space delta, meters. Pan is pure
 * translation: the span (the zoom level) never changes, and a delta that
 * would break the frame is ignored.
 */
export function panBy(view: ChartView, dxM: number, dyM: number): ChartView {
  if (!Number.isFinite(dxM) || !Number.isFinite(dyM)) return view
  const centerX = view.centerX + dxM
  const centerY = view.centerY + dyM
  if (!Number.isFinite(centerX) || !Number.isFinite(centerY)) return view
  return { ...view, centerX, centerY }
}

/** True when a zoom in would move the view (it is above the floor). */
export function canZoomIn(view: ChartView): boolean {
  return clampSpan(view.spanM) > SPAN_MIN_M
}

/** True when a zoom out would move the view (it is below the ceiling). */
export function canZoomOut(view: ChartView): boolean {
  return clampSpan(view.spanM) < SPAN_MAX_M
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

/**
 * Upper bound on lines one grid or label pass may emit, so a bad step can
 * never stall a render (the round steps keep the real count near ten).
 */
export const MAX_GRID_LINES = 200

/**
 * The multiples of `step` inside the visible range [min, max], inclusive:
 * the grid and label positions for the frame the viewport is showing, so
 * panning always draws over what is on screen and never runs off the grid's
 * edge. Positions are computed by index rather than repeated addition, which
 * cannot stall or drift at large coordinates; junk ranges and steps return
 * no lines, and the count is capped at `MAX_GRID_LINES`.
 */
export function linesOver(min: number, max: number, step: number): number[] {
  if (
    !Number.isFinite(min) ||
    !Number.isFinite(max) ||
    !Number.isFinite(step) ||
    step <= 0 ||
    max < min
  ) {
    return []
  }
  const first = Math.ceil(min / step)
  const count = Math.min(Math.max(Math.floor((max - first * step) / step) + 1, 0), MAX_GRID_LINES)
  const lines: number[] = []
  for (let i = 0; i < count; i++) {
    lines.push((first + i) * step)
  }
  return lines
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
