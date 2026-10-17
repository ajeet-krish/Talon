import type { FlightData, FlightPhase, Vec3 } from '@/types/flight'
import { surfaceClearance } from './clearance'

/**
 * Telemetry derivation for the Telemetry and Metrics views. Pure functions
 * over an already-loaded FlightData so they are unit-testable and so the
 * charts do not re-derive per frame (the view memoizes on the payload).
 */

/** Chart palette, kept separate from the Wong flight-phase palette. */
export const CHART_COLORS = {
  cyan: '#26b8d9',
  amber: '#d9862b',
  violet: '#9b7be0',
  teal: '#3fb6a8',
  rose: '#e2506a',
  steel: '#8b9bb4',
} as const

export interface TelemetrySeries {
  key: string
  label: string
  unit: string
  color: string
  times: number[]
  values: number[]
  /**
   * True when the series carries no meaningful signal over the run (a flat
   * zero line), so the view can drop it instead of drawing an empty axis.
   */
  flat: boolean
}

function magnitude(v: Vec3): number {
  return Math.hypot(v[0], v[1], v[2])
}

function distance(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
}

/**
 * Altitude is the negated NED down component. Negating a zero down value
 * yields negative zero, which would print as "-0.0" in the readouts, so
 * normalize it back to a plain zero.
 */
function altitudeOf(pos: Vec3): number {
  const up = -pos[2]
  return up === 0 ? 0 : up
}

/**
 * Derive the chart series for a scenario in a single pass over the samples:
 * altitude, ground speed, EKF position error, wind speed, commanded thrust,
 * 3-sigma position, and (when the scenario ships keep-out zones) surface
 * clearance to the nearest zone.
 */
export function deriveTelemetry(data: FlightData): TelemetrySeries[] {
  const n = data.time_s.length
  const times = data.time_s
  const altitude: number[] = new Array(n)
  const speed: number[] = new Array(n)
  const ekfError: number[] = new Array(n)
  const wind: number[] = new Array(n)
  const sigma3: number[] = new Array(n)
  const clearance: number[] = new Array(n)
  const hasObstacles = data.obstacles.length > 0

  for (let i = 0; i < n; i++) {
    const pos = data.truth_position[i] ?? [0, 0, 0]
    altitude[i] = altitudeOf(pos)
    speed[i] = magnitude(data.truth_velocity[i] ?? [0, 0, 0])
    ekfError[i] = distance(pos, data.ekf_position[i] ?? [0, 0, 0])
    wind[i] = magnitude(data.wind_ned[i] ?? [0, 0, 0])
    const variance = data.position_var[i] ?? [0, 0, 0]
    sigma3[i] = 3 * Math.sqrt(Math.max(variance[0], variance[1], variance[2], 0))
    if (hasObstacles) clearance[i] = surfaceClearance(pos, data.obstacles)
  }

  const series: TelemetrySeries[] = [
    {
      key: 'altitude',
      label: 'Altitude',
      unit: 'm',
      color: CHART_COLORS.cyan,
      times,
      values: altitude,
      flat: false,
    },
    {
      key: 'speed',
      label: 'Ground speed',
      unit: 'm/s',
      color: CHART_COLORS.teal,
      times,
      values: speed,
      flat: false,
    },
    {
      key: 'ekf_error',
      label: 'EKF position error',
      unit: 'm',
      color: CHART_COLORS.violet,
      times,
      values: ekfError,
      flat: false,
    },
    {
      key: 'wind',
      label: 'Wind speed',
      unit: 'm/s',
      color: CHART_COLORS.steel,
      times,
      values: wind,
      flat: maxOf(wind) < 0.5,
    },
    {
      key: 'thrust',
      label: 'Commanded thrust',
      unit: 'N',
      color: CHART_COLORS.amber,
      times,
      values: data.command_thrust,
      flat: false,
    },
    {
      key: 'sigma3',
      label: 'EKF 3-sigma position',
      unit: 'm',
      color: CHART_COLORS.rose,
      times,
      values: sigma3,
      flat: false,
    },
  ]

  if (hasObstacles) {
    series.push({
      key: 'clearance',
      label: 'Obstacle clearance',
      unit: 'm',
      color: CHART_COLORS.amber,
      times,
      values: clearance,
      flat: false,
    })
  }

  return series
}

function maxOf(values: readonly number[]): number {
  let max = 0
  for (const v of values) if (v > max) max = v
  return max
}

export interface RunSummary {
  durationS: number
  sampleCount: number
  rateHz: number
  phaseCount: number
  finalPhase: FlightPhase
  ekfRmsM: number
  ekfMaxM: number
  maxAltitudeM: number
  maxSpeedMps: number
  maxSigmaM: number
  maxWindMps: number
  minClearanceM: number | null
  distanceFlownM: number
}

/**
 * Headline numbers for the Metrics view, computed from the recorded samples
 * so the web report is derived from the same telemetry the charts draw.
 */
export function summarizeRun(data: FlightData): RunSummary {
  const n = data.time_s.length
  let sumSqEkf = 0
  let maxEkf = 0
  let maxAltitude = 0
  let maxSpeed = 0
  let maxSigma = 0
  let maxWind = 0
  let minClearance = Infinity
  let distanceFlown = 0
  let prev: Vec3 | null = null

  for (let i = 0; i < n; i++) {
    const pos = data.truth_position[i] ?? [0, 0, 0]
    const err = distance(pos, data.ekf_position[i] ?? [0, 0, 0])
    sumSqEkf += err * err
    if (err > maxEkf) maxEkf = err
    const altitude = altitudeOf(pos)
    if (altitude > maxAltitude) maxAltitude = altitude
    const speed = magnitude(data.truth_velocity[i] ?? [0, 0, 0])
    if (speed > maxSpeed) maxSpeed = speed
    const variance = data.position_var[i] ?? [0, 0, 0]
    const sigma =
      3 * Math.sqrt(Math.max(variance[0], variance[1], variance[2], 0))
    if (sigma > maxSigma) maxSigma = sigma
    const wind = magnitude(data.wind_ned[i] ?? [0, 0, 0])
    if (wind > maxWind) maxWind = wind
    if (data.obstacles.length > 0) {
      minClearance = Math.min(minClearance, surfaceClearance(pos, data.obstacles))
    }
    if (prev) distanceFlown += distance(pos, prev)
    prev = pos
  }

  const phases = new Set<FlightPhase>()
  for (const [, phase] of data.phase_timeline) phases.add(phase)
  const durationS = data.meta.duration_s
  const last = data.phase_timeline[data.phase_timeline.length - 1]
  return {
    durationS,
    sampleCount: data.meta.sample_count,
    rateHz: durationS > 0 ? data.meta.sample_count / durationS : 0,
    phaseCount: phases.size,
    finalPhase: last ? last[1] : 'idle',
    ekfRmsM: n > 0 ? Math.sqrt(sumSqEkf / n) : 0,
    ekfMaxM: maxEkf,
    maxAltitudeM: maxAltitude,
    maxSpeedMps: maxSpeed,
    maxSigmaM: maxSigma,
    maxWindMps: maxWind,
    minClearanceM: data.obstacles.length > 0 ? minClearance : null,
    distanceFlownM: distanceFlown,
  }
}
