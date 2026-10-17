import { describe, expect, it } from 'vitest'
import { deriveTelemetry, summarizeRun } from './telemetry'
import type { FlightData, Obstacle, Vec3 } from '@/types/flight'

function fixture(obstacles: Obstacle[] = []): FlightData {
  const position: Vec3[] = [
    [0, 0, 0],
    [1, -1, -1],
    [2, -2, -2],
  ]
  return {
    meta: {
      scenario: 'unit',
      seed: 1,
      schema_version: '1.1',
      generated_utc: '2026-10-17T00:00:00Z',
      sample_count: 3,
      duration_s: 1,
      dt: 0.5,
      units: {},
    },
    time_s: [0, 0.5, 1],
    truth_position: position,
    truth_velocity: [
      [0, 0, 0],
      [1, 0, 0],
      [0, 1, 0],
    ],
    ekf_position: [
      [0, 0, 0],
      [1.5, -1, -1],
      [2, -2, -2],
    ],
    ekf_velocity: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    position_var: [
      [0.01, 0.01, 0.01],
      [0.04, 0.01, 0.01],
      [0.01, 0.01, 0.09],
    ],
    gyro_bias: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    accel_bias: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    wind_ned: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    command_thrust: [5, 10, 15],
    command_moment: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    e_ct: [],
    e_ct_times: [],
    e_ct_leg: [],
    waypoints_ned: [],
    home_ned: [0, 0, 0],
    obstacles,
    phase_timeline: [
      [0, 'idle'],
      [0, 'takeoff'],
      [0.5, 'cruise'],
    ],
    event_log: [],
  }
}

function seriesByKey(data: FlightData, key: string) {
  return deriveTelemetry(data).find((s) => s.key === key)
}

describe('deriveTelemetry', () => {
  it('derives altitude, speed, EKF error, and sigma from the samples', () => {
    const data = fixture()
    expect(seriesByKey(data, 'altitude')?.values).toEqual([0, 1, 2])
    expect(seriesByKey(data, 'speed')?.values).toEqual([0, 1, 1])
    const ekf = seriesByKey(data, 'ekf_error')?.values ?? []
    expect(ekf[0]).toBeCloseTo(0, 10)
    expect(ekf[1]).toBeCloseTo(0.5, 10)
    expect(ekf[2]).toBeCloseTo(0, 10)
    const sigma = seriesByKey(data, 'sigma3')?.values ?? []
    expect(sigma[0]).toBeCloseTo(0.3, 10)
    expect(sigma[1]).toBeCloseTo(0.6, 10)
    expect(sigma[2]).toBeCloseTo(0.9, 10)
  })

  it('flags a zero wind channel as flat', () => {
    expect(seriesByKey(fixture(), 'wind')?.flat).toBe(true)
  })

  it('omits clearance when the scenario has no keep-out zones', () => {
    expect(seriesByKey(fixture(), 'clearance')).toBeUndefined()
  })

  it('adds a clearance series when keep-out zones are present', () => {
    const obstacle: Obstacle = { id: 'o', center_ned: [0, 0, 0], radius_m: 1 }
    const clearance = seriesByKey(fixture([obstacle]), 'clearance')
    expect(clearance).toBeDefined()
    expect(clearance?.values[0]).toBeCloseTo(-1, 10)
    expect(clearance?.values[1]).toBeCloseTo(Math.sqrt(3) - 1, 10)
  })
})

describe('summarizeRun', () => {
  it('reports run-level headline numbers', () => {
    const summary = summarizeRun(fixture())
    expect(summary.durationS).toBe(1)
    expect(summary.sampleCount).toBe(3)
    expect(summary.rateHz).toBe(3)
    expect(summary.phaseCount).toBe(3)
    expect(summary.finalPhase).toBe('cruise')
    expect(summary.ekfMaxM).toBeCloseTo(0.5, 10)
    expect(summary.ekfRmsM).toBeCloseTo(Math.sqrt(0.25 / 3), 10)
    expect(summary.maxAltitudeM).toBe(2)
    expect(summary.maxSpeedMps).toBe(1)
    expect(summary.maxSigmaM).toBeCloseTo(0.9, 10)
    expect(summary.maxWindMps).toBe(0)
    expect(summary.minClearanceM).toBeNull()
    expect(summary.distanceFlownM).toBeCloseTo(2 * Math.sqrt(3), 10)
  })

  it('reports minimum clearance when zones are present', () => {
    const obstacle: Obstacle = { id: 'o', center_ned: [0, 0, 0], radius_m: 1 }
    expect(summarizeRun(fixture([obstacle])).minClearanceM).toBeCloseTo(-1, 10)
  })
})
