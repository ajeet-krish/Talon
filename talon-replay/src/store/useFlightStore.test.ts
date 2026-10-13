import { beforeEach, describe, expect, it } from 'vitest'
import { useFlightStore } from './useFlightStore'
import type { FlightData, ScenarioId } from '@/types/flight'

function fakeFlightData(durationS: number): FlightData {
  return {
    meta: {
      scenario: 'gps_outage',
      seed: 1,
      schema_version: '1.0',
      generated_utc: '2026-10-13T00:00:00Z',
      sample_count: 3,
      duration_s: durationS,
      dt: 0.5,
      units: {},
    },
    time_s: [0, 0.5, 1],
    truth_position: [
      [0, 0, 0],
      [1, -1, 0],
      [2, -2, 0],
    ],
    truth_velocity: [
      [0, 0, 0],
      [1, -1, 0],
      [2, -2, 0],
    ],
    ekf_position: [
      [0, 0, 0],
      [1, -1, 0],
      [2, -2, 0],
    ],
    ekf_velocity: [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ],
    position_var: [
      [0.1, 0.1, 0.4],
      [0.1, 0.1, 0.4],
      [0.1, 0.1, 0.4],
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
    command_thrust: [5, 12, 17],
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
    phase_timeline: [
      [0, 'idle'],
      [0, 'takeoff'],
      [0.5, 'cruise'],
    ],
    event_log: [],
  }
}

const id: ScenarioId = 'gps_outage'

beforeEach(() => {
  useFlightStore.setState({
    scenarios: new Map([[id, fakeFlightData(10)]]),
    activeScenario: id,
    loading: false,
    error: null,
    currentTime: 4,
    isPlaying: false,
    playbackSpeed: 1,
    toggles: { trail: true, ellipsoid: true, wind: true, waypoints: true },
  })
})

describe('setTime', () => {
  it('sets a time inside the duration', () => {
    useFlightStore.getState().setTime(3.25)
    expect(useFlightStore.getState().currentTime).toBe(3.25)
  })

  it('clamps above the scenario duration', () => {
    useFlightStore.getState().setTime(999)
    expect(useFlightStore.getState().currentTime).toBe(10)
  })

  it('clamps below zero', () => {
    useFlightStore.getState().setTime(-5)
    expect(useFlightStore.getState().currentTime).toBe(0)
  })
})

describe('togglePlay', () => {
  it('flips the playing flag', () => {
    expect(useFlightStore.getState().isPlaying).toBe(false)
    useFlightStore.getState().togglePlay()
    expect(useFlightStore.getState().isPlaying).toBe(true)
    useFlightStore.getState().togglePlay()
    expect(useFlightStore.getState().isPlaying).toBe(false)
  })
})

describe('setToggle', () => {
  it('updates one toggle and leaves the rest', () => {
    useFlightStore.getState().setToggle('wind', false)
    expect(useFlightStore.getState().toggles).toEqual({
      trail: true,
      ellipsoid: true,
      wind: false,
      waypoints: true,
    })
  })
})

describe('tick', () => {
  it('advances time and clamps at the end, pausing playback', () => {
    useFlightStore.setState({ currentTime: 9.6, isPlaying: true })
    useFlightStore.getState().tick(0.5)
    expect(useFlightStore.getState().currentTime).toBe(10)
    expect(useFlightStore.getState().isPlaying).toBe(false)
  })
})
