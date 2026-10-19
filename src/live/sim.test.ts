/**
 * LiveSim tests.
 *
 * The autopilot itself is no longer in TypeScript, so these tests do not fly
 * anything: they cover the seam. A scripted fake session stands in for the Rust
 * core and the tests assert what LiveSim does with it, which is exactly the part
 * that can break without the native tests noticing: the summary assembly, the
 * clearance computation, the phase list, and the delegation of stepping.
 *
 * The native side has its own tests (`cargo test -p talon-core` and
 * `cargo test -p talon-wasm`), including that stepping in chunks matches a bulk
 * run, so parity between the two paths is checked where the physics lives.
 */

import { describe, expect, it, vi } from 'vitest'
import { LiveSim } from './sim'
import type { CoreSession } from './engine'
import type { FlightData, FlightPhase } from '@/types/flight'
import type { LiveConfig } from './types'

const CONFIG: LiveConfig = {
  id: 'waypoints',
  label: 'Waypoint circuit',
  blurb: 'test',
  duration_s: 10,
  takeoff_altitude: 10,
  start_position_ned: [0, 0, 0],
  waypoints: [
    { position: [20, 0, -10], speed: 5, heading: 0, radius: 2 },
  ],
  obstacles: [],
  wind_sigma: [0, 0, 0],
  wind_steady_ned: [0, 0, 0],
  seed: 7,
  gps_position_noise: 0.3,
  gps_position_noise_vertical: 0.5,
  gps_velocity_noise: 0.05,
  cruise_hold_s: 2,
  auto_arm: true,
}

/** One truth / estimate pair per sample, at 3 m of error on the first sample. */
const DOCUMENT: FlightData = {
  meta: {
    scenario: 'waypoints',
    seed: 7,
    schema_version: '1.1',
    generated_utc: '2026-10-18T00:00:00Z',
    sample_count: 3,
    duration_s: 9.99,
    dt: 0.01,
    units: {},
  },
  time_s: [0, 0.01, 0.02],
  truth_position: [
    [0, 0, 0],
    [1, 0, -10],
    [2, 0, -10],
  ],
  truth_velocity: [
    [0, 0, 0],
    [5, 0, 0],
    [5, 0, 0],
  ],
  ekf_position: [
    [3, 4, 0],
    [1, 0, -10],
    [2, 0, -10],
  ],
  ekf_velocity: [
    [0, 0, 0],
    [5, 0, 0],
    [5, 0, 0],
  ],
  position_var: [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
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
  command_thrust: [0, 0, 0],
  command_moment: [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ],
  e_ct: [0],
  e_ct_times: [0],
  e_ct_leg: [0],
  waypoints_ned: [[20, 0, -10]],
  home_ned: [0, 0, 0],
  obstacles: [],
  phase_timeline: [
    [0, 'idle' as FlightPhase],
    [0.5, 'arm' as FlightPhase],
    [0.5, 'takeoff' as FlightPhase],
    [3, 'cruise' as FlightPhase],
  ],
  event_log: [],
}

const SUMMARY = {
  duration_s: 9.99,
  waypoint_count: 1,
  waypoints_reached: 1,
  mission_complete: true,
  gps_accepted: 90,
  gps_gated: 2,
  gps_skipped: 0,
  final_phase: 'idle',
  final_truth_altitude_m: 0,
  ekf_converged: true,
  ekf_diverged: false,
  steps_done: 2000,
  total_steps: 2000,
}

/** A scripted stand-in for the native session. */
class FakeSession implements CoreSession {
  static lastConfigJson = ''
  prepareCalls = 0
  stepCalls = 0
  freeCalls = 0
  advanceReturns = 0

  constructor(configJson: string) {
    FakeSession.lastConfigJson = configJson
  }

  prepare(): void {
    this.prepareCalls += 1
  }
  step(): boolean {
    this.stepCalls += 1
    return false
  }
  advance(steps: number): number {
    this.advanceReturns = steps
    return steps
  }
  run_to_end(): number {
    return 0
  }
  get progress(): number {
    return 0.5
  }
  get finished(): boolean {
    return false
  }
  get prepared(): boolean {
    return this.prepareCalls > 0
  }
  get steps_done(): number {
    return 1000
  }
  get total_steps(): number {
    return 2000
  }
  get time(): number {
    return 5
  }
  get duration_s(): number {
    return 10
  }
  get scenario_id(): string {
    return 'waypoints'
  }
  get battery_soc(): number {
    return 1
  }
  get sample_count(): number {
    return 42
  }
  get gps_accepted(): bigint {
    return BigInt(90)
  }
  get gps_gated(): bigint {
    return BigInt(2)
  }
  get gps_skipped(): bigint {
    return BigInt(0)
  }
  get ekf_converged(): boolean {
    return true
  }
  get ekf_diverged(): boolean {
    return false
  }
  /** Whatever the test set, so the passthrough can be observed. */
  samplesFromReturns: Float64Array = new Float64Array(0)
  samplesFromArgs: number[] = []
  phasesJson = '[[0,"idle"],[0,"arm"]]'
  phase(): string {
    return 'cruise'
  }
  phases_json(): string {
    return this.phasesJson
  }
  samples_from(from: number): Float64Array {
    this.samplesFromArgs.push(from)
    return this.samplesFromReturns
  }
  truth_position(): Float64Array {
    return Float64Array.from([1, 2, -10])
  }
  truth_velocity(): Float64Array {
    return Float64Array.from([3, 4, 0])
  }
  ekf_position(): Float64Array {
    return Float64Array.from([1, 2, -10])
  }
  altitude_m(): number {
    return 10
  }
  document_json(): string {
    return JSON.stringify(DOCUMENT)
  }
  summary_json(): string {
    return JSON.stringify(SUMMARY)
  }
  free(): void {
    this.freeCalls += 1
  }
}

function run(overrides?: Partial<FlightData>, summary?: Partial<typeof SUMMARY>) {
  const document = { ...DOCUMENT, ...overrides } as FlightData
  const session = new FakeSession(JSON.stringify(CONFIG))
  session.document_json = () => JSON.stringify(document)
  session.summary_json = () => JSON.stringify({ ...SUMMARY, ...summary })
  const sim = new LiveSim(session)
  return { sim, session }
}

describe('LiveSim', () => {
  it('sends the scenario to the native session as config JSON', () => {
    LiveSim.create(CONFIG, FakeSession)
    expect(JSON.parse(FakeSession.lastConfigJson)).toEqual(CONFIG)
  })

  it('delegates preparation and stepping to the native session', () => {
    const { sim, session } = run()
    sim.prepare()
    sim.advance()
    sim.advanceMany(64)
    expect(session.prepareCalls).toBe(1)
    expect(session.stepCalls).toBe(1)
    expect(session.advanceReturns).toBe(64)
  })

  it('exposes the live pose, speed, altitude, and phase while flying', () => {
    const { sim } = run()
    expect(sim.phase).toBe('cruise')
    expect(sim.altitudeM).toBe(10)
    expect(sim.truthPosition).toEqual([1, 2, -10])
    expect(sim.groundSpeed).toBeCloseTo(5, 12)
    expect(sim.progress).toBe(0.5)
    expect(sim.finished).toBe(false)
  })

  it('falls back to idle for an unknown phase label', () => {
    const session = new FakeSession('{}')
    session.phase = () => 'not-a-phase'
    const sim = new LiveSim(session)
    expect(sim.phase).toBe('idle')
  })

  it('carries the summary straight through from the native session', () => {
    const { sim } = run()
    const { summary } = sim.finish()
    expect(summary.duration_s).toBe(SUMMARY.duration_s)
    expect(summary.waypoint_count).toBe(1)
    expect(summary.waypoints_reached).toBe(1)
    expect(summary.mission_complete).toBe(true)
    expect(summary.gps_accepted).toBe(90)
    expect(summary.gps_gated).toBe(2)
  })

  it('computes the estimator error from the document arrays', () => {
    const { sim } = run()
    const { summary } = sim.finish()
    // The first sample is off by a 3-4-5 triangle, so 5 m: RMSE over three
    // samples with one such error is sqrt(25/3).
    expect(summary.ekf_position_rmse_m).toBeCloseTo(Math.sqrt(25 / 3), 12)
    // The final error is the error at the last sample, which has converged.
    expect(summary.final_position_error_m).toBe(0)
  })

  it('reports the terminal error at the last sample, not the worst one', () => {
    const { sim } = run({
      ekf_position: [
        [0, 0, 0],
        [1, 0, -10],
        [5, 4, -10],
      ],
    })
    const { summary } = sim.finish()
    expect(summary.final_position_error_m).toBeCloseTo(5, 12)
  })

  it('reports the phases in order without repeats', () => {
    const { sim } = run()
    const { summary } = sim.finish()
    expect(summary.phases).toEqual(['idle', 'arm', 'takeoff', 'cruise'])
  })

  it('reports no clearance when the scenario has no obstacles', () => {
    const { sim } = run()
    expect(sim.finish().summary.min_clearance_m).toBeNull()
  })

  it('measures clearance to the nearest obstacle surface', () => {
    const obstacles = [{ id: 'obs', center_ned: [2, 0, -10] as [number, number, number], radius_m: 1 }]
    const { sim } = run({ obstacles })
    // Samples sit at 0 m, 1 m, and 0 m from the center, so at 3 m, 1 m, 3 m
    // respectively, which is a clearance of -1 m at the closest sample.
    const clearance = sim.finish().summary.min_clearance_m
    expect(clearance).not.toBeNull()
    expect(clearance as number).toBeCloseTo(-1, 12)
  })

  it('parses the document once and returns the same result afterwards', () => {
    const { sim, session } = run()
    const spy = vi.spyOn(session, 'document_json')
    const first = sim.finish()
    const second = sim.finish()
    expect(first).toBe(second)
    expect(spy).toHaveBeenCalledTimes(1)
  })

  it('passes the document through unchanged apart from the summary', () => {
    const { sim } = run()
    const { data } = sim.finish()
    expect(data.meta.scenario).toBe('waypoints')
    expect(data.time_s).toEqual(DOCUMENT.time_s)
    expect(data.truth_position).toEqual(DOCUMENT.truth_position)
  })

  it('releases the native session on dispose', () => {
    const { sim, session } = run()
    sim.dispose()
    expect(session.freeCalls).toBe(1)
  })

  it('passes the sample cursor through to the native session', () => {
    const { sim, session } = run()
    session.samplesFromReturns = Float64Array.from([1, 2, 3])
    expect(sim.sampleCount).toBe(42)
    expect([...sim.drain(0)]).toEqual([1, 2, 3])
    expect(sim.drain(5).length).toBe(3)
    expect(session.samplesFromArgs).toEqual([0, 5])
  })

  it('parses the phase timeline and survives a malformed one', () => {
    const { sim, session } = run()
    expect(sim.phases()).toEqual([
      [0, 'idle'],
      [0, 'arm'],
    ])
    session.phasesJson = 'not json'
    expect(sim.phases()).toEqual([])
  })

  it('reads the GPS and estimator flags off the session', () => {
    const { sim } = run()
    // The session reports the counts as bigint; gpsCounts narrows them to numbers.
    expect(sim.gpsCounts()).toEqual([90, 2, 0])
    expect(sim.ekfFlags()).toEqual([true, false])
  })

  it('prepares before a one-shot run', () => {
    const { sim, session } = run()
    sim.runToCompletion()
    expect(session.prepareCalls).toBe(1)
  })
})
