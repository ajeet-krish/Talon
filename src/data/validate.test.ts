/**
 * Input-hardening tests for the runtime validators.
 *
 * Two things are easy to get wrong and expensive to notice: the exact error
 * path (an operator needs `configs[2].duration_s must be finite`, not "invalid
 * config"), and the shipped data itself (a validator stricter than the
 * exporter turns every load into a failure). So every rejection here asserts
 * its path, and the committed configs and recordings are validated as
 * fixtures rather than fixtures standing in for them.
 */

import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'
import { SCENARIO_IDS } from '@/store/useFlightStore'
import { validateFlightData, validateLiveConfigs } from './validate'

// ---------------------------------------------------------------------------
// Fixtures: one valid config, one valid flight document, as plain JSON values.
// ---------------------------------------------------------------------------

function validConfig(): Record<string, unknown> {
  return {
    id: 'hover',
    label: 'Hover',
    blurb: 'Climb to 10 m and hold.',
    duration_s: 60,
    takeoff_altitude: 10,
    start_position_ned: [0, 0, 0],
    waypoints: [{ position: [0, 0, -10], speed: 5, heading: 0, radius: 2 }],
    obstacles: [{ id: 'obs_north', center_ned: [10, 0, -5], radius_m: 3 }],
    wind_sigma: [1, 1, 0.5],
    wind_steady_ned: [2, 0, 0],
    seed: 20261012,
    gps_position_noise: 0.3,
    gps_position_noise_vertical: 0.5,
    gps_velocity_noise: 0.05,
    cruise_hold_s: 2,
    auto_arm: true,
  }
}

function configsFile(...configs: Record<string, unknown>[]): unknown {
  return { generated_utc: '2026-10-19T00:00:00Z', configs }
}

function rows(count: number, value: (i: number) => unknown): unknown[] {
  return Array.from({ length: count }, (_, i) => value(i))
}

function validFlight(): Record<string, unknown> {
  return {
    meta: {
      scenario: 'test',
      seed: 1,
      schema_version: '1.1',
      generated_utc: '2026-10-19T00:00:00Z',
      sample_count: 3,
      duration_s: 0.02,
      dt: 0.01,
      units: { time: 's', position: 'm NED' },
    },
    time_s: [0, 0.01, 0.02],
    truth_position: [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ],
    truth_velocity: [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ],
    ekf_position: [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
    ],
    ekf_velocity: [
      [0, 0, 0],
      [1, 0, 0],
      [2, 0, 0],
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
    e_ct: [0.5, 0.25],
    e_ct_times: [0, 0.01],
    e_ct_leg: [0, 0],
    waypoints_ned: [[40, 0, -10]],
    home_ned: [0, 0, 0],
    obstacles: [{ id: 'obs_north', center_ned: [10, 0, -5], radius_m: 3 }],
    phase_timeline: [
      [0, 'idle'],
      [0.01, 'cruise'],
    ],
    event_log: [{ time: 0, event: 'arm', from_phase: 'idle', to_phase: 'arm', guard_ok: true }],
  }
}

/** The flight document with `key` deleted, for the schema 1.0 shape. */
function withoutKey(doc: Record<string, unknown>, key: string): Record<string, unknown> {
  const copy = JSON.parse(JSON.stringify(doc)) as Record<string, unknown>
  delete copy[key]
  return copy
}

// ---------------------------------------------------------------------------
// validateLiveConfigs
// ---------------------------------------------------------------------------

describe('validateLiveConfigs', () => {
  it('accepts every committed live config', () => {
    // The shipped file is the fixture: a validator stricter than the exporter
    // would fail every load, which no unit test with a hand-written config
    // would catch.
    const file = JSON.parse(readFileSync('public/data/live/configs.json', 'utf8')) as unknown
    const configs = validateLiveConfigs(file)
    expect(configs.map((c) => c.id)).toEqual([
      'hover',
      'waypoints',
      'obstacle_field',
      'avoidance',
      'wind',
    ])
    expect(configs.find((c) => c.id === 'avoidance')?.avoidance?.enabled).toBe(true)
  })

  it('accepts a valid config and returns it as LiveConfig', () => {
    const [cfg] = validateLiveConfigs(configsFile(validConfig()))
    expect(cfg?.id).toBe('hover')
    expect(cfg?.waypoints).toHaveLength(1)
    expect(cfg?.obstacles[0]?.radius_m).toBe(3)
  })

  it('requires an object with a configs array', () => {
    expect(() => validateLiveConfigs(null)).toThrow('value must be an object')
    expect(() => validateLiveConfigs([])).toThrow('value must be an object')
    expect(() => validateLiveConfigs({ generated_utc: 'x' })).toThrow(
      'configs must be an array',
    )
    expect(() => validateLiveConfigs({ configs: {} })).toThrow('configs must be an array')
  })

  it('names the first config index in the error path', () => {
    const third = validConfig()
    third.duration_s = Number.NaN
    expect(() => validateLiveConfigs(configsFile(validConfig(), validConfig(), third))).toThrow(
      'configs[2].duration_s must be finite',
    )
  })

  it('rejects a non-finite duration', () => {
    for (const duration of [Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]) {
      const cfg = validConfig()
      cfg.duration_s = duration
      expect(() => validateLiveConfigs(configsFile(cfg))).toThrow(
        'configs[0].duration_s must be finite',
      )
    }
  })

  it('rejects a duration outside (0, 900]', () => {
    for (const duration of [0, -1, 901]) {
      const cfg = validConfig()
      cfg.duration_s = duration
      expect(() => validateLiveConfigs(configsFile(cfg))).toThrow(
        'configs[0].duration_s must be in (0, 900]',
      )
    }
  })

  it('rejects a takeoff altitude outside (0, 1000]', () => {
    const nan = validConfig()
    nan.takeoff_altitude = Number.NaN
    expect(() => validateLiveConfigs(configsFile(nan))).toThrow(
      'configs[0].takeoff_altitude must be finite',
    )
    for (const altitude of [0, 1000.5]) {
      const cfg = validConfig()
      cfg.takeoff_altitude = altitude
      expect(() => validateLiveConfigs(configsFile(cfg))).toThrow(
        'configs[0].takeoff_altitude must be in (0, 1000]',
      )
    }
  })

  it('requires a string id and label', () => {
    const cfg = validConfig()
    cfg.id = 7
    expect(() => validateLiveConfigs(configsFile(cfg))).toThrow('configs[0].id must be a string')
    const other = validConfig()
    other.label = null
    expect(() => validateLiveConfigs(configsFile(other))).toThrow(
      'configs[0].label must be a string',
    )
  })

  it('rejects a start position that is not a finite triple', () => {
    const short = validConfig()
    short.start_position_ned = [0, 0]
    expect(() => validateLiveConfigs(configsFile(short))).toThrow(
      'configs[0].start_position_ned must have length 3 (got 2)',
    )
    const dirty = validConfig()
    dirty.start_position_ned = [0, Number.NaN, 0]
    expect(() => validateLiveConfigs(configsFile(dirty))).toThrow(
      'configs[0].start_position_ned[1] must be finite',
    )
  })

  it('rejects a waypoint with a non-finite or negative field', () => {
    const speed = validConfig()
    speed.waypoints = [{ position: [0, 0, -10], speed: Number.NaN, heading: 0, radius: 2 }]
    expect(() => validateLiveConfigs(configsFile(speed))).toThrow(
      'configs[0].waypoints[0].speed must be finite',
    )
    const radius = validConfig()
    radius.waypoints = [{ position: [0, 0, -10], speed: 5, heading: 0, radius: -1 }]
    expect(() => validateLiveConfigs(configsFile(radius))).toThrow(
      'configs[0].waypoints[0].radius must be >= 0',
    )
    const heading = validConfig()
    heading.waypoints = [{ position: [0, 0, -10], speed: 5, heading: Number.POSITIVE_INFINITY, radius: 2 }]
    expect(() => validateLiveConfigs(configsFile(heading))).toThrow(
      'configs[0].waypoints[0].heading must be finite',
    )
    const position = validConfig()
    position.waypoints = [{ position: [0, -10], speed: 5, heading: 0, radius: 2 }]
    expect(() => validateLiveConfigs(configsFile(position))).toThrow(
      'configs[0].waypoints[0].position must have length 3 (got 2)',
    )
  })

  it('caps waypoints and obstacles at 512 entries', () => {
    const waypoint = { position: [0, 0, -10], speed: 5, heading: 0, radius: 2 }
    const ok = validConfig()
    ok.waypoints = rows(512, () => waypoint)
    expect(() => validateLiveConfigs(configsFile(ok))).not.toThrow()

    const tooMany = validConfig()
    tooMany.waypoints = rows(513, () => waypoint)
    expect(() => validateLiveConfigs(configsFile(tooMany))).toThrow(
      'configs[0].waypoints must have at most 512 entries (got 513)',
    )

    const obstacle = { id: 'o', center_ned: [1, 2, -3], radius_m: 4 }
    const many = validConfig()
    many.obstacles = rows(513, () => obstacle)
    expect(() => validateLiveConfigs(configsFile(many))).toThrow(
      'configs[0].obstacles must have at most 512 entries (got 513)',
    )
  })

  it('rejects a keep-out zone with a bad id, centre, or radius', () => {
    const id = validConfig()
    id.obstacles = [{ id: 5, center_ned: [1, 2, -3], radius_m: 4 }]
    expect(() => validateLiveConfigs(configsFile(id))).toThrow(
      'configs[0].obstacles[0].id must be a string',
    )
    const centre = validConfig()
    centre.obstacles = [{ id: 'o', center_ned: [1, Number.POSITIVE_INFINITY, -3], radius_m: 4 }]
    expect(() => validateLiveConfigs(configsFile(centre))).toThrow(
      'configs[0].obstacles[0].center_ned[1] must be finite',
    )
    const radius = validConfig()
    radius.obstacles = [{ id: 'o', center_ned: [1, 2, -3], radius_m: -0.5 }]
    expect(() => validateLiveConfigs(configsFile(radius))).toThrow(
      'configs[0].obstacles[0].radius_m must be >= 0',
    )
  })

  it('rejects non-finite wind triples', () => {
    const sigma = validConfig()
    sigma.wind_sigma = [1, Number.NaN, 0.5]
    expect(() => validateLiveConfigs(configsFile(sigma))).toThrow(
      'configs[0].wind_sigma[1] must be finite',
    )
    const steady = validConfig()
    steady.wind_steady_ned = [2, 0]
    expect(() => validateLiveConfigs(configsFile(steady))).toThrow(
      'configs[0].wind_steady_ned must have length 3 (got 2)',
    )
  })

  it('rejects negative GPS noise and cruise hold, and a non-finite seed', () => {
    const gps = validConfig()
    gps.gps_position_noise = -0.1
    expect(() => validateLiveConfigs(configsFile(gps))).toThrow(
      'configs[0].gps_position_noise must be >= 0',
    )
    const velocity = validConfig()
    velocity.gps_velocity_noise = Number.POSITIVE_INFINITY
    expect(() => validateLiveConfigs(configsFile(velocity))).toThrow(
      'configs[0].gps_velocity_noise must be finite',
    )
    const hold = validConfig()
    hold.cruise_hold_s = -1
    expect(() => validateLiveConfigs(configsFile(hold))).toThrow(
      'configs[0].cruise_hold_s must be >= 0',
    )
    const seed = validConfig()
    seed.seed = Number.NaN
    expect(() => validateLiveConfigs(configsFile(seed))).toThrow(
      'configs[0].seed must be finite',
    )
  })

  it('requires auto_arm to be a boolean', () => {
    const cfg = validConfig()
    cfg.auto_arm = 'yes'
    expect(() => validateLiveConfigs(configsFile(cfg))).toThrow(
      'configs[0].auto_arm must be a boolean',
    )
  })

  it('validates battery_initial_soc in [0, 1] when present, absent otherwise', () => {
    const absent = validConfig()
    expect(() => validateLiveConfigs(configsFile(absent))).not.toThrow()
    const good = validConfig()
    good.battery_initial_soc = 0.25
    expect(() => validateLiveConfigs(configsFile(good))).not.toThrow()
    for (const soc of [-0.1, 1.5, Number.NaN]) {
      const cfg = validConfig()
      cfg.battery_initial_soc = soc
      expect(() => validateLiveConfigs(configsFile(cfg))).toThrow(
        'configs[0].battery_initial_soc ' +
          (Number.isNaN(soc) ? 'must be finite' : 'must be in [0, 1]'),
      )
    }
  })

  it('validates an avoidance block when present', () => {
    const good = validConfig()
    good.avoidance = { enabled: true, clearance_margin_m: 2, lookahead_s: 1.5, detour_buffer_m: 1 }
    expect(() => validateLiveConfigs(configsFile(good))).not.toThrow()
    const missingEnabled = validConfig()
    missingEnabled.avoidance = { clearance_margin_m: 2 }
    expect(() => validateLiveConfigs(configsFile(missingEnabled))).toThrow(
      'configs[0].avoidance.enabled must be a boolean',
    )
    const negative = validConfig()
    negative.avoidance = { enabled: true, lookahead_s: -1 }
    expect(() => validateLiveConfigs(configsFile(negative))).toThrow(
      'configs[0].avoidance.lookahead_s must be >= 0',
    )
  })
})

// ---------------------------------------------------------------------------
// validateFlightData
// ---------------------------------------------------------------------------

describe('validateFlightData', () => {
  it('accepts every committed scenario recording', () => {
    // The four recordings loadScenario can fetch, read from disk exactly as
    // the store reads them over the network.
    expect(SCENARIO_IDS.length).toBeGreaterThan(0)
    for (const id of SCENARIO_IDS) {
      const doc = JSON.parse(readFileSync(`public/data/${id}.json`, 'utf8')) as unknown
      const data = validateFlightData(doc)
      expect(data.time_s.length, id).toBe(data.meta.sample_count)
      expect(data.truth_position.length, id).toBe(data.time_s.length)
      expect(data.e_ct_times.length, id).toBe(data.e_ct.length)
      expect(data.obstacles, id).toBeDefined()
      expect(data.home_ned.length, id).toBe(3)
    }
  })

  it('accepts a valid document and returns it unchanged', () => {
    const doc = validFlight()
    expect(validateFlightData(doc)).toEqual(doc)
  })

  it('fills in obstacles for a schema 1.0 document without mutating the input', () => {
    const doc = withoutKey(validFlight(), 'obstacles')
    const data = validateFlightData(doc)
    expect(data.obstacles).toEqual([])
    expect('obstacles' in doc).toBe(false)
  })

  it('requires a meta object', () => {
    const doc = withoutKey(validFlight(), 'meta')
    expect(() => validateFlightData(doc)).toThrow('meta must be an object')
    const dirty = validFlight()
    ;(dirty.meta as Record<string, unknown>).duration_s = Number.NaN
    expect(() => validateFlightData(dirty)).toThrow('meta.duration_s must be finite')
    const units = validFlight()
    ;(units.meta as Record<string, unknown>).units = { time: 5 }
    expect(() => validateFlightData(units)).toThrow('meta.units.time must be a string')
  })

  it('requires meta.sample_count to match the time history', () => {
    const doc = validFlight()
    ;(doc.meta as Record<string, unknown>).sample_count = 5
    expect(() => validateFlightData(doc)).toThrow(
      'meta.sample_count must equal the time_s length (got 5, expected 3)',
    )
  })

  it('requires every history the schema declares', () => {
    for (const key of ['time_s', 'command_thrust', 'e_ct_leg', 'truth_velocity'] as const) {
      const doc = withoutKey(validFlight(), key)
      expect(() => validateFlightData(doc), key).toThrow(`${key} must be an array`)
    }
  })

  it('rejects a non-finite number anywhere in a history', () => {
    const time = validFlight()
    ;(time.time_s as number[])[2] = Number.POSITIVE_INFINITY
    expect(() => validateFlightData(time)).toThrow('time_s[2] must be finite')
    const position = validFlight()
    ;(position.truth_position as number[][])[1] = [1, 0, Number.NaN]
    expect(() => validateFlightData(position)).toThrow('truth_position[1][2] must be finite')
    const thrust = validFlight()
    ;(thrust.command_thrust as number[])[1] = Number.NaN
    expect(() => validateFlightData(thrust)).toThrow('command_thrust[1] must be finite')
  })

  it('requires the per-sample histories to match time_s', () => {
    const short = validFlight()
    short.truth_position = [[0, 0, 0], [1, 0, 0]]
    expect(() => validateFlightData(short)).toThrow(
      'truth_position must have 3 samples to match time_s (got 2)',
    )
    const thrust = validFlight()
    thrust.command_thrust = [5]
    expect(() => validateFlightData(thrust)).toThrow(
      'command_thrust must have 3 samples to match time_s (got 1)',
    )
  })

  it('requires the cross-track histories to share one length', () => {
    const times = validFlight()
    times.e_ct_times = [0]
    expect(() => validateFlightData(times)).toThrow(
      'e_ct_times must have 2 samples to match e_ct (got 1)',
    )
    const legs = validFlight()
    legs.e_ct_leg = [0, 0, 0]
    expect(() => validateFlightData(legs)).toThrow(
      'e_ct_leg must have 2 samples to match e_ct (got 3)',
    )
  })

  it('requires a finite triple home position and waypoints', () => {
    const home = validFlight()
    home.home_ned = [0, 0]
    expect(() => validateFlightData(home)).toThrow('home_ned must have length 3 (got 2)')
    const waypoints = validFlight()
    waypoints.waypoints_ned = [[40, Number.NaN, -10]]
    expect(() => validateFlightData(waypoints)).toThrow('waypoints_ned[0][1] must be finite')
  })

  it('validates obstacles when present', () => {
    const id = validFlight()
    ;(id.obstacles as Record<string, unknown>[])[0] = { center_ned: [1, 2, -3], radius_m: 4 }
    expect(() => validateFlightData(id)).toThrow('obstacles[0].id must be a string')
    const radius = validFlight()
    ;(radius.obstacles as Record<string, unknown>[])[0]!.radius_m = -1
    expect(() => validateFlightData(radius)).toThrow('obstacles[0].radius_m must be >= 0')
    const centre = validFlight()
    ;(centre.obstacles as Record<string, unknown>[])[0]!.center_ned = [1, 2]
    expect(() => validateFlightData(centre)).toThrow(
      'obstacles[0].center_ned must have length 3 (got 2)',
    )
  })

  it('rejects a malformed phase timeline or event log', () => {
    const pair = validFlight()
    pair.phase_timeline = [[0]]
    expect(() => validateFlightData(pair)).toThrow('phase_timeline[0] must be a [time, phase] pair')
    const time = validFlight()
    time.phase_timeline = [[Number.NaN, 'idle']]
    expect(() => validateFlightData(time)).toThrow('phase_timeline[0][0] must be finite')
    const phase = validFlight()
    phase.phase_timeline = [[0, 'hover']]
    expect(() => validateFlightData(phase)).toThrow(
      'phase_timeline[0][1] must be a known flight phase',
    )
    const event = validFlight()
    ;(event.event_log as Record<string, unknown>[])[0]!.from_phase = 'hover'
    expect(() => validateFlightData(event)).toThrow(
      'event_log[0].from_phase must be a known flight phase',
    )
    const guard = validFlight()
    ;(guard.event_log as Record<string, unknown>[])[0]!.guard_ok = 'true'
    expect(() => validateFlightData(guard)).toThrow('event_log[0].guard_ok must be a boolean')
  })
})
