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
import { LIMITS, SCENARIO_DEFAULTS } from '@/types/generated/talon_core'
import {
  MAX_LIST_LENGTH,
  validateFlightData,
  validateLiveConfigs,
  validateScenarioConfig,
} from './validate'

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

  it('rejects a keep-out id that is not a lowercase slug or is repeated', () => {
    const badId = validConfig()
    badId.obstacles = [{ id: 'North Zone', center_ned: [1, 2, -3], radius_m: 4 }]
    expect(() => validateLiveConfigs(configsFile(badId))).toThrow(
      'configs[0].obstacles[0].id must be a lowercase slug: up to 64 letters, digits, or underscores',
    )
    const tooLong = validConfig()
    tooLong.obstacles = [{ id: 'a'.repeat(65), center_ned: [1, 2, -3], radius_m: 4 }]
    expect(() => validateLiveConfigs(configsFile(tooLong))).toThrow(
      'configs[0].obstacles[0].id must be a lowercase slug: up to 64 letters, digits, or underscores',
    )
    const repeated = validConfig()
    repeated.obstacles = [
      { id: 'obs_a', center_ned: [1, 2, -3], radius_m: 4 },
      { id: 'obs_a', center_ned: [9, 9, -3], radius_m: 2 },
    ]
    expect(() => validateLiveConfigs(configsFile(repeated))).toThrow(
      'configs[0].obstacles[1].id must be unique in the list ("obs_a" is already used)',
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

  it('keeps MAX_LIST_LENGTH wired to the generated list caps', () => {
    // The exported name stays the wider of the two per-list caps, so a caller
    // holding one number cannot under-count either list.
    expect(MAX_LIST_LENGTH).toBe(Math.max(LIMITS.max_waypoints, LIMITS.max_obstacles))
  })

  it('enforces the core caps so the browser gate and the runner agree', () => {
    const speed = validConfig()
    speed.waypoints = [{ position: [0, 0, -10], speed: 200.5, heading: 0, radius: 2 }]
    expect(() => validateLiveConfigs(configsFile(speed))).toThrow(
      'configs[0].waypoints[0].speed must be <= 200',
    )
    const radius = validConfig()
    radius.obstacles = [{ id: 'o', center_ned: [1, 2, -3], radius_m: 1_000_001 }]
    expect(() => validateLiveConfigs(configsFile(radius))).toThrow(
      'configs[0].obstacles[0].radius_m must be <= 1000000',
    )
    const far = validConfig()
    far.start_position_ned = [1_000_001, 0, 0]
    expect(() => validateLiveConfigs(configsFile(far))).toThrow(
      'configs[0].start_position_ned[0] must be in [-1000000, 1000000]',
    )
    const sigma = validConfig()
    sigma.wind_sigma = [101, 0, 0]
    expect(() => validateLiveConfigs(configsFile(sigma))).toThrow(
      'configs[0].wind_sigma[0] must be <= 100',
    )
    const steady = validConfig()
    steady.wind_steady_ned = [0, -101, 0]
    expect(() => validateLiveConfigs(configsFile(steady))).toThrow(
      'configs[0].wind_steady_ned[1] must be in [-100, 100]',
    )
    const gps = validConfig()
    gps.gps_velocity_noise = 50.5
    expect(() => validateLiveConfigs(configsFile(gps))).toThrow(
      'configs[0].gps_velocity_noise must be <= 50',
    )
    const hold = validConfig()
    hold.cruise_hold_s = 60.5
    expect(() => validateLiveConfigs(configsFile(hold))).toThrow(
      'configs[0].cruise_hold_s must be <= 60',
    )
    const seed = validConfig()
    seed.seed = Number.MAX_SAFE_INTEGER + 2
    expect(() => validateLiveConfigs(configsFile(seed))).toThrow(
      'configs[0].seed must be <= 9007199254740991',
    )
  })
})

// ---------------------------------------------------------------------------
// validateScenarioConfig
// ---------------------------------------------------------------------------

/** The minimal authored draft: only the id, every other knob defaulted. */
function minimalScenario(): Record<string, unknown> {
  return { id: 'custom_mission' }
}

/** A draft with `overrides` applied on top of the minimal one. */
function scenario(...overrides: Record<string, unknown>[]): Record<string, unknown> {
  return { ...minimalScenario(), ...Object.assign({}, ...overrides) }
}

/** A full authored document: waypoints, zones, wind, avoidance, one of each fault. */
function fullScenario(): Record<string, unknown> {
  return {
    id: 'author_smoke',
    label: 'Author smoke',
    blurb: 'Every authored knob at once.',
    duration_s: 30,
    takeoff_altitude: 12,
    start_position_ned: [1, -2, 0],
    waypoints: [{ position: [10, 5, -12], speed: 6, heading: 1.5, radius: 2.5 }],
    obstacles: [{ id: 'obs_a', center_ned: [4, -3, -6], radius_m: 3 }],
    avoidance: { enabled: true, clearance_margin_m: 2.5, lookahead_s: 1.5, detour_buffer_m: 1 },
    wind_sigma: [1, 1, 0.5],
    wind_steady_ned: [2, 0, -0.5],
    battery_initial_soc: 0.9,
    gps_position_noise: 0.4,
    gps_position_noise_vertical: 0.6,
    gps_velocity_noise: 0.07,
    cruise_hold_s: 3,
    seed: 42,
    auto_arm: true,
    faults: [
      { kind: 'gps_outage', at_s: 1, duration_s: 2 },
      { kind: 'ekf_divergence', at_s: 5, mode: 'adversarial_gps' },
      { kind: 'battery_soc', at_s: 8, soc: 0.4 },
      { kind: 'motor_fault', at_s: 10 },
      { kind: 'emergency', at_s: 12 },
      { kind: 'rtl', at_s: 15 },
    ],
  }
}

describe('validateScenarioConfig', () => {
  it('fills a minimal document from the scenario defaults', () => {
    const cfg = validateScenarioConfig(minimalScenario())
    expect(cfg.id).toBe('custom_mission')
    expect(cfg.duration_s).toBe(60)
    expect(cfg.takeoff_altitude).toBe(10)
    expect(cfg.avoidance?.enabled).toBe(false)
    expect(cfg.avoidance?.clearance_margin_m).toBe(2)
    expect(cfg.faults).toEqual([])
    // The whole document equals the defaults with only the id swapped, which
    // pins every defaulted field at once.
    expect(cfg).toEqual({ ...SCENARIO_DEFAULTS, id: 'custom_mission' })
  })

  it('merges a partial avoidance block over the defaults', () => {
    const cfg = validateScenarioConfig(scenario({ avoidance: { enabled: true } }))
    expect(cfg.avoidance).toEqual({
      enabled: true,
      clearance_margin_m: SCENARIO_DEFAULTS.avoidance?.clearance_margin_m,
      lookahead_s: SCENARIO_DEFAULTS.avoidance?.lookahead_s,
      detour_buffer_m: SCENARIO_DEFAULTS.avoidance?.detour_buffer_m,
    })
  })

  it('accepts a full authored document with one of each fault kind', () => {
    const cfg = validateScenarioConfig(fullScenario())
    expect(cfg.label).toBe('Author smoke')
    expect(cfg.waypoints).toHaveLength(1)
    expect(cfg.obstacles).toHaveLength(1)
    expect(cfg.faults).toHaveLength(6)
    expect(cfg.faults?.[0]).toEqual({ kind: 'gps_outage', at_s: 1, duration_s: 2 })
    expect(cfg.faults?.[5]).toEqual({ kind: 'rtl', at_s: 15 })
    expect(cfg.avoidance).toEqual({
      enabled: true,
      clearance_margin_m: 2.5,
      lookahead_s: 1.5,
      detour_buffer_m: 1,
    })
    expect(cfg.wind_steady_ned).toEqual([2, 0, -0.5])
    expect(cfg.seed).toBe(42)
    expect(cfg.auto_arm).toBe(true)
  })

  it('requires a non-reserved slug for the id', () => {
    expect(() => validateScenarioConfig({})).toThrow('id must be a string')
    for (const id of ['Hover', '2nd_try', '', 'a'.repeat(65), 'has space']) {
      expect(() => validateScenarioConfig({ id }), id).toThrow(
        'id must be a lowercase slug: a letter, then up to 63 letters, digits, or underscores',
      )
    }
    for (const id of [
      'hover',
      'waypoints',
      'obstacle_field',
      'avoidance',
      'wind',
      'gps_outage',
      'emergency',
      'draft',
      'live',
    ]) {
      expect(() => validateScenarioConfig({ id }), id).toThrow(
        `id must not be a reserved scenario id ("${id}")`,
      )
    }
  })

  it('rejects a duration that is non-finite or outside (0, 900]', () => {
    for (const duration of [Number.NaN, Number.POSITIVE_INFINITY]) {
      expect(() => validateScenarioConfig(scenario({ duration_s: duration }))).toThrow(
        'duration_s must be finite',
      )
    }
    for (const duration of [0, -1, 901]) {
      expect(() => validateScenarioConfig(scenario({ duration_s: duration }))).toThrow(
        'duration_s must be in (0, 900]',
      )
    }
  })

  it('rejects a takeoff altitude outside (0, 1000]', () => {
    expect(() => validateScenarioConfig(scenario({ takeoff_altitude: Number.NaN }))).toThrow(
      'takeoff_altitude must be finite',
    )
    for (const altitude of [0, 1000.5]) {
      expect(() => validateScenarioConfig(scenario({ takeoff_altitude: altitude }))).toThrow(
        'takeoff_altitude must be in (0, 1000]',
      )
    }
  })

  it('rejects a coordinate beyond the coordinate cap', () => {
    expect(() => validateScenarioConfig(scenario({ start_position_ned: [1_000_001, 0, 0] }))).toThrow(
      'start_position_ned[0] must be in [-1000000, 1000000]',
    )
    const waypoint = { position: [0, -1_000_001, -10], speed: 5, heading: 0, radius: 2 }
    expect(() => validateScenarioConfig(scenario({ waypoints: [waypoint] }))).toThrow(
      'waypoints[0].position[1] must be in [-1000000, 1000000]',
    )
    const obstacle = { id: 'o', center_ned: [1, 2, 1_000_001], radius_m: 4 }
    expect(() => validateScenarioConfig(scenario({ obstacles: [obstacle] }))).toThrow(
      'obstacles[0].center_ned[2] must be in [-1000000, 1000000]',
    )
  })

  it('rejects a waypoint speed or radius above its cap', () => {
    const speed = { position: [0, 0, -10], speed: 200.5, heading: 0, radius: 2 }
    expect(() => validateScenarioConfig(scenario({ waypoints: [speed] }))).toThrow(
      'waypoints[0].speed must be <= 200',
    )
    const radius = { position: [0, 0, -10], speed: 5, heading: 0, radius: 1_000_001 }
    expect(() => validateScenarioConfig(scenario({ waypoints: [radius] }))).toThrow(
      'waypoints[0].radius must be <= 1000000',
    )
    const zone = { id: 'o', center_ned: [1, 2, -3], radius_m: 1_000_001 }
    expect(() => validateScenarioConfig(scenario({ obstacles: [zone] }))).toThrow(
      'obstacles[0].radius_m must be <= 1000000',
    )
  })

  it('rejects a keep-out id that is not a lowercase slug or is repeated', () => {
    expect(() =>
      validateScenarioConfig(
        scenario({
          obstacles: [{ id: 'North Zone', center_ned: [1, 2, -3], radius_m: 4 }],
        }),
      ),
    ).toThrow('obstacles[0].id must be a lowercase slug: up to 64 letters, digits, or underscores')
    expect(() =>
      validateScenarioConfig(
        scenario({
          obstacles: [{ id: 'zone a', center_ned: [1, 2, -3], radius_m: 4 }],
        }),
      ),
    ).toThrow('obstacles[0].id must be a lowercase slug: up to 64 letters, digits, or underscores')
    expect(() =>
      validateScenarioConfig(
        scenario({
          obstacles: [
            { id: 'obs_a', center_ned: [1, 2, -3], radius_m: 4 },
            { id: 'obs_a', center_ned: [9, 9, -3], radius_m: 2 },
          ],
        }),
      ),
    ).toThrow('obstacles[1].id must be unique in the list ("obs_a" is already used)')
  })

  it('accepts the recorded zone ids and the editor-generated zone_N ids', () => {
    const cfg = validateScenarioConfig(
      scenario({
        obstacles: [
          { id: 'obs_north', center_ned: [1, 2, -3], radius_m: 3 },
          { id: 'obs_east', center_ned: [5, 6, -3], radius_m: 3 },
          { id: 'zone_12', center_ned: [9, 9, -3], radius_m: 3 },
        ],
      }),
    )
    expect((cfg.obstacles ?? []).map((o) => o.id)).toEqual(['obs_north', 'obs_east', 'zone_12'])
  })

  it('caps waypoints, obstacles and faults at the generated list caps', () => {
    const waypoint = { position: [0, 0, -10], speed: 5, heading: 0, radius: 2 }
    const ok = scenario({ waypoints: rows(LIMITS.max_waypoints, () => waypoint) })
    expect(() => validateScenarioConfig(ok)).not.toThrow()
    const manyWaypoints = scenario({ waypoints: rows(513, () => waypoint) })
    expect(() => validateScenarioConfig(manyWaypoints)).toThrow(
      'waypoints must have at most 512 entries (got 513)',
    )
    const zone = { id: 'o', center_ned: [1, 2, -3], radius_m: 4 }
    const manyZones = scenario({ obstacles: rows(513, () => zone) })
    expect(() => validateScenarioConfig(manyZones)).toThrow(
      'obstacles must have at most 512 entries (got 513)',
    )
    const fault = { kind: 'rtl', at_s: 1 }
    const manyFaults = scenario({ faults: rows(17, () => fault) })
    expect(() => validateScenarioConfig(manyFaults)).toThrow(
      'faults must have at most 16 entries (got 17)',
    )
  })

  it('rejects an avoidance knob above its cap and a non-boolean switch', () => {
    expect(() =>
      validateScenarioConfig(scenario({ avoidance: { enabled: 'yes' } })),
    ).toThrow('avoidance.enabled must be a boolean')
    expect(() =>
      validateScenarioConfig(scenario({ avoidance: { clearance_margin_m: 1000.5 } })),
    ).toThrow('avoidance.clearance_margin_m must be <= 1000')
    expect(() => validateScenarioConfig(scenario({ avoidance: { lookahead_s: 60.5 } }))).toThrow(
      'avoidance.lookahead_s must be <= 60',
    )
    expect(() =>
      validateScenarioConfig(scenario({ avoidance: { detour_buffer_m: 1000.5 } })),
    ).toThrow('avoidance.detour_buffer_m must be <= 1000')
  })

  it('rejects a wind component beyond the wind cap', () => {
    expect(() => validateScenarioConfig(scenario({ wind_sigma: [101, 0, 0] }))).toThrow(
      'wind_sigma[0] must be <= 100',
    )
    expect(() => validateScenarioConfig(scenario({ wind_sigma: [-1, 0, 0] }))).toThrow(
      'wind_sigma[0] must be >= 0',
    )
    expect(() => validateScenarioConfig(scenario({ wind_steady_ned: [-101, 0, 0] }))).toThrow(
      'wind_steady_ned[0] must be in [-100, 100]',
    )
  })

  it('rejects a GPS noise or cruise hold above its cap', () => {
    expect(() => validateScenarioConfig(scenario({ gps_position_noise: 100.5 }))).toThrow(
      'gps_position_noise must be <= 100',
    )
    expect(() =>
      validateScenarioConfig(scenario({ gps_position_noise_vertical: 100.5 })),
    ).toThrow('gps_position_noise_vertical must be <= 100')
    expect(() => validateScenarioConfig(scenario({ gps_velocity_noise: 50.5 }))).toThrow(
      'gps_velocity_noise must be <= 50',
    )
    expect(() => validateScenarioConfig(scenario({ cruise_hold_s: 60.5 }))).toThrow(
      'cruise_hold_s must be <= 60',
    )
  })

  it('rejects a battery state of charge outside [0, 1]', () => {
    for (const soc of [-0.1, 1.2, Number.NaN]) {
      expect(() => validateScenarioConfig(scenario({ battery_initial_soc: soc }))).toThrow(
        Number.isNaN(soc)
          ? 'battery_initial_soc must be finite'
          : 'battery_initial_soc must be in [0, 1]',
      )
    }
  })

  it('requires a seed that is a safe non-negative integer', () => {
    expect(() => validateScenarioConfig(scenario({ seed: 1.5 }))).toThrow(
      'seed must be an integer',
    )
    expect(() => validateScenarioConfig(scenario({ seed: -3 }))).toThrow('seed must be >= 0')
    expect(() => validateScenarioConfig(scenario({ seed: Number.MAX_SAFE_INTEGER + 2 }))).toThrow(
      'seed must be <= 9007199254740991',
    )
  })

  it('requires a known fault kind and its kind-specific field', () => {
    expect(() => validateScenarioConfig(scenario({ faults: [{ at_s: 1 }] }))).toThrow(
      'faults[0].kind must be a string',
    )
    expect(() =>
      validateScenarioConfig(scenario({ faults: [{ kind: 'gps_blackout', at_s: 1 }] })),
    ).toThrow(
      'faults[0].kind must be one of gps_outage, ekf_divergence, battery_soc, motor_fault, emergency, rtl',
    )
    expect(() => validateScenarioConfig(scenario({ faults: [{ kind: 'gps_outage', at_s: 1 }] }))).toThrow(
      'faults[0].duration_s must be finite',
    )
    expect(() =>
      validateScenarioConfig(scenario({ faults: [{ kind: 'ekf_divergence', at_s: 1 }] })),
    ).toThrow('faults[0].mode must be a string')
    expect(() =>
      validateScenarioConfig(
        scenario({ faults: [{ kind: 'ekf_divergence', at_s: 1, mode: 'corrupt_all' }] }),
      ),
    ).toThrow('faults[0].mode must be one of corrupt_bias, suppress_updates, adversarial_gps')
    expect(() =>
      validateScenarioConfig(scenario({ faults: [{ kind: 'battery_soc', at_s: 1 }] })),
    ).toThrow('faults[0].soc must be finite')
  })

  it('rejects an outage duration or forced state of charge outside its range', () => {
    for (const duration of [0, 901]) {
      expect(() =>
        validateScenarioConfig(
          scenario({ faults: [{ kind: 'gps_outage', at_s: 1, duration_s: duration }] }),
        ),
      ).toThrow('faults[0].duration_s must be in (0, 900]')
    }
    for (const soc of [-0.1, 1.2]) {
      expect(() =>
        validateScenarioConfig(scenario({ faults: [{ kind: 'battery_soc', at_s: 1, soc }] })),
      ).toThrow('faults[0].soc must be in [0, 1]')
    }
  })

  it('requires each at_s inside the run window of its own duration', () => {
    expect(() =>
      validateScenarioConfig(scenario({ duration_s: 30, faults: [{ kind: 'rtl', at_s: Number.NaN }] })),
    ).toThrow('faults[0].at_s must be finite')
    // Negative, and at the duration itself: the last step clock of a 30 s run
    // is one step earlier, so a fault there would never fire.
    for (const at_s of [-1, 30]) {
      expect(() =>
        validateScenarioConfig(scenario({ duration_s: 30, faults: [{ kind: 'rtl', at_s }] })),
      ).toThrow('faults[0].at_s must be in [0,')
    }
  })

  it('rejects every fault when the run has no fault window', () => {
    expect(() =>
      validateScenarioConfig(scenario({ duration_s: 0.005, faults: [{ kind: 'rtl', at_s: 0 }] })),
    ).toThrow('faults must be empty: a run of 0.005 s has fewer than two steps')
  })

  it('accepts a fault on the last step clock and rejects one step later', () => {
    // The exact window end a 30 s run has: `(round(30 / 0.005) - 1) * 0.005`.
    const last = (Math.round(30 / 0.005) - 1) * 0.005
    const ok = validateScenarioConfig(
      scenario({ duration_s: 30, faults: [{ kind: 'rtl', at_s: last }] }),
    )
    expect(ok.faults).toEqual([{ kind: 'rtl', at_s: last }])
    const late = scenario({ duration_s: 30, faults: [{ kind: 'rtl', at_s: last + 0.005 }] })
    expect(() => validateScenarioConfig(late)).toThrow(
      `faults[0].at_s must be in [0, ${last}]`,
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
