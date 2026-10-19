/**
 * Runtime validation for JSON fetched over the network.
 *
 * The loaders used to cast parsed JSON straight into the app's types. That is
 * fine for files this repository writes and wrong the moment a stale build, a
 * truncated download, or an edited recording lands in `public/data/`. These
 * functions take `unknown` and return the app's types, or throw an Error that
 * names the first bad path (`configs[2].duration_s must be finite`) so a
 * rejected payload says which field is wrong instead of surfacing later as NaN
 * in a chart.
 *
 * Pure: no fetch, no store, no DOM. The two entry points are
 * `validateLiveConfigs` (the runnable scenarios behind the Run view) and
 * `validateFlightData` (a recorded flight document, schema 1.0 or 1.1).
 */

import type { AvoidanceConfig, LiveConfig, Waypoint } from '@/live/types'
import type {
  EventLogEntry,
  FlightData,
  FlightMeta,
  FlightPhase,
  Obstacle,
  PhaseEvent,
  Vec3,
} from '@/types/flight'

/** Largest waypoint or keep-out list a scenario config may ship. */
export const MAX_LIST_LENGTH = 512

/**
 * Flight-phase labels, the schema 1.1 lowercase spellings. A document that
 * spells a phase outside this list would render with no palette entry, so it is
 * rejected at the door rather than coloured as idle.
 */
const PHASE_LABELS = [
  'idle',
  'arm',
  'takeoff',
  'cruise',
  'waypoint',
  'rtl',
  'land',
  'emergency',
] as const satisfies readonly FlightPhase[]

type PhaseLabel = (typeof PHASE_LABELS)[number]

function isPhaseLabel(value: unknown): value is PhaseLabel {
  return typeof value === 'string' && (PHASE_LABELS as readonly string[]).includes(value)
}

/** Throw with the schema path that failed, e.g. `configs[2].duration_s must be finite`. */
function bad(path: string, why: string): never {
  throw new Error(`${path} ${why}`)
}

function asRecord(value: unknown, path: string): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    bad(path, 'must be an object')
  }
  return value as Record<string, unknown>
}

function asArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) bad(path, 'must be an array')
  return value
}

function asBoundedArray(value: unknown, path: string): unknown[] {
  const items = asArray(value, path)
  if (items.length > MAX_LIST_LENGTH) {
    bad(path, `must have at most ${MAX_LIST_LENGTH} entries (got ${items.length})`)
  }
  return items
}

function asFinite(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) bad(path, 'must be finite')
  return value
}

function asNonNegative(value: unknown, path: string): number {
  const n = asFinite(value, path)
  if (n < 0) bad(path, 'must be >= 0')
  return n
}

/** A number in [0, 1], the range the Rust core enforces on state of charge. */
function asUnitRange(value: unknown, path: string): number {
  const n = asFinite(value, path)
  if (n < 0 || n > 1) bad(path, 'must be in [0, 1]')
  return n
}

/** A positive bounded scalar; `label` is the printed interval. */
function asPositiveUpTo(value: unknown, path: string, hi: number): number {
  const n = asFinite(value, path)
  if (n <= 0 || n > hi) bad(path, `must be in (0, ${hi}]`)
  return n
}

function asString(value: unknown, path: string): string {
  if (typeof value !== 'string') bad(path, 'must be a string')
  return value
}

function asBoolean(value: unknown, path: string): boolean {
  if (typeof value !== 'boolean') bad(path, 'must be a boolean')
  return value
}

/** A finite NED triple. */
function asVec3(value: unknown, path: string): Vec3 {
  const items = asArray(value, path)
  if (items.length !== 3) bad(path, `must have length 3 (got ${items.length})`)
  return [
    asFinite(items[0], `${path}[0]`),
    asFinite(items[1], `${path}[1]`),
    asFinite(items[2], `${path}[2]`),
  ]
}

function asPhase(value: unknown, path: string): FlightPhase {
  if (!isPhaseLabel(value)) bad(path, 'must be a known flight phase')
  return value
}

/** A per-sample scalar history: an array where every entry is finite. */
function scalarSeries(value: unknown, path: string): number[] {
  return asArray(value, path).map((entry, i) => asFinite(entry, `${path}[${i}]`))
}

/** A per-sample vector history: an array where every entry is a finite triple. */
function vec3Series(value: unknown, path: string): Vec3[] {
  return asArray(value, path).map((entry, i) => asVec3(entry, `${path}[${i}]`))
}

/** One history's length against the history it is aligned with. */
function sameLength(actual: number, expected: number, path: string, like: string): void {
  if (actual !== expected) {
    bad(path, `must have ${expected} samples to match ${like} (got ${actual})`)
  }
}

// ---------------------------------------------------------------------------
// Live configs (public/data/live/configs.json)
// ---------------------------------------------------------------------------

function validateWaypoint(value: unknown, path: string): Waypoint {
  const w = asRecord(value, path)
  return {
    position: asVec3(w.position, `${path}.position`),
    speed: asNonNegative(w.speed, `${path}.speed`),
    heading: asFinite(w.heading, `${path}.heading`),
    radius: asNonNegative(w.radius, `${path}.radius`),
  }
}

function validateObstacle(value: unknown, path: string): Obstacle {
  const o = asRecord(value, path)
  return {
    id: asString(o.id, `${path}.id`),
    center_ned: asVec3(o.center_ned, `${path}.center_ned`),
    radius_m: asNonNegative(o.radius_m, `${path}.radius_m`),
  }
}

function validateAvoidance(value: unknown, path: string): AvoidanceConfig {
  const a = asRecord(value, path)
  const cfg: AvoidanceConfig = { enabled: asBoolean(a.enabled, `${path}.enabled`) }
  if (a.clearance_margin_m !== undefined) {
    cfg.clearance_margin_m = asNonNegative(a.clearance_margin_m, `${path}.clearance_margin_m`)
  }
  if (a.lookahead_s !== undefined) {
    cfg.lookahead_s = asNonNegative(a.lookahead_s, `${path}.lookahead_s`)
  }
  if (a.detour_buffer_m !== undefined) {
    cfg.detour_buffer_m = asNonNegative(a.detour_buffer_m, `${path}.detour_buffer_m`)
  }
  return cfg
}

function validateConfig(value: unknown, path: string): LiveConfig {
  const c = asRecord(value, path)
  const id = asString(c.id, `${path}.id`)
  const label = asString(c.label, `${path}.label`)
  const blurb = asString(c.blurb, `${path}.blurb`)
  const duration_s = asPositiveUpTo(c.duration_s, `${path}.duration_s`, 900)
  const takeoff_altitude = asPositiveUpTo(c.takeoff_altitude, `${path}.takeoff_altitude`, 1000)
  const start_position_ned = asVec3(c.start_position_ned, `${path}.start_position_ned`)
  const waypoints = asBoundedArray(c.waypoints, `${path}.waypoints`).map((entry, i) =>
    validateWaypoint(entry, `${path}.waypoints[${i}]`),
  )
  const obstacles = asBoundedArray(c.obstacles, `${path}.obstacles`).map((entry, i) =>
    validateObstacle(entry, `${path}.obstacles[${i}]`),
  )
  const avoidance =
    c.avoidance === undefined ? undefined : validateAvoidance(c.avoidance, `${path}.avoidance`)
  const wind_sigma = asVec3(c.wind_sigma, `${path}.wind_sigma`)
  const wind_steady_ned = asVec3(c.wind_steady_ned, `${path}.wind_steady_ned`)
  const seed = asFinite(c.seed, `${path}.seed`)
  const gps_position_noise = asNonNegative(c.gps_position_noise, `${path}.gps_position_noise`)
  const gps_position_noise_vertical = asNonNegative(
    c.gps_position_noise_vertical,
    `${path}.gps_position_noise_vertical`,
  )
  const gps_velocity_noise = asNonNegative(c.gps_velocity_noise, `${path}.gps_velocity_noise`)
  const cruise_hold_s = asNonNegative(c.cruise_hold_s, `${path}.cruise_hold_s`)
  const auto_arm = asBoolean(c.auto_arm, `${path}.auto_arm`)

  const cfg: LiveConfig = {
    id,
    label,
    blurb,
    duration_s,
    takeoff_altitude,
    start_position_ned,
    waypoints,
    obstacles,
    wind_sigma,
    wind_steady_ned,
    seed,
    gps_position_noise,
    gps_position_noise_vertical,
    gps_velocity_noise,
    cruise_hold_s,
    auto_arm,
  }
  if (avoidance !== undefined) cfg.avoidance = avoidance
  if (c.battery_initial_soc !== undefined) {
    // The browser subset of SILConfig does not declare the battery knob yet
    // (docs/reviews.md), but a config that carries it must obey the same
    // [0, 1] range the Rust core enforces; keep the validated value on the
    // object so it survives the round trip through the loader.
    Object.assign(cfg, { battery_initial_soc: asUnitRange(c.battery_initial_soc, `${path}.battery_initial_soc`) })
  }
  return cfg
}

/**
 * Validate `public/data/live/configs.json` and return its runnable scenarios.
 *
 * Throws naming the first bad path, e.g. `configs[2].duration_s must be finite`.
 */
export function validateLiveConfigs(value: unknown): LiveConfig[] {
  const root = asRecord(value, 'value')
  const configs = asArray(root.configs, 'configs')
  return configs.map((entry, index) => validateConfig(entry, `configs[${index}]`))
}

// ---------------------------------------------------------------------------
// Flight documents (public/data/<scenario>.json, schema 1.0 and 1.1)
// ---------------------------------------------------------------------------

function validateMeta(value: unknown, path: string): FlightMeta {
  const m = asRecord(value, path)
  const rawUnits = asRecord(m.units, `${path}.units`)
  const units: Record<string, string> = {}
  for (const key of Object.keys(rawUnits)) {
    units[key] = asString(rawUnits[key], `${path}.units.${key}`)
  }
  return {
    scenario: asString(m.scenario, `${path}.scenario`),
    seed: asFinite(m.seed, `${path}.seed`),
    schema_version: asString(m.schema_version, `${path}.schema_version`),
    generated_utc: asString(m.generated_utc, `${path}.generated_utc`),
    sample_count: asFinite(m.sample_count, `${path}.sample_count`),
    duration_s: asFinite(m.duration_s, `${path}.duration_s`),
    dt: asFinite(m.dt, `${path}.dt`),
    units,
  }
}

function validateTimeline(value: unknown, path: string): PhaseEvent[] {
  return asArray(value, path).map((entry, i) => {
    const pair = asArray(entry, `${path}[${i}]`)
    if (pair.length !== 2) bad(`${path}[${i}]`, 'must be a [time, phase] pair')
    const event: PhaseEvent = [asFinite(pair[0], `${path}[${i}][0]`), asPhase(pair[1], `${path}[${i}][1]`)]
    return event
  })
}

function validateEventLog(value: unknown, path: string): EventLogEntry[] {
  return asArray(value, path).map((entry, i) => {
    const e = asRecord(entry, `${path}[${i}]`)
    return {
      time: asFinite(e.time, `${path}[${i}].time`),
      event: asString(e.event, `${path}[${i}].event`),
      from_phase: asPhase(e.from_phase, `${path}[${i}].from_phase`),
      to_phase: asPhase(e.to_phase, `${path}[${i}].to_phase`),
      guard_ok: asBoolean(e.guard_ok, `${path}[${i}].guard_ok`),
    }
  })
}

/**
 * Validate a recorded flight document and return it as `FlightData`.
 *
 * Every history named by the schema must exist, every number must be finite,
 * and the per-sample histories must agree with `time_s` (the same grouping
 * `python/talon/export_json.py` enforces as `_SERIES_1D`, `_SERIES_3D`, and
 * `_ECT_1D`). A schema 1.0 document with no `obstacles` key is accepted and
 * returned with `obstacles: []`.
 *
 * Throws naming the first bad path, e.g. `truth_position[17][2] must be finite`.
 */
export function validateFlightData(value: unknown): FlightData {
  const doc = asRecord(value, 'value')
  const meta = validateMeta(doc.meta, 'meta')

  // (N,) times, then the histories aligned with them.
  const time_s = scalarSeries(doc.time_s, 'time_s')
  const samples = time_s.length
  if (meta.sample_count !== samples) {
    bad('meta.sample_count', `must equal the time_s length (got ${meta.sample_count}, expected ${samples})`)
  }

  const command_thrust = scalarSeries(doc.command_thrust, 'command_thrust')
  sameLength(command_thrust.length, samples, 'command_thrust', 'time_s')

  const truth_position = vec3Series(doc.truth_position, 'truth_position')
  sameLength(truth_position.length, samples, 'truth_position', 'time_s')
  const truth_velocity = vec3Series(doc.truth_velocity, 'truth_velocity')
  sameLength(truth_velocity.length, samples, 'truth_velocity', 'time_s')
  const ekf_position = vec3Series(doc.ekf_position, 'ekf_position')
  sameLength(ekf_position.length, samples, 'ekf_position', 'time_s')
  const ekf_velocity = vec3Series(doc.ekf_velocity, 'ekf_velocity')
  sameLength(ekf_velocity.length, samples, 'ekf_velocity', 'time_s')
  const position_var = vec3Series(doc.position_var, 'position_var')
  sameLength(position_var.length, samples, 'position_var', 'time_s')
  const gyro_bias = vec3Series(doc.gyro_bias, 'gyro_bias')
  sameLength(gyro_bias.length, samples, 'gyro_bias', 'time_s')
  const accel_bias = vec3Series(doc.accel_bias, 'accel_bias')
  sameLength(accel_bias.length, samples, 'accel_bias', 'time_s')
  const wind_ned = vec3Series(doc.wind_ned, 'wind_ned')
  sameLength(wind_ned.length, samples, 'wind_ned', 'time_s')
  const command_moment = vec3Series(doc.command_moment, 'command_moment')
  sameLength(command_moment.length, samples, 'command_moment', 'time_s')

  // (M,) cross-track histories, aligned with each other, not with time_s.
  const e_ct = scalarSeries(doc.e_ct, 'e_ct')
  const e_ct_times = scalarSeries(doc.e_ct_times, 'e_ct_times')
  sameLength(e_ct_times.length, e_ct.length, 'e_ct_times', 'e_ct')
  const e_ct_leg = scalarSeries(doc.e_ct_leg, 'e_ct_leg')
  sameLength(e_ct_leg.length, e_ct.length, 'e_ct_leg', 'e_ct')

  // Mission geometry: (K,) waypoints and the (3,) home point.
  const waypoints_ned = vec3Series(doc.waypoints_ned, 'waypoints_ned')
  const home_ned = asVec3(doc.home_ned, 'home_ned')

  // Schema 1.0 documents predate the obstacles key; normalize so the scene
  // never has to guard against undefined.
  const obstacles: Obstacle[] =
    doc.obstacles === undefined
      ? []
      : asArray(doc.obstacles, 'obstacles').map((entry, i) => validateObstacle(entry, `obstacles[${i}]`))

  const phase_timeline = validateTimeline(doc.phase_timeline, 'phase_timeline')
  const event_log = validateEventLog(doc.event_log, 'event_log')

  return {
    meta,
    time_s,
    truth_position,
    truth_velocity,
    ekf_position,
    ekf_velocity,
    position_var,
    gyro_bias,
    accel_bias,
    wind_ned,
    command_thrust,
    command_moment,
    e_ct,
    e_ct_times,
    e_ct_leg,
    waypoints_ned,
    home_ned,
    obstacles,
    phase_timeline,
    event_log,
  }
}
