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
 * Pure: no fetch, no store, no DOM. The three entry points are
 * `validateLiveConfigs` (the runnable scenarios behind the Run view),
 * `validateScenarioConfig` (an authored mission draft: sparse, filled from
 * the generated scenario defaults and checked against the generated caps),
 * and `validateFlightData` (a recorded flight document, schema 1.0 or 1.1).
 *
 * The numeric caps come from the generated `LIMITS` (the Rust core's
 * `talon-core::constants::Limits` mirror, D23), so the browser gate and the
 * core gate reject the same documents by construction.
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
import { LIMITS, SCENARIO_DEFAULTS } from '@/types/generated/talon_core'
import type { FaultSpec, ScenarioConfig } from '@/types/generated/talon_core'

/**
 * Largest waypoint or keep-out list a scenario config may ship, the wider of
 * the two generated list caps. Kept exported as one number for callers that
 * need a single bound; per-list validation uses each list's own `LIMITS` cap.
 */
export const MAX_LIST_LENGTH = Math.max(LIMITS.max_waypoints, LIMITS.max_obstacles)

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

/** An array capped by the caller, e.g. `LIMITS.max_waypoints` entries. */
function asBoundedArray(value: unknown, path: string, max: number): unknown[] {
  const items = asArray(value, path)
  if (items.length > max) {
    bad(path, `must have at most ${max} entries (got ${items.length})`)
  }
  return items
}

function asFinite(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) bad(path, 'must be finite')
  return value
}

/** A number in [0, hi]; the interval each core cap prints in its message. */
function asNonNegativeUpTo(value: unknown, path: string, hi: number): number {
  const n = asFinite(value, path)
  if (n < 0) bad(path, 'must be >= 0')
  if (n > hi) bad(path, `must be <= ${hi}`)
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

/** A component inside `[-hi, hi]`; finiteness first so NaN stays named. */
function asBoundedComponent(value: unknown, path: string, hi: number): number {
  const n = asFinite(value, path)
  if (n < -hi || n > hi) bad(path, `must be in [-${hi}, ${hi}]`)
  return n
}

/**
 * A NED position triple whose components stay inside the given cap, the
 * coordinate cap by default (`LIMITS.max_coordinate_m`).
 */
function asBoundedVec3(value: unknown, path: string, hi: number = LIMITS.max_coordinate_m): Vec3 {
  const items = asArray(value, path)
  if (items.length !== 3) bad(path, `must have length 3 (got ${items.length})`)
  return [
    asBoundedComponent(items[0], `${path}[0]`, hi),
    asBoundedComponent(items[1], `${path}[1]`, hi),
    asBoundedComponent(items[2], `${path}[2]`, hi),
  ]
}

/** A Dryden turbulence triple: each component in [0, max_wind_component_mps]. */
function asSigmaVec3(value: unknown, path: string): Vec3 {
  const items = asArray(value, path)
  if (items.length !== 3) bad(path, `must have length 3 (got ${items.length})`)
  const hi = LIMITS.max_wind_component_mps
  return [
    asNonNegativeUpTo(items[0], `${path}[0]`, hi),
    asNonNegativeUpTo(items[1], `${path}[1]`, hi),
    asNonNegativeUpTo(items[2], `${path}[2]`, hi),
  ]
}

/** An RNG seed: a non-negative integer double precision still represents exactly. */
function asSafeSeed(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) bad(path, 'must be finite')
  if (!Number.isInteger(value)) bad(path, 'must be an integer')
  if (value < 0) bad(path, 'must be >= 0')
  if (!Number.isSafeInteger(value)) bad(path, `must be <= ${Number.MAX_SAFE_INTEGER}`)
  return value
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
    position: asBoundedVec3(w.position, `${path}.position`),
    speed: asNonNegativeUpTo(w.speed, `${path}.speed`, LIMITS.max_waypoint_speed_mps),
    heading: asFinite(w.heading, `${path}.heading`),
    radius: asNonNegativeUpTo(w.radius, `${path}.radius`, LIMITS.max_radius_m),
  }
}

function validateObstacle(value: unknown, path: string): Obstacle {
  const o = asRecord(value, path)
  return {
    id: asString(o.id, `${path}.id`),
    center_ned: asBoundedVec3(o.center_ned, `${path}.center_ned`),
    radius_m: asNonNegativeUpTo(o.radius_m, `${path}.radius_m`, LIMITS.max_radius_m),
  }
}

/** The keep-out id shape: a lowercase slug with underscores, 1 to 64 characters. */
const OBSTACLE_ID_PATTERN = /^[a-z0-9_]{1,64}$/

/**
 * Check one already-typed keep-out list by id: every id is a lowercase slug
 * (`obs_north`, `zone_3`, `o`) and no id appears twice, so a document can
 * name its zones unambiguously. `path` is the list path the errors point at,
 * e.g. `configs[0].obstacles`. Applied by both list gates, the live config
 * gate and the authored scenario gate.
 */
function checkObstacleIds(obstacles: readonly Obstacle[], path: string): void {
  const seen = new Set<string>()
  for (const [index, obstacle] of obstacles.entries()) {
    if (!OBSTACLE_ID_PATTERN.test(obstacle.id)) {
      bad(
        `${path}[${index}].id`,
        'must be a lowercase slug: up to 64 letters, digits, or underscores',
      )
    }
    if (seen.has(obstacle.id)) {
      bad(`${path}[${index}].id`, `must be unique in the list ("${obstacle.id}" is already used)`)
    }
    seen.add(obstacle.id)
  }
}

function validateAvoidance(value: unknown, path: string): AvoidanceConfig {
  const a = asRecord(value, path)
  const cfg: AvoidanceConfig = { enabled: asBoolean(a.enabled, `${path}.enabled`) }
  if (a.clearance_margin_m !== undefined) {
    cfg.clearance_margin_m = asNonNegativeUpTo(
      a.clearance_margin_m,
      `${path}.clearance_margin_m`,
      LIMITS.max_avoidance_margin_m,
    )
  }
  if (a.lookahead_s !== undefined) {
    cfg.lookahead_s = asNonNegativeUpTo(
      a.lookahead_s,
      `${path}.lookahead_s`,
      LIMITS.max_avoidance_lookahead_s,
    )
  }
  if (a.detour_buffer_m !== undefined) {
    cfg.detour_buffer_m = asNonNegativeUpTo(
      a.detour_buffer_m,
      `${path}.detour_buffer_m`,
      LIMITS.max_avoidance_buffer_m,
    )
  }
  return cfg
}

function validateConfig(value: unknown, path: string): LiveConfig {
  const c = asRecord(value, path)
  const id = asString(c.id, `${path}.id`)
  const label = asString(c.label, `${path}.label`)
  const blurb = asString(c.blurb, `${path}.blurb`)
  const duration_s = asPositiveUpTo(c.duration_s, `${path}.duration_s`, LIMITS.max_duration_s)
  const takeoff_altitude = asPositiveUpTo(
    c.takeoff_altitude,
    `${path}.takeoff_altitude`,
    LIMITS.max_takeoff_altitude_m,
  )
  const start_position_ned = asBoundedVec3(c.start_position_ned, `${path}.start_position_ned`)
  const waypoints = asBoundedArray(c.waypoints, `${path}.waypoints`, LIMITS.max_waypoints).map(
    (entry, i) => validateWaypoint(entry, `${path}.waypoints[${i}]`),
  )
  const obstacles = asBoundedArray(c.obstacles, `${path}.obstacles`, LIMITS.max_obstacles).map(
    (entry, i) => validateObstacle(entry, `${path}.obstacles[${i}]`),
  )
  checkObstacleIds(obstacles, `${path}.obstacles`)
  const avoidance =
    c.avoidance === undefined ? undefined : validateAvoidance(c.avoidance, `${path}.avoidance`)
  const wind_sigma = asSigmaVec3(c.wind_sigma, `${path}.wind_sigma`)
  const wind_steady_ned = asBoundedVec3(
    c.wind_steady_ned,
    `${path}.wind_steady_ned`,
    LIMITS.max_wind_component_mps,
  )
  const seed = asSafeSeed(c.seed, `${path}.seed`)
  const gps_position_noise = asNonNegativeUpTo(
    c.gps_position_noise,
    `${path}.gps_position_noise`,
    LIMITS.max_gps_position_noise_m,
  )
  const gps_position_noise_vertical = asNonNegativeUpTo(
    c.gps_position_noise_vertical,
    `${path}.gps_position_noise_vertical`,
    LIMITS.max_gps_position_noise_m,
  )
  const gps_velocity_noise = asNonNegativeUpTo(
    c.gps_velocity_noise,
    `${path}.gps_velocity_noise`,
    LIMITS.max_gps_velocity_noise_mps,
  )
  const cruise_hold_s = asNonNegativeUpTo(
    c.cruise_hold_s,
    `${path}.cruise_hold_s`,
    LIMITS.max_cruise_hold_s,
  )
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
    // A config that carries the battery knob must obey the same [0, 1] range
    // the Rust core enforces. Absent means the core's default, so the key
    // stays off the object rather than landing as undefined.
    cfg.battery_initial_soc = asUnitRange(c.battery_initial_soc, `${path}.battery_initial_soc`)
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
// Authored scenarios (mission authoring, Phase 8)
// ---------------------------------------------------------------------------

/**
 * Ids an authored scenario must not own: the shipped configs' ids plus the
 * live run slot key. Exported for the mission editor, which must steer
 * generated ids (`custom_mission`, `zone_N`) around the same list so every id
 * the editor offers passes this gate.
 */
export const RESERVED_SCENARIO_IDS = [
  'hover',
  'waypoints',
  'obstacle_field',
  'avoidance',
  'wind',
  'gps_outage',
  'emergency',
  'draft',
  // The live run slot: an authored mission must not claim the key the Run
  // view files its in-browser run under, or it would read back as a shipped
  // scenario instead of the author's draft.
  'live',
] as const

/** The authored id shape: a lowercase slug, letter first, at most 64 characters. */
const SCENARIO_ID_PATTERN = /^[a-z][a-z0-9_]{0,63}$/

/** The fault families an authored scenario may schedule. */
const FAULT_KINDS = [
  'gps_outage',
  'ekf_divergence',
  'battery_soc',
  'motor_fault',
  'emergency',
  'rtl',
] as const

type AuthoredFaultKind = (typeof FAULT_KINDS)[number]

/** The estimator corruptions an `ekf_divergence` fault may apply. */
const DIVERGENCE_MODES = ['corrupt_bias', 'suppress_updates', 'adversarial_gps'] as const

type AuthoredDivergenceMode = (typeof DIVERGENCE_MODES)[number]

function isFaultKind(value: string): value is AuthoredFaultKind {
  return (FAULT_KINDS as readonly string[]).includes(value)
}

function isDivergenceMode(value: string): value is AuthoredDivergenceMode {
  return (DIVERGENCE_MODES as readonly string[]).includes(value)
}

/**
 * The generated defaults ship the complete avoidance block; the mirror types
 * every field as optional, so pin the full shape once here.
 */
const AVOIDANCE_DEFAULTS = SCENARIO_DEFAULTS.avoidance as Required<
  NonNullable<ScenarioConfig['avoidance']>
>

/** An authored field: an absent key means "take the generated default". */
function authored(raw: unknown, fallback: unknown): unknown {
  return raw === undefined ? fallback : raw
}

/** An authored scenario id: a lowercase slug that collides with no shipped scenario. */
function asScenarioId(value: unknown, path: string): string {
  const id = asString(value, path)
  if (!SCENARIO_ID_PATTERN.test(id)) {
    bad(path, 'must be a lowercase slug: a letter, then up to 63 letters, digits, or underscores')
  }
  if ((RESERVED_SCENARIO_IDS as readonly string[]).includes(id)) {
    bad(path, `must not be a reserved scenario id ("${id}")`)
  }
  return id
}

/**
 * The sim time of the last step of a run of `duration_s`, the latest clock a
 * fault still has a step to act on, or `null` when the run rounds to fewer
 * than two steps and so has no window at all. Mirrors the core runner: the
 * step count is `round(duration_s / dt)` and step `i` starts at clock
 * `i * dt`, so the last step's own clock is a valid fire time.
 */
function lastStepTime(duration_s: number): number | null {
  const steps = Math.max(1, Math.round(duration_s / LIMITS.dt))
  if (steps < 2) return null
  return (steps - 1) * LIMITS.dt
}

/** One authored fault, checked against the fire window of its own scenario. */
function validateFault(value: unknown, path: string, windowEnd: number): FaultSpec {
  const f = asRecord(value, path)
  const kind = asString(f.kind, `${path}.kind`)
  if (!isFaultKind(kind)) {
    bad(`${path}.kind`, `must be one of ${FAULT_KINDS.join(', ')}`)
  }
  const at_s = asFinite(f.at_s, `${path}.at_s`)
  if (at_s < 0 || at_s > windowEnd) {
    bad(`${path}.at_s`, `must be in [0, ${windowEnd}]`)
  }
  switch (kind) {
    case 'gps_outage':
      return {
        kind,
        at_s,
        duration_s: asPositiveUpTo(f.duration_s, `${path}.duration_s`, LIMITS.max_duration_s),
      }
    case 'ekf_divergence': {
      const mode = asString(f.mode, `${path}.mode`)
      if (!isDivergenceMode(mode)) {
        bad(`${path}.mode`, `must be one of ${DIVERGENCE_MODES.join(', ')}`)
      }
      return { kind, at_s, mode }
    }
    case 'battery_soc':
      return { kind, at_s, soc: asUnitRange(f.soc, `${path}.soc`) }
    default:
      // motor_fault, emergency and rtl carry no parameter of their own.
      return { kind, at_s }
  }
}

/** The whole authored fault list: count cap first, then the per-fault checks. */
function validateFaults(value: unknown, path: string, duration_s: number): FaultSpec[] {
  const items = asBoundedArray(value, path, LIMITS.max_faults)
  if (items.length === 0) return []
  const windowEnd = lastStepTime(duration_s)
  if (windowEnd === null) {
    bad(path, `must be empty: a run of ${duration_s} s has fewer than two steps`)
  }
  return items.map((entry, i) => validateFault(entry, `${path}[${i}]`, windowEnd))
}

/**
 * The authored avoidance block: a missing block is the defaults, and a
 * present block merges knob by knob, an absent knob taking its default.
 */
function validateScenarioAvoidance(value: unknown, path: string): AvoidanceConfig {
  if (value === undefined) return { ...AVOIDANCE_DEFAULTS }
  const a = asRecord(value, path)
  const enabled =
    a.enabled === undefined ? AVOIDANCE_DEFAULTS.enabled : asBoolean(a.enabled, `${path}.enabled`)
  const clearance_margin_m =
    a.clearance_margin_m === undefined
      ? AVOIDANCE_DEFAULTS.clearance_margin_m
      : asNonNegativeUpTo(
          a.clearance_margin_m,
          `${path}.clearance_margin_m`,
          LIMITS.max_avoidance_margin_m,
        )
  const lookahead_s =
    a.lookahead_s === undefined
      ? AVOIDANCE_DEFAULTS.lookahead_s
      : asNonNegativeUpTo(a.lookahead_s, `${path}.lookahead_s`, LIMITS.max_avoidance_lookahead_s)
  const detour_buffer_m =
    a.detour_buffer_m === undefined
      ? AVOIDANCE_DEFAULTS.detour_buffer_m
      : asNonNegativeUpTo(
          a.detour_buffer_m,
          `${path}.detour_buffer_m`,
          LIMITS.max_avoidance_buffer_m,
        )
  return { enabled, clearance_margin_m, lookahead_s, detour_buffer_m }
}

/**
 * Validate an authored scenario document and return it complete.
 *
 * Phase 8 mission authoring writes a sparse draft: every field but `id` may
 * be missing, and the core fills each gap with its scenario defaults. This is
 * the browser's copy of that contract. Missing fields come from
 * `SCENARIO_DEFAULTS`; present fields are checked against the same `LIMITS`
 * the core enforces, so the editor and the runner agree on what is runnable.
 * The result carries every field, so the Run view never has to guess. The
 * `id` is the one field that must be authored: a lowercase slug
 * (`^[a-z][a-z0-9_]{0,63}$`) that is not a reserved shipped scenario, since
 * it becomes the recording's `meta.scenario` and file name. Keep-out zones
 * carry their own id rule: a lowercase slug (`^[a-z0-9_]{1,64}$`) used once
 * per list, so a message naming a zone points at exactly one of them.
 *
 * Authored faults are checked against this scenario's own duration: a fault
 * fires when the clock reaches it, so the last accepted fire time is the run's
 * last step clock, `(round(duration_s / dt) - 1) * dt`, inclusive.
 *
 * Throws naming the first bad path, e.g. `faults[0].at_s must be in [0, 29.995]`.
 */
export function validateScenarioConfig(value: unknown): ScenarioConfig {
  const c = asRecord(value, 'value')
  const id = asScenarioId(c.id, 'id')
  const label = asString(authored(c.label, SCENARIO_DEFAULTS.label), 'label')
  const blurb = asString(authored(c.blurb, SCENARIO_DEFAULTS.blurb), 'blurb')
  const duration_s = asPositiveUpTo(
    authored(c.duration_s, SCENARIO_DEFAULTS.duration_s),
    'duration_s',
    LIMITS.max_duration_s,
  )
  const takeoff_altitude = asPositiveUpTo(
    authored(c.takeoff_altitude, SCENARIO_DEFAULTS.takeoff_altitude),
    'takeoff_altitude',
    LIMITS.max_takeoff_altitude_m,
  )
  const start_position_ned = asBoundedVec3(
    authored(c.start_position_ned, SCENARIO_DEFAULTS.start_position_ned),
    'start_position_ned',
  )
  const waypoints = asBoundedArray(
    authored(c.waypoints, SCENARIO_DEFAULTS.waypoints),
    'waypoints',
    LIMITS.max_waypoints,
  ).map((entry, i) => validateWaypoint(entry, `waypoints[${i}]`))
  const obstacles = asBoundedArray(
    authored(c.obstacles, SCENARIO_DEFAULTS.obstacles),
    'obstacles',
    LIMITS.max_obstacles,
  ).map((entry, i) => validateObstacle(entry, `obstacles[${i}]`))
  checkObstacleIds(obstacles, 'obstacles')
  const avoidance = validateScenarioAvoidance(c.avoidance, 'avoidance')
  const wind_sigma = asSigmaVec3(authored(c.wind_sigma, SCENARIO_DEFAULTS.wind_sigma), 'wind_sigma')
  const wind_steady_ned = asBoundedVec3(
    authored(c.wind_steady_ned, SCENARIO_DEFAULTS.wind_steady_ned),
    'wind_steady_ned',
    LIMITS.max_wind_component_mps,
  )
  const battery_initial_soc = asUnitRange(
    authored(c.battery_initial_soc, SCENARIO_DEFAULTS.battery_initial_soc),
    'battery_initial_soc',
  )
  const gps_position_noise = asNonNegativeUpTo(
    authored(c.gps_position_noise, SCENARIO_DEFAULTS.gps_position_noise),
    'gps_position_noise',
    LIMITS.max_gps_position_noise_m,
  )
  const gps_position_noise_vertical = asNonNegativeUpTo(
    authored(c.gps_position_noise_vertical, SCENARIO_DEFAULTS.gps_position_noise_vertical),
    'gps_position_noise_vertical',
    LIMITS.max_gps_position_noise_m,
  )
  const gps_velocity_noise = asNonNegativeUpTo(
    authored(c.gps_velocity_noise, SCENARIO_DEFAULTS.gps_velocity_noise),
    'gps_velocity_noise',
    LIMITS.max_gps_velocity_noise_mps,
  )
  const cruise_hold_s = asNonNegativeUpTo(
    authored(c.cruise_hold_s, SCENARIO_DEFAULTS.cruise_hold_s),
    'cruise_hold_s',
    LIMITS.max_cruise_hold_s,
  )
  const seed = asSafeSeed(authored(c.seed, SCENARIO_DEFAULTS.seed), 'seed')
  const auto_arm = asBoolean(authored(c.auto_arm, SCENARIO_DEFAULTS.auto_arm), 'auto_arm')
  const faults = validateFaults(authored(c.faults, SCENARIO_DEFAULTS.faults), 'faults', duration_s)

  return {
    id,
    label,
    blurb,
    duration_s,
    takeoff_altitude,
    start_position_ned,
    waypoints,
    obstacles,
    avoidance,
    wind_sigma,
    wind_steady_ned,
    battery_initial_soc,
    gps_position_noise,
    gps_position_noise_vertical,
    gps_velocity_noise,
    cruise_hold_s,
    seed,
    auto_arm,
    faults,
  }
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
