/**
 * Mission editor model (FlightLab Phase 8c). Pure data and pure functions:
 * no React, no DOM, no localStorage, no fetch. The Zustand store wraps these
 * with persistence and validation; the editor UI dispatches `EditorCommand`s
 * and renders the resulting `MissionConfig`.
 *
 * Three ideas run through the module:
 *
 * - `MissionConfig` is the normalized authored shape: every field of the
 *   generated `ScenarioConfig` required, the avoidance block complete, the
 *   fault list always present. The store and the UI never branch on optional
 *   fields because this type has none.
 * - `EditorHistory` is a present config plus undo/redo stacks of the same
 *   shape. `applyCommand` is immutable and never mutates its input, so the
 *   past entries can be the previous presents themselves. A command whose
 *   value already equals what the config holds is a no-op that returns the
 *   history unchanged, so tabbing through a field or re-issuing the same
 *   edit cannot pollute the undo stack or clear the redo stack.
 * - Altitudes are authored as "meters above the ground" for the UI while the
 *   schema stores NED z (negative up). `waypointAltitudeM` and
 *   `withWaypointAltitude` (and the obstacle pair) are the only sanctioned
 *   conversion, so the sign convention lives in one place.
 *
 * The estimates here (`routeDistanceM`, `estimateFlightTimeS`) are derived
 * only from authored data: they never step the simulator, and they are
 * labelled as estimates wherever they are shown.
 */

import { RESERVED_SCENARIO_IDS } from '@/data/validate'
import type { Vec3 } from '@/types/flight'
import type { FaultSpec, Obstacle, ScenarioConfig, Waypoint } from '@/types/generated/talon_core'
import { LIMITS, SCENARIO_DEFAULTS } from '@/types/generated/talon_core'
import { obstacleClearance } from './clearance'

// ---------------------------------------------------------------------------
// Normalized config
// ---------------------------------------------------------------------------

/** The generated defaults read as the normalized shape (the object is complete). */
const DEFAULTS = SCENARIO_DEFAULTS as Required<ScenarioConfig> & {
  avoidance: Required<NonNullable<ScenarioConfig['avoidance']>>
}

/**
 * The authored mission: every generated field required, the avoidance block
 * complete, and the fault list always present (the generated field is
 * optional). Assignable to `ScenarioConfig`, so it validates and runs as-is.
 */
export type MissionConfig = Required<ScenarioConfig> & {
  avoidance: Required<NonNullable<ScenarioConfig['avoidance']>>
  faults: FaultSpec[]
}

function cloneVec3(v: Vec3): Vec3 {
  return [v[0], v[1], v[2]]
}

/** A deep copy of one waypoint (position is a mutable tuple on the wire). */
export function cloneWaypoint(wp: Waypoint): Waypoint {
  return {
    position: cloneVec3(wp.position),
    speed: wp.speed,
    heading: wp.heading,
    radius: wp.radius,
  }
}

/** A deep copy of one keep-out zone. */
export function cloneObstacle(ob: Obstacle): Obstacle {
  return { id: ob.id, center_ned: cloneVec3(ob.center_ned), radius_m: ob.radius_m }
}

/** A deep copy of one fault spec (the variants are plain records). */
export function cloneFault(fault: FaultSpec): FaultSpec {
  return { ...fault }
}

/** A deep copy of a whole mission: no array, tuple, or fault is shared. */
export function cloneMission(config: MissionConfig): MissionConfig {
  return {
    ...config,
    start_position_ned: cloneVec3(config.start_position_ned),
    waypoints: config.waypoints.map(cloneWaypoint),
    obstacles: config.obstacles.map(cloneObstacle),
    avoidance: { ...config.avoidance },
    wind_sigma: cloneVec3(config.wind_sigma),
    wind_steady_ned: cloneVec3(config.wind_steady_ned),
    faults: config.faults.map(cloneFault),
  }
}

/**
 * Normalize any authored or shipped config into the editor shape. Absent
 * fields take the generated defaults (the same fills
 * `validateScenarioConfig` applies), arrays are deep-cloned so the result
 * shares nothing with the input, and authored values are preserved as-is.
 */
export function toMissionConfig(config: ScenarioConfig): MissionConfig {
  const d = DEFAULTS
  return {
    id: config.id,
    label: config.label ?? d.label,
    blurb: config.blurb ?? d.blurb,
    duration_s: config.duration_s ?? d.duration_s,
    takeoff_altitude: config.takeoff_altitude ?? d.takeoff_altitude,
    start_position_ned: cloneVec3(config.start_position_ned ?? d.start_position_ned),
    waypoints: (config.waypoints ?? []).map(cloneWaypoint),
    obstacles: (config.obstacles ?? []).map(cloneObstacle),
    avoidance: {
      enabled: config.avoidance?.enabled ?? d.avoidance.enabled,
      clearance_margin_m:
        config.avoidance?.clearance_margin_m ?? d.avoidance.clearance_margin_m,
      lookahead_s: config.avoidance?.lookahead_s ?? d.avoidance.lookahead_s,
      detour_buffer_m: config.avoidance?.detour_buffer_m ?? d.avoidance.detour_buffer_m,
    },
    wind_sigma: cloneVec3(config.wind_sigma ?? d.wind_sigma),
    wind_steady_ned: cloneVec3(config.wind_steady_ned ?? d.wind_steady_ned),
    battery_initial_soc: config.battery_initial_soc ?? d.battery_initial_soc,
    gps_position_noise: config.gps_position_noise ?? d.gps_position_noise,
    gps_position_noise_vertical:
      config.gps_position_noise_vertical ?? d.gps_position_noise_vertical,
    gps_velocity_noise: config.gps_velocity_noise ?? d.gps_velocity_noise,
    cruise_hold_s: config.cruise_hold_s ?? d.cruise_hold_s,
    seed: config.seed ?? d.seed,
    auto_arm: config.auto_arm ?? d.auto_arm,
    faults: (config.faults ?? []).map(cloneFault),
  }
}

/**
 * A blank mission: the generated defaults, armed, unlabelled, no route, no
 * zones, avoidance off, no faults. The starting point of every new draft and
 * the fallback whenever a persisted draft fails to load.
 */
export function emptyMission(id = 'custom_mission'): MissionConfig {
  const d = DEFAULTS
  return {
    id,
    label: 'Untitled mission',
    blurb: '',
    duration_s: d.duration_s,
    takeoff_altitude: d.takeoff_altitude,
    start_position_ned: cloneVec3(d.start_position_ned),
    waypoints: [],
    obstacles: [],
    avoidance: { ...d.avoidance, enabled: false },
    wind_sigma: cloneVec3(d.wind_sigma),
    wind_steady_ned: cloneVec3(d.wind_steady_ned),
    battery_initial_soc: d.battery_initial_soc,
    gps_position_noise: d.gps_position_noise,
    gps_position_noise_vertical: d.gps_position_noise_vertical,
    gps_velocity_noise: d.gps_velocity_noise,
    cruise_hold_s: d.cruise_hold_s,
    seed: d.seed,
    auto_arm: true,
    faults: [],
  }
}

/**
 * Clone a shipped live config or an imported config into a new editable
 * mission under a fresh id, armed. The input is never mutated and nothing is
 * shared with the result.
 */
export function cloneAsTemplate(config: ScenarioConfig, id: string): MissionConfig {
  return { ...toMissionConfig(config), id, auto_arm: true }
}

// ---------------------------------------------------------------------------
// Altitude conversions (NED z is negative up)
// ---------------------------------------------------------------------------

/** Altitude (m above the ground) of a waypoint: NED z is negative up. */
export function waypointAltitudeM(wp: Waypoint): number {
  const altitude = -wp.position[2]
  return altitude === 0 ? 0 : altitude // keep negative zero out of the readout
}

/** The same waypoint at a new altitude, every other field preserved. */
export function withWaypointAltitude(wp: Waypoint, altitude: number): Waypoint {
  return { ...wp, position: [wp.position[0], wp.position[1], -altitude] }
}

/** Altitude (m above the ground) of a keep-out zone center. */
export function obstacleAltitudeM(ob: Obstacle): number {
  const altitude = -ob.center_ned[2]
  return altitude === 0 ? 0 : altitude // keep negative zero out of the readout
}

/** The same zone at a new altitude, every other field preserved. */
export function withObstacleAltitude(ob: Obstacle, altitude: number): Obstacle {
  return { ...ob, center_ned: [ob.center_ned[0], ob.center_ned[1], -altitude] }
}

// ---------------------------------------------------------------------------
// Estimates (derived from authored data only)
// ---------------------------------------------------------------------------

/**
 * Planned route distance (m): start to the first waypoint, then leg to leg,
 * horizontal only (altitude changes are flown by the climb, not the route).
 * Zero when the mission has no waypoints.
 */
export function routeDistanceM(config: ScenarioConfig): number {
  const waypoints = config.waypoints ?? []
  if (waypoints.length === 0) return 0
  const start = config.start_position_ned ?? [0, 0, 0]
  let prev = start
  let total = 0
  for (const wp of waypoints) {
    total += Math.hypot(wp.position[0] - prev[0], wp.position[1] - prev[1])
    prev = wp.position
  }
  return total
}

/**
 * Estimated flight time (s), an estimate derived only from authored data
 * (never a simulator step): climb at 2 m/s to the takeoff altitude, then
 * each leg at its waypoint speed (floor 1 m/s), plus the cruise hold at
 * every waypoint and once after takeoff. A planning hint for the editor,
 * not a promise about the run.
 */
export function estimateFlightTimeS(config: ScenarioConfig): number {
  const takeoffAltitude = config.takeoff_altitude ?? SCENARIO_DEFAULTS.takeoff_altitude ?? 0
  const hold = config.cruise_hold_s ?? 0
  const waypoints = config.waypoints ?? []
  const start = config.start_position_ned ?? [0, 0, 0]
  let total = takeoffAltitude / 2.0 + hold
  let prev = start
  for (const wp of waypoints) {
    const speed = Math.max(1, wp.speed)
    total += Math.hypot(wp.position[0] - prev[0], wp.position[1] - prev[1]) / speed + hold
    prev = wp.position
  }
  return total
}

// ---------------------------------------------------------------------------
// Fault window (the single source for the UI and the validator's rule)
// ---------------------------------------------------------------------------

/**
 * The sim time of the last step of a run of `durationS` at step `dt`: the
 * latest clock a fault still has a step to act on, or `null` when the run
 * rounds to fewer than two steps and has no window at all. Exactly the rule
 * the Rust runner and `validateScenarioConfig` use: the step count is
 * `max(1, round(duration / dt))`, and step `i` starts at clock `i * dt`, so
 * the last step's own clock is a valid fire time. The editor UI must call
 * this instead of re-deriving the window.
 */
export function faultWindowEndS(durationS: number, dt: number): number | null {
  const steps = Math.max(1, Math.round(durationS / dt))
  if (steps < 2) return null
  return (steps - 1) * dt
}

// ---------------------------------------------------------------------------
// Warnings (never blocking; errors live in the validator)
// ---------------------------------------------------------------------------

/** Horizontal distance (m) at which two consecutive waypoints count as a repeat. */
const DUPLICATE_EPSILON_M = 0.1

/**
 * One non-blocking warning about a waypoint, in product language, or null.
 * Checks, in order: the waypoint inside a keep-out zone, a duplicate of the
 * waypoint before it (same north/east within 0.1 m), and an acceptance
 * radius that already covers the waypoint before it. `index` is zero-based;
 * the messages are one-based, the way the route list is numbered.
 */
export function waypointWarning(config: ScenarioConfig, index: number): string | null {
  const waypoints = config.waypoints ?? []
  const wp = waypoints[index]
  if (wp === undefined) return null
  const n = index + 1
  for (const zone of config.obstacles ?? []) {
    if (obstacleClearance(wp.position, zone) <= 0) {
      return `Waypoint ${n} is inside keep-out zone ${zone.id}`
    }
  }
  const prev = index > 0 ? waypoints[index - 1] : undefined
  if (prev !== undefined) {
    const leg = Math.hypot(wp.position[0] - prev.position[0], wp.position[1] - prev.position[1])
    if (leg <= DUPLICATE_EPSILON_M) {
      return `Waypoint ${n} repeats waypoint ${index}`
    }
    if (wp.radius >= leg) {
      return `Waypoint ${n}'s radius covers waypoint ${index}`
    }
  }
  return null
}

/**
 * Horizontal distance (m) from a point to a segment, in the north/east plane.
 * The projection onto the line is clamped to the segment, so a point level
 * with an end measures to that end, and a degenerate segment (both ends the
 * same point) measures to the point. Altitude is ignored: this is how a plan
 * view reads the distance to a leg.
 */
export function pointToSegmentDistanceM(point: Vec3, from: Vec3, to: Vec3): number {
  const px = point[0]
  const py = point[1]
  const ax = from[0]
  const ay = from[1]
  const dx = to[0] - ax
  const dy = to[1] - ay
  const lengthSquared = dx * dx + dy * dy
  const t =
    lengthSquared === 0
      ? 0
      : Math.min(1, Math.max(0, ((px - ax) * dx + (py - ay) * dy) / lengthSquared))
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy))
}

/**
 * Every non-blocking warning for a mission, in product language: one entry
 * per waypoint that has a `waypointWarning`, then, while reactive avoidance
 * is on, one entry per keep-out zone standing close to a planned leg.
 *
 * The zone check is geometry, not a guarantee: it fires whenever the zone's
 * center sits within `radius + clearance margin + detour buffer` of a leg,
 * the same standoff the bypass itself plans to keep. A bypass that wide can
 * carry the vehicle past the waypoint the leg leads to, so the message says
 * so and offers the two fixes (a larger acceptance radius, or moving the
 * zone) instead of promising a problem. Each zone and leg pair is warned
 * about at most once, even if a zone is listed twice.
 */
export function missionWarnings(config: ScenarioConfig): string[] {
  const warnings: string[] = []
  const waypoints = config.waypoints ?? []
  for (let i = 0; i < waypoints.length; i += 1) {
    const warning = waypointWarning(config, i)
    if (warning !== null) warnings.push(warning)
  }

  const avoidance = config.avoidance
  if (avoidance?.enabled !== true) return warnings
  const margin = avoidance.clearance_margin_m ?? DEFAULTS.avoidance.clearance_margin_m
  const buffer = avoidance.detour_buffer_m ?? DEFAULTS.avoidance.detour_buffer_m
  const obstacles = config.obstacles ?? []
  if (obstacles.length === 0) return warnings

  const seen = new Set(warnings)
  let from: Vec3 = config.start_position_ned ?? DEFAULTS.start_position_ned
  for (let i = 0; i < waypoints.length; i += 1) {
    const to = waypoints[i]
    if (to === undefined) break
    for (const zone of obstacles) {
      const standoff = zone.radius_m + margin + buffer
      if (pointToSegmentDistanceM(zone.center_ned, from, to.position) >= standoff) continue
      const n = i + 1
      const warning =
        `Keep-out zone ${zone.id} is close to the leg to waypoint ${n}. ` +
        `The bypass may carry the vehicle past waypoint ${n}: use a larger acceptance radius ` +
        `or move the zone.`
      if (!seen.has(warning)) {
        seen.add(warning)
        warnings.push(warning)
      }
    }
    from = to.position
  }
  return warnings
}

// ---------------------------------------------------------------------------
// Ids
// ---------------------------------------------------------------------------

/**
 * The next free `custom_mission` id: `custom_mission`, then `custom_mission_2`,
 * `custom_mission_3`, ... skipping both the ids the caller already uses and
 * the reserved ids the validator rejects (the shipped scenarios and the live
 * run slot).
 */
export function nextMissionId(used: Iterable<string>): string {
  const taken = new Set(used)
  const conflicts = (id: string): boolean =>
    taken.has(id) || (RESERVED_SCENARIO_IDS as readonly string[]).includes(id)
  if (!conflicts('custom_mission')) return 'custom_mission'
  for (let i = 2; ; i += 1) {
    const id = `custom_mission_${i}`
    if (!conflicts(id)) return id
  }
}

/** Smallest `zone_N` id not already taken by an existing zone. */
function uniqueZoneId(obstacles: readonly Obstacle[]): string {
  const taken = new Set(obstacles.map((o) => o.id))
  for (let n = 1; ; n += 1) {
    const id = `zone_${n}`
    if (!taken.has(id)) return id
  }
}

/**
 * Clamp a horizontal coordinate (north or east, m) to the cap
 * `validateScenarioConfig` enforces. Every path that authors a coordinate
 * from outside the config (the chart's double-click add, drags and nudges,
 * the waypoint panel's Add button) routes through this one rule, so they
 * agree on the bound no matter where the number came from.
 */
export function clampCoordinate(value: number): number {
  return Math.min(Math.max(value, -LIMITS.max_coordinate_m), LIMITS.max_coordinate_m)
}

// ---------------------------------------------------------------------------
// Selection and commands
// ---------------------------------------------------------------------------

/** What the editor is pointing at: one waypoint or one zone by list index. */
export type EditorSelection = { kind: 'waypoint' | 'obstacle'; index: number } | null

/** Default leg speed (m/s) for a waypoint the editor adds. */
const DEFAULT_WAYPOINT_SPEED = 5.0

/** Default acceptance radius (m) for a waypoint the editor adds. */
const DEFAULT_WAYPOINT_RADIUS = 2.0

/** Default radius (m) for a keep-out zone the editor adds. */
const DEFAULT_OBSTACLE_RADIUS = 3.0

/**
 * Every edit the UI can express, as a discriminated union with narrow
 * payloads. Commands are the only way the store's config changes, so the
 * undo history and the persistence hook see exactly the user's actions.
 */
export type EditorCommand =
  // Waypoints. Move carries horizontal position only: altitude is preserved.
  | { type: 'waypoint/add'; north: number; east: number; altitude?: number; speed?: number; index?: number }
  | { type: 'waypoint/move'; index: number; north: number; east: number }
  | { type: 'waypoint/set'; index: number; waypoint: Waypoint }
  | { type: 'waypoint/remove'; index: number }
  | { type: 'waypoint/reorder'; index: number; delta: -1 | 1 }
  // Keep-out zones. Move keeps the zone's altitude.
  | { type: 'obstacle/add'; north: number; east: number; altitude?: number; radius_m?: number }
  | { type: 'obstacle/move'; index: number; north: number; east: number }
  | { type: 'obstacle/resize'; index: number; radius_m: number }
  | { type: 'obstacle/set'; index: number; obstacle: Obstacle }
  | { type: 'obstacle/remove'; index: number }
  // Wind and avoidance.
  | { type: 'wind/steady'; ned: Vec3 }
  | { type: 'wind/sigma'; ned: Vec3 }
  | { type: 'avoidance/enabled'; enabled: boolean }
  | {
      type: 'avoidance/tuning'
      clearance_margin_m?: number
      lookahead_s?: number
      detour_buffer_m?: number
    }
  // Scalar params, narrowed per key group so each payload keeps its type.
  | { type: 'param/set'; key: 'label' | 'blurb'; value: string }
  | {
      type: 'param/set'
      key:
        | 'duration_s'
        | 'takeoff_altitude'
        | 'battery_initial_soc'
        | 'gps_position_noise'
        | 'gps_position_noise_vertical'
        | 'gps_velocity_noise'
        | 'cruise_hold_s'
        | 'seed'
      value: number
    }
  | { type: 'param/set'; key: 'auto_arm'; value: boolean }
  // Whole-list and whole-config changes.
  | { type: 'faults/set'; faults: FaultSpec[] }
  | { type: 'reset'; config: MissionConfig }

/** Undo/redo state: the current mission plus the stacks of previous ones. */
export interface EditorHistory {
  present: MissionConfig
  past: MissionConfig[]
  future: MissionConfig[]
}

/** Most entries kept in the undo stack. */
export const HISTORY_CAP = 100

/** A fresh history holding one config, with empty undo/redo stacks. */
export function createHistory(config: MissionConfig): EditorHistory {
  return { present: config, past: [], future: [] }
}

function withWaypointAt(cfg: MissionConfig, index: number, wp: Waypoint): MissionConfig {
  const waypoints = [...cfg.waypoints]
  waypoints[index] = wp
  return { ...cfg, waypoints }
}

function withoutWaypointAt(cfg: MissionConfig, index: number): MissionConfig {
  if (index < 0 || index >= cfg.waypoints.length) return cfg
  const waypoints = [...cfg.waypoints]
  waypoints.splice(index, 1)
  return { ...cfg, waypoints }
}

// ---------------------------------------------------------------------------
// Value equality (an equal-value command must not manufacture an undo entry)
// ---------------------------------------------------------------------------

/** True when two NED triples hold the same numbers. */
function vec3Equal(a: Vec3, b: Vec3): boolean {
  return a[0] === b[0] && a[1] === b[1] && a[2] === b[2]
}

/** True when two waypoints are the same on every field, including altitude. */
function waypointEqual(a: Waypoint, b: Waypoint): boolean {
  return (
    vec3Equal(a.position, b.position) &&
    a.speed === b.speed &&
    a.heading === b.heading &&
    a.radius === b.radius
  )
}

/** True when two zones are the same id, center (altitude included), radius. */
function obstacleEqual(a: Obstacle, b: Obstacle): boolean {
  return a.id === b.id && vec3Equal(a.center_ned, b.center_ned) && a.radius_m === b.radius_m
}

/**
 * True when two faults match field by field. The variants carry one to three
 * fields each, so the comparison is key-order independent and a rebuilt list
 * compares equal to the stored copy (lists are capped at 16 entries, which
 * keeps this cheap). Extra or missing fields make the comparison fail.
 */
function faultEqual(a: FaultSpec, b: FaultSpec): boolean {
  const left = a as Record<string, unknown>
  const right = b as Record<string, unknown>
  const keys = Object.keys(left)
  if (keys.length !== Object.keys(right).length) return false
  return keys.every((key) => left[key] === right[key])
}

/** True when two lists are the same length and pair up entry by entry. */
function listEqual<T>(a: readonly T[], b: readonly T[], equal: (x: T, y: T) => boolean): boolean {
  if (a.length !== b.length) return false
  return a.every((item, index) => {
    const other = b[index]
    return other !== undefined && equal(item, other)
  })
}

/**
 * True when two missions hold the same values on every field of the editor
 * shape, whatever the key order. Used by `reset` so re-issuing the config
 * that is already loaded is a no-op instead of an undo entry: the comparison
 * runs before the clone is made.
 */
function missionConfigEqual(a: MissionConfig, b: MissionConfig): boolean {
  const left = a as Record<string, unknown>
  const right = b as Record<string, unknown>
  if (Object.keys(left).length !== Object.keys(right).length) return false
  return (
    a.id === b.id &&
    a.label === b.label &&
    a.blurb === b.blurb &&
    a.duration_s === b.duration_s &&
    a.takeoff_altitude === b.takeoff_altitude &&
    a.battery_initial_soc === b.battery_initial_soc &&
    a.gps_position_noise === b.gps_position_noise &&
    a.gps_position_noise_vertical === b.gps_position_noise_vertical &&
    a.gps_velocity_noise === b.gps_velocity_noise &&
    a.cruise_hold_s === b.cruise_hold_s &&
    a.seed === b.seed &&
    a.auto_arm === b.auto_arm &&
    vec3Equal(a.start_position_ned, b.start_position_ned) &&
    vec3Equal(a.wind_sigma, b.wind_sigma) &&
    vec3Equal(a.wind_steady_ned, b.wind_steady_ned) &&
    a.avoidance.enabled === b.avoidance.enabled &&
    a.avoidance.clearance_margin_m === b.avoidance.clearance_margin_m &&
    a.avoidance.lookahead_s === b.avoidance.lookahead_s &&
    a.avoidance.detour_buffer_m === b.avoidance.detour_buffer_m &&
    listEqual(a.waypoints, b.waypoints, waypointEqual) &&
    listEqual(a.obstacles, b.obstacles, obstacleEqual) &&
    listEqual(a.faults, b.faults, faultEqual)
  )
}

/**
 * Apply one command to the mission, returning the next mission. The input is
 * never mutated; a command that changes nothing (an index out of range, a
 * reorder at the end of the list, or an edit whose value already equals what
 * the config holds) returns the same reference, so `applyCommand` can pass
 * the whole history through untouched.
 */
function applyToConfig(cfg: MissionConfig, command: EditorCommand): MissionConfig {
  switch (command.type) {
    case 'waypoint/add': {
      const count = cfg.waypoints.length
      const index = command.index ?? count
      if (index < 0 || index > count) return cfg
      const waypoint: Waypoint = {
        position: [command.north, command.east, -(command.altitude ?? cfg.takeoff_altitude)],
        speed: command.speed ?? DEFAULT_WAYPOINT_SPEED,
        heading: 0,
        radius: DEFAULT_WAYPOINT_RADIUS,
      }
      const waypoints = [...cfg.waypoints]
      waypoints.splice(index, 0, waypoint)
      return { ...cfg, waypoints }
    }
    case 'waypoint/move': {
      const wp = cfg.waypoints[command.index]
      if (wp === undefined) return cfg
      if (wp.position[0] === command.north && wp.position[1] === command.east) return cfg
      return withWaypointAt(cfg, command.index, {
        ...wp,
        position: [command.north, command.east, wp.position[2]],
      })
    }
    case 'waypoint/set': {
      const current = cfg.waypoints[command.index]
      if (current === undefined) return cfg
      if (waypointEqual(current, command.waypoint)) return cfg
      return withWaypointAt(cfg, command.index, cloneWaypoint(command.waypoint))
    }
    case 'waypoint/remove':
      return withoutWaypointAt(cfg, command.index)
    case 'waypoint/reorder': {
      const target = command.index + command.delta
      const wp = cfg.waypoints[command.index]
      const swap = cfg.waypoints[target]
      if (wp === undefined || swap === undefined) return cfg
      const waypoints = [...cfg.waypoints]
      waypoints[command.index] = swap
      waypoints[target] = wp
      return { ...cfg, waypoints }
    }
    case 'obstacle/add': {
      const obstacle: Obstacle = {
        id: uniqueZoneId(cfg.obstacles),
        center_ned: [command.north, command.east, -(command.altitude ?? cfg.takeoff_altitude)],
        radius_m: command.radius_m ?? DEFAULT_OBSTACLE_RADIUS,
      }
      return { ...cfg, obstacles: [...cfg.obstacles, obstacle] }
    }
    case 'obstacle/move': {
      const ob = cfg.obstacles[command.index]
      if (ob === undefined) return cfg
      if (ob.center_ned[0] === command.north && ob.center_ned[1] === command.east) return cfg
      const obstacles = [...cfg.obstacles]
      obstacles[command.index] = {
        ...ob,
        center_ned: [command.north, command.east, ob.center_ned[2]],
      }
      return { ...cfg, obstacles }
    }
    case 'obstacle/resize': {
      const ob = cfg.obstacles[command.index]
      if (ob === undefined) return cfg
      if (ob.radius_m === command.radius_m) return cfg
      const obstacles = [...cfg.obstacles]
      obstacles[command.index] = { ...ob, radius_m: command.radius_m }
      return { ...cfg, obstacles }
    }
    case 'obstacle/set': {
      const current = cfg.obstacles[command.index]
      if (current === undefined) return cfg
      if (obstacleEqual(current, command.obstacle)) return cfg
      const obstacles = [...cfg.obstacles]
      obstacles[command.index] = cloneObstacle(command.obstacle)
      return { ...cfg, obstacles }
    }
    case 'obstacle/remove': {
      if (command.index < 0 || command.index >= cfg.obstacles.length) return cfg
      const obstacles = [...cfg.obstacles]
      obstacles.splice(command.index, 1)
      return { ...cfg, obstacles }
    }
    case 'wind/steady':
      if (vec3Equal(cfg.wind_steady_ned, command.ned)) return cfg
      return { ...cfg, wind_steady_ned: cloneVec3(command.ned) }
    case 'wind/sigma':
      if (vec3Equal(cfg.wind_sigma, command.ned)) return cfg
      return { ...cfg, wind_sigma: cloneVec3(command.ned) }
    case 'avoidance/enabled':
      if (cfg.avoidance.enabled === command.enabled) return cfg
      return { ...cfg, avoidance: { ...cfg.avoidance, enabled: command.enabled } }
    case 'avoidance/tuning': {
      // Only the fields that actually differ go into the patch: a patch with
      // nothing left is a value-equal command and must return the input.
      const tunings: Partial<MissionConfig['avoidance']> = {}
      if (
        command.clearance_margin_m !== undefined &&
        command.clearance_margin_m !== cfg.avoidance.clearance_margin_m
      ) {
        tunings.clearance_margin_m = command.clearance_margin_m
      }
      if (command.lookahead_s !== undefined && command.lookahead_s !== cfg.avoidance.lookahead_s) {
        tunings.lookahead_s = command.lookahead_s
      }
      if (
        command.detour_buffer_m !== undefined &&
        command.detour_buffer_m !== cfg.avoidance.detour_buffer_m
      ) {
        tunings.detour_buffer_m = command.detour_buffer_m
      }
      if (Object.keys(tunings).length === 0) return cfg
      return { ...cfg, avoidance: { ...cfg.avoidance, ...tunings } }
    }
    case 'param/set':
      if (cfg[command.key] === command.value) return cfg
      switch (command.key) {
        case 'label':
          return { ...cfg, label: command.value }
        case 'blurb':
          return { ...cfg, blurb: command.value }
        case 'duration_s':
          return { ...cfg, duration_s: command.value }
        case 'takeoff_altitude':
          return { ...cfg, takeoff_altitude: command.value }
        case 'battery_initial_soc':
          return { ...cfg, battery_initial_soc: command.value }
        case 'gps_position_noise':
          return { ...cfg, gps_position_noise: command.value }
        case 'gps_position_noise_vertical':
          return { ...cfg, gps_position_noise_vertical: command.value }
        case 'gps_velocity_noise':
          return { ...cfg, gps_velocity_noise: command.value }
        case 'cruise_hold_s':
          return { ...cfg, cruise_hold_s: command.value }
        case 'seed':
          return { ...cfg, seed: command.value }
        case 'auto_arm':
          return { ...cfg, auto_arm: command.value }
        default:
          return cfg
      }
    case 'faults/set': {
      const next = command.faults
      const unchanged =
        cfg.faults.length === next.length &&
        cfg.faults.every((fault, index) => {
          const incoming = next[index]
          return incoming !== undefined && faultEqual(fault, incoming)
        })
      if (unchanged) return cfg
      return { ...cfg, faults: next.map(cloneFault) }
    }
    case 'reset':
      // A reset to the config already loaded changes nothing: compare the
      // values first so a no-op returns the present untouched, instead of
      // cloning it and pushing an undo entry that undoes nothing.
      return missionConfigEqual(cfg, command.config) ? cfg : cloneMission(command.config)
  }
}

/**
 * Push one command through the history: the present becomes the command's
 * result, the previous present is appended to `past` (capped at
 * `HISTORY_CAP`), and `future` is cleared. A command that changes nothing,
 * including an edit whose value already equals what the config holds, returns
 * the same history object: no undo entry is manufactured and the redo stack
 * survives, and the store skips the commit (recompute and persist) because it
 * sees the identical reference. Selection lives in the store, outside this
 * structure, and is preserved by construction.
 */
export function applyCommand(history: EditorHistory, command: EditorCommand): EditorHistory {
  const present = applyToConfig(history.present, command)
  if (present === history.present) return history
  return {
    present,
    past: [...history.past, history.present].slice(-HISTORY_CAP),
    future: [],
  }
}

/** Step the history back one edit, or return it unchanged at the start. */
export function undo(history: EditorHistory): EditorHistory {
  const previous = history.past[history.past.length - 1]
  if (previous === undefined) return history
  return {
    present: previous,
    past: history.past.slice(0, -1),
    future: [history.present, ...history.future],
  }
}

/** Step the history forward one undone edit, or return it unchanged. */
export function redo(history: EditorHistory): EditorHistory {
  const [next, ...rest] = history.future
  if (next === undefined) return history
  return {
    present: next,
    past: [...history.past, history.present].slice(-HISTORY_CAP),
    future: rest,
  }
}

/** True when `undo` would step back an edit. */
export function canUndo(history: EditorHistory): boolean {
  return history.past.length > 0
}

/** True when `redo` would step forward an undone edit. */
export function canRedo(history: EditorHistory): boolean {
  return history.future.length > 0
}
