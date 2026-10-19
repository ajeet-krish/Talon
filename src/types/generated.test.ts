/**
 * Drift guard for the generated TypeScript mirrors (Phase 5c, D23).
 *
 * `src/types/generated/talon_core.ts` is produced by ts-rs from the Rust
 * source of truth (`core/src/`) via `./scripts/gen_types.sh`. This test reads
 * it from disk and pins the properties the facades and the app depend on: the
 * banner, the shape of `FlightPhase`, the `phase_timeline` tuple, the
 * `number`-not-`bigint` treatment of the integer fields, the fixed-tuple NED
 * triple fields, the `RunSummary` field list with its number counters and
 * nullable clearance, the optionality of the serde-defaulted
 * `ScenarioConfig` fields, and the parsed `SCENARIO_DEFAULTS` literal.
 *
 * It is a text check, not a TypeScript parser: it only has to hold for the
 * output ts-rs actually emits.
 */

import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'

const GENERATED = 'src/types/generated/talon_core.ts'
const SOURCE = readFileSync(GENERATED, 'utf8')

/** The declaration body of `header`, up to and including its terminator. */
function declBody(header: string): string {
  const start = SOURCE.indexOf(header)
  if (start < 0) throw new Error(`missing declaration: ${header}`)
  // Object declarations end with `};`. Union declarations (FlightPhase) have
  // no braces and end at their first `;`. Key the object case off the closing
  // brace so a `;` inside a doc comment does not truncate the declaration early
  // (FlightData's obstacles doc reads "this key; a missing list reads as
  // empty").
  const afterHeader = SOURCE.slice(start + header.length).trimStart()
  const end = afterHeader.startsWith('{')
    ? SOURCE.indexOf('};', start) + 2
    : SOURCE.indexOf(';', start) + 1
  if (end < start + 1) throw new Error(`unterminated declaration: ${header}`)
  return SOURCE.slice(start, end)
}

describe('generated talon-core mirrors', () => {
  it('carries the "do not edit" banner as the first line', () => {
    const first = SOURCE.split('\n')[0]
    expect(first).toBe('// GENERATED FILE. DO NOT EDIT. Run ./scripts/gen_types.sh.')
  })

  it('emits FlightPhase as a lowercase string union with every phase', () => {
    const body = declBody('export type FlightPhase =')
    // Split the union on `|`, dropping the trailing `;` terminator, and keep
    // the quoted members.
    const members = body
      .slice(body.indexOf('=') + 1)
      .replace(/;\s*$/, '')
      .split('|')
      .map((s) => s.trim())
      .filter((s) => s.startsWith('"') && s.endsWith('"'))
      .map((s) => s.slice(1, -1))

    expect(members).toContain('idle')
    expect(members).toContain('emergency')
    expect(members).toContain('waypoint')
    expect(members.length).toBeGreaterThanOrEqual(8)
    // Every member is lowercase: no uppercase letters anywhere.
    for (const m of members) {
      expect(m, `phase ${m} should be lowercase`).toBe(m.toLowerCase())
    }
  })

  it('types FlightData.phase_timeline as an array of [number, FlightPhase] tuples', () => {
    const data = declBody('export type FlightData =')
    expect(data).toContain('phase_timeline: Array<[number, FlightPhase]>')
  })

  it('types the integer fields as number, not bigint', () => {
    const meta = declBody('export type FlightMeta =')
    expect(meta).toMatch(/seed: number/)
    expect(meta).toMatch(/sample_count: number/)
    expect(meta).not.toMatch(/seed: bigint/)
    expect(meta).not.toMatch(/sample_count: bigint/)
  })

  it('marks the serde-defaulted ScenarioConfig fields optional and id required', () => {
    const cfg = declBody('export type ScenarioConfig =')
    // Required: only `id` (present as a plain field, never as `id?:`).
    expect(cfg).toContain('id: string')
    expect(cfg).not.toContain('id?:')
    // Optional where serde defaults them.
    for (const field of [
      'label',
      'blurb',
      'duration_s',
      'takeoff_altitude',
      'start_position_ned',
      'waypoints',
      'obstacles',
      'avoidance',
      'wind_sigma',
      'wind_steady_ned',
      'battery_initial_soc',
      'gps_position_noise',
      'gps_position_noise_vertical',
      'gps_velocity_noise',
      'cruise_hold_s',
      'seed',
      'auto_arm',
    ]) {
      expect(cfg, `${field} should be optional`).toContain(`${field}?:`)
    }
  })

  it('declares the NED triple fields as fixed-length tuples, not open arrays', () => {
    // ts-rs renders `[f64; 3]` as the tuple `[number, number, number]`; a
    // serde `Vec<f64>` would instead render `Array<number>`. The scene and
    // mission code index these as fixed triples, so pin the tuple form.
    const obstacle = declBody('export type Obstacle =')
    expect(obstacle).toContain('center_ned: [number, number, number]')
    expect(obstacle).not.toMatch(/center_ned: Array<number>/)
    expect(obstacle).not.toMatch(/center_ned: number\[\]/)

    const data = declBody('export type FlightData =')
    expect(data).toContain('home_ned: [number, number, number]')
    expect(data).not.toMatch(/home_ned: Array<number>/)
    expect(data).not.toMatch(/home_ned: number\[\]/)

    const cfg = declBody('export type ScenarioConfig =')
    expect(cfg).toContain('start_position_ned?: [number, number, number]')
    expect(cfg).not.toMatch(/start_position_ned\?: Array<number>/)
    expect(cfg).not.toMatch(/start_position_ned\?: number\[\]/)
  })

  it('emits RunSummary with number counters and a nullable clearance', () => {
    const summary = declBody('export type RunSummary =')
    // Every field the run report reads must be declared on the type, so a
    // schema drop in core shows up here instead of at the read site.
    for (const field of [
      'duration_s',
      'waypoint_count',
      'waypoints_reached',
      'mission_complete',
      'gps_accepted',
      'gps_gated',
      'gps_skipped',
      'final_phase',
      'final_truth_altitude_m',
      'ekf_converged',
      'ekf_diverged',
      'min_obstacle_clearance_m',
      'avoidance_activations',
      'steps_done',
      'total_steps',
      'ekf_position_rmse_m',
      'final_position_error_m',
      'phases',
    ]) {
      expect(summary, `${field} should be declared`).toContain(`${field}:`)
    }
    // The u64/usize counters are plain JSON numbers on the wire.
    expect(summary).toMatch(/waypoint_count: number/)
    expect(summary).toMatch(/waypoints_reached: number/)
    expect(summary).toMatch(/gps_accepted: number/)
    expect(summary).toMatch(/gps_gated: number/)
    expect(summary).toMatch(/gps_skipped: number/)
    expect(summary).toMatch(/avoidance_activations: number/)
    expect(summary).toMatch(/steps_done: number/)
    expect(summary).toMatch(/total_steps: number/)
    // No counter is declared as bigint (the doc comment may mention the word).
    expect(summary).not.toMatch(/: bigint/)
    // The clearance is nullable: null when the scenario had no zones.
    expect(summary).toMatch(/min_obstacle_clearance_m: number \| null/)
    // The estimator metrics and the phase list.
    expect(summary).toMatch(/ekf_position_rmse_m: number/)
    expect(summary).toMatch(/final_position_error_m: number/)
    expect(summary).toContain('phases: Array<FlightPhase>')
  })

  it('emits SCENARIO_DEFAULTS that parses as an object with the required id', () => {
    const marker = 'export const SCENARIO_DEFAULTS: ScenarioConfig = '
    const start = SOURCE.indexOf(marker)
    expect(start, 'SCENARIO_DEFAULTS literal is present').toBeGreaterThanOrEqual(0)
    // The literal is JSON; strip the trailing `;` before parsing.
    const literal = SOURCE.slice(start + marker.length).replace(/;\s*$/, '')
    const parsed: unknown = JSON.parse(literal)

    expect(parsed).toBeTypeOf('object')
    expect(parsed).not.toBeNull()
    const obj = parsed as Record<string, unknown>
    expect(obj['id']).toBe('scenario')
    // The default avoidance block is off, matching the Rust default.
    const avoidance = obj['avoidance'] as Record<string, unknown>
    expect(avoidance['enabled']).toBe(false)
    expect(obj['seed']).toBe(20261012)
  })
})
