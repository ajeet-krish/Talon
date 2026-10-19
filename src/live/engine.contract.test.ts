/**
 * Drift guard between the hand-declared wasm surface and the generated one.
 *
 * `public/wasm-rust/talon_core.d.ts` is the source of truth: wasm-pack emits it
 * from `wasm/src/lib.rs`. The app never imports it directly, because the module
 * is loaded at runtime from a static asset, so `CoreSession` in `engine.ts` is
 * a hand-written mirror that tsc checks against. Nothing else ties the two
 * together, which is how the GPS counters drifted to `number` while the core
 * exports them as `bigint`.
 *
 * This test reads both files from disk and compares the shapes with a
 * deliberately simple line parser. It is a drift guard, not a TypeScript
 * parser: it only has to hold for the output wasm-pack actually emits.
 */

import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'

const DECL = readFileSync('public/wasm-rust/talon_core.d.ts', 'utf8')
const ENGINE = readFileSync('src/live/engine.ts', 'utf8')

/**
 * The body of the block opened by `header`, up to the first closing brace at
 * column 0. Good enough for the flat class and interface bodies these files
 * contain.
 */
function blockAfter(source: string, header: string): string {
  const start = source.indexOf(header)
  if (start < 0) throw new Error(`missing declaration: ${header}`)
  const end = source.indexOf('\n}', start)
  if (end < 0) throw new Error(`unterminated declaration: ${header}`)
  return source.slice(start, end)
}

/**
 * Members of a class body, mapping name to type: methods map to `'method'`,
 * readonly getters map to their declared type (`bigint`, `number`, ...).
 */
function classMembers(body: string): Map<string, string> {
  const members = new Map<string, string>()
  const method = /^ {4}([a-z_]\w*)\(/
  const getter = /^ {4}readonly (\w+): ([\w.]+);/
  for (const line of body.split('\n')) {
    const m = method.exec(line)
    if (m?.[1] !== undefined) {
      members.set(m[1], 'method')
      continue
    }
    const g = getter.exec(line)
    if (g?.[1] !== undefined && g[2] !== undefined) {
      members.set(g[1], g[2])
    }
  }
  return members
}

/** Members of the hand-written `CoreSession` interface, mapped to `'method'` or the field type. */
function coreSessionMembers(source: string): Map<string, string> {
  const body = blockAfter(source, 'export interface CoreSession {')
  const method = /^ {2}([a-z_]\w*)\(/
  const field = /^ {2}readonly (\w+): ([\w.]+)$/
  const members = new Map<string, string>()
  for (const line of body.split('\n')) {
    const m = method.exec(line)
    if (m?.[1] !== undefined) {
      members.set(m[1], 'method')
      continue
    }
    const f = field.exec(line)
    if (f?.[1] !== undefined && f[2] !== undefined) {
      members.set(f[1], f[2])
    }
  }
  return members
}

const SESSION = classMembers(blockAfter(DECL, 'export class Session {'))

describe('CoreSession mirrors the generated wasm declaration', () => {
  it('declares every member the app relies on from the Session class', () => {
    const appMembers = coreSessionMembers(ENGINE)
    expect(appMembers.size).toBeGreaterThan(0)
    for (const [name, appType] of appMembers) {
      const coreType = SESSION.get(name)
      expect(coreType, `generated Session is missing ${name}`).toBeDefined()
      // Field types must match the generated declaration exactly. Methods are
      // pinned by name here; their signatures change shape less often, and the
      // bigint drift this guards against lived on the GPS counter fields.
      if (appType !== 'method' && coreType !== undefined) {
        expect(
          coreType,
          `CoreSession.${name} is declared ${appType}, the generated Session says ${coreType}`,
        ).toBe(appType)
      }
    }
  })

  it('pins the members the live seam depends on by name', () => {
    const required = [
      'prepare',
      'step',
      'advance',
      'run_to_end',
      'phase',
      'phases_json',
      'samples_from',
      'truth_position',
      'truth_velocity',
      'ekf_position',
      'altitude_m',
      'document_json',
      'summary_json',
      'free',
      'progress',
      'finished',
      'prepared',
      'steps_done',
      'total_steps',
      'time',
      'duration_s',
      'scenario_id',
      'battery_soc',
      'sample_count',
      'gps_accepted',
      'gps_gated',
      'gps_skipped',
      'ekf_converged',
      'ekf_diverged',
    ]
    for (const name of required) {
      expect(SESSION.has(name), `generated Session is missing ${name}`).toBe(true)
    }
  })

  it('declares the GPS counters as bigint, not number', () => {
    expect(SESSION.get('gps_accepted')).toBe('bigint')
    expect(SESSION.get('gps_gated')).toBe('bigint')
    expect(SESSION.get('gps_skipped')).toBe('bigint')
  })

  it('exports sample_stride() as returning number', () => {
    expect(DECL).toMatch(/^export function sample_stride\(\): number;$/m)
  })

  it('exports the other module-level functions the loader binds', () => {
    expect(DECL).toMatch(/^export function core_version\(\): string;$/m)
    expect(DECL).toMatch(/^export function run_scenario\([^)]*\): string;$/m)
  })
})
