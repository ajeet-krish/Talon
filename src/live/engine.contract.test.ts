/**
 * Guard on the generated wasm declaration the loader binds.
 *
 * `src/wasm-rust/talon_core.d.ts` is the source of truth: wasm-pack emits it
 * from `wasm/src/lib.rs`, and `engine.ts` now imports the sibling
 * `talon_core.js` directly, so `CoreSession` and `CoreModule` are aliases of
 * the generated shapes. Shape drift between the app and the core is therefore
 * caught by `tsc` itself, and this test no longer compares a hand-written
 * mirror (that mirror is what let the GPS counters drift to `number` while
 * the core exports `bigint`).
 *
 * What `tsc` can not check is what lives on disk and in the wiring: that the
 * generated file is present where the loader imports it from, that it still
 * declares the members the live seam pins by name, and that the loader keeps
 * binding it through the module graph instead of fetching it from `public/`
 * at runtime (the dev-server failure this replaced).
 *
 * The file and member checks use a deliberately simple line parser. It is a
 * guard, not a TypeScript parser: it only has to hold for the output
 * wasm-pack actually emits.
 */

import { readFileSync } from 'node:fs'
import { describe, expect, it } from 'vitest'

const DECL_PATH = 'src/wasm-rust/talon_core.d.ts'
const DECL = readFileSync(DECL_PATH, 'utf8')
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

const SESSION = classMembers(blockAfter(DECL, 'export class Session {'))

describe('the generated wasm declaration is what the loader binds', () => {
  it('ships the generated declaration where engine.ts imports from', () => {
    // readFileSync throws above if the generated file is missing, so reaching
    // this assertion means the declaration exists at the imported path.
    expect(DECL).toContain('export class Session')
  })

  it('imports the kernel through the module graph, not from public/ at runtime', () => {
    expect(ENGINE).toMatch(/from '@\/wasm-rust\/talon_core\.js'/)
    expect(ENGINE).toMatch(/from '@\/wasm-rust\/talon_core_bg\.wasm\?url'/)
    // The dev server refuses to resolve a module that lives in publicDir, so
    // a runtime import of the public asset is the exact regression this
    // wiring exists to prevent.
    expect(ENGINE).not.toContain('@vite-ignore')
    expect(ENGINE).not.toContain('public/wasm-rust')
  })

  it('aliases the generated types instead of hand-mirroring them', () => {
    expect(ENGINE).toMatch(/export type CoreSession = Session\b/)
    expect(ENGINE).not.toMatch(/export interface CoreSession/)
    // CoreModule stays an interface, but its members are typeof bindings of
    // the generated exports rather than a re-declared surface.
    expect(ENGINE).toMatch(/Session: typeof Session/)
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

  it('exports the default init the loader awaits', () => {
    expect(DECL).toMatch(/export default function __wbg_init/)
  })
})
