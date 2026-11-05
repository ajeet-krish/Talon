/**
 * The wasm kernel loader stays in the Run view.
 *
 * The kernel fetch can fail at runtime (a stale `dist/`, a bad network), and
 * RunView turns that into a "KERNEL NOT LOADED" banner with the RUN button
 * gated off. Recorded scenarios must sail on regardless, and the structural
 * fact that keeps them safe is that nothing on the recorded path imports
 * `live/engine`: a value import would pull the whole kernel graph in at
 * module load, so a loader failure would take replay down with it.
 *
 * This file proves that fact two ways. First it walks the static import graph
 * from the recorded views and asserts `engine.ts` is unreachable, ignoring
 * type-only imports (which are erased at compile time, so `sim.ts` naming
 * `CoreSession` is not a runtime dependency). Then it asserts the global
 * invariant that RunView is the only file with a runtime import of the
 * loader. The converse walk from RunView keeps the import parser honest: a
 * walker that found no edges would fail that sanity check rather than pass
 * the isolation assertions vacuously.
 *
 * RunView's own fallback wiring is asserted from source, because there is no
 * DOM test tier (`vitest.config.ts` is `environment: 'node'`) and the scene
 * behind RunView needs WebGL, so the banner itself remains a human check.
 */

import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const SRC = path.resolve('src')
const ENGINE = path.join(SRC, 'live/engine.ts')
const RUN_VIEW = path.join(SRC, 'views/RunView.tsx')

/** The views that read recorded artifacts rather than flying the kernel. */
const RECORDED_VIEWS = [
  'views/ReplayView.tsx',
  'views/TelemetryView.tsx',
  'views/MetricsView.tsx',
  'views/VideosView.tsx',
].map((rel) => path.join(SRC, rel))

/** Every source file in `src`, minus the test files (tests may import anything). */
function listSourceFiles(dir: string, out: string[] = []): string[] {
  for (const entry of readdirSync(dir)) {
    const full = path.join(dir, entry)
    if (statSync(full).isDirectory()) {
      listSourceFiles(full, out)
    } else if (/\.tsx?$/.test(entry) && !/\.test\.tsx?$/.test(entry)) {
      out.push(full)
    }
  }
  return out
}

/**
 * Specifiers of the runtime imports and re-exports in a source file.
 * Type-only statements are erased at compile time and do not count; a
 * mixed `import { type A, b }` does, because `b` survives.
 */
function runtimeImportSpecifiers(source: string): string[] {
  // Side-effect imports have no `from` clause, so they are collected first
  // and removed; otherwise the statement matcher below could run from one
  // import into the next.
  const sideEffects = [...source.matchAll(/^\s*import\s+['"]([^'"]+)['"]/gm)].map(
    (m) => m[1] ?? '',
  )
  const withoutSideEffects = source.replace(/^\s*import\s+['"][^'"]+['"];?[ \t]*$/gm, '')
  const statement =
    /(?:^|\n)\s*(?:import|export)[ \t]+(type[ \t]+)?[^;'"]*?from[ \t]*['"]([^'"]+)['"]/g
  const fromClauses = [...withoutSideEffects.matchAll(statement)]
    .filter((m) => m[1] === undefined)
    .map((m) => m[2] ?? '')
  return [...sideEffects, ...fromClauses]
}

/** Resolve a specifier to an absolute file path, or null for bare packages. */
function resolveSpec(fromFile: string, spec: string): string | null {
  let base: string
  if (spec.startsWith('@/')) {
    base = path.join(SRC, spec.slice(2))
  } else if (spec.startsWith('.')) {
    base = path.resolve(path.dirname(fromFile), spec)
  } else {
    return null
  }
  const candidates = [
    base,
    `${base}.ts`,
    `${base}.tsx`,
    path.join(base, 'index.ts'),
    path.join(base, 'index.tsx'),
  ]
  for (const candidate of candidates) {
    if (existsSync(candidate) && statSync(candidate).isFile()) return candidate
  }
  return null
}

/** The set of files reachable from `roots` through runtime imports. */
function reachableFrom(roots: string[]): Set<string> {
  const seen = new Set<string>()
  const queue = [...roots]
  while (queue.length > 0) {
    const file = queue.pop()!
    if (seen.has(file)) continue
    seen.add(file)
    for (const spec of runtimeImportSpecifiers(readFileSync(file, 'utf8'))) {
      const resolved = resolveSpec(file, spec)
      if (resolved !== null && !seen.has(resolved)) queue.push(resolved)
    }
  }
  return seen
}

describe('the kernel loader is confined to the Run view', () => {
  it('the import parser skips type-only statements and keeps runtime ones', () => {
    // `sim.ts` names CoreSession only as a type, which is erased at compile
    // time; counting it as an edge would make the isolation proof fail on a
    // non-dependency, while missing a runtime edge would pass vacuously.
    expect(runtimeImportSpecifiers("import type { CoreSession } from './engine'")).toEqual([])
    expect(runtimeImportSpecifiers("import { loadEngine } from '@/live/engine'")).toEqual([
      '@/live/engine',
    ])
    expect(
      runtimeImportSpecifiers("import {\n  a,\n  b,\n} from './multi'\nexport type { X } from './types'"),
    ).toEqual(['./multi'])
    expect(runtimeImportSpecifiers("import './side.css'")).toEqual(['./side.css'])
  })

  it('the recorded views cannot reach the kernel loader', () => {
    const reached = reachableFrom(RECORDED_VIEWS)

    expect(reached.has(ENGINE)).toBe(false)

    // Sanity: the walk really follows edges. ReplayView imports the store,
    // which imports the recorded-data validator, so both must be in the set;
    // a parser that found nothing would fail here instead of passing the
    // isolation assertion vacuously.
    expect(reached.has(path.join(SRC, 'store/useFlightStore.ts'))).toBe(true)
    expect(reached.has(path.join(SRC, 'data/validate.ts'))).toBe(true)
    expect(reached.size).toBeGreaterThanOrEqual(12)
  })

  it('RunView is the only file that imports the kernel loader at runtime', () => {
    const importers: string[] = []
    for (const file of listSourceFiles(SRC)) {
      const hit = runtimeImportSpecifiers(readFileSync(file, 'utf8')).some(
        (spec) => resolveSpec(file, spec) === ENGINE,
      )
      if (hit) importers.push(path.relative(SRC, file).split(path.sep).join('/'))
    }

    expect(importers).toEqual(['views/RunView.tsx'])
  })

  it('the walk from RunView does reach the loader (the parser is sound)', () => {
    const reached = reachableFrom([RUN_VIEW])

    expect(reached.has(ENGINE)).toBe(true)
    expect(reached.has(path.join(SRC, 'live/sim.ts'))).toBe(true)
  })
})

describe('the Run view fallback wiring', () => {
  const runView = readFileSync(RUN_VIEW, 'utf8')

  it('turns a loader failure into the unavailable state with the error message', () => {
    // loadEngine().then(...).catch(...) sets 'unavailable' and surfaces why.
    expect(runView).toMatch(
      /loadEngine\(\)[\s\S]*?\.catch\(\(err: unknown\) => \{[\s\S]*?setWasm\('unavailable'\)[\s\S]*?setWasmError\(err instanceof Error \? err\.message : String\(err\)\)/,
    )
  })

  it('turns a sample-layout mismatch into the unavailable state too', () => {
    expect(runView).toMatch(
      /const stride = mod\.sample_stride\(\)[\s\S]{0,300}?stride !== SAMPLE_STRIDE[\s\S]{0,300}?setWasm\('unavailable'\)/,
    )
  })

  it('gates RUN on a ready kernel and tells the operator recorded scenarios are unaffected', () => {
    expect(runView).toMatch(
      /const canRun = wasm === 'ready' && selected !== null && runState !== 'running'/,
    )
    expect(runView).toContain('KERNEL NOT LOADED')
    // the JSX copy breaks the sentence across a source line
    expect(runView).toMatch(/Recorded\s+scenarios are unaffected/)
  })
})
