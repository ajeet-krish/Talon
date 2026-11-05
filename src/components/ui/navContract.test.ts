/**
 * Nav reorder contract for Phase 8d-B, asserted from source (the same tier as
 * viewTabs.test.ts: vitest runs environment 'node', so JSX is read out of the
 * files rather than rendered). Two claims the tab-order test cannot see:
 * App.tsx must actually mount a section for every header tab, in header order,
 * and no source or test file may carry a stale assertion that a first load
 * opens in 'replay' now that the landing view is the mission planner.
 */

import { readdirSync, readFileSync, statSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'
import type { ViewId } from '@/store/useFlightStore'

const APP_SOURCE = readFileSync(path.resolve('src/App.tsx'), 'utf8')

/** The sections in header order, mirroring ViewTabs' VIEWS array. */
const EXPECTED_ORDER: ViewId[] = [
  'mission',
  'telemetry',
  'metrics',
  'videos',
  'run',
  'replay',
]

/** The `{ activeView === 'id' && <Component /> }` entries of the main block. */
function dispatch(): { id: string; component: string }[] {
  const entries = [...APP_SOURCE.matchAll(/activeView === '([^']+)' && <([A-Za-z]+)/g)].map(
    (m) => ({ id: m[1] ?? '', component: m[2] ?? '' }),
  )
  expect(entries.length, 'App.tsx must dispatch a section per tab').toBeGreaterThan(0)
  return entries
}

describe('App view dispatch', () => {
  it('mounts one section per tab, in header order', () => {
    expect(dispatch().map((d) => d.id)).toEqual(EXPECTED_ORDER)
  })

  it('maps each tab to its own section component', () => {
    expect(dispatch()).toEqual([
      { id: 'mission', component: 'MissionView' },
      { id: 'telemetry', component: 'TelemetryView' },
      { id: 'metrics', component: 'MetricsView' },
      { id: 'videos', component: 'VideosView' },
      { id: 'run', component: 'RunView' },
      { id: 'replay', component: 'ReplayView' },
    ])
  })
})

/**
 * Stale-default tripwires. Navigation to replay goes through setView('replay')
 * call sites, which the patterns below deliberately do not match; they only
 * catch a literal assignment or a flipped first-load pin.
 */
const STALE_PATTERNS: { what: string; pattern: RegExp }[] = [
  { what: "literal activeView default of 'replay'", pattern: /activeView:\s*'replay'/ },
  { what: "first-load pin asserted as 'replay'", pattern: /FIRST_LOAD_VIEW\)\s*\.toBe\(\s*'replay'/ },
]

/** Every .ts/.tsx file under src, recursively. */
function sourceFilesUnder(dir: string): string[] {
  const files: string[] = []
  for (const name of readdirSync(dir)) {
    const full = path.join(dir, name)
    if (statSync(full).isDirectory()) files.push(...sourceFilesUnder(full))
    else if (name.endsWith('.ts') || name.endsWith('.tsx')) files.push(full)
  }
  return files
}

describe('no stale replay-default assertion', () => {
  it('scans every source and test file under src/', () => {
    const files = sourceFilesUnder(path.resolve('src'))
    // The scan must actually see the tree; a broken walk would vacuously pass.
    expect(files.length).toBeGreaterThan(20)

    const offenders: string[] = []
    for (const file of files) {
      const text = readFileSync(file, 'utf8')
      for (const { what, pattern } of STALE_PATTERNS) {
        if (pattern.test(text)) offenders.push(`${file}: ${what}`)
      }
    }
    expect(offenders).toEqual([])
  })
})
