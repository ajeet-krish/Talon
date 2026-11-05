/**
 * Header tab contract, asserted from source.
 *
 * There is no DOM test tier (`vitest.config.ts` is `environment: 'node'`), so
 * the rendered order and labels are read out of the component file the way
 * `live/engine.isolation.test.ts` reads RunView's wiring. The order is the
 * product decision: plan first, inspect after, replay last, and the planner
 * tab names what it does.
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'
import type { ViewId } from '@/store/useFlightStore'

const SOURCE = readFileSync(path.resolve('src/components/ui/ViewTabs.tsx'), 'utf8')

/** The sections in header order, as the nav must render them. */
const EXPECTED_ORDER: ViewId[] = [
  'mission',
  'telemetry',
  'metrics',
  'videos',
  'run',
  'replay',
]

/** The `{ id, label }` entries of the VIEWS array, in rendered order. */
function tabs(): { id: string; label: string }[] {
  const block = SOURCE.match(/const VIEWS[^=]*=\s*\[([\s\S]*?)\]/)?.[1]
  expect(block, 'VIEWS array not found in ViewTabs.tsx').toBeDefined()
  const entries = [...(block ?? '').matchAll(/id:\s*'([^']+)',\s*label:\s*'([^']+)'/g)].map(
    (m) => ({ id: m[1] ?? '', label: m[2] ?? '' }),
  )
  expect(entries.length, 'every VIEWS entry must carry an id and a label').toBeGreaterThan(0)
  return entries
}

describe('ViewTabs VIEWS', () => {
  it('renders every section once, in header order', () => {
    expect(tabs().map((t) => t.id)).toEqual(EXPECTED_ORDER)
  })

  it('labels the first tab MISSION PLANNER', () => {
    expect(tabs()[0]?.label).toBe('MISSION PLANNER')
  })

  it('covers the whole ViewId union with no extras', () => {
    const ids = tabs().map((t) => t.id)
    expect(new Set(ids).size).toBe(ids.length)
    expect([...ids].sort()).toEqual([...EXPECTED_ORDER].sort())
  })

  it('pins EXPECTED_ORDER as the whole ViewId union', () => {
    // The check above only compares two literals, so both could drop an id
    // in step. This Record is exhaustive over the union: a ViewId added to
    // the type must be added here (and to VIEWS), and a removed one fails
    // typecheck until it is dropped here too.
    const UNION: Record<ViewId, true> = {
      mission: true,
      telemetry: true,
      metrics: true,
      videos: true,
      run: true,
      replay: true,
    }
    expect(Object.keys(UNION).sort()).toEqual([...EXPECTED_ORDER].sort())
  })
})
