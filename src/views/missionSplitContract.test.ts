/**
 * Mission split contract (FlightLab Phase 8d-D), asserted from source (the
 * same tier as navContract.test.ts: vitest runs environment 'node', so the
 * JSX is read out of the file rather than rendered). The working area now
 * holds two framed surfaces beside the inspector; these are the claims a
 * typecheck cannot make: the 50/50 split from xl up, the stacked cramped
 * case at lg, the unchanged inspector, a validation bar that stays outside
 * the scrolling body at every width, and the per-pointer-move drag
 * transient staying out of the view and its panels so a drag re-renders
 * only the two surfaces that draw it.
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const VIEW = readFileSync(path.resolve('src/views/MissionView.tsx'), 'utf8')

/** The inspector panels: props in, no store subscriptions of their own. */
const PANELS = [
  'PlanSummary',
  'MissionParametersPanel',
  'WaypointPanel',
  'WindPanel',
  'ObstaclePanel',
  'FaultPanel',
  'MissionValidation',
].map((name) => readFileSync(path.resolve(`src/components/editor/${name}.tsx`), 'utf8'))

describe('working area layout', () => {
  it('splits the working area 50/50 from xl up, stacked at lg', () => {
    // One stages region: a column below xl, a row from xl up, scrolling
    // itself when the stacked case outruns the window.
    expect(VIEW).toContain('flex min-h-0 flex-1 flex-col lg:overflow-y-auto xl:flex-row')
    // Both stages are equal partners: same floor, same flex basis.
    const chartStage = 'min-h-[360px] flex-1 border-b border-grid xl:border-b-0 xl:border-r'
    const previewStage = 'min-h-[360px] flex-1 border-b border-grid lg:border-b-0'
    expect(VIEW).toContain(`className="${chartStage}"`)
    expect(VIEW).toContain(`className="${previewStage}"`)
    // Below lg both keep a bottom rule (chart over preview over inspector);
    // from lg the preview's goes (the validation bar's top rule is the
    // edge there); at xl the chart's right rule is the only separator.
    expect(chartStage).toContain('border-b')
    expect(previewStage).toContain('lg:border-b-0')
    expect(chartStage).toContain('xl:border-b-0')
    expect(chartStage).toContain('xl:border-r')
  })

  it('keeps the body a scrolling column below lg and a row from lg up', () => {
    expect(VIEW).toContain(
      'flex min-h-0 flex-1 flex-col overflow-y-auto lg:flex-row lg:overflow-hidden',
    )
    expect(VIEW).toContain('min-h-[360px]') // both surfaces keep a usable floor
  })

  it('keeps the inspector column unchanged beside the stages from lg up', () => {
    expect(VIEW).toContain('lg:w-[23rem] lg:border-l')
    expect(VIEW).toContain('w-full shrink-0 overflow-y-auto border-grid')
  })
})

describe('validation bar reachability', () => {
  it('sits outside the scrolling body at every width', () => {
    const bodyStart = VIEW.indexOf('overflow-y-auto lg:flex-row lg:overflow-hidden')
    expect(bodyStart).toBeGreaterThan(-1)
    const asideEnd = VIEW.indexOf('</aside>', bodyStart)
    expect(asideEnd).toBeGreaterThan(bodyStart)
    const bar = VIEW.indexOf('shrink-0 border-t border-grid')
    const validation = VIEW.indexOf('<MissionValidation')
    expect(validation).toBeGreaterThan(bar)
    expect(bar).toBeGreaterThan(asideEnd)
    // The body's own closing tag lands between the inspector and the bar:
    // the bar is a sibling of the scroll region, so nothing inside it can
    // scroll the bar out of reach.
    expect(VIEW.slice(asideEnd, bar)).toContain('</div>')
  })
})

describe('surface wiring', () => {
  it('mounts the preview propless and hands the chart its document epoch', () => {
    expect(VIEW).toContain('<MissionPreview />')
    expect(VIEW).toContain('<MissionChart')
    expect(VIEW).toContain('documentEpoch={documentEpoch}')
  })

  it('keeps the per-move drag transient out of the view and its panels', () => {
    // The preview and the chart shell subscribe themselves; the view and
    // the panels select committed state only, so a pointer-move write
    // re-renders the two surfaces and nothing else.
    expect(VIEW).not.toContain('dragPreview')
    for (const source of PANELS) expect(source).not.toContain('dragPreview')
  })
})
