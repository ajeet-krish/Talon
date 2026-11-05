/**
 * Viewport contract for Phase 8d-C, asserted from source (the same tier as
 * navContract.test.ts: vitest runs environment 'node', so the JSX is read
 * out of the files rather than rendered). Three claims a pure chart test
 * cannot see: the chart fits only on mount, on a document identity change,
 * and on the FIT button (never on an edit), the frame is derived from the
 * viewport rather than from the mission bounds at render time, and the
 * wheel listener is registered non-passively so the page cannot scroll
 * under the chart.
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const CANVAS = readFileSync(path.resolve('src/components/editor/MissionCanvas.tsx'), 'utf8')
const VIEW = readFileSync(path.resolve('src/views/MissionView.tsx'), 'utf8')

/**
 * The source of one top-level function inside the component, from its
 * declaration to the next one, so a call site can be pinned to (or ruled
 * out of) a specific code path.
 */
function functionBody(name: string): string {
  const start = CANVAS.indexOf(`function ${name}(`)
  if (start < 0) throw new Error(`function ${name} not found in MissionCanvas.tsx`)
  const next = CANVAS.indexOf('\n  function ', start + 1)
  return CANVAS.slice(start, next === -1 ? CANVAS.length : next)
}

describe('mission chart viewport', () => {
  it('fits only on mount, on the document epoch, and on the FIT button', () => {
    // Every viewFit call site: the state initializer, the epoch effect, FIT.
    expect(CANVAS.match(/viewFit\(/g) ?? []).toHaveLength(3)
    expect(CANVAS).toContain('[documentEpoch, aspectKnown]')
    // A plain config change must not reach the fit path.
    expect(CANVAS).not.toContain('const bounds = chartBounds(config)')
  })

  it('draws the frame from the view, not from the mission bounds at render', () => {
    expect(CANVAS).toContain('const bounds = viewBounds(view, aspect ?? 1)')
  })

  it('registers the wheel listener non-passively at the chart', () => {
    expect(CANVAS).toContain("addEventListener('wheel', onWheel, { passive: false })")
  })

  it('keeps zoom and pan out of the mission store', () => {
    expect(CANVAS).not.toContain('useMissionStore')
    expect(CANVAS).toContain('useState<ChartView>')
    expect(VIEW).toContain('documentEpoch={documentEpoch}')
  })

  it('pans on a background press and guards the double-click with moved flags', () => {
    expect(CANVAS).toContain('onPointerDown={beginPan}')
    expect(CANVAS).not.toContain('onClick={() => onSelect(null)}')
    expect(CANVAS).toContain('if (last.moved || prev.moved) return')
  })

  it('changes the view only from sanctioned viewport paths, never from an edit', () => {
    // The full setView call-site audit: the fit effect, the wheel, the zoom
    // buttons, the FIT button, and the pan. Nothing else may move the frame.
    const callSites = CANVAS.split('\n')
      .map((line) => line.trim())
      .filter((line) => line.includes('setView('))
    expect(callSites).toHaveLength(5)
    const sanctioned = [
      'setView(viewFit(chartBounds(config), aspect ?? 1))',
      'setView((current) => zoomAt(current, anchor, factor))',
      'setView((current) => zoomAt(current, { x: current.centerX, y: current.centerY }, factor))',
      'setView((current) => panBy(current, previous.x - next.x, previous.y - next.y))',
    ]
    for (const line of callSites) expect(sanctioned).toContain(line)
    const editPaths = [
      'beginDrag',
      'trackDrag',
      'endDrag',
      'cancelDrag',
      'addWaypointAt',
      'nudge',
      'onNodeKeyDown',
    ]
    for (const fn of editPaths) expect(functionBody(fn)).not.toContain('setView(')
  })

  it('ignores wheel input while a drag or pan is in progress', () => {
    const start = CANVAS.indexOf('const onWheel = (e: WheelEvent): void => {')
    expect(start).toBeGreaterThan(-1)
    const end = CANVAS.indexOf('svg.addEventListener', start)
    const onWheel = CANVAS.slice(start, end)
    expect(onWheel).toContain('e.preventDefault()')
    const guard = onWheel.indexOf('if (interactionRef.current !== null) return')
    const anchor = onWheel.indexOf('toChartPoint(svg, e.clientX, e.clientY)')
    expect(guard).toBeGreaterThan(-1) // the gesture guard comes first
    expect(anchor).toBeGreaterThan(guard) // before any anchor or view change
    expect(onWheel).toContain('setView((current) => zoomAt(current, anchor, factor))')
  })

  it('frames the current document from the FIT button, as a view change not an edit', () => {
    expect(CANVAS).toContain('onClick={fitView}')
    const body = functionBody('fitView')
    expect(body).toContain('setView(viewFit(chartBounds(config), aspect ?? 1))')
    expect(body).not.toContain('dispatch(') // fitting is never an edit
    expect(body).not.toContain('documentEpoch') // the button does not touch identity
  })

  it('never writes storage or the dirty flag from the viewport', () => {
    expect(CANVAS).not.toMatch(/localStorage\s*\.(getItem|setItem|removeItem)/)
    expect(CANVAS).not.toContain('DRAFT_STORAGE_KEY')
    expect(CANVAS).not.toContain('dirty:')
  })
})
