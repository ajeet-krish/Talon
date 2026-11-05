/**
 * Viewport contract for Phase 8d-C, extended in Phase 8d-D, asserted from
 * source (the same tier as navContract.test.ts: vitest runs environment
 * 'node', so the JSX is read out of the files rather than rendered). Claims
 * a pure chart test cannot see: the chart fits only on mount, on a document
 * identity change, and on the FIT button (never on an edit), with the first
 * measured aspect latched so a zero-size blip cannot re-fit either; the
 * frame is derived from the viewport rather than from the mission bounds at
 * render time; the wheel listener is registered non-passively so the page
 * cannot scroll under the chart; and the drag transient the 3D preview
 * mirrors is published through one end path, after the commit dispatch,
 * without the chart ever touching the store directly.
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { describe, expect, it } from 'vitest'

const CANVAS = readFileSync(path.resolve('src/components/editor/MissionCanvas.tsx'), 'utf8')
const SHELL = readFileSync(path.resolve('src/components/editor/MissionChart.tsx'), 'utf8')
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

  it('latches the first measured aspect so a zero-size blip cannot re-fit', () => {
    // The latch flips once, false to true, and the drawn frame and the fits
    // read the last known measurement rather than a nullable live one.
    expect(CANVAS).toContain('aspectRef.current = frameSize.width / frameSize.height')
    expect(CANVAS).toContain('measuredRef.current = true')
    expect(CANVAS).toContain('const aspect = aspectRef.current')
    expect(CANVAS).toContain('const aspectKnown = measuredRef.current')
    expect(CANVAS).not.toContain('aspect ?? 1')
    // The fit effect runs off the latch: aspect appears in the body, and
    // neither config nor aspect is a dependency.
    const start = CANVAS.indexOf('useLayoutEffect(() => {')
    const end = CANVAS.indexOf('}, [documentEpoch, aspectKnown])')
    expect(start).toBeGreaterThan(-1)
    expect(end).toBeGreaterThan(start)
    const fitEffect = CANVAS.slice(start, end)
    expect(fitEffect).toContain('setView(viewFit(chartBounds(config), aspect))')
    expect(fitEffect).not.toContain('dragPreview')
  })

  it('draws the frame from the view, not from the mission bounds at render', () => {
    expect(CANVAS).toContain('const bounds = viewBounds(view, aspect)')
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
      'setView(viewFit(chartBounds(config), aspect))',
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
    expect(body).toContain('setView(viewFit(chartBounds(config), aspect))')
    expect(body).not.toContain('dispatch(') // fitting is never an edit
    expect(body).not.toContain('documentEpoch') // the button does not touch identity
  })

  it('never writes storage or the dirty flag from the viewport', () => {
    expect(CANVAS).not.toMatch(/localStorage\s*\.(getItem|setItem|removeItem)/)
    expect(CANVAS).not.toContain('DRAFT_STORAGE_KEY')
    expect(CANVAS).not.toContain('dirty:')
  })
})

describe('shared drag transient (Phase 8d-D)', () => {
  it('refuses a second pointer while a gesture is in flight', () => {
    // The active-interaction guard beginPan already had, now on beginDrag:
    // a second pointer landing mid-pan must not overwrite gesture state.
    const body = functionBody('beginDrag')
    const guard = body.indexOf('if (interactionRef.current !== null) return')
    expect(guard).toBeGreaterThan(-1)
    expect(guard).toBeLessThan(body.indexOf('setPointerCapture')) // before capture
    expect(guard).toBeLessThan(body.indexOf('onSelect(')) // before selection
  })

  it('publishes the transient from the move path, and only the move path', () => {
    expect(functionBody('trackDrag')).toContain('onDragPreview({')
    expect(functionBody('beginDrag')).not.toContain('onDragPreview(')
    expect(functionBody('beginPan')).not.toContain('onDragPreview(')
    // The chart draws the shared value back through the one applier, with no
    // private drag state of its own to drift from the 3D preview.
    expect(CANVAS).toContain('const drawn = withDragPreview(config, dragPreview)')
    expect(CANVAS).not.toContain('useState<DragState')
  })

  it('clears the transient after the commit dispatch, on one end path', () => {
    const end = functionBody('endDrag')
    const commit = end.indexOf('dispatch({')
    const clear = end.indexOf('onDragPreview(null)')
    expect(commit).toBeGreaterThan(-1)
    expect(clear).toBeGreaterThan(commit) // the clear comes after the commit
    // Cancel drops the gesture without committing, and still clears.
    const cancel = functionBody('cancelDrag')
    expect(cancel).toContain('onDragPreview(null)')
    expect(cancel).not.toContain('dispatch(')
    // Lost capture shares the end path, so a vanished pointer cannot strand
    // a drag (or its interaction guard) on the chart.
    expect(CANVAS).toContain('onLostPointerCapture={endDrag}')
  })

  it('wires the transient through a store-backed shell, never the chart', () => {
    expect(CANVAS).not.toContain('useMissionStore') // the chart stays store-free
    expect(SHELL).toContain('useMissionStore((s) => s.dragPreview)')
    expect(SHELL).toContain('useMissionStore((s) => s.setDragPreview)')
    expect(SHELL).toContain('dragPreview={dragPreview}')
    expect(SHELL).toContain('onDragPreview={setDragPreview}')
    // A view switch mid-drag clears the transient from the shell's unmount,
    // the one path the chart's end handlers cannot reach.
    expect(SHELL).toContain('useEffect(() => () => useMissionStore.getState().setDragPreview(null), [])')
    expect(VIEW).toContain('<MissionChart')
    expect(VIEW).not.toContain('<MissionCanvas')
  })
})

describe('node keys during a gesture (Phase 8d-D hardening)', () => {
  it('ignores node key handling while a gesture is in flight', () => {
    // Pointer capture gates pointer events but not keys: without this guard a
    // Delete or an arrow nudge on the focused node fires mid-drag, shifts the
    // waypoint list, and the end-of-drag commit then runs against a stale
    // index, moving a different item. The guard matches the one beginDrag,
    // beginPan, and the wheel listener already carry.
    const body = functionBody('onNodeKeyDown')
    const guard = body.indexOf('if (interactionRef.current !== null) return')
    expect(guard).toBeGreaterThan(-1)
    // Before every key branch and before any state change the keys cause:
    // Enter/space selection, Delete/Backspace removal, and the arrow nudge.
    expect(guard).toBeLessThan(body.indexOf("e.key === 'Enter'"))
    expect(guard).toBeLessThan(body.indexOf("e.key === 'Delete'"))
    expect(guard).toBeLessThan(body.indexOf('nudge(e, target, index)'))
    expect(guard).toBeLessThan(body.indexOf('dispatch('))
    expect(guard).toBeLessThan(body.indexOf('onSelect('))
  })

  it('leaves keyboard nudges working when no gesture is active', () => {
    // The guard lives only on the shared handler: with no gesture in flight
    // the handler falls straight through to nudge, which still dispatches the
    // moves and carries no interaction check of its own.
    const handler = functionBody('onNodeKeyDown')
    expect(handler).toContain('nudge(e, target, index)')
    const nudge = functionBody('nudge')
    expect(nudge).not.toContain('interactionRef.current')
    expect(nudge).toContain("type: 'waypoint/move'")
    expect(nudge).toContain("type: 'obstacle/move'")
  })
})

describe('coordinate hardening (Phase 8d-D hardening)', () => {
  it('routes every coordinate the chart authors through clampCoordinate', () => {
    // The NaN rule lives in one shared function; the move drag, the
    // double-click add, and the keyboard nudge must all use it, or a
    // degenerate SVG matrix could still poison the drag preview and the
    // drawn config with a NaN the validator would then reject.
    expect(CANVAS).toContain("import { clampCoordinate, withDragPreview } from '@/utils/editor'")
    // Two call sites per authoring path: drag (north, east), add (north,
    // east), waypoint nudge (north, east), zone nudge (north, east).
    expect(CANVAS.match(/clampCoordinate\(/g) ?? []).toHaveLength(8)
    const track = functionBody('trackDrag')
    expect(track).toContain('clampCoordinate(world.north - drag.offsetY)')
    expect(track).toContain('clampCoordinate(world.east - drag.offsetX)')
    const add = functionBody('addWaypointAt')
    expect(add).toContain('clampCoordinate(world.north)')
    expect(add).toContain('clampCoordinate(world.east)')
    const nudgeBody = functionBody('nudge')
    expect(nudgeBody).toContain('clampCoordinate(wp.position[0] + dNorth)')
    expect(nudgeBody).toContain('clampCoordinate(wp.position[1] + dEast)')
    expect(nudgeBody).toContain('clampCoordinate(zone.center_ned[0] + dNorth)')
    expect(nudgeBody).toContain('clampCoordinate(zone.center_ned[1] + dEast)')
  })

  it('keeps the route panel Add button on the same shared rule', () => {
    // The panel's Add is the other entry point the clamp's contract names:
    // its dispatched add carries the clamped locals, never a raw position.
    const panel = readFileSync(path.resolve('src/components/editor/WaypointPanel.tsx'), 'utf8')
    expect(panel).toContain('const north = clampCoordinate(')
    expect(panel).toContain('const east = clampCoordinate(')
    expect(panel).toContain("dispatch({ type: 'waypoint/add', north, east })")
  })
})
