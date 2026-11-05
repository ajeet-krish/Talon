/**
 * Mission planning chart (FlightLab Phase 8c, viewport Phase 8d-C). A
 * true-scale, north-up top-down view of the authored mission: a round-meter
 * grid, the numbered route from home through every waypoint with its
 * acceptance circles and altitudes, keep-out zones as amber dashed circles,
 * a north arrow, and a scale bar. The frame comes from the component's own
 * viewport (`ChartView`): zoom and pan are user-controlled and live outside
 * the undo history, so editing never re-scales the chart. Fitting the whole
 * mission happens only on mount (with the first measured frame aspect),
 * when the document identity changes (`documentEpoch`), and on the FIT
 * button. The grid, its labels, the scale
 * bar, and the chrome scale are all derived from the visible span and drawn
 * over the visible rectangle, so panning never runs off the grid's edge.
 *
 * Editing: click a node to select it, drag a waypoint or zone to move it
 * (one undo entry per drag, dispatched on pointer up), drag the selected
 * zone's east-edge handle to resize it, double-click empty chart to add a
 * waypoint, single-click empty chart to clear the selection. Keyboard: focus
 * a node, Enter or Space selects, arrows nudge 1 m (Shift 5 m), Delete
 * removes the selected item. Dragging uses pointer capture with world
 * coordinates from `getScreenCTM`, so it tracks the cursor under any zoom
 * and works with touch (`touch-action: none`).
 *
 * A drag in flight is published through the `onDragPreview` prop into the
 * store's transient slot, so the 3D course preview beside the chart tracks
 * the pointer at move rate; both surfaces draw it through `withDragPreview`.
 * One end path (pointer up, cancel, lost capture) clears it, after the
 * commit dispatch when there is one. The transient never enters the undo
 * history, the dirty flag, storage, or the document epoch.
 *
 * Viewport: ZOOM IN and ZOOM OUT step by a fixed factor about the view
 * centre, FIT frames the mission, the wheel zooms at the cursor, and
 * dragging empty background pans. A pan or drag never reads as a click or a
 * double-click (the moved-flag gesture guard), and the wheel is ignored
 * mid-gesture so the frame cannot move while an edit is in progress.
 */

import { useEffect, useLayoutEffect, useRef, useState } from 'react'
import type { MouseEvent as ReactMouseEvent, PointerEvent as ReactPointerEvent } from 'react'
import { LIMITS } from '@/types/generated/talon_core'
import { useElementSize } from '@/hooks/useElementSize'
import { CHROME_BUTTON } from '@/components/ui/chrome'
import { clampCoordinate, withDragPreview } from '@/utils/editor'
import type { DragPreview, EditorCommand, EditorSelection, MissionConfig } from '@/utils/editor'
import {
  boundsHeight,
  boundsViewBox,
  boundsWidth,
  canZoomIn,
  canZoomOut,
  chartBounds,
  gridStepM,
  labelStepM,
  linesOver,
  panBy,
  scaleBarLengthM,
  viewBounds,
  viewFit,
  zoomAt,
  type ChartPoint,
  type ChartView,
} from './chart'

export interface MissionCanvasProps {
  /** The mission being edited. */
  config: MissionConfig
  /** What the inspector points at, or null. */
  selected: EditorSelection
  /** Select one waypoint or zone, or clear the selection with null. */
  onSelect: (sel: EditorSelection) => void
  /** Apply one edit (move, resize, add, remove). */
  dispatch: (cmd: EditorCommand) => void
  /**
   * Store counter of document replacements (load, template load, import,
   * CLEAR). The chart re-fits when it changes; an edit never advances it,
   * so zoom and pan survive the editing session.
   */
  documentEpoch: number
  /**
   * The live drag shared with the 3D course preview, or null when no
   * gesture is in flight. The chart publishes it through `onDragPreview`
   * and draws the same value back, so the two surfaces show one transient
   * at pointer-move rate.
   */
  dragPreview: DragPreview | null
  /**
   * Publish the shared drag preview, or clear it with null. Backed by the
   * store's transient slot: it never enters the undo history, the dirty
   * flag, storage, or the document epoch, and the chart's own viewport
   * stays local state as before.
   */
  onDragPreview: (preview: DragPreview | null) => void
}

/** A live drag on a waypoint or zone handle: what is being dragged, where it started, and the last point. */
interface DragState {
  kind: 'move' | 'resize'
  target: 'waypoint' | 'obstacle'
  index: number
  pointerId: number
  /** Client coordinates at pointer down, to tell a drag from a click. */
  downX: number
  downY: number
  /** Grab offset (world) for moves: pointer minus the item's position. */
  offsetX: number
  offsetY: number
  /** True once the pointer has travelled far enough to count as a drag. */
  moved: boolean
  /** Latest previewed position (and radius, for a resize). */
  north: number
  east: number
  radius: number
}

/** A live pan on the empty chart background: start point and last pointer position. */
interface PanState {
  kind: 'pan'
  pointerId: number
  /** Client coordinates at pointer down, to tell a pan from a click. */
  downX: number
  downY: number
  /** True once the pointer has travelled past `DRAG_THRESHOLD_PX`. */
  moved: boolean
  /** Previous pointer position, for the world delta of the next move. */
  lastX: number
  lastY: number
}

/** One pointer gesture on the chart: a node drag or a background pan. */
type Interaction = DragState | PanState

/** What one finished gesture was, so a later click can tell them apart. */
interface GestureRecord {
  kind: 'background' | 'node'
  /** True when the gesture travelled past `DRAG_THRESHOLD_PX`. */
  moved: boolean
}

/**
 * Chart-space coordinates of a client point, or null without a screen CTM.
 * The CTM already carries the current zoom and pan, so this is the single
 * source of world coordinates for drags, pans, wheel anchors, and
 * double-clicks alike.
 */
function toChartPoint(svg: SVGSVGElement, clientX: number, clientY: number): ChartPoint | null {
  const ctm = svg.getScreenCTM()
  if (ctm === null) return null
  const point = svg.createSVGPoint()
  point.x = clientX
  point.y = clientY
  const local = point.matrixTransform(ctm.inverse())
  return { x: local.x, y: local.y }
}

/** Clamp a dragged radius to a sensible, validator-legal range. */
function clampRadius(value: number): number {
  return Math.min(Math.max(value, 0.5), Math.min(200, LIMITS.max_radius_m))
}

/** Distance (px) the pointer must travel before a press counts as a drag. */
const DRAG_THRESHOLD_PX = 2

/** Fixed zoom step for the ZOOM IN and ZOOM OUT buttons, per press. */
const ZOOM_STEP = 1.3

/** Fixed zoom step for one wheel tick at the cursor. */
const WHEEL_ZOOM_STEP = 1.25

/** The route and zone story for the chart's screen-reader summary. */
function chartSummary(config: MissionConfig): string {
  const [startNorth, startEast] = config.start_position_ned
  const parts = [
    `Mission planning chart, north up. Home at ${startNorth} meters north, ${startEast} meters east.`,
  ]
  if (config.waypoints.length === 0) {
    parts.push('No waypoints yet.')
  } else {
    parts.push(`${config.waypoints.length} waypoints:`)
    config.waypoints.forEach((wp, i) => {
      parts.push(
        `${i + 1} at ${wp.position[0]} north, ${wp.position[1]} east, ${-wp.position[2]} meters altitude.`,
      )
    })
  }
  if (config.obstacles.length > 0) {
    parts.push(`${config.obstacles.length} keep-out zones:`)
    for (const ob of config.obstacles) {
      parts.push(`${ob.id} at ${ob.center_ned[0]} north, ${ob.center_ned[1]} east, radius ${ob.radius_m} meters.`)
    }
  }
  parts.push(config.avoidance.enabled ? 'Avoidance on.' : 'Avoidance off.')
  return parts.join(' ')
}

export function MissionCanvas({
  config,
  selected,
  onSelect,
  dispatch,
  documentEpoch,
  dragPreview,
  onDragPreview,
}: MissionCanvasProps) {
  const svgRef = useRef<SVGSVGElement | null>(null)
  const interactionRef = useRef<Interaction | null>(null)
  /**
   * The last two finished gestures. A double-click only counts when both
   * clicks were clean presses on empty background, so a pan or a node drag
   * (either over `DRAG_THRESHOLD_PX`) never drops a waypoint by accident.
   */
  const prevGestureRef = useRef<GestureRecord>({ kind: 'background', moved: false })
  const lastGestureRef = useRef<GestureRecord>({ kind: 'background', moved: false })

  /** The chart box, watched for resizes that change the frame's aspect. */
  const [frameRef, frameSize] = useElementSize<HTMLDivElement>()
  /**
   * The last measured box aspect (width / height), latched: 1 until the
   * first measurement and never sent back afterwards, so a box that
   * measures, collapses to zero, and measures again (a hidden tab, a
   * collapsed parent) cannot re-fit outside the sanctioned list. The latch
   * flips once (`aspectKnown`) and only ever false to true, and the drawn
   * frame keeps the last known aspect through a zero-size blip instead of
   * jumping to a square one.
   */
  const aspectRef = useRef(1)
  const measuredRef = useRef(false)
  if (frameSize.width > 0 && frameSize.height > 0) {
    aspectRef.current = frameSize.width / frameSize.height
    measuredRef.current = true
  }
  /** The aspect every fit and the drawn frame read, last measurement. */
  const aspect = aspectRef.current
  const aspectKnown = measuredRef.current

  /**
   * The viewport: the visible centre and span in meters. Component state,
   * outside the undo history, so zooming and panning never enable UNDO,
   * never mark the draft EDITED, and never write to localStorage.
   */
  const [view, setView] = useState<ChartView>(() => viewFit(chartBounds(config), 1))

  /**
   * Fit the frame on mount, on the first measured aspect, and whenever the
   * document identity changes (`documentEpoch`: template load, import,
   * CLEAR). Edits, undo, and redo leave the epoch alone, so they never
   * re-fit; `config` and `aspect` are read from the render the epoch change
   * belongs to, and listing them here would re-fit on every edit. A
   * zero-size blip changes neither dependency: `aspectKnown` only ever goes
   * false to true, and the latch keeps the last measured aspect for the
   * unmeasured case.
   */
  useLayoutEffect(() => {
    setView(viewFit(chartBounds(config), aspect))
  }, [documentEpoch, aspectKnown])

  /**
   * Wheel zoom at the cursor. Registered by hand with `passive: false`:
   * React's synthetic wheel listener is passive at the root and cannot call
   * preventDefault, so the page would scroll under the chart. Wheel input is
   * ignored while a drag or pan is in progress: the frame must not move
   * during an edit gesture at any point.
   */
  useEffect(() => {
    const svg = svgRef.current
    if (svg === null) return
    const onWheel = (e: WheelEvent): void => {
      e.preventDefault()
      e.stopPropagation()
      if (interactionRef.current !== null) return
      const anchor = toChartPoint(svg, e.clientX, e.clientY)
      if (anchor === null) return
      const factor = e.deltaY < 0 ? WHEEL_ZOOM_STEP : 1 / WHEEL_ZOOM_STEP
      setView((current) => zoomAt(current, anchor, factor))
    }
    svg.addEventListener('wheel', onWheel, { passive: false })
    return () => svg.removeEventListener('wheel', onWheel)
  }, [])

  /** The visible rectangle: the viewBox, the grid's extent, the chrome's frame. */
  const bounds = viewBounds(view, aspect)
  const spanX = boundsWidth(bounds)
  const spanY = boundsHeight(bounds)
  /** The visible span: grid, labels, scale bar, and chrome scale come from this. */
  const span = Math.max(spanX, spanY)
  const gridStep = gridStepM(span)
  const labelStep = labelStepM(span)
  const barLength = scaleBarLengthM(span)
  /** World size of screen chrome (labels, nodes, arrow): keeps text legible. */
  const ui = span / 55
  const zoomableIn = canZoomIn(view)
  const zoomableOut = canZoomOut(view)

  /** Chart-space coordinates of a pointer event, or null without a screen CTM. */
  function toChart(e: { clientX: number; clientY: number }): ChartPoint | null {
    const svg = svgRef.current
    if (svg === null) return null
    return toChartPoint(svg, e.clientX, e.clientY)
  }

  /** World coordinates of a pointer event, or null without a screen CTM. */
  function toWorld(e: { clientX: number; clientY: number }): { north: number; east: number } | null {
    const chart = toChart(e)
    if (chart === null) return null
    return { north: -chart.y, east: chart.x }
  }

  /** Step the zoom by a fixed factor, anchored on the view centre. */
  function zoomStep(factor: number): void {
    setView((current) => zoomAt(current, { x: current.centerX, y: current.centerY }, factor))
  }

  /** Frame the whole mission again: the FIT button's one job. */
  function fitView(): void {
    setView(viewFit(chartBounds(config), aspect))
  }

  /** Remember a finished gesture for the click and double-click guards. */
  function noteGesture(gesture: GestureRecord): void {
    prevGestureRef.current = lastGestureRef.current
    lastGestureRef.current = gesture
  }

  /** Begin a drag on a node or handle; the item also becomes selected. */
  function beginDrag(
    e: ReactPointerEvent<SVGElement>,
    kind: 'move' | 'resize',
    target: 'waypoint' | 'obstacle',
    index: number,
    north: number,
    east: number,
    radius: number,
  ): void {
    if (e.button !== 0 && e.pointerType === 'mouse') return
    // One gesture at a time: a second pointer landing mid-drag or mid-pan
    // must not overwrite the gesture state the first pointer owns.
    if (interactionRef.current !== null) return
    const world = toWorld(e)
    if (world === null) return
    e.stopPropagation()
    svgRef.current?.setPointerCapture(e.pointerId)
    interactionRef.current = {
      kind,
      target,
      index,
      pointerId: e.pointerId,
      downX: e.clientX,
      downY: e.clientY,
      offsetX: kind === 'move' ? world.east - east : 0,
      offsetY: kind === 'move' ? world.north - north : 0,
      moved: false,
      north,
      east,
      radius,
    }
    onSelect({ kind: target, index })
  }

  /** Begin a pan on the empty chart background (drag to move the view). */
  function beginPan(e: ReactPointerEvent<SVGRectElement>): void {
    if (e.button !== 0 && e.pointerType === 'mouse') return
    if (interactionRef.current !== null) return
    const svg = svgRef.current
    if (svg === null) return
    svg.setPointerCapture(e.pointerId)
    interactionRef.current = {
      kind: 'pan',
      pointerId: e.pointerId,
      downX: e.clientX,
      downY: e.clientY,
      moved: false,
      lastX: e.clientX,
      lastY: e.clientY,
    }
  }

  /** Track the pointer: pan the view, or update a drag's local preview. */
  function trackDrag(e: ReactPointerEvent<SVGElement>): void {
    const interaction = interactionRef.current
    if (interaction === null || e.pointerId !== interaction.pointerId) return
    if (interaction.kind === 'pan') {
      trackPan(interaction, e)
      return
    }
    const drag = interaction
    const world = toWorld(e)
    if (world === null) return

    if (!drag.moved) {
      const travelled = Math.hypot(e.clientX - drag.downX, e.clientY - drag.downY)
      if (travelled < DRAG_THRESHOLD_PX) return
      drag.moved = true
    }

    if (drag.kind === 'move') {
      drag.north = clampCoordinate(world.north - drag.offsetY)
      drag.east = clampCoordinate(world.east - drag.offsetX)
    } else {
      const zone = config.obstacles[drag.index]
      if (zone === undefined) return
      drag.radius = clampRadius(
        Math.hypot(world.east - zone.center_ned[1], world.north - zone.center_ned[0]),
      )
    }
    // Publish the transient for both surfaces at pointer-move rate. This is
    // the store's dragPreview slot only: no history entry, no dirty flag,
    // no storage write, no epoch advance.
    onDragPreview({
      kind: drag.kind,
      target: drag.target,
      index: drag.index,
      north: drag.north,
      east: drag.east,
      radius: drag.radius,
    })
  }

  /**
   * Pan the view so the chart follows the cursor. The frame moves only once
   * the pointer clears `DRAG_THRESHOLD_PX`, and the delta is taken from the
   * previous pointer position, so a pan starts smoothly with no jump.
   */
  function trackPan(pan: PanState, e: ReactPointerEvent<SVGElement>): void {
    const svg = svgRef.current
    if (svg === null) return
    const previous = toChartPoint(svg, pan.lastX, pan.lastY)
    const next = toChartPoint(svg, e.clientX, e.clientY)
    if (previous === null || next === null) return
    pan.lastX = e.clientX
    pan.lastY = e.clientY
    if (!pan.moved) {
      const travelled = Math.hypot(e.clientX - pan.downX, e.clientY - pan.downY)
      if (travelled < DRAG_THRESHOLD_PX) return
      pan.moved = true
    }
    // Content follows the pointer: the centre shifts opposite the world delta.
    setView((current) => panBy(current, previous.x - next.x, previous.y - next.y))
  }

  /**
   * Finish one gesture: commit a drag, clear on a bare press, pan or not.
   * This is also the lost-capture path (the svg wires
   * `onLostPointerCapture` here), so a gesture whose pointer disappears
   * still commits what was tracked and still clears the shared transient.
   * The clear runs after the commit dispatch: one event, one final state.
   */
  function endDrag(e: ReactPointerEvent<SVGElement>): void {
    const interaction = interactionRef.current
    if (interaction === null || e.pointerId !== interaction.pointerId) return
    interactionRef.current = null
    if (svgRef.current?.hasPointerCapture(e.pointerId) === true) {
      svgRef.current.releasePointerCapture(e.pointerId)
    }
    noteGesture({
      kind: interaction.kind === 'pan' ? 'background' : 'node',
      moved: interaction.moved,
    })
    if (interaction.kind === 'pan') {
      // A clean press on empty space clears the selection; a pan leaves it.
      if (!interaction.moved) onSelect(null)
      return
    }
    const drag = interaction
    if (drag.moved) {
      if (drag.kind === 'move' && drag.target === 'waypoint') {
        dispatch({ type: 'waypoint/move', index: drag.index, north: drag.north, east: drag.east })
      } else if (drag.kind === 'move') {
        dispatch({ type: 'obstacle/move', index: drag.index, north: drag.north, east: drag.east })
      } else {
        dispatch({ type: 'obstacle/resize', index: drag.index, radius_m: drag.radius })
      }
    }
    onDragPreview(null)
  }

  /** Drop a gesture without committing (the pointer was cancelled). */
  function cancelDrag(e: ReactPointerEvent<SVGElement>): void {
    const interaction = interactionRef.current
    if (interaction === null || e.pointerId !== interaction.pointerId) return
    interactionRef.current = null
    onDragPreview(null)
    noteGesture({
      kind: interaction.kind === 'pan' ? 'background' : 'node',
      moved: interaction.moved,
    })
  }

  /** Add a waypoint where the chart was double-clicked, and select it. */
  function addWaypointAt(e: { clientX: number; clientY: number }): void {
    if (config.waypoints.length >= LIMITS.max_waypoints) return // the panel's cap, honored here too
    const world = toWorld(e)
    if (world === null) return
    const index = config.waypoints.length
    dispatch({
      type: 'waypoint/add',
      north: clampCoordinate(world.north),
      east: clampCoordinate(world.east),
    })
    onSelect({ kind: 'waypoint', index })
  }

  /**
   * Double-click on empty chart adds a waypoint. The gesture guard decides,
   * not the event target (pointer capture can retarget a click to the svg):
   * both of the last two gestures must be clean presses on empty background,
   * so a pan or a node drag never drops a waypoint, and a double-click on a
   * waypoint or zone stays that item's business.
   */
  function onChartDoubleClick(e: ReactMouseEvent<SVGSVGElement>): void {
    const last = lastGestureRef.current
    const prev = prevGestureRef.current
    if (last.kind !== 'background' || prev.kind !== 'background') return
    if (last.moved || prev.moved) return
    addWaypointAt(e)
  }

  /** Nudge the focused item by one grid step; Shift takes five. */
  function nudge(
    e: React.KeyboardEvent<SVGElement>,
    target: 'waypoint' | 'obstacle',
    index: number,
  ): void {
    const step = e.shiftKey ? 5 : 1
    let dNorth = 0
    let dEast = 0
    if (e.key === 'ArrowUp') dNorth = step
    else if (e.key === 'ArrowDown') dNorth = -step
    else if (e.key === 'ArrowLeft') dEast = -step
    else if (e.key === 'ArrowRight') dEast = step
    else return
    e.preventDefault()
    onSelect({ kind: target, index })
    if (target === 'waypoint') {
      const wp = config.waypoints[index]
      if (wp === undefined) return
      dispatch({
        type: 'waypoint/move',
        index,
        north: clampCoordinate(wp.position[0] + dNorth),
        east: clampCoordinate(wp.position[1] + dEast),
      })
    } else {
      const zone = config.obstacles[index]
      if (zone === undefined) return
      dispatch({
        type: 'obstacle/move',
        index,
        north: clampCoordinate(zone.center_ned[0] + dNorth),
        east: clampCoordinate(zone.center_ned[1] + dEast),
      })
    }
  }

  /**
   * Shared keyboard behaviour for a waypoint or zone group. A gesture in
   * flight owns the chart: pointer capture gates pointer events, but keys
   * still reach the focused node, so a Delete (which shrinks the list) or an
   * arrow nudge (which dispatches a move) mid-drag would leave the drag's
   * stored index pointing at a different item when it commits. Every node
   * key is ignored until the gesture ends, the same guard beginDrag, beginPan
   * and the wheel listener use; with no gesture active the keys behave as
   * always: Enter and space select, Delete removes, arrows nudge.
   */
  function onNodeKeyDown(
    e: React.KeyboardEvent<SVGElement>,
    target: 'waypoint' | 'obstacle',
    index: number,
  ): void {
    if (interactionRef.current !== null) return
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault()
      onSelect({ kind: target, index })
      return
    }
    if (e.key === 'Delete' || e.key === 'Backspace') {
      const isSelected = selected !== null && selected.kind === target && selected.index === index
      if (!isSelected) return
      e.preventDefault()
      dispatch({ type: target === 'waypoint' ? 'waypoint/remove' : 'obstacle/remove', index })
      onSelect(null)
      return
    }
    nudge(e, target, index)
  }

  /**
   * The drawn mission: the config with the shared drag preview applied.
   * Both surfaces draw through `withDragPreview`, so what the chart shows
   * mid-drag is exactly what the 3D preview shows; with no gesture in
   * flight it is the config itself, unchanged by identity.
   */
  const drawn = withDragPreview(config, dragPreview)
  const waypoints = drawn.waypoints
  const obstacles = drawn.obstacles

  // Grid line positions over the visible rectangle (x east, y = -north):
  // the frame the viewport is showing, so panning never leaves the grid.
  const verticals = linesOver(bounds.minX, bounds.maxX, gridStep)
  const horizontals = linesOver(bounds.minY, bounds.maxY, gridStep)
  const eastLabels = linesOver(bounds.minX, bounds.maxX, labelStep)
  const northLabels = linesOver(bounds.minY, bounds.maxY, labelStep)

  // The route polyline: home, then every waypoint in order.
  const routePoints = [
    `${config.start_position_ned[1]} ${-config.start_position_ned[0]}`,
    ...waypoints.map((wp) => `${wp.position[1]} ${-wp.position[0]}`),
  ].join(' ')

  const selectedZone =
    selected?.kind === 'obstacle' ? obstacles[selected.index] : undefined

  const nodeRadius = ui * 0.9
  const stroke = { vectorEffect: 'non-scaling-stroke' as const }

  return (
    <div ref={frameRef} className="relative h-full w-full touch-none">
      <svg
        ref={svgRef}
        className="block h-full w-full"
        viewBox={boundsViewBox(bounds)}
        preserveAspectRatio="xMidYMid meet"
        role="group"
        aria-label={chartSummary(config)}
        onPointerMove={trackDrag}
        onPointerUp={endDrag}
        onPointerCancel={cancelDrag}
        onLostPointerCapture={endDrag}
        onDoubleClick={onChartDoubleClick}
      >
        {/* Chart ground and the pan surface; a clean press clears the selection. */}
        <rect
          x={bounds.minX}
          y={bounds.minY}
          width={spanX}
          height={spanY}
          className="fill-panel-2"
          onPointerDown={beginPan}
        />

        {/* Round-meter grid, labelled every fifth line. */}
        <g className="pointer-events-none" aria-hidden="true">
          {verticals.map((x) => (
            <line
              key={`v${x}`}
              x1={x}
              y1={bounds.minY}
              x2={x}
              y2={bounds.maxY}
              className="stroke-grid"
              strokeWidth={1}
              {...stroke}
            />
          ))}
          {horizontals.map((y) => (
            <line
              key={`h${y}`}
              x1={bounds.minX}
              y1={y}
              x2={bounds.maxX}
              y2={y}
              className="stroke-grid"
              strokeWidth={1}
              {...stroke}
            />
          ))}
          {eastLabels.map((x) => (
            <text
              key={`el${x}`}
              x={x}
              y={bounds.maxY - ui * 0.6}
              textAnchor="middle"
              className="fill-ink-faint"
              fontSize={ui * 0.8}
            >
              {Math.round(x)}
            </text>
          ))}
          {northLabels.map((y) => (
            <text
              key={`nl${y}`}
              x={bounds.minX + ui * 0.5}
              y={y + ui * 0.3}
              textAnchor="start"
              className="fill-ink-faint"
              fontSize={ui * 0.8}
            >
              {Math.round(-y)}
            </text>
          ))}
        </g>

        {/* Keep-out zones: amber dashed circles with a translucent core. */}
        {obstacles.map((ob, i) => {
          const cx = ob.center_ned[1]
          const cy = -ob.center_ned[0]
          const isSelected = selected?.kind === 'obstacle' && selected.index === i
          return (
            <g key={`ob${ob.id}`}>
              <circle
                cx={cx}
                cy={cy}
                r={ob.radius_m}
                className="fill-warn/10 stroke-warn"
                strokeWidth={isSelected ? 2 : 1.25}
                strokeDasharray="5 4"
                {...stroke}
              />
              <text
                x={cx}
                y={cy + ui * 0.3}
                textAnchor="middle"
                className="pointer-events-none fill-warn"
                fontSize={ui * 0.85}
              >
                {ob.id}
              </text>
              {/* Focusable, draggable hit target for the zone. */}
              <circle
                cx={cx}
                cy={cy}
                r={Math.max(ob.radius_m, ui * 1.5)}
                fill="transparent"
                role="button"
                tabIndex={0}
                aria-label={`Keep-out zone ${ob.id}, ${ob.center_ned[0]} meters north, ${ob.center_ned[1]} meters east, radius ${ob.radius_m} meters`}
                className="cursor-move focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-signal"
                onPointerDown={(e) =>
                  beginDrag(e, 'move', 'obstacle', i, ob.center_ned[0], ob.center_ned[1], ob.radius_m)
                }
                onKeyDown={(e) => onNodeKeyDown(e, 'obstacle', i)}
                onFocus={() => onSelect({ kind: 'obstacle', index: i })}
              />
            </g>
          )
        })}

        {/* Resize handle on the selected zone's east edge. */}
        {selectedZone !== undefined && selected?.kind === 'obstacle' && (
          <circle
            cx={selectedZone.center_ned[1] + selectedZone.radius_m}
            cy={-selectedZone.center_ned[0]}
            r={ui * 0.7}
            className="cursor-ew-resize fill-warn stroke-panel"
            strokeWidth={1.5}
            {...stroke}
            aria-label={`Resize keep-out zone ${selectedZone.id}`}
            onPointerDown={(e) =>
              beginDrag(
                e,
                'resize',
                'obstacle',
                selected.index,
                selectedZone.center_ned[0],
                selectedZone.center_ned[1],
                selectedZone.radius_m,
              )
            }
          />
        )}

        {/* The route: home to every waypoint, in order. */}
        <polyline
          points={routePoints}
          className="pointer-events-none fill-none stroke-signal"
          strokeWidth={1.5}
          strokeLinejoin="round"
          strokeLinecap="round"
          {...stroke}
        />

        {/* Home: a diamond at the start point. */}
        <g className="pointer-events-none" aria-hidden="true">
          <path
            d={`M ${config.start_position_ned[1]} ${-config.start_position_ned[0] - ui * 1.1} L ${
              config.start_position_ned[1] + ui * 1.1
            } ${-config.start_position_ned[0]} L ${config.start_position_ned[1]} ${
              -config.start_position_ned[0] + ui * 1.1
            } L ${config.start_position_ned[1] - ui * 1.1} ${-config.start_position_ned[0]} Z`}
            className="fill-panel-2 stroke-ink-dim"
            strokeWidth={1.25}
            {...stroke}
          />
          <text
            x={config.start_position_ned[1]}
            y={-config.start_position_ned[0] + ui * 2.6}
            textAnchor="middle"
            className="fill-ink-faint"
            fontSize={ui * 0.8}
          >
            HOME
          </text>
        </g>

        {/* Waypoints: acceptance circle, numbered node, altitude label. */}
        {waypoints.map((wp, i) => {
          const cx = wp.position[1]
          const cy = -wp.position[0]
          const isSelected = selected?.kind === 'waypoint' && selected.index === i
          return (
            <g key={`wp${i}`}>
              <circle
                cx={cx}
                cy={cy}
                r={wp.radius}
                className="pointer-events-none fill-signal/5 stroke-signal/40"
                strokeWidth={1}
                {...stroke}
              />
              {isSelected && (
                <circle
                  cx={cx}
                  cy={cy}
                  r={nodeRadius * 1.7}
                  className="pointer-events-none fill-none stroke-signal"
                  strokeWidth={1.5}
                  {...stroke}
                />
              )}
              <text
                x={cx}
                y={cy - nodeRadius - ui * 0.35}
                textAnchor="middle"
                className="pointer-events-none fill-ink-faint"
                fontSize={ui * 0.8}
              >
                {-wp.position[2]} m
              </text>
              {/* Focusable, draggable node carrying the route index. */}
              <circle
                cx={cx}
                cy={cy}
                r={nodeRadius}
                role="button"
                tabIndex={0}
                aria-label={`Waypoint ${i + 1}, ${wp.position[0]} meters north, ${wp.position[1]} meters east, ${-wp.position[2]} meters altitude`}
                className={`cursor-move focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-signal ${
                  isSelected ? 'fill-signal stroke-panel' : 'fill-panel-3 stroke-signal'
                }`}
                strokeWidth={1.5}
                {...stroke}
                onPointerDown={(e) => beginDrag(e, 'move', 'waypoint', i, wp.position[0], wp.position[1], wp.radius)}
                onKeyDown={(e) => onNodeKeyDown(e, 'waypoint', i)}
                onFocus={() => onSelect({ kind: 'waypoint', index: i })}
              />
              <text
                x={cx}
                y={cy + ui * 0.3}
                textAnchor="middle"
                className={`pointer-events-none ${isSelected ? 'fill-panel' : 'fill-signal'}`}
                fontSize={ui * 0.95}
              >
                {i + 1}
              </text>
            </g>
          )
        })}

        {/* North arrow and scale bar: quiet chrome in the frame's corners. */}
        <g className="pointer-events-none" aria-hidden="true">
          <g transform={`translate(${bounds.maxX - ui * 2} ${bounds.minY + ui * 1.5})`}>
            <line
              x1={0}
              y1={ui * 2.4}
              x2={0}
              y2={0}
              className="stroke-ink-dim"
              strokeWidth={1.25}
              {...stroke}
            />
            <path
              d={`M ${-ui * 0.5} ${ui * 0.7} L 0 0 L ${ui * 0.5} ${ui * 0.7}`}
              className="fill-none stroke-ink-dim"
              strokeWidth={1.25}
              {...stroke}
            />
            <text x={0} y={-ui * 0.5} textAnchor="middle" className="fill-ink-dim" fontSize={ui * 0.9}>
              N
            </text>
          </g>
          <g transform={`translate(${bounds.minX + ui * 1.5} ${bounds.maxY - ui * 2})`}>
            <line
              x1={0}
              y1={0}
              x2={barLength}
              y2={0}
              className="stroke-ink-dim"
              strokeWidth={1.25}
              {...stroke}
            />
            <line x1={0} y1={-ui * 0.5} x2={0} y2={ui * 0.5} className="stroke-ink-dim" strokeWidth={1.25} {...stroke} />
            <line
              x1={barLength}
              y1={-ui * 0.5}
              x2={barLength}
              y2={ui * 0.5}
              className="stroke-ink-dim"
              strokeWidth={1.25}
              {...stroke}
            />
            <text
              x={barLength / 2}
              y={-ui * 0.9}
              textAnchor="middle"
              className="fill-ink-dim"
              fontSize={ui * 0.8}
            >
              {barLength} m
            </text>
          </g>
        </g>
      </svg>

      {/* Chart chrome: zoom and fit controls. They touch only the local
          viewport state, so they never enter the undo history, the dirty
          flag, or localStorage. */}
      <div
        role="group"
        aria-label="Chart zoom controls"
        className="absolute bottom-2 right-2 flex items-center gap-1"
      >
        <button
          type="button"
          onClick={() => zoomStep(ZOOM_STEP)}
          disabled={!zoomableIn}
          title={zoomableIn ? `Zoom in, ${ZOOM_STEP}x per press` : 'Already at the closest zoom'}
          className={CHROME_BUTTON}
        >
          ZOOM IN
        </button>
        <button
          type="button"
          onClick={() => zoomStep(1 / ZOOM_STEP)}
          disabled={!zoomableOut}
          title={zoomableOut ? `Zoom out, ${ZOOM_STEP}x per press` : 'Already at the widest view'}
          className={CHROME_BUTTON}
        >
          ZOOM OUT
        </button>
        <button
          type="button"
          onClick={fitView}
          title="Frame the whole mission"
          className={CHROME_BUTTON}
        >
          FIT
        </button>
      </div>

      {/* Empty-route prompt, laid over the chart without stealing pointers. */}
      {config.waypoints.length === 0 && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center p-4">
          <div className="border border-grid bg-panel/85 px-4 py-3 text-center">
            <p className="font-mono text-[10px] tracking-[0.2em] text-ink-dim">NO ROUTE YET</p>
            <p className="mt-1 max-w-[22rem] text-[11px] leading-relaxed text-ink-faint">
              Double-click the chart to drop a waypoint, or use Add waypoint in the route
              panel.
            </p>
          </div>
        </div>
      )}

      {/* Text alternative: the same route as a plain list. */}
      <ul className="sr-only">
        {config.waypoints.map((wp, i) => (
          <li key={`sr${i}`}>
            Waypoint {i + 1}: {wp.position[0]} m north, {wp.position[1]} m east, altitude{' '}
            {-wp.position[2]} m, speed {wp.speed} m/s, acceptance radius {wp.radius} m.
          </li>
        ))}
        {config.obstacles.map((ob) => (
          <li key={`sro${ob.id}`}>
            Keep-out zone {ob.id}: {ob.center_ned[0]} m north, {ob.center_ned[1]} m east,
            altitude {-ob.center_ned[2]} m, radius {ob.radius_m} m.
          </li>
        ))}
      </ul>
    </div>
  )
}
