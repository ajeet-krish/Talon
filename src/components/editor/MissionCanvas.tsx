/**
 * Mission planning chart (FlightLab Phase 8c). A true-scale, north-up
 * top-down view of the authored mission: a round-meter grid, the numbered
 * route from home through every waypoint with its acceptance circles and
 * altitudes, keep-out zones as amber dashed circles, a north arrow, and a
 * scale bar. The frame comes from `chartBounds` so the whole mission is
 * always in view; chrome (node dots, labels, the arrow) is sized off the
 * span so it stays legible at any zoom.
 *
 * Editing: click a node to select it, drag a waypoint or zone to move it
 * (one undo entry per drag, dispatched on pointer up), drag the selected
 * zone's east-edge handle to resize it, double-click empty chart to add a
 * waypoint, single-click empty chart to clear the selection. Keyboard: focus
 * a node, Enter or Space selects, arrows nudge 1 m (Shift 5 m), Delete
 * removes the selected item. Dragging uses pointer capture with world
 * coordinates from `getScreenCTM`, so it tracks the cursor under any zoom
 * and works with touch (`touch-action: none`).
 */

import { useRef, useState } from 'react'
import type { PointerEvent as ReactPointerEvent } from 'react'
import { LIMITS } from '@/types/generated/talon_core'
import { clampCoordinate } from '@/utils/editor'
import type { EditorCommand, EditorSelection, MissionConfig } from '@/utils/editor'
import {
  boundsHeight,
  boundsViewBox,
  boundsWidth,
  chartBounds,
  gridStepM,
  labelStepM,
  scaleBarLengthM,
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
}

/** A live drag: what is being dragged, where it started, and the last point. */
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

/** Clamp a dragged radius to a sensible, validator-legal range. */
function clampRadius(value: number): number {
  return Math.min(Math.max(value, 0.5), Math.min(200, LIMITS.max_radius_m))
}

/** Distance (px) the pointer must travel before a press counts as a drag. */
const DRAG_THRESHOLD_PX = 2

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

export function MissionCanvas({ config, selected, onSelect, dispatch }: MissionCanvasProps) {
  const svgRef = useRef<SVGSVGElement | null>(null)
  const dragRef = useRef<DragState | null>(null)
  const [preview, setPreview] = useState<DragState | null>(null)

  const bounds = chartBounds(config)
  const spanX = boundsWidth(bounds)
  const spanY = boundsHeight(bounds)
  const span = Math.max(spanX, spanY)
  const gridStep = gridStepM(span)
  const labelStep = labelStepM(span)
  const barLength = scaleBarLengthM(span)
  /** World size of screen chrome (labels, nodes, arrow): keeps text legible. */
  const ui = span / 55

  /** World coordinates of a pointer event, or null without a screen CTM. */
  function toWorld(e: { clientX: number; clientY: number }): { north: number; east: number } | null {
    const svg = svgRef.current
    if (svg === null) return null
    const ctm = svg.getScreenCTM()
    if (ctm === null) return null
    const point = svg.createSVGPoint()
    point.x = e.clientX
    point.y = e.clientY
    const local = point.matrixTransform(ctm.inverse())
    return { north: -local.y, east: local.x }
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
    const world = toWorld(e)
    if (world === null) return
    e.stopPropagation()
    svgRef.current?.setPointerCapture(e.pointerId)
    dragRef.current = {
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

  /** Track the pointer; update the local preview (no history entry yet). */
  function trackDrag(e: ReactPointerEvent<SVGElement>): void {
    const drag = dragRef.current
    if (drag === null || e.pointerId !== drag.pointerId) return
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
    setPreview({ ...drag })
  }

  /** Commit one command for the whole drag, or drop a bare click. */
  function endDrag(e: ReactPointerEvent<SVGElement>): void {
    const drag = dragRef.current
    if (drag === null || e.pointerId !== drag.pointerId) return
    dragRef.current = null
    setPreview(null)
    if (svgRef.current?.hasPointerCapture(e.pointerId) === true) {
      svgRef.current.releasePointerCapture(e.pointerId)
    }
    if (!drag.moved) return
    if (drag.kind === 'move' && drag.target === 'waypoint') {
      dispatch({ type: 'waypoint/move', index: drag.index, north: drag.north, east: drag.east })
    } else if (drag.kind === 'move') {
      dispatch({ type: 'obstacle/move', index: drag.index, north: drag.north, east: drag.east })
    } else {
      dispatch({ type: 'obstacle/resize', index: drag.index, radius_m: drag.radius })
    }
  }

  /** Drop a drag without committing (the pointer was cancelled). */
  function cancelDrag(e: ReactPointerEvent<SVGElement>): void {
    const drag = dragRef.current
    if (drag === null || e.pointerId !== drag.pointerId) return
    dragRef.current = null
    setPreview(null)
  }

  /** Add a waypoint where the chart was double-clicked, and select it. */
  function addWaypointAt(e: ReactPointerEvent<SVGRectElement>): void {
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

  /** Shared keyboard behaviour for a waypoint or zone group. */
  function onNodeKeyDown(
    e: React.KeyboardEvent<SVGElement>,
    target: 'waypoint' | 'obstacle',
    index: number,
  ): void {
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

  /** The drawn waypoints: the config, with any drag preview applied. */
  const waypoints = config.waypoints.map((wp, i) => {
    if (preview?.kind === 'move' && preview.target === 'waypoint' && preview.index === i) {
      return { ...wp, position: [preview.north, preview.east, wp.position[2]] as [number, number, number] }
    }
    return wp
  })

  /** The drawn zones: the config, with any move or resize preview applied. */
  const obstacles = config.obstacles.map((ob, i) => {
    if (preview === null || preview.target !== 'obstacle' || preview.index !== i) return ob
    const north = preview.kind === 'move' ? preview.north : ob.center_ned[0]
    const east = preview.kind === 'move' ? preview.east : ob.center_ned[1]
    const radius = preview.kind === 'resize' ? preview.radius : ob.radius_m
    return { ...ob, center_ned: [north, east, ob.center_ned[2]] as [number, number, number], radius_m: radius }
  })

  // Grid line positions in chart space (x east, y = -north).
  const verticals: number[] = []
  for (let x = Math.ceil(bounds.minX / gridStep) * gridStep; x <= bounds.maxX; x += gridStep) {
    verticals.push(x)
  }
  const horizontals: number[] = []
  for (let y = Math.ceil(bounds.minY / gridStep) * gridStep; y <= bounds.maxY; y += gridStep) {
    horizontals.push(y)
  }
  const eastLabels: number[] = []
  for (let x = Math.ceil(bounds.minX / labelStep) * labelStep; x <= bounds.maxX; x += labelStep) {
    eastLabels.push(x)
  }
  const northLabels: number[] = []
  for (let y = Math.ceil(bounds.minY / labelStep) * labelStep; y <= bounds.maxY; y += labelStep) {
    northLabels.push(y)
  }

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
    <div className="relative h-full w-full touch-none">
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
      >
        {/* Chart ground and the empty-chart click surface. */}
        <rect
          x={bounds.minX}
          y={bounds.minY}
          width={spanX}
          height={spanY}
          className="fill-panel-2"
          onClick={() => onSelect(null)}
          onDoubleClick={addWaypointAt}
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
