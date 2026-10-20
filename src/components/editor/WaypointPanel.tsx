/**
 * Waypoint panel (FlightLab Phase 8c): the ordered route list. Each row
 * carries its index, altitude (authored as meters above the ground through
 * the sanctioned `waypointAltitudeM` / `withWaypointAltitude` pair), leg
 * speed, acceptance radius, and reorder/remove buttons. Reordering is
 * button-driven on purpose: the route's order is the flight's order, and a
 * control that cannot be reached by keyboard cannot be trusted with it.
 *
 * Selecting a row points the editor at that waypoint, which highlights it on
 * the chart; a waypoint-level warning (inside a zone, repeating its
 * predecessor, radius overlapping) shows inline in amber because none of
 * them block a run.
 */

import { LIMITS } from '@/types/generated/talon_core'
import type { EditorCommand, EditorSelection, MissionConfig } from '@/utils/editor'
import {
  clampCoordinate,
  waypointAltitudeM,
  waypointWarning,
  withWaypointAltitude,
} from '@/utils/editor'
import { NumberField } from './NumberField'
import { Section } from './Section'

export interface WaypointPanelProps {
  /** The mission being edited. */
  config: MissionConfig
  /** What the editor points at, or null. */
  selected: EditorSelection
  /** Apply one edit. */
  dispatch: (cmd: EditorCommand) => void
  /** Select one waypoint or zone, or clear the selection with null. */
  onSelect: (sel: EditorSelection) => void
}

/** Small square icon button used for reorder and remove. */
function RowButton({
  label,
  disabled,
  onClick,
  children,
}: {
  label: string
  disabled?: boolean
  onClick: () => void
  children: string
}) {
  return (
    <button
      type="button"
      aria-label={label}
      disabled={disabled}
      onClick={onClick}
      className="h-6 w-6 border border-grid font-mono text-[10px] leading-none text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-30 disabled:hover:bg-transparent"
    >
      {children}
    </button>
  )
}

export function WaypointPanel({ config, selected, dispatch, onSelect }: WaypointPanelProps) {
  const waypoints = config.waypoints
  const atCap = waypoints.length >= LIMITS.max_waypoints

  /** Replace one waypoint, preserving everything the field did not edit. */
  function setWaypoint(index: number, next: MissionConfig['waypoints'][number]): void {
    dispatch({ type: 'waypoint/set', index, waypoint: next })
  }

  /**
   * Add a waypoint ten meters beyond the last one, or at the plan origin.
   * The coordinates go through the shared `clampCoordinate`, the same rule
   * the chart's double-click add uses, so both paths land in bounds.
   */
  function addWaypoint(): void {
    if (atCap) return
    const last = waypoints[waypoints.length - 1]
    const north = clampCoordinate(last === undefined ? 10 : last.position[0] + 10)
    const east = clampCoordinate(last === undefined ? 0 : last.position[1])
    const index = waypoints.length
    dispatch({ type: 'waypoint/add', north, east })
    onSelect({ kind: 'waypoint', index })
  }

  return (
    <Section title="Waypoints" aside={`${waypoints.length} / ${LIMITS.max_waypoints}`}>
      {waypoints.length === 0 ? (
        <p className="pb-2 text-[11px] leading-relaxed text-ink-faint">
          No waypoints yet. Add one here, or double-click the chart where the route should
          turn.
        </p>
      ) : (
        <ol className="flex flex-col gap-1.5">
          {waypoints.map((wp, i) => {
            const isSelected = selected?.kind === 'waypoint' && selected.index === i
            const warning = waypointWarning(config, i)
            return (
              <li
                key={i}
                className={`border-l-2 px-2 py-1.5 transition-colors ${
                  isSelected ? 'border-signal bg-panel-3' : 'border-grid bg-panel-2/40'
                }`}
              >
                <div className="flex items-center gap-2">
                  <button
                    type="button"
                    onClick={() => onSelect({ kind: 'waypoint', index: i })}
                    aria-pressed={isSelected}
                    className={`font-mono text-[11px] font-semibold tracking-[0.1em] transition-colors ${
                      isSelected ? 'text-signal' : 'text-ink-dim hover:text-ink'
                    }`}
                  >
                    WP {i + 1}
                  </button>
                  <div className="ml-auto flex items-center gap-1">
                    <RowButton
                      label={`Move waypoint ${i + 1} earlier`}
                      disabled={i === 0}
                      onClick={() => dispatch({ type: 'waypoint/reorder', index: i, delta: -1 })}
                    >
                      UP
                    </RowButton>
                    <RowButton
                      label={`Move waypoint ${i + 1} later`}
                      disabled={i === waypoints.length - 1}
                      onClick={() => dispatch({ type: 'waypoint/reorder', index: i, delta: 1 })}
                    >
                      DN
                    </RowButton>
                    <RowButton
                      label={`Remove waypoint ${i + 1}`}
                      onClick={() => {
                        dispatch({ type: 'waypoint/remove', index: i })
                        if (isSelected) onSelect(null)
                      }}
                    >
                      X
                    </RowButton>
                  </div>
                </div>
                <div className="mt-1.5 grid grid-cols-3 gap-1.5">
                  <NumberField
                    label="Altitude m"
                    value={waypointAltitudeM(wp)}
                    suffix="m"
                    min={0}
                    max={LIMITS.max_takeoff_altitude_m}
                    onCommit={(altitude) => setWaypoint(i, withWaypointAltitude(wp, altitude))}
                  />
                  <NumberField
                    label={`Speed m/s (max ${LIMITS.max_waypoint_speed_mps})`}
                    value={wp.speed}
                    suffix="m/s"
                    min={0}
                    max={LIMITS.max_waypoint_speed_mps}
                    onCommit={(speed) => setWaypoint(i, { ...wp, speed })}
                  />
                  <NumberField
                    label="Radius m"
                    value={wp.radius}
                    suffix="m"
                    min={0}
                    max={LIMITS.max_radius_m}
                    onCommit={(radius) => setWaypoint(i, { ...wp, radius })}
                  />
                </div>
                {warning !== null && (
                  <p className="mt-1.5 font-mono text-[9px] leading-tight text-warn">
                    {warning.toUpperCase()}
                  </p>
                )}
              </li>
            )
          })}
        </ol>
      )}

      <button
        type="button"
        onClick={addWaypoint}
        disabled={atCap}
        title={atCap ? `A mission may hold at most ${LIMITS.max_waypoints} waypoints` : undefined}
        className="mt-2 w-full border border-grid px-2 py-1.5 font-mono text-[10px] tracking-[0.15em] text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-40"
      >
        ADD WAYPOINT
      </button>
    </Section>
  )
}
