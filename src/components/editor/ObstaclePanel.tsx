/**
 * Keep-out zone panel (FlightLab Phase 8c): the zone list plus the
 * avoidance law's switches. Each zone carries its id, radius, and altitude;
 * selecting a row highlights the zone on the chart and reveals its resize
 * handle. The panel states the load-bearing fact about zones plainly: with
 * avoidance off they are scenery, and the autopilot flies the planned legs
 * straight through them.
 */

import { LIMITS } from '@/types/generated/talon_core'
import type { Obstacle } from '@/types/generated/talon_core'
import type { EditorCommand, EditorSelection, MissionConfig } from '@/utils/editor'
import { obstacleAltitudeM, withObstacleAltitude } from '@/utils/editor'
import { NumberField } from './NumberField'
import { Section } from './Section'

export interface ObstaclePanelProps {
  /** The mission being edited. */
  config: MissionConfig
  /** What the editor points at, or null. */
  selected: EditorSelection
  /** Apply one edit. */
  dispatch: (cmd: EditorCommand) => void
  /** Select one waypoint or zone, or clear the selection with null. */
  onSelect: (sel: EditorSelection) => void
}

export function ObstaclePanel({ config, selected, dispatch, onSelect }: ObstaclePanelProps) {
  const zones = config.obstacles
  const atCap = zones.length >= LIMITS.max_obstacles

  /** Add a zone at the plan origin, or ten meters clear of the last one. */
  function addZone(): void {
    if (atCap) return
    const last = zones[zones.length - 1]
    const north = last === undefined ? 0 : last.center_ned[0] + 10
    const east = last === undefined ? 0 : last.center_ned[1]
    dispatch({ type: 'obstacle/add', north, east })
    // The new zone is always last; select it so its handle appears.
    onSelect({ kind: 'obstacle', index: zones.length })
  }

  return (
    <Section title="Keep-out zones" aside={`${zones.length} / ${LIMITS.max_obstacles}`}>
      <p className="mb-2 text-[10px] leading-relaxed text-ink-faint">
        Zones alone do not change the path. The autopilot only detours around them when
        avoidance is on below.
      </p>

      {zones.length === 0 ? (
        <p className="pb-1 text-[11px] leading-relaxed text-ink-faint">
          No keep-out zones. Add one to mark airspace the route must not enter.
        </p>
      ) : (
        <ul className="flex flex-col gap-1.5">
          {zones.map((zone, i) => {
            const isSelected = selected?.kind === 'obstacle' && selected.index === i
            return (
              <li
                key={zone.id}
                className={`border-l-2 px-2 py-1.5 transition-colors ${
                  isSelected ? 'border-warn bg-panel-3' : 'border-grid bg-panel-2/40'
                }`}
              >
                <div className="flex items-center gap-2">
                  <button
                    type="button"
                    onClick={() => onSelect({ kind: 'obstacle', index: i })}
                    aria-pressed={isSelected}
                    className={`font-mono text-[11px] font-semibold tracking-[0.1em] transition-colors ${
                      isSelected ? 'text-warn' : 'text-ink-dim hover:text-ink'
                    }`}
                  >
                    {zone.id}
                  </button>
                  <button
                    type="button"
                    aria-label={`Remove keep-out zone ${zone.id}`}
                    onClick={() => {
                      dispatch({ type: 'obstacle/remove', index: i })
                      if (isSelected) onSelect(null)
                    }}
                    className="ml-auto h-6 w-6 border border-grid font-mono text-[10px] leading-none text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal"
                  >
                    X
                  </button>
                </div>
                <div className="mt-1.5 grid grid-cols-2 gap-1.5">
                  <NumberField
                    label="Radius m"
                    value={zone.radius_m}
                    suffix="m"
                    min={0}
                    max={LIMITS.max_radius_m}
                    onCommit={(radius_m) => {
                      const next = { ...zone, radius_m }
                      dispatch({ type: 'obstacle/set', index: i, obstacle: next })
                    }}
                  />
                  <NumberField
                    label="Altitude m"
                    value={obstacleAltitudeM(zone)}
                    suffix="m"
                    min={0}
                    max={LIMITS.max_takeoff_altitude_m}
                    onCommit={(altitude) => {
                      const next: Obstacle = withObstacleAltitude(zone, altitude)
                      dispatch({ type: 'obstacle/set', index: i, obstacle: next })
                    }}
                  />
                </div>
              </li>
            )
          })}
        </ul>
      )}

      <button
        type="button"
        onClick={addZone}
        disabled={atCap}
        title={atCap ? `A mission may hold at most ${LIMITS.max_obstacles} zones` : undefined}
        className="mt-2 w-full border border-grid px-2 py-1.5 font-mono text-[10px] tracking-[0.15em] text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-40"
      >
        ADD ZONE
      </button>

      <div className="mt-3 border-t border-grid pt-2.5">
        <label className="flex cursor-pointer items-center gap-2 text-[11px] text-ink-dim transition-colors hover:text-ink">
          <input
            type="checkbox"
            checked={config.avoidance.enabled}
            onChange={(e) =>
              dispatch({ type: 'avoidance/enabled', enabled: e.target.checked })
            }
            className="h-3 w-3 accent-signal"
          />
          Reactive avoidance
        </label>
        <div
          className={`mt-2 grid grid-cols-3 gap-1.5 ${config.avoidance.enabled ? '' : 'opacity-50'}`}
        >
          <NumberField
            label={`Margin m (max ${LIMITS.max_avoidance_margin_m})`}
            value={config.avoidance.clearance_margin_m}
            suffix="m"
            min={0}
            max={LIMITS.max_avoidance_margin_m}
            step={0.5}
            onCommit={(clearance_margin_m) =>
              dispatch({ type: 'avoidance/tuning', clearance_margin_m })
            }
          />
          <NumberField
            label={`Lookahead s (max ${LIMITS.max_avoidance_lookahead_s})`}
            value={config.avoidance.lookahead_s}
            suffix="s"
            min={0}
            max={LIMITS.max_avoidance_lookahead_s}
            step={0.5}
            onCommit={(lookahead_s) => dispatch({ type: 'avoidance/tuning', lookahead_s })}
          />
          <NumberField
            label={`Buffer m (max ${LIMITS.max_avoidance_buffer_m})`}
            value={config.avoidance.detour_buffer_m}
            suffix="m"
            min={0}
            max={LIMITS.max_avoidance_buffer_m}
            step={0.25}
            onCommit={(detour_buffer_m) => dispatch({ type: 'avoidance/tuning', detour_buffer_m })}
          />
        </div>
      </div>
    </Section>
  )
}
