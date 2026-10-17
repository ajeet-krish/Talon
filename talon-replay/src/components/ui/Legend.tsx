import { useMemo } from 'react'
import type { FlightData, FlightPhase } from '@/types/flight'
import { PHASE_COLORS } from '@/utils/colors'
import { CLEARANCE_COLORS, minSurfaceClearance } from '@/utils/clearance'

/**
 * Instrument key pinned top-right of the scene. The flight-phase block lists
 * only the phases the active scenario traverses. When the scenario ships
 * keep-out zones, a second block lists each zone with its radius and the
 * minimum surface clearance achieved over the whole flight, the number the
 * obstacle field is graded on. Same panel chrome as the HUD, so the color
 * language of the trail, timeline, and clearance bands is one glance away.
 */
export function Legend({ data }: { data: FlightData }) {
  const phases = useMemo(() => {
    const seen = new Set<FlightPhase>()
    for (const [, phase] of data.phase_timeline) seen.add(phase)
    return [...seen]
  }, [data.phase_timeline])

  const obstacles = data.obstacles
  const minClearance = useMemo(
    () => minSurfaceClearance(data.truth_position, obstacles),
    [data.truth_position, obstacles],
  )

  if (phases.length === 0 && obstacles.length === 0) return null

  return (
    <div className="pointer-events-none absolute right-3 top-3 flex flex-col gap-3 rounded-[3px] border border-grid bg-panel/85 px-3 py-2 backdrop-blur-sm">
      {phases.length > 0 && (
        <div>
          <div className="mb-1.5 font-mono text-[9px] tracking-widest text-ink-faint">
            FLIGHT PHASES
          </div>
          <div className="flex flex-col gap-1">
            {phases.map((phase) => (
              <div key={phase} className="flex items-center gap-2">
                <span
                  className="h-2 w-2 shrink-0 rounded-[1px]"
                  style={{ backgroundColor: PHASE_COLORS[phase] }}
                />
                <span className="font-mono text-[10px] uppercase tracking-wider text-ink-dim">
                  {phase}
                </span>
              </div>
            ))}
          </div>
        </div>
      )}
      {obstacles.length > 0 && (
        <div>
          <div className="mb-1.5 font-mono text-[9px] tracking-widest text-ink-faint">
            KEEP-OUT ZONES
          </div>
          <div className="flex flex-col gap-1">
            {obstacles.map((obstacle) => (
              <div key={obstacle.id} className="flex items-center gap-2">
                <span
                  className="h-2 w-2 shrink-0 rounded-full"
                  style={{ backgroundColor: CLEARANCE_COLORS.clear }}
                />
                <span className="font-mono text-[10px] tracking-wider text-ink-dim">
                  {obstacle.id}
                </span>
                <span className="ml-auto pl-3 font-mono text-[10px] tabular-nums text-ink-faint">
                  R{obstacle.radius_m.toFixed(1)}
                </span>
              </div>
            ))}
          </div>
          <div className="mt-1.5 flex items-center justify-between gap-3 border-t border-grid pt-1.5 font-mono text-[10px] tabular-nums">
            <span className="text-ink-faint">MIN CLR</span>
            <span className="font-semibold text-ink">
              {minClearance.toFixed(2)} M
            </span>
          </div>
        </div>
      )}
    </div>
  )
}
