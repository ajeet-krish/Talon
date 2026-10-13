import { useMemo } from 'react'
import type { FlightData, FlightPhase } from '@/types/flight'
import { PHASE_COLORS } from '@/utils/colors'

/**
 * Wong-palette phase key, listing only the phases the active scenario
 * actually traverses. Pinned top-right of the scene, same panel chrome as
 * the HUD, so the color language of the trail and timeline is always
 * one glance away.
 */
export function Legend({ data }: { data: FlightData }) {
  const phases = useMemo(() => {
    const seen = new Set<FlightPhase>()
    for (const [, phase] of data.phase_timeline) seen.add(phase)
    return [...seen]
  }, [data.phase_timeline])

  if (phases.length === 0) return null

  return (
    <div className="pointer-events-none absolute right-3 top-3 rounded-[3px] border border-grid bg-panel/85 px-3 py-2 backdrop-blur-sm">
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
  )
}
