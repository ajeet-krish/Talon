import type { FlightData } from '@/types/flight'
import { nedToScene, phaseAtTime, sampleVec3 } from '@/utils/interpolation'
import { PHASE_COLORS } from '@/utils/colors'
import {
  CLEARANCE_CAUTION_M,
  CLEARANCE_COLORS,
  clearanceBand,
  nearestObstacleId,
  surfaceClearance,
} from '@/utils/clearance'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * Compact top-left telemetry readout: mission clock, live phase chip
 * (background is the Wong phase color, so the chip changes hue as the
 * state machine advances), altitude, ground speed, and, when the scenario
 * ships keep-out zones, the live surface clearance to the nearest zone with
 * its name. Numbers are mono and tabular so digits do not jitter while they
 * update every frame.
 */
export function HUD({ data }: { data: FlightData }) {
  const currentTime = useFlightStore((s) => s.currentTime)

  const phase = phaseAtTime(data.phase_timeline, currentTime)
  const pos = sampleVec3(currentTime, data.time_s, data.truth_position)
  const vel = sampleVec3(currentTime, data.time_s, data.truth_velocity)
  const [, altUp] = nedToScene(pos)
  const altitude = Math.max(altUp, 0)
  const speed = Math.hypot(vel[0], vel[1], vel[2])

  const hasObstacles = data.obstacles.length > 0
  const clearance = hasObstacles ? surfaceClearance(pos, data.obstacles) : Infinity
  const band = hasObstacles
    ? clearanceBand(clearance, CLEARANCE_CAUTION_M)
    : 'clear'
  const nearest = hasObstacles ? nearestObstacleId(pos, data.obstacles) : null

  return (
    <div className="pointer-events-none absolute left-3 top-3 flex flex-col gap-1.5 font-mono">
      <div className="flex items-center gap-2">
        <span className="text-[10px] tracking-wider text-ink-faint">T+</span>
        <span className="text-lg font-semibold leading-none tabular-nums text-ink">
          {currentTime.toFixed(2)}
        </span>
        <span className="text-[10px] text-ink-faint">S</span>
        <span
          className="ml-1 rounded-[2px] px-1.5 py-0.5 text-[10px] font-semibold tracking-widest text-panel"
          style={{ backgroundColor: PHASE_COLORS[phase] }}
        >
          {phase.toUpperCase()}
        </span>
      </div>
      <div className="flex gap-3 text-[10px] tabular-nums">
        <span className="text-ink-dim">
          ALT <span className="text-ink">{altitude.toFixed(1)}</span> M
        </span>
        <span className="text-ink-dim">
          SPD <span className="text-ink">{speed.toFixed(2)}</span> M/S
        </span>
      </div>
      {hasObstacles && (
        <div className="flex items-center gap-2 text-[10px] tabular-nums">
          <span className="text-ink-faint">CLR</span>
          <span
            className="font-semibold"
            style={{ color: CLEARANCE_COLORS[band] }}
          >
            {clearance.toFixed(2)} M
          </span>
          <span className="tracking-wider text-ink-faint">
            {band === 'breach' ? 'BREACH' : (nearest ?? '')}
          </span>
        </div>
      )}
    </div>
  )
}
