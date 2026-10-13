import { useMemo } from 'react'
import type { FlightData } from '@/types/flight'
import { buildPhaseSegments, phaseAtTime } from '@/utils/interpolation'
import { PHASE_COLORS } from '@/utils/colors'
import { useFlightStore } from '@/store/useFlightStore'

/**
 * The signature element: a full-width scrubber whose track is the flight
 * itself, cut into phase-colored segments (Wong palette, same as the
 * Matplotlib plots). Dragging the cyan playhead walks the state machine
 * takeoff -> cruise -> waypoint -> rtl -> land, and each segment lights up
 * as the playhead enters it. Event ticks mark discrete transitions
 * (arming, guard checks, landings) from the autopilot log.
 */
export function Timeline({ data }: { data: FlightData }) {
  const currentTime = useFlightStore((s) => s.currentTime)
  const setTime = useFlightStore((s) => s.setTime)

  const duration = data.meta.duration_s
  const segments = useMemo(
    () => buildPhaseSegments(duration, data.phase_timeline),
    [duration, data.phase_timeline],
  )
  const currentPhase = phaseAtTime(data.phase_timeline, currentTime)

  return (
    <div className="w-full">
      <div className="relative h-7 select-none" aria-hidden="true">
        {/* Phase segments: the state machine story, one bar per phase. */}
        <div className="absolute inset-x-0 top-1 h-4 overflow-hidden rounded-[2px] border border-grid bg-panel-2">
          {segments.map((seg, i) => {
            const left = (seg.t0 / duration) * 100
            const width = ((seg.t1 - seg.t0) / duration) * 100
            const active = seg.phase === currentPhase && currentTime >= seg.t0 && currentTime < seg.t1
            return (
              <div
                key={`${seg.phase}-${i}`}
                title={`${seg.phase}  ${seg.t0.toFixed(1)}-${seg.t1.toFixed(1)}s`}
                className="absolute inset-y-0 border-r border-panel transition-opacity duration-150"
                style={{
                  left: `${left}%`,
                  width: `${width}%`,
                  backgroundColor: PHASE_COLORS[seg.phase],
                  opacity: active ? 1 : 0.4,
                }}
              />
            )
          })}
          {/* Discrete event ticks from the autopilot event log. */}
          {data.event_log.map((evt) => (
            <div
              key={`${evt.time}-${evt.event}`}
              title={`${evt.event} @ ${evt.time.toFixed(2)}s (${evt.from_phase} -> ${evt.to_phase})`}
              className="absolute top-0 h-full w-px bg-ink/70"
              style={{ left: `${(evt.time / duration) * 100}%` }}
            />
          ))}
        </div>
        {/* Scrubber: full-height drag surface, thumb doubles as playhead. */}
        <input
          type="range"
          className="scrubber absolute inset-0 h-7 w-full"
          min={0}
          max={duration}
          step={data.meta.dt}
          value={currentTime}
          onChange={(e) => setTime(Number(e.target.value))}
          aria-label="Scrub flight time"
          aria-valuetext={`${currentTime.toFixed(2)} seconds, phase ${currentPhase}`}
        />
      </div>
      <div className="mt-1 flex justify-between font-mono text-[10px] text-ink-faint tabular-nums">
        <span>T+{currentTime.toFixed(2)}s</span>
        <span>{duration.toFixed(2)}s</span>
      </div>
    </div>
  )
}
