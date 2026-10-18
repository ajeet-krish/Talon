import { useFlightStore } from '@/store/useFlightStore'
import { SCENARIOS } from '@/data/scenarios'
import type { ScenarioId } from '@/types/flight'

/**
 * Vertical instrument list of simulation scenarios. Active entry gets a
 * signal-cyan rail and label; loading entries show a fetch indicator. Once an
 * in-browser run has completed, a LIVE RUN entry appears at the top of the
 * list, holding the result of the most recent run.
 */
export function ScenarioSelector() {
  const active = useFlightStore((s) => s.activeScenario)
  const loading = useFlightStore((s) => s.loading)
  const loaded = useFlightStore((s) => s.scenarios)
  const setActive = useFlightStore((s) => s.setActive)
  const liveRun = useFlightStore((s) => s.liveRun)

  const liveActive = active === 'live'
  const liveLoaded = loaded.has('live')

  return (
    <div className="flex flex-col gap-px">
      {liveLoaded && (
        <button
          type="button"
          onClick={() => void setActive('live')}
          aria-current={liveActive ? 'true' : undefined}
          className={`group flex flex-col items-start gap-0.5 border-l-2 px-3 py-2 text-left transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-2px] focus-visible:outline-signal ${
            liveActive
              ? 'border-signal bg-panel-3'
              : 'border-transparent hover:border-grid-2 hover:bg-panel-2'
          }`}
        >
          <span
            className={`font-mono text-[11px] font-semibold tracking-wider ${
              liveActive ? 'text-signal' : 'text-ink-dim group-hover:text-ink'
            }`}
          >
            LIVE RUN
          </span>
          <span className="text-[10px] leading-tight text-ink-faint">
            {liveRun === null
              ? 'in-browser WASM run'
              : `${liveRun.waypoints_reached}/${liveRun.waypoint_count} wpt, EKF ${liveRun.ekf_position_rmse_m.toFixed(2)} m`}
          </span>
        </button>
      )}
      {(Object.keys(SCENARIOS) as ScenarioId[]).map((id) => {
        const meta = SCENARIOS[id]
        const isActive = id === active
        const isLoaded = loaded.has(id)
        return (
          <button
            key={id}
            type="button"
            onClick={() => void setActive(id)}
            aria-current={isActive ? 'true' : undefined}
            className={`group flex flex-col items-start gap-0.5 border-l-2 px-3 py-2 text-left transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-2px] focus-visible:outline-signal ${
              isActive
                ? 'border-signal bg-panel-3'
                : 'border-transparent hover:border-grid-2 hover:bg-panel-2'
            }`}
          >
            <span
              className={`font-mono text-[11px] font-semibold tracking-wider ${
                isActive ? 'text-signal' : 'text-ink-dim group-hover:text-ink'
              }`}
            >
              {meta.label}
            </span>
            <span className="text-[10px] leading-tight text-ink-faint">
              {!isLoaded && loading && isActive ? 'loading\u2026' : meta.subtitle}
            </span>
          </button>
        )
      })}
    </div>
  )
}
