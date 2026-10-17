import { useFlightStore } from '@/store/useFlightStore'
import type { ScenarioId } from '@/types/flight'

interface ScenarioMeta {
  label: string
  subtitle: string
}

/**
 * Scenario facts pulled from the data itself: durations are exact, and the
 * subtitles name the engineering story each run demonstrates.
 */
const SCENARIOS: Record<ScenarioId, ScenarioMeta> = {
  gps_outage: {
    label: 'GPS OUTAGE',
    subtitle: 'waypoint mission, GPS denied mid-flight',
  },
  wind: {
    label: 'WIND',
    subtitle: '2 m/s gust, hold with wind estimation',
  },
  emergency: {
    label: 'EMERGENCY',
    subtitle: 'critical fault, EKF divergence',
  },
  obstacle_field: {
    label: 'OBSTACLE FIELD',
    subtitle: 'detour avoidance around keep-out zones',
  },
}

/**
 * Vertical instrument list of simulation scenarios. Active entry gets a
 * signal-cyan rail and label; loading entries show a fetch indicator.
 */
export function ScenarioSelector() {
  const active = useFlightStore((s) => s.activeScenario)
  const loading = useFlightStore((s) => s.loading)
  const loaded = useFlightStore((s) => s.scenarios)
  const setActive = useFlightStore((s) => s.setActive)

  return (
    <div className="flex flex-col gap-px">
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
