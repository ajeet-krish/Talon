import { useFlightStore } from '@/store/useFlightStore'
import { FlightScene } from '@/components/scene/FlightScene'
import { HUD } from '@/components/ui/HUD'
import { Legend } from '@/components/ui/Legend'
import { StatusPanel } from '@/components/ui/StatusPanel'

/**
 * Interactive replay: the full-bleed 3D scene with the HUD and legend
 * overlaid. Loading and error states render as an instrument panel over the
 * canvas so the view never shows an empty black frame.
 */
export function ReplayView() {
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))
  const loading = useFlightStore((s) => s.loading)
  const error = useFlightStore((s) => s.error)

  return (
    <div className="relative min-h-[280px] flex-1">
      <FlightScene />
      {data && <HUD data={data} />}
      {data && <Legend data={data} />}
      {!data && (
        <div className="absolute inset-0 flex items-center justify-center">
          {error ? (
            <StatusPanel title="TELEMETRY LOAD FAILED" detail={error} tone="error" />
          ) : (
            <StatusPanel
              title={loading ? 'LOADING TELEMETRY\u2026' : 'NO SCENARIO LOADED'}
            />
          )}
        </div>
      )}
    </div>
  )
}
