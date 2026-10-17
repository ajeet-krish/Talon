import { useMemo } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { SCENARIOS } from '@/data/scenarios'
import { LineChart } from '@/components/charts/LineChart'
import { StatTile } from '@/components/ui/StatTile'
import { StatusPanel } from '@/components/ui/StatusPanel'
import { deriveTelemetry, summarizeRun } from '@/utils/telemetry'

/**
 * Telemetry screen: the recorded channels drawn as a stack of time-series.
 * Series come straight from the exported per-sample arrays, so suspension,
 * EKF, guidance, and the obstacle field all read from one payload. Every
 * chart shares the replay clock: scrubbing in the transport bar moves the
 * cursor on all of them at once.
 */
export function TelemetryView() {
  const activeId = useFlightStore((s) => s.activeScenario)
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))
  const loading = useFlightStore((s) => s.loading)

  const series = useMemo(() => (data ? deriveTelemetry(data) : []), [data])
  const summary = useMemo(() => (data ? summarizeRun(data) : null), [data])

  if (!data || !summary) {
    return (
      <div className="flex flex-1 items-center justify-center">
        <StatusPanel
          title={loading ? 'LOADING TELEMETRY\u2026' : 'NO SCENARIO LOADED'}
        />
      </div>
    )
  }

  const meta = SCENARIOS[activeId]
  const t1 = data.meta.duration_s

  return (
    <div className="min-h-0 flex-1 overflow-y-auto px-4 py-4">
      <header className="mb-4 flex flex-col gap-3 border-b border-grid pb-3 lg:flex-row lg:items-end lg:justify-between">
        <div className="min-w-0">
          <div className="font-mono text-[11px] font-semibold tracking-widest text-signal">
            {meta.label}
          </div>
          <div className="text-[11px] text-ink-faint">{meta.subtitle}</div>
        </div>
        <div className="grid shrink-0 grid-cols-2 gap-x-5 gap-y-2 sm:grid-cols-4">
          <StatTile label="DURATION" value={summary.durationS.toFixed(1)} unit="S" />
          <StatTile label="RATE" value={summary.rateHz.toFixed(0)} unit="HZ" />
          <StatTile label="EKF RMSE" value={summary.ekfRmsM.toFixed(3)} unit="M" />
          <StatTile label="MAX SPD" value={summary.maxSpeedMps.toFixed(2)} unit="M/S" />
        </div>
      </header>
      <div className="grid gap-x-5 gap-y-5 lg:grid-cols-2">
        {series.map((s) => (
          <LineChart
            key={s.key}
            label={s.label}
            unit={s.unit}
            color={s.color}
            times={s.times}
            values={s.values}
            t0={0}
            t1={t1}
          />
        ))}
      </div>
      <p className="mt-4 font-mono text-[9px] tracking-wider text-ink-faint">
        CURSOR LOCKED TO THE REPLAY CLOCK. SCRUB FROM THE TRANSPORT BAR BELOW.
      </p>
    </div>
  )
}
