import { useEffect, useMemo } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { SCENARIOS } from '@/data/scenarios'
import { StatTile } from '@/components/ui/StatTile'
import { StatusPanel } from '@/components/ui/StatusPanel'
import { summarizeRun } from '@/utils/telemetry'
import { formatMetric } from '@/utils/format'

/** Verdict colors, kept outside the reserved Wong flight-phase palette. */
const PASS_COLOR = '#3fb6a8'
const FAIL_COLOR = '#e2506a'

/**
 * Metrics screen: two panels, one per question the tool answers.
 *
 * RUN SUMMARY is derived on the client from the active scenario's recorded
 * samples, so it always matches the charts. VALIDATION GATES is the driver's
 * own acceptance table (sims/run_all.py -> plots/json/validation.json), so
 * the web report and the printed report cannot drift.
 */
export function MetricsView() {
  const report = useFlightStore((s) => s.validation)
  const validationError = useFlightStore((s) => s.validationError)
  const loadValidation = useFlightStore((s) => s.loadValidation)
  const activeId = useFlightStore((s) => s.activeScenario)
  const data = useFlightStore((s) => s.scenarios.get(s.activeScenario))

  useEffect(() => {
    void loadValidation()
  }, [loadValidation])

  const summary = useMemo(() => (data ? summarizeRun(data) : null), [data])

  return (
    <div className="min-h-0 flex-1 overflow-y-auto px-4 py-4">
      <section>
        <SectionHeader
          title="RUN SUMMARY"
          note={SCENARIOS[activeId].label}
          hint="derived from the recorded samples"
        />
        {summary ? (
          <div className="grid grid-cols-2 gap-x-6 gap-y-3 sm:grid-cols-3 lg:grid-cols-4">
            <StatTile label="DURATION" value={summary.durationS.toFixed(1)} unit="S" />
            <StatTile label="SAMPLES" value={String(summary.sampleCount)} />
            <StatTile label="RATE" value={summary.rateHz.toFixed(0)} unit="HZ" />
            <StatTile label="DISTANCE" value={formatMetric(summary.distanceFlownM)} unit="M" />
            <StatTile label="EKF RMSE" value={summary.ekfRmsM.toFixed(3)} unit="M" />
            <StatTile label="EKF MAX" value={summary.ekfMaxM.toFixed(3)} unit="M" />
            <StatTile label="MAX ALT" value={formatMetric(summary.maxAltitudeM)} unit="M" />
            <StatTile label="MAX SPD" value={summary.maxSpeedMps.toFixed(2)} unit="M/S" />
            <StatTile label="3-SIGMA MAX" value={formatMetric(summary.maxSigmaM)} unit="M" />
            <StatTile label="MAX WIND" value={summary.maxWindMps.toFixed(2)} unit="M/S" />
            <StatTile label="PHASES" value={String(summary.phaseCount)} />
            {summary.minClearanceM !== null && (
              <StatTile
                label="MIN CLEARANCE"
                value={summary.minClearanceM.toFixed(2)}
                unit="M"
              />
            )}
          </div>
        ) : (
          <StatusPanel title="NO SCENARIO LOADED" />
        )}
      </section>

      <section className="mt-8">
        <SectionHeader
          title="VALIDATION GATES"
          note={report ? `${report.passed}/${report.total} PASS` : undefined}
          hint={report ? `run_all.py ${report.generated_utc}` : undefined}
        />
        {report ? (
          <div className="overflow-x-auto">
            <table className="w-full border-collapse font-mono text-[10px]">
              <thead>
                <tr className="border-b border-grid text-left text-ink-faint">
                  <th className="py-1.5 pr-3 font-normal tracking-widest">TEST</th>
                  <th className="py-1.5 pr-3 font-normal tracking-widest">TARGET</th>
                  <th className="py-1.5 pr-3 font-normal tracking-widest">ACHIEVED</th>
                  <th className="py-1.5 font-normal tracking-widest">VERDICT</th>
                </tr>
              </thead>
              <tbody>
                {report.rows.map((row, i) => (
                  <tr key={`${row.test}-${i}`} className="border-b border-grid/60">
                    <td className="py-1.5 pr-3 text-ink-dim">{row.test}</td>
                    <td className="py-1.5 pr-3 tabular-nums text-ink-faint">
                      {row.target}
                    </td>
                    <td className="py-1.5 pr-3 tabular-nums text-ink">{row.achieved}</td>
                    <td
                      className="py-1.5 font-semibold"
                      style={{ color: row.passed ? PASS_COLOR : FAIL_COLOR }}
                    >
                      {row.passed ? 'PASS' : 'FAIL'}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        ) : validationError ? (
          <StatusPanel
            title="VALIDATION REPORT UNAVAILABLE"
            detail={`${validationError}. run scripts/copy_data.sh`}
            tone="error"
          />
        ) : (
          <StatusPanel title="LOADING VALIDATION\u2026" />
        )}
      </section>
    </div>
  )
}

function SectionHeader({
  title,
  note,
  hint,
}: {
  title: string
  note?: string
  hint?: string
}) {
  return (
    <div className="mb-3 flex flex-wrap items-baseline justify-between gap-x-4 gap-y-1 border-b border-grid pb-1.5">
      <div className="flex items-baseline gap-3">
        <span className="font-mono text-[10px] tracking-[0.2em] text-ink-dim">
          {title}
        </span>
        {note && (
          <span className="font-mono text-[10px] tracking-widest text-signal">
            {note}
          </span>
        )}
      </div>
      {hint && (
        <span className="font-mono text-[9px] tracking-wider text-ink-faint">
          {hint}
        </span>
      )}
    </div>
  )
}
