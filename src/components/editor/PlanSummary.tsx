/**
 * Plan summary panel (FlightLab Phase 8c): the whole mission as a handful of
 * numbers a planner checks first. Route distance and flight time come from
 * the authored data only (`routeDistanceM`, `estimateFlightTimeS`), and the
 * time is labelled an estimate wherever it appears, because nothing here has
 * run the simulator.
 */

import type { MissionConfig } from '@/utils/editor'
import { estimateFlightTimeS, routeDistanceM } from '@/utils/editor'
import { formatMinutesSeconds, formatWindSummary } from './chart'
import { Section } from './Section'

export interface PlanSummaryProps {
  /** The mission being edited. */
  config: MissionConfig
  /** Non-blocking warning count from the store. */
  warnings: number
}

/** One labelled readout: mono label, tabular value, faint unit. */
function Stat({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div className="flex items-baseline justify-between gap-2 border-b border-grid py-1">
      <span className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint">
        {label}
      </span>
      <span className="font-mono text-[11px] tabular-nums text-ink">
        {value}
        {unit !== undefined && <span className="ml-1 text-[9px] text-ink-faint">{unit}</span>}
      </span>
    </div>
  )
}

export function PlanSummary({ config, warnings }: PlanSummaryProps) {
  const distance = routeDistanceM(config)
  const time = estimateFlightTimeS(config)
  const [windNorth, windEast] = [config.wind_steady_ned[0], config.wind_steady_ned[1]]

  return (
    <Section title="Plan summary" aside={warnings > 0 ? `${warnings} WARN` : undefined}>
      <Stat label="Waypoints" value={String(config.waypoints.length)} />
      <Stat label="Route distance" value={distance.toFixed(1)} unit="m" />
      <Stat label="Flight time (estimate)" value={formatMinutesSeconds(time)} />
      <Stat label="Wind" value={formatWindSummary(windNorth, windEast)} />
      <Stat label="Keep-out zones" value={String(config.obstacles.length)} />
      <Stat label="Faults" value={String(config.faults.length)} />
      <Stat label="Avoidance" value={config.avoidance.enabled ? 'ON' : 'OFF'} />
    </Section>
  )
}
