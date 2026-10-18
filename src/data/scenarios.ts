import type { ScenarioId } from '@/types/flight'

export interface ScenarioMeta {
  label: string
  subtitle: string
}

/**
 * Scenario facts pulled from the data itself: durations are exact, and the
 * subtitles name the engineering story each run demonstrates. Shared by the
 * scenario rail, the telemetry header, and the metrics summary.
 */
export const SCENARIOS: Record<ScenarioId, ScenarioMeta> = {
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

/** Meta for the in-browser live run, which has no recorded scenario entry. */
export const LIVE_RUN_META: ScenarioMeta = {
  label: 'LIVE RUN',
  subtitle: 'in-browser WASM run',
}

/**
 * Meta for any scenario key, including the live run slot. Recorded scenario
 * ids resolve to their SCENARIOS entry; anything else (the live run) falls
 * back to LIVE_RUN_META.
 */
export function scenarioMeta(id: string): ScenarioMeta {
  return (SCENARIOS as Record<string, ScenarioMeta | undefined>)[id] ?? LIVE_RUN_META
}
