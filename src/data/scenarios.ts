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
