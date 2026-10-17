import type { ScenarioId } from '@/types/flight'

export interface Recording {
  id: string
  label: string
  /** File name inside public/videos/ (staged from docs/videos). */
  file: string
  /** Scenario the recording illustrates, when one exists in the app. */
  scenario: ScenarioId | null
  subtitle: string
}

/**
 * Recorded flight videos rendered by the Matplotlib waypoint exporter
 * (docs/videos). They are the professional publications half of the tool;
 * the replay scene is the interactive half. Recordings with a matching
 * scenario can jump straight into the interactive replay.
 *
 * Files are committed under public/videos/, staged from docs/videos/ by
 * scripts/copy_data.sh after a validation driver run.
 */
export const RECORDINGS: Recording[] = [
  {
    id: 'waypoints',
    label: 'WAYPOINT CIRCUIT',
    file: 'waypoints.mp4',
    scenario: null,
    subtitle: 'five-waypoint 40 x 30 m circuit, NLGL capture',
  },
  {
    id: 'gps_outage',
    label: 'GPS OUTAGE',
    file: 'gps_outage.mp4',
    scenario: 'gps_outage',
    subtitle: 'GPS denied mid-mission, EKF dead reckoning',
  },
  {
    id: 'wind',
    label: 'WIND',
    file: 'wind.mp4',
    scenario: 'wind',
    subtitle: '2 m/s Dryden gust, hold with wind estimation',
  },
  {
    id: 'emergency',
    label: 'EMERGENCY',
    file: 'emergency.mp4',
    scenario: 'emergency',
    subtitle: 'critical fault, EKF divergence, emergency land',
  },
  {
    id: 'obstacle_field',
    label: 'OBSTACLE FIELD',
    file: 'obstacle_field.mp4',
    scenario: 'obstacle_field',
    subtitle: 'detour plan around spherical keep-out zones',
  },
]
