/**
 * Mission templates (FlightLab Phase 8c). The ordered starting points the
 * editor offers: one blank mission, five shaped after the shipped live
 * configs, and the teaching preset.
 *
 * Pure and offline: the template data mirrors `public/data/live/configs.json`
 * (a committed fixture the validator tests already pin) rather than fetching
 * it, so this module stays importable from tests and pure editor code. Each
 * `build` returns a fresh `MissionConfig` with a non-reserved id, so the
 * result passes `validateScenarioConfig` and can be saved or run directly.
 */

import type { Obstacle, Waypoint } from '@/types/generated/talon_core'
import { emptyMission, type MissionConfig } from '@/utils/editor'

/** One entry in the template picker. */
export interface MissionTemplate {
  /** Stable picker id (not the mission id the built config carries). */
  id: string
  label: string
  description: string
  /** Build a fresh mission; never shares state with a previous call. */
  build: () => MissionConfig
}

/** A leg waypoint at 5 m/s, 2 m acceptance radius, level heading. */
function leg(north: number, east: number, altitude: number): Waypoint {
  return {
    position: [north, east, -altitude],
    speed: 5.0,
    heading: 0.0,
    radius: 2.0,
  }
}

function zone(
  id: string,
  north: number,
  east: number,
  altitude: number,
  radiusM: number,
): Obstacle {
  return { id, center_ned: [north, east, -altitude], radius_m: radiusM }
}

/** The shipped "waypoints" circuit: a 40 m x 30 m rectangle at 10 m. */
function circuitRoute(altitude: number): Waypoint[] {
  return [
    leg(0, 0, altitude),
    leg(40, 0, altitude),
    leg(40, 30, altitude),
    leg(0, 30, altitude),
    leg(0, 0, altitude),
  ]
}

/** The shipped obstacle/avoidance route: the rectangle with three bulges. */
function fieldRoute(altitude: number): Waypoint[] {
  return [
    leg(0, 0, altitude),
    leg(20, -6.5, altitude),
    leg(40, 0, altitude),
    leg(46, 15, altitude),
    leg(40, 30, altitude),
    leg(0, 30, altitude),
    leg(-7, 22, altitude),
    leg(0, 0, altitude),
  ]
}

/** The three keep-out spheres the shipped obstacle scenarios share. */
function fieldZones(altitude: number): Obstacle[] {
  return [
    zone('obs_north', 20, 3, altitude, 4.0),
    zone('obs_east', 37, 15, altitude, 3.5),
    zone('obs_west', 3, 22, altitude, 4.5),
  ]
}

/**
 * A small teaching circuit: five waypoints at 10 m, a steady 3 m/s north
 * wind with light turbulence, and one GPS outage at 20 s for 4 s. Sized for
 * a lesson: every knob that matters is non-default, and the whole flight
 * fits inside the 60 s window with margin.
 */
export function teachingPreset(): MissionConfig {
  const altitude = 10
  return {
    ...emptyMission('teaching_preset'),
    label: 'Teaching preset',
    blurb: 'Five waypoints at 10 m, 3 m/s north wind, GPS denied from 20 s to 24 s.',
    duration_s: 60,
    takeoff_altitude: altitude,
    waypoints: [
      leg(15, 0, altitude),
      leg(15, 15, altitude),
      leg(0, 15, altitude),
      leg(-15, 15, altitude),
      leg(-15, 0, altitude),
    ],
    wind_steady_ned: [3.0, 0.0, 0.0],
    wind_sigma: [0.5, 0.5, 0.25],
    faults: [{ kind: 'gps_outage', at_s: 20, duration_s: 4 }],
  }
}

/**
 * The ordered template list the picker renders. The middle five mirror the
 * shipped live configs (hover, waypoints, obstacle_field, avoidance, wind)
 * under fresh, non-reserved ids so an edited template is saveable.
 */
export const MISSION_TEMPLATES: MissionTemplate[] = [
  {
    id: 'empty',
    label: 'Empty mission',
    description: 'Climb and hold on a blank slate: no route, no zones, calm air.',
    build: () => ({ ...emptyMission('empty_mission'), label: 'Empty mission' }),
  },
  {
    id: 'hover',
    label: 'Hover',
    description: 'Climb to 10 m and hold station for 60 s in calm air.',
    build: () => ({
      ...emptyMission('hover_mission'),
      label: 'Hover',
      blurb: 'Climb to 10 m and hold station for 60 s in calm air.',
    }),
  },
  {
    id: 'waypoint_circuit',
    label: 'Waypoint circuit',
    description: '40 m x 30 m rectangle at 10 m, 5 m/s legs, light turbulence.',
    build: () => ({
      ...emptyMission('waypoint_circuit'),
      label: 'Waypoint circuit',
      blurb: '40 m x 30 m rectangle at 10 m, 5 m/s legs, light turbulence.',
      duration_s: 120,
      waypoints: circuitRoute(10),
      wind_sigma: [2.0, 2.0, 1.0],
    }),
  },
  {
    id: 'obstacle_field',
    label: 'Obstacle field',
    description: 'The circuit flown around three keep-out spheres via planned detours.',
    build: () => ({
      ...emptyMission('obstacle_mission'),
      label: 'Obstacle field',
      blurb: 'The waypoint circuit flown around three keep-out spheres via the planned detours.',
      duration_s: 70,
      waypoints: fieldRoute(10),
      obstacles: fieldZones(10),
    }),
  },
  {
    id: 'avoidance_demo',
    label: 'Avoidance demo',
    description: 'The plain legs straight through the zones: the autopilot bypasses and rejoins.',
    build: () => ({
      ...emptyMission('avoidance_demo'),
      label: 'Avoidance demo',
      blurb: 'The plain circuit straight through the same three keep-out spheres: the autopilot sees each zone, bypasses it, and rejoins the leg.',
      duration_s: 70,
      waypoints: circuitRoute(10),
      obstacles: fieldZones(10),
      avoidance: {
        enabled: true,
        clearance_margin_m: 2.0,
        lookahead_s: 3.0,
        detour_buffer_m: 0.5,
      },
    }),
  },
  {
    id: 'wind_hold',
    label: 'Hold in wind',
    description: 'Hover at 10 m against 2 m/s north wind with strong turbulence.',
    build: () => ({
      ...emptyMission('wind_hold'),
      label: 'Hold in wind',
      blurb: 'Hover at 10 m against 2 m/s north wind with strong Dryden turbulence.',
      wind_sigma: [3.0, 3.0, 1.5],
      wind_steady_ned: [2.0, 0.0, 0.0],
    }),
  },
  {
    id: 'teaching',
    label: 'Teaching preset',
    description: 'Five waypoints at 10 m, 3 m/s wind, one GPS outage at 20 s.',
    build: teachingPreset,
  },
]
