/**
 * Types for the in-browser live run (FlightLab Phase 4).
 *
 * Mirrors the Python dataclasses the SIL loop consumes: talon.types.Waypoint
 * and talon.mavlink_sil.SILConfig (the browser-facing subset). One
 * LiveConfig is exported per runnable scenario so the browser config never
 * drifts from the Python scenario definitions.
 */

import type { FlightData, FlightPhase, Obstacle } from '@/types/flight'
import type { Vec3 } from './math'

/** One mission waypoint (talon.types.Waypoint). */
export interface Waypoint {
  /** NED position [m]. */
  position: Vec3
  /** Leg cruise speed [m/s]. */
  speed: number
  /** Desired heading [rad]; the NLGL law derives yaw from the track. */
  heading: number
  /** Horizontal acceptance radius [m]. */
  radius: number
}

/**
 * Everything the browser needs to run one SIL scenario. Field names are
 * snake_case to match the exported JSON (SILConfig) one-to-one.
 */
export interface LiveConfig {
  id: string
  label: string
  blurb: string
  /** Sim seconds of main-loop time (excludes the convergence pre-roll). */
  duration_s: number
  takeoff_altitude: number
  start_position_ned: Vec3
  waypoints: Waypoint[]
  obstacles: Obstacle[]
  wind_sigma: [number, number, number]
  wind_steady_ned: Vec3
  seed: number
  gps_position_noise: number
  gps_position_noise_vertical: number
  gps_velocity_noise: number
  cruise_hold_s: number
  /** Queue arm + takeoff at t = 0 (every FlightLab scenario does). */
  auto_arm: boolean
}

/** Headline numbers for the Run view (derived from the generated run). */
export interface LiveRunSummary {
  duration_s: number
  waypoint_count: number
  waypoints_reached: number
  mission_complete: boolean
  gps_accepted: number
  gps_gated: number
  final_position_error_m: number
  ekf_position_rmse_m: number
  min_clearance_m: number | null
  phases: FlightPhase[]
}

/** One completed live run: the flight document plus its summary. */
export interface LiveRunResult {
  data: FlightData
  summary: LiveRunSummary
}
