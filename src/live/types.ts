/**
 * Types for the in-browser live run (FlightLab Phase 4).
 *
 * Two shapes sit on top of the ts-rs generated mirrors (Phase 5c):
 *
 * - `ScenarioDraft` is the sparse authored scenario, exactly as the Rust core
 *   serializes it: every knob optional because serde fills the default.
 * - `LiveConfig` is that shape normalized into the run contract the browser
 *   needs, so the fields the SIL loop reads are required here even though the
 *   wire shape leaves them optional. One LiveConfig is exported per runnable
 *   scenario so the browser config never drifts from the Rust scenario
 *   definitions.
 */

import type { FlightData, Obstacle, Vec3 } from '@/types/flight'
import type {
  AvoidanceConfig as SparseAvoidanceConfig,
  RunSummary,
  ScenarioConfig,
  Waypoint,
} from '@/types/generated/talon_core'

/** One mission waypoint (talon.types.Waypoint). */
export type { Waypoint }

/**
 * The authored scenario shape as `talon_core::ScenarioConfig` serializes it,
 * most fields optional (serde defaults them). Phase 8 mission authoring writes
 * this shape.
 */
export type ScenarioDraft = ScenarioConfig

/**
 * Reactive avoidance tuning (talon-core `AvoidanceConfig`). Absent means the
 * law is off; anything absent inside the block takes the core's default, so a
 * scenario can turn it on with `{"enabled": true}` alone. The run contract
 * keeps the master switch itself required; the sparse authored block stays
 * reachable as `ScenarioConfig['avoidance']`.
 */
export type AvoidanceConfig = SparseAvoidanceConfig & { enabled: boolean }

/**
 * Everything the browser needs to run one SIL scenario. Field names are
 * snake_case to match the exported JSON (SILConfig) one-to-one.
 *
 * The generated `ScenarioConfig` is intersected with the required run fields:
 * the wire shape is sparse, the contract the app consumes is not.
 */
export type LiveConfig = ScenarioConfig & {
  id: string
  label: string
  blurb: string
  /** Sim seconds of main-loop time (excludes the convergence pre-roll). */
  duration_s: number
  takeoff_altitude: number
  start_position_ned: Vec3
  waypoints: Waypoint[]
  obstacles: Obstacle[]
  avoidance?: AvoidanceConfig
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

/**
 * Headline numbers for the Run view: the Rust core's own run summary
 * (talon-core `RunSummary`), parsed from `summary_json()` as-is. The page
 * displays it; it does not recompute any of it from the document arrays.
 */
export type LiveRunSummary = RunSummary

/** One completed live run: the flight document plus its summary. */
export interface LiveRunResult {
  data: FlightData
  summary: LiveRunSummary
}
