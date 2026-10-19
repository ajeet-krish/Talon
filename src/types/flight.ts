/**
 * Talon flight-export schema 1.1.
 *
 * One JSON file per simulation scenario. Arrays are per-sample (N = meta.sample_count)
 * at a uniform rate meta.dt, unless noted. Positions/velocities are NED
 * (north, east, down) in meters / meters per second.
 *
 * 1.1 adds the top-level `obstacles` list (spherical keep-out zones). It is
 * backward compatible: a 1.0 document without the key is read as an empty
 * list, so the loader never sees an undefined field.
 *
 * The document types themselves live in `./generated/talon_core.ts`, the ts-rs
 * output over the Rust source of truth (D23). This module stays the app's
 * public import path for them (Phase 5c): consumers keep importing from
 * `@/types/flight` while the field sets follow the Rust structs.
 */

import type { FlightPhase } from './generated/talon_core'

export type {
  EventLogEntry,
  FlightData,
  FlightMeta,
  FlightPhase,
  Obstacle,
} from './generated/talon_core'

export const SCHEMA_VERSION = '1.1'

/** Scenario ids shipped with the app (keys into public/data/). */
export type ScenarioId = 'gps_outage' | 'wind' | 'emergency' | 'obstacle_field'

/** NED triple: [north, east, down]. */
export type Vec3 = [number, number, number]

/** phase_timeline entry: [time_s, phase]; transitions sorted by time. */
export type PhaseEvent = [number, FlightPhase]
