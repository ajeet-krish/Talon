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
 */

export const SCHEMA_VERSION = '1.1'

/** Flight-phase state machine labels, lowercase as exported. */
export type FlightPhase =
  | 'idle'
  | 'arm'
  | 'takeoff'
  | 'cruise'
  | 'waypoint'
  | 'rtl'
  | 'land'
  | 'emergency'

/** Scenario ids shipped with the app (keys into public/data/). */
export type ScenarioId = 'gps_outage' | 'wind' | 'emergency' | 'obstacle_field'

/** NED triple: [north, east, down]. */
export type Vec3 = [number, number, number]

/**
 * Spherical keep-out zone (schema 1.1). A position is inside the obstacle
 * when its 3-D Euclidean distance to `center_ned` is less than `radius_m`.
 */
export interface Obstacle {
  /** Stable identifier, e.g. "obs_north". */
  id: string
  /** Sphere centre, NED [m]. */
  center_ned: Vec3
  /** Keep-out radius [m]; always positive. */
  radius_m: number
}

export interface FlightMeta {
  scenario: string
  seed: number
  schema_version: string
  generated_utc: string
  sample_count: number
  duration_s: number
  dt: number
  units: Record<string, string>
}

/** phase_timeline entry: [time_s, phase]; transitions sorted by time. */
export type PhaseEvent = [number, FlightPhase]

/** event_log entry: discrete state-machine transitions with guard status. */
export interface EventLogEntry {
  time: number
  event: string
  from_phase: FlightPhase
  to_phase: FlightPhase
  guard_ok: boolean
}

export interface FlightData {
  meta: FlightMeta
  /** (N,) sample times in seconds. */
  time_s: number[]
  /** (N,3) simulated ground-truth position, NED [m]. */
  truth_position: Vec3[]
  /** (N,3) simulated ground-truth velocity, NED [m/s]. */
  truth_velocity: Vec3[]
  /** (N,3) EKF position estimate, NED [m]. */
  ekf_position: Vec3[]
  /** (N,3) EKF velocity estimate, NED [m/s]. */
  ekf_velocity: Vec3[]
  /** (N,3) EKF position variance diagonal [m^2]. */
  position_var: Vec3[]
  /** (N,3) gyro bias estimate [rad/s]. */
  gyro_bias: Vec3[]
  /** (N,3) accel bias estimate [m/s^2]. */
  accel_bias: Vec3[]
  /** (N,3) true wind velocity, NED [m/s]. */
  wind_ned: Vec3[]
  /** (N,) total commanded thrust [N]. */
  command_thrust: number[]
  /** (N,3) commanded body moment [N*m]. */
  command_moment: Vec3[]
  /** (M,) NLGL commanded-thrust error along the mission leg. */
  e_ct: number[]
  /** (M,) times corresponding to e_ct samples [s]. */
  e_ct_times: number[]
  /** (M,) mission-leg index corresponding to e_ct samples. */
  e_ct_leg: number[]
  /** (K,3) mission waypoints, NED [m]. */
  waypoints_ned: Vec3[]
  /** (3,) home position, NED [m]. */
  home_ned: Vec3
  /** Spherical keep-out zones the mission was planned around (schema 1.1). */
  obstacles: Obstacle[]
  /** [[t, phase], ...] phase onset times, sorted ascending. */
  phase_timeline: PhaseEvent[]
  /** Discrete autopilot events (arming, guards, landings). */
  event_log: EventLogEntry[]
}
