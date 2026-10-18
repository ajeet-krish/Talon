/**
 * Flight-phase state machine (TypeScript port of talon/state_machine.py).
 *
 * An 8-state guarded transition machine over the lowercase FlightPhase
 * labels of the flight schema. Every state change goes through
 * FlightPhaseMachine.request, which evaluates one event against the
 * transition table plus its guard on the current Telemetry snapshot.
 * Accepted and rejected attempts are both logged, sim-time stamped, so a
 * post-flight review can reconstruct when and why the phase changed.
 */

import type { FlightPhase } from '@/types/flight'
import type { Vec3 } from './math'

export interface Telemetry {
  /** EKF position estimate, NED [m]. */
  position_ned: Vec3
  /** EKF velocity estimate, NED [m/s]. */
  velocity_ned: Vec3
  battery_soc: number
  ekf_converged: boolean
  ekf_diverged: boolean
  gps_outage_duration: number
  mission_waypoint_count: number
  mission_complete: boolean
  motors_armed: boolean
  home_position_ned: Vec3
  emergency_descent_active: boolean
  motor_fault: boolean
}

/** Altitude above home (m, positive up; NED-safe). */
export function altitudeAboveHome(t: Telemetry): number {
  return t.home_position_ned[2] - t.position_ned[2]
}

/** Horizontal (North-East plane) distance to home (m). */
export function horizontalDistToHome(t: Telemetry): number {
  const dn = t.position_ned[0] - t.home_position_ned[0]
  const de = t.position_ned[1] - t.home_position_ned[1]
  return Math.sqrt(dn * dn + de * de)
}

/** NED down velocity (m/s, positive descending). */
export function velZ(t: Telemetry): number {
  return t.velocity_ned[2]
}

export interface PhaseTransition {
  time: number
  from_phase: FlightPhase
  to_phase: FlightPhase
  event: string
  guard_ok: boolean
}

type GuardFn = (t: Telemetry, takeoffAlt: number) => boolean

const G_ARM: GuardFn = (t) => t.ekf_converged && t.battery_soc > 0.3
const G_TAKEOFF: GuardFn = (t) => t.motors_armed
const G_ALTITUDE: GuardFn = (t, alt) => altitudeAboveHome(t) > alt - 0.5
const G_MISSION_START: GuardFn = (t) => t.mission_waypoint_count > 0
const G_MISSION_COMPLETE: GuardFn = (t) => t.mission_complete
const G_LOW_BATTERY: GuardFn = (t) => t.battery_soc < 0.2
const G_GPS_LOST: GuardFn = (t) => t.gps_outage_duration > 5.0
const G_HOME_REACHED: GuardFn = (t) => horizontalDistToHome(t) < 1.0
const G_LANDED: GuardFn = (t) => altitudeAboveHome(t) < 0.1 && Math.abs(velZ(t)) < 0.1
const G_EMERGENCY_LANDING: GuardFn = (t) => t.emergency_descent_active
const G_OPERATOR: GuardFn = () => true

const table = new Map<string, [FlightPhase, GuardFn]>()
function add(from: FlightPhase, event: string, to: FlightPhase, guard: GuardFn): void {
  table.set(`${from}|${event}`, [to, guard])
}
add('idle', 'arm', 'arm', G_ARM)
add('arm', 'takeoff', 'takeoff', G_TAKEOFF)
add('takeoff', 'altitude_reached', 'cruise', G_ALTITUDE)
add('cruise', 'mission_start', 'waypoint', G_MISSION_START)
add('waypoint', 'mission_complete', 'rtl', G_MISSION_COMPLETE)
add('waypoint', 'low_battery', 'rtl', G_LOW_BATTERY)
add('waypoint', 'gps_lost', 'emergency', G_GPS_LOST)
add('rtl', 'home_reached', 'land', G_HOME_REACHED)
add('land', 'landed', 'idle', G_LANDED)
add('emergency', 'emergency_landing', 'land', G_EMERGENCY_LANDING)
// Operator-RTL extension.
add('takeoff', 'rtl', 'rtl', G_OPERATOR)
add('cruise', 'rtl', 'rtl', G_OPERATOR)
add('waypoint', 'rtl', 'rtl', G_OPERATOR)

/** Guarded 8-state flight phase machine with a transition event log. */
export class FlightPhaseMachine {
  private readonly _takeoffAltitude: number
  private _phase: FlightPhase = 'idle'
  private readonly _eventLog: PhaseTransition[] = []

  constructor(takeoffAltitude = 10.0) {
    if (takeoffAltitude <= 0.0) {
      throw new Error(`takeoff_altitude must be positive, got ${takeoffAltitude}`)
    }
    this._takeoffAltitude = takeoffAltitude
  }

  get phase(): FlightPhase {
    return this._phase
  }

  get eventLog(): PhaseTransition[] {
    return this._eventLog.slice()
  }

  get takeoffAltitude(): number {
    return this._takeoffAltitude
  }

  request(event: string, telemetry: Telemetry, timeS: number): boolean {
    if (event === 'critical_fault') {
      if (telemetry.ekf_diverged || telemetry.motor_fault) {
        if (this._phase === 'emergency') {
          this._log(timeS, this._phase, this._phase, 'critical_fault:already_emergency', true)
          return true
        }
        this._log(timeS, this._phase, 'emergency', event, true)
        this._phase = 'emergency'
        return true
      }
      this._log(timeS, this._phase, this._phase, event, false)
      return false
    }

    const entry = table.get(`${this._phase}|${event}`)
    if (entry === undefined) {
      this._log(timeS, this._phase, this._phase, event, false)
      return false
    }
    const [target, guard] = entry
    if (!guard(telemetry, this._takeoffAltitude)) {
      this._log(timeS, this._phase, this._phase, event, false)
      return false
    }
    this._log(timeS, this._phase, target, event, true)
    this._phase = target
    return true
  }

  /** Test hook: set the phase directly, logging the jump. */
  forcePhase(phase: FlightPhase, timeS: number, reason: string): void {
    if (phase === this._phase) return
    this._log(timeS, this._phase, phase, `force:${reason}`, true)
    this._phase = phase
  }

  private _log(
    timeS: number,
    from: FlightPhase,
    to: FlightPhase,
    event: string,
    guardOk: boolean,
  ): void {
    this._eventLog.push({
      time: timeS,
      from_phase: from,
      to_phase: to,
      event,
      guard_ok: guardOk,
    })
  }
}
