/**
 * NLGL guidance law (TypeScript port of talon/guidance.py).
 *
 * Park, Deyst & How 2004/2007 in the ArduPilot AP_L1_Control scalar form,
 * specialized to quadrotor velocity commands. Signs follow the cross2
 * convention and the NED frames of docs/conventions.md. Guidance is a pure
 * function of its inputs plus config; the mission manager owns index
 * advancement.
 *
 * v_des = cruise_speed * [cos(chi_d), sin(chi_d), 0] + altitude channel,
 * with the documented choices carried over verbatim: ground velocity in Nu2,
 * Nu clipped to nu_clip, Nu2 zeroed below the ground-speed floor, behind-A
 * recovery, and the NED altitude channel (climb => v_des_z < 0).
 */

import { cross2, wrapAngle } from './math'
import type { Vec3 } from './math'
import type { WaypointMission } from './mission'

/** Legs shorter than this (m, horizontal) are degenerate. */
const LEG_LENGTH_EPS = 1.0e-6
/** Behind-A recovery trigger (ArduPilot uses the same 0.7071). */
const BEHIND_S_THRESHOLD = -0.7071

export interface NLGLConfig {
  zeta: number
  period_t: number
  l1_min: number
  nu1_clip_deg: number
  nu_clip_deg: number
  cruise_speed: number
  kp_alt: number
  kd_alt: number
  climb_rate_max: number
  min_ground_speed_for_nu2: number
}

/** Default tuning (talon.guidance.NLGLConfig). */
export function defaultNLGLConfig(): NLGLConfig {
  return {
    zeta: 0.75,
    period_t: 5.0,
    l1_min: 3.0,
    nu1_clip_deg: 45.0,
    nu_clip_deg: 90.0,
    cruise_speed: 5.0,
    kp_alt: 1.0,
    kd_alt: 0.5,
    climb_rate_max: 2.0,
    min_ground_speed_for_nu2: 0.3,
  }
}

export interface GuidanceDiagnostics {
  /** Signed cross-track error (m, + = left of path). */
  e_ct: number
  nu1: number
  nu2: number
  nu: number
  l1: number
  chi_d: number
  behind_a_recovery: boolean
  a_lat: number
}

export interface GuidanceResult {
  v_des: Vec3
  diagnostics: GuidanceDiagnostics
}

export class NLGLGuidance {
  private readonly _config: NLGLConfig

  constructor(config?: NLGLConfig) {
    this._config = config ?? defaultNLGLConfig()
  }

  get config(): NLGLConfig {
    return this._config
  }

  /** NLGL velocity command for the leg wp_a -> wp_b. */
  computeLeg(position: Vec3, velocity: Vec3, wpA: Vec3, wpB: Vec3): GuidanceResult {
    const cfg = this._config
    const nu1Clip = (cfg.nu1_clip_deg * Math.PI) / 180
    const nuClip = (cfg.nu_clip_deg * Math.PI) / 180

    const abN = wpB[0] - wpA[0]
    const abE = wpB[1] - wpA[1]
    const legLength = Math.sqrt(abN * abN + abE * abE)
    const rN = position[0] - wpA[0]
    const rE = position[1] - wpA[1]
    const vgN = velocity[0]
    const vgE = velocity[1]
    const groundSpeed = Math.sqrt(vgN * vgN + vgE * vgE)

    const l1 = Math.max(cfg.l1_min, (cfg.zeta * cfg.period_t * groundSpeed) / Math.PI)

    let eCt: number
    let nu1: number
    let nu2: number
    let nu: number
    let chiD: number
    let behindARecovery: boolean

    if (legLength < LEG_LENGTH_EPS) {
      // Degenerate leg: steer straight at wp_b.
      eCt = 0.0
      nu1 = 0.0
      nu2 = 0.0
      nu = 0.0
      const toBN = wpB[0] - position[0]
      const toBE = wpB[1] - position[1]
      chiD = Math.sqrt(toBN * toBN + toBE * toBE) < 1e-12 ? 0.0 : Math.atan2(toBE, toBN)
      behindARecovery = false
    } else {
      const uN = abN / legLength
      const uE = abE / legLength
      eCt = rN * uE - rE * uN
      const s = (rN * uN + rE * uE) / legLength

      // Nu1: cross-track angle, clipped so a large offset cannot demand
      // more than nu1_clip of capture angle.
      const sinClip = Math.sin(nu1Clip)
      nu1 = Math.asin(Math.min(sinClip, Math.max(-sinClip, eCt / l1)))

      // Nu2 from the EKF GROUND velocity (NE plane); zeroed below the
      // ground-speed floor (course undefined at rest).
      if (groundSpeed >= cfg.min_ground_speed_for_nu2) {
        nu2 = Math.atan2(cross2([vgN, vgE], [uN, uE]), vgN * uN + vgE * uE)
      } else {
        nu2 = 0.0
      }
      nu = Math.min(nuClip, Math.max(-nuClip, nu1 + nu2))

      const chiQ = Math.atan2(uE, uN)

      // Behind-A recovery: well behind the leg start and far from A.
      behindARecovery = s < BEHIND_S_THRESHOLD && Math.sqrt(rN * rN + rE * rE) > l1
      if (behindARecovery) {
        const toAN = wpA[0] - position[0]
        const toAE = wpA[1] - position[1]
        chiD =
          Math.sqrt(toAN * toAN + toAE * toAE) < 1e-12
            ? chiQ
            : Math.atan2(toAE, toAN)
      } else {
        // chi_d = chi_q + Nu with Nu CLIPPED (ArduPilot form).
        chiD = wrapAngle(chiQ + nu)
      }
    }

    const vDes: Vec3 = [
      cfg.cruise_speed * Math.cos(chiD),
      cfg.cruise_speed * Math.sin(chiD),
      0.0,
    ]

    // Altitude channel: P/PD on the leg end waypoint's NED z.
    const vDesZ = Math.min(
      cfg.climb_rate_max,
      Math.max(
        -cfg.climb_rate_max,
        cfg.kp_alt * (wpB[2] - position[2]) + cfg.kd_alt * (0.0 - velocity[2]),
      ),
    )
    vDes[2] = vDesZ

    const kL1 = 4.0 * cfg.zeta * cfg.zeta
    const aLat = ((kL1 * groundSpeed * groundSpeed) / l1) * Math.sin(nu)

    return {
      v_des: vDes,
      diagnostics: {
        e_ct: eCt,
        nu1,
        nu2,
        nu,
        l1,
        chi_d: chiD,
        behind_a_recovery: behindARecovery,
        a_lat: aLat,
      },
    }
  }

  /** NLGL velocity command toward the mission's active waypoint. */
  computeMission(
    position: Vec3,
    velocity: Vec3,
    mission: WaypointMission,
  ): GuidanceResult {
    const active = mission.activeWaypoint()
    const legA = mission.previousPoint(position)
    return this.computeLeg(position, velocity, legA, active.position)
  }
}
