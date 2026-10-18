/**
 * Motor mixing and first-order lag (TypeScript port of talon/motors.py).
 *
 * X-configuration mixing with l = arm_length / sqrt(2) and c = kM / kF. The
 * inverse mix is written in closed form (algebraically identical to the
 * numpy left-inverse) so no matrix library is needed here.
 */

import {
  ARM_LENGTH,
  DT,
  MOTOR_FORCE_MAX,
  MOTOR_FORCE_MIN,
  MOTOR_KF,
  MOTOR_KM,
  MOTOR_TIME_CONST,
} from './constants'
import type { Vec3 } from './math'

const MIX_L = ARM_LENGTH / Math.SQRT2
const MIX_C = MOTOR_KM / MOTOR_KF

/** (collective thrust, body moment) -> 4 motor forces (may be out of range). */
export function motorMixing(thrust: number, moment: Vec3): number[] {
  const t = thrust
  const a = moment[0] / MIX_L
  const b = moment[1] / MIX_L
  const c = moment[2] / MIX_C
  return [
    (t - a - b - c) / 4.0,
    (t - a + b + c) / 4.0,
    (t + a - b + c) / 4.0,
    (t + a + b - c) / 4.0,
  ]
}

/** Forward mix: 4 motor forces -> [collective thrust, mx, my, mz]. */
export function forwardMix(forces: readonly number[]): [number, Vec3] {
  const [f1, f2, f3, f4] = forces as [number, number, number, number]
  const thrust = f1 + f2 + f3 + f4
  const mx = -MIX_L * f1 - MIX_L * f2 + MIX_L * f3 + MIX_L * f4
  const my = -MIX_L * f1 + MIX_L * f2 - MIX_L * f3 + MIX_L * f4
  const mz = -MIX_C * f1 + MIX_C * f2 + MIX_C * f3 - MIX_C * f4
  return [thrust, [mx, my, mz]]
}

/** Four first-order lag motor responses with per-motor saturation. */
export class MotorModel {
  private readonly _timeConst: number
  private _forces: number[] = [0, 0, 0, 0]

  constructor(timeConst: number = MOTOR_TIME_CONST) {
    if (timeConst <= 0.0) {
      throw new Error(`time_const must be positive, got ${timeConst}`)
    }
    this._timeConst = timeConst
  }

  reset(): void {
    this._forces = [0, 0, 0, 0]
  }

  /** Advance one step; returns the actual (lagged) motor forces. */
  step(thrust: number, moment: Vec3, dt: number = DT): number[] {
    if (dt < 0.0) {
      throw new Error(`dt must be non-negative, got ${dt}`)
    }
    const commanded = motorMixing(thrust, moment)
    const alpha = 1.0 - Math.exp(-dt / this._timeConst)
    for (let i = 0; i < 4; i += 1) {
      const f = (this._forces[i] as number) + ((commanded[i] as number) - (this._forces[i] as number)) * alpha
      this._forces[i] = Math.min(MOTOR_FORCE_MAX, Math.max(MOTOR_FORCE_MIN, f))
    }
    return this._forces.slice()
  }

  get forces(): number[] {
    return this._forces.slice()
  }
}

export { MIX_L, MIX_C }
