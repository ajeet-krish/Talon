/**
 * Truth-side sensor models (TypeScript port of talon/sensors.py).
 *
 * IMU accel is the specific force in body frame; in hover it reads
 * [0, 0, -g] under the NED convention. The TRUE biases random-walk each IMU
 * step, exercising the EKF bias-estimation path. GPS noise is applied in
 * meters in the local NED frame.
 *
 * The noise stream is the seeded Rng in math.ts (documented as statistically
 * equivalent to, but not bit-identical with, numpy's default_rng).
 */

import { GRAVITY, M_PER_DEG } from './constants'
import { Rng } from './math'
import type { Vec3 } from './math'

export interface IMUConfig {
  accel_noise_density: number
  gyro_noise_density: number
  accel_bias_initial: Vec3
  gyro_bias_initial: Vec3
  accel_bias_walk: number
  gyro_bias_walk: number
}

export interface GPSConfig {
  position_noise: number
  position_noise_vertical: number
  velocity_noise: number
  update_rate: number
}

export function defaultIMUConfig(): IMUConfig {
  return {
    accel_noise_density: 0.004,
    gyro_noise_density: 0.003,
    accel_bias_initial: [0.05, -0.03, 0.02],
    gyro_bias_initial: [0.001, -0.002, 0.0015],
    accel_bias_walk: 5e-4,
    gyro_bias_walk: 1e-5,
  }
}

export function defaultGPSConfig(): GPSConfig {
  return {
    position_noise: 1.5,
    position_noise_vertical: 3.0,
    velocity_noise: 0.1,
    update_rate: 10.0,
  }
}

/**
 * True body-frame specific force from finite-differenced true velocity:
 * f_body = R^T (a_ned - g_ned) with R the body-to-NED DCM, row-major (9).
 */
export function specificForceFromVelocity(
  vPrev: Vec3,
  vCurr: Vec3,
  dcmRowMajor: readonly number[],
  dt: number,
): Vec3 {
  if (dt <= 0.0) {
    throw new Error(`dt must be positive, got ${dt}`)
  }
  const aNed: Vec3 = [
    (vCurr[0] - vPrev[0]) / dt,
    (vCurr[1] - vPrev[1]) / dt,
    (vCurr[2] - vPrev[2]) / dt,
  ]
  const rel: Vec3 = [aNed[0], aNed[1], aNed[2] - GRAVITY]
  // R^T @ rel: component i is the dot of column i of R with rel.
  const r = dcmRowMajor
  return [
    (r[0] as number) * rel[0] + (r[3] as number) * rel[1] + (r[6] as number) * rel[2],
    (r[1] as number) * rel[0] + (r[4] as number) * rel[1] + (r[7] as number) * rel[2],
    (r[2] as number) * rel[0] + (r[5] as number) * rel[1] + (r[8] as number) * rel[2],
  ]
}

export interface GPSFix {
  timestamp: number
  lat: number
  lon: number
  alt: number
  vn: number
  ve: number
  vd: number
}

export interface IMUSample {
  timestamp: number
  accel: Vec3
  gyro: Vec3
}

/** GPS fix -> local NED position [n, e, d] (m), flat-Earth mapping. */
export function gpsToNed(gps: GPSFix): Vec3 {
  return [gps.lat * M_PER_DEG, gps.lon * M_PER_DEG, -gps.alt]
}

/** GPS fix -> NED velocity [vn, ve, vd] (m/s). */
export function gpsVelocityNed(gps: GPSFix): Vec3 {
  return [gps.vn, gps.ve, gps.vd]
}

/** Seeded IMU + GPS truth-side measurement generator. */
export class SensorSuite {
  private readonly _imuConfig: IMUConfig
  private readonly _gpsConfig: GPSConfig
  private readonly _rng: Rng
  private _accelBias: Vec3
  private _gyroBias: Vec3
  private _time = 0.0

  constructor(imuConfig?: IMUConfig, gpsConfig?: GPSConfig, seed = 0) {
    this._imuConfig = imuConfig ?? defaultIMUConfig()
    this._gpsConfig = gpsConfig ?? defaultGPSConfig()
    this._rng = new Rng(seed)
    this._accelBias = [...this._imuConfig.accel_bias_initial] as Vec3
    this._gyroBias = [...this._imuConfig.gyro_bias_initial] as Vec3
  }

  get accelBias(): Vec3 {
    return [...this._accelBias] as Vec3
  }

  get gyroBias(): Vec3 {
    return [...this._gyroBias] as Vec3
  }

  get time(): number {
    return this._time
  }

  /** One IMU sample from the true specific force (body) and true body rate. */
  generateImu(trueSpecificForceBody: Vec3, trueAngularRate: Vec3, dt: number): IMUSample {
    if (dt <= 0.0) {
      throw new Error(`dt must be positive, got ${dt}`)
    }
    const accelSigma = this._imuConfig.accel_noise_density / Math.sqrt(dt)
    const gyroSigma = this._imuConfig.gyro_noise_density / Math.sqrt(dt)

    const accelNoise = this._rng.standardNormalVec(3)
    const gyroNoise = this._rng.standardNormalVec(3)
    const accel: Vec3 = [
      trueSpecificForceBody[0] + this._accelBias[0] + accelSigma * (accelNoise[0] as number),
      trueSpecificForceBody[1] + this._accelBias[1] + accelSigma * (accelNoise[1] as number),
      trueSpecificForceBody[2] + this._accelBias[2] + accelSigma * (accelNoise[2] as number),
    ]
    const gyro: Vec3 = [
      trueAngularRate[0] + this._gyroBias[0] + gyroSigma * (gyroNoise[0] as number),
      trueAngularRate[1] + this._gyroBias[1] + gyroSigma * (gyroNoise[1] as number),
      trueAngularRate[2] + this._gyroBias[2] + gyroSigma * (gyroNoise[2] as number),
    ]

    // True bias random walk (units per sqrt(s) * sqrt(dt)).
    const walkDt = Math.sqrt(dt)
    const accelWalk = this._rng.standardNormalVec(3)
    const gyroWalk = this._rng.standardNormalVec(3)
    this._accelBias = [
      this._accelBias[0] + this._imuConfig.accel_bias_walk * walkDt * (accelWalk[0] as number),
      this._accelBias[1] + this._imuConfig.accel_bias_walk * walkDt * (accelWalk[1] as number),
      this._accelBias[2] + this._imuConfig.accel_bias_walk * walkDt * (accelWalk[2] as number),
    ]
    this._gyroBias = [
      this._gyroBias[0] + this._imuConfig.gyro_bias_walk * walkDt * (gyroWalk[0] as number),
      this._gyroBias[1] + this._imuConfig.gyro_bias_walk * walkDt * (gyroWalk[1] as number),
      this._gyroBias[2] + this._imuConfig.gyro_bias_walk * walkDt * (gyroWalk[2] as number),
    ]

    const measurement: IMUSample = { timestamp: this._time, accel, gyro }
    this._time += dt
    return measurement
  }

  /** One GPS fix from the true position/velocity (noise applied in meters). */
  generateGps(truePosition: Vec3, trueVelocity: Vec3): GPSFix {
    const posNoise = this._rng.standardNormalVec(3)
    const velNoise = this._rng.standardNormalVec(3)
    const n = truePosition[0] + this._gpsConfig.position_noise * (posNoise[0] as number)
    const e = truePosition[1] + this._gpsConfig.position_noise * (posNoise[1] as number)
    const d =
      truePosition[2] + this._gpsConfig.position_noise_vertical * (posNoise[2] as number)
    return {
      timestamp: this._time,
      lat: n / M_PER_DEG,
      lon: e / M_PER_DEG,
      alt: -d,
      vn: trueVelocity[0] + this._gpsConfig.velocity_noise * (velNoise[0] as number),
      ve: trueVelocity[1] + this._gpsConfig.velocity_noise * (velNoise[1] as number),
      vd: trueVelocity[2] + this._gpsConfig.velocity_noise * (velNoise[2] as number),
    }
  }
}
