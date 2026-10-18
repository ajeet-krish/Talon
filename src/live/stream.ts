/**
 * Live sample codec and recorder.
 *
 * The native core streams its recorded history as flat `f64` blocks, one block
 * per control-rate sample (`talon_core::sim::SAMPLE_STRIDE` doubles). This module
 * decodes those blocks into typed series a GCS can draw, and owns the growing
 * buffers so the render loop never allocates per frame.
 *
 * Two things are deliberate:
 *
 * * The field offsets are mirrored from the Rust side by hand and checked
 *   against the running build (`sample_stride()` comes back over the boundary),
 *   because the wasm module and this file have to agree on a memory layout and
 *   nothing in the type system enforces it.
 * * The recorder keeps scene-space trail positions and a handful of derived
 *   scalar series as it appends, rather than recomputing them per frame. At
 *   100 Hz over a 120 s run that is 12000 samples and six series; doing it once
 *   on arrival is what keeps the frame budget free for rendering.
 */

import type { Vec3, PhaseEvent } from '@/types/flight'
import { nedToScene } from '@/utils/interpolation'

/** Doubles per streamed sample; mirrors `talon_core::sim::SAMPLE_STRIDE`. */
export const SAMPLE_STRIDE = 36

/** Offset of each field within one sample block, in doubles. */
export const SAMPLE_FIELD = {
  TIME: 0,
  TRUTH_POSITION: 1,
  TRUTH_VELOCITY: 4,
  TRUTH_QUATERNION: 7,
  TRUTH_ANGULAR_RATE: 11,
  EKF_POSITION: 14,
  EKF_VELOCITY: 17,
  POSITION_VARIANCE: 20,
  GYRO_BIAS: 23,
  ACCEL_BIAS: 26,
  WIND: 29,
  THRUST: 32,
  MOMENT: 33,
} as const

/** One decoded sample. */
export interface LiveFrame {
  /** Sim time (s). */
  time: number
  /** Truth position, NED (m). */
  truthPosition: Vec3
  /** Truth velocity, NED (m/s). */
  truthVelocity: Vec3
  /** Truth attitude, Hamilton `[w, x, y, z]`, body to NED. */
  quaternion: [number, number, number, number]
  /** Truth body angular rate (rad/s). */
  angularRate: Vec3
  /** Estimated position, NED (m). */
  ekfPosition: Vec3
  /** Estimated velocity, NED (m/s). */
  ekfVelocity: Vec3
  /** Estimated position variance diagonal (m^2). */
  positionVariance: Vec3
  /** Estimated gyro bias (rad/s). */
  gyroBias: Vec3
  /** Estimated accelerometer bias (m/s^2). */
  accelBias: Vec3
  /** Wind, NED (m/s). */
  wind: Vec3
  /** Delivered thrust (N). */
  thrust: number
  /** Delivered body moment (N*m). */
  moment: Vec3
}

/** A fresh frame, so callers can hold one and reuse it. */
export function emptyFrame(): LiveFrame {
  return {
    time: 0,
    truthPosition: [0, 0, 0],
    truthVelocity: [0, 0, 0],
    quaternion: [1, 0, 0, 0],
    angularRate: [0, 0, 0],
    ekfPosition: [0, 0, 0],
    ekfVelocity: [0, 0, 0],
    positionVariance: [0, 0, 0],
    gyroBias: [0, 0, 0],
    accelBias: [0, 0, 0],
    wind: [0, 0, 0],
    thrust: 0,
    moment: [0, 0, 0],
  }
}

/**
 * Decode one sample block into `out`.
 *
 * `base` is the block's offset in doubles. Kept free of allocation: the caller
 * owns `out` and reuses it.
 */
export function decodeSample(flat: Float64Array, base: number, out: LiveFrame): void {
  const f = SAMPLE_FIELD
  out.time = flat[base + f.TIME] ?? 0
  out.truthPosition = [
    flat[base + f.TRUTH_POSITION] ?? 0,
    flat[base + f.TRUTH_POSITION + 1] ?? 0,
    flat[base + f.TRUTH_POSITION + 2] ?? 0,
  ]
  out.truthVelocity = [
    flat[base + f.TRUTH_VELOCITY] ?? 0,
    flat[base + f.TRUTH_VELOCITY + 1] ?? 0,
    flat[base + f.TRUTH_VELOCITY + 2] ?? 0,
  ]
  out.quaternion = [
    flat[base + f.TRUTH_QUATERNION] ?? 1,
    flat[base + f.TRUTH_QUATERNION + 1] ?? 0,
    flat[base + f.TRUTH_QUATERNION + 2] ?? 0,
    flat[base + f.TRUTH_QUATERNION + 3] ?? 0,
  ]
  out.angularRate = [
    flat[base + f.TRUTH_ANGULAR_RATE] ?? 0,
    flat[base + f.TRUTH_ANGULAR_RATE + 1] ?? 0,
    flat[base + f.TRUTH_ANGULAR_RATE + 2] ?? 0,
  ]
  out.ekfPosition = [
    flat[base + f.EKF_POSITION] ?? 0,
    flat[base + f.EKF_POSITION + 1] ?? 0,
    flat[base + f.EKF_POSITION + 2] ?? 0,
  ]
  out.ekfVelocity = [
    flat[base + f.EKF_VELOCITY] ?? 0,
    flat[base + f.EKF_VELOCITY + 1] ?? 0,
    flat[base + f.EKF_VELOCITY + 2] ?? 0,
  ]
  out.positionVariance = [
    flat[base + f.POSITION_VARIANCE] ?? 0,
    flat[base + f.POSITION_VARIANCE + 1] ?? 0,
    flat[base + f.POSITION_VARIANCE + 2] ?? 0,
  ]
  out.gyroBias = [
    flat[base + f.GYRO_BIAS] ?? 0,
    flat[base + f.GYRO_BIAS + 1] ?? 0,
    flat[base + f.GYRO_BIAS + 2] ?? 0,
  ]
  out.accelBias = [
    flat[base + f.ACCEL_BIAS] ?? 0,
    flat[base + f.ACCEL_BIAS + 1] ?? 0,
    flat[base + f.ACCEL_BIAS + 2] ?? 0,
  ]
  out.wind = [
    flat[base + f.WIND] ?? 0,
    flat[base + f.WIND + 1] ?? 0,
    flat[base + f.WIND + 2] ?? 0,
  ]
  out.thrust = flat[base + f.THRUST] ?? 0
  out.moment = [
    flat[base + f.MOMENT] ?? 0,
    flat[base + f.MOMENT + 1] ?? 0,
    flat[base + f.MOMENT + 2] ?? 0,
  ]
}

/** Series the rail plots, in the order the recorder maintains them. */
export const LIVE_SERIES = [
  'altitude',
  'groundSpeed',
  'verticalSpeed',
  'estimateError',
  'windSpeed',
  'thrust',
] as const

export type LiveSeriesName = (typeof LIVE_SERIES)[number]

/**
 * Append-only recorder for one live run.
 *
 * Owns the decoded series, the scene-space trail, and the derived scalars. The
 * caller feeds it the deltas the session hands back and reads whatever it needs
 * for a frame; nothing here allocates after construction.
 */
export class LiveBuffer {
  /** Samples recorded so far. */
  count = 0
  /** Sim time of the newest sample (s). */
  latestTime = 0
  /** Phase timeline as reported by the core, oldest first. */
  phases: PhaseEvent[] = []

  private readonly capacity: number
  private readonly flat: Float64Array
  /** Trail position in scene space, 3 floats per sample. */
  readonly trail: Float32Array
  /** Derived scalar series, one `Float32Array` per `LIVE_SERIES` entry. */
  readonly series: Record<LiveSeriesName, Float32Array>
  /** Sim time per sample, for the rail's time axis. */
  readonly times: Float32Array

  constructor(capacitySamples: number) {
    this.capacity = Math.max(capacitySamples, 1)
    this.flat = new Float64Array(this.capacity * SAMPLE_STRIDE)
    this.trail = new Float32Array(this.capacity * 3)
    this.times = new Float32Array(this.capacity)
    this.series = {
      altitude: new Float32Array(this.capacity),
      groundSpeed: new Float32Array(this.capacity),
      verticalSpeed: new Float32Array(this.capacity),
      estimateError: new Float32Array(this.capacity),
      windSpeed: new Float32Array(this.capacity),
      thrust: new Float32Array(this.capacity),
    }
  }

  /** Capacity in samples; the run's configured length. */
  get sampleCapacity(): number {
    return this.capacity
  }

  /** True once the buffer is full, which means the run outran its config. */
  get isFull(): boolean {
    return this.count >= this.capacity
  }

  /**
   * Append a streamed delta and derive everything the display needs.
   *
   * Returns the number of samples appended. A delta longer than the remaining
   * capacity is truncated rather than throwing, and `isFull` reports it: an
   * overrun means the config disagreed with the core, which the run view shows
   * rather than hides.
   */
  append(delta: Float64Array): number {
    const available = Math.floor(delta.length / SAMPLE_STRIDE)
    const room = this.capacity - this.count
    const n = Math.min(available, room)
    const base = this.count
    for (let i = 0; i < n; i += 1) {
      const src = i * SAMPLE_STRIDE
      const index = base + i
      const dst = index * SAMPLE_STRIDE
      this.flat.set(delta.subarray(src, src + SAMPLE_STRIDE), dst)
      const t = delta[src + SAMPLE_FIELD.TIME] ?? 0
      const pn = delta[src + SAMPLE_FIELD.TRUTH_POSITION] ?? 0
      const pe = delta[src + SAMPLE_FIELD.TRUTH_POSITION + 1] ?? 0
      const pd = delta[src + SAMPLE_FIELD.TRUTH_POSITION + 2] ?? 0
      const vn = delta[src + SAMPLE_FIELD.TRUTH_VELOCITY] ?? 0
      const ve = delta[src + SAMPLE_FIELD.TRUTH_VELOCITY + 1] ?? 0
      const vd = delta[src + SAMPLE_FIELD.TRUTH_VELOCITY + 2] ?? 0
      const en = delta[src + SAMPLE_FIELD.EKF_POSITION] ?? 0
      const ee = delta[src + SAMPLE_FIELD.EKF_POSITION + 1] ?? 0
      const ed = delta[src + SAMPLE_FIELD.EKF_POSITION + 2] ?? 0
      const wn = delta[src + SAMPLE_FIELD.WIND] ?? 0
      const we = delta[src + SAMPLE_FIELD.WIND + 1] ?? 0
      const wd = delta[src + SAMPLE_FIELD.WIND + 2] ?? 0

      const [x, y, z] = nedToScene([pn, pe, pd])
      this.trail[index * 3] = x
      this.trail[index * 3 + 1] = y
      this.trail[index * 3 + 2] = z
      this.times[index] = t
      this.series.altitude[index] = -pd
      this.series.groundSpeed[index] = Math.hypot(vn, ve)
      this.series.verticalSpeed[index] = -vd
      this.series.estimateError[index] = Math.hypot(pn - en, pe - ee, pd - ed)
      this.series.windSpeed[index] = Math.hypot(wn, we, wd)
      this.series.thrust[index] = delta[src + SAMPLE_FIELD.THRUST] ?? 0
      this.latestTime = t
    }
    this.count = base + n
    return n
  }

  /** Replace the phase timeline with the core's current list. */
  setPhases(phases: PhaseEvent[]): void {
    this.phases = phases
  }

  /**
   * Index of the newest recorded sample at or before `t`, clamped to what has
   * arrived. Returns -1 when nothing has been recorded yet.
   *
   * Floor rather than nearest: a live display must never read ahead of the
   * simulation, so a sample that has not arrived can not be sampled. Samples are
   * uniform in time, so the index is estimated directly and then walked to the
   * exact answer, which also absorbs any drift in the estimated period.
   */
  indexAt(t: number): number {
    if (this.count === 0) return -1
    const first = this.times[0] ?? 0
    const last = this.count - 1
    if (t <= first) return 0
    if (t >= (this.times[last] ?? 0)) return last
    const period = samplePeriod(this.times, this.count)
    let i = Math.floor((t - first) / period)
    if (i < 0) i = 0
    if (i > last) i = last
    while (i < last && (this.times[i + 1] ?? 0) <= t) i += 1
    while (i > 0 && (this.times[i] ?? 0) > t) i -= 1
    return i
  }

  /** Decode the sample at `index` into `out`. No-op out of range. */
  frameAt(index: number, out: LiveFrame): boolean {
    if (index < 0 || index >= this.count) return false
    decodeSample(this.flat, index * SAMPLE_STRIDE, out)
    return true
  }

  /** Decode the newest sample into `out`. */
  latest(out: LiveFrame): boolean {
    return this.frameAt(this.count - 1, out)
  }
}

/** Mean sample period from the recorded times, or the 100 Hz control rate. */
function samplePeriod(times: Float32Array, count: number): number {
  if (count < 2) return 0.01
  const first = times[0] ?? 0
  const last = times[count - 1] ?? 0
  const span = last - first
  if (span <= 0) return 0.01
  return span / (count - 1)
}
