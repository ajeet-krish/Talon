/**
 * Chart helpers for the SVG telemetry view. Deliberately DOM-free and pure so
 * the node vitest environment can exercise them without a canvas or WebGL.
 */

export interface Series {
  times: number[]
  values: number[]
}

/**
 * Min/max bucket decimation. Reduces a series to roughly 2 * buckets points
 * while preserving local extremes, so spikes (EKF divergence, gust peaks,
 * clearance minimums) survive the reduction and the polyline does not
 * zigzag: within each bucket the earlier-index extreme is emitted first.
 *
 * Returns shallow copies of the inputs when no reduction is needed.
 */
export function decimateMinMax(
  times: readonly number[],
  values: readonly number[],
  maxPoints: number,
): Series {
  const n = Math.min(times.length, values.length)
  if (maxPoints < 2 || n <= maxPoints) {
    return { times: [...times], values: [...values] }
  }
  const buckets = Math.max(1, Math.floor(maxPoints / 2))
  const bucketSize = n / buckets
  const outTimes: number[] = []
  const outValues: number[] = []
  for (let b = 0; b < buckets; b++) {
    const start = Math.floor(b * bucketSize)
    const end = Math.min(n, Math.floor((b + 1) * bucketSize))
    if (end <= start) continue
    let iMin = start
    let iMax = start
    for (let i = start + 1; i < end; i++) {
      const v = values[i] as number
      if (v < (values[iMin] as number)) iMin = i
      if (v > (values[iMax] as number)) iMax = i
    }
    const [first, second] = iMin <= iMax ? [iMin, iMax] : [iMax, iMin]
    outTimes.push(times[first] as number)
    outValues.push(values[first] as number)
    if (second !== first) {
      outTimes.push(times[second] as number)
      outValues.push(values[second] as number)
    }
  }
  return { times: outTimes, values: outValues }
}

/** Padded [min, max] domain that always has non-zero height. */
export function valueDomain(values: readonly number[]): [number, number] {
  if (values.length === 0) return [0, 1]
  let min = Infinity
  let max = -Infinity
  for (const v of values) {
    if (v < min) min = v
    if (v > max) max = v
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [0, 1]
  if (min === max) {
    const pad = Math.abs(min) * 0.1 || 1
    return [min - pad, max + pad]
  }
  const pad = (max - min) * 0.08
  return [min - pad, max + pad]
}

/** Map a time value onto a chart x pixel (left-aligned plot area). */
export function xScale(t: number, t0: number, t1: number, width: number): number {
  if (t1 <= t0) return 0
  return ((t - t0) / (t1 - t0)) * width
}

/** Map a data value onto a chart y pixel (inverted: max at the top). */
export function yScale(v: number, lo: number, hi: number, height: number): number {
  if (hi <= lo) return height
  return height - ((v - lo) / (hi - lo)) * height
}

/**
 * SVG path string for a series inside a plot rect of the given pixel size,
 * with the time axis spanning [t0, t1] and the value axis spanning [lo, hi].
 */
export function buildPath(
  times: readonly number[],
  values: readonly number[],
  t0: number,
  t1: number,
  lo: number,
  hi: number,
  width: number,
  height: number,
): string {
  const n = Math.min(times.length, values.length)
  if (n === 0) return ''
  let d = ''
  for (let i = 0; i < n; i++) {
    const x = xScale(times[i] as number, t0, t1, width)
    const y = yScale(values[i] as number, lo, hi, height)
    d += `${i === 0 ? 'M' : 'L'}${x.toFixed(1)} ${y.toFixed(1)}`
    if (i !== n - 1) d += ' '
  }
  return d
}
