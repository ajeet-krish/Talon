import type { FlightPhase, PhaseEvent, Vec3 } from '@/types/flight'

/**
 * Coordinate conventions.
 *
 * Telemetry is NED (north, east, down): +x north, +y down, +z east.
 * Three.js scene is a right-handed frame with +y up:
 *
 *   scene = [n, -d, e]   i.e.   x = north, y = up (= -down), z = east
 *
 * The transform matrix has determinant +1, so it is a pure rotation and
 * preserves handedness (no mirrored geometry or crossed products).
 */
export function nedToScene(v: Vec3): [number, number, number] {
  return [v[0], -v[1], v[2]]
}

/**
 * Largest index i such that times[i] <= t, returned with i + 1 so the pair
 * brackets t for linear interpolation. Handles endpoints by clamping:
 * t <= times[0] -> [0, 1]; t >= times[N-1] -> [N-2, N-1].
 * Assumes times is sorted ascending with at least 2 samples.
 */
export function findBracketingIndices(
  t: number,
  times: readonly number[],
): [number, number] {
  const n = times.length
  if (n === 0) return [0, 0]
  if (n === 1) return [0, 0]
  const first = times[0] as number
  const last = times[n - 1] as number
  const tc = Math.min(Math.max(t, first), last)
  if (tc <= first) return [0, 1]
  if (tc >= last) return [n - 2, n - 1]
  // Invariant: times[lo] <= tc < times[hi]; bisect until adjacent.
  let lo = 0
  let hi = n - 1
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if ((times[mid] as number) <= tc) lo = mid
    else hi = mid
  }
  return [lo, hi]
}

export function lerp1(a: number, b: number, u: number): number {
  return a + (b - a) * u
}

export function lerp3(a: Vec3, b: Vec3, u: number): Vec3 {
  return [lerp1(a[0], b[0], u), lerp1(a[1], b[1], u), lerp1(a[2], b[2], u)]
}

/**
 * Linearly interpolate a per-sample vector series at time t.
 * Returns the NED triple; convert with nedToScene for rendering.
 */
export function sampleVec3(
  t: number,
  times: readonly number[],
  values: readonly Vec3[],
): Vec3 {
  const [i0, i1] = findBracketingIndices(t, times)
  const a = values[i0] as Vec3
  const b = values[i1] as Vec3
  const t0 = times[i0] as number
  const t1 = times[i1] as number
  const u = t1 > t0 ? (Math.min(Math.max(t, t0), t1) - t0) / (t1 - t0) : 0
  return lerp3(a, b, u)
}

/** Linearly interpolate a per-sample scalar series at time t. */
export function sampleScalar(
  t: number,
  times: readonly number[],
  values: readonly number[],
): number {
  const [i0, i1] = findBracketingIndices(t, times)
  const a = values[i0] as number
  const b = values[i1] as number
  const t0 = times[i0] as number
  const t1 = times[i1] as number
  const u = t1 > t0 ? (Math.min(Math.max(t, t0), t1) - t0) / (t1 - t0) : 0
  return lerp1(a, b, u)
}

/**
 * Phase active at time t: the last timeline entry whose onset is <= t.
 * Duplicate onset times (idle/arm/takeoff all at t=0) resolve to the
 * final transition, matching the state machine's end state at that instant.
 */
export function phaseAtTime(
  timeline: readonly PhaseEvent[],
  t: number,
): FlightPhase {
  if (timeline.length === 0) return 'idle'
  let phase = timeline[0]?.[1] ?? 'idle'
  for (const [onset, p] of timeline) {
    if (onset <= t) phase = p
    else break
  }
  return phase
}

export interface PhaseSegment {
  t0: number
  t1: number
  phase: FlightPhase
}

/**
 * Expand a phase_timeline into contiguous [t0, t1) segments covering
 * [0, duration]. Zero-width segments (instantaneous transitions stacked
 * at one instant) are dropped so the timeline bar stays readable.
 */
export function buildPhaseSegments(
  duration: number,
  timeline: readonly PhaseEvent[],
): PhaseSegment[] {
  const segments: PhaseSegment[] = []
  for (let i = 0; i < timeline.length; i++) {
    const entry = timeline[i] as PhaseEvent
    const next = timeline[i + 1]
    const t0 = entry[0]
    const t1 = next ? next[0] : duration
    if (t1 - t0 > 1e-9) segments.push({ t0, t1, phase: entry[1] })
  }
  return segments
}
