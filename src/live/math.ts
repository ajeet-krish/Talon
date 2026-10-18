/**
 * Small pure vector / quaternion / RNG helpers for the live-run glue.
 *
 * Kept dependency-free (no three.js, no numpy-alike) so the glue is testable in
 * the node vitest environment and mirrors the numpy expressions in the Python
 * sources one-to-one.
 */

/** NED triple: [north, east, down]. */
export type Vec3 = [number, number, number]

export function v3(x: number, y: number, z: number): Vec3 {
  return [x, y, z]
}

export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s]
}

export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

export function norm(a: Vec3): number {
  return Math.sqrt(dot(a, a))
}

/** 2-D cross product z component: a.x * b.y - a.y * b.x. */
export function cross2(a: readonly [number, number], b: readonly [number, number]): number {
  return a[0] * b[1] - a[1] * b[0]
}

/** Wrap an angle to [-pi, pi). */
export function wrapAngle(theta: number): number {
  return ((theta + Math.PI) % (2 * Math.PI)) - Math.PI
}

export function clamp(value: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, value))
}

export function clampVec(a: Vec3, lo: number, hi: number): Vec3 {
  return [clamp(a[0], lo, hi), clamp(a[1], lo, hi), clamp(a[2], lo, hi)]
}

/**
 * Seeded normal-noise stream (mulberry32 uniforms + Box-Muller, caching the
 * second deviate).
 *
 * Deterministic for a given seed, so a live run is reproducible in the browser
 * and in tests. NOTE: this is deliberately NOT numpy's PCG64 + ziggurat, so a
 * browser run is statistically equivalent to, but not bit-identical with, the
 * recorded Python runs; the browser/TS glue is checked against Python fixtures
 * on the deterministic math (guidance, mission, mixing, and the kernel), not on
 * the noise realization.
 */
export class Rng {
  private state: number
  private hasSpare = false
  private spare = 0

  constructor(seed: number) {
    this.state = seed >>> 0
  }

  /** Uniform in [0, 1). */
  nextUniform(): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0
    let t = this.state
    t = Math.imul(t ^ (t >>> 15), t | 1)
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61)
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296
  }

  /** Standard normal via Box-Muller. */
  standardNormal(): number {
    if (this.hasSpare) {
      this.hasSpare = false
      return this.spare
    }
    let u = 0
    while (u === 0) u = this.nextUniform()
    const v = this.nextUniform()
    const mag = Math.sqrt(-2 * Math.log(u))
    this.spare = mag * Math.sin(2 * Math.PI * v)
    this.hasSpare = true
    return mag * Math.cos(2 * Math.PI * v)
  }

  /** (n,) vector of standard normals. */
  standardNormalVec(n: number): number[] {
    const out = new Array<number>(n)
    for (let i = 0; i < n; i += 1) out[i] = this.standardNormal()
    return out
  }
}
