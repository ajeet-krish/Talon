import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { NLGLGuidance } from './guidance'
import type { Vec3 } from './math'

interface GuidanceCase {
  name: string
  position: Vec3
  velocity: Vec3
  wp_a: Vec3
  wp_b: Vec3
  v_des: Vec3
  e_ct: number
  nu1: number
  nu2: number
  nu: number
  l1: number
  chi_d: number
  a_lat: number
  behind_a_recovery: boolean
}

function readJson(rel: string): unknown {
  return JSON.parse(
    readFileSync(decodeURIComponent(new URL(rel, import.meta.url).pathname), 'utf8'),
  )
}

const cases = (readJson('../../fixtures/live_fixtures.json') as { guidance: GuidanceCase[] })
  .guidance

describe('NLGL guidance (TypeScript) matches the Python reference', () => {
  const guidance = new NLGLGuidance()

  it('has reference cases', () => {
    expect(cases.length).toBeGreaterThanOrEqual(5)
  })

  for (const c of cases) {
    it(`leg case: ${c.name}`, () => {
      const { v_des, diagnostics } = guidance.computeLeg(
        c.position,
        c.velocity,
        c.wp_a,
        c.wp_b,
      )
      expect(v_des[0]).toBeCloseTo(c.v_des[0], 9)
      expect(v_des[1]).toBeCloseTo(c.v_des[1], 9)
      expect(v_des[2]).toBeCloseTo(c.v_des[2], 9)
      expect(diagnostics.e_ct).toBeCloseTo(c.e_ct, 9)
      expect(diagnostics.nu1).toBeCloseTo(c.nu1, 9)
      expect(diagnostics.nu2).toBeCloseTo(c.nu2, 9)
      expect(diagnostics.nu).toBeCloseTo(c.nu, 9)
      expect(diagnostics.l1).toBeCloseTo(c.l1, 9)
      expect(diagnostics.chi_d).toBeCloseTo(c.chi_d, 9)
      expect(diagnostics.a_lat).toBeCloseTo(c.a_lat, 9)
      expect(diagnostics.behind_a_recovery).toBe(c.behind_a_recovery)
    })
  }

  it('wraps commanded course to [-pi, pi)', () => {
    const { diagnostics } = guidance.computeLeg(
      [0, 0, -10],
      [0, 0, 0],
      [0, 0, -10],
      [40, 0, -10],
    )
    expect(diagnostics.chi_d).toBeGreaterThanOrEqual(-Math.PI)
    expect(diagnostics.chi_d).toBeLessThan(Math.PI)
  })
})
