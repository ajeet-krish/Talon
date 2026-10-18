import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { forwardMix, MotorModel, motorMixing } from './motors'
import type { Vec3 } from './math'

interface MotorsFixture {
  mixing: { thrust: number; moment: Vec3; forces: number[] }[]
  forward: { forces: number[]; thrust: number; moment: Vec3 }[]
  lag: { step: number; forces: number[] }[]
  lag_dt: number
  lag_thrust: number
}

function readJson(rel: string): unknown {
  return JSON.parse(
    readFileSync(decodeURIComponent(new URL(rel, import.meta.url).pathname), 'utf8'),
  )
}

const fixture = (readJson('../../fixtures/live_fixtures.json') as { motors: MotorsFixture })
  .motors

describe('Motor mixing and lag (TypeScript) match the Python reference', () => {
  it('inverse mix matches', () => {
    for (const c of fixture.mixing) {
      const forces = motorMixing(c.thrust, c.moment)
      for (let i = 0; i < 4; i += 1) {
        expect(forces[i] as number).toBeCloseTo(c.forces[i] as number, 9)
      }
    }
  })

  it('forward mix matches', () => {
    for (const c of fixture.forward) {
      const [thrust, moment] = forwardMix(c.forces)
      expect(thrust).toBeCloseTo(c.thrust, 9)
      for (let i = 0; i < 3; i += 1) {
        expect(moment[i]).toBeCloseTo(c.moment[i] as number, 9)
      }
    }
  })

  it('inverse then forward mix round-trips', () => {
    const moment: Vec3 = [0.05, -0.03, 0.01]
    const forces = motorMixing(20.0, moment)
    const [thrust, recovered] = forwardMix(forces)
    expect(thrust).toBeCloseTo(20.0, 9)
    for (let i = 0; i < 3; i += 1) {
      expect(recovered[i]).toBeCloseTo(moment[i] as number, 9)
    }
  })

  it('first-order lag sequence matches', () => {
    const model = new MotorModel()
    for (const entry of fixture.lag) {
      const forces = model.step(fixture.lag_thrust, [0, 0, 0], fixture.lag_dt)
      for (let i = 0; i < 4; i += 1) {
        expect(forces[i] as number).toBeCloseTo(entry.forces[i] as number, 9)
      }
    }
  })
})
