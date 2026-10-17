import { describe, expect, it } from 'vitest'
import {
  buildPhaseSegments,
  findBracketingIndices,
  lerp1,
  lerp3,
  nedToScene,
  phaseAtTime,
  sampleScalar,
  sampleVec3,
} from './interpolation'
import type { PhaseEvent } from '@/types/flight'

describe('findBracketingIndices', () => {
  const times = [0, 1, 2, 3, 4, 5]

  it('brackets an interior time', () => {
    expect(findBracketingIndices(2.4, times)).toEqual([2, 3])
    expect(findBracketingIndices(2.0, times)).toEqual([2, 3])
  })

  it('clamps below the first sample', () => {
    expect(findBracketingIndices(-3, times)).toEqual([0, 1])
    expect(findBracketingIndices(0, times)).toEqual([0, 1])
  })

  it('clamps above the last sample', () => {
    expect(findBracketingIndices(5, times)).toEqual([4, 5])
    expect(findBracketingIndices(99, times)).toEqual([4, 5])
  })

  it('handles uniform dt arrays of realistic size', () => {
    const n = 7000
    const dt = 0.01
    const full = Array.from({ length: n }, (_, i) => i * dt)
    expect(findBracketingIndices(34.5678, full)).toEqual([3456, 3457])
    expect(findBracketingIndices(69.99, full)).toEqual([n - 2, n - 1])
  })

  it('degenerates gracefully on tiny arrays', () => {
    expect(findBracketingIndices(1, [])).toEqual([0, 0])
    expect(findBracketingIndices(1, [7])).toEqual([0, 0])
  })
})

describe('lerp1 / lerp3', () => {
  it('interpolates scalars', () => {
    expect(lerp1(2, 6, 0.25)).toBe(3)
    expect(lerp1(2, 6, 0)).toBe(2)
    expect(lerp1(2, 6, 1)).toBe(6)
  })

  it('interpolates vectors componentwise', () => {
    expect(lerp3([0, -10, 2], [10, 0, -2], 0.5)).toEqual([5, -5, 0])
  })
})

describe('sampleVec3 / sampleScalar', () => {
  const times = [0, 1, 2]
  const positions: [number, number, number][] = [
    [0, 0, 0],
    [10, -4, 2],
    [20, -8, 4],
  ]

  it('samples midway between bracketing samples', () => {
    expect(sampleVec3(0.5, times, positions)).toEqual([5, -2, 1])
  })

  it('clamps outside the time range', () => {
    expect(sampleVec3(-1, times, positions)).toEqual([0, 0, 0])
    expect(sampleVec3(99, times, positions)).toEqual([20, -8, 4])
  })

  it('samples scalar series with the same bracketing rule', () => {
    expect(sampleScalar(1.5, times, [0, 10, 20])).toBe(15)
  })
})

describe('phaseAtTime', () => {
  // Mirrors gps_outage: stacked instantaneous transitions at t=0.
  const timeline: PhaseEvent[] = [
    [0, 'idle'],
    [0, 'arm'],
    [0, 'takeoff'],
    [5.125, 'cruise'],
    [21.26, 'rtl'],
  ]

  it('resolves stacked t=0 transitions to the final state', () => {
    expect(phaseAtTime(timeline, 0)).toBe('takeoff')
  })

  it('tracks phase onsets in order', () => {
    expect(phaseAtTime(timeline, 2)).toBe('takeoff')
    expect(phaseAtTime(timeline, 5.125)).toBe('cruise')
    expect(phaseAtTime(timeline, 30)).toBe('rtl')
  })

  it('falls back to idle on an empty timeline', () => {
    expect(phaseAtTime([], 1)).toBe('idle')
  })
})

describe('buildPhaseSegments', () => {
  it('drops zero-width stacked transitions and extends the last phase', () => {
    const timeline: PhaseEvent[] = [
      [0, 'idle'],
      [0, 'arm'],
      [0, 'takeoff'],
      [5, 'cruise'],
    ]
    expect(buildPhaseSegments(10, timeline)).toEqual([
      { t0: 0, t1: 5, phase: 'takeoff' },
      { t0: 5, t1: 10, phase: 'cruise' },
    ])
  })
})

describe('nedToScene', () => {
  it('maps NED to scene with up = -down', () => {
    expect(nedToScene([3, -2, 5])).toEqual([3, 2, 5])
  })
})
