import { describe, expect, it } from 'vitest'
import {
  buildPath,
  decimateMinMax,
  valueDomain,
  xScale,
  yScale,
} from './chart'

describe('decimateMinMax', () => {
  it('returns a copy when the series already fits', () => {
    const out = decimateMinMax([0, 1, 2], [5, 6, 7], 10)
    expect(out.times).toEqual([0, 1, 2])
    expect(out.values).toEqual([5, 6, 7])
    expect(out.times).not.toBe([0, 1, 2])
  })

  it('reduces below the budget and keeps time order', () => {
    const times = Array.from({ length: 200 }, (_, i) => i)
    const values = times.map((t) => Math.sin(t / 10))
    const out = decimateMinMax(times, values, 40)
    expect(out.values.length).toBeLessThanOrEqual(40)
    for (let i = 1; i < out.times.length; i++) {
      expect(out.times[i] as number).toBeGreaterThanOrEqual(
        out.times[i - 1] as number,
      )
    }
  })

  it('preserves a lone spike', () => {
    const times = Array.from({ length: 10 }, (_, i) => i)
    const values = [0, 0, 0, 0, 10, 0, 0, 0, 0, 0]
    const out = decimateMinMax(times, values, 4)
    expect(Math.max(...out.values)).toBe(10)
  })
})

describe('valueDomain', () => {
  it('falls back to a unit domain for an empty series', () => {
    expect(valueDomain([])).toEqual([0, 1])
  })

  it('pads a normal range', () => {
    const [lo, hi] = valueDomain([0, 10])
    expect(lo).toBeLessThan(0)
    expect(hi).toBeGreaterThan(10)
  })

  it('gives a flat series non-zero height', () => {
    const [lo, hi] = valueDomain([4, 4, 4])
    expect(hi).toBeGreaterThan(lo)
  })
})

describe('scales', () => {
  it('maps the time endpoints onto the plot edges', () => {
    expect(xScale(0, 0, 10, 100)).toBe(0)
    expect(xScale(10, 0, 10, 100)).toBe(100)
    expect(xScale(5, 0, 10, 100)).toBe(50)
  })

  it('inverts the value axis so the maximum sits at the top', () => {
    expect(yScale(0, 0, 10, 100)).toBe(100)
    expect(yScale(10, 0, 10, 100)).toBe(0)
  })
})

describe('buildPath', () => {
  it('is empty with no samples', () => {
    expect(buildPath([], [], 0, 1, 0, 1, 100, 50)).toBe('')
  })

  it('starts with a move and continues with lines', () => {
    const d = buildPath([0, 1], [0, 1], 0, 1, 0, 1, 100, 50)
    expect(d.startsWith('M')).toBe(true)
    expect(d).toContain('L')
  })
})
