import { describe, expect, it } from 'vitest'
import { formatMetric } from './format'

describe('formatMetric', () => {
  it('keeps three decimals below ten', () => {
    expect(formatMetric(0.0123)).toBe('0.012')
    expect(formatMetric(1.2)).toBe('1.200')
  })

  it('trims decimals as magnitude grows', () => {
    expect(formatMetric(12.5)).toBe('12.50')
    expect(formatMetric(123.4)).toBe('123.4')
    expect(formatMetric(1234)).toBe('1234')
  })

  it('renders zero plainly and guards non-finite input', () => {
    expect(formatMetric(0)).toBe('0')
    expect(formatMetric(Number.NaN)).toBe('--')
    expect(formatMetric(Infinity)).toBe('--')
  })

  it('keeps the sign on negative values', () => {
    expect(formatMetric(-1.5)).toBe('-1.500')
  })
})
