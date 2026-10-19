/**
 * Recorder sizing tests.
 *
 * `bufferCapacity` is the only thing between a scenario config and a typed
 * array the renderer allocates once per scenario, so the two failure modes are
 * sized here: a corrupt duration that must never reach the allocator, and a
 * run long enough to ask for an unbounded buffer. The zero-capacity case is
 * pinned because the run view builds `new LiveBuffer(bufferCapacity(...))`
 * directly: capacity 0 must construct, not throw.
 */

import { describe, expect, it } from 'vitest'
import { bufferCapacity, MAX_BUFFER_SAMPLES } from './readout'
import { LiveBuffer, SAMPLE_STRIDE } from './stream'
import type { LiveConfig } from './types'

function config(overrides: Partial<LiveConfig> = {}): LiveConfig {
  return {
    id: 'test',
    label: 'Test',
    blurb: '',
    duration_s: 60,
    takeoff_altitude: 10,
    start_position_ned: [0, 0, 0],
    waypoints: [],
    obstacles: [],
    wind_sigma: [0, 0, 0],
    wind_steady_ned: [0, 0, 0],
    seed: 1,
    gps_position_noise: 0,
    gps_position_noise_vertical: 0,
    gps_velocity_noise: 0,
    cruise_hold_s: 0,
    auto_arm: true,
    ...overrides,
  }
}

describe('bufferCapacity', () => {
  it('sizes a run at the control rate plus headroom', () => {
    expect(bufferCapacity(config({ duration_s: 120 }), 0.01)).toBe(12_064)
    expect(bufferCapacity(config({ duration_s: 70 }), 0.01)).toBe(7_064)
    expect(bufferCapacity(config({ duration_s: 60 }), 0.005)).toBe(12_064)
  })

  it('caps the allocation at 900 s of 200 Hz plus headroom', () => {
    expect(MAX_BUFFER_SAMPLES).toBe(180_064)
    // Exactly the bound: 900 s at 200 Hz, plus the 64-sample headroom.
    expect(bufferCapacity(config({ duration_s: 900 }), 0.005)).toBe(180_064)
    // Anything longer is clamped rather than allocated.
    expect(bufferCapacity(config({ duration_s: 900 }), 0.001)).toBe(180_064)
    expect(bufferCapacity(config({ duration_s: 3600 }), 0.01)).toBe(180_064)
  })

  it('returns 0 for a non-finite or non-positive duration', () => {
    expect(bufferCapacity(config({ duration_s: 0 }), 0.01)).toBe(0)
    expect(bufferCapacity(config({ duration_s: -30 }), 0.01)).toBe(0)
    expect(bufferCapacity(config({ duration_s: Number.NaN }), 0.01)).toBe(0)
    expect(bufferCapacity(config({ duration_s: Number.POSITIVE_INFINITY }), 0.01)).toBe(0)
    expect(bufferCapacity(config({ duration_s: Number.NEGATIVE_INFINITY }), 0.01)).toBe(0)
  })

  it('returns 0 rather than NaN when the control period can not divide the duration', () => {
    expect(bufferCapacity(config({ duration_s: 60 }), 0)).toBe(0)
    expect(bufferCapacity(config({ duration_s: 60 }), Number.NaN)).toBe(0)
    expect(bufferCapacity(config({ duration_s: 60 }), -0.01)).toBe(0)
  })

  it('never returns a negative or non-finite capacity', () => {
    for (const duration of [0.001, 1, 899, 900]) {
      for (const dt of [0.001, 0.005, 0.01, 1]) {
        const capacity = bufferCapacity(config({ duration_s: duration }), dt)
        expect(Number.isFinite(capacity), `${duration}@${dt}`).toBe(true)
        expect(capacity, `${duration}@${dt}`).toBeGreaterThanOrEqual(0)
        expect(capacity, `${duration}@${dt}`).toBeLessThanOrEqual(MAX_BUFFER_SAMPLES)
      }
    }
  })

  it('constructs the LiveBuffer the run view builds for a normal config', () => {
    const buffer = new LiveBuffer(bufferCapacity(config({ duration_s: 120 }), 0.01))
    expect(buffer.sampleCapacity).toBe(12_064)
    expect(buffer.count).toBe(0)
  })

  it('constructs a LiveBuffer at capacity 0 instead of crashing the run view', () => {
    // RunView does `new LiveBuffer(bufferCapacity(selected, DT_CONTROL))`, so
    // a rejected duration must survive the constructor. LiveBuffer floors its
    // own capacity at one sample: nothing can be recorded, the first append
    // reports the buffer full, and the run shows as an overrun.
    const capacity = bufferCapacity(config({ duration_s: 0 }), 0.01)
    expect(capacity).toBe(0)
    const buffer = new LiveBuffer(capacity)
    expect(buffer.sampleCapacity).toBe(1)
    expect(buffer.append(new Float64Array(SAMPLE_STRIDE))).toBe(1)
    expect(buffer.isFull).toBe(true)
  })
})
