/**
 * Pacer tests.
 *
 * The pacer decides how much of the run to reveal per frame, so these tests are
 * about the two things that go wrong in a paced display: drifting from the wall
 * clock, and jumping when the rate or the pause state changes.
 *
 * Wall time is fed in frame-sized steps with an exact divisor, because a single
 * huge tick is clamped on purpose (a hidden tab must not teleport the vehicle)
 * and a test that pretends otherwise would be testing the wrong thing.
 */

import { describe, expect, it } from 'vitest'
import { MAX_FRAME_MS, Pacer, RATES, rateLabel } from './pacing'

/** Feed `totalMs` of wall time in `frameMs` steps. */
function advance(pacer: Pacer, totalMs: number, frameMs = 20): void {
  let remaining = totalMs
  while (remaining > 0) {
    const step = Math.min(frameMs, remaining)
    pacer.tick(step)
    remaining -= step
  }
}

describe('Pacer', () => {
  it('does not move until it is started', () => {
    const pacer = new Pacer(60, 1)
    advance(pacer, 500)
    expect(pacer.targetTime()).toBe(0)
    expect(pacer.isRunning).toBe(false)
  })

  it('advances one second of sim time per second of wall time at 1x', () => {
    const pacer = new Pacer(60, 1)
    pacer.start(0)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBeCloseTo(1, 9)
    advance(pacer, 250, 25)
    expect(pacer.targetTime()).toBeCloseTo(1.25, 9)
  })

  it('scales sim time with the rate', () => {
    for (const rate of [1, 4, 16]) {
      const pacer = new Pacer(600, rate)
      pacer.start(0)
      advance(pacer, 1000)
      expect(pacer.targetTime(), `${rate}x`).toBeCloseTo(rate, 9)
    }
  })

  it('treats an infinite rate as run to the end, without any wall time', () => {
    const pacer = new Pacer(120, Infinity)
    pacer.start(0)
    expect(pacer.targetTime()).toBe(120)
    expect(pacer.isComplete()).toBe(true)
  })

  it('holds the target while paused and resumes from there', () => {
    const pacer = new Pacer(60, 4)
    pacer.start(0)
    advance(pacer, 500, 25)
    expect(pacer.targetTime()).toBeCloseTo(2, 9)
    pacer.pause(2)
    pacer.tick(10_000)
    expect(pacer.targetTime()).toBeCloseTo(2, 9)
    pacer.start(2)
    advance(pacer, 250, 25)
    expect(pacer.targetTime()).toBeCloseTo(3, 9)
  })

  it('re-bases on the current sim time when the rate changes', () => {
    // The point: switching from 16x to 1x must not snap the vehicle.
    const pacer = new Pacer(600, 16)
    pacer.start(0)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBeCloseTo(16, 9)
    const actualSimTime = 12.5
    pacer.setRate(1, actualSimTime)
    expect(pacer.targetTime()).toBeCloseTo(actualSimTime, 9)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBeCloseTo(13.5, 9)
    expect(pacer.playbackRate).toBe(1)
  })

  it('never runs past the end of the scenario', () => {
    const pacer = new Pacer(10, 16)
    pacer.start(0)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBe(10)
    expect(pacer.isComplete()).toBe(true)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBe(10)
  })

  it('clamps a tab-switch time jump instead of teleporting the vehicle', () => {
    const pacer = new Pacer(600, 1)
    pacer.start(0)
    pacer.tick(30_000)
    expect(pacer.targetTime()).toBeCloseTo(MAX_FRAME_MS / 1000, 9)
  })

  it('ignores a negative or zero frame delta', () => {
    const pacer = new Pacer(60, 1)
    pacer.start(0)
    pacer.tick(-5)
    pacer.tick(0)
    expect(pacer.targetTime()).toBe(0)
  })

  it('resets to the start', () => {
    const pacer = new Pacer(60, 4)
    pacer.start(0)
    advance(pacer, 1000)
    pacer.reset()
    expect(pacer.isRunning).toBe(false)
    expect(pacer.targetTime()).toBe(0)
  })

  it('starts from a non-zero sim time when asked', () => {
    const pacer = new Pacer(60, 1, 5)
    pacer.start(5)
    advance(pacer, 1000)
    expect(pacer.targetTime()).toBeCloseTo(6, 9)
  })
})

describe('rate labels', () => {
  it('labels the finite rates with a multiplier and the last one as max', () => {
    expect(rateLabel(1)).toBe('1\u00d7')
    expect(rateLabel(16)).toBe('16\u00d7')
    expect(rateLabel(Infinity)).toBe('MAX')
  })

  it('offers the documented rates in order', () => {
    expect(RATES).toEqual([1, 4, 16, Infinity])
  })
})
