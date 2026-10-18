/**
 * Live stream codec tests.
 *
 * These cover the agreement between the layout the Rust core streams and the
 * layout this side decodes, which nothing in the type system enforces. The
 * blocks here are built by writing at the same named offsets the core uses, so a
 * reordered field fails a test rather than silently mislabelling a series.
 */

import { describe, expect, it } from 'vitest'
import {
  LIVE_SERIES,
  LiveBuffer,
  PHASE_ORDER,
  SAMPLE_FIELD,
  SAMPLE_STRIDE,
  decodeSample,
  emptyFrame,
  phaseIndexOf,
} from './stream'

type FieldName = keyof typeof SAMPLE_FIELD

/** Build one sample block at the named offsets. */
function block(
  time: number,
  fields: Partial<Record<FieldName, number[]>> = {},
): Float64Array {
  const b = new Float64Array(SAMPLE_STRIDE)
  b[SAMPLE_FIELD.TIME] = time
  for (const key of Object.keys(fields) as FieldName[]) {
    const values = fields[key]
    if (!values) continue
    const offset = SAMPLE_FIELD[key]
    values.forEach((v, i) => {
      b[offset + i] = v
    })
  }
  return b
}

function concat(blocks: Float64Array[]): Float64Array {
  const out = new Float64Array(blocks.length * SAMPLE_STRIDE)
  blocks.forEach((b, i) => out.set(b, i * SAMPLE_STRIDE))
  return out
}

/** A representative block: 12 m up, 3 m/s north, level. */
function sample(time: number): Float64Array {
  return block(time, {
    TRUTH_POSITION: [10, 4, -12],
    TRUTH_VELOCITY: [3, 0, -0.5],
    TRUTH_QUATERNION: [1, 0, 0, 0],
    TRUTH_ANGULAR_RATE: [0, 0, 0],
    EKF_POSITION: [10.5, 4.5, -11.5],
    EKF_VELOCITY: [3, 0, -0.5],
    POSITION_VARIANCE: [0.25, 0.25, 0.5],
    GYRO_BIAS: [0.001, 0, 0],
    ACCEL_BIAS: [0.01, 0, 0],
    WIND: [2, -1, 0.2],
    THRUST: [14.7],
    MOMENT: [0.1, 0, 0],
  })
}

describe('sample layout', () => {
  it('closes the stride with the last field', () => {
    expect(SAMPLE_FIELD.MOMENT + 3).toBe(SAMPLE_STRIDE)
  })

  it('keeps every field inside the stride', () => {
    for (const [name, offset] of Object.entries(SAMPLE_FIELD)) {
      expect(offset, name).toBeGreaterThanOrEqual(0)
      expect(offset, name).toBeLessThan(SAMPLE_STRIDE)
    }
  })
})

describe('decodeSample', () => {
  it('reads every field at its documented offset', () => {
    const out = emptyFrame()
    decodeSample(sample(1.25), 0, out)
    expect(out.time).toBe(1.25)
    expect(out.truthPosition).toEqual([10, 4, -12])
    expect(out.truthVelocity).toEqual([3, 0, -0.5])
    expect(out.quaternion).toEqual([1, 0, 0, 0])
    expect(out.ekfPosition).toEqual([10.5, 4.5, -11.5])
    expect(out.positionVariance).toEqual([0.25, 0.25, 0.5])
    expect(out.wind).toEqual([2, -1, 0.2])
    expect(out.thrust).toBe(14.7)
    expect(out.moment).toEqual([0.1, 0, 0])
  })

  it('reads a block at a non-zero base', () => {
    const out = emptyFrame()
    const flat = concat([sample(0), sample(0.01)])
    decodeSample(flat, SAMPLE_STRIDE, out)
    expect(out.time).toBe(0.01)
    expect(out.truthPosition).toEqual([10, 4, -12])
  })
})

describe('LiveBuffer', () => {
  it('derives altitude, speed, vertical speed, error, wind, and thrust', () => {
    const buf = new LiveBuffer(4)
    expect(buf.append(concat([sample(0)]))).toBe(1)
    expect(buf.count).toBe(1)
    expect(buf.latestTime).toBe(0)

    const out = emptyFrame()
    expect(buf.latest(out)).toBe(true)
    expect(buf.series.altitude[0]).toBeCloseTo(12, 6)
    expect(buf.series.groundSpeed[0]).toBeCloseTo(3, 6)
    expect(buf.series.verticalSpeed[0]).toBeCloseTo(0.5, 6)
    // Truth (10, 4, -12) against estimate (10.5, 4.5, -11.5): half a metre on
    // each axis, so sqrt(0.75).
    expect(buf.series.estimateError[0]).toBeCloseTo(Math.sqrt(0.75), 6)
    expect(buf.series.windSpeed[0]).toBeCloseTo(Math.hypot(2, 1, 0.2), 6)
    // The derived series are Float32, so a truck-scale comparison is enough.
    expect(buf.series.thrust[0]).toBeCloseTo(14.7, 5)
  })

  it('writes the trail in scene space, not NED', () => {
    const buf = new LiveBuffer(2)
    buf.append(concat([sample(0)]))
    // NED [north, east, down] = [10, 4, -12] becomes scene [10, 12, 4].
    expect(buf.trail[0]).toBeCloseTo(10, 6)
    expect(buf.trail[1]).toBeCloseTo(12, 6)
    expect(buf.trail[2]).toBeCloseTo(4, 6)
  })

  it('appends several samples in one delta and keeps their order', () => {
    const buf = new LiveBuffer(8)
    expect(buf.append(concat([sample(0), sample(0.01), sample(0.02)]))).toBe(3)
    expect(buf.count).toBe(3)
    expect(buf.times[0]).toBeCloseTo(0, 6)
    expect(buf.times[1]).toBeCloseTo(0.01, 6)
    expect(buf.times[2]).toBeCloseTo(0.02, 6)
    expect(buf.latestTime).toBeCloseTo(0.02, 6)
  })

  it('truncates a delta that would overrun the capacity and says so', () => {
    const buf = new LiveBuffer(2)
    expect(buf.append(concat([sample(0), sample(0.01), sample(0.02)]))).toBe(2)
    expect(buf.count).toBe(2)
    expect(buf.isFull).toBe(true)
    expect(buf.append(concat([sample(0.03)]))).toBe(0)
  })

  it('ignores a partial block rather than decoding half a sample', () => {
    const buf = new LiveBuffer(4)
    // One and a half blocks: only the whole one counts.
    const partial = new Float64Array(SAMPLE_STRIDE + 5)
    partial.set(sample(0), 0)
    expect(buf.append(partial)).toBe(1)
    expect(buf.count).toBe(1)
  })

  it('reports a negative index before anything has arrived', () => {
    const buf = new LiveBuffer(4)
    expect(buf.indexAt(0)).toBe(-1)
    expect(buf.latest(emptyFrame())).toBe(false)
  })

  it('finds the newest sample at or before the requested time', () => {
    const buf = new LiveBuffer(8)
    const blocks: Float64Array[] = []
    for (let i = 0; i < 5; i += 1) blocks.push(sample(i * 0.01))
    buf.append(concat(blocks))
    expect(buf.indexAt(-1)).toBe(0)
    expect(buf.indexAt(0)).toBe(0)
    expect(buf.indexAt(0.004)).toBe(0)
    expect(buf.indexAt(0.01)).toBe(1)
    expect(buf.indexAt(0.016)).toBe(1)
    expect(buf.indexAt(0.02)).toBe(2)
    expect(buf.indexAt(10)).toBe(4)
  })

  it('handles a ragged sample period without going out of range', () => {
    const buf = new LiveBuffer(8)
    buf.append(concat([sample(0), sample(0.01), sample(0.05), sample(0.06)]))
    for (const t of [-1, 0, 0.02, 0.04, 0.055, 0.06, 5]) {
      const i = buf.indexAt(t)
      expect(i).toBeGreaterThanOrEqual(0)
      expect(i).toBeLessThan(buf.count)
    }
  })

  it('decodes an indexed frame and refuses one out of range', () => {
    const buf = new LiveBuffer(4)
    buf.append(concat([sample(0), sample(0.01)]))
    const out = emptyFrame()
    expect(buf.frameAt(1, out)).toBe(true)
    expect(out.time).toBeCloseTo(0.01, 6)
    expect(buf.frameAt(2, out)).toBe(false)
    expect(buf.frameAt(-1, out)).toBe(false)
  })

  it('takes the phase timeline from the core verbatim', () => {
    const buf = new LiveBuffer(2)
    buf.setPhases([
      [0, 'idle'],
      [0, 'arm'],
      [2, 'takeoff'],
    ])
    expect(buf.phases.map((p) => p[1])).toEqual(['idle', 'arm', 'takeoff'])
  })

  it('exposes one series per documented name', () => {
    const buf = new LiveBuffer(1)
    expect(Object.keys(buf.series).sort()).toEqual([...LIVE_SERIES].sort())
  })

  it('interpolates the pose between samples so the draw does not step', () => {
    const buf = new LiveBuffer(8)
    buf.append(
      concat([
        block(0, { TRUTH_POSITION: [0, 0, -10], TRUTH_VELOCITY: [2, 0, -1] }),
        block(0.05, { TRUTH_POSITION: [0.1, 0.2, -11], TRUTH_VELOCITY: [4, 0, -3] }),
      ]),
    )
    const frame = emptyFrame()
    expect(buf.poseAt(0.025, frame)).toBe(true)
    expect(frame.time).toBeCloseTo(0.025, 6)
    expect(frame.truthPosition[0]).toBeCloseTo(0.05, 6)
    expect(frame.truthPosition[1]).toBeCloseTo(0.1, 6)
    expect(frame.truthPosition[2]).toBeCloseTo(-10.5, 6)
    expect(frame.truthVelocity[0]).toBeCloseTo(3, 6)
    expect(frame.truthVelocity[2]).toBeCloseTo(-2, 6)
  })

  it('holds at the newest sample rather than extrapolating', () => {
    const buf = new LiveBuffer(8)
    buf.append(concat([block(0, { TRUTH_POSITION: [0, 0, -10] }), block(0.01, { TRUTH_POSITION: [1, 0, -10] })]))
    const frame = emptyFrame()
    buf.poseAt(99, frame)
    expect(frame.truthPosition[0]).toBeCloseTo(1, 6)
    buf.poseAt(-5, frame)
    expect(frame.truthPosition[0]).toBeCloseTo(0, 6)
  })

  it('takes the shortest attitude path across a quaternion sign flip', () => {
    // q and -q are the same rotation. Interpolating naively through zero would
    // gimbal-lock the airframe; the sign flip avoids it.
    const buf = new LiveBuffer(8)
    buf.append(
      concat([
        block(0, { TRUTH_QUATERNION: [1, 0, 0, 0] }),
        block(0.01, { TRUTH_QUATERNION: [-1, 0, 0, 0] }),
      ]),
    )
    const frame = emptyFrame()
    buf.poseAt(0.005, frame)
    const [w] = frame.quaternion
    expect(Math.abs(w)).toBeCloseTo(1, 6)
  })

  it('refuses to pose the empty buffer', () => {
    const buf = new LiveBuffer(4)
    expect(buf.poseAt(0, emptyFrame())).toBe(false)
  })

  it('tags each sample with the phase in effect when it was recorded', () => {
    const buf = new LiveBuffer(8)
    // The timeline arrives first, as the run view reads it before draining.
    buf.setPhases([
      [0, 'idle'],
      [0, 'arm'],
      [0.01, 'takeoff'],
      [0.03, 'cruise'],
    ])
    const blocks: Float64Array[] = []
    for (let i = 0; i < 5; i += 1) blocks.push(sample(i * 0.01))
    buf.append(concat(blocks))
    const names = [...buf.phaseIndex].slice(0, buf.count).map((i) => PHASE_ORDER[i])
    expect(names).toEqual(['arm', 'takeoff', 'takeoff', 'cruise', 'cruise'])
  })

  it('keeps tagging new samples as the timeline grows', () => {
    const buf = new LiveBuffer(8)
    buf.setPhases([[0, 'idle'], [0, 'arm']])
    buf.append(concat([sample(0), sample(0.01)]))
    expect([...buf.phaseIndex].slice(0, 2).map((i) => PHASE_ORDER[i])).toEqual(['arm', 'arm'])
    // The core reports the takeoff event, then more samples arrive.
    buf.setPhases([[0, 'idle'], [0, 'arm'], [0.02, 'takeoff']])
    buf.append(concat([sample(0.02), sample(0.03)]))
    expect([...buf.phaseIndex].slice(0, 4).map((i) => PHASE_ORDER[i])).toEqual([
      'arm',
      'arm',
      'takeoff',
      'takeoff',
    ])
  })

  it('falls back to the first phase for an unknown label', () => {
    expect(phaseIndexOf('not-a-phase')).toBe(0)
    expect(phaseIndexOf('cruise')).toBe(PHASE_ORDER.indexOf('cruise'))
  })
})
