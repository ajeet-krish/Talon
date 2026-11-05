/**
 * WebGL probe tests (FlightLab Phase 8d-D): the one question the 3D course
 * preview asks before mounting a canvas. Runs against a stubbed document,
 * so the path a browser without WebGL 2 takes is covered without a GPU, and
 * the probe's promise (it hands its context straight back) is pinned.
 */

import { afterEach, describe, expect, it, vi } from 'vitest'
import { supportsWebGL } from './webgl'

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('supportsWebGL', () => {
  it('reads as false where there is no document at all', () => {
    // The node/vitest world: no DOM, no context, and no crash.
    expect(supportsWebGL()).toBe(false)
  })

  it('reads as false when the context cannot be created', () => {
    vi.stubGlobal('document', { createElement: () => ({ getContext: () => null }) })
    expect(supportsWebGL()).toBe(false)
  })

  it('reads a throwing context creation as no WebGL', () => {
    vi.stubGlobal('document', {
      createElement: () => ({
        getContext: () => {
          throw new Error('context blocked')
        },
      }),
    })
    expect(supportsWebGL()).toBe(false)
  })

  it('is true on a WebGL 2 context and hands it straight back', () => {
    const loseContext = vi.fn()
    vi.stubGlobal('document', {
      createElement: () => ({
        getContext: (id: string) =>
          id === 'webgl2' ? { getExtension: () => ({ loseContext }) } : null,
      }),
    })
    expect(supportsWebGL()).toBe(true)
    // The probe must not cost one of the browser's limited context slots.
    expect(loseContext).toHaveBeenCalledTimes(1)
  })

  it('does not probe WebGL 1, which three.js no longer renders', () => {
    const requested: string[] = []
    vi.stubGlobal('document', {
      createElement: () => ({
        getContext: (id: string) => {
          requested.push(id)
          return null
        },
      }),
    })
    expect(supportsWebGL()).toBe(false)
    expect(requested).toEqual(['webgl2'])
  })
})
