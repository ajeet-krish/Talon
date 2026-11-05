/**
 * WebGL availability probe (FlightLab Phase 8d-D). One guarded question the
 * mission preview asks before it mounts a canvas: can this environment make
 * a WebGL 2 context at all? The check runs in the browser only, releases
 * the context it creates so the probe itself cannot consume one of the
 * browser's limited contexts, and degrades to "no" wherever there is no
 * DOM (node/vitest), so the fallback path is testable without a GPU.
 */

/**
 * True when a WebGL 2 context can be created here. Three.js r163+ renders
 * WebGL 2 only, so WebGL 1 support is deliberately not probed. Any throw
 * (privacy modes, disabled contexts) reads as "no": the preview falls back
 * to its quiet message rather than crashing the page.
 */
export function supportsWebGL(): boolean {
  try {
    if (typeof document === 'undefined') return false
    const probe = document.createElement('canvas')
    const gl = probe.getContext('webgl2')
    if (gl === null) return false
    // Hand the context back immediately: the probe must not cost a slot.
    gl.getExtension('WEBGL_lose_context')?.loseContext()
    return true
  } catch {
    return false
  }
}
