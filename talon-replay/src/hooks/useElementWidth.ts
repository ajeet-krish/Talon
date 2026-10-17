import { useEffect, useRef, useState } from 'react'

/**
 * Measure an element's content width and keep it in sync with layout changes.
 * The telemetry charts draw in pixel coordinates (so strokes and point marks
 * stay undistorted) which means they need a real width rather than a scaled
 * viewBox. Returns a ref to attach and the current width in CSS pixels.
 *
 * Width is 0 until the first effect runs, so callers render a placeholder for
 * one frame instead of a degenerate path.
 */
export function useElementWidth<T extends HTMLElement>() {
  const ref = useRef<T | null>(null)
  const [width, setWidth] = useState(0)

  useEffect(() => {
    const el = ref.current
    if (!el) return
    const update = () => setWidth(el.clientWidth)
    update()
    if (typeof ResizeObserver === 'undefined') return
    const observer = new ResizeObserver(update)
    observer.observe(el)
    return () => observer.disconnect()
  }, [])

  return [ref, width] as const
}
