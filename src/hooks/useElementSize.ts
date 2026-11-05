import { useLayoutEffect, useRef, useState } from 'react'

/** An element's box in CSS pixels. */
export interface ElementSize {
  width: number
  height: number
}

/**
 * Measure an element's box and keep it in sync with layout changes, the way
 * `useElementWidth` does for the pixel-space telemetry charts. The mission
 * chart needs the height as well as the width: its `viewBox` tracks the
 * element's aspect ratio, so the drawn frame and the visible area are the
 * same rectangle at every zoom and pan, and the grid can be drawn over
 * exactly what is on screen. Measured in a layout effect so the first
 * painted frame already carries the true aspect. Size is 0 x 0 until that
 * effect runs; callers read it as "not measured yet" rather than a
 * degenerate box.
 */
export function useElementSize<T extends HTMLElement>() {
  const ref = useRef<T | null>(null)
  const [size, setSize] = useState<ElementSize>({ width: 0, height: 0 })

  useLayoutEffect(() => {
    const el = ref.current
    if (el === null) return
    const update = () => setSize({ width: el.clientWidth, height: el.clientHeight })
    update()
    if (typeof ResizeObserver === 'undefined') return
    const observer = new ResizeObserver(update)
    observer.observe(el)
    return () => observer.disconnect()
  }, [])

  return [ref, size] as const
}
