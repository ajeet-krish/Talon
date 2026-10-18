/**
 * Trailing-window strip chart for the live rail.
 *
 * The telemetry view's LineChart is bound to the replay clock (its cursor and
 * live value come from the store), so it can not be reused while a run is
 * actually in progress. This is the small version of the same idea: a polyline
 * over the last stretch of samples, redrawn as the rail re-renders.
 *
 * Values are drawn straight from the recorder's `Float32Array`, so no series is
 * copied to build a chart. The window is subsampled to a bounded number of
 * points, which keeps the path short regardless of how long the run has been
 * going and avoids a polyline with thousands of vertices for a 300 pixel strip.
 */

const SAMPLES_PER_CHART = 90

export interface LiveSparkProps {
  /** Full series, as recorded. */
  values: Float32Array
  /** First sample of the visible window, inclusive. */
  from: number
  /** One past the last sample of the visible window. */
  to: number
  /** Axis limits. Pass equal values to have them expanded automatically. */
  min: number
  max: number
  color: string
  /** Left-hand label, for example ALT. */
  label: string
  /** Formatted current value. */
  value: string
  /** Unit, printed after the value. */
  unit: string
}

/** Build the polyline points for a window, in a 0..100 by 0..100 box. */
function windowPath(
  values: Float32Array,
  from: number,
  to: number,
  min: number,
  max: number,
): { points: string; lo: number; hi: number } {
  const n = Math.max(to - from, 0)
  if (n === 0) return { points: '', lo: min, hi: max }

  // Expand the axis if the window left it, so a spike is never clipped flat.
  let lo = min
  let hi = max
  for (let i = from; i < to; i += 1) {
    const v = values[i]
    if (v === undefined) continue
    if (v < lo) lo = v
    if (v > hi) hi = v
  }
  const span = hi - lo > 1e-9 ? hi - lo : 1

  const step = Math.max(1, Math.ceil(n / SAMPLES_PER_CHART))
  const parts: string[] = []
  for (let i = from; i < to; i += step) {
    const v = values[i] ?? lo
    const x = ((i - from) / Math.max(n - 1, 1)) * 100
    const y = 100 - ((v - lo) / span) * 100
    parts.push(`${x.toFixed(2)},${y.toFixed(2)}`)
  }
  // Always land on the newest sample, so the head of the trace is exact.
  const lastV = values[to - 1] ?? lo
  parts.push(`100.00,${(100 - ((lastV - lo) / span) * 100).toFixed(2)}`)
  return { points: parts.join(' '), lo, hi }
}

export function LiveSpark({
  values,
  from,
  to,
  min,
  max,
  color,
  label,
  value,
  unit,
}: LiveSparkProps) {
  const { points, lo, hi } = windowPath(values, from, to, min, max)
  return (
    <div className="border border-grid bg-panel">
      <div className="flex items-baseline justify-between px-2 py-1">
        <span className="font-mono text-[9px] tracking-[0.18em] text-ink-faint">{label}</span>
        <span className="font-mono text-[10px] tabular-nums text-ink">
          {value}
          <span className="ml-0.5 text-ink-faint">{unit}</span>
        </span>
      </div>
      <div className="relative h-12 border-t border-grid">
        <svg
          viewBox="0 0 100 100"
          preserveAspectRatio="none"
          className="absolute inset-0 h-full w-full"
          aria-hidden="true"
        >
          {/* Midline so a flat trace still reads as a chart and not as empty. */}
          <line x1="0" y1="50" x2="100" y2="50" stroke="#1a2333" strokeWidth="0.6" />
          {points !== '' && (
            <polyline
              points={points}
              fill="none"
              stroke={color}
              strokeWidth="1.4"
              vectorEffect="non-scaling-stroke"
            />
          )}
        </svg>
        <span className="absolute bottom-0.5 left-1 font-mono text-[8px] tabular-nums text-ink-faint">
          {lo.toFixed(1)}
        </span>
        <span className="absolute left-1 top-0.5 font-mono text-[8px] tabular-nums text-ink-faint">
          {hi.toFixed(1)}
        </span>
      </div>
    </div>
  )
}
