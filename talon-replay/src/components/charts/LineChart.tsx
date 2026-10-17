import { useMemo } from 'react'
import { useElementWidth } from '@/hooks/useElementWidth'
import { useFlightStore } from '@/store/useFlightStore'
import {
  buildPath,
  decimateMinMax,
  valueDomain,
  xScale,
  yScale,
} from '@/utils/chart'
import { formatMetric } from '@/utils/format'
import { sampleScalar } from '@/utils/interpolation'

const PLOT_HEIGHT = 104
/** Reduction budget per series: enough to keep spikes, cheap to render. */
const MAX_POINTS = 700
const GRID = '#1a2333'
const FRAME = '#243044'

interface LineChartProps {
  /** Series label, shown uppercase in the caption. */
  label: string
  /** Engineering unit appended to the live and bound readouts. */
  unit: string
  /** Stroke color (chart palette, distinct from the phase palette). */
  color: string
  times: number[]
  values: number[]
  /** Shared time domain across the whole chart stack. */
  t0: number
  t1: number
}

/**
 * One telemetry time-series chart: a framed plot with gridlines, a min/max
 * decimated line (spikes preserved), a live playhead cursor, and a live value
 * readout. The static geometry is memoized; only the cursor and the live
 * value subscribe to the playback clock, so a playing scenario does not
 * rebuild the paths every frame.
 */
export function LineChart({
  label,
  unit,
  color,
  times,
  values,
  t0,
  t1,
}: LineChartProps) {
  const [ref, width] = useElementWidth<HTMLDivElement>()

  const { path, area, lo, hi } = useMemo(() => {
    const reduced = decimateMinMax(times, values, MAX_POINTS)
    const [lo, hi] = valueDomain(reduced.values)
    const w = width || 1
    const line = buildPath(reduced.times, reduced.values, t0, t1, lo, hi, w, PLOT_HEIGHT)
    const fill = line ? `${line} L${w} ${PLOT_HEIGHT} L0 ${PLOT_HEIGHT} Z` : ''
    return { path: line, area: fill, lo, hi }
  }, [times, values, t0, t1, width])

  return (
    <figure className="flex min-w-0 flex-col gap-1">
      <figcaption className="flex items-baseline justify-between gap-2">
        <span className="truncate font-mono text-[10px] uppercase tracking-wider text-ink-dim">
          {label}
        </span>
        <span className="shrink-0 font-mono text-[10px] tabular-nums text-ink-faint">
          <LiveValue times={times} values={values} color={color} />
          <span className="ml-1">{unit}</span>
        </span>
      </figcaption>
      <div ref={ref} className="relative" style={{ height: PLOT_HEIGHT }}>
        {width > 0 && (
          <svg width={width} height={PLOT_HEIGHT} className="block">
            <rect
              x={0.5}
              y={0.5}
              width={Math.max(width - 1, 0)}
              height={PLOT_HEIGHT - 1}
              fill="none"
              stroke={FRAME}
            />
            {[0.25, 0.5, 0.75].map((f) => (
              <line
                key={f}
                x1={0}
                x2={width}
                y1={PLOT_HEIGHT * f}
                y2={PLOT_HEIGHT * f}
                stroke={GRID}
              />
            ))}
            {area && <path d={area} fill={color} opacity={0.08} />}
            {path && (
              <path
                d={path}
                fill="none"
                stroke={color}
                strokeWidth={1.25}
                strokeLinejoin="round"
                strokeLinecap="round"
              />
            )}
            <ChartCursor
              times={times}
              values={values}
              t0={t0}
              t1={t1}
              lo={lo}
              hi={hi}
              width={width}
              color={color}
            />
          </svg>
        )}
        <span className="pointer-events-none absolute right-1.5 top-1 font-mono text-[9px] tabular-nums text-ink-faint">
          {formatMetric(hi)}
        </span>
        <span className="pointer-events-none absolute bottom-1 right-1.5 font-mono text-[9px] tabular-nums text-ink-faint">
          {formatMetric(lo)}
        </span>
      </div>
      <div className="flex justify-between font-mono text-[9px] tabular-nums text-ink-faint">
        <span>0</span>
        <span>{t0.toFixed(0)} S</span>
        <span>{t1.toFixed(0)} S</span>
      </div>
    </figure>
  )
}

/** Vertical playhead line plus a dot riding the curve, synced to the store. */
function ChartCursor({
  times,
  values,
  t0,
  t1,
  lo,
  hi,
  width,
  color,
}: {
  times: number[]
  values: number[]
  t0: number
  t1: number
  lo: number
  hi: number
  width: number
  color: string
}) {
  const currentTime = useFlightStore((s) => s.currentTime)
  const x = xScale(currentTime, t0, t1, width)
  const y = yScale(sampleScalar(currentTime, times, values), lo, hi, PLOT_HEIGHT)
  return (
    <g>
      <line x1={x} x2={x} y1={0} y2={PLOT_HEIGHT} stroke="#26b8d9" opacity={0.55} />
      <circle cx={x} cy={y} r={2.5} fill={color} stroke="#0b1020" strokeWidth={1} />
    </g>
  )
}

/** Current sample value at the playhead, in the series color. */
function LiveValue({
  times,
  values,
  color,
}: {
  times: number[]
  values: number[]
  color: string
}) {
  const currentTime = useFlightStore((s) => s.currentTime)
  return (
    <span style={{ color }}>{formatMetric(sampleScalar(currentTime, times, values))}</span>
  )
}
