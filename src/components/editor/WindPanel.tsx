/**
 * Wind panel (FlightLab Phase 8c): the steady wind, a compass readout of
 * where it comes from, Dryden turbulence presets, and the raw sigma triple
 * behind them. The compass states the source bearing (meteorological
 * convention, the way a wind sock reads), and the arrow points the way the
 * air travels. A preset is highlighted only while the sigma triple still
 * matches it exactly, so the highlight never lies after a manual tweak.
 */

import { LIMITS } from '@/types/generated/talon_core'
import type { Vec3 } from '@/types/flight'
import type { EditorCommand, MissionConfig } from '@/utils/editor'
import { NumberField } from './NumberField'
import { Section } from './Section'
import { compassPoint, windFromDirectionDeg, windSpeedMps } from './chart'

export interface WindPanelProps {
  /** The mission being edited. */
  config: MissionConfig
  /** Apply one edit. */
  dispatch: (cmd: EditorCommand) => void
}

/** One named turbulence preset, applied to the sigma triple. */
interface WindPreset {
  label: string
  sigma: [number, number, number]
  description: string
}

const PRESETS: WindPreset[] = [
  { label: 'CALM', sigma: [0, 0, 0], description: 'No turbulence.' },
  { label: 'LIGHT', sigma: [0.5, 0.5, 0.25], description: 'Light turbulence.' },
  { label: 'MODERATE', sigma: [1.5, 1.5, 0.75], description: 'Moderate turbulence.' },
  { label: 'STRONG', sigma: [3, 3, 1.5], description: 'Strong turbulence.' },
]

/** True when the triple still equals the preset, to light its button. */
function matchesPreset(sigma: Vec3, preset: WindPreset): boolean {
  return (
    Math.abs(sigma[0] - preset.sigma[0]) < 1e-9 &&
    Math.abs(sigma[1] - preset.sigma[1]) < 1e-9 &&
    Math.abs(sigma[2] - preset.sigma[2]) < 1e-9
  )
}

/**
 * The compass: a ring with cardinal ticks, an arrow along the direction the
 * wind travels, and the source bearing printed in the middle. Still air
 * shows a dash and CALM rather than a meaningless heading.
 */
function WindCompass({ north, east }: { north: number; east: number }) {
  const bearing = windFromDirectionDeg(north, east)
  const speed = windSpeedMps(north, east)
  // Chart space matches the mission chart: up is north, right is east.
  const angle = bearing === null ? 0 : bearing + 180
  const cardinal = (label: string, dx: number, dy: number) => (
    <text
      x={dx}
      y={dy}
      textAnchor="middle"
      dominantBaseline="middle"
      className="fill-ink-faint"
      fontSize={7}
    >
      {label}
    </text>
  )

  return (
    // The text readout beside the compass carries the same information, so
    // the graphic stays out of the accessibility tree.
    <svg viewBox="-40 -40 80 80" className="h-20 w-20 shrink-0" aria-hidden="true">
      <circle cx={0} cy={0} r={30} className="fill-panel-2 stroke-grid" strokeWidth={1.5} />
      {cardinal('N', 0, -33)}
      {cardinal('E', 34, 1)}
      {cardinal('S', 0, 34)}
      {cardinal('W', -34, 1)}
      {bearing === null ? (
        <line x1={-10} y1={0} x2={10} y2={0} className="stroke-ink-faint" strokeWidth={2} />
      ) : (
        <g transform={`rotate(${angle})`}>
          <line x1={0} y1={16} x2={0} y2={-20} className="stroke-signal" strokeWidth={2} />
          <path
            d="M -4 -14 L 0 -22 L 4 -14"
            className="fill-none stroke-signal"
            strokeWidth={2}
            strokeLinejoin="round"
          />
        </g>
      )}
      <text x={0} y={3} textAnchor="middle" dominantBaseline="middle" className="fill-ink" fontSize={10}>
        {bearing === null ? 'CALM' : `${speed.toFixed(1)}`}
      </text>
    </svg>
  )
}

export function WindPanel({ config, dispatch }: WindPanelProps) {
  const [north, east] = [config.wind_steady_ned[0], config.wind_steady_ned[1]]
  const sigma = config.wind_sigma
  const bearing = windFromDirectionDeg(north, east)
  const maxComponent = LIMITS.max_wind_component_mps

  return (
    <Section title="Wind">
      <div className="flex items-start gap-3">
        <WindCompass north={north} east={east} />
        <div className="min-w-0 flex-1">
          <p className="font-mono text-[11px] tabular-nums text-ink">
            {bearing === null
              ? 'CALM'
              : `${windSpeedMps(north, east).toFixed(1)} m/s FROM ${Math.round(bearing)}° (${compassPoint(bearing)})`}
          </p>
          <p className="mt-1 text-[10px] leading-relaxed text-ink-faint">
            Steady wind, given as the speed and direction the air travels; the bearing is
            where it comes from.
          </p>
        </div>
      </div>

      <div className="mt-2.5 grid grid-cols-2 gap-2">
        <NumberField
          label={`Steady north m/s (max ${maxComponent})`}
          value={north}
          suffix="m/s"
          min={-maxComponent}
          max={maxComponent}
          step={0.5}
          onCommit={(value) =>
            dispatch({ type: 'wind/steady', ned: [value, east, config.wind_steady_ned[2]] })
          }
        />
        <NumberField
          label={`Steady east m/s (max ${maxComponent})`}
          value={east}
          suffix="m/s"
          min={-maxComponent}
          max={maxComponent}
          step={0.5}
          onCommit={(value) =>
            dispatch({ type: 'wind/steady', ned: [north, value, config.wind_steady_ned[2]] })
          }
        />
      </div>

      <div className="mt-3">
        <span className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint">
          Turbulence
        </span>
        <div className="mt-1.5 flex flex-wrap gap-1">
          {PRESETS.map((preset) => {
            const active = matchesPreset(sigma, preset)
            return (
              <button
                key={preset.label}
                type="button"
                aria-pressed={active}
                title={preset.description}
                onClick={() => dispatch({ type: 'wind/sigma', ned: preset.sigma })}
                className={`border px-2 py-1 font-mono text-[10px] tracking-[0.1em] transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal ${
                  active
                    ? 'border-signal bg-panel-3 text-signal'
                    : 'border-grid text-ink-dim hover:bg-panel-3'
                }`}
              >
                {preset.label}
              </button>
            )
          })}
        </div>
        <div className="mt-2 grid grid-cols-3 gap-2">
          {([
            { axis: 'North', index: 0 },
            { axis: 'East', index: 1 },
            { axis: 'Down', index: 2 },
          ] as const).map(({ axis, index }) => (
            <NumberField
              key={axis}
              label={`${axis} σ (max ${maxComponent})`}
              value={sigma[index] ?? 0}
              suffix="m/s"
              min={0}
              max={maxComponent}
              step={0.25}
              onCommit={(value) => {
                const next: Vec3 = [sigma[0], sigma[1], sigma[2]]
                next[index] = value
                dispatch({ type: 'wind/sigma', ned: next })
              }}
            />
          ))}
        </div>
      </div>
    </Section>
  )
}
