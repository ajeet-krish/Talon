/**
 * The live instrument rail: the numbers, the health, and what the autopilot is
 * doing, next to the scene.
 *
 * This is the part that makes the section a ground station rather than an
 * animation. Everything on it comes from the running native session: the state
 * the mission manager is in, the attitude the controller is holding, how far the
 * estimator is from the truth, and how the sensor pipeline is being treated.
 *
 * It re-renders at about 10 Hz rather than per sample. The scene is animated from
 * the recorder directly, so the text never needs to keep up with 100 Hz, and
 * decoupling them keeps the frame budget for rendering.
 */

import type { LiveConfig } from '@/live/types'
import { DT_CONTROL, HOVER_THRUST } from '@/live/constants'
import { PHASE_COLORS } from '@/utils/colors'
import { CLEARANCE_CAUTION_M, CLEARANCE_COLORS, clearanceBand } from '@/utils/clearance'
import type { LiveBuffer } from '@/live/stream'
import type { LiveReadout } from '@/live/readout'
import { LiveSpark } from './LiveSpark'
import { PHASE_CAPTIONS, READOUT_NOTES } from './captions'

/** Seconds of history each strip chart shows. */
const SPARK_WINDOW_S = 20

export type RunState = 'idle' | 'running' | 'paused' | 'done'

const PHASE_LABELS: Record<string, string> = {
  idle: 'IDLE',
  arm: 'ARM',
  takeoff: 'TAKEOFF',
  cruise: 'CRUISE',
  waypoint: 'WAYPOINT',
  rtl: 'RTL',
  land: 'LAND',
  emergency: 'EMERGENCY',
}

function fmt(value: number, digits = 1): string {
  return Number.isFinite(value) ? value.toFixed(digits) : 'n/a'
}

/** Degrees from radians, for the attitude readouts. */
function deg(rad: number): number {
  return (rad * 180) / Math.PI
}

function Metric({
  label,
  value,
  unit,
  tone = 'ink',
}: {
  label: string
  value: string
  unit?: string
  tone?: 'ink' | 'signal' | 'dim'
}) {
  const color = tone === 'signal' ? 'text-signal' : tone === 'dim' ? 'text-ink-dim' : 'text-ink'
  return (
    <div className="bg-panel px-2 py-1.5">
      <div className="font-mono text-[9px] tracking-[0.15em] text-ink-faint">{label}</div>
      <div className={`mt-0.5 font-mono text-[12px] tabular-nums ${color}`}>
        {value}
        {unit !== undefined && <span className="ml-0.5 text-[9px] text-ink-faint">{unit}</span>}
      </div>
    </div>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <section className="border border-grid">
      <div className="border-b border-grid px-2 py-1 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
        {title}
      </div>
      {children}
    </section>
  )
}

export interface TelemetryRailProps {
  cfg: LiveConfig
  buffer: LiveBuffer
  readout: LiveReadout | null
  runState: RunState
}

export function TelemetryRail({ cfg, buffer, readout, runState }: TelemetryRailProps) {
  const windowSamples = Math.max(2, Math.round(SPARK_WINDOW_S / DT_CONTROL))
  const to = buffer.count
  const from = Math.max(0, to - windowSamples)

  const phase = readout?.phase ?? 'idle'
  const caption = PHASE_CAPTIONS[phase]
  const progress = readout?.progress ?? 0
  const time = readout?.time ?? 0

  const hasObstacles = cfg.obstacles.length > 0
  const clearance = readout?.clearance ?? Infinity
  const band = clearanceBand(clearance, CLEARANCE_CAUTION_M)
  const clearanceHex =
    band === 'breach'
      ? CLEARANCE_COLORS.breach
      : band === 'caution'
        ? CLEARANCE_COLORS.caution
        : CLEARANCE_COLORS.clear

  const thrustPct = readout === null ? 0 : (readout.thrust / HOVER_THRUST) * 100

  return (
    <div className="flex h-full min-h-0 flex-col gap-2 overflow-y-auto pr-0.5">
      <Section title="FLIGHT STATE">
        <div className="flex items-center justify-between gap-2 px-2 py-2">
          <span className="flex items-center gap-2">
            <span
              className="inline-block h-2.5 w-2.5"
              style={{ backgroundColor: PHASE_COLORS[phase] }}
            />
            <span className="font-mono text-[13px] font-semibold tracking-[0.12em] text-ink">
              {PHASE_LABELS[phase] ?? phase.toUpperCase()}
            </span>
          </span>
          <span className="font-mono text-[11px] tabular-nums text-ink-dim">
            T+ {fmt(time, 2)} <span className="text-ink-faint">/ {fmt(cfg.duration_s, 0)} s</span>
          </span>
        </div>
        <div className="px-2 pb-2">
          <div className="h-1 w-full bg-panel-3">
            <div
              className="h-1 bg-signal"
              style={{ width: `${Math.round(Math.min(Math.max(progress, 0), 1) * 100)}%` }}
            />
          </div>
          <div className="mt-1 flex justify-between font-mono text-[9px] text-ink-faint">
            <span>
              {runState === 'done'
                ? 'RUN COMPLETE'
                : runState === 'paused'
                  ? 'PAUSED'
                  : runState === 'running'
                    ? 'PACED AGAINST WALL CLOCK'
                    : 'READY'}
            </span>
            <span className="tabular-nums">{Math.round(progress * 100)}%</span>
          </div>
        </div>
      </Section>

      <Section title="VEHICLE STATE">
        <dl className="grid grid-cols-2 gap-px bg-grid">
          <Metric label="ALTITUDE" value={fmt(readout?.altitude ?? 0)} unit="m" tone="signal" />
          <Metric label="CLIMB" value={fmt(readout?.verticalSpeed ?? 0)} unit="m/s" />
          <Metric label="GROUND SPD" value={fmt(readout?.groundSpeed ?? 0)} unit="m/s" />
          <Metric label="THRUST" value={fmt(thrustPct, 0)} unit="% HOV" />
          <Metric label="ROLL" value={fmt(deg(readout?.roll ?? 0))} unit="deg" />
          <Metric label="PITCH" value={fmt(deg(readout?.pitch ?? 0))} unit="deg" />
          <Metric label="HEADING" value={fmt(deg(readout?.heading ?? 0), 0)} unit="deg" />
          <Metric label="BATTERY" value={fmt((readout?.battery ?? 0) * 100, 0)} unit="%" />
        </dl>
      </Section>

      <Section title="ESTIMATOR AND SENSORS">
        <dl className="grid grid-cols-2 gap-px bg-grid">
          <Metric
            label="EST ERROR"
            value={fmt(readout?.estimateError ?? 0, 2)}
            unit="m"
            tone="signal"
          />
          <Metric label="WIND" value={fmt(readout?.windSpeed ?? 0)} unit="m/s" />
          <Metric
            label="GPS FIXES"
            value={
              readout === null
                ? '0 / 0'
                : `${readout.gps[0]} / ${readout.gps[0] + readout.gps[1]}`
            }
            tone="dim"
          />
          <Metric
            label="EKF"
            value={
              readout === null
                ? 'n/a'
                : readout.ekf[1]
                  ? 'DIVERGED'
                  : readout.ekf[0]
                    ? 'CONVERGED'
                    : 'TRACKING'
            }
            tone={readout?.ekf[1] === true ? 'dim' : 'ink'}
          />
        </dl>
        <p className="border-t border-grid px-2 py-1.5 text-[10px] leading-snug text-ink-faint">
          {READOUT_NOTES.estimate}
        </p>
        <p className="border-t border-grid px-2 py-1.5 text-[10px] leading-snug text-ink-faint">
          {READOUT_NOTES.gps}
        </p>
      </Section>

      {hasObstacles && (
        <Section title="KEEP-OUT CLEARANCE">
          <div className="px-2 py-2">
            <div className="flex items-baseline justify-between">
              <span
                className="font-mono text-[16px] tabular-nums"
                style={{ color: clearanceHex }}
              >
                {Number.isFinite(clearance) ? `${clearance.toFixed(2)} m` : 'n/a'}
              </span>
              <span className="font-mono text-[9px] tracking-[0.15em]" style={{ color: clearanceHex }}>
                {band.toUpperCase()}
              </span>
            </div>
            <p className="mt-1 text-[10px] leading-snug text-ink-faint">
              {READOUT_NOTES.clearance} The shell tint in the scene uses the same bands.
            </p>
          </div>
        </Section>
      )}

      <Section title={`LAST ${SPARK_WINDOW_S} s`}>
        <div className="flex flex-col gap-1 p-1">
          <LiveSpark
            values={buffer.series.altitude}
            from={from}
            to={to}
            min={0}
            max={Math.max(cfg.takeoff_altitude * 1.4, 5)}
            color="#26b8d9"
            label="ALTITUDE"
            value={fmt(readout?.altitude ?? 0)}
            unit="m"
          />
          <LiveSpark
            values={buffer.series.groundSpeed}
            from={from}
            to={to}
            min={0}
            max={12}
            color="#56B4E9"
            label="GROUND SPEED"
            value={fmt(readout?.groundSpeed ?? 0)}
            unit="m/s"
          />
          <LiveSpark
            values={buffer.series.estimateError}
            from={from}
            to={to}
            min={0}
            max={3}
            color="#009E73"
            label="ESTIMATE ERROR"
            value={fmt(readout?.estimateError ?? 0, 2)}
            unit="m"
          />
        </div>
      </Section>

      <Section title="WHAT THE AUTOPILOT IS DOING">
        <div className="px-2 py-2">
          <div className="font-mono text-[10px] tracking-[0.18em] text-signal">
            {caption.title}
          </div>
          <p className="mt-1 text-[10px] leading-snug text-ink-dim">{caption.body}</p>
        </div>
      </Section>

      <Section title="PHASE TIMELINE">
        <ul className="flex flex-col">
          {buffer.phases.map(([t, label], i) => {
            const current =
              readout !== null &&
              (i === buffer.phases.length - 1 || (buffer.phases[i + 1]?.[0] ?? Infinity) > time)
            return (
              <li
                key={`${label}-${i}`}
                className={`flex items-center justify-between px-2 py-1 font-mono text-[10px] ${
                  current ? 'bg-panel-3' : 'bg-panel'
                }`}
              >
                <span className="flex items-center gap-2">
                  <span
                    className="inline-block h-2 w-2"
                    style={{ backgroundColor: PHASE_COLORS[label] ?? '#999999' }}
                  />
                  <span className={current ? 'text-ink' : 'text-ink-dim'}>
                    {PHASE_LABELS[label] ?? label.toUpperCase()}
                  </span>
                </span>
                <span className="tabular-nums text-ink-faint">{t.toFixed(2)} s</span>
              </li>
            )
          })}
          {buffer.phases.length === 0 && (
            <li className="bg-panel px-2 py-2 font-mono text-[10px] text-ink-faint">
              AWAITING FIRST SAMPLE
            </li>
          )}
        </ul>
      </Section>
    </div>
  )
}
