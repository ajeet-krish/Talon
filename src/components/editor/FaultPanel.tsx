/**
 * Fault panel (FlightLab Phase 8c): the authored fault list plus a timeline
 * strip that maps each fault onto the run it will interrupt. A fault fires
 * when the clock reaches it, so the last accepted fire time is the run's own
 * last step clock (`faultWindowEndS`, the same rule the validator and the
 * Rust runner use). When that window is empty the panel says so and refuses
 * to add faults rather than letting the validator reject them later.
 *
 * Faults are edited as whole lists through `faults/set`, because the model
 * treats the list as one authored value; every field change is one command.
 */

import { LIMITS } from '@/types/generated/talon_core'
import type { DivergenceMode, FaultKind, FaultSpec } from '@/types/generated/talon_core'
import type { EditorCommand, MissionConfig } from '@/utils/editor'
import { faultWindowEndS } from '@/utils/editor'
import { NumberField } from './NumberField'
import { Section } from './Section'
import { faultLabel } from './chart'

export interface FaultPanelProps {
  /** The mission being edited. */
  config: MissionConfig
  /** Apply one edit. */
  dispatch: (cmd: EditorCommand) => void
}

const FAULT_KINDS: { kind: FaultKind; label: string }[] = [
  { kind: 'gps_outage', label: 'GPS outage' },
  { kind: 'ekf_divergence', label: 'EKF divergence' },
  { kind: 'battery_soc', label: 'Battery SOC' },
  { kind: 'motor_fault', label: 'Motor fault' },
  { kind: 'emergency', label: 'Emergency' },
  { kind: 'rtl', label: 'Return to launch' },
]

const DIVERGENCE_MODES: { mode: DivergenceMode; label: string }[] = [
  { mode: 'corrupt_bias', label: 'Corrupt bias' },
  { mode: 'suppress_updates', label: 'Suppress updates' },
  { mode: 'adversarial_gps', label: 'Adversarial GPS' },
]

/** Shared styling for the selects in this panel. */
const SELECT =
  'w-full border border-grid bg-panel px-2 py-1 font-mono text-[11px] text-ink transition-colors focus-visible:border-signal'

/** A fresh fault of a kind, at a clock, with that kind's default parameter. */
function newFault(kind: FaultKind, atS: number): FaultSpec {
  switch (kind) {
    case 'gps_outage':
      return { kind, at_s: atS, duration_s: 4 }
    case 'ekf_divergence':
      return { kind, at_s: atS, mode: 'corrupt_bias' }
    case 'battery_soc':
      return { kind, at_s: atS, soc: 0.4 }
    default:
      return { kind, at_s: atS }
  }
}

/** Swap a fault's kind, keeping its clock and giving the new kind a default. */
function withKind(fault: FaultSpec, kind: FaultKind): FaultSpec {
  return newFault(kind, fault.at_s)
}

/** The timeline strip: fault ticks across the run, window end marked. */
function FaultTimeline({
  config,
  windowEnd,
}: {
  config: MissionConfig
  windowEnd: number | null
}) {
  const duration = config.duration_s
  const pct = (t: number): number => Math.min(100, Math.max(0, (t / duration) * 100))
  return (
    <div className="mb-2">
      <div
        className="relative h-6 border border-grid bg-panel"
        role="img"
        aria-label={`Fault timeline over a ${duration} second run${windowEnd === null ? '' : `, fault window ends at ${windowEnd} seconds`}`}
      >
        {config.faults.map((fault, i) => (
          <div
            key={i}
            className="absolute inset-y-0 w-0.5 bg-warn"
            style={{ left: `${pct(fault.at_s)}%` }}
            title={faultLabel(fault)}
          />
        ))}
        {windowEnd !== null && windowEnd < duration && (
          <div
            className="absolute inset-y-0 border-l border-dashed border-ink-faint"
            style={{ left: `${pct(windowEnd)}%` }}
          />
        )}
      </div>
      <div className="mt-0.5 flex justify-between font-mono text-[9px] tabular-nums text-ink-faint">
        <span>0 s</span>
        {windowEnd !== null && windowEnd < duration && <span>window ends {windowEnd} s</span>}
        <span>{duration} s</span>
      </div>
    </div>
  )
}

export function FaultPanel({ config, dispatch }: FaultPanelProps) {
  const faults = config.faults
  const windowEnd = faultWindowEndS(config.duration_s, LIMITS.dt)
  const atCap = faults.length >= LIMITS.max_faults

  /** Replace the whole list: every fault edit is one command. */
  function setFaults(next: FaultSpec[]): void {
    dispatch({ type: 'faults/set', faults: next })
  }

  function addFault(): void {
    if (windowEnd === null || atCap) return
    const atS = Math.min(10, windowEnd)
    setFaults([...faults, newFault('gps_outage', atS)])
  }

  return (
    <Section title="Faults" aside={`${faults.length} / ${LIMITS.max_faults}`}>
      <FaultTimeline config={config} windowEnd={windowEnd} />

      {windowEnd === null ? (
        <p className="text-[11px] leading-relaxed text-ink-faint">
          This mission is too short for faults: a run of {config.duration_s} s has fewer than
          two steps, so no fault could ever fire. Raise the duration to add one.
        </p>
      ) : (
        <>
          {faults.length === 0 ? (
            <p className="pb-1 text-[11px] leading-relaxed text-ink-faint">
              No faults. The run flies the plan as authored.
            </p>
          ) : (
            <ul className="flex flex-col gap-2">
              {faults.map((fault, i) => (
                <li key={i} className="border-l-2 border-grid bg-panel-2/40 px-2 py-1.5">
                  <div className="flex items-center gap-2">
                    <span className="font-mono text-[11px] font-semibold tracking-[0.1em] text-ink-dim">
                      F{i + 1}
                    </span>
                    <button
                      type="button"
                      aria-label={`Remove fault ${i + 1}`}
                      onClick={() => setFaults(faults.filter((_, j) => j !== i))}
                      className="ml-auto h-6 w-6 border border-grid font-mono text-[10px] leading-none text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal"
                    >
                      X
                    </button>
                  </div>
                  <div className="mt-1.5 grid grid-cols-2 gap-1.5">
                    <div className="flex flex-col gap-1">
                      <label
                        htmlFor={`fault-kind-${i}`}
                        className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint"
                      >
                        Kind
                      </label>
                      <select
                        id={`fault-kind-${i}`}
                        value={fault.kind}
                        onChange={(e) => {
                          const kind = e.target.value as FaultKind
                          const next = [...faults]
                          next[i] = withKind(fault, kind)
                          setFaults(next)
                        }}
                        className={SELECT}
                      >
                        {FAULT_KINDS.map((entry) => (
                          <option key={entry.kind} value={entry.kind}>
                            {entry.label}
                          </option>
                        ))}
                      </select>
                    </div>
                    <NumberField
                      label={`At s (0 to ${windowEnd})`}
                      value={fault.at_s}
                      suffix="s"
                      min={0}
                      max={windowEnd}
                      step={0.5}
                      onCommit={(at_s) => {
                        const next = [...faults]
                        next[i] = { ...fault, at_s }
                        setFaults(next)
                      }}
                    />
                  </div>

                  {fault.kind === 'gps_outage' && (
                    <div className="mt-1.5">
                      <NumberField
                        label={`Outage length s (max ${LIMITS.max_duration_s})`}
                        value={fault.duration_s}
                        suffix="s"
                        min={0.005}
                        max={LIMITS.max_duration_s}
                        step={0.5}
                        onCommit={(duration_s) => {
                          const next = [...faults]
                          next[i] = { ...fault, duration_s }
                          setFaults(next)
                        }}
                      />
                    </div>
                  )}

                  {fault.kind === 'ekf_divergence' && (
                    <div className="mt-1.5 flex flex-col gap-1">
                      <label
                        htmlFor={`fault-mode-${i}`}
                        className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint"
                      >
                        Corruption
                      </label>
                      <select
                        id={`fault-mode-${i}`}
                        value={fault.mode}
                        onChange={(e) => {
                          const mode = e.target.value as DivergenceMode
                          const next = [...faults]
                          next[i] = { ...fault, mode }
                          setFaults(next)
                        }}
                        className={SELECT}
                      >
                        {DIVERGENCE_MODES.map((entry) => (
                          <option key={entry.mode} value={entry.mode}>
                            {entry.label}
                          </option>
                        ))}
                      </select>
                    </div>
                  )}

                  {fault.kind === 'battery_soc' && (
                    <div className="mt-1.5">
                      <NumberField
                        label="Forced SOC (0 to 1)"
                        value={fault.soc}
                        suffix="frac"
                        min={0}
                        max={LIMITS.max_battery_soc}
                        step={0.05}
                        onCommit={(soc) => {
                          const next = [...faults]
                          next[i] = { ...fault, soc }
                          setFaults(next)
                        }}
                      />
                    </div>
                  )}

                  <p className="mt-1.5 font-mono text-[9px] leading-tight text-ink-faint">
                    {faultLabel(fault).toUpperCase()}
                  </p>
                </li>
              ))}
            </ul>
          )}

          <button
            type="button"
            onClick={addFault}
            disabled={atCap}
            title={
              atCap
                ? `A mission may schedule at most ${LIMITS.max_faults} faults`
                : 'Add a fault at 10 s (or the window end, if sooner)'
            }
            className="mt-2 w-full border border-grid px-2 py-1.5 font-mono text-[10px] tracking-[0.15em] text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-40"
          >
            ADD FAULT
          </button>
        </>
      )}
    </Section>
  )
}
