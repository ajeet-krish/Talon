/**
 * Validation readout (FlightLab Phase 8c), rendered in the mission view's
 * bottom bar. Errors come first because they are the ones blocking a run;
 * warnings follow because they are advisory. Each message carries a leading
 * label rather than an icon, so the severity reads in a screen reader and in
 * monochrome. A clean plan gets one quiet line instead of a scoreboard.
 */

export interface MissionValidationProps {
  /** Blocking messages from `validateScenarioConfig`. */
  errors: string[]
  /** Non-blocking advisories from `missionWarnings`. */
  warnings: string[]
}

/** Most messages the bar shows before it starts summarising. */
const MAX_SHOWN = 3

export function MissionValidation({ errors, warnings }: MissionValidationProps) {
  if (errors.length === 0 && warnings.length === 0) {
    return (
      <p className="font-mono text-[10px] tracking-[0.15em] text-ink-faint">READY TO FLY</p>
    )
  }

  const hidden = errors.length + warnings.length - MAX_SHOWN
  const rows: { severity: 'error' | 'warning'; text: string }[] = [
    ...errors.map((text) => ({ severity: 'error' as const, text })),
    ...warnings.map((text) => ({ severity: 'warning' as const, text })),
  ].slice(0, MAX_SHOWN)

  return (
    <div className="flex flex-col gap-0.5">
      <p className="font-mono text-[9px] tabular-nums tracking-[0.15em] text-ink-dim">
        {errors.length > 0 && (
          <span className="text-danger">{errors.length} ERROR{errors.length === 1 ? '' : 'S'}</span>
        )}
        {errors.length > 0 && warnings.length > 0 && (
          <span className="text-ink-faint"> / </span>
        )}
        {warnings.length > 0 && (
          <span className="text-warn">
            {warnings.length} WARNING{warnings.length === 1 ? '' : 'S'}
          </span>
        )}
      </p>
      {rows.map((row, i) => (
        <p key={i} className="flex gap-1.5 text-[10px] leading-relaxed">
          <span
            className={`shrink-0 font-mono text-[9px] font-semibold tracking-[0.1em] ${
              row.severity === 'error' ? 'text-danger' : 'text-warn'
            }`}
          >
            {row.severity === 'error' ? 'ERROR' : 'WARN'}
          </span>
          <span className="min-w-0 text-ink-dim">{row.text}</span>
        </p>
      ))}
      {hidden > 0 && (
        <p className="font-mono text-[9px] text-ink-faint">+{hidden} MORE</p>
      )}
    </div>
  )
}
