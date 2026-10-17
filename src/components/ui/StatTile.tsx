/**
 * One labelled numeric readout in the instrument strips (telemetry header,
 * metrics summary). Mono and tabular so columns line up and digits do not
 * jitter when values change.
 */
export function StatTile({
  label,
  value,
  unit,
}: {
  label: string
  value: string
  unit?: string
}) {
  return (
    <div className="flex min-w-0 flex-col gap-0.5 border-l-2 border-grid-2 pl-2">
      <span className="font-mono text-[9px] tracking-widest text-ink-faint">
        {label}
      </span>
      <span className="font-mono text-sm leading-none tabular-nums text-ink">
        {value}
        {unit && <span className="ml-1 text-[10px] text-ink-faint">{unit}</span>}
      </span>
    </div>
  )
}
