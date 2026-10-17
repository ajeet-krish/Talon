interface StatusPanelProps {
  title: string
  detail?: string
  tone?: 'dim' | 'error'
}

/**
 * Small instrument-chrome panel for empty, loading, and error states shared
 * by every view. Layout-agnostic: callers decide whether to center it over
 * the scene or place it in normal flow.
 */
export function StatusPanel({ title, detail, tone = 'dim' }: StatusPanelProps) {
  return (
    <div className="rounded-[3px] border border-grid bg-panel/85 px-4 py-3 text-center">
      <div
        className={`font-mono text-[11px] tracking-wider ${
          tone === 'error' ? 'text-[#D55E00]' : 'text-ink-dim'
        }`}
      >
        {title}
      </div>
      {detail && (
        <div className="mt-1 font-mono text-[10px] text-ink-faint">{detail}</div>
      )}
    </div>
  )
}
