import { useFlightStore } from '@/store/useFlightStore'
import type { ViewId } from '@/store/useFlightStore'

const VIEWS: { id: ViewId; label: string }[] = [
  { id: 'replay', label: 'REPLAY' },
  { id: 'telemetry', label: 'TELEMETRY' },
  { id: 'metrics', label: 'METRICS' },
  { id: 'videos', label: 'VIDEOS' },
]

/**
 * Section switcher for the tool. Rendered as instrument tabs in the header;
 * the active section gets the signal-cyan underline.
 */
export function ViewTabs() {
  const activeView = useFlightStore((s) => s.activeView)
  const setView = useFlightStore((s) => s.setView)

  return (
    <nav aria-label="Tool sections" className="flex items-stretch gap-1">
      {VIEWS.map((view) => {
        const active = view.id === activeView
        return (
          <button
            key={view.id}
            type="button"
            onClick={() => setView(view.id)}
            aria-current={active ? 'page' : undefined}
            className={`border-b-2 px-2 py-1 font-mono text-[10px] tracking-[0.2em] transition-colors ${
              active
                ? 'border-signal text-signal'
                : 'border-transparent text-ink-faint hover:text-ink'
            }`}
          >
            {view.label}
          </button>
        )
      })}
    </nav>
  )
}
