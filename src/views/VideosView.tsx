import { useState } from 'react'
import { useFlightStore } from '@/store/useFlightStore'
import { RECORDINGS } from '@/data/videos'
import { StatusPanel } from '@/components/ui/StatusPanel'
import type { ScenarioId } from '@/types/flight'

/**
 * Videos screen: the recorded Matplotlib flights (docs/videos), the
 * publication half of the tool. The MP4s are staged into public/videos by
 * scripts/copy_data.sh. A recording that has a matching scenario can jump
 * straight into the interactive replay for that run.
 */
export function VideosView() {
  const [selected, setSelected] = useState(RECORDINGS[0]?.id ?? '')
  const [failed, setFailed] = useState<Record<string, boolean>>({})
  const setActive = useFlightStore((s) => s.setActive)
  const setView = useFlightStore((s) => s.setView)

  const base = import.meta.env.BASE_URL ?? '/'
  const recording =
    RECORDINGS.find((r) => r.id === selected) ?? RECORDINGS[0]
  if (!recording) return null

  const openInReplay = (scenario: ScenarioId) => {
    void setActive(scenario)
    setView('replay')
  }
  const unavailable = failed[recording.id] === true

  return (
    <div className="flex min-h-0 flex-1 flex-col lg:flex-row">
      {/* Recording list */}
      <div className="shrink-0 border-b border-grid lg:w-64 lg:overflow-y-auto lg:border-b-0 lg:border-r">
        <div className="px-3 pb-1 pt-3 font-mono text-[9px] tracking-[0.2em] text-ink-faint">
          RECORDED FLIGHTS
        </div>
        <div className="flex flex-row gap-px overflow-x-auto lg:flex-col">
          {RECORDINGS.map((r) => {
            const active = r.id === recording.id
            return (
              <button
                key={r.id}
                type="button"
                onClick={() => setSelected(r.id)}
                aria-current={active ? 'true' : undefined}
                className={`flex shrink-0 flex-col items-start gap-0.5 border-l-2 px-3 py-2 text-left transition-colors ${
                  active
                    ? 'border-signal bg-panel-3'
                    : 'border-transparent hover:border-grid-2 hover:bg-panel-2'
                }`}
              >
                <span
                  className={`font-mono text-[11px] font-semibold tracking-wider ${
                    active ? 'text-signal' : 'text-ink-dim'
                  }`}
                >
                  {r.label}
                </span>
                <span className="text-[10px] leading-tight text-ink-faint">
                  {r.subtitle}
                </span>
              </button>
            )
          })}
        </div>
      </div>

      {/* Player */}
      <div className="flex min-h-0 flex-1 flex-col gap-3 p-4">
        <div className="flex flex-wrap items-end justify-between gap-3">
          <div className="min-w-0">
            <div className="font-mono text-[11px] font-semibold tracking-widest text-ink">
              {recording.label}
            </div>
            <div className="text-[11px] text-ink-faint">{recording.subtitle}</div>
          </div>
          {recording.scenario && (
            <button
              type="button"
              onClick={() => openInReplay(recording.scenario as ScenarioId)}
              className="rounded-[3px] border border-grid-2 px-2.5 py-1 font-mono text-[10px] tracking-widest text-ink-dim transition-colors hover:border-signal hover:text-signal"
            >
              OPEN IN REPLAY
            </button>
          )}
        </div>
        <div className="flex min-h-0 flex-1 items-center justify-center overflow-hidden rounded-[3px] border border-grid bg-black/40">
          {unavailable ? (
            <StatusPanel
              title="RECORDING UNAVAILABLE"
              detail="run scripts/copy_data.sh"
              tone="error"
            />
          ) : (
            <video
              key={recording.id}
              src={`${base}videos/${recording.file}`}
              controls
              preload="metadata"
              className="max-h-full max-w-full"
              onError={() =>
                setFailed((f) => ({ ...f, [recording.id]: true }))
              }
            />
          )}
        </div>
      </div>
    </div>
  )
}
