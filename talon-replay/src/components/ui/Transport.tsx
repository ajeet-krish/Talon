import { useFlightStore } from '@/store/useFlightStore'

const SPEEDS = [0.5, 1, 2, 4] as const

function PlayIcon({ playing }: { playing: boolean }) {
  return playing ? (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="currentColor" aria-hidden="true">
      <rect x="2" y="1" width="3.5" height="12" rx="0.5" />
      <rect x="8.5" y="1" width="3.5" height="12" rx="0.5" />
    </svg>
  ) : (
    <svg width="14" height="14" viewBox="0 0 14 14" fill="currentColor" aria-hidden="true">
      <path d="M3 1.5v11l9-5.5-9-5.5z" />
    </svg>
  )
}

/**
 * Transport controls: play/pause plus playback-rate selection, styled as
 * a segmented instrument switch. Mono labels keep the rates reading as
 * data, not decoration.
 */
export function Transport() {
  const isPlaying = useFlightStore((s) => s.isPlaying)
  const speed = useFlightStore((s) => s.playbackSpeed)
  const togglePlay = useFlightStore((s) => s.togglePlay)
  const setSpeed = useFlightStore((s) => s.setSpeed)

  return (
    <div className="flex items-center gap-3 border-r border-grid px-4">
      <button
        type="button"
        onClick={togglePlay}
        aria-label={isPlaying ? 'Pause replay' : 'Play replay'}
        className="flex h-8 w-8 items-center justify-center rounded-[3px] border border-grid-2 bg-panel-3 text-signal transition-colors hover:border-signal hover:bg-panel-2 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-signal"
      >
        <PlayIcon playing={isPlaying} />
      </button>
      <div
        className="flex items-center overflow-hidden rounded-[3px] border border-grid-2"
        role="group"
        aria-label="Playback speed"
      >
        {SPEEDS.map((s) => (
          <button
            key={s}
            type="button"
            onClick={() => setSpeed(s)}
            aria-pressed={speed === s}
            className={`px-2 py-1 font-mono text-[11px] leading-none tabular-nums transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-2px] focus-visible:outline-signal ${
              speed === s
                ? 'bg-signal/15 text-signal'
                : 'text-ink-faint hover:text-ink-dim'
            }`}
          >
            {s}x
          </button>
        ))}
      </div>
    </div>
  )
}
