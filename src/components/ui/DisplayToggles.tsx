import { useFlightStore } from '@/store/useFlightStore'
import type { DisplayToggles } from '@/store/useFlightStore'

const BASE_TOGGLES: { key: keyof DisplayToggles; label: string }[] = [
  { key: 'trail', label: 'Truth trail' },
  { key: 'ellipsoid', label: 'EKF 3-sigma' },
  { key: 'wind', label: 'Wind field' },
  { key: 'waypoints', label: 'Waypoints' },
]

/**
 * Scene-layer checkboxes as a compact instrument list. Each row is a real
 * checkbox (keyboard- and screen-reader-friendly) with an accent-color
 * override to signal cyan. The keep-out toggle appears only when the active
 * scenario ships obstacles, so scenarios with none do not show a dead switch.
 */
export function DisplayToggles() {
  const toggles = useFlightStore((s) => s.toggles)
  const setToggle = useFlightStore((s) => s.setToggle)
  const hasObstacles = useFlightStore(
    (s) => (s.scenarios.get(s.activeScenario)?.obstacles.length ?? 0) > 0,
  )

  const rows = hasObstacles
    ? [...BASE_TOGGLES, { key: 'obstacles' as const, label: 'Keep-out zones' }]
    : BASE_TOGGLES

  return (
    <div className="flex flex-col gap-1.5 px-3">
      {rows.map(({ key, label }) => (
        <label
          key={key}
          className="flex cursor-pointer items-center gap-2 py-0.5 text-[11px] text-ink-dim transition-colors hover:text-ink"
        >
          <input
            type="checkbox"
            checked={toggles[key]}
            onChange={(e) => setToggle(key, e.target.checked)}
            className="h-3 w-3 accent-signal"
          />
          {label}
        </label>
      ))}
    </div>
  )
}
