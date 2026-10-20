/**
 * Run-view config list helpers (FlightLab Phase 8c). Pure functions over the
 * live-run config list: the fetched shipped scenarios plus, when one exists,
 * the mission authored in the editor. No React, no fetch, no store, so the
 * placement rule and the button captions are unit-testable on their own.
 */

import type { LiveConfig } from '@/live/types'

/**
 * The config list the Run view offers: the authored mission placed ahead of
 * the fetched scenarios, or the fetched list unchanged when nothing was
 * authored. The fetched array is never mutated and its entries keep their
 * object identity, and the authored entry keeps its own config id, which is
 * the one the session and the recording metadata use.
 */
export function withAuthoredMission(
  fetched: LiveConfig[] | null,
  authored: LiveConfig | null,
): LiveConfig[] | null {
  if (authored === null) return fetched
  if (fetched === null) return [authored]
  return [authored, ...fetched]
}

/**
 * Button caption for one config in the Run view strip (and its scene
 * overlay). The authored mission is marked as yours, so it never reads as a
 * shipped scenario; every other label passes through, uppercased.
 */
export function configButtonLabel(config: LiveConfig, authored: LiveConfig | null): string {
  if (authored === null || config.id !== authored.id) return config.label.toUpperCase()
  const label = config.label.trim()
  return label === '' ? 'MY MISSION' : `MY MISSION: ${label.toUpperCase()}`
}
