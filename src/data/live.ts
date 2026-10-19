import type { LiveConfig } from '@/live/types'
import { validateLiveConfigs } from '@/data/validate'

/** public/data/live/configs.json, written by sims/export_live_configs.py. */
export interface LiveConfigsFile {
  generated_utc: string
  configs: LiveConfig[]
}

/**
 * Fetch the live-run scenario configs (one fetch per session).
 *
 * The payload is validated rather than cast: a stale or edited configs file
 * fails here, naming the first bad path, instead of reaching the run view with
 * a duration or waypoint it can not fly.
 */
export async function loadLiveConfigs(): Promise<LiveConfig[]> {
  const base = import.meta.env.BASE_URL ?? '/'
  const res = await fetch(`${base}data/live/configs.json`)
  if (!res.ok) {
    throw new Error(`live config fetch failed (HTTP ${res.status})`)
  }
  const payload: unknown = await res.json()
  return validateLiveConfigs(payload)
}
