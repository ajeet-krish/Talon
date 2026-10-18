import type { LiveConfig } from '@/live/types'

/** public/data/live/configs.json, written by sims/export_live_configs.py. */
export interface LiveConfigsFile {
  generated_utc: string
  configs: LiveConfig[]
}

/** Fetch the live-run scenario configs (one fetch per session). */
export async function loadLiveConfigs(): Promise<LiveConfig[]> {
  const base = import.meta.env.BASE_URL ?? '/'
  const res = await fetch(`${base}data/live/configs.json`)
  if (!res.ok) {
    throw new Error(`live config fetch failed (HTTP ${res.status})`)
  }
  const file = (await res.json()) as LiveConfigsFile
  return file.configs
}
