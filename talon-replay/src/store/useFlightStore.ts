import { create } from 'zustand'
import type { FlightData, ScenarioId } from '@/types/flight'

export interface DisplayToggles {
  trail: boolean
  ellipsoid: boolean
  wind: boolean
  waypoints: boolean
  obstacles: boolean
}

export const SCENARIO_IDS: ScenarioId[] = [
  'gps_outage',
  'wind',
  'emergency',
  'obstacle_field',
]

interface FlightStore {
  /** Loaded scenario payloads, keyed by scenario id. */
  scenarios: Map<ScenarioId, FlightData>
  activeScenario: ScenarioId
  loading: boolean
  error: string | null
  /** Playback cursor, seconds into the active scenario. */
  currentTime: number
  isPlaying: boolean
  playbackSpeed: number
  toggles: DisplayToggles

  /** Fetch scenario JSON (no-op if already cached). */
  loadScenario: (id: ScenarioId) => Promise<void>
  /** Switch the active scenario, resetting the playhead. */
  setActive: (id: ScenarioId) => Promise<void>
  /** Move the playhead, clamped to [0, duration]. */
  setTime: (t: number) => void
  togglePlay: () => void
  setSpeed: (speed: number) => void
  setToggle: (key: keyof DisplayToggles, value: boolean) => void
  /** Advance playback by wall-clock dt scaled by playbackSpeed. */
  tick: (dtSeconds: number) => void
}

function clampTime(t: number, duration: number): number {
  if (!Number.isFinite(t)) return 0
  return Math.min(Math.max(t, 0), duration)
}

export const useFlightStore = create<FlightStore>((set, get) => ({
  scenarios: new Map<ScenarioId, FlightData>(),
  activeScenario: 'gps_outage',
  loading: false,
  error: null,
  currentTime: 0,
  isPlaying: false,
  playbackSpeed: 1,
  toggles: { trail: true, ellipsoid: true, wind: true, waypoints: true, obstacles: true },

  loadScenario: async (id) => {
    if (get().scenarios.has(id)) return
    set({ loading: true, error: null })
    try {
      const base = import.meta.env.BASE_URL ?? '/'
      const res = await fetch(`${base}data/${id}.json`)
      if (!res.ok) throw new Error(`telemetry fetch failed (HTTP ${res.status})`)
      const data = (await res.json()) as FlightData
      // Schema 1.0 documents predate the obstacles key; normalize so the
      // scene never has to guard against undefined.
      if (!Array.isArray(data.obstacles)) data.obstacles = []
      set((s) => ({
        scenarios: new Map(s.scenarios).set(id, data),
        loading: false,
      }))
    } catch (err) {
      set({
        loading: false,
        error: err instanceof Error ? err.message : 'telemetry fetch failed',
      })
    }
  },

  setActive: async (id) => {
    if (id === get().activeScenario && get().scenarios.has(id)) return
    set({ activeScenario: id, currentTime: 0, isPlaying: false, error: null })
    await get().loadScenario(id)
  },

  setTime: (t) => {
    const data = get().scenarios.get(get().activeScenario)
    const duration = data?.meta.duration_s ?? 0
    set({ currentTime: clampTime(t, duration) })
  },

  togglePlay: () => set((s) => ({ isPlaying: !s.isPlaying })),

  setSpeed: (speed) => set({ playbackSpeed: speed }),

  setToggle: (key, value) => set((s) => ({ toggles: { ...s.toggles, [key]: value } })),

  tick: (dtSeconds) =>
    set((s) => {
      const data = s.scenarios.get(s.activeScenario)
      const duration = data?.meta.duration_s ?? 0
      const next = s.currentTime + dtSeconds
      if (next >= duration) return { currentTime: duration, isPlaying: false }
      return { currentTime: Math.max(0, next) }
    }),
}))
