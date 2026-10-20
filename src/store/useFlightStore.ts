import { create } from 'zustand'
import type { FlightData, ScenarioId } from '@/types/flight'
import type { LiveConfig, LiveRunSummary } from '@/live/types'
import { validateFlightData } from '@/data/validate'

export interface DisplayToggles {
  trail: boolean
  ellipsoid: boolean
  wind: boolean
  waypoints: boolean
  obstacles: boolean
}

/** Top-level tool sections. */
export type ViewId = 'replay' | 'telemetry' | 'metrics' | 'videos' | 'run' | 'mission'

/** One row of the validation driver's machine-readable report. */
export interface ValidationRow {
  test: string
  target: string
  achieved: string
  passed: boolean
}

/** plots/json/validation.json, written by sims/run_all.py. */
export interface ValidationReport {
  generated_utc: string
  passed: number
  total: number
  rows: ValidationRow[]
}

export const SCENARIO_IDS: ScenarioId[] = [
  'gps_outage',
  'wind',
  'emergency',
  'obstacle_field',
]

/**
 * Key into the scenario map: a recorded scenario id, or the single 'live'
 * slot holding the most recent in-browser WASM run. The live slot is only
 * present after a run completes; it is never fetched.
 */
export type ScenarioKey = ScenarioId | 'live'

interface FlightStore {
  /** Loaded scenario payloads, keyed by scenario id (or the live slot). */
  scenarios: Map<ScenarioKey, FlightData>
  activeScenario: ScenarioKey
  loading: boolean
  error: string | null
  /** Playback cursor, seconds into the active scenario. */
  currentTime: number
  isPlaying: boolean
  playbackSpeed: number
  toggles: DisplayToggles
  /** Active top-level tool section. */
  activeView: ViewId
  /** Validation driver report (lazy; fetched by the Metrics view). */
  validation: ValidationReport | null
  validationError: string | null
  /** Summary of the most recent live run, or null before the first run. */
  liveRun: LiveRunSummary | null
  /**
   * The validated mission authored in the editor, ready to fly, or null.
   * The store only holds it: starting the run belongs to the Run view.
   */
  authoredMission: LiveConfig | null

  /** Fetch scenario JSON (no-op if already cached, or if id is 'live'). */
  loadScenario: (id: ScenarioKey) => Promise<void>
  /** Switch the active scenario, resetting the playhead. */
  setActive: (id: ScenarioKey) => Promise<void>
  /** Store a completed live run under the 'live' key and select it. */
  setLiveRun: (data: FlightData, summary: LiveRunSummary) => void
  /** Hand the editor's validated mission to the Run view, or clear it. Starts no run. */
  setAuthoredMission: (config: LiveConfig | null) => void
  /** Move the playhead, clamped to [0, duration]. */
  setTime: (t: number) => void
  togglePlay: () => void
  setSpeed: (speed: number) => void
  setToggle: (key: keyof DisplayToggles, value: boolean) => void
  /** Switch the active tool section. */
  setView: (view: ViewId) => void
  /** Fetch the validation report (no-op if already loaded). */
  loadValidation: () => Promise<void>
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
  activeView: 'replay',
  validation: null,
  validationError: null,
  liveRun: null,
  authoredMission: null,

  loadScenario: async (id) => {
    if (id === 'live') return
    if (get().scenarios.has(id)) return
    set({ loading: true, error: null })
    try {
      const base = import.meta.env.BASE_URL ?? '/'
      const res = await fetch(`${base}data/${id}.json`)
      if (!res.ok) throw new Error(`telemetry fetch failed (HTTP ${res.status})`)
      // Validated rather than cast: every history must exist, every number
      // must be finite, and schema 1.0 documents without an obstacles key
      // come back with obstacles: []. A malformed recording fails here and
      // lands in `error` below, instead of surfacing as NaN in a chart.
      const payload: unknown = await res.json()
      const data = validateFlightData(payload)
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

  setLiveRun: (data, summary) =>
    set((s) => ({
      scenarios: new Map(s.scenarios).set('live', data),
      liveRun: summary,
      activeScenario: 'live',
      currentTime: 0,
      isPlaying: false,
      error: null,
    })),

  // Pure hand-off: the Run view watches `authoredMission` and owns the reset,
  // the selection, and the run itself, so no run starts here.
  setAuthoredMission: (config) => set({ authoredMission: config }),

  setTime: (t) => {
    const data = get().scenarios.get(get().activeScenario)
    const duration = data?.meta.duration_s ?? 0
    set({ currentTime: clampTime(t, duration) })
  },

  togglePlay: () => set((s) => ({ isPlaying: !s.isPlaying })),

  setSpeed: (speed) => set({ playbackSpeed: speed }),

  setToggle: (key, value) => set((s) => ({ toggles: { ...s.toggles, [key]: value } })),

  setView: (view) => set({ activeView: view }),

  loadValidation: async () => {
    if (get().validation) return
    try {
      const base = import.meta.env.BASE_URL ?? '/'
      const res = await fetch(`${base}data/validation.json`)
      if (!res.ok) {
        throw new Error(`validation fetch failed (HTTP ${res.status})`)
      }
      const data = (await res.json()) as ValidationReport
      set({ validation: data, validationError: null })
    } catch (err) {
      set({
        validationError:
          err instanceof Error ? err.message : 'validation fetch failed',
      })
    }
  },

  tick: (dtSeconds) =>
    set((s) => {
      const data = s.scenarios.get(s.activeScenario)
      const duration = data?.meta.duration_s ?? 0
      const next = s.currentTime + dtSeconds
      if (next >= duration) return { currentTime: duration, isPlaying: false }
      return { currentTime: Math.max(0, next) }
    }),
}))
