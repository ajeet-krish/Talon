/**
 * Mission editor store (FlightLab Phase 8c). Zustand state around the pure
 * editor model in `@/utils/editor`: the undo history, the current selection,
 * the validation errors and non-blocking warnings for the present config,
 * and the draft's saved/dirty state.
 *
 * The draft persists to `localStorage` under `talon.mission.draft.v1` after
 * every edit, undo, redo, and load that validates. An invalid draft is held
 * in memory (so the user can fix it) and never overwrites storage: the last
 * valid config stays on disk and `lastSavedAt` keeps the time of that write
 * until a valid commit persists again. Every storage touch is guarded, and
 * the store module itself never assumes a DOM exists, so node/vitest can
 * import it and run against stubbed storage.
 *
 * Errors come from `validateScenarioConfig` on the present config (one
 * message is enough to point the user at the gate), warnings from
 * `missionWarnings` (every waypoint warning, plus a keep-out zone standing
 * close to a planned leg while avoidance is on). Neither blocks editing: the
 * user can hold an invalid draft, fix it, and only save or run when the
 * errors clear.
 */

import { create } from 'zustand'
import { validateScenarioConfig } from '@/data/validate'
import type { ScenarioConfig } from '@/types/generated/talon_core'
import {
  applyCommand,
  createHistory,
  emptyMission,
  missionWarnings,
  redo as redoHistory,
  toMissionConfig,
  undo as undoHistory,
  type EditorCommand,
  type EditorHistory,
  type EditorSelection,
  type MissionConfig,
} from '@/utils/editor'

/** localStorage key of the persisted draft. */
export const DRAFT_STORAGE_KEY = 'talon.mission.draft.v1'

/** Largest accepted import file, bytes. */
export const MAX_IMPORT_BYTES = 1_000_000

/** Read-only access to storage that degrades to null when there is none. */
function storage(): Storage | null {
  try {
    return typeof localStorage === 'undefined' ? null : localStorage
  } catch {
    // Storage can also throw on access (privacy modes); treat that as absent.
    return null
  }
}

/** Parse and validate the persisted draft, or fall back to a blank mission. */
function loadDraft(): MissionConfig {
  const store = storage()
  if (store === null) return emptyMission()
  try {
    const raw = store.getItem(DRAFT_STORAGE_KEY)
    if (raw === null) return emptyMission()
    return toMissionConfig(validateScenarioConfig(JSON.parse(raw)))
  } catch {
    // A stale, truncated, or hand-edited draft must never brick the editor.
    return emptyMission()
  }
}

/** Persist the present config; a storage failure leaves the draft in memory. */
function persist(config: MissionConfig): void {
  const store = storage()
  if (store === null) return
  try {
    store.setItem(DRAFT_STORAGE_KEY, JSON.stringify(config))
  } catch {
    // Quota or privacy failure: the in-memory draft stays authoritative.
  }
}

/** One validation message for the present config, or none. */
function configErrors(config: MissionConfig): string[] {
  try {
    validateScenarioConfig(config)
    return []
  } catch (err) {
    return [err instanceof Error ? err.message : 'invalid mission']
  }
}

export interface MissionStore {
  /** Undo/redo state: the present mission plus the stacks of previous ones. */
  history: EditorHistory
  /** What the editor points at, or null. Kept outside the history. */
  selected: EditorSelection
  /** Blocking validation messages for the present config. */
  errors: string[]
  /** Non-blocking advisory messages for the present config. */
  warnings: string[]
  /** True when the draft has edits not yet exported. */
  dirty: boolean
  /** When the draft was last written to storage (only valid configs are written), or null. */
  lastSavedAt: number | null

  /** Apply one edit: new present, undo entry, errors/warnings, draft saved when valid. */
  dispatch: (cmd: EditorCommand) => void
  /** Step one edit back (also persists, when the result validates). */
  undo: () => void
  /** Step one undone edit forward (also persists, when the result validates). */
  redo: () => void
  /** Point the editor at one waypoint or zone, or clear the selection. */
  select: (sel: EditorSelection) => void
  /** Validate and replace the draft, resetting the history. Throws on invalid. */
  loadConfig: (config: ScenarioConfig) => void
  /** Build a template and load it as the draft. */
  loadTemplate: (build: () => MissionConfig) => void
  /** Import a JSON mission file (max 1 MB). Throws, keeping the draft, on failure. */
  importFile: (file: File) => Promise<void>
  /**
   * Download the present config as pretty JSON and mark the draft saved.
   * Throws, leaving everything as it was, when the config has validation errors.
   */
  exportFile: () => void
  /** Forget the persisted draft and reset to a blank mission. */
  clearDraft: () => void
}

export const useMissionStore = create<MissionStore>((set, get) => {
  /**
   * Install a new history as the draft: recompute errors and warnings, mark
   * dirty, and persist only when the config validates. An invalid draft
   * stays in memory (with its errors) so the user can fix it; storage keeps
   * the last valid config and `lastSavedAt` keeps the time of that write.
   */
  function commit(history: EditorHistory, dirty: boolean): void {
    const config = history.present
    const errors = configErrors(config)
    const persisted = errors.length === 0
    set({
      history,
      errors,
      warnings: missionWarnings(config),
      dirty,
      lastSavedAt: persisted ? Date.now() : get().lastSavedAt,
    })
    if (persisted) persist(config)
  }

  const initial = loadDraft()

  return {
    history: createHistory(initial),
    selected: null,
    errors: configErrors(initial),
    warnings: missionWarnings(initial),
    dirty: false,
    lastSavedAt: null,

    dispatch: (cmd) => {
      const history = applyCommand(get().history, cmd)
      if (history === get().history) return // no-op command: leave the draft alone
      commit(history, true)
    },

    undo: () => {
      const history = undoHistory(get().history)
      if (history === get().history) return
      commit(history, true)
    },

    redo: () => {
      const history = redoHistory(get().history)
      if (history === get().history) return
      commit(history, true)
    },

    select: (sel) => set({ selected: sel }),

    loadConfig: (config) => {
      // Validation throws naming the first bad path; the previous draft stays.
      const valid = validateScenarioConfig(config)
      commit(createHistory(toMissionConfig(valid)), false)
    },

    loadTemplate: (build) => {
      get().loadConfig(build())
    },

    importFile: async (file) => {
      if (file.size > MAX_IMPORT_BYTES) {
        throw new Error(
          `import rejected: the file is larger than ${MAX_IMPORT_BYTES} bytes (got ${file.size})`,
        )
      }
      const text = await file.text()
      let parsed: unknown
      try {
        parsed = JSON.parse(text)
      } catch (err) {
        // Pathologically nested input can exhaust the parser's stack before
        // it decides the text is malformed; say that plainly instead of
        // calling it a syntax error. Either way nothing is parsed, so the
        // draft is untouched.
        if (err instanceof RangeError) {
          throw new Error('import rejected: the file is nested too deeply to read')
        }
        throw new Error('import rejected: the file is not valid JSON')
      }
      try {
        const valid = validateScenarioConfig(parsed)
        commit(createHistory(toMissionConfig(valid)), false)
      } catch (err) {
        throw new Error(
          `import rejected: ${err instanceof Error ? err.message : 'invalid mission'}`,
        )
      }
    },

    exportFile: () => {
      const config = get().history.present
      // Refuse before touching the page: an invalid draft must not be
      // written to disk as a file the user would have to repair later. The
      // caller (MissionView) turns this into the banner the import path uses.
      const errors = configErrors(config)
      if (errors.length > 0) {
        const first = errors[0] ?? 'the mission did not validate'
        throw new Error(
          `export refused: fix ${errors.length} validation error${errors.length === 1 ? '' : 's'} first (${first})`,
        )
      }
      const json = `${JSON.stringify(config, null, 2)}\n`
      const blob = new Blob([json], { type: 'application/json' })
      if (typeof document !== 'undefined' && typeof URL.createObjectURL === 'function') {
        const url = URL.createObjectURL(blob)
        const anchor = document.createElement('a')
        anchor.href = url
        anchor.download = `${config.id}.json`
        anchor.click()
        URL.revokeObjectURL(url)
      }
      set({ dirty: false, lastSavedAt: Date.now() })
    },

    clearDraft: () => {
      const store = storage()
      if (store !== null) {
        try {
          store.removeItem(DRAFT_STORAGE_KEY)
        } catch {
          // Nothing to do: the reset below is what matters.
        }
      }
      // Reset in memory without re-persisting: the point is an absent draft.
      const config = emptyMission()
      set({
        history: createHistory(config),
        selected: null,
        errors: [],
        warnings: [],
        dirty: false,
        lastSavedAt: null,
      })
    },
  }
})
