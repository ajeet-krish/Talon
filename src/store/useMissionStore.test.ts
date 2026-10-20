/**
 * Mission editor store tests (Phase 8c).
 *
 * The store is the only writer of the persisted draft, so the behaviors
 * pinned here are the ones a user would notice: every valid edit lands in
 * the undo stack and in localStorage, an invalid draft is held (not hidden)
 * with its error message, never overwrites the last valid stored config, and
 * cannot be exported, a failed import never destroys the work in progress,
 * and the module boots without a DOM so node/vitest can run it. Storage is
 * an in-memory stub; the initial-load cases re-import the module against
 * different stubs, the way a fresh tab would.
 */

import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { ScenarioConfig, Waypoint } from '@/types/generated/talon_core'
import { createHistory, emptyMission } from '@/utils/editor'
import { teachingPreset } from '@/data/presets'
import {
  DRAFT_STORAGE_KEY,
  MAX_IMPORT_BYTES,
  useMissionStore,
} from './useMissionStore'

/** An in-memory Storage so persistence assertions never touch the real thing. */
function memoryStorage(): Storage {
  const map = new Map<string, string>()
  return {
    getItem: (key: string) => map.get(key) ?? null,
    setItem: (key: string, value: string) => void map.set(key, value),
    removeItem: (key: string) => void map.delete(key),
    clear: () => map.clear(),
    key: (index: number) => [...map.keys()][index] ?? null,
    get length() {
      return map.size
    },
  }
}

/** A File stand-in: the importer only reads size and text(). */
function fakeFile(text: string, sizeBytes = text.length): File {
  return { size: sizeBytes, text: async () => text } as unknown as File
}

function wp(north: number, east: number, down: number): Waypoint {
  return { position: [north, east, down], speed: 5, heading: 0, radius: 2 }
}

let storage: Storage

beforeEach(() => {
  storage = memoryStorage()
  vi.stubGlobal('localStorage', storage)
  useMissionStore.setState({
    history: createHistory(emptyMission()),
    selected: null,
    errors: [],
    warnings: [],
    dirty: false,
    lastSavedAt: null,
  })
})

afterEach(() => {
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

describe('dispatch', () => {
  it('applies the edit, marks the draft dirty, and persists it', () => {
    useMissionStore.getState().dispatch({ type: 'waypoint/add', north: 10, east: 0 })
    const state = useMissionStore.getState()
    expect(state.history.present.waypoints).toHaveLength(1)
    expect(state.history.past).toHaveLength(1) // the start state is the undo entry
    expect(state.dirty).toBe(true)
    expect(typeof state.lastSavedAt).toBe('number')
    const saved = JSON.parse(storage.getItem(DRAFT_STORAGE_KEY) ?? 'null') as {
      waypoints: Waypoint[]
    }
    expect(saved.waypoints).toHaveLength(1)
    expect(saved.waypoints[0]?.position).toEqual([10, 0, -10])
  })

  it('records a validation error for an invalid edit and clears it when fixed', () => {
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'duration_s', value: 0 })
    expect(useMissionStore.getState().errors[0]).toContain('duration_s')
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'duration_s', value: 60 })
    expect(useMissionStore.getState().errors).toEqual([])
  })

  it('collects a warning per waypoint that has one', () => {
    useMissionStore
      .getState()
      .dispatch({ type: 'obstacle/add', north: 20, east: 3, radius_m: 4 })
    useMissionStore.getState().dispatch({ type: 'waypoint/add', north: 20, east: 0 })
    expect(useMissionStore.getState().warnings).toEqual([
      'Waypoint 1 is inside keep-out zone zone_1',
    ])
  })

  it('warns when a keep-out zone sits close to a planned leg and avoidance is on', () => {
    useMissionStore.getState().dispatch({ type: 'avoidance/enabled', enabled: true })
    useMissionStore.getState().dispatch({ type: 'waypoint/add', north: 15, east: 0 })
    useMissionStore.getState().dispatch({ type: 'obstacle/add', north: 7.5, east: 0, radius_m: 3 })
    expect(useMissionStore.getState().warnings).toEqual([
      'Keep-out zone zone_1 is close to the leg to waypoint 1. ' +
        'The bypass may carry the vehicle past waypoint 1: use a larger acceptance radius ' +
        'or move the zone.',
    ])
  })

  it('leaves the draft alone for a command that changes nothing', () => {
    useMissionStore.getState().dispatch({ type: 'waypoint/reorder', index: 0, delta: 1 })
    const state = useMissionStore.getState()
    expect(state.history.present.waypoints).toHaveLength(0)
    expect(state.dirty).toBe(false)
    expect(storage.getItem(DRAFT_STORAGE_KEY)).toBeNull()
  })

  it('holds an invalid draft in memory and persists only what validates', () => {
    // A mocked clock makes the lastSavedAt assertions exact, never flaky.
    vi.useFakeTimers({ toFake: ['Date'] })
    vi.setSystemTime(new Date('2026-01-01T00:00:00Z'))
    try {
      // A valid edit lands on disk and stamps the save time.
      useMissionStore
        .getState()
        .dispatch({ type: 'param/set', key: 'label', value: 'Valid before' })
      const persisted = storage.getItem(DRAFT_STORAGE_KEY)
      const savedAt = useMissionStore.getState().lastSavedAt
      expect(persisted).not.toBeNull()
      expect(savedAt).toBe(Date.now())

      // An invalid edit is held in memory; storage keeps the last valid draft.
      vi.setSystemTime(Date.now() + 1000)
      useMissionStore.getState().dispatch({ type: 'param/set', key: 'duration_s', value: 0 })
      const invalid = useMissionStore.getState()
      expect(invalid.errors[0]).toContain('duration_s')
      expect(invalid.history.present.duration_s).toBe(0) // the draft stays fixable
      expect(storage.getItem(DRAFT_STORAGE_KEY)).toBe(persisted)
      expect(invalid.lastSavedAt).toBe(savedAt) // not persisted: the stamp stands

      // The next valid commit persists the corrected config and stamps it.
      vi.setSystemTime(Date.now() + 2000)
      useMissionStore.getState().dispatch({ type: 'param/set', key: 'duration_s', value: 90 })
      const fixed = useMissionStore.getState()
      expect(fixed.errors).toEqual([])
      expect(fixed.lastSavedAt).toBe(Date.now())
      const saved = JSON.parse(storage.getItem(DRAFT_STORAGE_KEY) ?? 'null') as {
        duration_s: number
        label: string
      }
      expect(saved.duration_s).toBe(90)
      expect(saved.label).toBe('Valid before')
    } finally {
      vi.useRealTimers()
    }
  })
})

describe('undo and redo', () => {
  it('walks edits back and forward, persisting each step', () => {
    const store = useMissionStore.getState()
    store.dispatch({ type: 'waypoint/add', north: 10, east: 0 })
    store.dispatch({ type: 'waypoint/add', north: 10, east: 10 })
    expect(useMissionStore.getState().history.present.waypoints).toHaveLength(2)

    useMissionStore.getState().undo()
    expect(useMissionStore.getState().history.present.waypoints).toHaveLength(1)
    const undone = JSON.parse(storage.getItem(DRAFT_STORAGE_KEY) ?? 'null') as {
      waypoints: Waypoint[]
    }
    expect(undone.waypoints).toHaveLength(1)
    expect(useMissionStore.getState().dirty).toBe(true)

    useMissionStore.getState().redo()
    expect(useMissionStore.getState().history.present.waypoints).toHaveLength(2)
    expect(useMissionStore.getState().history.future).toHaveLength(0)
  })

  it('is a no-op at the start of the stack', () => {
    useMissionStore.getState().undo()
    expect(useMissionStore.getState().history.past).toHaveLength(0)
    expect(useMissionStore.getState().dirty).toBe(false)
  })
})

describe('select', () => {
  it('points at a waypoint and clears again', () => {
    useMissionStore.getState().select({ kind: 'waypoint', index: 2 })
    expect(useMissionStore.getState().selected).toEqual({ kind: 'waypoint', index: 2 })
    useMissionStore.getState().select(null)
    expect(useMissionStore.getState().selected).toBeNull()
  })
})

describe('loadConfig', () => {
  it('replaces the draft, resetting the history and the dirty flag', () => {
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'label', value: 'scratch' })
    const config: ScenarioConfig = { id: 'loaded_mission', waypoints: [wp(5, 5, -10)] }
    useMissionStore.getState().loadConfig(config)
    const state = useMissionStore.getState()
    expect(state.history.present.id).toBe('loaded_mission')
    expect(state.history.present.waypoints).toHaveLength(1)
    expect(state.history.past).toEqual([])
    expect(state.dirty).toBe(false)
    expect(state.errors).toEqual([])
  })

  it('keeps the previous draft when the config is invalid', () => {
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'label', value: 'keep me' })
    const before = useMissionStore.getState().history.present
    expect(() =>
      useMissionStore.getState().loadConfig({ id: 'hover' }), // reserved id
    ).toThrow(/reserved/)
    expect(useMissionStore.getState().history.present).toBe(before)
  })
})

describe('loadTemplate', () => {
  it('loads the teaching preset as the draft', () => {
    useMissionStore.getState().loadTemplate(teachingPreset)
    const state = useMissionStore.getState()
    expect(state.history.present.id).toBe('teaching_preset')
    expect(state.history.present.waypoints).toHaveLength(5)
    expect(state.errors).toEqual([])
    expect(state.warnings).toEqual([])
    expect(state.dirty).toBe(false)
  })
})

describe('importFile', () => {
  it('accepts a valid mission file and persists it', async () => {
    const text = JSON.stringify({ id: 'imported_mission', waypoints: [wp(1, 2, -10)] })
    await useMissionStore.getState().importFile(fakeFile(text))
    const state = useMissionStore.getState()
    expect(state.history.present.id).toBe('imported_mission')
    expect(state.history.present.waypoints).toHaveLength(1)
    expect(state.dirty).toBe(false)
    expect(storage.getItem(DRAFT_STORAGE_KEY)).toContain('imported_mission')
  })

  it('rejects malformed JSON and keeps the previous draft', async () => {
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'label', value: 'safe' })
    const before = useMissionStore.getState().history.present
    await expect(useMissionStore.getState().importFile(fakeFile('{not json'))).rejects.toThrow(
      /not valid JSON/,
    )
    expect(useMissionStore.getState().history.present).toBe(before)
  })

  it('labels a parser RangeError instead of calling it bad JSON, keeping the draft', async () => {
    // Deep nesting can exhaust the parser's stack before it rules on syntax;
    // the browser engines differ here, so the throw is provoked directly.
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'label', value: 'safe' })
    const before = useMissionStore.getState().history.present
    const parse = vi.spyOn(JSON, 'parse').mockImplementation(() => {
      throw new RangeError('Maximum call stack size exceeded')
    })
    try {
      await expect(useMissionStore.getState().importFile(fakeFile('[[[[[]]]]]'))).rejects.toThrow(
        'import rejected: the file is nested too deeply to read',
      )
    } finally {
      parse.mockRestore()
    }
    expect(useMissionStore.getState().history.present).toBe(before)
    expect(useMissionStore.getState().dirty).toBe(true) // only the label edit, still there
  })

  it('rejects an invalid config with the validator reason and keeps the draft', async () => {
    const before = useMissionStore.getState().history.present
    const text = JSON.stringify({ id: 'hover', duration_s: -5 }) // reserved id, bad duration
    await expect(useMissionStore.getState().importFile(fakeFile(text))).rejects.toThrow(
      /import rejected/,
    )
    expect(useMissionStore.getState().history.present).toBe(before)
  })

  it('rejects an oversized file before parsing it', async () => {
    const before = useMissionStore.getState().history.present
    const huge = fakeFile('{}', MAX_IMPORT_BYTES + 1)
    await expect(useMissionStore.getState().importFile(huge)).rejects.toThrow(
      /larger than 1000000 bytes/,
    )
    expect(useMissionStore.getState().history.present).toBe(before)
  })
})

describe('exportFile', () => {
  it('downloads pretty JSON as id.json and marks the draft saved', async () => {
    const anchor: { href?: string; download?: string; clicked: boolean } = { clicked: false }
    const captured: { blob: Blob | null } = { blob: null }
    vi.stubGlobal('URL', {
      createObjectURL: (blob: Blob) => {
        captured.blob = blob
        return 'blob:mock'
      },
      revokeObjectURL: vi.fn(),
    })
    vi.stubGlobal('document', {
      createElement: () => ({
        set href(value: string) {
          anchor.href = value
        },
        set download(value: string) {
          anchor.download = value
        },
        click: () => {
          anchor.clicked = true
        },
      }),
    })

    useMissionStore.getState().dispatch({ type: 'param/set', key: 'label', value: 'Exported' })
    useMissionStore.getState().exportFile()

    expect(anchor.download).toBe('custom_mission.json')
    expect(anchor.href).toBe('blob:mock')
    expect(anchor.clicked).toBe(true)
    expect(useMissionStore.getState().dirty).toBe(false)

    const text = await captured.blob?.text()
    expect(text).toContain('\n  "id": "custom_mission"')
    const parsed = JSON.parse(text ?? 'null') as { label: string }
    expect(parsed.label).toBe('Exported')
  })

  it('refuses to export a draft with validation errors, leaving everything in place', () => {
    useMissionStore.getState().dispatch({ type: 'param/set', key: 'duration_s', value: 0 })
    const before = useMissionStore.getState()
    expect(before.errors).toHaveLength(1)

    expect(() => useMissionStore.getState().exportFile()).toThrow(
      'export refused: fix 1 validation error first (duration_s must be in (0, 900])',
    )

    const after = useMissionStore.getState()
    expect(after.dirty).toBe(true) // still edited, nothing was downloaded
    expect(after.lastSavedAt).toBeNull() // the success path never ran
    expect(storage.getItem(DRAFT_STORAGE_KEY)).toBeNull() // nothing written
  })
})

describe('clearDraft', () => {
  it('forgets the stored draft and resets to a blank mission', () => {
    useMissionStore.getState().dispatch({ type: 'waypoint/add', north: 10, east: 0 })
    useMissionStore.getState().select({ kind: 'waypoint', index: 0 })
    expect(storage.getItem(DRAFT_STORAGE_KEY)).not.toBeNull()

    useMissionStore.getState().clearDraft()
    const state = useMissionStore.getState()
    expect(state.history.present).toEqual(emptyMission())
    expect(state.selected).toBeNull()
    expect(state.dirty).toBe(false)
    expect(state.errors).toEqual([])
    expect(storage.getItem(DRAFT_STORAGE_KEY)).toBeNull()
  })
})

describe('initial draft load', () => {
  /** Re-import the store module the way a fresh tab would. */
  async function freshStore(): Promise<typeof useMissionStore> {
    vi.resetModules()
    return (await import('./useMissionStore')).useMissionStore
  }

  it('restores a valid persisted draft', async () => {
    storage.setItem(
      DRAFT_STORAGE_KEY,
      JSON.stringify({ id: 'stored_mission', label: 'From disk', waypoints: [wp(3, 4, -10)] }),
    )
    const fresh = await freshStore()
    const state = fresh.getState()
    expect(state.history.present.id).toBe('stored_mission')
    expect(state.history.present.label).toBe('From disk')
    expect(state.history.present.waypoints).toHaveLength(1)
    expect(state.errors).toEqual([])
    expect(state.dirty).toBe(false)
  })

  it('falls back to a blank mission on corrupt JSON', async () => {
    storage.setItem(DRAFT_STORAGE_KEY, '{"id": "trunc')
    const fresh = await freshStore()
    expect(fresh.getState().history.present).toEqual(emptyMission())
  })

  it('falls back to a blank mission on an invalid draft', async () => {
    storage.setItem(DRAFT_STORAGE_KEY, JSON.stringify({ id: 'bad draft', duration_s: -1 }))
    const fresh = await freshStore()
    expect(fresh.getState().history.present).toEqual(emptyMission())
  })

  it('falls back when there is no storage at all', async () => {
    vi.stubGlobal('localStorage', undefined)
    const fresh = await freshStore()
    expect(fresh.getState().history.present).toEqual(emptyMission())
    expect(fresh.getState().dirty).toBe(false)
  })
})
