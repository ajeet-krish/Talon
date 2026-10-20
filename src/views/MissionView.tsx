/**
 * Mission editor (FlightLab Phase 8c): the section where a mission is
 * authored. A toolbar carries the draft's identity and its file verbs, the
 * body is the planning chart beside a scrolling inspector, and a validation
 * bar pins the run gate to the bottom.
 *
 * The chart is the working surface: a north-up, true-scale plan you can
 * click, drag, and keyboard-edit. The inspector holds the same edits as
 * fields for when precision beats pointing. Nothing here runs the simulator:
 * the Fly button validates the draft, hands it to the flight store's
 * authored-mission slot, and opens the Run section, which owns the run. An
 * optional `onFly` prop overrides that wiring for embedders and tests.
 */

import { useRef, useState } from 'react'
import { MISSION_TEMPLATES } from '@/data/presets'
import { validateScenarioConfig } from '@/data/validate'
import { useFlightStore } from '@/store/useFlightStore'
import { useMissionStore } from '@/store/useMissionStore'
import { canRedo, canUndo, toMissionConfig } from '@/utils/editor'
import type { MissionConfig } from '@/utils/editor'
import { MissionCanvas } from '@/components/editor/MissionCanvas'
import { FaultPanel } from '@/components/editor/FaultPanel'
import { MissionParametersPanel } from '@/components/editor/MissionParametersPanel'
import { MissionValidation } from '@/components/editor/MissionValidation'
import { ObstaclePanel } from '@/components/editor/ObstaclePanel'
import { PlanSummary } from '@/components/editor/PlanSummary'
import { WindPanel } from '@/components/editor/WindPanel'
import { WaypointPanel } from '@/components/editor/WaypointPanel'

export interface MissionViewProps {
  /**
   * Override the Fly wiring. By default the Fly button validates the draft,
   * stores it as the flight store's authored mission, and opens the Run
   * section; a handler here intercepts that instead.
   */
  onFly?: (config: MissionConfig) => void
}

/** Toolbar button base: quiet border, mono, one hover state. */
const TOOL_BUTTON =
  'border border-grid px-2.5 py-1 font-mono text-[10px] tracking-[0.15em] text-ink-dim transition-colors hover:bg-panel-3 hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal disabled:opacity-40 disabled:hover:bg-transparent disabled:hover:text-ink-dim'

/** `HH:MM` for the saved marker, or `--:--` when the draft was never saved. */
function savedTime(lastSavedAt: number | null): string {
  if (lastSavedAt === null) return '--:--'
  return new Date(lastSavedAt).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
}

export function MissionView({ onFly }: MissionViewProps) {
  const config = useMissionStore((s) => s.history.present)
  const history = useMissionStore((s) => s.history)
  const selected = useMissionStore((s) => s.selected)
  const errors = useMissionStore((s) => s.errors)
  const warnings = useMissionStore((s) => s.warnings)
  const dirty = useMissionStore((s) => s.dirty)
  const lastSavedAt = useMissionStore((s) => s.lastSavedAt)
  const dispatch = useMissionStore((s) => s.dispatch)
  const undo = useMissionStore((s) => s.undo)
  const redo = useMissionStore((s) => s.redo)
  const select = useMissionStore((s) => s.select)
  const loadTemplate = useMissionStore((s) => s.loadTemplate)
  const importFile = useMissionStore((s) => s.importFile)
  const exportFile = useMissionStore((s) => s.exportFile)
  const clearDraft = useMissionStore((s) => s.clearDraft)
  const setAuthoredMission = useFlightStore((s) => s.setAuthoredMission)
  const setView = useFlightStore((s) => s.setView)

  const fileInputRef = useRef<HTMLInputElement | null>(null)
  /**
   * Dismissible banner for a rejected import, a refused export, or a refused
   * Fly. It only ever reports: what lands here never touches the draft.
   */
  const [banner, setBanner] = useState<{ label: string; message: string } | null>(null)

  /** Load a template as the new draft, confirming over unsaved edits. */
  function applyTemplate(id: string): void {
    const template = MISSION_TEMPLATES.find((t) => t.id === id)
    if (template === undefined) return
    if (
      dirty &&
      !window.confirm(`Replace this draft with the "${template.label}" template? Your edits will be lost.`)
    ) {
      return
    }
    loadTemplate(template.build)
    select(null)
    setBanner(null)
  }

  /** Read a picked file into the draft; a rejection keeps the draft intact. */
  async function onPickFile(file: File): Promise<void> {
    try {
      await importFile(file)
      select(null)
      setBanner(null)
    } catch (err) {
      setBanner({
        label: 'IMPORT FAILED',
        message: err instanceof Error ? err.message : 'import failed',
      })
    }
  }

  /** Forget the draft, confirming over unsaved edits first. */
  function onClear(): void {
    if (
      dirty &&
      !window.confirm('Clear this mission and start from a blank draft? Your edits will be lost.')
    ) {
      return
    }
    clearDraft()
    setBanner(null)
  }

  /** Download the draft; a refusal lands in the banner and touches nothing. */
  function onExport(): void {
    try {
      exportFile()
      setBanner(null)
    } catch (err) {
      setBanner({
        label: 'EXPORT REFUSED',
        message: err instanceof Error ? err.message : 'the mission did not validate',
      })
    }
  }

  /**
   * Hand the validated draft to the run pipeline: store it as the authored
   * mission and open the Run section, which owns the reset, the selection,
   * and the run itself. The validator is the same gate the store's error list
   * came from, so a ready button passes; if it still refuses, the editor
   * stays and the banner says why.
   */
  function onFlyMission(): void {
    try {
      // The editor shape is already the run contract: `toMissionConfig`
      // normalizes into the complete config the Run view's LiveConfig needs,
      // cloning every array so later edits never reach the stored mission.
      const valid = toMissionConfig(validateScenarioConfig(config))
      setAuthoredMission(valid)
      setView('run')
    } catch (err) {
      setBanner({
        label: 'FLY REFUSED',
        message: err instanceof Error ? err.message : 'the mission did not validate',
      })
    }
  }

  const undoable = canUndo(history)
  const redoable = canRedo(history)
  const flyReady = errors.length === 0
  const flyTitle =
    errors.length > 0
      ? `Fix ${errors.length} validation error${errors.length === 1 ? '' : 's'} first`
      : undefined

  return (
    <div className="flex min-h-0 flex-1 flex-col">
      {/* Toolbar: identity on the left, verbs on the right, wraps when tight. */}
      <div className="flex shrink-0 flex-wrap items-center gap-x-3 gap-y-2 border-b border-grid bg-panel-2 px-3 py-2">
        <div className="flex min-w-0 items-baseline gap-2">
          <span className="font-mono text-[11px] font-semibold tracking-[0.2em] text-ink">
            MISSION EDITOR
          </span>
          <span className="truncate font-sans text-[11px] text-ink-dim">{config.label}</span>
          <span
            className={`shrink-0 font-mono text-[9px] tracking-[0.15em] ${
              dirty ? 'text-warn' : 'text-ink-faint'
            }`}
            title={dirty ? 'Edited since the last export' : 'Draft saved to this browser'}
          >
            {dirty ? 'EDITED' : `SAVED ${savedTime(lastSavedAt)}`}
          </span>
        </div>

        <div className="flex min-w-0 items-center gap-1.5">
          <label
            htmlFor="mission-template"
            className="font-mono text-[9px] tracking-[0.15em] text-ink-faint"
          >
            TEMPLATE
          </label>
          <select
            id="mission-template"
            value=""
            onChange={(e) => {
              if (e.target.value !== '') applyTemplate(e.target.value)
            }}
            className="max-w-[11rem] border border-grid bg-panel px-2 py-1 font-mono text-[10px] text-ink-dim transition-colors hover:text-ink focus-visible:border-signal"
          >
            <option value="">LOAD...</option>
            {MISSION_TEMPLATES.map((template) => (
              <option key={template.id} value={template.id}>
                {template.label}
              </option>
            ))}
          </select>
        </div>

        <div className="flex items-center gap-1">
          <button
            type="button"
            onClick={undo}
            disabled={!undoable}
            title={undoable ? 'Undo the last edit' : 'Nothing to undo'}
            className={TOOL_BUTTON}
          >
            UNDO
          </button>
          <button
            type="button"
            onClick={redo}
            disabled={!redoable}
            title={redoable ? 'Redo the last undone edit' : 'Nothing to redo'}
            className={TOOL_BUTTON}
          >
            REDO
          </button>
        </div>

        <div className="flex items-center gap-1">
          <button
            type="button"
            onClick={() => fileInputRef.current?.click()}
            className={TOOL_BUTTON}
          >
            IMPORT
          </button>
          <button type="button" onClick={onExport} className={TOOL_BUTTON}>
            EXPORT
          </button>
          <button type="button" onClick={onClear} className={TOOL_BUTTON}>
            CLEAR
          </button>
        </div>

        <div className="ml-auto flex items-center gap-2">
          <button
            type="button"
            onClick={() => (onFly !== undefined ? onFly(config) : onFlyMission())}
            disabled={!flyReady}
            title={flyTitle ?? 'Validate this mission, store it, and open the Run section'}
            className={`border px-3 py-1 font-mono text-[10px] tracking-[0.2em] transition-colors focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal ${
              flyReady
                ? 'border-signal text-signal hover:bg-panel-3'
                : 'border-grid text-ink-faint'
            }`}
          >
            FLY THIS MISSION
          </button>
        </div>

        <input
          ref={fileInputRef}
          type="file"
          accept="application/json,.json"
          className="hidden"
          aria-label="Import a mission JSON file"
          onChange={(e) => {
            const file = e.target.files?.[0]
            // Reset so picking the same file again re-fires the change.
            e.target.value = ''
            if (file !== undefined) void onPickFile(file)
          }}
        />
      </div>

      {/* Import or Fly failure: a dismissible banner that never touches the draft. */}
      {banner !== null && (
        <div className="flex shrink-0 items-center gap-2 border-b border-grid bg-panel-3 px-3 py-1.5">
          <span className="shrink-0 font-mono text-[9px] tracking-[0.15em] text-danger">
            {banner.label}
          </span>
          <span className="min-w-0 flex-1 truncate font-mono text-[10px] text-ink-dim">
            {banner.message}
          </span>
          <button
            type="button"
            onClick={() => setBanner(null)}
            className="shrink-0 font-mono text-[9px] tracking-[0.15em] text-ink-faint transition-colors hover:text-ink focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-1 focus-visible:outline-signal"
          >
            DISMISS
          </button>
        </div>
      )}

      {/* Body: the chart at working size, the inspector scrolling beside it.
          On narrow screens the two stack and scroll together, so the
          validation bar stays reachable at the bottom of the view. */}
      <div className="flex min-h-0 flex-1 flex-col overflow-y-auto lg:flex-row lg:overflow-hidden">
        <div className="min-h-[360px] flex-1 border-b border-grid lg:border-b-0">
          <MissionCanvas
            config={config}
            selected={selected}
            onSelect={select}
            dispatch={dispatch}
          />
        </div>
        <aside className="w-full shrink-0 overflow-y-auto border-grid lg:w-[23rem] lg:border-l">
          <PlanSummary config={config} warnings={warnings.length} />
          <MissionParametersPanel config={config} dispatch={dispatch} />
          <WaypointPanel
            config={config}
            selected={selected}
            dispatch={dispatch}
            onSelect={select}
          />
          <WindPanel config={config} dispatch={dispatch} />
          <ObstaclePanel
            config={config}
            selected={selected}
            dispatch={dispatch}
            onSelect={select}
          />
          <FaultPanel config={config} dispatch={dispatch} />
        </aside>
      </div>

      {/* Validation bar: the run gate, pinned to the bottom of the view. */}
      <div className="shrink-0 border-t border-grid bg-panel-2 px-3 py-2">
        <MissionValidation errors={errors} warnings={warnings} />
      </div>
    </div>
  )
}
