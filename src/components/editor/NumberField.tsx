/**
 * Controlled numeric field for the mission editor (FlightLab Phase 8c).
 *
 * The field keeps a local string while it has focus so typing never fights
 * the store, and commits on blur or Enter only when the text parses to a
 * finite number inside `[min, max]` (and, with `integer`, a whole number).
 * Anything else shows a red border and an inline message and commits
 * nothing, so a half-typed "1." or an out-of-range value can never reach the
 * mission config. Escape reverts the text to the last committed value; the
 * arrow keys step by `step` (`1` for an integer field), clamped to the same
 * bounds, which is what makes `step` mean something on a text input.
 *
 * A blur or Enter whose text parses back to the committed value commits
 * nothing at all: tabbing through the form (or retyping the same number)
 * must not create an undo entry in the store.
 */

import { useId, useState } from 'react'

export interface NumberFieldProps {
  /** Visible field label (required: every input in the editor is labelled). */
  label: string
  /** Committed value shown whenever the field is not being edited. */
  value: number
  /** Engineering unit rendered inside the field, e.g. `m` or `m/s`. */
  suffix?: string
  /** Inclusive lower bound. */
  min?: number
  /** Inclusive upper bound. */
  max?: number
  /** `step` attribute for the native input. */
  step?: number
  /** Accept whole numbers only (rejects fractions, arrows step by 1). */
  integer?: boolean
  /** Called with the parsed, in-range value on blur or Enter. */
  onCommit: (value: number) => void
}

/** Format a committed value for editing: integers bare, fractions trimmed. */
function editString(value: number): string {
  if (!Number.isFinite(value)) return ''
  if (Number.isInteger(value)) return String(value)
  return String(Number(value.toFixed(4)))
}

/** Step a value by `delta * step`, trimmed of float dust, clamped to bounds. */
function stepValue(
  value: number,
  delta: number,
  step: number,
  min: number | undefined,
  max: number | undefined,
): number {
  const next = Number((value + delta * step).toFixed(6))
  if (min !== undefined && next < min) return min
  if (max !== undefined && next > max) return max
  return next
}

/** One problem with the draft text, or null when it is committable. */
function draftProblem(
  text: string,
  min: number | undefined,
  max: number | undefined,
  integer: boolean,
): string | null {
  if (text.trim() === '') return 'Enter a number'
  const parsed = Number(text)
  if (!Number.isFinite(parsed)) return 'Not a number'
  if (integer && !Number.isInteger(parsed)) return 'Whole numbers only'
  if (min !== undefined && parsed < min) return `At least ${min}`
  if (max !== undefined && parsed > max) return `At most ${max}`
  return null
}

/**
 * What a blur or Enter should do with the draft text: the message to show
 * when it cannot be committed, otherwise the value to commit. A draft that
 * parses back to the committed value commits nothing (`commit: null` with no
 * problem), which is what keeps tabbing through the form from marking the
 * mission edited. Pure and exported so the rules are testable without a DOM.
 */
export function draftOutcome(
  text: string,
  committed: number,
  min: number | undefined,
  max: number | undefined,
  integer: boolean,
): { problem: string | null; commit: number | null } {
  const problem = draftProblem(text, min, max, integer)
  if (problem !== null) return { problem, commit: null }
  const parsed = Number(text)
  return { problem: null, commit: parsed === committed ? null : parsed }
}

export function NumberField({
  label,
  value,
  suffix,
  min,
  max,
  step,
  integer = false,
  onCommit,
}: NumberFieldProps) {
  const id = useId()
  const messageId = `${id}-message`
  const [draft, setDraft] = useState<string | null>(null)
  const [problem, setProblem] = useState<string | null>(null)

  const text = draft ?? editString(value)
  const invalid = problem !== null
  // An integer field always steps by 1: a fraction could never commit.
  const stepSize = integer ? 1 : (step ?? 1)

  return (
    <div className="flex flex-col gap-1">
      <label
        htmlFor={id}
        className="font-mono text-[9px] uppercase tracking-[0.15em] text-ink-faint"
      >
        {label}
      </label>
      <div
        className={`flex items-center gap-1 border bg-panel px-2 py-1 transition-colors focus-within:border-signal ${
          invalid ? 'border-danger' : 'border-grid'
        }`}
      >
        <input
          id={id}
          type="text"
          inputMode={integer ? 'numeric' : 'decimal'}
          autoComplete="off"
          value={text}
          aria-invalid={invalid}
          aria-describedby={invalid ? messageId : undefined}
          onChange={(e) => {
            const next = e.target.value
            setDraft(next)
            setProblem(draftProblem(next, min, max, integer))
          }}
          onFocus={() => setDraft(editString(value))}
          onBlur={() => {
            if (draft === null) return
            const outcome = draftOutcome(draft, value, min, max, integer)
            if (outcome.problem !== null) {
              // Keep the offending text and the message; commit nothing.
              setProblem(outcome.problem)
              return
            }
            // A draft that parses back to the committed value commits
            // nothing: tabbing through the form is not an edit.
            if (outcome.commit !== null) onCommit(outcome.commit)
            setDraft(null)
            setProblem(null)
          }}
          onKeyDown={(e) => {
            if (e.key === 'Enter') {
              e.preventDefault()
              e.currentTarget.blur()
            } else if (e.key === 'Escape') {
              e.preventDefault()
              setDraft(null)
              setProblem(null)
            } else if (e.key === 'ArrowUp' || e.key === 'ArrowDown') {
              // Step the committed value; the arrows never type into the text.
              e.preventDefault()
              const base =
                draft !== null && draftProblem(draft, min, max, integer) === null
                  ? Number(draft)
                  : value
              const next = stepValue(base, e.key === 'ArrowUp' ? 1 : -1, stepSize, min, max)
              setDraft(null)
              setProblem(null)
              if (integer && !Number.isInteger(next)) return // never commit a fraction
              if (next !== value) onCommit(next)
            }
          }}
          className="w-full min-w-0 bg-transparent font-mono text-[11px] tabular-nums text-ink outline-none focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-[-1px] focus-visible:outline-signal"
        />
        {suffix !== undefined && (
          <span className="shrink-0 font-mono text-[9px] text-ink-faint">{suffix}</span>
        )}
      </div>
      {invalid && (
        <p id={messageId} className="font-mono text-[9px] leading-tight text-danger">
          {problem}
        </p>
      )}
    </div>
  )
}
