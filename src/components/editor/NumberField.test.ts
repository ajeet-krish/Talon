/**
 * NumberField draft rules (Phase 8c).
 *
 * The field's blur/Enter decision is a pure function (`draftOutcome`), so the
 * three behaviors the editor relies on are pinned without a DOM: an
 * uncommittable draft (blank, non-finite, out of range, or a fraction in an
 * integer field) produces the inline message and no value, a draft that
 * parses back to the committed value commits nothing (tabbing through a form
 * is not an edit and must not create an undo entry), and a real change
 * commits exactly the parsed number.
 */

import { describe, expect, it } from 'vitest'
import { draftOutcome } from './NumberField'

describe('draftOutcome', () => {
  it('commits a real change as the parsed number', () => {
    expect(draftOutcome('42', 12, 0, 100, false)).toEqual({ problem: null, commit: 42 })
    expect(draftOutcome('2.5', 12, 0, 100, false)).toEqual({ problem: null, commit: 2.5 })
  })

  it('commits nothing when the text parses back to the committed value', () => {
    expect(draftOutcome('12', 12, 0, 100, false)).toEqual({ problem: null, commit: null })
    expect(draftOutcome('12.0', 12, 0, 100, false)).toEqual({ problem: null, commit: null })
    expect(draftOutcome(' 12 ', 12, 0, 100, false)).toEqual({ problem: null, commit: null })
  })

  it('rejects a fraction in an integer field with the inline message', () => {
    expect(draftOutcome('2.5', 2, 0, 100, true)).toEqual({
      problem: 'Whole numbers only',
      commit: null,
    })
    expect(draftOutcome('7', 2, 0, 100, true)).toEqual({ problem: null, commit: 7 })
  })

  it('keeps the blank, non-finite, and out-of-range messages', () => {
    expect(draftOutcome('', 1, 0, 100, false)).toEqual({ problem: 'Enter a number', commit: null })
    expect(draftOutcome('abc', 1, 0, 100, false)).toEqual({
      problem: 'Not a number',
      commit: null,
    })
    expect(draftOutcome('-1', 1, 0, 100, false)).toEqual({ problem: 'At least 0', commit: null })
    expect(draftOutcome('101', 1, 0, 100, false)).toEqual({
      problem: 'At most 100',
      commit: null,
    })
    expect(draftOutcome('-1', 1, 0, 100, true)).toEqual({ problem: 'At least 0', commit: null })
  })

  it('never commits a value for a draft that has a problem', () => {
    for (const text of ['', 'x', '-1', '101', '2.5', '7', '2']) {
      const outcome = draftOutcome(text, 2, 0, 100, true)
      if (outcome.problem !== null) expect(outcome.commit).toBeNull()
      if (outcome.commit !== null) expect(outcome.problem).toBeNull()
    }
  })
})
