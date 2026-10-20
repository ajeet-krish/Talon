/**
 * Inspector section chrome for the mission editor (FlightLab Phase 8c): a
 * mono uppercase label followed by a hairline rule, the same quiet instrument
 * language the rails use elsewhere in the tool. Panels stack these to make
 * the inspector scannable without boxes or cards.
 */

import type { ReactNode } from 'react'

export interface SectionProps {
  /** Section title, rendered uppercase mono. */
  title: string
  /** Optional trailing count or status shown at the rule's end. */
  aside?: ReactNode
  children: ReactNode
}

export function Section({ title, aside, children }: SectionProps) {
  return (
    <section className="px-3 py-3">
      <div className="mb-2 flex items-baseline gap-2">
        <h2 className="font-mono text-[9px] uppercase tracking-[0.2em] text-ink-faint">
          {title}
        </h2>
        <div className="h-px flex-1 translate-y-[-2px] bg-grid" />
        {aside !== undefined && (
          <span className="font-mono text-[9px] tabular-nums text-ink-faint">{aside}</span>
        )}
      </div>
      {children}
    </section>
  )
}
