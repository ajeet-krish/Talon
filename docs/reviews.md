# Talon Reviews

Last updated: 2026-10-17

## FlightLab Phase 3: unified entry point (2026-10-17)

Scope: the four-section shell in `src/views/` and `App.tsx`, the
new pure utilities (`src/utils/chart.ts`, `src/utils/telemetry.ts`,
`src/utils/format.ts`), the SVG chart (`src/components/charts/LineChart.tsx`),
the `validation.json` sidecar in `python/sims/run_all.py`, and the video staging
in `scripts/copy_data.sh`.

Verdict: PASS.

### Correctness

- PASS: `npm run build` (tsc project check plus vite production build) exits
  0; `dist/` contains `data/` (four scenarios plus `validation.json`) and
  `videos/` (five recordings).
- PASS: hooks order is unconditional in every view. `TelemetryView` and
  `MetricsView` call their `useMemo`/`useEffect` before any early return, so
  switching sections cannot violate the rules of hooks.
- PASS: `src/utils/chart.ts` decimation preserves local extremes
  (`chart.test.ts` proves a lone spike survives a four-point budget) and emits
  non-decreasing times, so the polyline cannot zigzag.
- PASS: `src/utils/telemetry.ts` normalizes signed zero on the altitude
  channel (`altitudeOf`), so a zero down value renders as `0` and not `-0.0`.
- PASS: `python/sims/run_all.py` writes `validation.json` from the same `ReportRow`
  list as the markdown report, so the two cannot disagree; the driver run
  reports 25/25 with the JSON sidecar present.

### Conventions

- PASS: no em dashes, double hyphens, or dash equivalents in any new source,
  test, doc, or script.
- PASS: C++/Python/TS conventions untouched; new TS uses `noUncheckedIndexedAccess`
  guards (`times[i] as number`, `?? [0, 0, 0]`) rather than non-null assertions.
- PASS: chart colors (`CHART_COLORS`) and verdict colors stay outside the
  reserved Wong flight-phase palette in `src/utils/colors.ts`, so phase color
  keeps a single meaning.

### Maintainability and performance

- PASS: one clearance module and one telemetry module feed scene, HUD, legend,
  charts, and metrics; no duplicated derivation.
- PASS: chart geometry is memoized on the payload; only `ChartCursor` and
  `LiveValue` subscribe to `currentTime`, so a playing scenario does not
  rebuild a roughly seven-hundred-point path per chart per frame (see
  decisions D13, D14).
- PASS: shared scenario metadata is centralized in `src/data/scenarios.ts` and
  consumed by the rail and the telemetry header (removes the duplicate record
  that previously lived in `ScenarioSelector.tsx`).
- PASS: `copy_data.sh` names both the scenario and recording lists explicitly,
  so a new scenario or recording is a one-line addition.

### Security review

- PASS: no new network surface. `loadValidation` fetches a static same-origin
  path built from `import.meta.env.BASE_URL`; scenario ids come from the fixed
  `SCENARIO_IDS` union and recording files from a fixed list, so no
  user-controlled string reaches a fetch or a `src`.
- PASS: no `dangerouslySetInnerHTML`, `eval`, `new Function`, or dynamic HTML.
- PASS: validation rows are rendered as React text nodes; the driver-supplied
  strings are escaped by React, not interpolated into markup.
- PASS: `LineChart` renders `NaN`-safe readouts via `formatMetric` (guards
  non-finite input), so a malformed sample cannot crash a chart.
