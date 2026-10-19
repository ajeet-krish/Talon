#!/usr/bin/env node
/**
 * Headless smoke test for the live run.
 *
 * The browser path can not be driven from CI (no browser), and the rust tests
 * cover the core rather than the boundary. What is left unverified is the seam:
 * the wasm build that ships in `public/wasm-rust/`, the sample layout the page
 * decodes, and the schema 1.1 document assembled from a *stepped* session rather
 * than a one-shot run.
 *
 * So this flies every live config through the built module the way the run view
 * does (step, drain the recorded samples, check what came back) and asserts the
 * outcomes the browser is expected to show. Run it after `./wasm/build.sh`:
 *
 *     node scripts/smoke_live_wasm.mjs
 */

import { readFileSync } from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const wasmDir = path.join(root, 'public/wasm-rust')

/** Sample layout, mirrored from `talon_core::sim::SAMPLE_FIELD`. */
const FIELD = {
  TIME: 0,
  TRUTH_POSITION: 1,
  TRUTH_VELOCITY: 4,
  EKF_POSITION: 14,
  WIND: 29,
  THRUST: 32,
}

/** One plant step (s), for the paced step count. */
const DT = 0.005
/** One control sample (s), which is what the streamed records are spaced at. */
const DT_CONTROL = 0.01
/** Plant steps per simulated frame, standing in for the frame budget. */
const STEPS_PER_FRAME = 200
/**
 * Tracking tolerance the recorded obstacle-field run allows on its clearance
 * margin (m), mirrored from `scenario_obstacle_field.CLEARANCE_TOLERANCE_M`.
 * The reactive law is judged the same way: it must clear the margin it was
 * given, less this much for tracking error.
 */
const CLEARANCE_TOLERANCE_M = 1.0

const failures = []
function check(ok, label, detail = '') {
  if (ok) {
    console.log(`  ok   ${label}`)
  } else {
    console.log(`  FAIL ${label}${detail === '' ? '' : ` :: ${detail}`}`)
    failures.push(label)
  }
}

const mod = await import(path.join(wasmDir, 'talon_core.js'))
await mod.default({
  module_or_path: readFileSync(path.join(wasmDir, 'talon_core_bg.wasm')),
})

const stride = mod.sample_stride()
const configs = JSON.parse(
  readFileSync(path.join(root, 'public/data/live/configs.json'), 'utf8'),
).configs

console.log(`core ${mod.core_version()}, streamed stride ${stride} doubles/sample`)
check(stride === 36, 'the wasm build streams the layout the page decodes', `got ${stride}`)
check(configs.length > 0, 'live configs are present')

/** One paced run, returning everything the checks below need. */
function fly(cfg) {
  const session = new mod.Session(JSON.stringify(cfg))
  session.prepare()

  const capacity = Math.ceil(cfg.duration_s / DT_CONTROL) + 64
  let drained = 0
  let phasesJson = session.phases_json()
  let refreshed = 0
  let frames = 0
  let maxDelta = 0

  // Step one frame's worth at a time, exactly as the paced loop does at 1x.
  while (!session.finished && frames < 200_000) {
    frames += 1
    session.advance(STEPS_PER_FRAME)
    const next = session.phases_json()
    if (next !== phasesJson) {
      phasesJson = next
      refreshed += 1
    }
    const recorded = session.sample_count
    if (recorded > drained) {
      const from = drained
      const delta = session.samples_from(from)
      const got = delta.length / stride
      // The delta must line up with the cursor, or the page would lose samples.
      maxDelta = Math.max(maxDelta, Math.abs(got - (recorded - from)))
      drained += got
    }
  }

  const block = session.samples_from(0)
  const last = block.length - stride
  const doc = JSON.parse(session.document_json('2026-01-01T00:00:00Z'))
  const summary = JSON.parse(session.summary_json())
  const phases = JSON.parse(phasesJson)
  const result = {
    samples: session.sample_count,
    drained,
    capacity,
    frames,
    refreshed,
    maxDelta,
    duration: cfg.duration_s,
    lastTime: block[last + FIELD.TIME],
    lastAltitude: -block[last + FIELD.TRUTH_POSITION + 2],
    sampleCountDoc: doc.time_s.length,
    obstacles: doc.obstacles.length,
    lastPhase: phases[phases.length - 1][1],
    phases: phases.length,
    summary,
    schema: doc.meta.schema_version,
  }
  session.free()
  return result
}

for (const cfg of configs) {
  console.log(`\n${cfg.id} (${cfg.duration_s} s, ${cfg.waypoints.length} waypoints, ${cfg.obstacles.length} zones)`)
  const r = fly(cfg)

  check(r.samples === Math.ceil(r.duration / DT_CONTROL), 'records one sample per control step', `got ${r.samples}`)
  check(r.drained === r.samples, 'draining returns every recorded sample', `drained ${r.drained} of ${r.samples}`)
  check(r.maxDelta === 0, 'each drain lines up with the cursor')
  check(r.drained <= r.capacity, 'the recorder sized to the config never overflows', `${r.drained} > ${r.capacity}`)
  check(r.schema === '1.1', 'the stepped document is schema 1.1', `got ${r.schema}`)
  check(r.sampleCountDoc === r.samples, 'the document carries every sample', `doc ${r.sampleCountDoc} vs stream ${r.samples}`)
  check(Math.abs(r.lastTime - (r.duration - DT_CONTROL)) < 1e-6, 'the last sample lands one control step short of the end', `got ${r.lastTime}`)
  check(r.obstacles === cfg.obstacles.length, 'the document carries the scenario keep-out zones')
  check(r.phases >= 4, 'the phase timeline reports the flight phases', `got ${r.phases}`)
  check(r.summary.ekf_converged === true, 'the estimator converged')
  check(r.summary.ekf_diverged === false, 'the estimator did not diverge')

  // Scenario-specific outcomes: the same acceptance the browser is judged on.
  // A scenario with waypoints flies a mission and comes home; one without is a
  // hold at the takeoff altitude that stays up until the clock runs out.
  if (cfg.waypoints.length > 0) {
    check(r.lastPhase === 'idle', 'a completed mission returns to idle', `got ${r.lastPhase}`)
    check(r.summary.mission_complete === true, 'the waypoint mission completed')
    check(
      r.summary.waypoints_reached === r.summary.waypoint_count,
      'every waypoint was reached',
      `${r.summary.waypoints_reached}/${r.summary.waypoint_count}`,
    )
    check(r.lastAltitude < 0.5, 'the vehicle landed', `alt ${r.lastAltitude.toFixed(2)} m`)
  } else {
    check(
      r.lastAltitude > 9.5 && r.lastAltitude < 10.5,
      'the vehicle holds the takeoff altitude',
      `alt ${r.lastAltitude.toFixed(2)} m`,
    )
  }

  // The reactive avoidance scenario is the one whose path is not planned in
  // advance: the mission runs straight through the zones and the autopilot
  // deviates around each one itself. The clearance is judged the way the
  // recorded obstacle-field run is judged, against the margin the config
  // states minus the tracking tolerance that run allows.
  if (cfg.avoidance && cfg.avoidance.enabled) {
    check(
      r.summary.avoidance_activations > 0,
      'the avoidance law took at least one bypass',
      `activations ${r.summary.avoidance_activations}`,
    )
    const margin = cfg.avoidance.clearance_margin_m ?? 2.0
    const clearance = r.summary.min_obstacle_clearance_m
    check(
      typeof clearance === 'number' && clearance >= margin - CLEARANCE_TOLERANCE_M,
      'the flown path held the clearance margin',
      `clearance ${clearance} m against a ${margin} m margin`,
    )
    console.log(
      `  info avoidance activations=${r.summary.avoidance_activations} ` +
        `minClearance=${clearance.toFixed(2)}m`,
    )
  }
  console.log(
    `  info frames=${r.frames} phases=${r.phases} (${r.refreshed} refreshes) ` +
      `lastTime=${r.lastTime.toFixed(2)}s lastAlt=${r.lastAltitude.toFixed(2)}m ` +
      `gps=${r.summary.gps_accepted}/${r.summary.gps_gated}`,
  )
}

/**
 * Paced run: the same arithmetic the run view uses.
 *
 * The pacer asks "what sim time should we be at now" and the view advances by
 * `round((target - time) / DT)` steps. Rounding per frame must not accumulate
 * into a drift, so a full second of frames has to land within a frame of the
 * wall clock at 1x, and there must be no leftover when the run ends.
 */
function paced(cfg, rate, frameMs) {
  const session = new mod.Session(JSON.stringify(cfg))
  session.prepare()

  let wallMs = 0
  let frames = 0
  let maxLag = 0
  let stages = 0
  while (!session.finished && frames < 500_000) {
    frames += 1
    wallMs += frameMs
    const target = Math.min((wallMs / 1000) * rate, cfg.duration_s)
    const steps = Math.round((target - session.time) / DT)
    if (steps > 0) session.advance(steps)
    maxLag = Math.max(maxLag, target - session.time)
    if (session.sample_count > stages) stages = session.sample_count
  }
  const time = session.time
  session.free()
  return { frames, wallMs, time, maxLag, stages }
}

console.log('\npaced loop (1x, 60 Hz frames)')
const hoverCfg = configs.find((c) => c.id === 'hover')
if (hoverCfg === undefined) {
  check(false, 'the hover config exists')
} else {
  const p = paced(hoverCfg, 1, 1000 / 60)
  check(
    Math.abs(p.time - hoverCfg.duration_s) < DT_CONTROL,
    'a paced run reaches the end of the scenario',
    `time ${p.time.toFixed(3)} of ${hoverCfg.duration_s}`,
  )
  check(
    p.maxLag < 0.05,
    'the revealed time never lags the wall clock by more than a frame',
    `max lag ${p.maxLag.toFixed(4)} s`,
  )
  check(
    Math.abs(p.wallMs / 1000 - hoverCfg.duration_s) < 0.5,
    'a run at 1x takes about as long as the scenario',
    `${(p.wallMs / 1000).toFixed(2)} s of wall time`,
  )
  console.log(
    `  info frames=${p.frames} wall=${(p.wallMs / 1000).toFixed(2)}s sim=${p.time.toFixed(2)}s`,
  )
}

console.log('')
if (failures.length > 0) {
  console.error(`${failures.length} check(s) failed`)
  process.exit(1)
}
console.log('all live smoke checks passed')
