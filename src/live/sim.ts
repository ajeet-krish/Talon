/**
 * Live in-browser SIL (FlightLab Phase 5b): the seam between the page and the
 * Rust autopilot.
 *
 * In Phase 4 this file was the whole orchestration loop in TypeScript: it
 * stepped a scalar WebAssembly kernel per plant step and ran guidance, the
 * mission manager, the motor mixer, the sensor suite, and the phase machine
 * itself. All of that now lives in the Rust core (`talon_core::sim`), the same
 * code the desktop harness and the batch runs execute, so this class is small
 * on purpose. It does three things and no autopilot work at all:
 *
 * 1. hands the scenario JSON to a native session,
 * 2. advances that session inside a frame budget so the page stays responsive,
 * 3. turns the finished session into the flight document, alongside the run
 *    summary the core itself produced, which the replay, telemetry, and
 *    metrics views already consume.
 *
 * The summary is a passthrough: the core computes RMSE, final error, obstacle
 * clearance, and the phase list, and this file parses that without changing it,
 * apart from normalizing a missing phase list to an empty one.
 *
 * Because the native core owns the loop, the browser run and a batch run of the
 * same scenario are the same flight. The one deliberate difference is the sensor
 * noise stream: the core uses its own generator rather than numpy's, so a live
 * run is statistically equivalent to a recorded one, not bit-identical.
 */

import type { FlightData, FlightPhase, PhaseEvent, Vec3 } from '@/types/flight'
import type { CoreSession } from './engine'
import type { LiveConfig, LiveRunResult, LiveRunSummary } from './types'

const KNOWN_PHASES: readonly FlightPhase[] = [
  'idle',
  'arm',
  'takeoff',
  'cruise',
  'waypoint',
  'rtl',
  'land',
  'emergency',
]

function isPhase(value: string): value is FlightPhase {
  return (KNOWN_PHASES as readonly string[]).includes(value)
}

/**
 * One live run of one scenario.
 *
 * A run is steppable (`advance`) or one-shot (`runToCompletion`), and either
 * way ends with `finish` producing the same document a recorded flight has.
 */
export class LiveSim {
  private readonly session: CoreSession
  private result: LiveRunResult | null = null

  constructor(session: CoreSession) {
    this.session = session
  }

  /**
   * Start a run from a scenario and a loaded kernel.
   *
   * The scenario goes over as JSON in exactly the shape the config file uses,
   * so there is no second mapping to keep in step.
   */
  static create(cfg: LiveConfig, Session: new (json: string) => CoreSession): LiveSim {
    return new LiveSim(new Session(JSON.stringify(cfg)))
  }

  /** Convene the estimator before the timed run. Idempotent. */
  prepare(): void {
    this.session.prepare()
  }

  /**
   * Advance one plant step.
   *
   * Returns true once the run is finished, which is the loop condition the Run
   * view uses to decide between another frame and the result.
   */
  advance(): boolean {
    return this.session.step()
  }

  /** Advance up to `steps` plant steps; returns how many were advanced. */
  advanceMany(steps: number): number {
    return this.session.advance(steps)
  }

  /** Fly the rest of the run in one call. */
  runToCompletion(): void {
    if (!this.session.prepared) this.session.prepare()
    this.session.run_to_end()
  }

  /** Fraction of the run completed, in [0, 1]. */
  get progress(): number {
    return this.session.progress
  }

  get finished(): boolean {
    return this.session.finished
  }

  /** Sim time (s), 0 to `duration_s` after the pre-roll. */
  get time(): number {
    return this.session.time
  }

  /** Current phase, as the schema spells it. */
  get phase(): FlightPhase {
    const label = this.session.phase()
    return isPhase(label) ? label : 'idle'
  }

  /** Live altitude above the ground plane (m, positive up). */
  get altitudeM(): number {
    return this.session.altitude_m()
  }

  /** Live truth position, NED (m). */
  get truthPosition(): Vec3 {
    const p = this.session.truth_position()
    return [p[0] ?? 0, p[1] ?? 0, p[2] ?? 0]
  }

  /** Live ground speed (m/s), from the truth velocity. */
  get groundSpeed(): number {
    const v = this.session.truth_velocity()
    return Math.hypot(v[0] ?? 0, v[1] ?? 0)
  }

  /** Battery state of charge in [0, 1]. */
  get batterySoc(): number {
    return this.session.battery_soc
  }

  /**
   * Samples recorded so far, at the control rate.
   *
   * This is also the cursor: the number of samples already consumed is exactly
   * what the next `drain` should ask for from.
   */
  get sampleCount(): number {
    return this.session.sample_count
  }

  /** Pull the samples recorded since `from` as a flat block. */
  drain(from: number): Float64Array {
    return this.session.samples_from(from)
  }

  /** The phase timeline as `[time_s, phase]` pairs. */
  phases(): PhaseEvent[] {
    try {
      return JSON.parse(this.session.phases_json()) as PhaseEvent[]
    } catch {
      return []
    }
  }

  /** GPS fix accounting from the core: accepted, gated, skipped. */
  gpsCounts(): [number, number, number] {
    // The core counts are u64 and arrive as bigint; they are narrowed here for display.
    return [
      Number(this.session.gps_accepted),
      Number(this.session.gps_gated),
      Number(this.session.gps_skipped),
    ]
  }

  /** Estimator flags from the core: converged, diverged. */
  ekfFlags(): [boolean, boolean] {
    return [this.session.ekf_converged, this.session.ekf_diverged]
  }

  /**
   * The flight document and the core's run summary for the run so far.
   *
   * The summary is whatever `talon_core::sim` wrote, parsed once as the
   * generated `RunSummary`. Nothing here recomputes RMSE, final error,
   * clearance, or the phase list: the core owns those numbers. The single
   * adjustment is defensive: an absent or non-array `phases` becomes `[]`
   * so the Run view never maps over undefined.
   */
  finish(): LiveRunResult {
    if (this.result !== null) return this.result

    const data = JSON.parse(this.session.document_json(new Date().toISOString())) as FlightData
    const parsed = JSON.parse(this.session.summary_json()) as LiveRunSummary

    // The wasm fallback for an impossible serialization error is `{}`, and the
    // Run view maps over `summary.phases`, so an absent (or null) list must not
    // throw. Only the list is touched: it becomes `[]` unless it already is an
    // array, and every other field stays exactly as the core wrote it.
    const summary: LiveRunSummary = {
      ...parsed,
      phases: Array.isArray(parsed?.phases) ? parsed.phases : [],
    }

    this.result = { data, summary }
    return this.result
  }

  /** Release the native session. */
  dispose(): void {
    this.session.free()
  }
}
