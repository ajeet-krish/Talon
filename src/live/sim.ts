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
 * 3. turns the finished session into the flight document and the run summary
 *    the replay, telemetry, and metrics views already consume.
 *
 * Because the native core owns the loop, the browser run and a batch run of the
 * same scenario are the same flight. The one deliberate difference is the sensor
 * noise stream: the core uses its own generator rather than numpy's, so a live
 * run is statistically equivalent to a recorded one, not bit-identical.
 */

import type { FlightData, FlightPhase, Obstacle, PhaseEvent, Vec3 } from '@/types/flight'
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

/** The subset of the native summary that the flight document does not carry. */
interface CoreSummary {
  duration_s: number
  waypoint_count: number
  waypoints_reached: number
  mission_complete: boolean
  gps_accepted: number
  gps_gated: number
  gps_skipped: number
  final_phase: string
  final_truth_altitude_m: number
  ekf_converged: boolean
  ekf_diverged: boolean
  steps_done: number
  total_steps: number
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

  /** The flight document and the summary for the run so far. */
  finish(): LiveRunResult {
    if (this.result !== null) return this.result

    const data = JSON.parse(this.session.document_json(new Date().toISOString())) as FlightData
    const core = JSON.parse(this.session.summary_json()) as CoreSummary

    const n = data.truth_position.length
    let sumSq = 0
    let finalError = 0
    for (let i = 0; i < n; i += 1) {
      const truth = data.truth_position[i]
      const estimate = data.ekf_position[i]
      if (truth === undefined || estimate === undefined) continue
      const d2 = (truth[0] - estimate[0]) ** 2 + (truth[1] - estimate[1]) ** 2 + (truth[2] - estimate[2]) ** 2
      sumSq += d2
      finalError = Math.sqrt(d2)
    }

    const summary: LiveRunSummary = {
      duration_s: core.duration_s,
      waypoint_count: core.waypoint_count,
      waypoints_reached: core.waypoints_reached,
      mission_complete: core.mission_complete,
      gps_accepted: core.gps_accepted,
      gps_gated: core.gps_gated,
      final_position_error_m: finalError,
      ekf_position_rmse_m: n === 0 ? 0 : Math.sqrt(sumSq / n),
      min_clearance_m: minClearance(data.truth_position, data.obstacles),
      phases: phasesVisited(data.phase_timeline),
    }

    this.result = { data, summary }
    return this.result
  }

  /** Release the native session. */
  dispose(): void {
    this.session.free()
  }
}

/**
 * Smallest distance from any truth sample to any obstacle surface (m), or null
 * when the scenario declares no keep-out zones. Negative means a penetration.
 */
function minClearance(truth: Vec3[], obstacles: Obstacle[]): number | null {
  if (obstacles.length === 0) return null
  let best = Number.POSITIVE_INFINITY
  for (const p of truth) {
    for (const o of obstacles) {
      const d = Math.sqrt(
        (p[0] - o.center_ned[0]) ** 2 +
          (p[1] - o.center_ned[1]) ** 2 +
          (p[2] - o.center_ned[2]) ** 2,
      )
      best = Math.min(best, d - o.radius_m)
    }
  }
  return Number.isFinite(best) ? best : null
}

/** The phases a run passed through, in order, without repeats. */
function phasesVisited(timeline: [number, FlightPhase][]): FlightPhase[] {
  const out: FlightPhase[] = []
  for (const entry of timeline) {
    const phase = entry[1]
    if (out[out.length - 1] !== phase) out.push(phase)
  }
  return out
}
