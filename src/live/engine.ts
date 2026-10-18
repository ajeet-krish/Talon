/**
 * Loader for the Rust GNC core compiled to WebAssembly.
 *
 * The kernel is built by `wasm/build.sh` into `public/wasm-rust/` as an ES
 * module plus a `.wasm`, and is loaded here at runtime from the Vite base URL,
 * so it ships as a static asset instead of being bundled. That keeps the
 * several-hundred-kilobyte module out of the JS graph and lets a stale or
 * missing build degrade to a clear message rather than a failed bundle.
 *
 * The exported surface is declared by hand, mirroring the generated
 * `talon_core.d.ts`. A runtime import carries no types, so hand-declaring the
 * small surface the app uses is what keeps `tsc` honest about it. The names and
 * signatures are the contract: if the Rust side changes them, this fails to
 * typecheck, which is the point.
 */

/** One steppable mission run, backed by `talon_core::sim::SimRunner`. */
export interface CoreSession {
  /** Convene the estimator before the timed run. Idempotent. */
  prepare(): void
  /** Advance one plant step; true once the run is finished. */
  step(): boolean
  /** Advance up to `steps` plant steps; returns how many were advanced. */
  advance(steps: number): number
  /** Advance to the end of the run; returns how many steps were advanced. */
  run_to_end(): number
  /** Fraction of the run completed, in [0, 1]. */
  readonly progress: number
  readonly finished: boolean
  readonly prepared: boolean
  readonly steps_done: number
  readonly total_steps: number
  /** Sim time (s), 0 to `duration_s` after the pre-roll. */
  readonly time: number
  readonly duration_s: number
  readonly scenario_id: string
  readonly battery_soc: number
  /** Samples recorded so far, at the control rate. */
  readonly sample_count: number
  /** Accepted GPS fixes since the run started. */
  readonly gps_accepted: number
  /** GPS fixes rejected by the innovation gate. */
  readonly gps_gated: number
  /** GPS fixes dropped for an injected outage. */
  readonly gps_skipped: number
  /** Whether the estimator has converged. */
  readonly ekf_converged: boolean
  /** Whether the estimator has been declared divergent. */
  readonly ekf_diverged: boolean
  /** Current flight-phase label, lowercase. */
  phase(): string
  /** The phase timeline as JSON: `[[time_s, "phase"], ...]`, oldest first. */
  phases_json(): string
  /**
   * The samples in `[from, sample_count)` as a flat `Float64Array`,
   * `sample_stride()` doubles per sample.
   */
  samples_from(from: number): Float64Array
  /** Live truth position, NED (m). */
  truth_position(): Float64Array
  /** Live truth velocity, NED (m/s). */
  truth_velocity(): Float64Array
  /** Live estimated position, NED (m). */
  ekf_position(): Float64Array
  /** Live altitude above the ground plane (m, positive up). */
  altitude_m(): number
  /** Schema 1.1 flight document for the run so far, as JSON. */
  document_json(generatedUtc: string): string
  /** Headline run numbers, as JSON. */
  summary_json(): string
  /** Release the underlying Rust object. */
  free(): void
}

/** The loaded WebAssembly module. */
export interface CoreModule {
  Session: new (configJson: string) => CoreSession
  core_version(): string
  run_scenario(configJson: string, generatedUtc: string): string
  /** Doubles per streamed sample, checked against the local decoder's layout. */
  sample_stride(): number
}

/** The shape wasm-pack's web target emits for the module default export. */
interface WasmPackModule {
  default: (input?: unknown) => Promise<unknown>
  Session: new (configJson: string) => CoreSession
  core_version: () => string
  run_scenario: (configJson: string, generatedUtc: string) => string
  sample_stride: () => number
}

let pending: Promise<CoreModule> | null = null

async function load(): Promise<CoreModule> {
  const base = import.meta.env.BASE_URL ?? '/'
  const url = `${base}wasm-rust/talon_core.js`
  const mod = (await import(/* @vite-ignore */ url)) as WasmPackModule
  // wasm-pack's web target exports an init function that fetches and
  // instantiates the sibling `.wasm`; nothing may be used until it resolves.
  await mod.default()
  return {
    Session: mod.Session,
    core_version: mod.core_version,
    run_scenario: mod.run_scenario,
    sample_stride: mod.sample_stride,
  }
}

/**
 * Load the WebAssembly kernel, once per session.
 *
 * Concurrent callers share one attempt, and a failure is not cached, so a
 * transient fetch error does not permanently disable the Run view.
 */
export function loadEngine(): Promise<CoreModule> {
  if (pending === null) {
    pending = load().catch((err: unknown) => {
      pending = null
      throw err
    })
  }
  return pending
}
