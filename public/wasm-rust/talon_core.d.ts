/* tslint:disable */
/* eslint-disable */

/**
 * One steppable mission run.
 */
export class Session {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Advance up to `steps` plant steps. Returns how many were advanced.
     *
     * This is the frame-loop entry point: the page asks for a chunk of work,
     * gets the count back, and can measure how long the chunk took.
     */
    advance(steps: number): number;
    /**
     * Live altitude above the ground plane (m, positive up).
     *
     * Ground is the scenario start height; height is the negated NED down
     * coordinate, which is the spine the scene already renders on.
     */
    altitude_m(): number;
    /**
     * The schema 1.1 flight document for the run so far, as JSON.
     *
     * `generated_utc` is supplied by the caller because this crate has no clock.
     */
    document_json(generated_utc: string): string;
    /**
     * Live estimated position, NED (m).
     */
    ekf_position(): Float64Array;
    /**
     * Create a session from a scenario JSON document.
     *
     * The JSON is the same shape served at `public/data/live/configs.json`. If
     * the scenario asks for `auto_arm`, arm and takeoff are queued now so the
     * first control tick starts the mission, matching the batch harness.
     */
    constructor(config_json: string);
    /**
     * Current flight-phase label, lowercase.
     */
    phase(): string;
    /**
     * Convene the estimator at rest, on a negative clock, before the timed run.
     *
     * Idempotent, so a caller that prepares twice does not pay twice or shift
     * the clock. Must be called before the first step for the estimate to be
     * converged when the mission starts.
     */
    prepare(): void;
    /**
     * Advance to the end of the run. Returns the number of steps advanced.
     */
    run_to_end(): number;
    /**
     * Advance one plant step. Returns true once the run is finished.
     */
    step(): boolean;
    /**
     * Headline run numbers, as JSON.
     */
    summary_json(): string;
    /**
     * Live truth position, NED (m).
     */
    truth_position(): Float64Array;
    /**
     * Live truth velocity, NED (m/s).
     */
    truth_velocity(): Float64Array;
    /**
     * Battery state of charge in [0, 1].
     */
    readonly battery_soc: number;
    /**
     * Configured run duration (s).
     */
    readonly duration_s: number;
    /**
     * Whether the run has reached its configured duration.
     */
    readonly finished: boolean;
    /**
     * Whether the convergence pre-roll has run.
     */
    readonly prepared: boolean;
    /**
     * Fraction of the run completed, in [0, 1].
     */
    readonly progress: number;
    /**
     * Scenario id, written to `meta.scenario`.
     */
    readonly scenario_id: string;
    /**
     * Plant steps advanced so far.
     */
    readonly steps_done: number;
    /**
     * Sim time (s). Runs from 0 to `duration_s`; negative during the pre-roll.
     */
    readonly time: number;
    /**
     * Plant steps in the whole run.
     */
    readonly total_steps: number;
}

/**
 * Core version, for the UI to display alongside the run.
 */
export function core_version(): string;

/**
 * Run a scenario to completion and return its flight document.
 *
 * The one-call form of [`Session`], for tests and for any caller that does not
 * need to step the run itself.
 */
export function run_scenario(config_json: string, generated_utc: string): string;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_session_free: (a: number, b: number) => void;
    readonly core_version: () => [number, number];
    readonly run_scenario: (a: number, b: number, c: number, d: number) => [number, number, number, number];
    readonly session_advance: (a: number, b: number) => number;
    readonly session_altitude_m: (a: number) => number;
    readonly session_battery_soc: (a: number) => number;
    readonly session_document_json: (a: number, b: number, c: number) => [number, number, number, number];
    readonly session_duration_s: (a: number) => number;
    readonly session_ekf_position: (a: number) => [number, number];
    readonly session_finished: (a: number) => number;
    readonly session_new: (a: number, b: number) => [number, number, number];
    readonly session_phase: (a: number) => [number, number];
    readonly session_prepare: (a: number) => void;
    readonly session_prepared: (a: number) => number;
    readonly session_progress: (a: number) => number;
    readonly session_run_to_end: (a: number) => number;
    readonly session_scenario_id: (a: number) => [number, number];
    readonly session_step: (a: number) => number;
    readonly session_steps_done: (a: number) => number;
    readonly session_summary_json: (a: number) => [number, number];
    readonly session_time: (a: number) => number;
    readonly session_total_steps: (a: number) => number;
    readonly session_truth_position: (a: number) => [number, number];
    readonly session_truth_velocity: (a: number) => [number, number];
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
