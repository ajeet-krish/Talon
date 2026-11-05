/**
 * Loader for the Rust GNC core compiled to WebAssembly.
 *
 * The kernel is built by `wasm/build.sh` into `src/wasm-rust/` as an ES
 * module plus a `.wasm`, both committed, and imported here through the
 * module graph: `talon_core.js` as a plain static import, the binary through
 * Vite's `?url` suffix (a hashed asset URL in the build, a served source path
 * in dev). Nothing is fetched from `public/` at runtime, so the dev server
 * resolves the kernel like any other source module instead of refusing to
 * import from `publicDir`.
 *
 * The exported names are aliases of the generated declarations: with
 * `talon_core.d.ts` sitting next to the imported `.js`, `CoreSession` is the
 * generated `Session` class itself. A change on the Rust side therefore fails
 * to typecheck here directly, with no hand-mirror left to drift (the GPS
 * counters once drifted to `number` against the core's `u64`).
 * `engine.contract.test.ts` still reads the generated file from disk and
 * pins the members the live seam depends on.
 */

import init, {
  Session,
  core_version,
  run_scenario,
  sample_stride,
} from '@/wasm-rust/talon_core.js'
import wasmUrl from '@/wasm-rust/talon_core_bg.wasm?url'

/** One steppable mission run, backed by `talon_core::sim::SimRunner`. */
export type CoreSession = Session

/** The loaded WebAssembly module: the generated exports the loader binds. */
export interface CoreModule {
  Session: typeof Session
  core_version: typeof core_version
  run_scenario: typeof run_scenario
  /** Doubles per streamed sample, checked against the local decoder's layout. */
  sample_stride: typeof sample_stride
}

let pending: Promise<CoreModule> | null = null

async function load(): Promise<CoreModule> {
  // wasm-pack's web target init fetches and instantiates the binary at
  // `wasmUrl`; nothing may be used until it resolves.
  await init({ module_or_path: wasmUrl })
  return { Session, core_version, run_scenario, sample_stride }
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
