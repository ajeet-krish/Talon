/**
 * WASM kernel loader for the live run (FlightLab Phase 4).
 *
 * The C++ GNC kernel (dynamics RK4, Dryden wind, cascaded PD, 15-state MEKF)
 * is compiled to WebAssembly with Emscripten + embind (wasm/build.sh) and
 * serves as the numerically heavy engine. This module loads the Emscripten
 * glue via a script tag (the Emscripten MODULARIZE global) and hands back the
 * Kernel constructor.
 *
 * If the module cannot load, the live run is unavailable and the rest of the
 * app (the recorded scenarios) is unaffected: a TypeScript port of the
 * 15-state MEKF would be a second source of truth for the most safety-critical
 * math, so the graceful fallback is to disable Run with a clear message
 * rather than to silently run a lower-fidelity estimator.
 */

/** The embind Kernel surface the loop drives. Scalar in/out for cheap calls. */
export interface KernelHandle {
  // truth
  set_truth_pos(x: number, y: number, z: number): void
  set_truth_vel(x: number, y: number, z: number): void
  set_truth_quat(w: number, x: number, y: number, z: number): void
  set_truth_rate(x: number, y: number, z: number): void
  truth_pos(i: number): number
  truth_vel(i: number): number
  truth_rate(i: number): number
  truth_quat(i: number): number
  /** Body-to-NED DCM, row-major, 9 numbers. */
  truth_dcm_flat(): number[]
  // wind
  wind_configure(
    su: number,
    sv: number,
    sw: number,
    lu: number,
    lv: number,
    lw: number,
    dt: number,
    seed: number,
  ): void
  wind_steady(x: number, y: number, z: number): void
  wind_reset(): void
  last_wind(i: number): number
  // plant
  plant_step(
    thrust: number,
    mx: number,
    my: number,
    mz: number,
    dt: number,
    airspeed: number,
    groundZ: number,
  ): void
  // EKF
  ekf_init_pv(px: number, py: number, pz: number, vx: number, vy: number, vz: number): void
  ekf_init_from_accel(ax: number, ay: number, az: number, yaw: number): void
  ekf_init_level(yaw: number): void
  ekf_predict(
    ax: number,
    ay: number,
    az: number,
    gx: number,
    gy: number,
    gz: number,
    dt: number,
  ): void
  ekf_update(
    px: number,
    py: number,
    pz: number,
    vx: number,
    vy: number,
    vz: number,
  ): number
  ekf_pos(i: number): number
  ekf_vel(i: number): number
  ekf_quat(i: number): number
  ekf_gyro_bias(i: number): number
  ekf_accel_bias(i: number): number
  ekf_pos_var(i: number): number
  ekf_pos_trace(): number
  // controller
  controller_reset(): void
  set_control_target(
    px: number,
    py: number,
    pz: number,
    vx: number,
    vy: number,
    vz: number,
    yaw: number,
  ): void
  control_step(dt: number, gx: number, gy: number, gz: number): void
  cmd_thrust(): number
  cmd_moment(i: number): number
  // diagnostics
  hover_thrust(): number
}

/** The Emscripten Module object: carries the embind Kernel constructor. */
export interface TalonModule {
  Kernel: new () => KernelHandle
}

/** The Emscripten MODULARIZE factory (exposed as the talonKernel global). */
export type KernelFactory = () => Promise<TalonModule>

const GLOBAL_KEY = 'talonKernel'

function globalFactory(): KernelFactory | undefined {
  const g = globalThis as unknown as Record<string, unknown>
  const f = g[GLOBAL_KEY]
  return typeof f === 'function' ? (f as KernelFactory) : undefined
}

/** Resolve the public path of the Emscripten glue (respects the Vite base). */
export function wasmGlueUrl(): string {
  const base = import.meta.env.BASE_URL || '/'
  return `${base.endsWith('/') ? base : `${base}/`}wasm/talon.js`
}

/**
 * Load and instantiate the WASM kernel. Loads the Emscripten glue script if
 * the global factory is not already present, then calls it. Rejects with a
 * clear error when the module cannot load, so the caller can disable Run.
 */
export async function loadWasmModule(): Promise<TalonModule> {
  let factory = globalFactory()
  if (factory === undefined) {
    const url = wasmGlueUrl()
    await new Promise<void>((resolve, reject) => {
      const script = document.createElement('script')
      script.src = url
      script.async = true
      script.onload = () => resolve()
      script.onerror = () =>
        reject(new Error(`Failed to load the WASM kernel script at ${url}`))
      document.head.appendChild(script)
    })
    factory = globalFactory()
  }
  if (factory === undefined) {
    throw new Error('The WASM kernel script loaded but did not expose talonKernel')
  }
  return factory()
}
