/**
 * Timing and thrust constants the live GCS needs before a session exists.
 *
 * The authoritative values live in the Rust core (`talon_core::constants`),
 * which both the browser run (through `wasm/`) and the Python harness (through
 * `bindings/`) execute. Only the few numbers the shell needs up front are
 * mirrored here; everything else is read from the streamed samples.
 *
 * `RunView` asserts the streamed sample stride against
 * `talon_core.sample_stride()` at load, so a drift between these numbers and
 * the core surfaces as a visible message rather than a pacing bug.
 */

/** Vehicle mass (kg): `talon_core::constants::MASS`. */
const MASS = 1.5

/** Gravitational acceleration (m/s^2): `talon_core::constants::GRAVITY`. */
const GRAVITY = 9.81

/** Plant step (s): `talon_core::constants::DT`. */
export const DT = 0.005

/** Control period (s): `talon_core::constants::DT_CONTROL`. */
export const DT_CONTROL = 0.01

/** Collective thrust at the hover equilibrium (N): mass * gravity. */
export const HOVER_THRUST = MASS * GRAVITY
