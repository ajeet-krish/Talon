/**
 * Node-only loader for the WASM kernel glue, used exclusively by the vitest
 * suites (vitest runs in the node environment). It mirrors kernel.ts's
 * loadWasmModule without a DOM: it reads the Emscripten glue from
 * public/wasm/, evaluates it with a CommonJS module wrapper, and calls the
 * factory. The browser path (script tag + talonKernel global) is unaffected.
 */

import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import type { KernelFactory, TalonModule } from './kernel'

const GLUE_URL = new URL('../../public/wasm/talon.js', import.meta.url)

/** True when the WASM artifacts have been built (public/wasm/talon.js). */
export function wasmBuilt(): boolean {
  try {
    readFileSync(decodeURIComponent(GLUE_URL.pathname), 'utf8')
    return true
  } catch {
    return false
  }
}

/** Instantiate the WASM kernel in the node test environment. */
export function loadWasmModuleForTest(): Promise<TalonModule> {
  const gluePath = decodeURIComponent(GLUE_URL.pathname)
  const dir = gluePath.slice(0, gluePath.lastIndexOf('/'))
  const source = readFileSync(gluePath, 'utf8')
  const mod: { exports: unknown } = { exports: {} }
  const req = createRequire(import.meta.url)
  // Evaluate the Emscripten glue with a CommonJS wrapper (the repo is an ESM
  // package, so the file cannot be required directly).
  const factory = new Function(
    'module',
    'exports',
    'require',
    '__dirname',
    '__filename',
    source,
  ) as (
    mod: unknown,
    exports: unknown,
    require: unknown,
    dirname: string,
    filename: string,
  ) => void
  factory(mod, mod.exports, req, dir, gluePath)
  const kernelFactory = mod.exports as KernelFactory
  return kernelFactory()
}
