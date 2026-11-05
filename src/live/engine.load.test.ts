/**
 * Runtime behaviour of the wasm kernel loader.
 *
 * `engine.contract.test.ts` pins what lives on disk and in the wiring: the
 * generated declaration, the module-graph imports, the type aliases. What it
 * cannot pin is what `loadEngine` does at runtime, because it never imports
 * the module. These tests do, with the generated glue and the `?url` asset
 * mocked, and they cover the four runtime claims the loader's doc comment
 * makes: `init` receives the binary's module URL, the resolved value is the
 * generated exports themselves, concurrent callers share one attempt, and a
 * failure is not cached (so a transient fetch error does not permanently
 * disable the Run view). Type shapes are `tsc`'s job and are not restated
 * here.
 */

import { describe, expect, it, vi } from 'vitest'

const kernel = vi.hoisted(() => ({
  init: vi.fn(),
  Session: class FakeSession {},
  core_version: vi.fn(() => 'talon-test'),
  run_scenario: vi.fn(() => '{"schema_version":"1.1"}'),
  sample_stride: vi.fn(() => 36),
  wasmUrl: '/src/wasm-rust/talon_core_bg.wasm',
}))

vi.mock('@/wasm-rust/talon_core.js', () => ({
  default: kernel.init,
  Session: kernel.Session,
  core_version: kernel.core_version,
  run_scenario: kernel.run_scenario,
  sample_stride: kernel.sample_stride,
}))

vi.mock('@/wasm-rust/talon_core_bg.wasm?url', () => ({ default: kernel.wasmUrl }))

/**
 * A fresh loader instance. `pending` is module state, so every test re-imports
 * `engine.ts` after `vi.resetModules()` instead of sharing one attempt across
 * the file; the mock functions themselves are hoisted and survive the reset.
 */
async function freshLoader(): Promise<typeof import('./engine')> {
  vi.resetModules()
  kernel.init.mockReset()
  kernel.core_version.mockClear()
  kernel.run_scenario.mockClear()
  kernel.sample_stride.mockClear()
  return import('./engine')
}

describe('loadEngine', () => {
  it('awaits init with the module URL of the wasm binary', async () => {
    const { loadEngine } = await freshLoader()
    kernel.init.mockResolvedValue(undefined)

    await loadEngine()

    expect(kernel.init).toHaveBeenCalledTimes(1)
    expect(kernel.init).toHaveBeenCalledWith({ module_or_path: kernel.wasmUrl })
  })

  it('resolves to the generated exports the loader binds', async () => {
    const { loadEngine } = await freshLoader()
    kernel.init.mockResolvedValue(undefined)

    const mod = await loadEngine()

    expect(mod.Session).toBe(kernel.Session)
    expect(mod.core_version).toBe(kernel.core_version)
    expect(mod.run_scenario).toBe(kernel.run_scenario)
    expect(mod.sample_stride).toBe(kernel.sample_stride)
    // the bound members still call through to the generated functions
    expect(mod.sample_stride()).toBe(36)
    expect(mod.core_version()).toBe('talon-test')
  })

  it('hands nothing out before init resolves', async () => {
    const { loadEngine } = await freshLoader()
    let release!: () => void
    kernel.init.mockImplementation(
      () =>
        new Promise<void>((resolve) => {
          release = resolve
        }),
    )

    let settled = false
    const pending = loadEngine().then((mod) => {
      settled = true
      return mod
    })
    await Promise.resolve()
    await Promise.resolve()
    expect(settled).toBe(false)

    release()
    const mod = await pending
    expect(settled).toBe(true)
    expect(mod.Session).toBe(kernel.Session)
  })

  it('shares one attempt across concurrent callers', async () => {
    const { loadEngine } = await freshLoader()
    let release!: () => void
    kernel.init.mockImplementation(
      () =>
        new Promise<void>((resolve) => {
          release = resolve
        }),
    )

    const first = loadEngine()
    const second = loadEngine()
    // the second caller joins the in-flight attempt rather than starting one
    expect(second).toBe(first)
    expect(kernel.init).toHaveBeenCalledTimes(1)

    release()
    const [a, b] = await Promise.all([first, second])
    expect(b).toBe(a)
    expect(kernel.init).toHaveBeenCalledTimes(1)
  })

  it('caches a successful load for the rest of the session', async () => {
    const { loadEngine } = await freshLoader()
    kernel.init.mockResolvedValue(undefined)

    const first = await loadEngine()
    const second = await loadEngine()

    expect(second).toBe(first)
    expect(kernel.init).toHaveBeenCalledTimes(1)
  })

  it('rejects every caller of a shared failed attempt, then retries on the next call', async () => {
    const failure = new Error('wasm fetch failed')
    const { loadEngine } = await freshLoader()
    kernel.init.mockRejectedValueOnce(failure)

    const first = loadEngine()
    const second = loadEngine()
    expect(second).toBe(first)
    await expect(first).rejects.toBe(failure)
    await expect(second).rejects.toBe(failure)
    expect(kernel.init).toHaveBeenCalledTimes(1)

    // the failure is not cached: the next attempt calls init again and succeeds
    kernel.init.mockResolvedValue(undefined)
    const mod = await loadEngine()
    expect(mod.sample_stride()).toBe(36)
    expect(kernel.init).toHaveBeenCalledTimes(2)
  })
})
