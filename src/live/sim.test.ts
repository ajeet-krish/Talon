import { beforeAll, describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { LiveSim } from './sim'
import { loadWasmModuleForTest, wasmBuilt } from './wasmNode'
import type { KernelHandle, TalonModule } from './kernel'
import type { LiveConfig } from './types'
import type { Vec3 } from './math'

function readJson(rel: string): unknown {
  return JSON.parse(
    readFileSync(decodeURIComponent(new URL(rel, import.meta.url).pathname), 'utf8'),
  )
}

function readJsonOptional(rel: string): unknown | null {
  try {
    return readJson(rel)
  } catch {
    return null
  }
}

const configs = (readJson('../../public/data/live/configs.json') as { configs: LiveConfig[] })
  .configs

function configById(id: string): LiveConfig {
  const found = configs.find((c) => c.id === id)
  if (found === undefined) throw new Error(`missing live config ${id}`)
  return found
}

interface SilReference {
  sample_dt: number
  truth_position: Vec3[]
  phase_timeline: [number, string][]
}

const silReference = readJsonOptional('../../fixtures/sil_reference.json') as Record<
  string,
  SilReference
> | null

// The live run needs the WASM build output; skip when it is absent.
describe.skipIf(!wasmBuilt())('Live in-browser SIL', () => {
  let module: TalonModule
  beforeAll(async () => {
    module = await loadWasmModuleForTest()
  })

  function kernel(): KernelHandle {
    return new module.Kernel()
  }

  it('flies the hover profile to 10 m and converges the EKF', () => {
    const cfg = configById('hover')
    const { data, summary } = new LiveSim(cfg, kernel()).runToCompletion()

    expect(data.meta.sample_count).toBe(Math.round(cfg.duration_s / 0.01))
    expect(summary.mission_complete).toBe(false)
    expect(summary.phases).toContain('takeoff')
    expect(summary.phases).toContain('cruise')

    const last = data.truth_position[data.truth_position.length - 1] as Vec3
    expect(last[2]).toBeCloseTo(-cfg.takeoff_altitude, 0)
    expect(summary.ekf_position_rmse_m).toBeLessThan(0.5)
  })

  it('flies the obstacle circuit around the keep-out spheres', () => {
    const cfg = configById('obstacle_field')
    const { summary } = new LiveSim(cfg, kernel()).runToCompletion()

    expect(summary.phases).toContain('waypoint')
    expect(summary.waypoints_reached).toBeGreaterThanOrEqual(6)
    expect(summary.min_clearance_m).not.toBeNull()
    expect(summary.min_clearance_m as number).toBeGreaterThan(0)
  })

  it.skipIf(silReference === null)(
    'tracks the Python SIL reference trajectory within tolerance',
    () => {
      const cfg = configById('hover')
      const { data } = new LiveSim(cfg, kernel()).runToCompletion()
      const reference = (silReference as Record<string, SilReference>).hover
      const refLast = reference.truth_position[reference.truth_position.length - 1] as Vec3
      const last = data.truth_position[data.truth_position.length - 1] as Vec3
      // The noise realization differs (documented), so this is a statistical
      // agreement check, not a bit-exact one.
      expect(Math.hypot(last[0] - refLast[0], last[1] - refLast[1])).toBeLessThan(3.0)
      expect(Math.abs(last[2] - refLast[2])).toBeLessThan(1.0)
      expect(data.phase_timeline.map((p) => p[1])).toContain('cruise')
    },
  )
})
