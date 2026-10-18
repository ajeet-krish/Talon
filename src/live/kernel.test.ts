import { beforeAll, describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { loadWasmModuleForTest, wasmBuilt } from './wasmNode'
import type { KernelHandle, TalonModule } from './kernel'
import type { Vec3 } from './math'

interface KernelFixture {
  propagate: {
    initial_position: Vec3
    initial_velocity: Vec3
    thrust: number
    moment: Vec3
    wind: Vec3
    dt: number
    final_position: Vec3
    final_velocity: Vec3
    final_quat: number[]
  }
  ekf: {
    init_position: Vec3
    init_velocity: Vec3
    init_yaw: number
    accel: Vec3
    gyro: Vec3
    dt: number
    gps_position: Vec3
    gps_velocity: Vec3
    accepted: boolean
    position: Vec3
    velocity: Vec3
  }
  controller: {
    init_position: Vec3
    init_velocity: Vec3
    init_yaw: number
    reference: Vec3
    v_cmd: Vec3
    yaw: number
    dt: number
    gyro: Vec3
    thrust: number
    moment: Vec3
  }
}

function readJson(rel: string): unknown {
  return JSON.parse(
    readFileSync(decodeURIComponent(new URL(rel, import.meta.url).pathname), 'utf8'),
  )
}

const fixture = (readJson('../../fixtures/live_fixtures.json') as { kernel: KernelFixture })
  .kernel

// The WASM artifacts are build outputs (wasm/build.sh); skip when absent so
// the suite still runs on a checkout without Emscripten.
describe.skipIf(!wasmBuilt())('WASM kernel matches the Python C++ reference', () => {
  let module: TalonModule
  beforeAll(async () => {
    module = await loadWasmModuleForTest()
  })

  function newKernel(): KernelHandle {
    return new module.Kernel()
  }

  it('plants one RK4 step identically to talon.propagate', () => {
    const f = fixture.propagate
    const k = newKernel()
    k.set_truth_pos(f.initial_position[0], f.initial_position[1], f.initial_position[2])
    k.set_truth_vel(f.initial_velocity[0], f.initial_velocity[1], f.initial_velocity[2])
    k.set_truth_quat(1, 0, 0, 0)
    k.set_truth_rate(0, 0, 0)
    k.wind_configure(0, 0, 0, 200, 200, 50, f.dt, 42)
    k.wind_steady(f.wind[0], f.wind[1], f.wind[2])
    // Ground plane at home (z = 0); the vehicle starts 3 m up, so no clamp.
    k.plant_step(f.thrust, f.moment[0], f.moment[1], f.moment[2], f.dt, 0, 0)
    for (let i = 0; i < 3; i += 1) {
      expect(k.truth_pos(i)).toBeCloseTo(f.final_position[i] as number, 9)
      expect(k.truth_vel(i)).toBeCloseTo(f.final_velocity[i] as number, 9)
    }
    for (let i = 0; i < 4; i += 1) {
      expect(k.truth_quat(i)).toBeCloseTo(f.final_quat[i] as number, 9)
    }
  })

  it('predicts and updates the EKF identically', () => {
    const f = fixture.ekf
    const k = newKernel()
    k.ekf_init_pv(
      f.init_position[0],
      f.init_position[1],
      f.init_position[2],
      f.init_velocity[0],
      f.init_velocity[1],
      f.init_velocity[2],
    )
    k.ekf_init_level(f.init_yaw)
    k.ekf_predict(f.accel[0], f.accel[1], f.accel[2], f.gyro[0], f.gyro[1], f.gyro[2], f.dt)
    const accepted = k.ekf_update(
      f.gps_position[0],
      f.gps_position[1],
      f.gps_position[2],
      f.gps_velocity[0],
      f.gps_velocity[1],
      f.gps_velocity[2],
    )
    expect(accepted === 1).toBe(f.accepted)
    for (let i = 0; i < 3; i += 1) {
      expect(k.ekf_pos(i)).toBeCloseTo(f.position[i] as number, 7)
      expect(k.ekf_vel(i)).toBeCloseTo(f.velocity[i] as number, 7)
    }
  })

  it('computes the cascaded PD command identically', () => {
    const f = fixture.controller
    const k = newKernel()
    k.ekf_init_pv(
      f.init_position[0],
      f.init_position[1],
      f.init_position[2],
      f.init_velocity[0],
      f.init_velocity[1],
      f.init_velocity[2],
    )
    k.ekf_init_level(f.init_yaw)
    k.set_control_target(
      f.reference[0],
      f.reference[1],
      f.reference[2],
      f.v_cmd[0],
      f.v_cmd[1],
      f.v_cmd[2],
      f.yaw,
    )
    k.control_step(f.dt, f.gyro[0], f.gyro[1], f.gyro[2])
    expect(k.cmd_thrust()).toBeCloseTo(f.thrust, 7)
    for (let i = 0; i < 3; i += 1) {
      expect(k.cmd_moment(i)).toBeCloseTo(f.moment[i] as number, 7)
    }
  })

  it('reports mass times gravity as hover thrust', () => {
    const k = newKernel()
    expect(k.hover_thrust()).toBeCloseTo(1.5 * 9.81, 6)
  })
})
