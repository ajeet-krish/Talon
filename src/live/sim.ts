/**
 * Live in-browser SIL (FlightLab Phase 4): the TypeScript orchestration loop
 * that mirrors talon/mavlink_sil.py SILRunner one-to-one, driving the WASM
 * kernel for the physics, estimator, and controller.
 *
 * Rates (docs/conventions.md): plant 200 Hz (DT), control 100 Hz (ZOH),
 * GPS 10 Hz, history sampled at the 100 Hz control rate. The loop runs a
 * 5 s convergence pre-roll (negative sim time) before the timed run, exactly
 * like the Python SIL, so the EKF is converged before arming.
 *
 * Scope: the fault-injection paths (GPS outage, EKF divergence, battery
 * drain, RTL forcing) are recorded-only and are not exercisable here; the
 * battery state of charge is held at 1.0 so the low-battery guard never
 * fires. Everything else (takeoff, cruise, waypoint, RTL, land, emergency
 * descent, wind, avoid-configured mission) is the same code path as Python.
 */

import {
  CONVERGENCE_MIN_GPS_UPDATES,
  CONVERGENCE_POS_TRACE_M2,
  DIVERGENCE_GATED_STREAK,
  DIVERGENCE_MIN_TIME_S,
  DIVERGENCE_POS_ERROR_M,
  DT,
  DT_CONTROL,
  DT_GPS,
  PREROLL_MAX_S,
} from './constants'
import { NLGLGuidance } from './guidance'
import { forwardMix, MotorModel } from './motors'
import { WaypointMission } from './mission'
import {
  defaultGPSConfig,
  defaultIMUConfig,
  gpsToNed,
  gpsVelocityNed,
  SensorSuite,
  specificForceFromVelocity,
} from './sensors'
import { FlightPhaseMachine, altitudeAboveHome, velZ } from './stateMachine'
import type { Telemetry } from './stateMachine'
import type { LiveConfig, LiveRunResult, LiveRunSummary } from './types'
import type { Vec3 } from './math'
import type { KernelHandle } from './kernel'
import { clamp } from './math'
import type {
  EventLogEntry,
  FlightData,
  FlightPhase,
  Obstacle,
  PhaseEvent,
} from '@/types/flight'
import { SCHEMA_VERSION } from '@/types/flight'

const STEPS_PER_CONTROL = Math.max(1, Math.round(DT_CONTROL / DT))
const STEPS_PER_GPS = Math.max(1, Math.round(DT_GPS / DT))

const UNITS: Record<string, string> = {
  time: 's',
  position: 'm NED',
  velocity: 'm/s NED',
  acceleration: 'm/s^2',
  angular_rate: 'rad/s',
  thrust: 'N',
  moment: 'N*m',
  wind: 'm/s',
  covariance: 'm^2',
}

function readVec3(read: (i: number) => number): Vec3 {
  return [read(0), read(1), read(2)]
}

function norm(a: Vec3): number {
  return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
}

/**
 * One live SIL run. Construct with a LiveConfig and a loaded WASM kernel,
 * then either call runToCompletion() or drive advance() from a frame loop
 * for progress reporting.
 */
export class LiveSim {
  private readonly cfg: LiveConfig
  private readonly k: KernelHandle
  private readonly machine: FlightPhaseMachine
  private readonly guidance = new NLGLGuidance()
  private readonly motors = new MotorModel()
  private readonly home: Vec3
  private readonly groundZ: number

  private sensors: SensorSuite | null = null
  private mission: WaypointMission | null = null
  private rtlMission: WaypointMission | null = null

  private time = 0.0
  private stepIndex = 0
  private totalSteps = 0
  private prepared = false

  // estimator-derived flags
  private converged = false
  private diverged = false
  private divergedPrev = false
  private gpsAccepted = 0
  private gpsGated = 0
  private gatedStreak = 0

  // autopilot
  private armed = false
  private emergencyDescentActive = false
  private vPrev: Vec3 = [0, 0, 0]
  private lastGyro: Vec3 = [0, 0, 0]
  private commandThrust = 0.0
  private commandMoment: Vec3 = [0, 0, 0]
  private vCmd: Vec3 = [0, 0, 0]
  private reference: Vec3 = [0, 0, 0]
  private referencePhase: FlightPhase | null = null
  private lastWind: Vec3 = [0, 0, 0]

  // phase bookkeeping
  private phaseTimeline: PhaseEvent[] = []
  private phaseEnterTime = 0.0
  private lastPhase: FlightPhase = 'idle'
  private eventQueue: string[] = []

  // history (control rate)
  private readonly histTime: number[] = []
  private readonly histTruthPos: Vec3[] = []
  private readonly histTruthVel: Vec3[] = []
  private readonly histEkfPos: Vec3[] = []
  private readonly histEkfVel: Vec3[] = []
  private readonly histPosVar: Vec3[] = []
  private readonly histGyroBias: Vec3[] = []
  private readonly histAccelBias: Vec3[] = []
  private readonly histWind: Vec3[] = []
  private readonly histThrust: number[] = []
  private readonly histMoment: Vec3[] = []
  private readonly histEct: number[] = []
  private readonly histEctTimes: number[] = []
  private readonly histEctLeg: number[] = []

  constructor(cfg: LiveConfig, kernel: KernelHandle) {
    this.cfg = cfg
    this.k = kernel
    this.machine = new FlightPhaseMachine(cfg.takeoff_altitude)
    this.home = [...cfg.start_position_ned] as Vec3
    this.groundZ = this.home[2]
  }

  get progress(): number {
    return this.totalSteps === 0 ? 0 : this.stepIndex / this.totalSteps
  }

  get finished(): boolean {
    return this.prepared && this.stepIndex >= this.totalSteps
  }

  /** Initialize the truth/estimator and run the convergence pre-roll. */
  prepare(): void {
    if (this.prepared) return
    const cfg = this.cfg
    const k = this.k

    k.set_truth_pos(cfg.start_position_ned[0], cfg.start_position_ned[1], cfg.start_position_ned[2])
    k.set_truth_vel(0, 0, 0)
    k.set_truth_quat(1, 0, 0, 0)
    k.set_truth_rate(0, 0, 0)
    this.motors.reset()
    k.controller_reset()

    const gps = defaultGPSConfig()
    gps.position_noise = cfg.gps_position_noise
    gps.position_noise_vertical = cfg.gps_position_noise_vertical
    gps.velocity_noise = cfg.gps_velocity_noise
    gps.update_rate = 1.0 / DT_GPS
    this.sensors = new SensorSuite(defaultIMUConfig(), gps, cfg.seed)

    // Turbulence always configured (zero sigma freezes the process, matching
    // the Python None branch: no turbulence evolution).
    k.wind_configure(
      cfg.wind_sigma[0],
      cfg.wind_sigma[1],
      cfg.wind_sigma[2],
      200.0,
      200.0,
      50.0,
      DT,
      cfg.seed,
    )
    k.wind_steady(cfg.wind_steady_ned[0], cfg.wind_steady_ned[1], cfg.wind_steady_ned[2])

    // EKF starts level at home at rest (talon SIL __init__).
    k.ekf_init_pv(
      cfg.start_position_ned[0],
      cfg.start_position_ned[1],
      cfg.start_position_ned[2],
      0,
      0,
      0,
    )
    k.ekf_init_level(0)

    this.mission = cfg.waypoints.length > 0 ? new WaypointMission(cfg.waypoints) : null
    this.vPrev = [0, 0, 0]

    // Convergence pre-roll (negative sim time).
    const prerollSteps = Math.round(PREROLL_MAX_S / DT)
    for (let i = 0; i < prerollSteps; i += 1) {
      const t = -PREROLL_MAX_S + i * DT
      this.sensorStep(t, i)
      this.updateDerived(t)
      this.plantStep(0.0, [0, 0, 0])
      if (this.converged) break
    }
    this.time = 0.0

    if (cfg.auto_arm) {
      this.eventQueue.push('arm')
      this.eventQueue.push('takeoff')
    }
    this.totalSteps = Math.round(cfg.duration_s / DT)
    this.stepIndex = 0
    this.prepared = true
  }

  /** Advance one plant step (200 Hz). Returns true when the run is done. */
  advance(): boolean {
    if (!this.prepared) this.prepare()
    if (this.stepIndex >= this.totalSteps) return true

    const i = this.stepIndex
    const isControl = i % STEPS_PER_CONTROL === 0

    this.sensorStep(this.time, i)
    this.updateDerived(this.time)
    this.handleEvents()
    if (isControl) this.controlTick()

    const forces = this.motors.step(this.commandThrust, this.commandMoment, DT)
    const [thrustActual, momentActual] = forwardMix(forces)
    this.plantStep(thrustActual, momentActual)
    if (isControl) this.recordHistory(thrustActual, momentActual)

    this.time += DT
    this.stepIndex += 1
    return this.stepIndex >= this.totalSteps
  }

  /** Run to completion (blocking) and assemble the FlightData document. */
  runToCompletion(): LiveRunResult {
    this.prepare()
    while (!this.advance()) {
      // spin
    }
    return this.finish()
  }

  // ------------------------------------------------------------------
  // Steps
  // ------------------------------------------------------------------

  private sensorStep(tSim: number, i: number): void {
    void tSim
    const k = this.k
    const truthVel = readVec3((idx) => k.truth_vel(idx))
    const dcm = k.truth_dcm_flat()
    const rate = readVec3((idx) => k.truth_rate(idx))
    const fBody = specificForceFromVelocity(this.vPrev, truthVel, dcm, DT)
    this.vPrev = truthVel

    const sensors = this.sensors as SensorSuite
    const imu = sensors.generateImu(fBody, rate, DT)
    this.lastGyro = imu.gyro
    k.ekf_predict(imu.accel[0], imu.accel[1], imu.accel[2], imu.gyro[0], imu.gyro[1], imu.gyro[2], DT)

    if (i % STEPS_PER_GPS !== 0) return
    const truthPos = readVec3((idx) => k.truth_pos(idx))
    const fix = sensors.generateGps(truthPos, truthVel)
    const ned = gpsToNed(fix)
    const vNed = gpsVelocityNed(fix)
    const accepted = k.ekf_update(ned[0], ned[1], ned[2], vNed[0], vNed[1], vNed[2])
    if (accepted === 1) {
      this.gpsAccepted += 1
      this.gatedStreak = 0
    } else {
      this.gpsGated += 1
      this.gatedStreak += 1
    }
  }

  private updateDerived(tSim: number): void {
    const posTrace = this.k.ekf_pos_trace()
    this.converged =
      this.gpsAccepted >= CONVERGENCE_MIN_GPS_UPDATES && posTrace < CONVERGENCE_POS_TRACE_M2
    const posError = norm(this.sub(this.ekfPos(), this.truthPos()))
    this.diverged =
      tSim >= DIVERGENCE_MIN_TIME_S &&
      (posError > DIVERGENCE_POS_ERROR_M || this.gatedStreak >= DIVERGENCE_GATED_STREAK)
  }

  private handleEvents(): void {
    const machine = this.machine
    const phase = machine.phase

    if ((phase === 'cruise' || phase === 'waypoint') && this.mission !== null) {
      this.mission.update(this.ekfPos())
    }

    // Automatic critical fault on the rising edge of ekf_diverged.
    if (this.diverged && !this.divergedPrev) {
      machine.request('critical_fault', this.buildTelemetry(), this.time)
      this.notePhaseChange()
    }
    this.divergedPrev = this.diverged

    let telemetry = this.buildTelemetry()

    while (this.eventQueue.length > 0) {
      const event = this.eventQueue.shift() as string
      if (machine.request(event, telemetry, this.time)) {
        this.notePhaseChange()
        this.armed = machine.phase !== 'idle'
        telemetry = this.buildTelemetry()
      }
    }

    const now = machine.phase
    if (now === 'takeoff' && altitudeAboveHome(telemetry) > this.cfg.takeoff_altitude - 0.5) {
      if (machine.request('altitude_reached', telemetry, this.time)) this.notePhaseChange()
    }
    if (now === 'cruise') {
      if (
        this.mission !== null &&
        this.time - this.phaseEnterTime >= this.cfg.cruise_hold_s
      ) {
        if (machine.request('mission_start', telemetry, this.time)) this.notePhaseChange()
      }
    }
    if (now === 'waypoint') {
      if (telemetry.mission_complete) {
        if (machine.request('mission_complete', telemetry, this.time)) this.notePhaseChange()
      }
      if (telemetry.battery_soc < 0.2) {
        if (machine.request('low_battery', telemetry, this.time)) this.notePhaseChange()
      }
      if (telemetry.gps_outage_duration > 5.0) {
        if (machine.request('gps_lost', telemetry, this.time)) this.notePhaseChange()
      }
    }
    if (now === 'rtl' && this.horizontalToHome(telemetry) < 1.0) {
      if (machine.request('home_reached', telemetry, this.time)) this.notePhaseChange()
    }
    if (now === 'land' && altitudeAboveHome(telemetry) < 0.1 && Math.abs(velZ(telemetry)) < 0.1) {
      if (machine.request('landed', telemetry, this.time)) this.notePhaseChange()
    }
    if (now === 'emergency') {
      if (!this.emergencyDescentActive) {
        this.emergencyDescentActive = true
        telemetry = this.buildTelemetry()
      }
      if (altitudeAboveHome(telemetry) < 0.5) {
        if (machine.request('emergency_landing', telemetry, this.time)) this.notePhaseChange()
      }
    }

    this.armed = machine.phase !== 'idle'
  }

  private controlTick(): void {
    const k = this.k
    const machine = this.machine
    const phase = machine.phase
    const ekfPosition = this.ekfPos()

    if (phase !== this.referencePhase) {
      this.reference = [...ekfPosition] as Vec3
      this.referencePhase = phase
      if (phase === 'rtl') {
        const homeHover: Vec3 = [
          this.home[0],
          this.home[1],
          this.home[2] - this.cfg.takeoff_altitude,
        ]
        this.rtlMission = new WaypointMission([
          { position: homeHover, speed: 5.0, heading: 0.0, radius: 1.0 },
        ])
        this.rtlMission.setStart(ekfPosition)
      }
    }

    if (phase === 'idle' || phase === 'arm') {
      this.commandThrust = 0.0
      this.commandMoment = [0, 0, 0]
      this.vCmd = [0, 0, 0]
      k.controller_reset()
      return
    }

    const cfg = this.guidance.config
    const homeZ = this.home[2]
    const takeoffAlt = this.cfg.takeoff_altitude
    let yawDesired = 0.0

    if (phase === 'takeoff') {
      const zDes = homeZ - takeoffAlt
      const rateZ = clamp(
        cfg.kp_alt * (zDes - ekfPosition[2]),
        -cfg.climb_rate_max,
        cfg.climb_rate_max,
      )
      this.vCmd = [0, 0, rateZ]
      this.reference = this.add(this.reference, this.scale(this.vCmd, DT_CONTROL))
    } else if (phase === 'cruise') {
      const zDes = homeZ - takeoffAlt
      const rateZ = clamp(
        cfg.kp_alt * (zDes - ekfPosition[2]),
        -cfg.climb_rate_max,
        cfg.climb_rate_max,
      )
      this.vCmd = [0, 0, rateZ]
      this.reference = [this.reference[0], this.reference[1], ekfPosition[2] + rateZ * DT_CONTROL]
    } else if (phase === 'waypoint') {
      const mission = this.mission as WaypointMission
      const ekfVelocity = this.ekfVel()
      const { v_des, diagnostics } = this.guidance.computeMission(ekfPosition, ekfVelocity, mission)
      this.vCmd = v_des
      this.reference = this.add(this.reference, this.scale(this.vCmd, DT_CONTROL))
      yawDesired = diagnostics.chi_d
      this.histEct.push(Math.abs(diagnostics.e_ct))
      this.histEctTimes.push(this.time)
      this.histEctLeg.push(mission.activeIndex)
    } else if (phase === 'rtl') {
      const rtl = this.rtlMission as WaypointMission
      const ekfVelocity = this.ekfVel()
      rtl.update(ekfPosition)
      const { v_des, diagnostics } = this.guidance.computeMission(ekfPosition, ekfVelocity, rtl)
      this.vCmd = v_des
      this.reference = this.add(this.reference, this.scale(this.vCmd, DT_CONTROL))
      yawDesired = diagnostics.chi_d
    } else if (phase === 'land') {
      const altitude = Math.max(0.0, homeZ - ekfPosition[2])
      let rateZ: number
      if (altitude > 0.5) rateZ = 0.5
      else if (altitude > 0.15) rateZ = 0.05 + ((altitude - 0.15) / 0.35) * 0.45
      else rateZ = 0.05
      const neError: Vec3 = [this.home[0] - ekfPosition[0], this.home[1] - ekfPosition[1], 0]
      let vNe: [number, number]
      if (Math.sqrt(neError[0] * neError[0] + neError[1] * neError[1]) < 5.0) {
        vNe = [clamp(0.8 * neError[0], -1.0, 1.0), clamp(0.8 * neError[1], -1.0, 1.0)]
      } else {
        vNe = [0.0, 0.0]
      }
      this.vCmd = [vNe[0], vNe[1], rateZ]
      this.reference = this.add(this.reference, this.scale(this.vCmd, DT_CONTROL))
    } else if (phase === 'emergency') {
      this.vCmd = [0, 0, 1.0]
      this.reference = [this.reference[0], this.reference[1], this.reference[2] + DT_CONTROL]
    }

    k.set_control_target(
      this.reference[0],
      this.reference[1],
      this.reference[2],
      this.vCmd[0],
      this.vCmd[1],
      this.vCmd[2],
      yawDesired,
    )
    k.control_step(DT_CONTROL, this.lastGyro[0], this.lastGyro[1], this.lastGyro[2])
    this.commandThrust = Math.max(0.0, k.cmd_thrust())
    this.commandMoment = [k.cmd_moment(0), k.cmd_moment(1), k.cmd_moment(2)]
  }

  private plantStep(thrust: number, moment: Vec3): void {
    const k = this.k
    const vel = readVec3((idx) => k.truth_vel(idx))
    const airspeed = Math.hypot(vel[0], vel[1])
    k.plant_step(thrust, moment[0], moment[1], moment[2], DT, airspeed, this.groundZ)
    this.lastWind = readVec3((idx) => k.last_wind(idx))
  }

  private recordHistory(thrustActual: number, momentActual: Vec3): void {
    const k = this.k
    this.histTime.push(this.time)
    this.histTruthPos.push(readVec3((idx) => k.truth_pos(idx)))
    this.histTruthVel.push(readVec3((idx) => k.truth_vel(idx)))
    this.histEkfPos.push(readVec3((idx) => k.ekf_pos(idx)))
    this.histEkfVel.push(readVec3((idx) => k.ekf_vel(idx)))
    this.histPosVar.push([k.ekf_pos_var(0), k.ekf_pos_var(1), k.ekf_pos_var(2)])
    this.histGyroBias.push(readVec3((idx) => k.ekf_gyro_bias(idx)))
    this.histAccelBias.push(readVec3((idx) => k.ekf_accel_bias(idx)))
    this.histWind.push([...this.lastWind] as Vec3)
    this.histThrust.push(thrustActual)
    this.histMoment.push([...momentActual] as Vec3)
  }

  // ------------------------------------------------------------------
  // Accessors / helpers
  // ------------------------------------------------------------------

  private truthPos(): Vec3 {
    return readVec3((idx) => this.k.truth_pos(idx))
  }

  private ekfPos(): Vec3 {
    return readVec3((idx) => this.k.ekf_pos(idx))
  }

  private ekfVel(): Vec3 {
    return readVec3((idx) => this.k.ekf_vel(idx))
  }

  private add(a: Vec3, b: Vec3): Vec3 {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
  }

  private sub(a: Vec3, b: Vec3): Vec3 {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
  }

  private scale(a: Vec3, s: number): Vec3 {
    return [a[0] * s, a[1] * s, a[2] * s]
  }

  private horizontalToHome(t: Telemetry): number {
    const dn = t.position_ned[0] - t.home_position_ned[0]
    const de = t.position_ned[1] - t.home_position_ned[1]
    return Math.sqrt(dn * dn + de * de)
  }

  private buildTelemetry(): Telemetry {
    return {
      position_ned: this.ekfPos(),
      velocity_ned: this.ekfVel(),
      battery_soc: 1.0,
      ekf_converged: this.converged,
      ekf_diverged: this.diverged,
      gps_outage_duration: 0.0,
      mission_waypoint_count: this.mission === null ? 0 : this.cfg.waypoints.length,
      mission_complete: this.mission === null ? false : this.mission.complete,
      motors_armed: this.armed,
      home_position_ned: this.home,
      emergency_descent_active: this.emergencyDescentActive,
      motor_fault: false,
    }
  }

  private notePhaseChange(): void {
    if (this.machine.phase !== this.lastPhase) {
      this.phaseTimeline.push([this.time, this.machine.phase])
      this.phaseEnterTime = this.time
      this.lastPhase = this.machine.phase
    }
  }

  private minClearance(): number | null {
    const obstacles = this.cfg.obstacles
    if (obstacles.length === 0) return null
    let best = Number.POSITIVE_INFINITY
    for (const p of this.histTruthPos) {
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

  /** Assemble the FlightData document (schema 1.1) plus a run summary. */
  finish(): LiveRunResult {
    const n = this.histTime.length
    let sumSq = 0
    let finalError = 0
    for (let i = 0; i < n; i += 1) {
      const tp = this.histTruthPos[i] as Vec3
      const ep = this.histEkfPos[i] as Vec3
      const d2 =
        (tp[0] - ep[0]) ** 2 + (tp[1] - ep[1]) ** 2 + (tp[2] - ep[2]) ** 2
      sumSq += d2
      finalError = Math.sqrt(d2)
    }
    const obstacles: Obstacle[] = this.cfg.obstacles.map((o) => ({
      id: o.id,
      center_ned: [...o.center_ned] as Vec3,
      radius_m: o.radius_m,
    }))

    const eventLog: EventLogEntry[] = this.machine.eventLog.map((e) => ({
      time: e.time,
      event: e.event,
      from_phase: e.from_phase,
      to_phase: e.to_phase,
      guard_ok: e.guard_ok,
    }))

    const data: FlightData = {
      meta: {
        scenario: this.cfg.id,
        seed: this.cfg.seed,
        schema_version: SCHEMA_VERSION,
        generated_utc: new Date().toISOString(),
        sample_count: n,
        duration_s: this.cfg.duration_s,
        dt: DT_CONTROL,
        units: UNITS,
      },
      time_s: this.histTime,
      truth_position: this.histTruthPos,
      truth_velocity: this.histTruthVel,
      ekf_position: this.histEkfPos,
      ekf_velocity: this.histEkfVel,
      position_var: this.histPosVar,
      gyro_bias: this.histGyroBias,
      accel_bias: this.histAccelBias,
      wind_ned: this.histWind,
      command_thrust: this.histThrust,
      command_moment: this.histMoment,
      e_ct: this.histEct,
      e_ct_times: this.histEctTimes,
      e_ct_leg: this.histEctLeg,
      waypoints_ned: this.cfg.waypoints.map((w) => [...w.position] as Vec3),
      home_ned: [...this.home] as Vec3,
      obstacles,
      phase_timeline: this.phaseTimeline.slice(),
      event_log: eventLog,
    }

    const mission = this.mission
    const summary: LiveRunSummary = {
      duration_s: this.cfg.duration_s,
      waypoint_count: this.cfg.waypoints.length,
      waypoints_reached: mission === null ? 0 : mission.activeIndex + (mission.complete ? 1 : 0),
      mission_complete: mission === null ? false : mission.complete,
      gps_accepted: this.gpsAccepted,
      gps_gated: this.gpsGated,
      final_position_error_m: finalError,
      ekf_position_rmse_m: n === 0 ? 0 : Math.sqrt(sumSq / n),
      min_clearance_m: this.minClearance(),
      phases: this.phaseTimeline.map((p) => p[1]),
    }

    return { data, summary }
  }
}
