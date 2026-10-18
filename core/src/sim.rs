//! Full-stack closed-loop SIL runner.
//!
//! Ported from the previous Python source (`mavlink_sil.py`). This is the piece
//! that closes the loop: truth plant to sensors to estimator to state machine to
//! guidance to controller to motors back to the plant, with fault injection, at
//! the documented rates.
//!
//! Rates and ordering (unchanged from the Python source, and load bearing):
//!
//! * Plant / IMU step `DT = 0.005 s` (200 Hz).
//! * Control tick every `DT_CONTROL / DT = 2` steps (100 Hz, zero-order hold:
//!   the controller output is held across both plant steps).
//! * GPS every `DT_GPS / DT = 20` steps (10 Hz).
//! * Per step: due faults and queued commands, sensors and estimator, derived
//!   flags, state machine events, control tick (when due), motors and plant and
//!   ground clamp, battery and history (when due).
//!
//! One deliberate difference from the Python source, forced by the move: there
//! is no UDP socket, no MAVLink codec and no background thread here. Those are
//! transport, they belong at the edge, and the Python harness still owns them.
//! The runner is a pure stepper, which is exactly what the browser needs: the
//! same object can be advanced one step at a time from a requestAnimationFrame
//! loop or run to completion in a test.
//!
//! Behaviour is statistically equivalent to the recorded Python runs, not
//! bit-identical, because the sensor noise generator is [`crate::rng::Rng`]
//! (splitmix64 plus Box-Muller) rather than numpy's ziggurat stream. Every other
//! quantity, including the fault-injection timings, is the same computation.

use std::collections::VecDeque;
use std::fmt;

use crate::battery::{BatteryConfig, BatteryError, BatteryModel, BatteryState};
use crate::constants::{DT, DT_CONTROL, DT_GPS};
use crate::controller::CascadedController;
use crate::dynamics::propagate;
use crate::ekf::{Ekf, EkfConfig};
use crate::guidance::{NlglConfig, NlglGuidance};
use crate::math;
use crate::mission::{MissionError, WaypointMission};
use crate::motors::{forward_mix, MotorError, MotorModel};
use crate::sensors::{
    gps_to_ned, gps_velocity_ned, specific_force_from_velocity, GpsConfig, ImuConfig, SensorSuite,
};
use crate::state_machine::{FlightPhaseMachine, PhaseError, Telemetry};
use crate::types::{
    to_arr3, to_vec3, Arr3, EventLogEntry, FlightPhase, State, VehicleParams, Waypoint,
};
use crate::wind::DrydenWind;

/// Accepted GPS updates needed before the estimate is called converged.
pub const CONVERGENCE_MIN_GPS_UPDATES: u64 = 20;
/// Position covariance trace (m^2) below which the estimate is converged.
pub const CONVERGENCE_POS_TRACE_M2: f64 = 4.0;
/// No divergence is reported before this sim time (s).
pub const DIVERGENCE_MIN_TIME_S: f64 = 5.0;
/// Position error (m) above which the estimate is diverged.
pub const DIVERGENCE_POS_ERROR_M: f64 = 20.0;
/// Consecutive gated fixes that indicate divergence.
pub const DIVERGENCE_GATED_STREAK: u64 = 5;
/// Maximum pre-roll duration (s) while waiting for estimator convergence.
pub const PREROLL_MAX_S: f64 = 5.0;

/// Everything needed to configure one SIL run.
///
/// Module tuning (EKF, IMU, guidance, gains) stays at the production defaults;
/// these are the scenario-facing knobs. The GPS noise defaults are the Phase 4
/// "measure the estimator, not the GPS floor" setting that every Phase 5
/// scenario reuses.
#[derive(Debug, Clone)]
pub struct SimConfig {
    pub waypoints: Vec<Waypoint>,
    /// Target altitude above home (m, positive up).
    pub takeoff_altitude: f64,
    /// Launch point, NED metres.
    pub start_position_ned: Arr3,
    /// Dryden turbulence sigmas (m/s); all zero disables turbulence.
    pub wind_sigma: [f64; 3],
    /// Steady wind, NED m/s.
    pub wind_steady_ned: Arr3,
    pub battery_initial_soc: f64,
    pub gps_position_noise: f64,
    pub gps_position_noise_vertical: f64,
    pub gps_velocity_noise: f64,
    /// Hold time in CRUISE before releasing the mission (s).
    pub cruise_hold_s: f64,
    pub seed: u64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            waypoints: Vec::new(),
            takeoff_altitude: 10.0,
            start_position_ned: [0.0; 3],
            wind_sigma: [0.0; 3],
            wind_steady_ned: [0.0; 3],
            battery_initial_soc: 1.0,
            gps_position_noise: 0.3,
            gps_position_noise_vertical: 0.5,
            gps_velocity_noise: 0.05,
            cruise_hold_s: 2.0,
            seed: 20_261_012,
        }
    }
}

impl SimConfig {
    /// Validate the scenario parameters.
    pub fn validate(&self) -> Result<(), SimError> {
        if !(self.takeoff_altitude > 0.0) {
            return Err(SimError::NonPositiveTakeoffAltitude(self.takeoff_altitude));
        }
        Ok(())
    }
}

/// Why a run could not be constructed or started.
#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    /// Run duration must be strictly positive.
    NonPositiveDuration(f64),
    /// Takeoff altitude must be strictly positive.
    NonPositiveTakeoffAltitude(f64),
    /// A GPS outage must last a strictly positive time.
    NonPositiveGpsOutage(f64),
    /// The battery configuration was invalid.
    Battery(BatteryError),
    /// The motor configuration was invalid.
    Motor(MotorError),
    /// The mission was invalid.
    Mission(MissionError),
    /// The takeoff altitude was rejected by the phase machine.
    Phase(PhaseError),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveDuration(v) => write!(f, "duration_s must be positive, got {v}"),
            Self::NonPositiveTakeoffAltitude(v) => {
                write!(f, "takeoff_altitude must be positive, got {v}")
            }
            Self::NonPositiveGpsOutage(v) => {
                write!(f, "gps outage duration must be positive, got {v}")
            }
            Self::Battery(e) => write!(f, "battery: {e}"),
            Self::Motor(e) => write!(f, "motor: {e}"),
            Self::Mission(e) => write!(f, "mission: {e}"),
            Self::Phase(e) => write!(f, "phase: {e}"),
        }
    }
}

impl std::error::Error for SimError {}

impl From<BatteryError> for SimError {
    fn from(e: BatteryError) -> Self {
        Self::Battery(e)
    }
}
impl From<MotorError> for SimError {
    fn from(e: MotorError) -> Self {
        Self::Motor(e)
    }
}
impl From<MissionError> for SimError {
    fn from(e: MissionError) -> Self {
        Self::Mission(e)
    }
}
impl From<PhaseError> for SimError {
    fn from(e: PhaseError) -> Self {
        Self::Phase(e)
    }
}

/// The estimator divergence injections the RTL and FDIR scenarios use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DivergenceMode {
    /// One-shot +25 m offset on the nominal north position. The innovation gate
    /// then rejects every subsequent fix, so the corruption persists and
    /// divergence trips on the position-error clause within one control step.
    CorruptBias,
    /// Stop feeding GPS to the filter; dead-reckon forever.
    SuppressUpdates,
    /// Offset every GPS position fed to the filter by +50 m north, so a
    /// converged filter gates every fix out and the gated-streak clause trips.
    AdversarialGps,
}

/// Histories and logs from one run.
///
/// Histories are sampled at the control rate (100 Hz) except
/// `gps_update_times` (one entry per accepted GPS update, 10 Hz) and `e_ct` /
/// `e_ct_times` / `e_ct_leg` (one entry per WAYPOINT-phase control tick, since
/// cross-track error only exists while a leg is flown). `phase_timeline` holds
/// `(sim_time, phase)` change events; the phase at any sample is the last entry
/// at or before that sample's time.
#[derive(Debug, Clone, Default)]
pub struct SimResult {
    pub time_s: Vec<f64>,
    pub truth_position: Vec<Arr3>,
    pub truth_velocity: Vec<Arr3>,
    pub ekf_position: Vec<Arr3>,
    pub ekf_velocity: Vec<Arr3>,
    pub phase_timeline: Vec<(f64, FlightPhase)>,
    pub event_log: Vec<EventLogEntry>,
    pub e_ct: Vec<f64>,
    pub gps_update_times: Vec<f64>,
    pub gps_skipped: u64,
    pub gps_gated: u64,
    pub gps_accepted: u64,
    pub home_ned: Arr3,
    pub waypoints_ned: Vec<Arr3>,
    pub command_thrust: Vec<f64>,
    pub command_moment: Vec<Arr3>,
    pub wind_ned: Vec<Arr3>,
    pub position_var: Vec<Arr3>,
    pub gyro_bias: Vec<Arr3>,
    pub accel_bias: Vec<Arr3>,
    pub e_ct_times: Vec<f64>,
    pub e_ct_leg: Vec<f64>,
    pub ekf_converged: bool,
    pub ekf_diverged: bool,
    pub duration_s: f64,
}

impl SimResult {
    /// The last phase on the timeline.
    pub fn final_phase(&self) -> FlightPhase {
        self.phase_timeline
            .last()
            .map(|(_, p)| *p)
            .unwrap_or(FlightPhase::Idle)
    }

    /// All phases that appeared on the timeline.
    pub fn phases_visited(&self) -> Vec<FlightPhase> {
        let mut seen = Vec::new();
        for (_, p) in &self.phase_timeline {
            if !seen.contains(p) {
                seen.push(*p);
            }
        }
        seen
    }

    /// Final truth altitude above the ground plane (m, positive up).
    pub fn final_truth_altitude_m(&self) -> f64 {
        let ground = self.home_ned[2];
        self.truth_position
            .last()
            .map(|p| ground - p[2])
            .unwrap_or(0.0)
    }

    /// Number of waypoints the mission reached (0 when there was no mission).
    pub fn waypoints_reached(&self) -> u64 {
        self.e_ct_leg
            .last()
            .map(|leg| *leg as u64 + 1)
            .unwrap_or(0)
    }
}

/// Full-stack closed-loop SIL, single-threaded and steppable.
///
/// Build with [`SimRunner::new`], configure faults, then either [`SimRunner::run`]
/// to completion or [`SimRunner::step_once`] repeatedly for an interactive loop.
pub struct SimRunner {
    config: SimConfig,
    params: VehicleParams,
    controller: CascadedController,
    guidance: NlglGuidance,
    mission: Option<WaypointMission>,
    rtl_mission: Option<WaypointMission>,

    truth: State,
    home: Arr3,
    ground_z: f64,
    motors: MotorModel,

    sensors: SensorSuite,
    ekf: Ekf,

    battery: BatteryModel,
    battery_state: BatteryState,
    wind: Option<DrydenWind>,
    wind_steady: Arr3,

    state_machine: FlightPhaseMachine,
    armed: bool,
    emergency_descent_active: bool,

    gps_outage_start: f64,
    gps_outage_end: f64,
    fault_suppress_updates: bool,
    adversarial_gps_offset: Option<Arr3>,

    gps_accepted: u64,
    gps_gated: u64,
    gps_skipped: u64,
    gated_streak: u64,
    ekf_converged: bool,
    ekf_diverged: bool,
    ekf_diverged_prev: bool,

    time: f64,
    v_prev: Arr3,
    last_gyro: Arr3,
    reference: Arr3,
    reference_phase: Option<FlightPhase>,
    v_cmd: Arr3,
    command_thrust: f64,
    command_moment: Arr3,
    last_thrust_actual: f64,
    last_moment_actual: Arr3,
    last_wind_ned: Arr3,
    phase_enter_time: f64,
    last_phase: FlightPhase,

    event_queue: VecDeque<String>,
    scheduled: Vec<(f64, Box<dyn FnOnce(&mut SimRunner)>)>,

    history_time: Vec<f64>,
    history_truth_pos: Vec<Arr3>,
    history_truth_vel: Vec<Arr3>,
    history_ekf_pos: Vec<Arr3>,
    history_ekf_vel: Vec<Arr3>,
    history_e_ct: Vec<f64>,
    history_e_ct_times: Vec<f64>,
    history_e_ct_leg: Vec<f64>,
    history_gps_times: Vec<f64>,
    history_thrust: Vec<f64>,
    history_moment: Vec<Arr3>,
    history_wind: Vec<Arr3>,
    history_pos_var: Vec<Arr3>,
    history_gyro_bias: Vec<Arr3>,
    history_accel_bias: Vec<Arr3>,
    phase_timeline: Vec<(f64, FlightPhase)>,
}

impl SimRunner {
    /// Build a runner from a scenario configuration.
    pub fn new(config: SimConfig) -> Result<Self, SimError> {
        config.validate()?;
        let params = VehicleParams::default();
        let controller = CascadedController::with_defaults();
        let guidance = NlglGuidance::new(NlglConfig::default());

        let mission = if config.waypoints.is_empty() {
            None
        } else {
            Some(WaypointMission::new(config.waypoints.clone())?)
        };

        let mut truth = State::default();
        truth.position = to_vec3(config.start_position_ned);
        let home = config.start_position_ned;
        let ground_z = home[2];

        let sensors = SensorSuite::new(
            ImuConfig::default(),
            GpsConfig {
                position_noise: config.gps_position_noise,
                position_noise_vertical: config.gps_position_noise_vertical,
                velocity_noise: config.gps_velocity_noise,
                update_rate: 1.0 / DT_GPS,
            },
            config.seed,
        );

        let mut ekf = Ekf::new(EkfConfig::default());
        ekf.initialize_level(to_vec3(home), nalgebra::Vector3::zeros(), 0.0);

        let battery = BatteryModel::new(BatteryConfig::default(), config.battery_initial_soc)?;
        let battery_state = BatteryState {
            soc: battery.soc(),
            voltage: 11.1,
            current_a: 0.0,
        };

        let any_turbulence = config.wind_sigma.iter().any(|s| *s > 0.0);
        let wind = if any_turbulence {
            Some(DrydenWind::new(
                config.wind_sigma,
                [200.0, 200.0, 50.0],
                DT,
                config.seed,
            ))
        } else {
            None
        };

        let state_machine = FlightPhaseMachine::new(config.takeoff_altitude)?;

        Ok(Self {
            reference: config.start_position_ned,
            wind_steady: config.wind_steady_ned,
            config,
            params,
            controller,
            guidance,
            mission,
            rtl_mission: None,
            truth,
            home,
            ground_z,
            motors: MotorModel::with_defaults(),
            sensors,
            ekf,
            battery,
            battery_state,
            wind,
            state_machine,
            armed: false,
            emergency_descent_active: false,
            gps_outage_start: f64::NEG_INFINITY,
            gps_outage_end: f64::NEG_INFINITY,
            fault_suppress_updates: false,
            adversarial_gps_offset: None,
            gps_accepted: 0,
            gps_gated: 0,
            gps_skipped: 0,
            gated_streak: 0,
            ekf_converged: false,
            ekf_diverged: false,
            ekf_diverged_prev: false,
            time: 0.0,
            v_prev: [0.0; 3],
            last_gyro: [0.0; 3],
            reference_phase: None,
            v_cmd: [0.0; 3],
            command_thrust: 0.0,
            command_moment: [0.0; 3],
            last_thrust_actual: 0.0,
            last_moment_actual: [0.0; 3],
            last_wind_ned: [0.0; 3],
            phase_enter_time: 0.0,
            last_phase: FlightPhase::Idle,
            event_queue: VecDeque::new(),
            scheduled: Vec::new(),
            history_time: Vec::new(),
            history_truth_pos: Vec::new(),
            history_truth_vel: Vec::new(),
            history_ekf_pos: Vec::new(),
            history_ekf_vel: Vec::new(),
            history_e_ct: Vec::new(),
            history_e_ct_times: Vec::new(),
            history_e_ct_leg: Vec::new(),
            history_gps_times: Vec::new(),
            history_thrust: Vec::new(),
            history_moment: Vec::new(),
            history_wind: Vec::new(),
            history_pos_var: Vec::new(),
            history_gyro_bias: Vec::new(),
            history_accel_bias: Vec::new(),
            phase_timeline: vec![(0.0, FlightPhase::Idle)],
        })
    }

    /// The live flight phase machine.
    pub fn state_machine(&self) -> &FlightPhaseMachine {
        &self.state_machine
    }

    /// Current sim time (s). Negative only during the pre-roll.
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Current truth state.
    pub fn truth(&self) -> &State {
        &self.truth
    }

    /// Current estimator position (NED, m).
    pub fn ekf_position(&self) -> Arr3 {
        to_arr3(&self.ekf.nominal().position)
    }

    /// Current battery state of charge.
    pub fn battery_soc(&self) -> f64 {
        self.battery.soc()
    }

    /// Queue a state machine event. `None` applies at the next opportunity; a
    /// sim time schedules it against the clock, which starts at 0 after the
    /// convergence pre-roll.
    pub fn request_event(&mut self, event: &str, at_time: Option<f64>) {
        match at_time {
            None => self.event_queue.push_back(event.to_string()),
            Some(t) => {
                let owned = event.to_string();
                self.schedule(t, Box::new(move |s: &mut SimRunner| s.event_queue.push_back(owned)));
            }
        }
    }

    /// Run `action` once sim time reaches `at_time`.
    pub fn schedule(&mut self, at_time: f64, action: Box<dyn FnOnce(&mut SimRunner)>) {
        self.scheduled.push((at_time, action));
    }

    /// Black out GPS for `duration_s` sim seconds from now.
    ///
    /// During the outage no fix is generated or fed to the filter, so the filter
    /// dead-reckons on IMU only. Repeated calls extend the outage.
    pub fn inject_gps_outage(&mut self, duration_s: f64) -> Result<(), SimError> {
        if duration_s <= 0.0 {
            return Err(SimError::NonPositiveGpsOutage(duration_s));
        }
        if self.time >= self.gps_outage_end {
            self.gps_outage_start = self.time;
        }
        self.gps_outage_end = f64::max(self.gps_outage_end, self.time + duration_s);
        Ok(())
    }

    /// Force the battery state of charge.
    pub fn inject_battery_soc(&mut self, soc: f64) {
        self.battery.force_soc(soc);
        self.battery_state = BatteryState {
            soc: self.battery.soc(),
            voltage: 11.1,
            current_a: 0.0,
        };
    }

    /// Force the estimator into a diverged state.
    pub fn inject_ekf_divergence(&mut self, mode: DivergenceMode) {
        match mode {
            DivergenceMode::CorruptBias => {
                self.ekf.mutable_nominal().position[0] += 25.0;
            }
            DivergenceMode::SuppressUpdates => {
                self.fault_suppress_updates = true;
            }
            DivergenceMode::AdversarialGps => {
                self.adversarial_gps_offset = Some([50.0, 0.0, 0.0]);
            }
        }
    }

    /// Run the convergence pre-roll, then `duration_s` sim seconds.
    ///
    /// A runner is intended for one run: histories accumulate.
    pub fn run(&mut self, duration_s: f64) -> Result<SimResult, SimError> {
        if duration_s <= 0.0 {
            return Err(SimError::NonPositiveDuration(duration_s));
        }
        self.preroll();
        let steps = (duration_s / DT).round() as usize;
        let steps_per_control = ((DT_CONTROL / DT).round() as usize).max(1);
        let steps_per_gps = ((DT_GPS / DT).round() as usize).max(1);
        for i in 0..steps {
            self.step(i, steps_per_control, steps_per_gps);
        }
        Ok(self.finish(duration_s))
    }

    /// Advance exactly one plant step, for an interactive or streaming loop.
    ///
    /// Does not pre-roll: call [`SimRunner::preroll`] once first if the estimate
    /// needs to converge before the clock starts.
    pub fn step_once(&mut self, i: usize) {
        let steps_per_control = ((DT_CONTROL / DT).round() as usize).max(1);
        let steps_per_gps = ((DT_GPS / DT).round() as usize).max(1);
        self.step(i, steps_per_control, steps_per_gps);
    }

    /// Converge the estimate at rest before the main loop, on a negative clock.
    pub fn preroll(&mut self) {
        self.time = -PREROLL_MAX_S;
        let total_steps = (PREROLL_MAX_S / DT).round() as usize;
        let steps_per_gps = ((DT_GPS / DT).round() as usize).max(1);
        for i in 0..total_steps {
            self.sensor_step(i, steps_per_gps);
            self.update_derived();
            self.plant_step(None, None);
            // Advance the negative clock per step so accepted-update timestamps
            // count up from -PREROLL_MAX_S instead of sitting at -5.0.
            self.time += DT;
            if self.ekf_converged {
                break;
            }
        }
        self.time = 0.0;
    }

    // ------------------------------------------------------------------
    // One step
    // ------------------------------------------------------------------

    fn step(&mut self, i: usize, steps_per_control: usize, steps_per_gps: usize) {
        let is_control = i % steps_per_control == 0;

        // 1. Due scheduled faults, then queued events.
        self.fire_scheduled();

        // 2. Sensors and estimator.
        self.sensor_step(i, steps_per_gps);

        // 3. Derived estimate flags from the fresh estimate.
        self.update_derived();

        // 4. State machine: critical fault plus auto events.
        self.handle_events();

        // 5-6. Phase setpoints plus cascaded PD (100 Hz, zero-order hold).
        if is_control {
            self.control_tick();
        }

        // 7. Motors, wind, truth, ground clamp.
        let forces = self.motors.step(self.command_thrust, self.command_moment, DT);
        let (thrust_actual, moment_actual) = forward_mix(forces);
        self.plant_step(
            Some(thrust_actual),
            Some(nalgebra::Vector3::new(
                moment_actual[0],
                moment_actual[1],
                moment_actual[2],
            )),
        );

        // 8. Battery at the control rate from the actual mixed thrust.
        if is_control {
            self.battery_state = self.battery.update(thrust_actual, DT_CONTROL);
            self.record_history();
        }

        self.time += DT;
    }

    /// IMU and EKF predict every step; GPS update when due.
    fn sensor_step(&mut self, i: usize, steps_per_gps: usize) {
        let truth_velocity = self.truth.velocity;
        let rotation = self.truth.orientation.to_dcm();
        let f_body = specific_force_from_velocity(
            &to_vec3(self.v_prev),
            &truth_velocity,
            &rotation,
            DT,
        );
        self.v_prev = to_arr3(&truth_velocity);
        let imu = self
            .sensors
            .generate_imu(&self.truth.angular_rate, to_arr3(&f_body), DT);
        self.last_gyro = imu.gyro;
        let accel = nalgebra::Vector3::new(imu.accel[0], imu.accel[1], imu.accel[2]);
        let gyro = nalgebra::Vector3::new(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
        self.ekf.predict(&accel, &gyro, DT);

        if i % steps_per_gps != 0 {
            return;
        }
        if self.time < self.gps_outage_end || self.fault_suppress_updates {
            self.gps_skipped += 1;
            return;
        }
        let gps = self
            .sensors
            .generate_gps(&self.truth.position, &self.truth.velocity);
        let mut position = gps_to_ned(&gps);
        let velocity = gps_velocity_ned(&gps);
        if let Some(offset) = self.adversarial_gps_offset {
            position = [
                position[0] + offset[0],
                position[1] + offset[1],
                position[2] + offset[2],
            ];
        }
        if self.ekf.update(&to_vec3(position), &to_vec3(velocity)) {
            self.gps_accepted += 1;
            self.gated_streak = 0;
            self.history_gps_times.push(self.time);
        } else {
            self.gps_gated += 1;
            self.gated_streak += 1;
        }
    }

    /// One truth RK4 step plus wind and the ground clamp.
    fn plant_step(&mut self, thrust_actual: Option<f64>, moment_actual: Option<nalgebra::Vector3<f64>>) {
        let (thrust, moment) = match (thrust_actual, moment_actual) {
            (Some(t), Some(m)) => (t, m),
            _ => {
                // Idle (pre-roll): motors commanded to zero this step.
                let forces = self.motors.step(0.0, [0.0; 3], DT);
                let (t, m) = forward_mix(forces);
                (t, nalgebra::Vector3::new(m[0], m[1], m[2]))
            }
        };
        let wind_ned = match self.wind.as_mut() {
            Some(wind) => {
                let airspeed = math::sqrt(
                    self.truth.velocity[0] * self.truth.velocity[0]
                        + self.truth.velocity[1] * self.truth.velocity[1],
                );
                let turbulence = wind.step(airspeed);
                turbulence + to_vec3(self.wind_steady)
            }
            None => to_vec3(self.wind_steady),
        };
        // Mirror the applied actuator output and wind for record_history: what
        // the plant actually felt this step.
        self.last_thrust_actual = thrust;
        self.last_moment_actual = to_arr3(&moment);
        self.last_wind_ned = to_arr3(&wind_ned);
        self.truth = propagate(&self.truth, thrust, &moment, &wind_ned, DT, &self.params);
        self.apply_ground_clamp();
    }

    /// Simple ground contact: cap z at home and kill downward velocity.
    fn apply_ground_clamp(&mut self) {
        if self.truth.position[2] > self.ground_z {
            self.truth.position[2] = self.ground_z;
            if self.truth.velocity[2] > 0.0 {
                self.truth.velocity[2] = 0.0;
            }
        }
    }

    /// Recompute the converged and diverged flags.
    fn update_derived(&mut self) {
        let covariance = self.ekf.covariance();
        let pos_trace = covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)];
        self.ekf_converged = self.gps_accepted >= CONVERGENCE_MIN_GPS_UPDATES
            && pos_trace < CONVERGENCE_POS_TRACE_M2;
        let position_error = (self.ekf.nominal().position - self.truth.position).norm();
        self.ekf_diverged = self.time >= DIVERGENCE_MIN_TIME_S
            && (position_error > DIVERGENCE_POS_ERROR_M
                || self.gated_streak >= DIVERGENCE_GATED_STREAK);
    }

    fn gps_outage_duration(&self) -> f64 {
        if self.time >= self.gps_outage_end {
            return 0.0;
        }
        self.time - self.gps_outage_start
    }

    /// Record an accepted phase change on the timeline.
    ///
    /// Multiple guarded transitions can be accepted within one control step
    /// (arm then takeoff at t = 0), and each accepted change gets its own entry
    /// so a post-flight review never skips a phase the vehicle passed through.
    fn note_phase_change(&mut self) {
        if self.state_machine.phase() != self.last_phase {
            self.phase_timeline.push((self.time, self.state_machine.phase()));
            self.phase_enter_time = self.time;
            self.last_phase = self.state_machine.phase();
        }
    }

    fn build_telemetry(&self) -> Telemetry {
        Telemetry {
            position_ned: to_arr3(&self.ekf.nominal().position),
            velocity_ned: to_arr3(&self.ekf.nominal().velocity),
            battery_soc: self.battery.soc(),
            ekf_converged: self.ekf_converged,
            ekf_diverged: self.ekf_diverged,
            gps_outage_duration: self.gps_outage_duration(),
            mission_waypoint_count: if self.mission.is_some() {
                self.config.waypoints.len()
            } else {
                0
            },
            mission_complete: self.mission.as_ref().map(|m| m.complete()).unwrap_or(false),
            motors_armed: self.armed,
            home_position_ned: self.home,
            emergency_descent_active: self.emergency_descent_active,
            motor_fault: false,
        }
    }

    fn handle_events(&mut self) {
        // Mission acceptance uses the fresh estimator position.
        let phase = self.state_machine.phase();
        if phase == FlightPhase::Cruise || phase == FlightPhase::Waypoint {
            let ekf_position = self.ekf_position();
            if let Some(mission) = self.mission.as_mut() {
                mission.update(ekf_position);
            }
        }

        // Automatic critical_fault fires on the RISING EDGE of divergence: one
        // response per fault episode. A diverged estimate that never recovers
        // never re-fires, so the emergency sequence owns the vehicle until
        // landed; when the estimate recovers the trigger re-arms.
        if self.ekf_diverged && !self.ekf_diverged_prev {
            let telemetry = self.build_telemetry();
            let t = self.time;
            self.state_machine.request("critical_fault", &telemetry, t);
            self.note_phase_change();
        }
        self.ekf_diverged_prev = self.ekf_diverged;

        let mut telemetry = self.build_telemetry();

        // Queued events use the fresh snapshot; each accepted event refreshes
        // it so a same-step follow-up (arm then takeoff) sees the new state.
        while let Some(event) = self.event_queue.pop_front() {
            let t = self.time;
            if self.state_machine.request(&event, &telemetry, t) {
                self.note_phase_change();
                self.armed = self.state_machine.phase() != FlightPhase::Idle;
                telemetry = self.build_telemetry();
            }
        }

        let phase = self.state_machine.phase();
        let t = self.time;
        if phase == FlightPhase::Takeoff
            && telemetry.altitude_above_home() > self.config.takeoff_altitude - 0.5
        {
            if self.state_machine.request("altitude_reached", &telemetry, t) {
                self.note_phase_change();
            }
        }
        if phase == FlightPhase::Cruise
            && self.mission.is_some()
            && (self.time - self.phase_enter_time) >= self.config.cruise_hold_s
        {
            if self.state_machine.request("mission_start", &telemetry, t) {
                self.note_phase_change();
            }
        }
        if phase == FlightPhase::Waypoint {
            if telemetry.mission_complete {
                if self.state_machine.request("mission_complete", &telemetry, t) {
                    self.note_phase_change();
                }
            }
            if telemetry.battery_soc < 0.20 {
                if self.state_machine.request("low_battery", &telemetry, t) {
                    self.note_phase_change();
                }
            }
            if telemetry.gps_outage_duration > 5.0 {
                if self.state_machine.request("gps_lost", &telemetry, t) {
                    self.note_phase_change();
                }
            }
        }
        if phase == FlightPhase::Rtl && telemetry.horizontal_dist_to_home() < 1.0 {
            if self.state_machine.request("home_reached", &telemetry, t) {
                self.note_phase_change();
            }
        }
        if phase == FlightPhase::Land
            && telemetry.altitude_above_home() < 0.1
            && telemetry.vel_z().abs() < 0.1
        {
            if self.state_machine.request("landed", &telemetry, t) {
                self.note_phase_change();
            }
        }
        if phase == FlightPhase::Emergency {
            // Latch the descent flag on the first step in EMERGENCY so the
            // emergency_landing guard and LAND entry can see it.
            if !self.emergency_descent_active {
                self.emergency_descent_active = true;
                telemetry = self.build_telemetry();
            }
            if telemetry.altitude_above_home() < 0.5 {
                if self.state_machine.request("emergency_landing", &telemetry, t) {
                    self.note_phase_change();
                }
            }
        }

        // The armed flag follows the phase; IDLE disarms.
        self.armed = self.state_machine.phase() != FlightPhase::Idle;
    }

    fn estimated_state(&self, gyro: &Arr3) -> State {
        let nominal = self.ekf.nominal();
        State {
            position: nominal.position,
            velocity: nominal.velocity,
            orientation: nominal.attitude,
            angular_rate: self.ekf.estimated_body_rates(&to_vec3(*gyro)),
        }
    }

    fn control_tick(&mut self) {
        let phase = self.state_machine.phase();
        let ekf_position = self.ekf_position();

        if self.reference_phase != Some(phase) {
            // Fresh reference on every phase entry.
            self.reference = ekf_position;
            self.reference_phase = Some(phase);
            if phase == FlightPhase::Rtl {
                let home_hover = [
                    self.home[0],
                    self.home[1],
                    self.home[2] - self.config.takeoff_altitude,
                ];
                let mut rtl = WaypointMission::new(vec![Waypoint::new(
                    home_hover,
                    5.0,
                    0.0,
                    1.0,
                )])
                .expect("a single waypoint is a valid mission");
                rtl.set_start(ekf_position);
                self.rtl_mission = Some(rtl);
            }
        }

        if phase == FlightPhase::Idle || phase == FlightPhase::Arm {
            // Hold on the ground: zero thrust, reset the PD internals.
            self.command_thrust = 0.0;
            self.command_moment = [0.0; 3];
            self.v_cmd = [0.0; 3];
            self.controller.reset();
            return;
        }

        let estimated = self.estimated_state(&self.last_gyro);
        let mut yaw_desired = 0.0;

        match phase {
            FlightPhase::Takeoff => {
                let cfg = *self.guidance.config();
                let z_des = self.home[2] - self.config.takeoff_altitude;
                let rate_z = math::clamp(
                    cfg.kp_alt * (z_des - ekf_position[2]),
                    -cfg.climb_rate_max,
                    cfg.climb_rate_max,
                );
                self.v_cmd = [0.0, 0.0, rate_z];
                self.reference = [
                    self.reference[0] + self.v_cmd[0] * DT_CONTROL,
                    self.reference[1] + self.v_cmd[1] * DT_CONTROL,
                    self.reference[2] + self.v_cmd[2] * DT_CONTROL,
                ];
            }
            FlightPhase::Cruise => {
                // Hold the entry position; altitude via a P channel like NLGL.
                let cfg = *self.guidance.config();
                let z_des = self.home[2] - self.config.takeoff_altitude;
                let rate_z = math::clamp(
                    cfg.kp_alt * (z_des - ekf_position[2]),
                    -cfg.climb_rate_max,
                    cfg.climb_rate_max,
                );
                self.v_cmd = [0.0, 0.0, rate_z];
                self.reference = [
                    self.reference[0],
                    self.reference[1],
                    ekf_position[2] + rate_z * DT_CONTROL,
                ];
            }
            FlightPhase::Waypoint => {
                let ekf_velocity = to_arr3(&self.ekf.nominal().velocity);
                let mission = self
                    .mission
                    .as_ref()
                    .expect("WAYPOINT implies a mission");
                let (v_des, diagnostics) =
                    self.guidance
                        .compute_mission(ekf_position, ekf_velocity, mission);
                let leg = mission.active_index() as f64;
                self.v_cmd = v_des;
                self.reference = [
                    self.reference[0] + v_des[0] * DT_CONTROL,
                    self.reference[1] + v_des[1] * DT_CONTROL,
                    self.reference[2] + v_des[2] * DT_CONTROL,
                ];
                yaw_desired = diagnostics.chi_d;
                self.history_e_ct.push(diagnostics.e_ct.abs());
                self.history_e_ct_times.push(self.time);
                self.history_e_ct_leg.push(leg);
            }
            FlightPhase::Rtl => {
                let ekf_velocity = to_arr3(&self.ekf.nominal().velocity);
                let rtl = self
                    .rtl_mission
                    .as_mut()
                    .expect("RTL implies an RTL mission");
                rtl.update(ekf_position);
                let (v_des, diagnostics) =
                    self.guidance.compute_mission(ekf_position, ekf_velocity, rtl);
                self.v_cmd = v_des;
                self.reference = [
                    self.reference[0] + v_des[0] * DT_CONTROL,
                    self.reference[1] + v_des[1] * DT_CONTROL,
                    self.reference[2] + v_des[2] * DT_CONTROL,
                ];
                yaw_desired = diagnostics.chi_d;
            }
            FlightPhase::Land => {
                // Descent profile: 0.5 m/s above 0.5 m, tapering to a 0.05 m/s
                // creep below 0.15 m so |vz| < 0.1 is reachable for the landed
                // guard while the ground clamp absorbs the last bit.
                let altitude = f64::max(0.0, self.home[2] - ekf_position[2]);
                let rate_z = if altitude > 0.5 {
                    0.5
                } else if altitude > 0.15 {
                    0.05 + (altitude - 0.15) / 0.35 * 0.45
                } else {
                    0.05
                };
                // Gentle horizontal re-centering on home, NEAR-FIELD ONLY: a
                // large horizontal error (say one manufactured by a corrupted
                // estimate) is not chased during landing, so a fault cannot
                // command a long lateral translation while touching down.
                let ne_error = [
                    self.home[0] - ekf_position[0],
                    self.home[1] - ekf_position[1],
                ];
                let v_ne = if math::sqrt(ne_error[0] * ne_error[0] + ne_error[1] * ne_error[1]) < 5.0
                {
                    [
                        math::clamp(0.8 * ne_error[0], -1.0, 1.0),
                        math::clamp(0.8 * ne_error[1], -1.0, 1.0),
                    ]
                } else {
                    [0.0, 0.0]
                };
                self.v_cmd = [v_ne[0], v_ne[1], rate_z];
                self.reference = [
                    self.reference[0] + self.v_cmd[0] * DT_CONTROL,
                    self.reference[1] + self.v_cmd[1] * DT_CONTROL,
                    self.reference[2] + self.v_cmd[2] * DT_CONTROL,
                ];
            }
            FlightPhase::Emergency => {
                // Descend at 1.0 m/s, zero lateral command; the velocity
                // reference makes the PD produce about 0.7 mg of thrust.
                self.v_cmd = [0.0, 0.0, 1.0];
                self.reference = [
                    self.reference[0],
                    self.reference[1],
                    self.reference[2] + DT_CONTROL,
                ];
            }
            FlightPhase::Idle | FlightPhase::Arm => unreachable!("handled above"),
        }

        let command = self
            .controller
            .step(&estimated, &to_vec3(self.reference), &to_vec3(self.v_cmd), yaw_desired, DT_CONTROL);
        self.command_thrust = f64::max(0.0, command.collective_thrust);
        self.command_moment = to_arr3(&command.moment);
    }

    fn fire_scheduled(&mut self) {
        if self.scheduled.is_empty() {
            return;
        }
        let scheduled = std::mem::take(&mut self.scheduled);
        let mut remaining = Vec::new();
        for (at_time, action) in scheduled {
            if self.time >= at_time {
                action(self);
            } else {
                remaining.push((at_time, action));
            }
        }
        self.scheduled = remaining;
    }

    fn record_history(&mut self) {
        self.history_time.push(self.time);
        self.history_truth_pos.push(to_arr3(&self.truth.position));
        self.history_truth_vel.push(to_arr3(&self.truth.velocity));
        self.history_ekf_pos.push(to_arr3(&self.ekf.nominal().position));
        self.history_ekf_vel.push(to_arr3(&self.ekf.nominal().velocity));
        self.history_thrust.push(self.last_thrust_actual);
        self.history_moment.push(self.last_moment_actual);
        self.history_wind.push(self.last_wind_ned);
        let covariance = self.ekf.covariance();
        self.history_pos_var
            .push([covariance[(0, 0)], covariance[(1, 1)], covariance[(2, 2)]]);
        let nominal = self.ekf.nominal();
        self.history_gyro_bias.push(to_arr3(&nominal.gyro_bias));
        self.history_accel_bias.push(to_arr3(&nominal.accel_bias));
    }

    fn finish(&self, duration_s: f64) -> SimResult {
        SimResult {
            time_s: self.history_time.clone(),
            truth_position: self.history_truth_pos.clone(),
            truth_velocity: self.history_truth_vel.clone(),
            ekf_position: self.history_ekf_pos.clone(),
            ekf_velocity: self.history_ekf_vel.clone(),
            phase_timeline: self.phase_timeline.clone(),
            event_log: self.state_machine.event_log().to_vec(),
            e_ct: self.history_e_ct.clone(),
            gps_update_times: self.history_gps_times.clone(),
            gps_skipped: self.gps_skipped,
            gps_gated: self.gps_gated,
            gps_accepted: self.gps_accepted,
            home_ned: self.home,
            waypoints_ned: self.config.waypoints.iter().map(|w| w.position).collect(),
            command_thrust: self.history_thrust.clone(),
            command_moment: self.history_moment.clone(),
            wind_ned: self.history_wind.clone(),
            position_var: self.history_pos_var.clone(),
            gyro_bias: self.history_gyro_bias.clone(),
            accel_bias: self.history_accel_bias.clone(),
            e_ct_times: self.history_e_ct_times.clone(),
            e_ct_leg: self.history_e_ct_leg.clone(),
            ekf_converged: self.ekf_converged,
            ekf_diverged: self.ekf_diverged,
            duration_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn waypoint(n: f64, e: f64, d: f64, radius: f64) -> Waypoint {
        Waypoint::new([n, e, d], 5.0, 0.0, radius)
    }

    #[test]
    fn an_invalid_takeoff_altitude_is_rejected() {
        let cfg = SimConfig {
            takeoff_altitude: 0.0,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonPositiveTakeoffAltitude(_))
        ));
    }

    #[test]
    fn an_invalid_run_duration_is_rejected() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        assert!(matches!(
            r.run(0.0),
            Err(SimError::NonPositiveDuration(_))
        ));
    }

    #[test]
    fn the_pre_roll_converges_the_estimator() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        assert!(!r.ekf_converged, "not converged before the pre-roll");
        r.preroll();
        assert!(r.ekf_converged, "the pre-roll should converge the estimate");
        assert_eq!(r.time(), 0.0, "the main clock starts at zero");
    }

    #[test]
    fn the_vehicle_sits_on_the_ground_until_takeoff() {
        let cfg = SimConfig {
            start_position_ned: [0.0, 0.0, -0.0],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.preroll();
        for i in 0..200 {
            r.step_once(i);
        }
        // No arm, no takeoff: on the ground, at rest, nothing commanding thrust.
        assert!((r.truth().position[2]).abs() < 1e-9, "should be on the ground");
        assert!((r.truth().velocity[2]).abs() < 1e-6);
        assert_eq!(r.state_machine().phase(), FlightPhase::Idle);
    }

    #[test]
    fn arming_then_taking_off_climbs_to_the_target_altitude() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        // 12 s is plenty to cover 10 m at the 2 m/s climb limit, plus the
        // climb-in from the takeoff controller.
        let result = r.run(12.0).unwrap();
        let phase = result.final_phase();
        assert!(
            matches!(phase, FlightPhase::Cruise | FlightPhase::Takeoff),
            "should have climbed, final phase {phase:?}"
        );
        let altitude = result.final_truth_altitude_m();
        assert!(altitude > 8.0, "climbed only {altitude} m");
    }

    #[test]
    fn the_armed_flag_follows_the_phase() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.preroll();
        assert!(!r.armed);
        r.request_event("arm", None);
        r.step_once(0);
        assert!(r.armed, "arm should set the armed flag");
    }

    #[test]
    fn a_waypoint_mission_is_flown_and_reaches_rl_then_lands() {
        let cfg = SimConfig {
            waypoints: vec![
                waypoint(20.0, 0.0, -10.0, 3.0),
                waypoint(20.0, 20.0, -10.0, 3.0),
            ],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        // 90 s of sim time: climb, cruise hold, two legs, RTL, land.
        let result = r.run(90.0).unwrap();
        let visited = result.phases_visited();
        for expected in [
            FlightPhase::Takeoff,
            FlightPhase::Cruise,
            FlightPhase::Waypoint,
            FlightPhase::Rtl,
        ] {
            assert!(
                visited.contains(&expected),
                "never visited {expected:?}, timeline {:?}",
                result.phase_timeline
            );
        }
        assert!(
            result.e_ct.iter().any(|e| *e > 0.0),
            "a real leg should develop cross-track error"
        );
        assert_eq!(result.gps_skipped, 0, "no outage, so nothing skipped");
    }

    #[test]
    fn the_mission_histories_have_consistent_lengths() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(20.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(40.0).unwrap();
        let n = result.time_s.len();
        assert!(n > 0);
        for (name, len) in [
            ("truth_position", result.truth_position.len()),
            ("truth_velocity", result.truth_velocity.len()),
            ("ekf_position", result.ekf_position.len()),
            ("ekf_velocity", result.ekf_velocity.len()),
            ("command_thrust", result.command_thrust.len()),
            ("command_moment", result.command_moment.len()),
            ("wind_ned", result.wind_ned.len()),
            ("position_var", result.position_var.len()),
            ("gyro_bias", result.gyro_bias.len()),
            ("accel_bias", result.accel_bias.len()),
        ] {
            assert_eq!(len, n, "{name} length {len} vs time {n}");
        }
        assert_eq!(result.e_ct.len(), result.e_ct_times.len());
        assert_eq!(result.e_ct.len(), result.e_ct_leg.len());
    }

    #[test]
    fn a_gps_outage_is_counted_in_the_skipped_fixes() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.preroll();
        r.inject_gps_outage(3.0).unwrap();
        r.request_event("arm", None);
        r.run(5.0).unwrap();
        // 3 s at 10 Hz is about 30 skipped fixes.
        assert!(
            r.gps_skipped >= 25,
            "expected about 30 skipped fixes, got {}",
            r.gps_skipped
        );
    }

    #[test]
    fn a_non_positive_gps_outage_is_rejected() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        assert!(matches!(
            r.inject_gps_outage(0.0),
            Err(SimError::NonPositiveGpsOutage(_))
        ));
    }

    #[test]
    fn low_battery_returns_to_launch() {
        let cfg = SimConfig {
            // A far waypoint, so the mission is still in progress when the
            // battery is injected and RTL is triggered by low battery rather
            // than by mission completion.
            waypoints: vec![waypoint(200.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        // Inject AFTER arming: the arm guard requires soc > 0.30, so draining
        // the pack in advance would block the arm event instead.
        r.schedule(12.0, Box::new(|s| s.inject_battery_soc(0.15)));
        let result = r.run(30.0).unwrap();
        assert!(
            result.phases_visited().contains(&FlightPhase::Waypoint),
            "the mission should have started first, timeline {:?}",
            result.phase_timeline
        );
        assert!(
            result.phases_visited().contains(&FlightPhase::Rtl),
            "low battery should have triggered RTL, timeline {:?}",
            result.phase_timeline
        );
    }

    #[test]
    fn a_corrupted_estimate_triggers_the_emergency_sequence() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(40.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        // Divergence is only reported from 5 s of sim time.
        r.schedule(6.0, Box::new(|s| s.inject_ekf_divergence(DivergenceMode::CorruptBias)));
        let result = r.run(20.0).unwrap();
        let visited = result.phases_visited();
        assert!(
            visited.contains(&FlightPhase::Emergency),
            "a critical fault should reach EMERGENCY, timeline {:?}",
            result.phase_timeline
        );
        assert!(result.ekf_diverged);
    }

    #[test]
    fn a_scheduled_event_fires_at_its_time() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.preroll();
        r.request_event("arm", Some(1.0));
        r.request_event("takeoff", Some(1.5));
        let result = r.run(4.0).unwrap();
        // Arm and takeoff both landed on the timeline at roughly their times.
        let takeoff_time = result
            .phase_timeline
            .iter()
            .find(|(_, p)| *p == FlightPhase::Takeoff)
            .map(|(t, _)| *t)
            .expect("takeoff should have happened");
        assert!(
            (takeoff_time - 1.5).abs() < 0.05,
            "takeoff at {takeoff_time}, expected about 1.5"
        );
    }

    #[test]
    fn the_same_seed_reproduces_the_same_flight() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(25.0, 10.0, -10.0, 3.0)],
            ..Default::default()
        };
        let fly = || {
            let mut r = SimRunner::new(cfg.clone()).unwrap();
            r.request_event("arm", None);
            r.request_event("takeoff", None);
            r.run(25.0).unwrap()
        };
        let a = fly();
        let b = fly();
        assert_eq!(a.time_s.len(), b.time_s.len());
        for (x, y) in a.truth_position.iter().zip(b.truth_position.iter()) {
            assert_eq!(x, y, "same seed must reproduce the flight exactly");
        }
    }

    #[test]
    fn wind_turbulence_is_recorded() {
        let cfg = SimConfig {
            wind_sigma: [1.0, 1.0, 0.5],
            wind_steady_ned: [2.0, 0.0, 0.0],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(15.0).unwrap();
        // Steady wind dominates but turbulence perturbs it, so samples vary.
        assert!(result.wind_ned.iter().any(|w| w[0] > 1.0));
        let first = result.wind_ned.first().copied().unwrap_or([0.0; 3]);
        let last = result.wind_ned.last().copied().unwrap_or([0.0; 3]);
        assert!(first != last, "turbulence should evolve");
    }

    #[test]
    fn the_ground_clamp_never_lets_the_truth_go_below_home() {
        let cfg = SimConfig {
            start_position_ned: [0.0, 0.0, -0.0],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(20.0).unwrap();
        for p in &result.truth_position {
            assert!(p[2] <= 1e-9, "truth z went below the ground plane: {p:?}");
        }
    }
}
