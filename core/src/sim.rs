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

use serde::{Deserialize, Serialize};

use crate::avoidance::{
    min_surface_clearance, AvoidanceConfig, AvoidanceError, MissionLeg, ReactiveAvoidance,
};
use crate::battery::{BatteryConfig, BatteryError, BatteryModel, BatteryState};
use crate::constants::{
    DT, DT_CONTROL, DT_GPS, MAX_COORDINATE_M, MAX_CRUISE_HOLD_S, MAX_DURATION_S,
    MAX_GPS_POSITION_NOISE_M, MAX_GPS_VELOCITY_NOISE_MPS, MAX_OBSTACLES, MAX_RADIUS_M,
    MAX_TAKEOFF_ALTITUDE_M, MAX_WAYPOINTS, MAX_WAYPOINT_SPEED_MPS, MAX_WIND_COMPONENT_MPS,
};
use crate::controller::CascadedController;
use crate::dynamics::propagate;
use crate::ekf::{Ekf, EkfConfig};
use crate::faults::{FaultSpec, MAX_FAULTS};
use crate::guidance::{NlglConfig, NlglGuidance};
use crate::math;
use crate::mission::{MissionError, WaypointMission};
use crate::motors::{forward_mix, MotorError, MotorModel};
use crate::sensors::{
    gps_to_ned, gps_velocity_ned, specific_force_from_velocity, GpsConfig, ImuConfig, SensorSuite,
};
use crate::state_machine::{FlightPhaseMachine, PhaseError, Telemetry};
use crate::types::{
    to_arr3, to_vec3, Arr3, EventLogEntry, FlightPhase, Obstacle, State, VehicleParams, Waypoint,
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
    /// Spherical keep-out zones (schema 1.1). Empty means the law has nothing
    /// to avoid and the run is identical to one flown without zones.
    pub obstacles: Vec<Obstacle>,
    /// Reactive avoidance tuning. Disabled by default, so adding zones to a
    /// scenario is inert until the scenario asks for the law.
    pub avoidance: AvoidanceConfig,
    /// Authored faults, each scheduled at its absolute sim time. Empty by
    /// default, so a scenario without faults runs exactly as before.
    pub faults: Vec<FaultSpec>,
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
            obstacles: Vec::new(),
            avoidance: AvoidanceConfig::default(),
            faults: Vec::new(),
            seed: 20_261_012,
        }
    }
}

impl SimConfig {
    /// Validate the scenario parameters.
    ///
    /// Every scalar is checked for finiteness before it is range-checked:
    /// JSON `1e999` parses to `inf`, which a bare `> 0.0` test waves through
    /// (and would saturate the runner's step count to `usize::MAX`), while
    /// NaN defeats every comparison because all of them are false. Nothing is
    /// clamped; a bad value is rejected with the offending field named.
    pub fn validate(&self) -> Result<(), SimError> {
        // Takeoff altitude: NaN and non-positive keep the original rejection
        // (the phase machine would chase an undefined altitude); the cap then
        // catches the JSON `1e999` -> inf path that positivity alone misses.
        let takeoff = self.takeoff_altitude;
        if takeoff <= 0.0 || takeoff.is_nan() {
            return Err(SimError::NonPositiveTakeoffAltitude(takeoff));
        }
        if !takeoff.is_finite() {
            return Err(SimError::NonFiniteParameter {
                name: "takeoff_altitude",
                value: takeoff,
            });
        }
        if takeoff > MAX_TAKEOFF_ALTITUDE_M {
            return Err(SimError::OutOfRangeParameter {
                name: "takeoff_altitude",
                value: takeoff,
                min: 0.0,
                max: MAX_TAKEOFF_ALTITUDE_M,
            });
        }

        for component in self.start_position_ned {
            check_coordinate("start_position_ned", component)?;
        }
        for sigma in self.wind_sigma {
            check_range("wind_sigma", sigma, 0.0, MAX_WIND_COMPONENT_MPS)?;
        }
        for component in self.wind_steady_ned {
            check_range(
                "wind_steady_ned",
                component,
                -MAX_WIND_COMPONENT_MPS,
                MAX_WIND_COMPONENT_MPS,
            )?;
        }
        check_range("battery_initial_soc", self.battery_initial_soc, 0.0, 1.0)?;
        check_range(
            "gps_position_noise",
            self.gps_position_noise,
            0.0,
            MAX_GPS_POSITION_NOISE_M,
        )?;
        check_range(
            "gps_position_noise_vertical",
            self.gps_position_noise_vertical,
            0.0,
            MAX_GPS_POSITION_NOISE_M,
        )?;
        check_range(
            "gps_velocity_noise",
            self.gps_velocity_noise,
            0.0,
            MAX_GPS_VELOCITY_NOISE_MPS,
        )?;
        check_range("cruise_hold_s", self.cruise_hold_s, 0.0, MAX_CRUISE_HOLD_S)?;

        if self.waypoints.len() > MAX_WAYPOINTS {
            return Err(SimError::TooManyItems {
                name: "waypoints",
                count: self.waypoints.len(),
                max: MAX_WAYPOINTS,
            });
        }
        for waypoint in &self.waypoints {
            for component in waypoint.position {
                check_coordinate("waypoint position", component)?;
            }
            check_range(
                "waypoint speed",
                waypoint.speed,
                0.0,
                MAX_WAYPOINT_SPEED_MPS,
            )?;
            // Heading is finite-only on purpose: guidance wraps angles, so any
            // finite heading names a real direction and needs no ceiling.
            if !waypoint.heading.is_finite() {
                return Err(SimError::NonFiniteParameter {
                    name: "waypoint heading",
                    value: waypoint.heading,
                });
            }
            check_range("waypoint radius", waypoint.radius, 0.0, MAX_RADIUS_M)?;
        }

        if self.obstacles.len() > MAX_OBSTACLES {
            return Err(SimError::TooManyItems {
                name: "obstacles",
                count: self.obstacles.len(),
                max: MAX_OBSTACLES,
            });
        }
        for obstacle in &self.obstacles {
            for component in obstacle.center_ned {
                check_coordinate("obstacle center_ned", component)?;
            }
            check_range("obstacle radius_m", obstacle.radius_m, 0.0, MAX_RADIUS_M)?;
        }

        if self.faults.len() > MAX_FAULTS {
            return Err(SimError::TooManyItems {
                name: "faults",
                count: self.faults.len(),
                max: MAX_FAULTS,
            });
        }
        for fault in &self.faults {
            // No run duration here: SimConfig is duration-agnostic (the
            // scenario carries it), so only finiteness and the parameter
            // ranges apply; the exact last-step window is enforced by
            // ScenarioConfig::validate, which knows the duration.
            fault.validate_fields()?;
        }

        self.avoidance.validate()?;
        Ok(())
    }
}

/// Reject a scalar that must be finite and lie in `[min, max]`.
///
/// Finiteness is checked first so `+inf` is reported as non-finite rather
/// than mislabeled as merely out of range, and so NaN (which fails every
/// comparison, so both range tests would wave it through) cannot slip past.
pub(crate) fn check_range(
    name: &'static str,
    value: f64,
    min: f64,
    max: f64,
) -> Result<(), SimError> {
    if !value.is_finite() {
        return Err(SimError::NonFiniteParameter { name, value });
    }
    if value < min || value > max {
        return Err(SimError::OutOfRangeParameter {
            name,
            value,
            min,
            max,
        });
    }
    Ok(())
}

/// Reject a NED coordinate that must be finite and within `MAX_COORDINATE_M`.
fn check_coordinate(name: &'static str, value: f64) -> Result<(), SimError> {
    check_range(name, value, -MAX_COORDINATE_M, MAX_COORDINATE_M)
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
    /// A parameter arrived as NaN or infinity, which JSON can produce from
    /// `NaN` literals or `1e999` and no physics quantity may hold.
    NonFiniteParameter {
        /// The offending field, e.g. `"duration_s"`.
        name: &'static str,
        /// The rejected value.
        value: f64,
    },
    /// A finite parameter fell outside its accepted range.
    OutOfRangeParameter {
        /// The offending field, e.g. `"takeoff_altitude"`.
        name: &'static str,
        /// The rejected value.
        value: f64,
        /// Smallest accepted value, inclusive.
        min: f64,
        /// Largest accepted value, inclusive.
        max: f64,
    },
    /// A run too short to schedule any fault into: with at most one plant
    /// step, no fault time could fire, so every fault is rejected rather
    /// than silently never applied.
    NoFaultWindow(f64),
    /// A list parameter carried more items than the cap allows.
    TooManyItems {
        /// The offending list, e.g. `"waypoints"`.
        name: &'static str,
        /// How many items arrived.
        count: usize,
        /// The cap that was exceeded.
        max: usize,
    },
    /// The battery configuration was invalid.
    Battery(BatteryError),
    /// The motor configuration was invalid.
    Motor(MotorError),
    /// The mission was invalid.
    Mission(MissionError),
    /// The takeoff altitude was rejected by the phase machine.
    Phase(PhaseError),
    /// The avoidance tuning was invalid.
    Avoidance(AvoidanceError),
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
            Self::NonFiniteParameter { name, value } => {
                write!(f, "{name} must be finite, got {value}")
            }
            Self::OutOfRangeParameter {
                name,
                value,
                min,
                max,
            } => write!(f, "{name} must be in [{min}, {max}], got {value}"),
            Self::TooManyItems { name, count, max } => {
                write!(f, "{name} has {count} items, at most {max} allowed")
            }
            Self::NoFaultWindow(v) => write!(
                f,
                "duration_s {v} leaves no fault window: a run must span at least two plant steps ({DT} s each) for a fault to fire"
            ),
            Self::Battery(e) => write!(f, "battery: {e}"),
            Self::Motor(e) => write!(f, "motor: {e}"),
            Self::Mission(e) => write!(f, "mission: {e}"),
            Self::Phase(e) => write!(f, "phase: {e}"),
            Self::Avoidance(e) => write!(f, "avoidance: {e}"),
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
impl From<AvoidanceError> for SimError {
    fn from(e: AvoidanceError) -> Self {
        Self::Avoidance(e)
    }
}

/// The estimator divergence injections the RTL and FDIR scenarios use.
///
/// Serialized `snake_case`, so the authored JSON spells the modes
/// `corrupt_bias`, `suppress_updates` and `adversarial_gps`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
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
    /// Waypoints in the configured mission (0 when there was no mission).
    pub mission_waypoint_count: usize,
    /// Waypoints actually reached. Counts a waypoint as reached once the
    /// mission has advanced past it, so the active index itself does not count
    /// while it is still being flown toward.
    pub mission_waypoints_reached: usize,
    /// Whether the mission ran to completion.
    pub mission_complete: bool,
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
    /// Closest surface clearance (m) the truth trajectory came to any keep-out
    /// zone, or `f64::INFINITY` when the scenario had none. This is the
    /// validation number: it is measured on the truth state, not the estimate
    /// the law steers with.
    pub min_obstacle_clearance_m: f64,
    /// How many bypasses the reactive law took. Zero means it never left the
    /// planned path.
    pub avoidance_activations: u64,
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
}

/// One-shot fault callback: runs against the runner when its fire time is due.
pub type ScheduledAction = Box<dyn FnOnce(&mut SimRunner)>;

/// Full-stack closed-loop SIL, single-threaded and steppable.
///
/// Build with [`SimRunner::new`], configure faults, then either [`SimRunner::run`]
/// to completion or [`SimRunner::step_once`] repeatedly for an interactive loop.
pub struct SimRunner {
    config: SimConfig,
    params: VehicleParams,
    controller: CascadedController,
    guidance: NlglGuidance,
    avoidance: ReactiveAvoidance,
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
    /// Raised by an authored MotorFault (and by any future actuator model
    /// that reports a failed motor). The rising edge in `handle_events` turns
    /// it into one `critical_fault` request; `build_telemetry` carries it so
    /// the guard can read it.
    motor_fault: bool,
    /// The flag's previous-step value, for that rising edge: one response per
    /// fault, the same edge discipline divergence uses.
    motor_fault_prev: bool,

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
    scheduled: Vec<(f64, ScheduledAction)>,

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
    history_quat: Vec<[f64; 4]>,
    history_omega: Vec<Arr3>,
    phase_timeline: Vec<(f64, FlightPhase)>,
}

/// Doubles per streamed sample in [`SimRunner::samples_from`].
///
/// The layout is also asserted by a test, so a change here has to be a
/// deliberate change on both sides of the wasm boundary.
pub const SAMPLE_STRIDE: usize = 36;

/// Offsets within one [`SAMPLE_STRIDE`] block, in doubles.
pub mod sample_field {
    /// Sim time at the sample (s).
    pub const TIME: usize = 0;
    /// Truth position, NED (m).
    pub const TRUTH_POSITION: usize = 1;
    /// Truth velocity, NED (m/s).
    pub const TRUTH_VELOCITY: usize = 4;
    /// Truth attitude, Hamilton quaternion `[w, x, y, z]`, body to NED.
    pub const TRUTH_QUATERNION: usize = 7;
    /// Truth body angular rate (rad/s).
    pub const TRUTH_ANGULAR_RATE: usize = 11;
    /// Estimated position, NED (m).
    pub const EKF_POSITION: usize = 14;
    /// Estimated velocity, NED (m/s).
    pub const EKF_VELOCITY: usize = 17;
    /// Estimated position variance diagonal (m^2).
    pub const POSITION_VARIANCE: usize = 20;
    /// Estimated gyro bias (rad/s).
    pub const GYRO_BIAS: usize = 23;
    /// Estimated accelerometer bias (m/s^2).
    pub const ACCEL_BIAS: usize = 26;
    /// Wind, NED (m/s).
    pub const WIND: usize = 29;
    /// Delivered thrust (N).
    pub const THRUST: usize = 32;
    /// Delivered body moment (N*m).
    pub const MOMENT: usize = 33;
}

impl SimRunner {
    /// Build a runner from a scenario configuration.
    pub fn new(config: SimConfig) -> Result<Self, SimError> {
        config.validate()?;
        let params = VehicleParams::default();
        let controller = CascadedController::with_defaults();
        let guidance = NlglGuidance::new(NlglConfig::default());
        let avoidance = ReactiveAvoidance::new(config.avoidance);

        let mission = if config.waypoints.is_empty() {
            None
        } else {
            Some(WaypointMission::new(config.waypoints.clone())?)
        };

        let truth = State {
            position: to_vec3(config.start_position_ned),
            ..State::default()
        };
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

        let mut runner = Self {
            reference: config.start_position_ned,
            wind_steady: config.wind_steady_ned,
            config,
            params,
            controller,
            guidance,
            avoidance,
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
            motor_fault: false,
            motor_fault_prev: false,
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
            history_quat: Vec::new(),
            history_omega: Vec::new(),
            phase_timeline: vec![(0.0, FlightPhase::Idle)],
        };

        // Authored faults ride the existing schedule: each spec becomes a
        // one-shot action pushed in list order, so equal fire times apply in
        // scenario order, and `fire_scheduled` applies each exactly once at
        // the first step whose clock reaches `at_s`, before the step's
        // sensors and events. Nothing can fire during the convergence
        // pre-roll: validation keeps `at_s >= 0`, and the pre-roll runs on a
        // negative clock without draining the schedule.
        for spec in runner.config.faults.clone() {
            let at_s = spec.at_s();
            runner.schedule(
                at_s,
                Box::new(move |runner| {
                    let _ = runner.apply_fault(&spec);
                }),
            );
        }
        Ok(runner)
    }

    /// Samples recorded so far, at the control rate (100 Hz).
    ///
    /// A live display polls this to learn how much new material there is, then
    /// pulls it with [`SimRunner::samples_from`].
    pub fn sample_count(&self) -> usize {
        self.history_time.len()
    }

    /// Pack the samples in `[from, sample_count())` into a flat `f64` buffer,
    /// [`SAMPLE_STRIDE`] doubles per sample.
    ///
    /// This is the streaming counterpart of [`SimRunner::snapshot`]: the same
    /// recorded history, handed out incrementally so a display can consume it
    /// without materializing a whole document every frame. It carries attitude
    /// and body rates as well, which the schema document deliberately does not,
    /// so a live GCS can show them.
    ///
    /// `from` is clamped to the recorded length, so a caller that has fallen
    /// behind (or asked for the future) gets what exists and no more.
    pub fn samples_from(&self, from: usize) -> Vec<f64> {
        let start = from.min(self.history_time.len());
        let n = self.history_time.len() - start;
        let mut out = Vec::with_capacity(n * SAMPLE_STRIDE);
        for i in start..self.history_time.len() {
            let q = self.history_quat[i];
            let omega = self.history_omega[i];
            out.push(self.history_time[i]);
            out.extend_from_slice(&self.history_truth_pos[i]);
            out.extend_from_slice(&self.history_truth_vel[i]);
            out.extend_from_slice(&[q[0], q[1], q[2], q[3]]);
            out.extend_from_slice(&omega);
            out.extend_from_slice(&self.history_ekf_pos[i]);
            out.extend_from_slice(&self.history_ekf_vel[i]);
            out.extend_from_slice(&self.history_pos_var[i]);
            out.extend_from_slice(&self.history_gyro_bias[i]);
            out.extend_from_slice(&self.history_accel_bias[i]);
            out.extend_from_slice(&self.history_wind[i]);
            out.push(self.history_thrust[i]);
            out.extend_from_slice(&self.history_moment[i]);
        }
        out
    }

    /// Phase transitions, oldest first.
    pub fn phase_timeline(&self) -> &[(f64, FlightPhase)] {
        &self.phase_timeline
    }

    /// GPS fix accounting: `(accepted, gated, skipped)`.
    pub fn gps_counts(&self) -> (u64, u64, u64) {
        (self.gps_accepted, self.gps_gated, self.gps_skipped)
    }

    /// Estimator flags: `(converged, diverged)`.
    pub fn ekf_flags(&self) -> (bool, bool) {
        (self.ekf_converged, self.ekf_diverged)
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
                self.schedule(
                    t,
                    Box::new(move |s: &mut SimRunner| s.event_queue.push_back(owned)),
                );
            }
        }
    }

    /// Run `action` once sim time reaches `at_time`.
    pub fn schedule(&mut self, at_time: f64, action: ScheduledAction) {
        self.scheduled.push((at_time, action));
    }

    /// Black out GPS for `duration_s` sim seconds from now.
    ///
    /// During the outage no fix is generated or fed to the filter, so the filter
    /// dead-reckons on IMU only. Repeated calls extend the outage. A
    /// non-finite duration is rejected: `inf` would black out GPS forever and
    /// NaN would corrupt the outage window arithmetic.
    pub fn inject_gps_outage(&mut self, duration_s: f64) -> Result<(), SimError> {
        if !duration_s.is_finite() {
            return Err(SimError::NonFiniteParameter {
                name: "gps_outage_duration",
                value: duration_s,
            });
        }
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

    /// Apply one authored fault now, mapping it onto the injection machinery.
    ///
    /// [`SimRunner::new`] schedules every scenario fault against this, so the
    /// per-step `fire_scheduled` runs it exactly once when the clock reaches
    /// its `at_s`, before the step's sensors and events. Validation at config
    /// time makes the error branch unreachable for authored faults, but a
    /// direct call (say, an operator command over the same API) still gets
    /// the injection's own rejection, such as a non-positive outage length.
    pub fn apply_fault(&mut self, spec: &FaultSpec) -> Result<(), SimError> {
        match spec {
            FaultSpec::GpsOutage { duration_s, .. } => self.inject_gps_outage(*duration_s),
            FaultSpec::EkfDivergence { mode, .. } => {
                self.inject_ekf_divergence(*mode);
                Ok(())
            }
            FaultSpec::BatterySoc { soc, .. } => {
                self.inject_battery_soc(*soc);
                Ok(())
            }
            FaultSpec::MotorFault { .. } => {
                // The flag rides telemetry; the rising edge in handle_events
                // turns it into one `critical_fault` on that same step's
                // event pass, so EMERGENCY is reached through the same path
                // a diverged estimate takes.
                self.motor_fault = true;
                Ok(())
            }
            FaultSpec::Emergency { .. } => {
                self.state_machine
                    .force_phase(FlightPhase::Emergency, self.time, "authored fault");
                self.note_phase_change();
                Ok(())
            }
            FaultSpec::Rtl { .. } => {
                let telemetry = self.build_telemetry();
                self.state_machine.request("rtl", &telemetry, self.time);
                self.note_phase_change();
                Ok(())
            }
        }
    }

    /// Run the convergence pre-roll, then `duration_s` sim seconds.
    ///
    /// A runner is intended for one run: histories accumulate. The duration
    /// is validated before any step is computed: `inf` would otherwise
    /// saturate the step count to `usize::MAX`, and NaN would silently run
    /// zero steps.
    pub fn run(&mut self, duration_s: f64) -> Result<SimResult, SimError> {
        if !duration_s.is_finite() {
            return Err(SimError::NonFiniteParameter {
                name: "duration_s",
                value: duration_s,
            });
        }
        if duration_s <= 0.0 {
            return Err(SimError::NonPositiveDuration(duration_s));
        }
        if duration_s > MAX_DURATION_S {
            return Err(SimError::OutOfRangeParameter {
                name: "duration_s",
                value: duration_s,
                min: 0.0,
                max: MAX_DURATION_S,
            });
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

    /// Assemble a [`SimResult`] from the histories accumulated so far.
    ///
    /// [`SimRunner::run`] calls this once at the end. An interactive or
    /// streaming loop can call it between steps to read out a partial flight
    /// without stopping the run, which is what the browser does while the
    /// vehicle is still flying.
    pub fn snapshot(&self, duration_s: f64) -> SimResult {
        self.finish(duration_s)
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
        let forces = self
            .motors
            .step(self.command_thrust, self.command_moment, DT);
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
        let f_body =
            specific_force_from_velocity(&to_vec3(self.v_prev), &truth_velocity, &rotation, DT);
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
    fn plant_step(
        &mut self,
        thrust_actual: Option<f64>,
        moment_actual: Option<nalgebra::Vector3<f64>>,
    ) {
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
            self.phase_timeline
                .push((self.time, self.state_machine.phase()));
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
            motor_fault: self.motor_fault,
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

        // Automatic critical_fault fires on the RISING EDGE of divergence or
        // of an authored motor fault: one response per fault episode. A
        // diverged estimate that never recovers never re-fires, so the
        // emergency sequence owns the vehicle until landed; when the estimate
        // recovers the trigger re-arms. The motor-fault flag is sticky, so its
        // own edge keeps the log from repeating `critical_fault` every step
        // and the idempotent emergency latch from looping after landing.
        let critical_fault_edge = (self.ekf_diverged && !self.ekf_diverged_prev)
            || (self.motor_fault && !self.motor_fault_prev);
        if critical_fault_edge {
            let telemetry = self.build_telemetry();
            let t = self.time;
            self.state_machine.request("critical_fault", &telemetry, t);
            self.note_phase_change();
        }
        self.ekf_diverged_prev = self.ekf_diverged;
        self.motor_fault_prev = self.motor_fault;

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
            && self
                .state_machine
                .request("altitude_reached", &telemetry, t)
        {
            self.note_phase_change();
        }
        if phase == FlightPhase::Cruise
            && self.mission.is_some()
            && (self.time - self.phase_enter_time) >= self.config.cruise_hold_s
            && self.state_machine.request("mission_start", &telemetry, t)
        {
            self.note_phase_change();
        }
        if phase == FlightPhase::Waypoint {
            if telemetry.mission_complete
                && self
                    .state_machine
                    .request("mission_complete", &telemetry, t)
            {
                self.note_phase_change();
            }
            if telemetry.battery_soc < 0.20
                && self.state_machine.request("low_battery", &telemetry, t)
            {
                self.note_phase_change();
            }
            if telemetry.gps_outage_duration > 5.0
                && self.state_machine.request("gps_lost", &telemetry, t)
            {
                self.note_phase_change();
            }
        }
        if phase == FlightPhase::Rtl
            && telemetry.horizontal_dist_to_home() < 1.0
            && self.state_machine.request("home_reached", &telemetry, t)
        {
            self.note_phase_change();
        }
        if phase == FlightPhase::Land
            && telemetry.altitude_above_home() < 0.1
            && telemetry.vel_z().abs() < 0.1
            && self.state_machine.request("landed", &telemetry, t)
        {
            self.note_phase_change();
        }
        if phase == FlightPhase::Emergency {
            // Latch the descent flag on the first step in EMERGENCY so the
            // emergency_landing guard and LAND entry can see it.
            if !self.emergency_descent_active {
                self.emergency_descent_active = true;
                telemetry = self.build_telemetry();
            }
            if telemetry.altitude_above_home() < 0.5
                && self
                    .state_machine
                    .request("emergency_landing", &telemetry, t)
            {
                self.note_phase_change();
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
                let mut rtl = WaypointMission::new(vec![Waypoint::new(home_hover, 5.0, 0.0, 1.0)])
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
                let mission = self.mission.as_ref().expect("WAYPOINT implies a mission");
                let mission_leg = MissionLeg {
                    origin: mission.previous_point(ekf_position),
                    waypoint: mission.active_waypoint().position,
                    index: mission.active_index(),
                };
                // The law may substitute a bypass for the waypoint. With no
                // zones, or with the law disabled, this returns the waypoint
                // itself and the leg below is the mission's own leg, so the
                // path is exactly what it was before avoidance existed.
                let directive = self.avoidance.update(
                    ekf_position,
                    ekf_velocity,
                    Some(mission_leg),
                    &self.config.obstacles,
                );
                let target = directive.target.unwrap_or(mission_leg.waypoint);
                let (v_des, diagnostics) = self.guidance.compute_leg(
                    ekf_position,
                    ekf_velocity,
                    mission_leg.origin,
                    target,
                );
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
                    self.guidance
                        .compute_mission(ekf_position, ekf_velocity, rtl);
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
                let v_ne =
                    if math::sqrt(ne_error[0] * ne_error[0] + ne_error[1] * ne_error[1]) < 5.0 {
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

        let command = self.controller.step(
            &estimated,
            &to_vec3(self.reference),
            &to_vec3(self.v_cmd),
            yaw_desired,
            DT_CONTROL,
        );
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
        self.history_ekf_pos
            .push(to_arr3(&self.ekf.nominal().position));
        self.history_ekf_vel
            .push(to_arr3(&self.ekf.nominal().velocity));
        self.history_thrust.push(self.last_thrust_actual);
        self.history_moment.push(self.last_moment_actual);
        self.history_wind.push(self.last_wind_ned);
        let covariance = self.ekf.covariance();
        self.history_pos_var
            .push([covariance[(0, 0)], covariance[(1, 1)], covariance[(2, 2)]]);
        let nominal = self.ekf.nominal();
        self.history_gyro_bias.push(to_arr3(&nominal.gyro_bias));
        self.history_accel_bias.push(to_arr3(&nominal.accel_bias));
        let q = self.truth.orientation;
        self.history_quat.push([q.w, q.x, q.y, q.z]);
        self.history_omega.push(to_arr3(&self.truth.angular_rate));
    }

    fn finish(&self, duration_s: f64) -> SimResult {
        let (mission_waypoint_count, mission_waypoints_reached, mission_complete) =
            match self.mission.as_ref() {
                Some(mission) => (
                    mission.waypoints().len(),
                    mission.active_index() + if mission.complete() { 1 } else { 0 },
                    mission.complete(),
                ),
                None => (0, 0, false),
            };
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
            mission_waypoint_count,
            mission_waypoints_reached,
            mission_complete,
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
            min_obstacle_clearance_m: min_surface_clearance(
                &self.history_truth_pos,
                &self.config.obstacles,
            ),
            avoidance_activations: self.avoidance.activations(),
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
    fn a_nan_takeoff_altitude_is_rejected_but_a_normal_one_is_accepted() {
        // Regression: `takeoff_altitude <= 0.0` alone would accept NaN,
        // pinning the phase machine's climb target to an undefined value.
        // These cases stay valid when the guard later tightens to
        // `!takeoff_altitude.is_finite()`.
        for bad in [f64::NAN, 0.0, -1.0] {
            let cfg = SimConfig {
                takeoff_altitude: bad,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::NonPositiveTakeoffAltitude(_))
                ),
                "takeoff_altitude {bad} should be rejected"
            );
        }
        SimRunner::new(SimConfig {
            takeoff_altitude: 10.0,
            ..Default::default()
        })
        .expect("a normal positive takeoff altitude is accepted");
    }

    #[test]
    fn an_invalid_run_duration_is_rejected() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        assert!(matches!(r.run(0.0), Err(SimError::NonPositiveDuration(_))));
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
        assert!(
            (r.truth().position[2]).abs() < 1e-9,
            "should be on the ground"
        );
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
        r.schedule(
            6.0,
            Box::new(|s| s.inject_ekf_divergence(DivergenceMode::CorruptBias)),
        );
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
    fn the_streamed_sample_layout_is_consistent() {
        use crate::sim::sample_field as f;
        assert_eq!(f::TIME, 0);
        assert_eq!(f::TRUTH_POSITION, 1);
        assert_eq!(f::TRUTH_VELOCITY, 4);
        assert_eq!(f::TRUTH_QUATERNION, 7);
        assert_eq!(f::TRUTH_ANGULAR_RATE, 11);
        assert_eq!(f::EKF_POSITION, 14);
        assert_eq!(f::EKF_VELOCITY, 17);
        assert_eq!(f::POSITION_VARIANCE, 20);
        assert_eq!(f::GYRO_BIAS, 23);
        assert_eq!(f::ACCEL_BIAS, 26);
        assert_eq!(f::WIND, 29);
        assert_eq!(f::THRUST, 32);
        assert_eq!(f::MOMENT, 33);
        // Every field must end inside the stride, and the last one must close it.
        assert_eq!(f::MOMENT + 3, SAMPLE_STRIDE);
    }

    #[test]
    fn a_streamed_sample_carries_a_unit_quaternion_with_thrust_below_weight() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(50.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        r.preroll();
        for i in 0..3000 {
            r.step_once(i);
        }
        let flat = r.samples_from(0);
        assert_eq!(flat.len(), r.sample_count() * SAMPLE_STRIDE);
        let last = flat.len() - SAMPLE_STRIDE;
        let w = flat[last + sample_field::TRUTH_QUATERNION];
        let x = flat[last + sample_field::TRUTH_QUATERNION + 1];
        let y = flat[last + sample_field::TRUTH_QUATERNION + 2];
        let z = flat[last + sample_field::TRUTH_QUATERNION + 3];
        assert!((w * w + x * x + y * y + z * z - 1.0).abs() < 1e-9);
        // A 1.5 kg quad needs about 14.7 N to hover; in flight it is in that
        // range, not zero and not at the motor limit.
        let thrust = flat[last + sample_field::THRUST];
        assert!(thrust > 10.0 && thrust < 20.0, "thrust {thrust}");
    }

    #[test]
    fn the_streamed_samples_agree_with_the_finished_document() {
        // The streaming path and the document path are the same history, so a
        // live display and a replay cannot disagree.
        let cfg = SimConfig {
            waypoints: vec![waypoint(30.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(20.0).unwrap();

        let flat = r.samples_from(0);
        assert_eq!(flat.len(), result.time_s.len() * SAMPLE_STRIDE);
        for i in 0..result.time_s.len() {
            let base = i * SAMPLE_STRIDE;
            assert_eq!(flat[base + sample_field::TIME], result.time_s[i]);
            let p = result.truth_position[i];
            assert_eq!(
                [
                    flat[base + sample_field::TRUTH_POSITION],
                    flat[base + sample_field::TRUTH_POSITION + 1],
                    flat[base + sample_field::TRUTH_POSITION + 2],
                ],
                p
            );
            let e = result.ekf_position[i];
            assert_eq!(
                [
                    flat[base + sample_field::EKF_POSITION],
                    flat[base + sample_field::EKF_POSITION + 1],
                    flat[base + sample_field::EKF_POSITION + 2],
                ],
                e
            );
        }
    }

    #[test]
    fn only_the_new_samples_come_out_of_a_delta_read() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(40.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        r.preroll();

        assert_eq!(r.sample_count(), 0);
        for i in 0..200 {
            r.step_once(i);
        }
        let first = r.sample_count();
        assert!(first > 0);
        let a = r.samples_from(0);
        assert_eq!(a.len(), first * SAMPLE_STRIDE);
        // Nothing new yet.
        assert!(r.samples_from(first).is_empty());

        for i in 200..400 {
            r.step_once(i);
        }
        let second = r.sample_count();
        assert!(second > first);
        let b = r.samples_from(first);
        assert_eq!(b.len(), (second - first) * SAMPLE_STRIDE);
        // The delta picks up exactly where the first read stopped: the last
        // sample of `a` and the first of `b` are one control period apart.
        let gap = b[sample_field::TIME] - a[a.len() - SAMPLE_STRIDE + sample_field::TIME];
        assert!((gap - DT_CONTROL).abs() < 1e-9, "gap {gap}");
    }

    #[test]
    fn a_read_past_the_end_returns_nothing_rather_than_panicking() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.request_event("arm", None);
        for i in 0..100 {
            r.step_once(i);
        }
        assert!(r.samples_from(usize::MAX).is_empty());
        assert!(r.samples_from(r.sample_count()).is_empty());
    }

    #[test]
    fn the_mission_outcome_is_reported() {
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
        let result = r.run(90.0).unwrap();
        assert_eq!(result.mission_waypoint_count, 2);
        assert!(
            result.mission_complete,
            "should have finished the mission, timeline {:?}",
            result.phase_timeline
        );
        assert_eq!(result.mission_waypoints_reached, 2);
    }

    #[test]
    fn a_run_with_no_mission_reports_no_waypoints() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(10.0).unwrap();
        assert_eq!(result.mission_waypoint_count, 0);
        assert_eq!(result.mission_waypoints_reached, 0);
        assert!(!result.mission_complete);
    }

    #[test]
    fn a_partly_flown_mission_counts_only_the_waypoints_passed() {
        // One waypoint far away, stopped early: it is still being flown toward,
        // so nothing counts as reached yet.
        let cfg = SimConfig {
            waypoints: vec![waypoint(500.0, 0.0, -10.0, 3.0)],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(30.0).unwrap();
        assert_eq!(result.mission_waypoint_count, 1);
        assert!(!result.mission_complete);
        assert_eq!(result.mission_waypoints_reached, 0);
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

    // ------------------------------------------------------------------
    // Reactive avoidance, closed loop
    // ------------------------------------------------------------------

    fn north_zone() -> Obstacle {
        Obstacle {
            id: "obs_north".to_string(),
            center_ned: [20.0, 3.0, -10.0],
            radius_m: 4.0,
        }
    }

    /// Two legs that cut straight across the north zone, which the offline
    /// planner would have detoured before the run.
    fn straight_through_config() -> SimConfig {
        SimConfig {
            waypoints: vec![
                waypoint(40.0, 0.0, -10.0, 3.0),
                waypoint(40.0, 30.0, -10.0, 3.0),
            ],
            obstacles: vec![north_zone()],
            ..Default::default()
        }
    }

    fn fly(cfg: SimConfig, duration_s: f64) -> SimResult {
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        r.run(duration_s).unwrap()
    }

    #[test]
    fn zones_alone_do_not_change_the_flown_path() {
        // The idle guarantee: with the law off, adding zones to a scenario
        // leaves the flight bit-identical, so the frozen runs and the four
        // shipped live configs cannot move because avoidance now exists.
        let with_zones = straight_through_config();
        let without_zones = SimConfig {
            obstacles: Vec::new(),
            ..straight_through_config()
        };
        let a = fly(with_zones, 45.0);
        let b = fly(without_zones, 45.0);
        assert_eq!(a.truth_position, b.truth_position);
        assert_eq!(a.avoidance_activations, 0);
        // The breach is still measured and reported: zones are observed even
        // when the law is not flying the detour.
        assert!(a.min_obstacle_clearance_m.is_finite());
        assert_eq!(b.min_obstacle_clearance_m, f64::INFINITY);
    }

    #[test]
    fn the_same_mission_without_the_law_flies_through_the_zone() {
        // The control for the test below: the planned path really does breach
        // the zone, so the clearance the law achieves is the law's doing.
        let result = fly(straight_through_config(), 45.0);
        assert!(
            result.min_obstacle_clearance_m < 0.0,
            "the planned path should breach the zone, got {}",
            result.min_obstacle_clearance_m
        );
    }

    #[test]
    fn the_law_flies_around_the_zone_and_completes_the_mission() {
        let cfg = SimConfig {
            avoidance: AvoidanceConfig {
                enabled: true,
                ..Default::default()
            },
            ..straight_through_config()
        };
        let result = fly(cfg, 45.0);
        assert!(
            result.avoidance_activations >= 1,
            "the law should have taken at least one bypass"
        );
        assert!(result.mission_complete, "the mission still finishes");
        assert_eq!(result.final_phase(), FlightPhase::Land);
        assert!(
            result.min_obstacle_clearance_m >= 2.0,
            "achieved clearance {} should hold the 2 m margin",
            result.min_obstacle_clearance_m
        );
    }

    #[test]
    fn the_law_clears_a_zone_on_a_later_leg() {
        // The second leg runs east along the 40 m line, so the zone sits on
        // the leg that starts after the first one is reached.
        let zone = Obstacle {
            id: "obs_east".to_string(),
            center_ned: [37.0, 15.0, -10.0],
            radius_m: 3.5,
        };
        let cfg = SimConfig {
            obstacles: vec![zone],
            avoidance: AvoidanceConfig {
                enabled: true,
                ..Default::default()
            },
            ..straight_through_config()
        };
        let result = fly(cfg, 45.0);
        assert!(result.avoidance_activations >= 1);
        assert!(result.mission_complete);
        assert!(
            result.min_obstacle_clearance_m >= 2.0,
            "achieved clearance {} should hold the 2 m margin",
            result.min_obstacle_clearance_m
        );
    }

    #[test]
    fn negative_avoidance_tuning_is_rejected_at_construction() {
        let cfg = SimConfig {
            avoidance: AvoidanceConfig {
                enabled: true,
                clearance_margin_m: -1.0,
                ..Default::default()
            },
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::Avoidance(
                AvoidanceError::NegativeClearanceMargin(_)
            ))
        ));
    }

    // ------------------------------------------------------------------
    // Input hardening: finiteness and magnitude at every seam
    // ------------------------------------------------------------------

    #[test]
    fn an_infinite_takeoff_altitude_is_rejected_as_non_finite() {
        // Regression: JSON `1e999` parses to `inf`, which the positivity test
        // waves through; the phase machine must never chase an undefined
        // climb target. NaN keeps its original rejection (pinned above), and
        // negative infinity is caught by the positivity guard.
        let cfg = SimConfig {
            takeoff_altitude: f64::INFINITY,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "takeoff_altitude",
                ..
            })
        ));
        let cfg = SimConfig {
            takeoff_altitude: f64::NEG_INFINITY,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonPositiveTakeoffAltitude(_))
        ));
    }

    #[test]
    fn a_takeoff_altitude_above_the_cap_is_rejected_as_out_of_range() {
        let cfg = SimConfig {
            takeoff_altitude: MAX_TAKEOFF_ALTITUDE_M + 1.0,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "takeoff_altitude",
                value,
                max,
                ..
            }) if value == MAX_TAKEOFF_ALTITUDE_M + 1.0 && max == MAX_TAKEOFF_ALTITUDE_M
        ));
        // The cap itself is accepted.
        SimRunner::new(SimConfig {
            takeoff_altitude: MAX_TAKEOFF_ALTITUDE_M,
            ..Default::default()
        })
        .expect("the cap itself is in range");
    }

    #[test]
    fn a_non_finite_start_position_is_rejected() {
        for component in 0..3 {
            let mut start = [0.0; 3];
            start[component] = f64::NAN;
            assert!(
                matches!(
                    SimRunner::new(SimConfig {
                        start_position_ned: start,
                        ..Default::default()
                    }),
                    Err(SimError::NonFiniteParameter {
                        name: "start_position_ned",
                        ..
                    })
                ),
                "NaN component {component} should be rejected"
            );
        }
        let cfg = SimConfig {
            start_position_ned: [0.0, f64::INFINITY, 0.0],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "start_position_ned",
                ..
            })
        ));
        // Finite but absurd: out of range, not non-finite.
        let cfg = SimConfig {
            start_position_ned: [MAX_COORDINATE_M + 1.0, 0.0, 0.0],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "start_position_ned",
                ..
            })
        ));
    }

    #[test]
    fn a_non_finite_or_over_cap_wind_component_is_rejected() {
        for component in 0..3 {
            let mut sigma = [0.0; 3];
            sigma[component] = f64::INFINITY;
            assert!(
                matches!(
                    SimRunner::new(SimConfig {
                        wind_sigma: sigma,
                        ..Default::default()
                    }),
                    Err(SimError::NonFiniteParameter {
                        name: "wind_sigma",
                        ..
                    })
                ),
                "infinite sigma {component} should be rejected"
            );
        }
        let cfg = SimConfig {
            wind_steady_ned: [f64::NAN, 0.0, 0.0],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "wind_steady_ned",
                ..
            })
        ));
        let cfg = SimConfig {
            wind_sigma: [MAX_WIND_COMPONENT_MPS + 1.0, 0.0, 0.0],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "wind_sigma",
                ..
            })
        ));
    }

    #[test]
    fn a_non_finite_or_over_cap_gps_noise_is_rejected() {
        for bad in [f64::NAN, f64::INFINITY] {
            let cfg = SimConfig {
                gps_position_noise: bad,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::NonFiniteParameter {
                        name: "gps_position_noise",
                        ..
                    })
                ),
                "gps_position_noise {bad} should be rejected"
            );
            let cfg = SimConfig {
                gps_position_noise_vertical: bad,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::NonFiniteParameter {
                        name: "gps_position_noise_vertical",
                        ..
                    })
                ),
                "gps_position_noise_vertical {bad} should be rejected"
            );
            let cfg = SimConfig {
                gps_velocity_noise: bad,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::NonFiniteParameter {
                        name: "gps_velocity_noise",
                        ..
                    })
                ),
                "gps_velocity_noise {bad} should be rejected"
            );
        }
        let cfg = SimConfig {
            gps_velocity_noise: MAX_GPS_VELOCITY_NOISE_MPS + 1.0,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "gps_velocity_noise",
                ..
            })
        ));
    }

    #[test]
    fn battery_initial_soc_must_be_finite_and_in_range() {
        // NaN would otherwise reach the battery model, which clamps but never
        // validates, and 1.5 would be silently clamped to 1.0.
        for bad in [f64::NAN, f64::INFINITY] {
            let cfg = SimConfig {
                battery_initial_soc: bad,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::NonFiniteParameter {
                        name: "battery_initial_soc",
                        ..
                    })
                ),
                "soc {bad} should be rejected"
            );
        }
        let cfg = SimConfig {
            battery_initial_soc: 1.5,
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "battery_initial_soc",
                max,
                ..
            }) if max == 1.0
        ));
        // The closed interval's endpoints are accepted.
        SimRunner::new(SimConfig {
            battery_initial_soc: 0.0,
            ..Default::default()
        })
        .expect("soc 0 is in range");
    }

    #[test]
    fn a_mission_over_the_waypoint_cap_is_rejected() {
        let cfg = SimConfig {
            waypoints: vec![Waypoint::default(); MAX_WAYPOINTS + 1],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::TooManyItems {
                name: "waypoints",
                count,
                max,
            }) if count == MAX_WAYPOINTS + 1 && max == MAX_WAYPOINTS
        ));
    }

    #[test]
    fn a_non_finite_or_negative_waypoint_field_is_rejected() {
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([f64::NAN, 0.0, -10.0], 5.0, 0.0, 2.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "waypoint position",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, f64::INFINITY, -10.0], 5.0, 0.0, 2.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "waypoint position",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], f64::NAN, 0.0, 2.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "waypoint speed",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], -1.0, 0.0, 2.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "waypoint speed",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], 5.0, f64::INFINITY, 2.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "waypoint heading",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], 5.0, 0.0, f64::NAN)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "waypoint radius",
                ..
            })
        ));
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], 5.0, 0.0, -1.0)],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "waypoint radius",
                ..
            })
        ));
    }

    #[test]
    fn a_non_finite_or_negative_obstacle_is_rejected() {
        let cfg = SimConfig {
            obstacles: vec![Obstacle {
                id: "zone".to_string(),
                center_ned: [0.0, 0.0, -10.0],
                radius_m: f64::NAN,
            }],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "obstacle radius_m",
                ..
            })
        ));
        let cfg = SimConfig {
            obstacles: vec![Obstacle {
                id: "zone".to_string(),
                center_ned: [f64::INFINITY, 0.0, -10.0],
                radius_m: 2.0,
            }],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::NonFiniteParameter {
                name: "obstacle center_ned",
                ..
            })
        ));
        let cfg = SimConfig {
            obstacles: vec![Obstacle {
                id: "zone".to_string(),
                center_ned: [0.0, 0.0, -10.0],
                radius_m: -1.0,
            }],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "obstacle radius_m",
                ..
            })
        ));
        let cfg = SimConfig {
            obstacles: vec![
                Obstacle {
                    id: "zone".to_string(),
                    center_ned: [0.0, 0.0, -10.0],
                    radius_m: 2.0,
                };
                MAX_OBSTACLES + 1
            ],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::TooManyItems {
                name: "obstacles",
                count,
                max,
            }) if count == MAX_OBSTACLES + 1 && max == MAX_OBSTACLES
        ));
    }

    #[test]
    fn waypoint_speed_and_radii_are_capped_but_heading_is_finite_only() {
        // Speed past any plausible flight: 201 m/s is a typo, not a mission.
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new(
                [0.0, 0.0, -10.0],
                MAX_WAYPOINT_SPEED_MPS + 1.0,
                0.0,
                2.0,
            )],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "waypoint speed",
                max,
                ..
            }) if max == MAX_WAYPOINT_SPEED_MPS
        ));

        // Radii are bounded at the coordinate scale, still non-negative.
        let cfg = SimConfig {
            waypoints: vec![Waypoint::new(
                [0.0, 0.0, -10.0],
                5.0,
                0.0,
                MAX_RADIUS_M + 1.0,
            )],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "waypoint radius",
                max,
                ..
            }) if max == MAX_RADIUS_M
        ));
        let cfg = SimConfig {
            obstacles: vec![Obstacle {
                id: "zone".to_string(),
                center_ned: [0.0, 0.0, -10.0],
                radius_m: MAX_RADIUS_M + 1.0,
            }],
            ..Default::default()
        };
        assert!(matches!(
            SimRunner::new(cfg),
            Err(SimError::OutOfRangeParameter {
                name: "obstacle radius_m",
                max,
                ..
            }) if max == MAX_RADIUS_M
        ));

        // Heading stays finite-only: guidance wraps angles, so a very large
        // (still finite) heading names a real direction and is accepted.
        SimRunner::new(SimConfig {
            waypoints: vec![Waypoint::new([0.0, 0.0, -10.0], 5.0, 1.0e6, 2.0)],
            ..Default::default()
        })
        .expect("a large finite heading is legal, angles wrap");
    }

    #[test]
    fn non_finite_avoidance_tuning_is_rejected_at_construction() {
        // The NaN bypass the caps close: NaN failed every `< 0.0` comparison
        // in the old guard, so a NaN margin reached the law unchallenged.
        for (avoidance, name) in [
            (
                AvoidanceConfig {
                    enabled: true,
                    clearance_margin_m: f64::NAN,
                    ..Default::default()
                },
                "clearance_margin_m",
            ),
            (
                AvoidanceConfig {
                    enabled: true,
                    detour_buffer_m: f64::INFINITY,
                    ..Default::default()
                },
                "detour_buffer_m",
            ),
            (
                AvoidanceConfig {
                    enabled: true,
                    lookahead_s: f64::INFINITY,
                    ..Default::default()
                },
                "lookahead_s",
            ),
        ] {
            let cfg = SimConfig {
                avoidance,
                ..Default::default()
            };
            assert!(
                matches!(
                    SimRunner::new(cfg),
                    Err(SimError::Avoidance(
                        AvoidanceError::NonFiniteParameter { name: matched, .. }
                    )) if matched == name
                ),
                "{name} should be rejected as non-finite"
            );
        }
    }

    #[test]
    fn an_unbounded_run_duration_cannot_saturate_the_step_count() {
        // Regression for the verified failure: `run(inf)` computed
        // `(inf / DT).round() as usize`, which saturates to `usize::MAX`.
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        for bad in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(
                matches!(
                    r.run(bad),
                    Err(SimError::NonFiniteParameter {
                        name: "duration_s",
                        ..
                    })
                ),
                "run({bad}) should be rejected as non-finite"
            );
        }
        assert!(matches!(
            r.run(MAX_DURATION_S + 0.1),
            Err(SimError::OutOfRangeParameter {
                name: "duration_s",
                ..
            })
        ));
        assert!(matches!(r.run(-1.0), Err(SimError::NonPositiveDuration(_))));
        assert_eq!(
            r.run(f64::INFINITY).unwrap_err().to_string(),
            "duration_s must be finite, got inf"
        );
    }

    #[test]
    fn a_non_finite_gps_outage_is_rejected() {
        let mut r = SimRunner::new(SimConfig::default()).unwrap();
        for bad in [f64::INFINITY, f64::NAN] {
            assert!(
                matches!(
                    r.inject_gps_outage(bad),
                    Err(SimError::NonFiniteParameter {
                        name: "gps_outage_duration",
                        ..
                    })
                ),
                "outage {bad} should be rejected as non-finite"
            );
        }
        assert!(matches!(
            r.inject_gps_outage(0.0),
            Err(SimError::NonPositiveGpsOutage(_))
        ));
    }

    // ------------------------------------------------------------------
    // Authored faults: one end-to-end run per kind, plus the schedule
    // ------------------------------------------------------------------

    #[test]
    fn an_authored_gps_outage_skips_fixes() {
        let cfg = SimConfig {
            faults: vec![FaultSpec::GpsOutage {
                at_s: 1.0,
                duration_s: 2.0,
            }],
            ..Default::default()
        };
        let result = fly(cfg, 6.0);
        // 2 s of outage at 10 Hz is 20 fixes; the band catches a single
        // dropped fix as surely as none at all.
        assert!(
            result.gps_skipped >= 15 && result.gps_skipped <= 25,
            "a 2 s outage at 10 Hz should skip about 20 fixes, got {}",
            result.gps_skipped
        );
    }

    #[test]
    fn an_authored_corrupt_bias_divergence_reaches_emergency() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(40.0, 0.0, -10.0, 3.0)],
            faults: vec![FaultSpec::EkfDivergence {
                at_s: 6.0,
                mode: DivergenceMode::CorruptBias,
            }],
            ..Default::default()
        };
        let result = fly(cfg, 20.0);
        let visited = result.phases_visited();
        assert!(
            visited.contains(&FlightPhase::Emergency),
            "an authored corruption should reach EMERGENCY, timeline {:?}",
            result.phase_timeline
        );
        assert!(result.ekf_diverged);
    }

    #[test]
    fn an_authored_suppress_updates_divergence_dead_reckons_without_emergency() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(20.0, 0.0, -10.0, 3.0)],
            faults: vec![FaultSpec::EkfDivergence {
                at_s: 8.0,
                mode: DivergenceMode::SuppressUpdates,
            }],
            ..Default::default()
        };
        // The run is deliberately modest: the mission completes into RTL
        // well before organic IMU drift could reach the 20 m divergence
        // clause, so any flag here would be the mode misbehaving, not the
        // test's patience running out.
        let result = fly(cfg, 25.0);
        // GPS updates stop being fed from the fault on: every due fix after
        // it is skipped, and the filter dead-reckons on IMU only.
        assert!(
            result.gps_skipped >= 100,
            "the suppressed filter should skip every later fix, got {}",
            result.gps_skipped
        );
        // Dead reckoning drifts slowly, so neither divergence clause trips:
        // no flag, and therefore no rising-edge critical fault.
        assert!(
            !result.ekf_diverged,
            "a suppressed filter must not be called diverged"
        );
        assert!(
            !result.phases_visited().contains(&FlightPhase::Emergency),
            "no emergency from a suppressed filter, timeline {:?}",
            result.phase_timeline
        );
        // And the mission runs its normal course into RTL.
        assert!(
            result.phases_visited().contains(&FlightPhase::Rtl),
            "the mission should complete into RTL, timeline {:?}",
            result.phase_timeline
        );
    }

    #[test]
    fn an_authored_adversarial_gps_divergence_reaches_emergency() {
        let cfg = SimConfig {
            waypoints: vec![waypoint(40.0, 0.0, -10.0, 3.0)],
            faults: vec![FaultSpec::EkfDivergence {
                at_s: 6.0,
                mode: DivergenceMode::AdversarialGps,
            }],
            ..Default::default()
        };
        let result = fly(cfg, 20.0);
        // The +50 m offset makes the converged filter gate every later fix,
        // the gated-streak clause trips divergence, and the rising edge
        // raises exactly one critical_fault into EMERGENCY.
        assert!(result.gps_gated > 0, "the offset should gate fixes");
        assert!(result.ekf_diverged, "the gated streak should diverge");
        let critical: Vec<_> = result
            .event_log
            .iter()
            .filter(|e| e.event == "critical_fault")
            .collect();
        assert_eq!(
            critical.len(),
            1,
            "one rising-edge response, log {:?}",
            result.event_log
        );
        assert!(critical[0].guard_ok);
        assert!(
            result.phases_visited().contains(&FlightPhase::Emergency),
            "an adversarial offset should reach EMERGENCY, timeline {:?}",
            result.phase_timeline
        );
    }

    #[test]
    fn an_authored_battery_soc_drop_triggers_rtl() {
        let cfg = SimConfig {
            // A far waypoint, so the mission is still in progress when the
            // pack is forced down, exactly like the low-battery scenario.
            waypoints: vec![waypoint(200.0, 0.0, -10.0, 3.0)],
            faults: vec![FaultSpec::BatterySoc {
                at_s: 12.0,
                soc: 0.1,
            }],
            ..Default::default()
        };
        let result = fly(cfg, 30.0);
        let visited = result.phases_visited();
        assert!(
            visited.contains(&FlightPhase::Waypoint),
            "the mission should have started first, timeline {:?}",
            result.phase_timeline
        );
        assert!(
            visited.contains(&FlightPhase::Rtl),
            "soc 0.1 should have triggered RTL, timeline {:?}",
            result.phase_timeline
        );
    }

    #[test]
    fn an_authored_motor_fault_reaches_emergency() {
        let cfg = SimConfig {
            faults: vec![FaultSpec::MotorFault { at_s: 4.0 }],
            ..Default::default()
        };
        // Run well past the emergency landing, so the sticky flag has every
        // chance to re-trip if the edge discipline ever broke.
        let result = fly(cfg, 25.0);
        let visited = result.phases_visited();
        assert!(
            visited.contains(&FlightPhase::Emergency),
            "a raised motor-fault flag should reach EMERGENCY, timeline {:?}",
            result.phase_timeline
        );
        // One response per flag lifetime: exactly one critical_fault, on the
        // flag's rising edge, even though the run then lands and settles.
        let critical: Vec<_> = result
            .event_log
            .iter()
            .filter(|e| e.event == "critical_fault")
            .collect();
        assert_eq!(
            critical.len(),
            1,
            "one critical_fault per flag lifetime, log {:?}",
            result.event_log
        );
        assert!(critical[0].guard_ok);
        assert_eq!(critical[0].to_phase, FlightPhase::Emergency);
        assert!(
            (critical[0].time - 4.0).abs() < DT,
            "tripped at {}",
            critical[0].time
        );
        // The absence of a second trip is only meaningful if the run really
        // did land and settle while the flag stayed raised.
        assert!(
            visited.contains(&FlightPhase::Land),
            "the emergency descent should land, timeline {:?}",
            result.phase_timeline
        );
        assert!(
            matches!(result.final_phase(), FlightPhase::Idle | FlightPhase::Land),
            "the vehicle settles after landing, final {:?}",
            result.final_phase()
        );
    }

    #[test]
    fn an_authored_emergency_reaches_emergency() {
        let cfg = SimConfig {
            faults: vec![FaultSpec::Emergency { at_s: 4.0 }],
            ..Default::default()
        };
        let result = fly(cfg, 15.0);
        assert!(
            result.phases_visited().contains(&FlightPhase::Emergency),
            "the forced jump should be on the timeline, timeline {:?}",
            result.phase_timeline
        );
        let forced = result
            .event_log
            .iter()
            .find(|e| e.event == "force:authored fault")
            .expect("the forced jump is logged for post-analysis");
        assert!(forced.guard_ok);
        assert_eq!(forced.to_phase, FlightPhase::Emergency);
        assert!((forced.time - 4.0).abs() < DT, "forced at {}", forced.time);
    }

    #[test]
    fn an_authored_rtl_reaches_rtl() {
        let cfg = SimConfig {
            faults: vec![FaultSpec::Rtl { at_s: 6.0 }],
            ..Default::default()
        };
        let result = fly(cfg, 15.0);
        assert!(
            result.phases_visited().contains(&FlightPhase::Rtl),
            "an authored operator RTL should be accepted, timeline {:?}",
            result.phase_timeline
        );
    }

    #[test]
    fn an_authored_fault_fires_once_at_its_time_and_not_before() {
        let cfg = SimConfig {
            faults: vec![FaultSpec::Rtl { at_s: 10.0 }],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let rtl_requests = |r: &SimRunner| {
            r.state_machine()
                .event_log()
                .iter()
                .filter(|e| e.event == "rtl")
                .count()
        };

        // The pre-roll runs on a negative clock and drains no schedule, so
        // nothing has fired yet.
        r.preroll();
        assert_eq!(rtl_requests(&r), 0, "the pre-roll must not fire faults");

        // Step while the clock is still below `at_s`: the fault stays queued.
        let mut i = 0;
        while r.time() < 10.0 {
            r.step_once(i);
            i += 1;
            assert_eq!(rtl_requests(&r), 0, "fired early at t = {}", r.time());
        }

        // The first step that starts at or past `at_s` fires it, once.
        r.step_once(i);
        assert_eq!(rtl_requests(&r), 1, "fires at at_s = 10");

        // And it never fires again for the rest of the run.
        for j in (i + 1)..(i + 1000) {
            r.step_once(j);
        }
        assert_eq!(rtl_requests(&r), 1, "fires exactly once");
    }

    #[test]
    fn a_fault_at_the_last_step_time_fires_exactly_once() {
        // The window's top is the run's own last step clock, inclusive:
        // 60 s is 12000 steps, the last step starts at 59.995, and a fault
        // exactly there fires on that final step. The RTL request is the
        // counter: every application logs exactly one `rtl` entry, accepted
        // or rejected.
        let duration_s = 60.0;
        let at_s = 59.995;
        let cfg = SimConfig {
            faults: vec![FaultSpec::Rtl { at_s }],
            ..Default::default()
        };
        let mut r = SimRunner::new(cfg).unwrap();
        r.request_event("arm", None);
        r.request_event("takeoff", None);
        let result = r.run(duration_s).unwrap();
        let requests: Vec<_> = result
            .event_log
            .iter()
            .filter(|e| e.event == "rtl")
            .collect();
        assert_eq!(
            requests.len(),
            1,
            "fires exactly once on the last step, log {:?}",
            result.event_log
        );
        // The logged clock is the last step's own, within one step of the
        // bound, and the run ended right after it.
        assert!(
            (requests[0].time - at_s).abs() <= DT,
            "fires on the last step, got {}",
            requests[0].time
        );
        assert!(requests[0].time < duration_s, "before the run's end");
    }

    #[test]
    fn equal_fire_times_apply_in_scenario_order() {
        // Two faults at the same instant: the event log records their
        // application order, and it must follow the list order both ways.
        let order = |faults: Vec<FaultSpec>| {
            let cfg = SimConfig {
                faults,
                ..Default::default()
            };
            let mut r = SimRunner::new(cfg).unwrap();
            let result = r.run(7.0).unwrap();
            let rtl = result
                .event_log
                .iter()
                .position(|e| e.event == "rtl")
                .expect("the rtl fault logs its request");
            let forced = result
                .event_log
                .iter()
                .position(|e| e.event.starts_with("force:"))
                .expect("the emergency fault logs its jump");
            (rtl, forced)
        };

        let (rtl_first, forced_first) = order(vec![
            FaultSpec::Rtl { at_s: 5.0 },
            FaultSpec::Emergency { at_s: 5.0 },
        ]);
        assert!(
            rtl_first < forced_first,
            "list order: rtl at {rtl_first} before force at {forced_first}"
        );

        let (rtl_second, forced_second) = order(vec![
            FaultSpec::Emergency { at_s: 5.0 },
            FaultSpec::Rtl { at_s: 5.0 },
        ]);
        assert!(
            forced_second < rtl_second,
            "list order reversed: force at {forced_second} before rtl at {rtl_second}"
        );
    }
}
