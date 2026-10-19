//! Core value types and the flight-export document schema (schema 1.1).
//!
//! Two vector flavours appear on purpose, mirroring the previous split between
//! the Eigen kernel and the array-based glue:
//!
//! * [`Arr3`] (`[f64; 3]`) for guidance, mission, sensors, motors, the state
//!   machine, and every document type. Index arithmetic keeps the guidance port
//!   a literal transcription of the Python and TypeScript sources.
//! * [`Vec3`] (`nalgebra::Vector3`) for dynamics, control and estimation, which
//!   need to interoperate with fixed-size matrices.
//!
//! The document types are the single source of truth for the JSON the web app
//! reads. Field names are snake_case and match the on-disk schema exactly.

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::constants::*;

/// NED triple: `[north, east, down]`, metres.
pub type Arr3 = [f64; 3];
/// Nalgebra 3-vector, for the matrix-based math.
pub type Vec3 = Vector3<f64>;
/// Nalgebra 3x3 matrix (direction cosines, inertia).
pub type Mat3 = Matrix3<f64>;

/// Export schema version written into every document.
pub const SCHEMA_VERSION: &str = "1.1";

/// Convert an `Arr3` into a nalgebra vector.
#[inline]
pub fn to_vec3(a: Arr3) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// Convert a nalgebra vector into an `Arr3`.
#[inline]
pub fn to_arr3(v: &Vec3) -> Arr3 {
    [v[0], v[1], v[2]]
}

// ---------------------------------------------------------------------------
// Attitude and state
// ---------------------------------------------------------------------------

/// Hamilton quaternion, scalar-first `[w, x, y, z]`, rotating body into NED:
/// `v_ned = R(q) * v_body`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn norm(&self) -> f64 {
        crate::math::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalized(&self) -> Self {
        let n = self.norm();
        Self::new(self.w / n, self.x / n, self.y / n, self.z / n)
    }

    pub fn conjugate(&self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Body-to-NED direction cosine matrix.
    pub fn to_dcm(&self) -> Mat3 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let (ww, xx, yy, zz) = (w * w, x * x, y * y, z * z);
        let _ = ww; // the DCM formula uses only the cross terms and x, y, z squares
        Mat3::from_row_slice(&[
            1.0 - 2.0 * (yy + zz),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (xx + zz),
            2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (xx + yy),
        ])
    }

    /// Rotate a vector from body frame to NED frame.
    pub fn rotate_vector(&self, v_body: &Vec3) -> Vec3 {
        self.to_dcm() * v_body
    }
}

/// Full vehicle state. Position and velocity in NED, angular rate in body.
#[derive(Debug, Clone, Copy)]
pub struct State {
    pub position: Vec3,
    pub velocity: Vec3,
    pub angular_rate: Vec3,
    pub orientation: Quaternion,
}

impl Default for State {
    fn default() -> Self {
        Self {
            position: Vec3::zeros(),
            velocity: Vec3::zeros(),
            angular_rate: Vec3::zeros(),
            orientation: Quaternion::default(),
        }
    }
}

/// Vehicle physical parameters, defaulting to the values in [`crate::constants`].
#[derive(Debug, Clone, Copy)]
pub struct VehicleParams {
    pub mass: f64,
    pub inertia_diag: Vec3,
    pub arm_length: f64,
    pub drag_coeff: f64,
    pub gravity: f64,
    pub air_density: f64,
    pub vehicle_area: f64,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            mass: MASS,
            inertia_diag: Vec3::new(IXX, IYY, IZZ),
            arm_length: ARM_LENGTH,
            drag_coeff: DRAG_COEFF,
            gravity: GRAVITY,
            air_density: AIR_DENSITY,
            vehicle_area: VEHICLE_AREA,
        }
    }
}

// ---------------------------------------------------------------------------
// Mission and scenario types
// ---------------------------------------------------------------------------

/// High-level autopilot mission phase.
///
/// The `ts-export` derive (D23, Phase 5c) mirrors this into
/// `src/types/generated/talon_core.ts`; serde's lowercase rename is what the
/// generated string union carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
#[serde(rename_all = "lowercase")]
pub enum FlightPhase {
    Idle,
    Arm,
    Takeoff,
    Cruise,
    Waypoint,
    Rtl,
    Land,
    Emergency,
}

impl FlightPhase {
    /// Lowercase label used in the export and the UI.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Arm => "arm",
            Self::Takeoff => "takeoff",
            Self::Cruise => "cruise",
            Self::Waypoint => "waypoint",
            Self::Rtl => "rtl",
            Self::Land => "land",
            Self::Emergency => "emergency",
        }
    }
}

/// One mission waypoint (matches `talon.types.Waypoint`).
///
/// `radius` is the horizontal acceptance radius: the mission advances when the
/// vehicle is within `radius` of `position` in the north-east plane. Altitude is
/// handled by the guidance channel, not the acceptance test.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct Waypoint {
    pub position: Arr3,
    pub speed: f64,
    pub heading: f64,
    pub radius: f64,
}

impl Waypoint {
    /// Waypoint with the documented defaults (5 m/s leg, 2 m acceptance).
    pub fn new(position: Arr3, speed: f64, heading: f64, radius: f64) -> Self {
        Self {
            position,
            speed,
            heading,
            radius,
        }
    }
}

impl Default for Waypoint {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            speed: 5.0,
            heading: 0.0,
            radius: 2.0,
        }
    }
}

/// Spherical keep-out zone (schema 1.1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct Obstacle {
    pub id: String,
    pub center_ned: Arr3,
    pub radius_m: f64,
}

// ---------------------------------------------------------------------------
// Export document (schema 1.1)
// ---------------------------------------------------------------------------

/// Per-document metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct FlightMeta {
    pub scenario: String,
    /// JSON number, not bigint: seeds are ordinary doubles in the schema.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub seed: u64,
    pub schema_version: String,
    pub generated_utc: String,
    /// JSON number, not bigint: the count is a plain sample total on the wire.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub sample_count: usize,
    pub duration_s: f64,
    pub dt: f64,
    pub units: std::collections::BTreeMap<String, String>,
}

/// `phase_timeline` entry: `[time_s, phase]`, sorted by time.
pub type PhaseEvent = (f64, FlightPhase);

/// One discrete autopilot event with its guard outcome.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct EventLogEntry {
    pub time: f64,
    pub event: String,
    pub from_phase: FlightPhase,
    pub to_phase: FlightPhase,
    pub guard_ok: bool,
}

/// One full flight document. Arrays are per sample at the uniform rate in
/// `meta.dt` unless noted.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct FlightData {
    pub meta: FlightMeta,
    pub time_s: Vec<f64>,
    pub truth_position: Vec<Arr3>,
    pub truth_velocity: Vec<Arr3>,
    pub ekf_position: Vec<Arr3>,
    pub ekf_velocity: Vec<Arr3>,
    pub position_var: Vec<Arr3>,
    pub gyro_bias: Vec<Arr3>,
    pub accel_bias: Vec<Arr3>,
    pub wind_ned: Vec<Arr3>,
    pub command_thrust: Vec<f64>,
    pub command_moment: Vec<Arr3>,
    pub e_ct: Vec<f64>,
    pub e_ct_times: Vec<f64>,
    pub e_ct_leg: Vec<f64>,
    pub waypoints_ned: Vec<Arr3>,
    pub home_ned: Arr3,
    /// Schema 1.0 documents omit this key; a missing list reads as empty.
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
    pub phase_timeline: Vec<PhaseEvent>,
    pub event_log: Vec<EventLogEntry>,
}

impl FlightData {
    /// Sample rate implied by `meta.dt`, or 0 when `dt` is not positive.
    pub fn rate_hz(&self) -> f64 {
        if self.meta.dt > 0.0 {
            1.0 / self.meta.dt
        } else {
            0.0
        }
    }
}

/// Headline run numbers that the flight document does not carry.
///
/// The mission outcome, the GPS fix accounting, the estimator health flags,
/// and the truth-versus-estimate error metrics, assembled once in
/// [`crate::export::run_summary`] so the wasm binding, the batch harness, and
/// the page all read the same shape.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub struct RunSummary {
    /// Last sample time (s), rounded like `meta.duration_s`.
    pub duration_s: f64,
    /// JSON number, not bigint: the counts are plain display numbers on the wire.
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub waypoint_count: usize,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub waypoints_reached: usize,
    pub mission_complete: bool,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub gps_accepted: u64,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub gps_gated: u64,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub gps_skipped: u64,
    pub final_phase: FlightPhase,
    pub final_truth_altitude_m: f64,
    pub ekf_converged: bool,
    pub ekf_diverged: bool,
    /// Closest surface clearance (m), null when the scenario had no zones.
    pub min_obstacle_clearance_m: Option<f64>,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub avoidance_activations: u64,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub steps_done: usize,
    #[cfg_attr(feature = "ts-export", ts(type = "number"))]
    pub total_steps: usize,
    /// Root-mean-square 3D distance between the truth and the estimate.
    pub ekf_position_rmse_m: f64,
    /// 3D distance between the last paired truth and estimate samples (m).
    pub final_position_error_m: f64,
    /// The phase timeline with consecutive duplicates collapsed, in order.
    pub phases: Vec<FlightPhase>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_quaternion_gives_identity_dcm() {
        let r = Quaternion::default().to_dcm();
        let identity = Mat3::identity();
        for i in 0..3 {
            for j in 0..3 {
                assert!((r[(i, j)] - identity[(i, j)]).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn ninety_degree_yaw_rotates_north_to_east() {
        // Yaw +90 deg about down: body +x should map to NED +y (east).
        let half = crate::math::to_radians(90.0) / 2.0;
        let q = Quaternion::new(crate::math::cos(half), 0.0, 0.0, crate::math::sin(half));
        let v = q.rotate_vector(&Vec3::new(1.0, 0.0, 0.0));
        assert!(v[0].abs() < 1e-12, "north leaked: {v}");
        assert!((v[1] - 1.0).abs() < 1e-12, "east wrong: {v}");
        assert!(v[2].abs() < 1e-12, "down leaked: {v}");
    }

    #[test]
    fn dcm_is_a_rotation() {
        let q = Quaternion::new(0.7, 0.1, -0.3, 0.6).normalized();
        let r = q.to_dcm();
        let det = r.determinant();
        assert!((det - 1.0).abs() < 1e-12, "determinant {det}");
        let orthogonality = r.transpose() * r;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((orthogonality[(i, j)] - expected).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn phase_labels_match_the_schema() {
        assert_eq!(FlightPhase::Rtl.as_str(), "rtl");
        assert_eq!(FlightPhase::Land.as_str(), "land");
        let json = serde_json::to_string(&FlightPhase::Emergency).unwrap();
        assert_eq!(json, "\"emergency\"");
        let back: FlightPhase = serde_json::from_str("\"waypoint\"").unwrap();
        assert_eq!(back, FlightPhase::Waypoint);
    }

    #[test]
    fn missing_obstacles_read_as_empty() {
        let doc = serde_json::json!({
            "meta": {
                "scenario": "x", "seed": 1, "schema_version": "1.0",
                "generated_utc": "now", "sample_count": 1, "duration_s": 0.01,
                "dt": 0.01, "units": {}
            },
            "time_s": [0.0],
            "truth_position": [[0.0, 0.0, 0.0]],
            "truth_velocity": [[0.0, 0.0, 0.0]],
            "ekf_position": [[0.0, 0.0, 0.0]],
            "ekf_velocity": [[0.0, 0.0, 0.0]],
            "position_var": [[0.0, 0.0, 0.0]],
            "gyro_bias": [[0.0, 0.0, 0.0]],
            "accel_bias": [[0.0, 0.0, 0.0]],
            "wind_ned": [[0.0, 0.0, 0.0]],
            "command_thrust": [0.0],
            "command_moment": [[0.0, 0.0, 0.0]],
            "e_ct": [], "e_ct_times": [], "e_ct_leg": [],
            "waypoints_ned": [], "home_ned": [0.0, 0.0, 0.0],
            "phase_timeline": [], "event_log": []
        });
        let data: FlightData = serde_json::from_value(doc).unwrap();
        assert!(data.obstacles.is_empty());
    }
}
