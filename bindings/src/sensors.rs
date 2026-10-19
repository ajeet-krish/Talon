//! Python boundary for the truth-side IMU and GPS measurement models.
//!
//! The core's `sensors` module ([`talon_core::sensors`]) is the single
//! implementation. This file only marshals: the noise conversion, the bias
//! random walk, the draw order and the flat-Earth GPS mapping are the same
//! numbers the SIL and the browser run.
//!
//! Caller-shape adaptation, noted here as the mission boundary notes its
//! tuples: the Rust core takes plain vectors where the retired Python module
//! took a `talon.State` object. `SensorSuite.generate_imu` takes
//! `true_angular_rate` (the state's body angular rate) instead of the state,
//! and `SensorSuite.generate_gps` takes `true_position` and `true_velocity`
//! instead of the state, so one Python `talon.State` is read for those fields
//! by the caller before the call. `gps_to_ned` and `gps_velocity_ned` accept
//! any object with the `GPSMeasurement` fields (`lat`, `lon`, `alt`, `vn`,
//! `ve`, `vd`), so one `talon.types.GPSMeasurement` drives both sides. The
//! optional `timestamp` field defaults only when `getattr` reports
//! `AttributeError`; an attribute that exists but raises on access
//! propagates its exception rather than silently reading as the default.
//!
//! Noisy realisations differ between the two sides by design: numpy's
//! ziggurat stream and the core's `Rng` are different generators
//! (docs/decisions.md, decision D18), so the parity suite compares only the
//! zero-noise configurations and the deterministic parts.
//!
//! A non-positive `dt` in `generate_imu` and `specific_force_from_velocity`
//! follows the Rust core (zero sample, zero force, no clock advance), while
//! the Python module raised `ValueError` there; the parity suite pins those
//! differences as registered divergences (`sensors.generate_imu_non_positive_dt`,
//! `sensors.specific_force_non_positive_dt`).

use pyo3::exceptions::PyAttributeError;
use pyo3::prelude::*;

use talon_core::sensors::{
    gps_to_ned, gps_velocity_ned, specific_force_from_velocity, GpsConfig, GpsMeasurement,
    ImuConfig, ImuMeasurement, SensorSuite,
};

use crate::convert::{arr3_from, mat3_from, vec3_from, vec3_to};

/// An attribute that the caller may omit entirely.
///
/// Only a genuinely missing attribute defaults: `getattr` raising
/// `AttributeError` is Python's own signal for "no such attribute", so
/// that is the single case treated as absent. An attribute that exists
/// but raises on access (a broken property) propagates its exception
/// instead of silently reading as the default.
fn optional_f64(value: &Bound<'_, PyAny>, name: &str) -> PyResult<Option<f64>> {
    match value.getattr(name) {
        Ok(attr) => Ok(Some(attr.extract()?)),
        Err(error) if error.is_instance_of::<PyAttributeError>(value.py()) => Ok(None),
        Err(error) => Err(error),
    }
}

/// IMU noise and bias parameters, with the same defaults as the Python
/// dataclass.
#[pyclass(name = "IMUConfig")]
pub struct PyImuConfig {
    pub(crate) inner: ImuConfig,
}

#[pymethods]
impl PyImuConfig {
    #[new]
    #[pyo3(signature = (
        accel_noise_density = None,
        gyro_noise_density = None,
        accel_bias_initial = None,
        gyro_bias_initial = None,
        accel_bias_walk = None,
        gyro_bias_walk = None,
    ))]
    fn new(
        accel_noise_density: Option<f64>,
        gyro_noise_density: Option<f64>,
        accel_bias_initial: Option<Bound<'_, PyAny>>,
        gyro_bias_initial: Option<Bound<'_, PyAny>>,
        accel_bias_walk: Option<f64>,
        gyro_bias_walk: Option<f64>,
    ) -> PyResult<Self> {
        let defaults = ImuConfig::default();
        Ok(Self {
            inner: ImuConfig {
                accel_noise_density: accel_noise_density.unwrap_or(defaults.accel_noise_density),
                gyro_noise_density: gyro_noise_density.unwrap_or(defaults.gyro_noise_density),
                accel_bias_initial: match accel_bias_initial {
                    Some(value) => arr3_from(&value, "accel_bias_initial")?,
                    None => defaults.accel_bias_initial,
                },
                gyro_bias_initial: match gyro_bias_initial {
                    Some(value) => arr3_from(&value, "gyro_bias_initial")?,
                    None => defaults.gyro_bias_initial,
                },
                accel_bias_walk: accel_bias_walk.unwrap_or(defaults.accel_bias_walk),
                gyro_bias_walk: gyro_bias_walk.unwrap_or(defaults.gyro_bias_walk),
            },
        })
    }

    /// Continuous accelerometer noise density (m/s^2/sqrt(Hz)).
    #[getter]
    fn accel_noise_density(&self) -> f64 {
        self.inner.accel_noise_density
    }

    #[setter]
    fn set_accel_noise_density(&mut self, value: f64) {
        self.inner.accel_noise_density = value;
    }

    /// Continuous gyro noise density (rad/s/sqrt(Hz)).
    #[getter]
    fn gyro_noise_density(&self) -> f64 {
        self.inner.gyro_noise_density
    }

    #[setter]
    fn set_gyro_noise_density(&mut self, value: f64) {
        self.inner.gyro_noise_density = value;
    }

    /// True accelerometer bias at t = 0 (m/s^2, body).
    #[getter]
    fn accel_bias_initial(&self) -> Vec<f64> {
        self.inner.accel_bias_initial.to_vec()
    }

    #[setter]
    fn set_accel_bias_initial(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.accel_bias_initial = arr3_from(value, "accel_bias_initial")?;
        Ok(())
    }

    /// True gyro bias at t = 0 (rad/s, body).
    #[getter]
    fn gyro_bias_initial(&self) -> Vec<f64> {
        self.inner.gyro_bias_initial.to_vec()
    }

    #[setter]
    fn set_gyro_bias_initial(&mut self, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.inner.gyro_bias_initial = arr3_from(value, "gyro_bias_initial")?;
        Ok(())
    }

    /// Accelerometer bias random walk (m/s^2/sqrt(s)).
    #[getter]
    fn accel_bias_walk(&self) -> f64 {
        self.inner.accel_bias_walk
    }

    #[setter]
    fn set_accel_bias_walk(&mut self, value: f64) {
        self.inner.accel_bias_walk = value;
    }

    /// Gyro bias random walk (rad/s/sqrt(s)).
    #[getter]
    fn gyro_bias_walk(&self) -> f64 {
        self.inner.gyro_bias_walk
    }

    #[setter]
    fn set_gyro_bias_walk(&mut self, value: f64) {
        self.inner.gyro_bias_walk = value;
    }
}

/// GPS noise parameters (1-sigma in metres and m/s in local NED), with the
/// same defaults as the Python dataclass.
#[pyclass(name = "GPSConfig")]
pub struct PyGpsConfig {
    pub(crate) inner: GpsConfig,
}

#[pymethods]
impl PyGpsConfig {
    #[new]
    #[pyo3(signature = (
        position_noise = None,
        position_noise_vertical = None,
        velocity_noise = None,
        update_rate = None,
    ))]
    fn new(
        position_noise: Option<f64>,
        position_noise_vertical: Option<f64>,
        velocity_noise: Option<f64>,
        update_rate: Option<f64>,
    ) -> Self {
        let defaults = GpsConfig::default();
        Self {
            inner: GpsConfig {
                position_noise: position_noise.unwrap_or(defaults.position_noise),
                position_noise_vertical: position_noise_vertical
                    .unwrap_or(defaults.position_noise_vertical),
                velocity_noise: velocity_noise.unwrap_or(defaults.velocity_noise),
                update_rate: update_rate.unwrap_or(defaults.update_rate),
            },
        }
    }

    /// Horizontal position 1-sigma (m).
    #[getter]
    fn position_noise(&self) -> f64 {
        self.inner.position_noise
    }

    #[setter]
    fn set_position_noise(&mut self, value: f64) {
        self.inner.position_noise = value;
    }

    /// Vertical position 1-sigma (m).
    #[getter]
    fn position_noise_vertical(&self) -> f64 {
        self.inner.position_noise_vertical
    }

    #[setter]
    fn set_position_noise_vertical(&mut self, value: f64) {
        self.inner.position_noise_vertical = value;
    }

    /// Velocity 1-sigma (m/s).
    #[getter]
    fn velocity_noise(&self) -> f64 {
        self.inner.velocity_noise
    }

    #[setter]
    fn set_velocity_noise(&mut self, value: f64) {
        self.inner.velocity_noise = value;
    }

    /// Update rate (Hz).
    #[getter]
    fn update_rate(&self) -> f64 {
        self.inner.update_rate
    }

    #[setter]
    fn set_update_rate(&mut self, value: f64) {
        self.inner.update_rate = value;
    }
}

/// One IMU sample. Read-only.
#[pyclass(name = "IMUMeasurement")]
pub struct PyImuMeasurement {
    inner: ImuMeasurement,
}

#[pymethods]
impl PyImuMeasurement {
    /// Sample timestamp (s).
    #[getter]
    fn timestamp(&self) -> f64 {
        self.inner.timestamp
    }

    /// Specific force in body frame (m/s^2).
    #[getter]
    fn accel(&self) -> Vec<f64> {
        self.inner.accel.to_vec()
    }

    /// Body angular rate (rad/s).
    #[getter]
    fn gyro(&self) -> Vec<f64> {
        self.inner.gyro.to_vec()
    }
}

/// One GPS fix: lat/lon in degrees, alt in m, NED velocity in m/s.
/// Read-only.
#[pyclass(name = "GPSMeasurement")]
pub struct PyGpsMeasurement {
    inner: GpsMeasurement,
}

#[pymethods]
impl PyGpsMeasurement {
    /// Sample timestamp (s).
    #[getter]
    fn timestamp(&self) -> f64 {
        self.inner.timestamp
    }

    /// Latitude (degrees).
    #[getter]
    fn lat(&self) -> f64 {
        self.inner.lat
    }

    /// Longitude (degrees).
    #[getter]
    fn lon(&self) -> f64 {
        self.inner.lon
    }

    /// Altitude (m, positive up).
    #[getter]
    fn alt(&self) -> f64 {
        self.inner.alt
    }

    /// North velocity (m/s).
    #[getter]
    fn vn(&self) -> f64 {
        self.inner.vn
    }

    /// East velocity (m/s).
    #[getter]
    fn ve(&self) -> f64 {
        self.inner.ve
    }

    /// Down velocity (m/s).
    #[getter]
    fn vd(&self) -> f64 {
        self.inner.vd
    }
}

/// True body-frame specific force from the finite difference of true NED
/// velocity. A non-positive `dt` yields zero force, as the core does.
#[pyfunction]
#[pyo3(name = "specific_force_from_velocity")]
#[pyo3(signature = (v_prev_ned, v_curr_ned, rotation_body_to_ned, dt))]
fn specific_force_py(
    v_prev_ned: &Bound<'_, PyAny>,
    v_curr_ned: &Bound<'_, PyAny>,
    rotation_body_to_ned: &Bound<'_, PyAny>,
    dt: f64,
) -> PyResult<Vec<f64>> {
    let force = specific_force_from_velocity(
        &vec3_from(v_prev_ned, "v_prev_ned")?,
        &vec3_from(v_curr_ned, "v_curr_ned")?,
        &mat3_from(rotation_body_to_ned, "rotation_body_to_ned")?,
        dt,
    );
    Ok(vec3_to(&force))
}

/// Read the scalar fields `gps_to_ned` and `gps_velocity_ned` need.
///
/// Only marshalling: the flat-Earth mapping itself stays in the core.
fn gps_fields_from(value: &Bound<'_, PyAny>) -> PyResult<GpsMeasurement> {
    Ok(GpsMeasurement {
        // Timestamp is not part of the mapping; the core functions ignore it.
        timestamp: optional_f64(value, "timestamp")?.unwrap_or(0.0),
        lat: value.getattr("lat")?.extract()?,
        lon: value.getattr("lon")?.extract()?,
        alt: value.getattr("alt")?.extract()?,
        vn: value.getattr("vn")?.extract()?,
        ve: value.getattr("ve")?.extract()?,
        vd: value.getattr("vd")?.extract()?,
    })
}

/// GPS fix to local NED position `[n, e, d]` (m), flat-Earth mapping.
#[pyfunction]
#[pyo3(name = "gps_to_ned")]
fn gps_to_ned_py(gps: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    Ok(gps_to_ned(&gps_fields_from(gps)?).to_vec())
}

/// GPS fix to NED velocity `[vn, ve, vd]` (m/s).
#[pyfunction]
#[pyo3(name = "gps_velocity_ned")]
fn gps_velocity_ned_py(gps: &Bound<'_, PyAny>) -> PyResult<Vec<f64>> {
    Ok(gps_velocity_ned(&gps_fields_from(gps)?).to_vec())
}

/// Seeded IMU and GPS truth-side measurement generator.
///
/// Holds the true bias states, initialized from the `IMUConfig` snapshot at
/// construction and random-walked on every `generate_imu` call.
#[pyclass(name = "SensorSuite")]
pub struct PySensorSuite {
    inner: SensorSuite,
}

#[pymethods]
impl PySensorSuite {
    #[new]
    #[pyo3(signature = (imu_config = None, gps_config = None, seed = 0))]
    fn new(
        imu_config: Option<PyRef<'_, PyImuConfig>>,
        gps_config: Option<PyRef<'_, PyGpsConfig>>,
        seed: u64,
    ) -> Self {
        Self {
            inner: SensorSuite::new(
                imu_config.map(|c| c.inner).unwrap_or_default(),
                gps_config.map(|c| c.inner).unwrap_or_default(),
                seed,
            ),
        }
    }

    /// Current true accelerometer bias (m/s^2, body).
    #[getter]
    fn accel_bias(&self) -> Vec<f64> {
        vec3_to(&self.inner.accel_bias())
    }

    /// Current true gyro bias (rad/s, body).
    #[getter]
    fn gyro_bias(&self) -> Vec<f64> {
        vec3_to(&self.inner.gyro_bias())
    }

    /// Internal sensor clock (s), advanced by `generate_imu`.
    #[getter]
    fn time(&self) -> f64 {
        self.inner.time()
    }

    /// One IMU sample from the true body angular rate and true body
    /// specific force (see the module docstring for the caller-shape
    /// adaptation from the Python `true_state` argument).
    #[pyo3(signature = (true_angular_rate, true_specific_force_body, dt))]
    fn generate_imu(
        &mut self,
        py: Python<'_>,
        true_angular_rate: &Bound<'_, PyAny>,
        true_specific_force_body: &Bound<'_, PyAny>,
        dt: f64,
    ) -> PyResult<Py<PyImuMeasurement>> {
        let measurement = self.inner.generate_imu(
            &vec3_from(true_angular_rate, "true_angular_rate")?,
            arr3_from(true_specific_force_body, "true_specific_force_body")?,
            dt,
        );
        Py::new(py, PyImuMeasurement { inner: measurement })
    }

    /// One GPS fix from the true NED position and velocity (see the module
    /// docstring for the caller-shape adaptation from the Python
    /// `true_state` argument).
    #[pyo3(signature = (true_position, true_velocity))]
    fn generate_gps(
        &mut self,
        py: Python<'_>,
        true_position: &Bound<'_, PyAny>,
        true_velocity: &Bound<'_, PyAny>,
    ) -> PyResult<Py<PyGpsMeasurement>> {
        let position = vec3_from(true_position, "true_position")?;
        let velocity = vec3_from(true_velocity, "true_velocity")?;
        let measurement = self.inner.generate_gps(&position, &velocity);
        Py::new(py, PyGpsMeasurement { inner: measurement })
    }
}

/// Register the sensors boundary on the extension module.
pub fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyImuConfig>()?;
    module.add_class::<PyGpsConfig>()?;
    module.add_class::<PyImuMeasurement>()?;
    module.add_class::<PyGpsMeasurement>()?;
    module.add_class::<PySensorSuite>()?;
    module.add_function(wrap_pyfunction!(specific_force_py, module)?)?;
    module.add_function(wrap_pyfunction!(gps_to_ned_py, module)?)?;
    module.add_function(wrap_pyfunction!(gps_velocity_ned_py, module)?)?;
    Ok(())
}
