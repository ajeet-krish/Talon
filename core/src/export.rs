//! Schema 1.1 flight document assembly.
//!
//! Ported from the previous Python source (`export_json.py`). The live run and
//! the batch runs both produce the same document the scene, the charts and the
//! recorded scenarios consume, so nothing downstream needs to know which engine
//! produced it.
//!
//! Two details are preserved deliberately from the Python exporter:
//!
//! * `meta.duration_s` is the LAST sample time, not the requested duration, so a
//!   run cut short still reports what it actually flew.
//! * Floats are rounded to `round_decimals` (6 by default) before serialization,
//!   which is what keeps a 120 s control-rate history a sensible size on the
//!   wire to the browser.
//!
//! `meta.generated_utc` is a caller-supplied string rather than a clock read,
//! because this crate has no clock: on the web it comes from the JavaScript
//! `Date`, and in tests and the batch harness it can be any fixed stamp. That
//! also makes every export reproducible in a test.

use std::collections::BTreeMap;

use crate::sim::SimResult;
use crate::types::{Arr3, FlightData, FlightMeta, Obstacle, PhaseEvent, SCHEMA_VERSION};

/// Sampling period of the exported histories (s), the control rate.
pub const SAMPLE_DT_S: f64 = 0.01;

/// Default rounding for exported floats, matching the Python exporter.
pub const DEFAULT_ROUND_DECIMALS: u32 = 6;

/// The `meta.units` legend, by quantity.
pub fn units() -> BTreeMap<String, String> {
    let pairs = [
        ("time", "s"),
        ("position", "m NED"),
        ("velocity", "m/s NED"),
        ("acceleration", "m/s^2"),
        ("angular_rate", "rad/s"),
        ("thrust", "N"),
        ("moment", "N*m"),
        ("wind", "m/s"),
        ("covariance", "m^2"),
    ];
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

fn round_to(value: f64, decimals: Option<u32>) -> f64 {
    match decimals {
        None => value,
        Some(d) => {
            let factor = 10f64.powi(d as i32);
            (value * factor).round() / factor
        }
    }
}

fn round_arr(value: Arr3, decimals: Option<u32>) -> Arr3 {
    [
        round_to(value[0], decimals),
        round_to(value[1], decimals),
        round_to(value[2], decimals),
    ]
}

fn round_vec(values: &[Arr3], decimals: Option<u32>) -> Vec<Arr3> {
    values.iter().map(|v| round_arr(*v, decimals)).collect()
}

fn round_1d(values: &[f64], decimals: Option<u32>) -> Vec<f64> {
    values.iter().map(|v| round_to(*v, decimals)).collect()
}

/// Build the schema 1.1 flight document from a finished run.
///
/// `scenario` is written to `meta.scenario` (the config id), `seed` to
/// `meta.seed`, and `generated_utc` is the caller-supplied UTC stamp. The phase
/// timeline is sorted ascending by time, as the Python exporter does, so a
/// consumer can scan it front to back.
pub fn flight_data(
    result: &SimResult,
    scenario: &str,
    seed: u64,
    obstacles: Vec<Obstacle>,
    generated_utc: String,
    round_decimals: Option<u32>,
) -> FlightData {
    let duration_s = result
        .time_s
        .last()
        .map(|t| round_to(*t, round_decimals))
        .unwrap_or(0.0);

    let mut phase_timeline: Vec<PhaseEvent> = result
        .phase_timeline
        .iter()
        .map(|(t, p)| (round_to(*t, round_decimals), *p))
        .collect();
    phase_timeline.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut event_log = result.event_log.clone();
    for event in event_log.iter_mut() {
        event.time = round_to(event.time, round_decimals);
    }

    FlightData {
        meta: FlightMeta {
            scenario: scenario.to_string(),
            seed,
            schema_version: SCHEMA_VERSION.to_string(),
            generated_utc,
            sample_count: result.time_s.len(),
            duration_s,
            dt: SAMPLE_DT_S,
            units: units(),
        },
        time_s: round_1d(&result.time_s, round_decimals),
        truth_position: round_vec(&result.truth_position, round_decimals),
        truth_velocity: round_vec(&result.truth_velocity, round_decimals),
        ekf_position: round_vec(&result.ekf_position, round_decimals),
        ekf_velocity: round_vec(&result.ekf_velocity, round_decimals),
        position_var: round_vec(&result.position_var, round_decimals),
        gyro_bias: round_vec(&result.gyro_bias, round_decimals),
        accel_bias: round_vec(&result.accel_bias, round_decimals),
        wind_ned: round_vec(&result.wind_ned, round_decimals),
        command_thrust: round_1d(&result.command_thrust, round_decimals),
        command_moment: round_vec(&result.command_moment, round_decimals),
        e_ct: round_1d(&result.e_ct, round_decimals),
        e_ct_times: round_1d(&result.e_ct_times, round_decimals),
        e_ct_leg: round_1d(&result.e_ct_leg, round_decimals),
        waypoints_ned: round_vec(&result.waypoints_ned, round_decimals),
        home_ned: round_arr(result.home_ned, round_decimals),
        obstacles,
        phase_timeline,
        event_log,
    }
}

/// Serialize a flight document to compact JSON.
pub fn to_json(data: &FlightData) -> Result<String, serde_json::Error> {
    serde_json::to_string(data)
}

/// Serialize a flight document to indented JSON.
pub fn to_json_pretty(data: &FlightData) -> Result<String, serde_json::Error> {
    serde_json::to_string_pretty(data)
}

/// Convenience: build and serialize in one call.
pub fn flight_data_json(
    result: &SimResult,
    scenario: &str,
    seed: u64,
    obstacles: Vec<Obstacle>,
    generated_utc: String,
    round_decimals: Option<u32>,
) -> Result<String, serde_json::Error> {
    to_json(&flight_data(
        result,
        scenario,
        seed,
        obstacles,
        generated_utc,
        round_decimals,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{SimConfig, SimRunner};
    use crate::types::Waypoint;

    fn fly() -> (SimResult, SimConfig) {
        let config = SimConfig {
            waypoints: vec![Waypoint::new([30.0, 20.0, -10.0], 5.0, 0.0, 3.0)],
            ..Default::default()
        };
        let mut runner = SimRunner::new(config.clone()).unwrap();
        runner.request_event("arm", None);
        runner.request_event("takeoff", None);
        let result = runner.run(45.0).unwrap();
        (result, config)
    }

    #[test]
    fn the_document_carries_the_schema_version_and_units() {
        let (result, config) = fly();
        let doc = flight_data(
            &result,
            "waypoints",
            config.seed,
            Vec::new(),
            "2026-10-18T00:00:00Z".to_string(),
            Some(DEFAULT_ROUND_DECIMALS),
        );
        assert_eq!(doc.meta.schema_version, SCHEMA_VERSION);
        assert_eq!(doc.meta.schema_version, "1.1");
        assert_eq!(doc.meta.scenario, "waypoints");
        assert_eq!(doc.meta.seed, config.seed);
        assert_eq!(doc.meta.dt, 0.01);
        assert_eq!(doc.meta.units.len(), 9);
        assert_eq!(doc.meta.units.get("position").map(String::as_str), Some("m NED"));
    }

    #[test]
    fn the_sample_count_matches_every_series() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), Some(6));
        let n = doc.meta.sample_count;
        assert!(n > 0);
        assert_eq!(doc.time_s.len(), n);
        assert_eq!(doc.truth_position.len(), n);
        assert_eq!(doc.truth_velocity.len(), n);
        assert_eq!(doc.ekf_position.len(), n);
        assert_eq!(doc.ekf_velocity.len(), n);
        assert_eq!(doc.position_var.len(), n);
        assert_eq!(doc.gyro_bias.len(), n);
        assert_eq!(doc.accel_bias.len(), n);
        assert_eq!(doc.wind_ned.len(), n);
        assert_eq!(doc.command_thrust.len(), n);
        assert_eq!(doc.command_moment.len(), n);
        assert_eq!(doc.e_ct.len(), doc.e_ct_times.len());
        assert_eq!(doc.e_ct.len(), doc.e_ct_leg.len());
    }

    #[test]
    fn duration_is_the_last_sample_time() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), Some(6));
        let last = *doc.time_s.last().unwrap();
        assert!((doc.meta.duration_s - last).abs() < 1e-12);
        // 45 s at the control rate means about 4500 samples, so the last time
        // is a hair under 45 s.
        assert!((doc.meta.duration_s - 44.99).abs() < 0.02, "{}", doc.meta.duration_s);
    }

    #[test]
    fn rounding_is_applied_to_every_float() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), Some(6));
        for t in &doc.time_s {
            let scaled = t * 1e6;
            assert!(
                (scaled - scaled.round()).abs() < 1e-6,
                "time {t} is not rounded to 6 decimals"
            );
        }
        for p in &doc.truth_position {
            for c in p {
                let scaled = c * 1e6;
                assert!((scaled - scaled.round()).abs() < 1e-3, "{c}");
            }
        }
    }

    #[test]
    fn no_rounding_keeps_full_precision() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), None);
        let mut found_long = false;
        for t in &doc.time_s {
            let scaled = t * 1e6;
            if (scaled - scaled.round()).abs() > 1e-6 {
                found_long = true;
                break;
            }
        }
        assert!(found_long, "None should preserve the unrounded samples");
    }

    #[test]
    fn the_phase_timeline_is_sorted_ascending() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), Some(6));
        assert!(!doc.phase_timeline.is_empty());
        for pair in doc.phase_timeline.windows(2) {
            assert!(
                pair[0].0 <= pair[1].0,
                "timeline out of order: {:?}",
                doc.phase_timeline
            );
        }
    }

    #[test]
    fn obstacles_are_serialized_and_survive_a_json_round_trip() {
        let (result, _) = fly();
        let obstacle = Obstacle {
            id: "obs_north".to_string(),
            center_ned: [30.0, 0.0, -10.0],
            radius_m: 4.0,
        };
        let json = flight_data_json(
            &result,
            "waypoints",
            result.phase_timeline.len() as u64,
            vec![obstacle.clone()],
            "x".into(),
            Some(6),
        )
        .unwrap();
        assert!(json.contains("obs_north"));
        let parsed: FlightData = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.obstacles.len(), 1);
        assert_eq!(parsed.obstacles[0].id, "obs_north");
        assert_eq!(parsed.obstacles[0].radius_m, 4.0);
        assert_eq!(parsed.obstacles[0].center_ned, [30.0, 0.0, -10.0]);
    }

    #[test]
    fn a_missing_obstacles_key_reads_as_empty() {
        // Schema 1.0 forward compatibility: the loader must never see undefined.
        let json = r#"{
            "meta": {"scenario":"x","seed":1,"schema_version":"1.0",
                     "generated_utc":"x","sample_count":0,"duration_s":0.0,
                     "dt":0.01,"units":{}},
            "time_s": [], "truth_position": [], "truth_velocity": [],
            "ekf_position": [], "ekf_velocity": [], "position_var": [],
            "gyro_bias": [], "accel_bias": [], "wind_ned": [],
            "command_thrust": [], "command_moment": [], "e_ct": [],
            "e_ct_times": [], "e_ct_leg": [], "waypoints_ned": [],
            "home_ned": [0.0,0.0,0.0], "phase_timeline": [], "event_log": []
        }"#;
        let parsed: FlightData = serde_json::from_str(json).unwrap();
        assert!(parsed.obstacles.is_empty());
    }

    #[test]
    fn event_log_phases_serialize_lowercase() {
        let (result, _) = fly();
        let doc = flight_data(&result, "waypoints", 1, Vec::new(), "x".into(), Some(6));
        let json = to_json(&doc).unwrap();
        assert!(json.contains("\"from_phase\":\"idle\""), "phases must be lowercase");
        assert!(!json.contains("\"Idle\""), "no Rust-style phase names in the document");
    }

    #[test]
    fn the_json_is_parseable_and_carries_the_mission() {
        let (result, config) = fly();
        let json = flight_data_json(
            &result,
            "waypoints",
            config.seed,
            Vec::new(),
            "2026-10-18T00:00:00Z".into(),
            Some(6),
        )
        .unwrap();
        let parsed: FlightData = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.waypoints_ned.len(), 1);
        assert_eq!(parsed.waypoints_ned[0], [30.0, 20.0, -10.0]);
        assert_eq!(parsed.home_ned, [0.0, 0.0, 0.0]);
        assert!(parsed.event_log.iter().any(|e| e.event == "takeoff"));
    }
}
