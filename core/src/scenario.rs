//! Browser and batch scenario configuration.
//!
//! The JSON this deserializes is the format produced by
//! `python/sims/export_live_configs.py` and served at
//! `public/data/live/configs.json`: one object per runnable scenario. It also
//! accepts the shape written into a recorded flight document's `meta`, so a
//! recorded scenario can be replayed through the Rust engine.
//!
//! This type lives in the core rather than in the wasm crate so it is covered
//! by native unit tests, where a schema drift shows up as a failing assert
//! rather than a runtime error inside the browser.
//!
//! Field names are `snake_case` to match the JSON one to one, and every field
//! carries a default so a hand-written or older config still loads. The
//! defaults are the documented scenario values: the Phase 4 GPS noise setting
//! (0.3 m horizontal, 0.5 m vertical, 0.05 m/s), a 10 m takeoff, and a 2 s
//! cruise hold.

use serde::{Deserialize, Serialize};

use crate::avoidance::AvoidanceConfig;
use crate::sim::{SimConfig, SimError};
use crate::types::{Arr3, Obstacle, Waypoint};

fn default_duration() -> f64 {
    60.0
}

fn default_takeoff_altitude() -> f64 {
    10.0
}

fn default_gps_position_noise() -> f64 {
    0.3
}

fn default_gps_position_noise_vertical() -> f64 {
    0.5
}

fn default_gps_velocity_noise() -> f64 {
    0.05
}

fn default_cruise_hold_s() -> f64 {
    2.0
}

fn default_seed() -> u64 {
    20_261_012
}

fn default_battery_soc() -> f64 {
    1.0
}

/// One runnable scenario.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioConfig {
    /// Stable id, also written to `meta.scenario`.
    pub id: String,
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub blurb: String,
    /// Sim seconds of main-loop time, excluding the convergence pre-roll.
    #[serde(default = "default_duration")]
    pub duration_s: f64,
    #[serde(default = "default_takeoff_altitude")]
    pub takeoff_altitude: f64,
    #[serde(default = "default_start_position")]
    pub start_position_ned: Arr3,
    #[serde(default)]
    pub waypoints: Vec<Waypoint>,
    #[serde(default)]
    pub obstacles: Vec<Obstacle>,
    /// Reactive avoidance tuning. Off unless a scenario asks for it, so zones
    /// alone do not change how a planned path is flown.
    #[serde(default)]
    pub avoidance: AvoidanceConfig,
    #[serde(default)]
    pub wind_sigma: [f64; 3],
    #[serde(default)]
    pub wind_steady_ned: Arr3,
    #[serde(default = "default_battery_soc")]
    pub battery_initial_soc: f64,
    #[serde(default = "default_gps_position_noise")]
    pub gps_position_noise: f64,
    #[serde(default = "default_gps_position_noise_vertical")]
    pub gps_position_noise_vertical: f64,
    #[serde(default = "default_gps_velocity_noise")]
    pub gps_velocity_noise: f64,
    #[serde(default = "default_cruise_hold_s")]
    pub cruise_hold_s: f64,
    #[serde(default = "default_seed")]
    pub seed: u64,
    /// Queue arm and takeoff at t = 0. Every FlightLab scenario does.
    #[serde(default)]
    pub auto_arm: bool,
}

fn default_start_position() -> Arr3 {
    [0.0; 3]
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        Self {
            id: "scenario".to_string(),
            label: String::new(),
            blurb: String::new(),
            duration_s: default_duration(),
            takeoff_altitude: default_takeoff_altitude(),
            start_position_ned: [0.0; 3],
            waypoints: Vec::new(),
            obstacles: Vec::new(),
            avoidance: AvoidanceConfig::default(),
            wind_sigma: [0.0; 3],
            wind_steady_ned: [0.0; 3],
            battery_initial_soc: 1.0,
            gps_position_noise: default_gps_position_noise(),
            gps_position_noise_vertical: default_gps_position_noise_vertical(),
            gps_velocity_noise: default_gps_velocity_noise(),
            cruise_hold_s: default_cruise_hold_s(),
            seed: default_seed(),
            auto_arm: false,
        }
    }
}

impl ScenarioConfig {
    /// Parse a scenario from JSON.
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Validate the scenario before a run.
    ///
    /// Checks the duration, which the runner needs, on top of everything
    /// [`SimConfig::validate`] already covers.
    pub fn validate(&self) -> Result<(), SimError> {
        if !(self.duration_s > 0.0) {
            return Err(SimError::NonPositiveDuration(self.duration_s));
        }
        self.to_sim_config().validate()
    }

    /// Project onto the runner's configuration.
    pub fn to_sim_config(&self) -> SimConfig {
        SimConfig {
            waypoints: self.waypoints.clone(),
            takeoff_altitude: self.takeoff_altitude,
            start_position_ned: self.start_position_ned,
            wind_sigma: self.wind_sigma,
            wind_steady_ned: self.wind_steady_ned,
            battery_initial_soc: self.battery_initial_soc,
            gps_position_noise: self.gps_position_noise,
            gps_position_noise_vertical: self.gps_position_noise_vertical,
            gps_velocity_noise: self.gps_velocity_noise,
            cruise_hold_s: self.cruise_hold_s,
            obstacles: self.obstacles.clone(),
            avoidance: self.avoidance,
            seed: self.seed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The exact JSON the browser loads for the hover scenario, trimmed to the
    /// fields that matter. This is the contract test: if `export_live_configs.py`
    /// changes shape, this fails.
    const HOVER_JSON: &str = r#"{
        "id": "hover",
        "label": "Hover",
        "blurb": "Climb to 10 m and hold station for 60 s in calm air.",
        "duration_s": 60.0,
        "takeoff_altitude": 10.0,
        "start_position_ned": [0.0, 0.0, 0.0],
        "waypoints": [],
        "obstacles": [],
        "wind_sigma": [0.0, 0.0, 0.0],
        "wind_steady_ned": [0.0, 0.0, 0.0],
        "seed": 20261012,
        "gps_position_noise": 0.3,
        "gps_position_noise_vertical": 0.5,
        "gps_velocity_noise": 0.05,
        "cruise_hold_s": 2.0,
        "auto_arm": true
    }"#;

    const WAYPOINTS_JSON: &str = r#"{
        "id": "waypoints",
        "label": "Waypoint circuit",
        "blurb": "40 m x 30 m rectangle at 10 m, 5 m/s legs, light turbulence.",
        "duration_s": 120.0,
        "takeoff_altitude": 10.0,
        "start_position_ned": [0.0, 0.0, 0.0],
        "waypoints": [
            {"position": [0.0, 0.0, -10.0], "speed": 5.0, "heading": 0.0, "radius": 2.0},
            {"position": [40.0, 0.0, -10.0], "speed": 5.0, "heading": 0.0, "radius": 2.0}
        ],
        "obstacles": [],
        "wind_sigma": [0.5, 0.5, 0.2],
        "wind_steady_ned": [2.0, 0.0, 0.0],
        "seed": 20261012,
        "gps_position_noise": 0.3,
        "gps_position_noise_vertical": 0.5,
        "gps_velocity_noise": 0.05,
        "cruise_hold_s": 2.0,
        "auto_arm": true
    }"#;

    #[test]
    fn the_hover_config_parses_field_for_field() {
        let cfg = ScenarioConfig::from_json(HOVER_JSON).unwrap();
        assert_eq!(cfg.id, "hover");
        assert_eq!(cfg.label, "Hover");
        assert_eq!(cfg.duration_s, 60.0);
        assert_eq!(cfg.takeoff_altitude, 10.0);
        assert_eq!(cfg.start_position_ned, [0.0, 0.0, 0.0]);
        assert!(cfg.waypoints.is_empty());
        assert_eq!(cfg.seed, 20_261_012);
        assert_eq!(cfg.gps_position_noise, 0.3);
        assert_eq!(cfg.gps_position_noise_vertical, 0.5);
        assert_eq!(cfg.gps_velocity_noise, 0.05);
        assert_eq!(cfg.cruise_hold_s, 2.0);
        assert!(cfg.auto_arm);
    }

    #[test]
    fn the_waypoint_config_parses_its_mission() {
        let cfg = ScenarioConfig::from_json(WAYPOINTS_JSON).unwrap();
        assert_eq!(cfg.waypoints.len(), 2);
        assert_eq!(cfg.waypoints[0].position, [0.0, 0.0, -10.0]);
        assert_eq!(cfg.waypoints[1].position, [40.0, 0.0, -10.0]);
        assert_eq!(cfg.waypoints[0].radius, 2.0);
        assert_eq!(cfg.wind_sigma, [0.5, 0.5, 0.2]);
        assert_eq!(cfg.wind_steady_ned, [2.0, 0.0, 0.0]);
    }

    #[test]
    fn the_projection_onto_the_runner_is_lossless_where_it_matters() {
        let cfg = ScenarioConfig::from_json(WAYPOINTS_JSON).unwrap();
        let sim = cfg.to_sim_config();
        assert_eq!(sim.waypoints.len(), 2);
        assert_eq!(sim.takeoff_altitude, 10.0);
        assert_eq!(sim.seed, cfg.seed);
        assert_eq!(sim.wind_sigma, cfg.wind_sigma);
        assert_eq!(sim.gps_position_noise, cfg.gps_position_noise);
        assert_eq!(sim.cruise_hold_s, cfg.cruise_hold_s);
        // battery_initial_soc was not in the JSON, so the default applies.
        assert_eq!(sim.battery_initial_soc, 1.0);
    }

    #[test]
    fn a_sparse_config_fills_in_the_documented_defaults() {
        // Only the required fields; everything else must default rather than
        // fail to deserialize.
        let json = r#"{"id":"sparse"}"#;
        let cfg = ScenarioConfig::from_json(json).unwrap();
        assert_eq!(cfg.duration_s, 60.0);
        assert_eq!(cfg.takeoff_altitude, 10.0);
        assert_eq!(cfg.gps_position_noise, 0.3);
        assert_eq!(cfg.gps_position_noise_vertical, 0.5);
        assert_eq!(cfg.gps_velocity_noise, 0.05);
        assert_eq!(cfg.cruise_hold_s, 2.0);
        assert_eq!(cfg.seed, 20_261_012);
        assert!(!cfg.auto_arm);
        assert!(cfg.waypoints.is_empty());
        assert!(cfg.obstacles.is_empty());
        // Avoidance is off unless a scenario asks for it.
        assert!(!cfg.avoidance.enabled);
    }

    #[test]
    fn the_avoidance_block_is_optional_and_may_be_partial() {
        // Absent: the documented defaults, disabled.
        let bare = ScenarioConfig::from_json(r#"{"id":"bare"}"#).unwrap();
        assert_eq!(bare.avoidance, crate::avoidance::AvoidanceConfig::default());
        // Only the switch: every other knob keeps its default.
        let on = ScenarioConfig::from_json(r#"{"id":"on","avoidance":{"enabled":true}}"#).unwrap();
        assert!(on.avoidance.enabled);
        assert_eq!(on.avoidance.clearance_margin_m, 2.0);
        assert_eq!(on.avoidance.lookahead_s, 3.0);
        assert_eq!(
            on.avoidance.detour_buffer_m,
            crate::avoidance::DETOUR_BUFFER_M
        );
        // And the projection carries both the zones and the tuning through.
        let sim = on.to_sim_config();
        assert!(sim.avoidance.enabled);
        assert_eq!(sim.avoidance, on.avoidance);
    }

    #[test]
    fn an_unknown_field_is_ignored_rather_than_fatal() {
        // Forward compatibility: a newer exporter adding a key must not break an
        // older bundle.
        let json = r#"{"id":"x","something_new":42}"#;
        assert!(ScenarioConfig::from_json(json).is_ok());
    }

    #[test]
    fn validation_rejects_a_non_positive_duration() {
        let cfg = ScenarioConfig {
            duration_s: 0.0,
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::NonPositiveDuration(_))
        ));
    }

    #[test]
    fn validation_rejects_a_non_positive_takeoff_altitude() {
        let cfg = ScenarioConfig {
            takeoff_altitude: -1.0,
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::NonPositiveTakeoffAltitude(_))
        ));
    }

    #[test]
    fn a_config_round_trips_through_json() {
        let cfg = ScenarioConfig::from_json(WAYPOINTS_JSON).unwrap();
        let text = serde_json::to_string(&cfg).unwrap();
        let back = ScenarioConfig::from_json(&text).unwrap();
        assert_eq!(cfg, back);
    }

    #[test]
    fn the_end_to_end_pipeline_runs_a_parsed_config() {
        use crate::export::flight_data_json;
        use crate::sim::SimRunner;

        let cfg = ScenarioConfig {
            id: "smoke".to_string(),
            duration_s: 20.0,
            waypoints: vec![Waypoint::new([30.0, 0.0, -10.0], 5.0, 0.0, 3.0)],
            auto_arm: true,
            ..Default::default()
        };
        cfg.validate().unwrap();

        let mut runner = SimRunner::new(cfg.to_sim_config()).unwrap();
        if cfg.auto_arm {
            runner.request_event("arm", None);
            runner.request_event("takeoff", None);
        }
        let result = runner.run(cfg.duration_s).unwrap();
        assert_eq!(result.mission_waypoint_count, 1);

        let json = flight_data_json(
            &result,
            &cfg.id,
            cfg.seed,
            cfg.obstacles.clone(),
            "2026-10-18T00:00:00Z".to_string(),
            Some(6),
        )
        .unwrap();
        assert!(json.contains("\"scenario\":\"smoke\""));
        assert!(json.contains("\"schema_version\":\"1.1\""));
    }
}
