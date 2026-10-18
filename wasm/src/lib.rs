//! WebAssembly bindings for the Talon GNC core.
//!
//! This crate owns no autopilot logic. It is the seam between the browser and
//! the native `talon-core` runner: the browser hands over a scenario as JSON,
//! steps the runner frame by frame so the page stays responsive, reads a live
//! pose while the vehicle is still flying, and pulls out a finished flight
//! document at the end. Everything it produces comes from the same code the
//! desktop harness and the batch runs use, so flight behaviour is defined once.
//!
//! Why a session object rather than one blocking call: a 120 s mission at the
//! 200 Hz plant rate is 24000 steps. Run to completion it takes a few tens of
//! milliseconds natively, but a browser will still freeze if that happens in one
//! go, and there is nothing to show while it happens. A stepwise session lets the
//! page advance the sim inside a frame budget, paint in between, and turn the
//! finished run into replay data. The same session is what a real-time paced
//! display will drive later without a change to the core.
//!
//! The document and summary builders are plain functions over the core types, so
//! their shape is unit tested natively with `cargo test -p talon-wasm`; only the
//! thin `#[wasm_bindgen]` methods below need a browser.

use serde_json::json;
use talon_core::constants::DT;
use talon_core::export;
use talon_core::scenario::ScenarioConfig;
use talon_core::sim::{SimResult, SimRunner};
use wasm_bindgen::prelude::*;

fn js_err(message: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&message.to_string())
}

/// Build the schema 1.1 flight document for the run so far.
///
/// Safe to call mid-flight: it snapshots the accumulated histories instead of
/// stopping the run, which is what lets the page show a document for a partial
/// flight as well as a finished one.
fn document_json(
    runner: &SimRunner,
    config: &ScenarioConfig,
    generated_utc: &str,
) -> Result<String, serde_json::Error> {
    let result = runner.snapshot(runner.time());
    export::flight_data_json(
        &result,
        &config.id,
        config.seed,
        config.obstacles.clone(),
        generated_utc.to_string(),
        Some(export::DEFAULT_ROUND_DECIMALS),
    )
}

/// Headline run numbers that the flight document does not carry.
///
/// The page derives the estimator error and the obstacle clearance from the
/// document's own arrays; what it cannot derive is the mission outcome and the
/// fix accounting, so those are reported here straight from the runner.
///
/// `duration_s` is the last sample time, matching `meta.duration_s` in the
/// document, and is rounded the same way: the runner's clock accumulates a
/// little float drift over a long run, and nothing should show `119.9999999`.
fn summary_json(result: &SimResult, steps_done: usize, total_steps: usize) -> String {
    let raw_duration = result.time_s.last().copied().unwrap_or(0.0);
    let scale = 10f64.powi(export::DEFAULT_ROUND_DECIMALS as i32);
    let duration_s = (raw_duration * scale).round() / scale;
    json!({
        "duration_s": duration_s,
        "waypoint_count": result.mission_waypoint_count,
        "waypoints_reached": result.mission_waypoints_reached,
        "mission_complete": result.mission_complete,
        "gps_accepted": result.gps_accepted,
        "gps_gated": result.gps_gated,
        "gps_skipped": result.gps_skipped,
        "final_phase": result.final_phase().as_str(),
        "final_truth_altitude_m": result.final_truth_altitude_m(),
        "ekf_converged": result.ekf_converged,
        "ekf_diverged": result.ekf_diverged,
        "steps_done": steps_done,
        "total_steps": total_steps,
    })
    .to_string()
}

/// One steppable mission run.
#[wasm_bindgen]
pub struct Session {
    runner: SimRunner,
    config: ScenarioConfig,
    total_steps: usize,
    steps_done: usize,
    prepared: bool,
}

#[wasm_bindgen]
impl Session {
    /// Create a session from a scenario JSON document.
    ///
    /// The JSON is the same shape served at `public/data/live/configs.json`. If
    /// the scenario asks for `auto_arm`, arm and takeoff are queued now so the
    /// first control tick starts the mission, matching the batch harness.
    #[wasm_bindgen(constructor)]
    pub fn new(config_json: &str) -> Result<Session, JsValue> {
        let config = ScenarioConfig::from_json(config_json).map_err(js_err)?;
        config.validate().map_err(js_err)?;
        let mut runner = SimRunner::new(config.to_sim_config()).map_err(js_err)?;
        if config.auto_arm {
            runner.request_event("arm", None);
            runner.request_event("takeoff", None);
        }
        let total_steps = ((config.duration_s / DT).round() as usize).max(1);
        Ok(Session {
            runner,
            config,
            total_steps,
            steps_done: 0,
            prepared: false,
        })
    }

    /// Convene the estimator at rest, on a negative clock, before the timed run.
    ///
    /// Idempotent, so a caller that prepares twice does not pay twice or shift
    /// the clock. Must be called before the first step for the estimate to be
    /// converged when the mission starts.
    pub fn prepare(&mut self) {
        if !self.prepared {
            self.runner.preroll();
            self.prepared = true;
        }
    }

    /// Advance one plant step. Returns true once the run is finished.
    pub fn step(&mut self) -> bool {
        if self.steps_done < self.total_steps {
            self.runner.step_once(self.steps_done);
            self.steps_done += 1;
        }
        self.steps_done >= self.total_steps
    }

    /// Advance up to `steps` plant steps. Returns how many were advanced.
    ///
    /// This is the frame-loop entry point: the page asks for a chunk of work,
    /// gets the count back, and can measure how long the chunk took.
    pub fn advance(&mut self, steps: usize) -> usize {
        let mut done = 0;
        while done < steps && self.steps_done < self.total_steps {
            self.runner.step_once(self.steps_done);
            self.steps_done += 1;
            done += 1;
        }
        done
    }

    /// Advance to the end of the run. Returns the number of steps advanced.
    pub fn run_to_end(&mut self) -> usize {
        self.advance(usize::MAX)
    }

    /// Fraction of the run completed, in [0, 1].
    #[wasm_bindgen(getter)]
    pub fn progress(&self) -> f64 {
        self.steps_done as f64 / self.total_steps as f64
    }

    /// Whether the run has reached its configured duration.
    #[wasm_bindgen(getter)]
    pub fn finished(&self) -> bool {
        self.steps_done >= self.total_steps
    }

    /// Whether the convergence pre-roll has run.
    #[wasm_bindgen(getter)]
    pub fn prepared(&self) -> bool {
        self.prepared
    }

    /// Plant steps advanced so far.
    #[wasm_bindgen(getter)]
    pub fn steps_done(&self) -> usize {
        self.steps_done
    }

    /// Plant steps in the whole run.
    #[wasm_bindgen(getter)]
    pub fn total_steps(&self) -> usize {
        self.total_steps
    }

    /// Sim time (s). Runs from 0 to `duration_s`; negative during the pre-roll.
    #[wasm_bindgen(getter)]
    pub fn time(&self) -> f64 {
        self.runner.time()
    }

    /// Configured run duration (s).
    #[wasm_bindgen(getter)]
    pub fn duration_s(&self) -> f64 {
        self.config.duration_s
    }

    /// Scenario id, written to `meta.scenario`.
    #[wasm_bindgen(getter)]
    pub fn scenario_id(&self) -> String {
        self.config.id.clone()
    }

    /// Current flight-phase label, lowercase.
    pub fn phase(&self) -> String {
        self.runner.state_machine().phase().as_str().to_string()
    }

    /// Battery state of charge in [0, 1].
    #[wasm_bindgen(getter)]
    pub fn battery_soc(&self) -> f64 {
        self.runner.battery_soc()
    }

    /// Live truth position, NED (m).
    pub fn truth_position(&self) -> Vec<f64> {
        let p = self.runner.truth().position;
        vec![p[0], p[1], p[2]]
    }

    /// Live truth velocity, NED (m/s).
    pub fn truth_velocity(&self) -> Vec<f64> {
        let v = self.runner.truth().velocity;
        vec![v[0], v[1], v[2]]
    }

    /// Live estimated position, NED (m).
    pub fn ekf_position(&self) -> Vec<f64> {
        self.runner.ekf_position().to_vec()
    }

    /// Live altitude above the ground plane (m, positive up).
    ///
    /// Ground is the scenario start height; height is the negated NED down
    /// coordinate, which is the spine the scene already renders on.
    pub fn altitude_m(&self) -> f64 {
        let ground = self.config.start_position_ned[2];
        ground - self.runner.truth().position[2]
    }

    /// Samples recorded so far, at the control rate (100 Hz).
    #[wasm_bindgen(getter)]
    pub fn sample_count(&self) -> usize {
        self.runner.sample_count()
    }

    /// Pull the samples in `[from, sample_count())` as a flat `Float64Array`,
    /// `sample_stride()` doubles per sample.
    ///
    /// This is what a live display consumes: ask for the new tail since the last
    /// read, append it, and never materialize a whole document mid-flight. The
    /// layout is documented on the Rust side (`sim::sample_field`); the field
    /// offsets there and the parser in `src/live/stream.ts` must agree, which a
    /// test on each side pins.
    pub fn samples_from(&self, from: usize) -> Vec<f64> {
        self.runner.samples_from(from)
    }

    /// The phase timeline as JSON: `[[time_s, "phase"], ...]`, oldest first.
    ///
    /// Small enough to re-read on a display tick rather than track increments.
    pub fn phases_json(&self) -> String {
        let entries: Vec<(f64, &str)> = self
            .runner
            .phase_timeline()
            .iter()
            .map(|(t, p)| (*t, p.as_str()))
            .collect();
        serde_json::to_string(&entries).unwrap_or_else(|_| "[]".to_string())
    }

    /// Accepted GPS fixes since the run started.
    #[wasm_bindgen(getter)]
    pub fn gps_accepted(&self) -> u64 {
        self.runner.gps_counts().0
    }

    /// GPS fixes rejected by the innovation gate.
    #[wasm_bindgen(getter)]
    pub fn gps_gated(&self) -> u64 {
        self.runner.gps_counts().1
    }

    /// GPS fixes dropped for an injected outage.
    #[wasm_bindgen(getter)]
    pub fn gps_skipped(&self) -> u64 {
        self.runner.gps_counts().2
    }

    /// Whether the estimator has converged.
    #[wasm_bindgen(getter)]
    pub fn ekf_converged(&self) -> bool {
        self.runner.ekf_flags().0
    }

    /// Whether the estimator has been declared divergent.
    #[wasm_bindgen(getter)]
    pub fn ekf_diverged(&self) -> bool {
        self.runner.ekf_flags().1
    }

    /// The schema 1.1 flight document for the run so far, as JSON.
    ///
    /// `generated_utc` is supplied by the caller because this crate has no clock.
    pub fn document_json(&self, generated_utc: &str) -> Result<String, JsValue> {
        document_json(&self.runner, &self.config, generated_utc).map_err(js_err)
    }

    /// Headline run numbers, as JSON.
    pub fn summary_json(&self) -> String {
        let result = self.runner.snapshot(self.runner.time());
        summary_json(&result, self.steps_done, self.total_steps)
    }
}

/// Run a scenario to completion and return its flight document.
///
/// The one-call form of [`Session`], for tests and for any caller that does not
/// need to step the run itself.
#[wasm_bindgen]
pub fn run_scenario(config_json: &str, generated_utc: &str) -> Result<String, JsValue> {
    let mut session = Session::new(config_json)?;
    session.prepare();
    session.run_to_end();
    session.document_json(generated_utc)
}

/// Core version, for the UI to display alongside the run.
#[wasm_bindgen]
pub fn core_version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// Doubles per streamed sample, matching `talon_core::sim::SAMPLE_STRIDE`.
///
/// Exported so the JavaScript parser can assert its own layout against the
/// build it is actually running, rather than trusting a hard-coded copy.
#[wasm_bindgen]
pub fn sample_stride() -> usize {
    talon_core::sim::SAMPLE_STRIDE
}

#[cfg(test)]
mod tests {
    use super::*;

    fn waypoint_scenario() -> ScenarioConfig {
        ScenarioConfig {
            id: "waypoints".to_string(),
            duration_s: 90.0,
            waypoints: vec![talon_core::types::Waypoint::new(
                [30.0, 20.0, -10.0],
                5.0,
                0.0,
                3.0,
            )],
            auto_arm: true,
            wind_sigma: [0.4, 0.4, 0.2],
            ..Default::default()
        }
    }

    /// Build a runner the same way `Session::new` does, without the JS wrapper.
    fn prepared_runner(config: &ScenarioConfig) -> SimRunner {
        let mut runner = SimRunner::new(config.to_sim_config()).unwrap();
        if config.auto_arm {
            runner.request_event("arm", None);
            runner.request_event("takeoff", None);
        }
        runner.preroll();
        runner
    }

    #[test]
    fn the_document_builder_produces_a_loadable_flight_document() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        let steps = (config.duration_s / DT).round() as usize;
        for i in 0..steps {
            runner.step_once(i);
        }
        let json = document_json(&runner, &config, "2026-10-18T00:00:00Z").unwrap();
        let parsed: talon_core::types::FlightData = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.meta.scenario, "waypoints");
        assert_eq!(parsed.meta.schema_version, "1.1");
        assert_eq!(parsed.meta.sample_count, parsed.time_s.len());
        assert!(parsed.meta.sample_count > 1000, "{}", parsed.meta.sample_count);
        assert_eq!(parsed.waypoints_ned.len(), 1);
    }

    #[test]
    fn a_partial_run_still_yields_a_document() {
        // The page can read out mid-flight, so this must not need a finish.
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        for i in 0..200 {
            runner.step_once(i);
        }
        let json = document_json(&runner, &config, "x").unwrap();
        let parsed: talon_core::types::FlightData = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.meta.sample_count, 100, "1.0 s at the 100 Hz sample rate");
        assert!(parsed.meta.duration_s < 1.0);
    }

    #[test]
    fn the_summary_reports_the_mission_outcome() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        let steps = (config.duration_s / DT).round() as usize;
        for i in 0..steps {
            runner.step_once(i);
        }
        let result = runner.snapshot(runner.time());
        let text = summary_json(&result, steps, steps);
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(value["waypoint_count"], 1);
        assert_eq!(value["mission_complete"], true);
        assert_eq!(value["waypoints_reached"], 1);
        assert_eq!(value["steps_done"], steps);
        assert!(value["gps_accepted"].as_u64().unwrap() > 0);
        // The duration is the last sample time, rounded, not the runner's
        // drifting clock.
        let duration = value["duration_s"].as_f64().unwrap();
        assert!((duration - 89.99).abs() < 0.02, "{duration}");
        assert_eq!(value["ekf_diverged"], false);
        // Flown to the end: the mission lands and the machine settles in idle.
        assert_eq!(value["final_phase"].as_str().unwrap(), "idle");
        // Landed, so the final altitude is back on the ground.
        assert!(value["final_truth_altitude_m"].as_f64().unwrap().abs() < 0.05);
    }

    #[test]
    fn the_summary_is_well_formed_early_in_a_run() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        runner.step_once(0);
        let result = runner.snapshot(runner.time());
        let value: serde_json::Value =
            serde_json::from_str(&summary_json(&result, 1, 4000)).unwrap();
        assert_eq!(value["steps_done"], 1);
        assert_eq!(value["total_steps"], 4000);
        assert_eq!(value["mission_complete"], false);
        assert_eq!(value["waypoints_reached"], 0);
    }

    #[test]
    fn stepping_one_at_a_time_matches_a_bulk_run() {
        // The interactive path and the batch path must agree step for step, or
        // the browser would fly a different flight than the recorded one.
        let config = waypoint_scenario();

        let mut bulk = prepared_runner(&config);
        let steps = (config.duration_s / DT).round() as usize;
        for i in 0..steps {
            bulk.step_once(i);
        }
        let bulk_result = bulk.snapshot(bulk.time());

        let mut piecewise = prepared_runner(&config);
        let mut index = 0;
        while index < steps {
            let chunk = 37.min(steps - index);
            for _ in 0..chunk {
                piecewise.step_once(index);
                index += 1;
            }
        }
        let piecewise_result = piecewise.snapshot(piecewise.time());

        assert_eq!(bulk_result.time_s.len(), piecewise_result.time_s.len());
        for (a, b) in bulk_result
            .truth_position
            .iter()
            .zip(piecewise_result.truth_position.iter())
        {
            assert_eq!(a, b);
        }
        assert_eq!(bulk_result.gps_accepted, piecewise_result.gps_accepted);
        assert_eq!(bulk_result.event_log.len(), piecewise_result.event_log.len());
    }

    #[test]
    fn the_altitude_helper_is_positive_in_the_air_and_zero_on_the_ground() {
        // Mirrors `Session::altitude_m`: ground is NED down = 0, so altitude is
        // the negated down coordinate. Checked here against the runner directly.
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        let ground = -runner.truth().position[2];
        assert!(ground.abs() < 1e-9, "starts on the ground, got {ground}");
        let mut best = 0.0f64;
        for i in 0..3000 {
            runner.step_once(i);
            best = best.max(-runner.truth().position[2]);
        }
        assert!(best > 5.0, "should climb, best altitude {best}");
    }

    #[test]
    fn the_streaming_layout_is_exported_and_consistent() {
        assert_eq!(sample_stride(), talon_core::sim::SAMPLE_STRIDE);
        assert_eq!(sample_stride(), 36);
    }

    #[test]
    fn a_streamed_read_covers_every_recorded_sample() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        let steps = (config.duration_s / DT).round() as usize;
        for i in 0..steps {
            runner.step_once(i);
        }
        let count = runner.sample_count();
        assert!(count > 1000, "{count}");
        let flat = runner.samples_from(0);
        assert_eq!(flat.len(), count * sample_stride());
        // The counter the page polls and the buffer it pulls agree.
        assert!(runner.samples_from(count).is_empty());
    }

    #[test]
    fn the_phase_timeline_serializes_as_ordered_pairs() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        let steps = (config.duration_s / DT).round() as usize;
        for i in 0..steps {
            runner.step_once(i);
        }
        let entries: Vec<(f64, String)> = runner
            .phase_timeline()
            .iter()
            .map(|(t, p)| (*t, p.as_str().to_string()))
            .collect();
        let text = serde_json::to_string(&entries).unwrap();
        let parsed: Vec<(f64, String)> = serde_json::from_str(&text).unwrap();
        assert_eq!(parsed.len(), entries.len());
        assert!(parsed.len() >= 5, "a full mission visits several phases");
        for pair in parsed.windows(2) {
            assert!(pair[0].0 <= pair[1].0);
        }
        assert_eq!(parsed[0].1, "idle");
        assert!(parsed.iter().any(|(_, p)| p == "land"));
    }

    #[test]
    fn the_fix_counters_come_straight_off_the_runner() {
        let config = waypoint_scenario();
        let mut runner = prepared_runner(&config);
        for i in 0..4000 {
            runner.step_once(i);
        }
        let (accepted, gated, skipped) = runner.gps_counts();
        assert!(accepted > 0);
        assert_eq!(gated, 0, "the nominal scenario gates nothing");
        assert_eq!(skipped, 0, "and has no outage");
        let (converged, diverged) = runner.ekf_flags();
        assert!(converged);
        assert!(!diverged);
    }

    #[test]
    fn a_bad_scenario_is_rejected() {
        let parsed = ScenarioConfig::from_json(r#"{"id":"x","duration_s":0.0}"#).unwrap();
        assert!(parsed.validate().is_err());
    }
}
