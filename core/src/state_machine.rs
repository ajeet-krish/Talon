//! Flight-phase state machine.
//!
//! An eight-state guarded transition machine over [`FlightPhase`]. Every state
//! change goes through [`FlightPhaseMachine::request`], which evaluates one
//! event against the transition table plus its guard on the current
//! [`Telemetry`] snapshot. Accepted and rejected attempts are both appended to
//! the event log, sim-time stamped, so a post-flight review can reconstruct
//! exactly when and why the phase changed.
//!
//! Transition table. Guards read the telemetry snapshot; `alt` is altitude
//! above home, positive up:
//!
//! ```text
//! IDLE      -> ARM       on "arm"               if ekf_converged and soc > 0.30
//! ARM       -> TAKEOFF   on "takeoff"           if motors_armed
//! TAKEOFF   -> CRUISE    on "altitude_reached"  if alt > takeoff_alt - 0.5
//! CRUISE    -> WAYPOINT  on "mission_start"     if mission_waypoint_count > 0
//! WAYPOINT  -> RTL       on "mission_complete"  if mission_complete
//! WAYPOINT  -> RTL       on "low_battery"       if soc < 0.20
//! WAYPOINT  -> EMERGENCY on "gps_lost"          if gps_outage_duration > 5.0 s
//! RTL       -> LAND      on "home_reached"      if horizontal distance to home < 1.0 m
//! LAND      -> IDLE      on "landed"            if alt < 0.1 m and |vz| < 0.1 m/s
//! EMERGENCY -> LAND      on "emergency_landing" if emergency_descent_active
//! ANY       -> EMERGENCY on "critical_fault"    if ekf_diverged or motor_fault
//! ```
//!
//! `critical_fault` is checked first in [`FlightPhaseMachine::request`]
//! regardless of the current phase, so EMERGENCY is reachable from every state
//! in one call.
//!
//! Operator-RTL extension, beyond the spec table and documented: the MAVLink
//! SIL maps `MAV_CMD_NAV_RETURN_TO_LAUNCH` to an `"rtl"` event, so TAKEOFF,
//! CRUISE and WAYPOINT also accept `"rtl"` with an always-true guard. An
//! operator command needs no health guard, and the descent logic is identical
//! regardless of what sent the vehicle home.
//!
//! `ekf_converged` and `ekf_diverged` are caller-computed: the state machine
//! does not own the estimator. The SIL defines them from the covariance trace,
//! the accepted-update count and the EKF-versus-truth error.

use std::fmt;

use crate::math::horizontal_distance;
use crate::types::{Arr3, EventLogEntry, FlightPhase};

/// Why the machine could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PhaseError {
    /// The takeoff altitude must be strictly positive.
    NonPositiveTakeoffAltitude(f64),
}

impl fmt::Display for PhaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonPositiveTakeoffAltitude(v) => {
                write!(f, "takeoff_altitude must be positive, got {v}")
            }
        }
    }
}

impl std::error::Error for PhaseError {}

/// Snapshot of everything the guards read, at one instant.
///
/// Positions and velocities are the EKF estimates (the autopilot's view), and
/// home is the recorded launch point in NED metres.
#[derive(Debug, Clone, Copy)]
pub struct Telemetry {
    pub position_ned: Arr3,
    pub velocity_ned: Arr3,
    pub battery_soc: f64,
    pub ekf_converged: bool,
    pub ekf_diverged: bool,
    pub gps_outage_duration: f64,
    pub mission_waypoint_count: usize,
    pub mission_complete: bool,
    pub motors_armed: bool,
    pub home_position_ned: Arr3,
    pub emergency_descent_active: bool,
    pub motor_fault: bool,
}

impl Telemetry {
    /// Altitude above the home point (m, positive up, NED-safe).
    pub fn altitude_above_home(&self) -> f64 {
        self.home_position_ned[2] - self.position_ned[2]
    }

    /// Horizontal (north-east plane) distance to home (m).
    pub fn horizontal_dist_to_home(&self) -> f64 {
        horizontal_distance(self.position_ned, self.home_position_ned)
    }

    /// NED down velocity (m/s, positive descending).
    pub fn vel_z(&self) -> f64 {
        self.velocity_ned[2]
    }
}

impl Default for Telemetry {
    /// A neutral snapshot: at home, motors down, battery full, EKF converged.
    fn default() -> Self {
        Self {
            position_ned: [0.0; 3],
            velocity_ned: [0.0; 3],
            battery_soc: 1.0,
            ekf_converged: true,
            ekf_diverged: false,
            gps_outage_duration: 0.0,
            mission_waypoint_count: 0,
            mission_complete: false,
            motors_armed: false,
            home_position_ned: [0.0; 3],
            emergency_descent_active: false,
            motor_fault: false,
        }
    }
}

/// Guards take the telemetry snapshot and the takeoff altitude.
type GuardFn = fn(&Telemetry, f64) -> bool;

fn g_arm(t: &Telemetry, _alt: f64) -> bool {
    t.ekf_converged && t.battery_soc > 0.30
}

fn g_takeoff(t: &Telemetry, _alt: f64) -> bool {
    t.motors_armed
}

fn g_altitude_reached(t: &Telemetry, takeoff_alt: f64) -> bool {
    t.altitude_above_home() > takeoff_alt - 0.5
}

fn g_mission_start(t: &Telemetry, _alt: f64) -> bool {
    t.mission_waypoint_count > 0
}

fn g_mission_complete(t: &Telemetry, _alt: f64) -> bool {
    t.mission_complete
}

fn g_low_battery(t: &Telemetry, _alt: f64) -> bool {
    t.battery_soc < 0.20
}

fn g_gps_lost(t: &Telemetry, _alt: f64) -> bool {
    t.gps_outage_duration > 5.0
}

fn g_home_reached(t: &Telemetry, _alt: f64) -> bool {
    t.horizontal_dist_to_home() < 1.0
}

fn g_landed(t: &Telemetry, _alt: f64) -> bool {
    t.altitude_above_home() < 0.1 && t.vel_z().abs() < 0.1
}

fn g_emergency_landing(t: &Telemetry, _alt: f64) -> bool {
    t.emergency_descent_active
}

/// Always-true guard for operator commands.
fn g_operator(_t: &Telemetry, _alt: f64) -> bool {
    true
}

/// `(from_phase, event) -> (to_phase, guard)`, or `None` when the event does
/// not apply from that phase.
fn lookup(phase: FlightPhase, event: &str) -> Option<(FlightPhase, GuardFn)> {
    use FlightPhase::*;
    Some(match (phase, event) {
        (Idle, "arm") => (Arm, g_arm),
        (Arm, "takeoff") => (Takeoff, g_takeoff),
        (Takeoff, "altitude_reached") => (Cruise, g_altitude_reached),
        (Cruise, "mission_start") => (Waypoint, g_mission_start),
        (Waypoint, "mission_complete") => (Rtl, g_mission_complete),
        (Waypoint, "low_battery") => (Rtl, g_low_battery),
        (Waypoint, "gps_lost") => (Emergency, g_gps_lost),
        (Rtl, "home_reached") => (Land, g_home_reached),
        (Land, "landed") => (Idle, g_landed),
        (Emergency, "emergency_landing") => (Land, g_emergency_landing),
        // Operator-RTL extension.
        (Takeoff, "rtl") => (Rtl, g_operator),
        (Cruise, "rtl") => (Rtl, g_operator),
        (Waypoint, "rtl") => (Rtl, g_operator),
        _ => return None,
    })
}

/// Guarded eight-state flight phase machine with a transition event log.
#[derive(Debug, Clone)]
pub struct FlightPhaseMachine {
    takeoff_altitude: f64,
    phase: FlightPhase,
    event_log: Vec<EventLogEntry>,
}

impl FlightPhaseMachine {
    /// Build a machine that starts in `IDLE`.
    pub fn new(takeoff_altitude: f64) -> Result<Self, PhaseError> {
        if !(takeoff_altitude > 0.0) {
            return Err(PhaseError::NonPositiveTakeoffAltitude(takeoff_altitude));
        }
        Ok(Self {
            takeoff_altitude,
            phase: FlightPhase::Idle,
            event_log: Vec::new(),
        })
    }

    /// The current flight phase.
    pub fn phase(&self) -> FlightPhase {
        self.phase
    }

    /// All event evaluations, oldest first, accepted and rejected alike.
    pub fn event_log(&self) -> &[EventLogEntry] {
        &self.event_log
    }

    /// The takeoff altitude the `altitude_reached` guard uses (m).
    pub fn takeoff_altitude(&self) -> f64 {
        self.takeoff_altitude
    }

    /// Evaluate one event against the current phase.
    ///
    /// `critical_fault` is checked first, `ANY -> EMERGENCY`, regardless of the
    /// current phase. Otherwise the `(phase, event)` table entry and its guard
    /// decide. On success the phase changes, the transition is logged with
    /// `guard_ok = true`, and `true` is returned. On failure the attempt is
    /// logged with `guard_ok = false` and `to_phase` equal to the current phase,
    /// and `false` is returned. An already-EMERGENCY `critical_fault` is
    /// idempotent: accepted, logged as `critical_fault:already_emergency` so the
    /// audit trail still records the call, but without a phase change.
    pub fn request(&mut self, event: &str, telemetry: &Telemetry, time_s: f64) -> bool {
        if event == "critical_fault" {
            if telemetry.ekf_diverged || telemetry.motor_fault {
                if self.phase == FlightPhase::Emergency {
                    let phase = self.phase;
                    self.log(time_s, phase, phase, "critical_fault:already_emergency", true);
                    return true;
                }
                let from = self.phase;
                self.log(time_s, from, FlightPhase::Emergency, event, true);
                self.phase = FlightPhase::Emergency;
                return true;
            }
            let phase = self.phase;
            self.log(time_s, phase, phase, event, false);
            return false;
        }

        let Some((target, guard)) = lookup(self.phase, event) else {
            // Event not applicable from the current phase.
            let phase = self.phase;
            self.log(time_s, phase, phase, event, false);
            return false;
        };
        if !guard(telemetry, self.takeoff_altitude) {
            let phase = self.phase;
            self.log(time_s, phase, phase, event, false);
            return false;
        }
        let from = self.phase;
        self.log(time_s, from, target, event, true);
        self.phase = target;
        true
    }

    /// Test and scenario hook: set the phase directly, logging the jump.
    ///
    /// The event field is `force:<reason>` so a forced jump is never mistaken
    /// for a guarded transition in post-analysis.
    pub fn force_phase(&mut self, phase: FlightPhase, time_s: f64, reason: &str) {
        if phase == self.phase {
            return;
        }
        let from = self.phase;
        self.log(time_s, from, phase, &format!("force:{reason}"), true);
        self.phase = phase;
    }

    fn log(
        &mut self,
        time_s: f64,
        from_phase: FlightPhase,
        to_phase: FlightPhase,
        event: &str,
        guard_ok: bool,
    ) {
        self.event_log.push(EventLogEntry {
            time: time_s,
            event: event.to_string(),
            from_phase,
            to_phase,
            guard_ok,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use FlightPhase::*;

    fn machine() -> FlightPhaseMachine {
        FlightPhaseMachine::new(10.0).unwrap()
    }

    #[test]
    fn starts_idle_with_an_empty_log() {
        let m = machine();
        assert_eq!(m.phase(), Idle);
        assert!(m.event_log().is_empty());
        assert_eq!(m.takeoff_altitude(), 10.0);
    }

    #[test]
    fn non_positive_takeoff_altitude_is_rejected() {
        assert!(matches!(
            FlightPhaseMachine::new(0.0),
            Err(PhaseError::NonPositiveTakeoffAltitude(_))
        ));
        assert!(FlightPhaseMachine::new(-1.0).is_err());
        assert!(FlightPhaseMachine::new(f64::NAN).is_err());
    }

    #[test]
    fn arm_requires_convergence_and_a_charged_battery() {
        let mut m = machine();
        let mut t = Telemetry {
            battery_soc: 0.5,
            ..Default::default()
        };
        assert!(m.request("arm", &t, 0.0));
        assert_eq!(m.phase(), Arm);

        // A diverged estimate blocks arming.
        let mut m = machine();
        t.ekf_converged = false;
        assert!(!m.request("arm", &t, 0.0));
        assert_eq!(m.phase(), Idle);

        // So does a battery at or below 30 percent.
        let mut m = machine();
        t.ekf_converged = true;
        t.battery_soc = 0.30;
        assert!(!m.request("arm", &t, 0.0), "the guard is a strict inequality");
        t.battery_soc = 0.31;
        assert!(m.request("arm", &t, 0.0));
    }

    #[test]
    fn takeoff_requires_armed_motors() {
        let mut m = machine();
        m.force_phase(Arm, 0.0, "test");
        let t = Telemetry::default();
        assert!(!m.request("takeoff", &t, 1.0));
        let armed = Telemetry {
            motors_armed: true,
            ..Default::default()
        };
        assert!(m.request("takeoff", &armed, 1.0));
        assert_eq!(m.phase(), Takeoff);
    }

    #[test]
    fn altitude_reached_uses_the_half_metre_margin() {
        let mut m = machine();
        m.force_phase(Takeoff, 0.0, "test");
        let below = Telemetry {
            position_ned: [0.0, 0.0, -9.4],
            ..Default::default()
        };
        assert!(!m.request("altitude_reached", &below, 1.0), "9.4 m is short");
        let above = Telemetry {
            position_ned: [0.0, 0.0, -9.6],
            ..Default::default()
        };
        assert!(m.request("altitude_reached", &above, 1.0), "9.6 m clears");
        assert_eq!(m.phase(), Cruise);
    }

    #[test]
    fn mission_start_requires_a_waypoint() {
        let mut m = machine();
        m.force_phase(Cruise, 0.0, "test");
        let empty = Telemetry::default();
        assert!(!m.request("mission_start", &empty, 1.0));
        let with_wp = Telemetry {
            mission_waypoint_count: 3,
            ..Default::default()
        };
        assert!(m.request("mission_start", &with_wp, 1.0));
        assert_eq!(m.phase(), Waypoint);
    }

    #[test]
    fn low_battery_forces_rtl_from_waypoint() {
        let mut m = machine();
        m.force_phase(Waypoint, 0.0, "test");
        let ok = Telemetry {
            battery_soc: 0.5,
            ..Default::default()
        };
        assert!(!m.request("low_battery", &ok, 1.0));
        let low = Telemetry {
            battery_soc: 0.19,
            ..Default::default()
        };
        assert!(m.request("low_battery", &low, 2.0));
        assert_eq!(m.phase(), Rtl);
    }

    #[test]
    fn gps_outage_must_exceed_five_seconds() {
        let mut m = machine();
        m.force_phase(Waypoint, 0.0, "test");
        let five = Telemetry {
            gps_outage_duration: 5.0,
            ..Default::default()
        };
        assert!(!m.request("gps_lost", &five, 1.0));
        let six = Telemetry {
            gps_outage_duration: 5.1,
            ..Default::default()
        };
        assert!(m.request("gps_lost", &six, 2.0));
        assert_eq!(m.phase(), Emergency);
    }

    #[test]
    fn home_reached_and_landed_close_the_loop() {
        let mut m = machine();
        m.force_phase(Rtl, 0.0, "test");
        let far = Telemetry {
            position_ned: [5.0, 0.0, -10.0],
            ..Default::default()
        };
        assert!(!m.request("home_reached", &far, 1.0));
        let close = Telemetry {
            position_ned: [0.5, 0.0, -10.0],
            ..Default::default()
        };
        assert!(m.request("home_reached", &close, 2.0));
        assert_eq!(m.phase(), Land);

        // Still descending: the landed guard needs low altitude and low speed.
        let descending = Telemetry {
            position_ned: [0.0, 0.0, -0.5],
            velocity_ned: [0.0, 0.0, 0.5],
            ..Default::default()
        };
        assert!(!m.request("landed", &descending, 3.0));
        let settled = Telemetry::default();
        assert!(m.request("landed", &settled, 4.0));
        assert_eq!(m.phase(), Idle);
    }

    #[test]
    fn emergency_landing_requires_an_active_descent() {
        let mut m = machine();
        m.force_phase(Emergency, 0.0, "test");
        let idle_desc = Telemetry::default();
        assert!(!m.request("emergency_landing", &idle_desc, 1.0));
        let descending = Telemetry {
            emergency_descent_active: true,
            ..Default::default()
        };
        assert!(m.request("emergency_landing", &descending, 2.0));
        assert_eq!(m.phase(), Land);
    }

    #[test]
    fn critical_fault_reaches_emergency_from_every_phase() {
        for phase in [
            Idle, Arm, Takeoff, Cruise, Waypoint, Rtl, Land, Emergency,
        ] {
            let mut m = machine();
            m.force_phase(phase, 0.0, "test");
            let fault = Telemetry {
                ekf_diverged: true,
                ..Default::default()
            };
            assert!(m.request("critical_fault", &fault, 1.0), "from {phase:?}");
            assert_eq!(m.phase(), Emergency, "from {phase:?}");
        }
    }

    #[test]
    fn a_motor_fault_also_triggers_emergency() {
        let mut m = machine();
        m.force_phase(Cruise, 0.0, "test");
        let fault = Telemetry {
            motor_fault: true,
            ..Default::default()
        };
        assert!(m.request("critical_fault", &fault, 1.0));
        assert_eq!(m.phase(), Emergency);
    }

    #[test]
    fn critical_fault_without_a_fault_is_rejected() {
        let mut m = machine();
        m.force_phase(Cruise, 0.0, "test");
        assert!(!m.request("critical_fault", &Telemetry::default(), 1.0));
        assert_eq!(m.phase(), Cruise);
    }

    #[test]
    fn critical_fault_when_already_emergency_is_idempotent_but_logged() {
        let mut m = machine();
        m.force_phase(Emergency, 0.0, "test");
        let fault = Telemetry {
            ekf_diverged: true,
            ..Default::default()
        };
        assert!(m.request("critical_fault", &fault, 1.0));
        assert_eq!(m.phase(), Emergency);
        let last = m.event_log().last().unwrap();
        assert_eq!(last.event, "critical_fault:already_emergency");
        assert_eq!(last.from_phase, Emergency);
        assert_eq!(last.to_phase, Emergency);
        assert!(last.guard_ok);
    }

    #[test]
    fn operator_rtl_works_from_flight_phases_only() {
        for phase in [Takeoff, Cruise, Waypoint] {
            let mut m = machine();
            m.force_phase(phase, 0.0, "test");
            assert!(
                m.request("rtl", &Telemetry::default(), 1.0),
                "operator RTL should work from {phase:?}"
            );
            assert_eq!(m.phase(), Rtl);
        }
        for phase in [Idle, Arm, Rtl, Land] {
            let mut m = machine();
            m.force_phase(phase, 0.0, "test");
            assert!(
                !m.request("rtl", &Telemetry::default(), 1.0),
                "operator RTL should not apply from {phase:?}"
            );
        }
    }

    #[test]
    fn a_rejected_attempt_is_logged_with_guard_false() {
        let mut m = machine();
        assert!(!m.request("takeoff", &Telemetry::default(), 3.5));
        assert_eq!(m.event_log().len(), 1);
        let e = &m.event_log()[0];
        assert_eq!(e.time, 3.5);
        assert_eq!(e.event, "takeoff");
        assert_eq!(e.from_phase, Idle);
        assert_eq!(e.to_phase, Idle);
        assert!(!e.guard_ok);
    }

    #[test]
    fn an_unknown_event_is_logged_and_rejected() {
        let mut m = machine();
        assert!(!m.request("warp_drive", &Telemetry::default(), 1.0));
        assert_eq!(m.event_log().len(), 1);
        assert_eq!(m.event_log()[0].event, "warp_drive");
        assert!(!m.event_log()[0].guard_ok);
    }

    #[test]
    fn force_phase_logs_the_reason_and_skips_no_ops() {
        let mut m = machine();
        m.force_phase(Cruise, 2.0, "scenario_setup");
        assert_eq!(m.phase(), Cruise);
        assert_eq!(m.event_log().len(), 1);
        assert_eq!(m.event_log()[0].event, "force:scenario_setup");
        assert_eq!(m.event_log()[0].from_phase, Idle);
        assert_eq!(m.event_log()[0].to_phase, Cruise);
        // Forcing the current phase is a no-op and logs nothing.
        m.force_phase(Cruise, 3.0, "again");
        assert_eq!(m.event_log().len(), 1);
    }

    #[test]
    fn the_log_is_ordered_oldest_first() {
        let mut m = machine();
        let armed = Telemetry {
            battery_soc: 0.9,
            motors_armed: true,
            ..Default::default()
        };
        m.request("arm", &armed, 0.0);
        m.request("takeoff", &armed, 1.0);
        m.request("arm", &armed, 2.0); // not applicable from TAKEOFF
        let times: Vec<f64> = m.event_log().iter().map(|e| e.time).collect();
        assert_eq!(times, vec![0.0, 1.0, 2.0]);
        assert!(m.event_log()[0].guard_ok);
        assert!(m.event_log()[1].guard_ok);
        assert!(!m.event_log()[2].guard_ok);
    }

    #[test]
    fn a_full_nominal_flight_progresses_through_every_phase() {
        let mut m = machine();
        let armed = Telemetry {
            battery_soc: 0.9,
            motors_armed: true,
            ..Default::default()
        };
        assert!(m.request("arm", &armed, 0.0));
        assert!(m.request("takeoff", &armed, 1.0));
        let at_altitude = Telemetry {
            position_ned: [0.0, 0.0, -10.0],
            motors_armed: true,
            ..Default::default()
        };
        assert!(m.request("altitude_reached", &at_altitude, 12.0));
        let mission = Telemetry {
            position_ned: [0.0, 0.0, -10.0],
            mission_waypoint_count: 4,
            ..Default::default()
        };
        assert!(m.request("mission_start", &mission, 13.0));
        let done = Telemetry {
            position_ned: [30.0, 40.0, -10.0],
            mission_complete: true,
            ..Default::default()
        };
        assert!(m.request("mission_complete", &done, 60.0));
        let home = Telemetry {
            position_ned: [0.0, 0.0, -10.0],
            ..Default::default()
        };
        assert!(m.request("home_reached", &home, 70.0));
        assert!(m.request("landed", &Telemetry::default(), 75.0));
        assert_eq!(m.phase(), Idle);
        // Every accepted entry moved the machine and the sequence is complete.
        assert_eq!(m.event_log().len(), 7);
        assert!(m.event_log().iter().all(|e| e.guard_ok));
    }

    #[test]
    fn telemetry_derived_quantities_are_ned_safe() {
        let t = Telemetry {
            position_ned: [30.0, 40.0, -12.0],
            velocity_ned: [0.0, 0.0, 1.5],
            home_position_ned: [0.0, 0.0, -2.0],
            ..Default::default()
        };
        assert!((t.altitude_above_home() - 10.0).abs() < 1e-12, "up is positive");
        assert!((t.horizontal_dist_to_home() - 50.0).abs() < 1e-12);
        assert!((t.vel_z() - 1.5).abs() < 1e-12, "positive is descending");
    }
}
