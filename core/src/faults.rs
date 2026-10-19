//! Authored, scheduled faults.
//!
//! A scenario places faults at absolute sim times; [`crate::sim::SimRunner`]
//! turns each [`FaultSpec`] into a one-shot scheduled action, so the fault
//! applies exactly once when the clock reaches it, before the step's sensors
//! and events. This module owns the data model and the validation rules; the
//! application onto the injection machinery lives in
//! [`crate::sim::SimRunner::apply_fault`].
//!
//! The JSON form is internally tagged on `kind`, so an authored fault reads
//! `{"kind": "gps_outage", "at_s": 2.0, "duration_s": 3.0}` and the ts-rs
//! mirror comes out as a discriminated union (D37), which the mission editor
//! can round-trip without a hand-written adapter.
//!
//! Validation is finiteness first (JSON `1e999` parses to `inf`, and NaN
//! defeats every comparison), then ranges, then the count cap. Nothing is
//! clamped; a bad fault is rejected with the offending field named.

use serde::{Deserialize, Serialize};

use crate::constants::{DT, MAX_DURATION_S};
use crate::sim::{check_range, DivergenceMode, SimError};

/// Most authored faults one scenario may schedule; defined with the other
/// input-validation caps in [`crate::constants`] so it rides along with
/// [`crate::constants::Limits`] into the generated TypeScript mirror, and
/// re-exported here so the fault call sites keep one import.
pub use crate::constants::MAX_FAULTS;

/// The sim time of the last step of a run of `duration_s`, the latest clock
/// at which a fault still has a step to act in, or `None` when the run is
/// too short to have one.
///
/// Exact rather than approximate, because it mirrors the runner: the step
/// count is `(duration_s / DT).round()` (the same expression
/// [`crate::sim::SimRunner::run`] uses), step `i` starts at its own clock
/// `i * DT`, and [`crate::sim::SimRunner`] fires schedules on
/// `time >= at_s`, so the last step's clock `(steps - 1) * DT` is itself a
/// valid fire time. Fewer than two steps means no window: with one step the
/// only clock is `0`, and the pre-roll convention keeps the main loop's
/// clock starting there.
fn last_step_time(duration_s: f64) -> Option<f64> {
    let steps = (duration_s / DT).round();
    if steps < 2.0 {
        return None;
    }
    Some((steps - 1.0) * DT)
}

/// The fault families a scenario can author.
///
/// Serialized `snake_case`, so the JSON tag values are `gps_outage`,
/// `ekf_divergence`, `battery_soc`, `motor_fault`, `emergency` and `rtl`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub enum FaultKind {
    /// Black out GPS so the filter dead-reckons on IMU only.
    GpsOutage,
    /// Corrupt the estimator the way the FDIR scenarios do.
    EkfDivergence,
    /// Force the battery state of charge.
    BatterySoc,
    /// Raise the motor-fault flag, which trips `critical_fault`.
    MotorFault,
    /// Force the phase machine straight to EMERGENCY.
    Emergency,
    /// Command operator RTL.
    Rtl,
}

/// One authored fault: a family, the sim time it fires at, and the family's
/// own parameter where it has one.
///
/// Internally tagged: serde writes and reads `kind` plus the variant's fields,
/// and ts-rs mirrors that shape as a discriminated union, so the authored JSON,
/// the Rust core and the generated TypeScript agree by construction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts-export", derive(ts_rs::TS))]
pub enum FaultSpec {
    /// Black out GPS from `at_s` for `duration_s` sim seconds.
    GpsOutage {
        /// Sim time the outage starts (s): finite, at or before the run's last step clock.
        at_s: f64,
        /// Outage length (s): finite and strictly positive.
        duration_s: f64,
    },
    /// Corrupt the estimator at `at_s`.
    EkfDivergence {
        /// Sim time of the corruption (s): finite, at or before the run's last step clock.
        at_s: f64,
        /// Which corruption to apply.
        mode: DivergenceMode,
    },
    /// Force the battery state of charge at `at_s`.
    BatterySoc {
        /// Sim time of the force (s): finite, at or before the run's last step clock.
        at_s: f64,
        /// Forced state of charge: finite, in `[0, 1]`.
        soc: f64,
    },
    /// Raise the motor-fault flag at `at_s`; the critical-fault rising edge
    /// in the event pass takes it to EMERGENCY.
    ///
    /// The flag is sticky for the runner's lifetime and the response is one
    /// per flag lifetime: a second MotorFault, or one after the vehicle has
    /// landed, is inert.
    MotorFault {
        /// Sim time the flag is raised (s): finite, at or before the run's last step clock.
        at_s: f64,
    },
    /// Force the phase machine to EMERGENCY at `at_s`.
    Emergency {
        /// Sim time of the forced jump (s): finite, at or before the run's last step clock.
        at_s: f64,
    },
    /// Command operator RTL at `at_s`.
    Rtl {
        /// Sim time of the command (s): finite, at or before the run's last step clock.
        at_s: f64,
    },
}

impl FaultSpec {
    /// The sim time this fault fires at (s).
    pub fn at_s(&self) -> f64 {
        match self {
            Self::GpsOutage { at_s, .. }
            | Self::EkfDivergence { at_s, .. }
            | Self::BatterySoc { at_s, .. }
            | Self::MotorFault { at_s }
            | Self::Emergency { at_s }
            | Self::Rtl { at_s } => *at_s,
        }
    }

    /// The fault family this spec belongs to.
    pub fn kind(&self) -> FaultKind {
        match self {
            Self::GpsOutage { .. } => FaultKind::GpsOutage,
            Self::EkfDivergence { .. } => FaultKind::EkfDivergence,
            Self::BatterySoc { .. } => FaultKind::BatterySoc,
            Self::MotorFault { .. } => FaultKind::MotorFault,
            Self::Emergency { .. } => FaultKind::Emergency,
            Self::Rtl { .. } => FaultKind::Rtl,
        }
    }

    /// Validate this fault against the run duration.
    ///
    /// Finiteness first, then the fire window, then the family's own
    /// parameter. The window is exact: a run of `duration_s` takes
    /// `steps = (duration_s / DT).round()` steps, step `i` starts at its own
    /// clock `i * DT`, and `fire_scheduled` fires on `time >= at_s`, so a
    /// fault fires exactly when `0.0 <= at_s <= (steps - 1) * DT`. The top is
    /// inclusive: the last step's own clock still fires, on the last step. A
    /// run that rounds to fewer than two steps has no window at all and
    /// rejects every fault with [`SimError::NoFaultWindow`]. A non-finite
    /// window is rejected too, because every comparison against NaN is false
    /// and would wave any finite `at_s` through.
    pub fn validate(&self, duration_s: f64) -> Result<(), SimError> {
        if !duration_s.is_finite() {
            return Err(SimError::NonFiniteParameter {
                name: "duration_s",
                value: duration_s,
            });
        }
        let Some(window_end) = last_step_time(duration_s) else {
            return Err(SimError::NoFaultWindow(duration_s));
        };
        let at_s = self.at_s();
        if !at_s.is_finite() {
            return Err(SimError::NonFiniteParameter {
                name: "fault at_s",
                value: at_s,
            });
        }
        if at_s < 0.0 || at_s > window_end {
            return Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                value: at_s,
                min: 0.0,
                max: window_end,
            });
        }
        self.validate_fields()
    }

    /// The checks that need no run duration: `at_s` finite and inside the
    /// fire window of the longest accepted run, plus the family's parameter
    /// range.
    ///
    /// [`crate::sim::SimConfig`] validates through this because it carries no
    /// duration; [`crate::scenario::ScenarioConfig`] applies the window of
    /// the scenario's own duration in [`FaultSpec::validate`]. The bare
    /// window is the widest one with the same exact rule, so the two paths
    /// agree for a run of [`MAX_DURATION_S`]: no accepted run can last
    /// longer, so a fault outside this window could never fire.
    pub(crate) fn validate_fields(&self) -> Result<(), SimError> {
        // The longest accepted run spans far more than two plant steps, so
        // its window always exists.
        let window_end =
            last_step_time(MAX_DURATION_S).expect("MAX_DURATION_S spans more than one plant step");
        check_range("fault at_s", self.at_s(), 0.0, window_end)?;
        match self {
            Self::GpsOutage { duration_s, .. } => {
                // Finiteness before positivity: `inf` would black out GPS
                // forever, and NaN would corrupt the outage window arithmetic.
                if !duration_s.is_finite() {
                    return Err(SimError::NonFiniteParameter {
                        name: "fault duration_s",
                        value: *duration_s,
                    });
                }
                if *duration_s <= 0.0 {
                    return Err(SimError::NonPositiveGpsOutage(*duration_s));
                }
                // Capped like every other magnitude: an outage longer than
                // the longest accepted run could never end inside a run.
                if *duration_s > MAX_DURATION_S {
                    return Err(SimError::OutOfRangeParameter {
                        name: "fault duration_s",
                        value: *duration_s,
                        min: 0.0,
                        max: MAX_DURATION_S,
                    });
                }
                Ok(())
            }
            Self::BatterySoc { soc, .. } => check_range("fault soc", *soc, 0.0, 1.0),
            Self::EkfDivergence { .. }
            | Self::MotorFault { .. }
            | Self::Emergency { .. }
            | Self::Rtl { .. } => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimConfig;

    /// A representative run length for the window checks.
    const DURATION: f64 = 60.0;

    #[test]
    fn at_s_must_be_finite_and_inside_the_run_window() {
        // Grid duration: 60 s is 12000 steps, so the last step's clock is
        // 11999 * DT = 59.995, and the window's top is inclusive: a fault
        // exactly there still fires, on the last step.
        assert!(FaultSpec::Emergency { at_s: 0.0 }
            .validate(DURATION)
            .is_ok());
        assert!(
            FaultSpec::Emergency { at_s: 59.995 }
                .validate(DURATION)
                .is_ok(),
            "the last step's own clock is inside the window"
        );
        // One clock past the last step has no step to fire in.
        assert!(matches!(
            FaultSpec::Emergency { at_s: 59.996 }.validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));
        // duration_s - 0.001 also lands past the last step of a grid run.
        assert!(matches!(
            FaultSpec::Emergency {
                at_s: DURATION - 0.001
            }
            .validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));

        // A non-grid duration whose quotient rounds DOWN: 60.002 / DT is
        // 12000.4, which rounds to 12000 steps, so the window still ends at
        // the clock 59.995 rather than anywhere near 60.002.
        const NON_GRID: f64 = 60.002;
        assert!(
            FaultSpec::Emergency { at_s: 59.995 }
                .validate(NON_GRID)
                .is_ok(),
            "the last step of a round-down run is inside the window"
        );
        assert!(matches!(
            FaultSpec::Emergency { at_s: 59.996 }.validate(NON_GRID),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));
        assert!(matches!(
            FaultSpec::Emergency {
                at_s: NON_GRID - 0.001
            }
            .validate(NON_GRID),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));

        // The reported window top is the last step clock itself.
        assert!(matches!(
            FaultSpec::Emergency { at_s: DURATION }.validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                value,
                min: 0.0,
                max,
            }) if value == DURATION && max == last_step_time(DURATION).unwrap()
        ));
        assert!(matches!(
            FaultSpec::Emergency {
                at_s: DURATION + 1.0
            }
            .validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));

        // A negative fire time would land in the pre-roll, where the clock is
        // negative and schedules never fire; rejected instead.
        assert!(matches!(
            FaultSpec::Rtl { at_s: -1.0 }.validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                value,
                min: 0.0,
                max,
                ..
            }) if value == -1.0 && max == last_step_time(DURATION).unwrap()
        ));

        // Finiteness first: NaN and infinities never reach a range test,
        // because every comparison against them is false.
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                matches!(
                    FaultSpec::Emergency { at_s: bad }.validate(DURATION),
                    Err(SimError::NonFiniteParameter {
                        name: "fault at_s",
                        ..
                    })
                ),
                "at_s {bad} should be rejected as non-finite"
            );
        }

        // A NaN window would wave any finite fire time through, so it is
        // rejected before the window is applied.
        assert!(matches!(
            FaultSpec::Emergency { at_s: 1.0 }.validate(f64::NAN),
            Err(SimError::NonFiniteParameter {
                name: "duration_s",
                ..
            })
        ));
    }

    #[test]
    fn a_run_of_one_step_or_less_has_no_fault_window() {
        // A run that rounds to fewer than two steps has no last-step clock
        // a fault could fire at, so every fault is rejected with the window
        // named rather than silently never firing. One full step plus a bit
        // (1.4 * DT rounds to one step) is included on purpose: the rule is
        // the step count, not the raw duration.
        for duration in [0.001, DT / 2.0, DT, 1.4 * DT] {
            assert!(
                matches!(
                    FaultSpec::Emergency { at_s: 0.0 }.validate(duration),
                    Err(SimError::NoFaultWindow(v)) if v == duration
                ),
                "duration {duration} should leave no fault window"
            );
        }
        // At two steps the window reopens: the only clock is the second
        // step's, inclusive on both ends of its tiny span.
        FaultSpec::Emergency { at_s: 0.0 }
            .validate(1.5 * DT)
            .expect("a two-step run has a window");
        FaultSpec::Emergency { at_s: DT }
            .validate(1.5 * DT)
            .expect("the last step's clock is inside a two-step window");
        // The message names the duration and the step requirement.
        assert_eq!(
            FaultSpec::Emergency { at_s: 0.0 }
                .validate(DT)
                .unwrap_err()
                .to_string(),
            "duration_s 0.005 leaves no fault window: a run must span at least two plant steps (0.005 s each) for a fault to fire"
        );
    }

    #[test]
    fn a_gps_outage_duration_is_finite_positive_and_capped() {
        assert!(FaultSpec::GpsOutage {
            at_s: 1.0,
            duration_s: 0.5
        }
        .validate(DURATION)
        .is_ok());

        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(
                matches!(
                    FaultSpec::GpsOutage {
                        at_s: 1.0,
                        duration_s: bad
                    }
                    .validate(DURATION),
                    Err(SimError::NonFiniteParameter {
                        name: "fault duration_s",
                        ..
                    })
                ),
                "duration {bad} should be rejected as non-finite"
            );
        }

        // Zero and negative outages reuse the runner's own rejection, the
        // same one `inject_gps_outage` raises.
        for bad in [0.0, -1.0] {
            assert!(
                matches!(
                    FaultSpec::GpsOutage {
                        at_s: 1.0,
                        duration_s: bad
                    }
                    .validate(DURATION),
                    Err(SimError::NonPositiveGpsOutage(v)) if v == bad
                ),
                "duration {bad} should be rejected as non-positive"
            );
        }

        // Capped like every other magnitude: an outage cannot outlast the
        // longest accepted run, and the cap itself is accepted.
        assert!(FaultSpec::GpsOutage {
            at_s: 1.0,
            duration_s: MAX_DURATION_S
        }
        .validate(DURATION)
        .is_ok());
        assert!(matches!(
            FaultSpec::GpsOutage {
                at_s: 1.0,
                duration_s: MAX_DURATION_S + 1.0
            }
            .validate(DURATION),
            Err(SimError::OutOfRangeParameter {
                name: "fault duration_s",
                value,
                min: 0.0,
                max,
            }) if value == MAX_DURATION_S + 1.0 && max == MAX_DURATION_S
        ));
    }

    #[test]
    fn an_authored_soc_must_be_finite_and_inside_the_pack() {
        for good in [0.0, 0.1, 1.0] {
            assert!(
                FaultSpec::BatterySoc {
                    at_s: 5.0,
                    soc: good
                }
                .validate(DURATION)
                .is_ok(),
                "soc {good} is in range"
            );
        }
        for bad in [f64::NAN, f64::INFINITY] {
            assert!(
                matches!(
                    FaultSpec::BatterySoc {
                        at_s: 5.0,
                        soc: bad
                    }
                    .validate(DURATION),
                    Err(SimError::NonFiniteParameter {
                        name: "fault soc",
                        ..
                    })
                ),
                "soc {bad} should be rejected as non-finite"
            );
        }
        for bad in [-0.01, 1.01] {
            assert!(
                matches!(
                    FaultSpec::BatterySoc {
                        at_s: 5.0,
                        soc: bad
                    }
                    .validate(DURATION),
                    Err(SimError::OutOfRangeParameter {
                        name: "fault soc",
                        min,
                        max,
                        ..
                    }) if min == 0.0 && max == 1.0
                ),
                "soc {bad} should be out of range"
            );
        }
    }

    #[test]
    fn the_fault_list_is_capped_at_sixteen() {
        let one = FaultSpec::Emergency { at_s: 1.0 };
        let at_cap = SimConfig {
            faults: vec![one.clone(); MAX_FAULTS],
            ..Default::default()
        };
        at_cap
            .validate()
            .expect("the cap itself is accepted, count checked by the caller");

        let over_cap = SimConfig {
            faults: vec![one; MAX_FAULTS + 1],
            ..Default::default()
        };
        assert!(matches!(
            over_cap.validate(),
            Err(SimError::TooManyItems {
                name: "faults",
                count,
                max,
            }) if count == 17 && max == MAX_FAULTS
        ));
    }

    #[test]
    fn the_runner_level_checks_need_no_run_duration() {
        // A negative fire time is rejected outright.
        let cfg = SimConfig {
            faults: vec![FaultSpec::Rtl { at_s: -1.0 }],
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                ..
            })
        ));
        // So is a state of charge outside the pack.
        let cfg = SimConfig {
            faults: vec![FaultSpec::BatterySoc {
                at_s: 5.0,
                soc: 2.0,
            }],
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::OutOfRangeParameter {
                name: "fault soc",
                ..
            })
        ));
        // SimConfig carries no run duration, so the scenario's own window is
        // not applied here: a far-future fault is still finite and
        // non-negative, and the scenario's validate applies the window.
        SimConfig {
            faults: vec![FaultSpec::Emergency { at_s: 500.0 }],
            ..Default::default()
        }
        .validate()
        .expect("SimConfig has no duration to bound against");
        // The bare window is the longest run's exact window, inclusive: 900 s
        // is 180000 steps, so 179999 * DT = 899.995 is the last accepted fire
        // time, and it agrees with a scenario of MAX_DURATION_S step for
        // step.
        let last_step = last_step_time(MAX_DURATION_S).unwrap();
        assert_eq!(last_step, 899.995);
        SimConfig {
            faults: vec![FaultSpec::Emergency { at_s: last_step }],
            ..Default::default()
        }
        .validate()
        .expect("the longest run's last step is inside the bare window");
        // One clock past it could never fire in any accepted run.
        let cfg = SimConfig {
            faults: vec![FaultSpec::Emergency {
                at_s: last_step + DT,
            }],
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                value,
                min: 0.0,
                max,
            }) if value == last_step + DT && max == last_step
        ));
        let cfg = SimConfig {
            faults: vec![FaultSpec::Emergency {
                at_s: MAX_DURATION_S + 1.0,
            }],
            ..Default::default()
        };
        assert!(matches!(
            cfg.validate(),
            Err(SimError::OutOfRangeParameter {
                name: "fault at_s",
                max,
                ..
            }) if max == last_step
        ));
        // Well clear of the cap stays accepted.
        SimConfig {
            faults: vec![FaultSpec::Emergency {
                at_s: MAX_DURATION_S - 2.0 * DT,
            }],
            ..Default::default()
        }
        .validate()
        .expect("two steps before the absolute cap is inside every window");
    }

    #[test]
    fn faults_parse_from_the_internally_tagged_json_shape() {
        let spec: FaultSpec =
            serde_json::from_str(r#"{"kind":"gps_outage","at_s":2.0,"duration_s":3.0}"#).unwrap();
        assert_eq!(
            spec,
            FaultSpec::GpsOutage {
                at_s: 2.0,
                duration_s: 3.0
            }
        );
        assert_eq!(spec.at_s(), 2.0);
        assert_eq!(spec.kind(), FaultKind::GpsOutage);

        let spec: FaultSpec =
            serde_json::from_str(r#"{"kind":"ekf_divergence","at_s":6.0,"mode":"corrupt_bias"}"#)
                .unwrap();
        assert_eq!(
            spec,
            FaultSpec::EkfDivergence {
                at_s: 6.0,
                mode: DivergenceMode::CorruptBias
            }
        );
        assert_eq!(spec.kind(), FaultKind::EkfDivergence);

        // snake_case on the wire: the tags and the mode names.
        assert_eq!(
            serde_json::to_string(&FaultKind::MotorFault).unwrap(),
            "\"motor_fault\""
        );
        assert_eq!(
            serde_json::to_string(&DivergenceMode::SuppressUpdates).unwrap(),
            "\"suppress_updates\""
        );
        let text = serde_json::to_string(&FaultSpec::Rtl { at_s: 4.0 }).unwrap();
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(value["kind"], "rtl");
        assert_eq!(value["at_s"], 4.0);

        // A round trip keeps every variant equal to itself.
        for spec in [
            FaultSpec::GpsOutage {
                at_s: 1.0,
                duration_s: 2.0,
            },
            FaultSpec::BatterySoc {
                at_s: 3.0,
                soc: 0.25,
            },
            FaultSpec::MotorFault { at_s: 4.0 },
            FaultSpec::Emergency { at_s: 5.0 },
        ] {
            let text = serde_json::to_string(&spec).unwrap();
            let back: FaultSpec = serde_json::from_str(&text).unwrap();
            assert_eq!(back, spec, "{text}");
        }
    }
}
