//! Talon GNC core.
//!
//! Single source of truth for the Talon autopilot: rigid-body dynamics, the
//! Dryden wind field, the cascaded PD controller, the 15-state MEKF, NLGL
//! guidance, the waypoint mission manager, the flight-phase state machine,
//! sensor and actuator models, and the software-in-the-loop runner. The
//! desktop harness (Python, through PyO3) and the browser (WebAssembly,
//! through wasm-bindgen) both drive this same code.

pub mod battery;
pub mod constants;
pub mod controller;
pub mod dynamics;
pub mod guidance;
pub mod ekf;
pub mod math;
pub mod mission;
pub mod motors;
pub mod rng;
pub mod sensors;
pub mod sim;
pub mod state_machine;
pub mod types;
pub mod wind;
