#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "talon/constants.hpp"
#include "talon/controller.hpp"
#include "talon/dynamics.hpp"
#include "talon/ekf.hpp"
#include "talon/types.hpp"
#include "talon/wind.hpp"

namespace py = pybind11;

using talon::CascadedController;
using talon::ControlCommand;
using talon::ControlGains;
using talon::DrydenWind;
using talon::EKFConfig;
using talon::EKFNominalState;
using talon::Ekf;
using talon::Quaternion;
using talon::State;
using talon::VehicleParams;
using talon::Vec3;

PYBIND11_MODULE(_core, m) {
    m.doc() =
        "Talon C++ computational kernel: rigid-body dynamics, wind, cascaded "
        "PD autopilot, MEKF state estimator, constants.";

    // Constants (single source of truth: cpp/include/talon/constants.hpp).
    m.attr("MASS") = talon::MASS;
    m.attr("GRAVITY") = talon::GRAVITY;
    m.attr("ARM_LENGTH") = talon::ARM_LENGTH;
    m.attr("IXX") = talon::IXX;
    m.attr("IYY") = talon::IYY;
    m.attr("IZZ") = talon::IZZ;
    m.attr("MOTOR_KF") = talon::MOTOR_KF;
    m.attr("MOTOR_KM") = talon::MOTOR_KM;
    m.attr("MOTOR_TIME_CONST") = talon::MOTOR_TIME_CONST;
    m.attr("DRAG_COEFF") = talon::DRAG_COEFF;
    m.attr("AIR_DENSITY") = talon::AIR_DENSITY;
    m.attr("VEHICLE_AREA") = talon::VEHICLE_AREA;
    m.attr("DT") = talon::DT;
    m.attr("DT_CONTROL") = talon::DT_CONTROL;
    m.attr("DT_GPS") = talon::DT_GPS;
    m.attr("DT_IMU") = talon::DT_IMU;
    m.attr("M_MAX") = talon::M_MAX;

    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<double, double, double, double>(),
             py::arg("w") = 1.0, py::arg("x") = 0.0,
             py::arg("y") = 0.0, py::arg("z") = 0.0)
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def("norm", &Quaternion::norm)
        .def("normalized", &Quaternion::normalized)
        .def("conjugate", &Quaternion::conjugate)
        .def("rotate_vector", &Quaternion::rotate_vector,
             "Rotate a vector from body frame to NED frame.")
        .def("to_dcm", &Quaternion::to_dcm,
             "Body-to-NED direction cosine matrix (3x3).")
        .def("__repr__", [](const Quaternion& q) {
            return "Quaternion(w=" + std::to_string(q.w) +
                   ", x=" + std::to_string(q.x) +
                   ", y=" + std::to_string(q.y) +
                   ", z=" + std::to_string(q.z) + ")";
        });

    py::class_<State>(m, "State")
        .def(py::init<>())
        .def_readwrite("position", &State::position, "NED position [n, e, d] (m)")
        .def_readwrite("velocity", &State::velocity, "NED velocity (m/s)")
        .def_readwrite("angular_rate", &State::angular_rate, "Body angular rate (rad/s)")
        .def_readwrite("orientation", &State::orientation, "Body-to-NED quaternion")
        .def("__repr__", [](const State& s) {
            return "State(position=[" + std::to_string(s.position.x()) + ", " +
                   std::to_string(s.position.y()) + ", " + std::to_string(s.position.z()) + "])";
        });

    py::class_<VehicleParams>(m, "VehicleParams")
        .def(py::init<>())
        .def_readwrite("mass", &VehicleParams::mass)
        .def_readwrite("inertia_diag", &VehicleParams::inertia_diag)
        .def_readwrite("arm_length", &VehicleParams::arm_length)
        .def_readwrite("drag_coeff", &VehicleParams::drag_coeff)
        .def_readwrite("gravity", &VehicleParams::gravity)
        .def_readwrite("air_density", &VehicleParams::air_density)
        .def_readwrite("vehicle_area", &VehicleParams::vehicle_area);

    m.def(
        "propagate",
        [](const State& state, double thrust, const Vec3& moment,
           const Vec3& wind_ned, double dt, const VehicleParams& params) {
            return talon::propagate(state, thrust, moment, wind_ned, dt, params);
        },
        py::arg("state"), py::arg("thrust"), py::arg("moment"),
        py::arg("wind"), py::arg("dt"),
        py::arg("params") = VehicleParams{},
        "One RK4 step of the 13-state rigid-body model. Thrust acts along "
        "-body-z; wind is the NED wind velocity; quaternion renormalized.");

    m.def("rotation_matrix", &talon::rotation_matrix, py::arg("q"),
          "Body-to-NED DCM from a Hamilton quaternion.");

    m.def("hover_thrust",
          [](const VehicleParams& params) { return talon::hover_thrust(params); },
          py::arg("params") = VehicleParams{},
          "Collective thrust (N) required to hover.");

    py::class_<DrydenWind>(m, "DrydenWind")
        .def(py::init<double, double, double, double, double, double, double,
                      unsigned int>(),
             py::arg("sigma_u") = 2.0, py::arg("sigma_v") = 2.0,
             py::arg("sigma_w") = 1.0, py::arg("L_u") = 200.0,
             py::arg("L_v") = 200.0, py::arg("L_w") = 50.0,
             py::arg("dt") = talon::DT, py::arg("seed") = 42u)
        .def("step", &DrydenWind::step, py::arg("airspeed"),
             "Advance one step; returns current wind [u, v, w] in NED (m/s).")
        .def("reset", &DrydenWind::reset,
             "Re-seed the generator and zero the wind state.")
        .def("wind", &DrydenWind::wind, "Current wind state in NED (m/s).");

    py::class_<ControlGains>(m, "ControlGains")
        .def(py::init<>())
        .def_readwrite("kp_pos", &ControlGains::kp_pos,
                       "Position P gains ((m/s^2) per m)")
        .def_readwrite("kd_pos", &ControlGains::kd_pos,
                       "Velocity D gains ((m/s^2) per m/s)")
        .def_readwrite("kp_att", &ControlGains::kp_att,
                       "Attitude P gains ((N*m) per rad)")
        .def_readwrite("kd_att", &ControlGains::kd_att,
                       "Rate D gains ((N*m) per rad/s)")
        .def_static("defaults", &ControlGains::defaults,
                    "Analytically derived Crazyflie-class starting gains "
                    "documented in cpp/include/talon/controller.hpp.")
        .def("__repr__", [](const ControlGains& g) {
            return "ControlGains(kp_pos=[" + std::to_string(g.kp_pos.x()) +
                   ", " + std::to_string(g.kp_pos.y()) + ", " +
                   std::to_string(g.kp_pos.z()) + "])";
        });

    py::class_<ControlCommand>(m, "ControlCommand")
        .def(py::init<>())
        .def_readwrite("collective_thrust", &ControlCommand::collective_thrust,
                       "Collective thrust (N, along -body-z)")
        .def_readwrite("moment", &ControlCommand::moment,
                       "Body-frame moment (N*m)")
        .def("__repr__", [](const ControlCommand& c) {
            return "ControlCommand(collective_thrust=" +
                   std::to_string(c.collective_thrust) + ")";
        });

    py::class_<CascadedController>(m, "CascadedController")
        .def(py::init<const ControlGains&, const VehicleParams&>(),
             py::arg("gains"), py::arg("params") = VehicleParams{},
             "Cascaded PD autopilot (position outer loop, attitude inner "
             "loop; see cpp/include/talon/controller.hpp for conventions).")
        .def("step", &CascadedController::step,
             py::arg("estimated_state"), py::arg("pos_desired"),
             py::arg("vel_desired"), py::arg("yaw_desired"), py::arg("dt"),
             "One controller update. pos/vel desired in NED, yaw in rad; "
             "dt is the caller's control period (reserved for future "
             "estimator-coupled terms).")
        .def("reset", &CascadedController::reset,
             "Clear internal state (desired rates, telemetry).")
        .def("set_omega_desired", &CascadedController::set_omega_desired,
             py::arg("omega_desired"),
             "Desired body rate for the inner loop (rad/s); zero by default.")
        .def("last_a_des", &CascadedController::last_a_des,
             "Last gravity-compensated thrust-accel command (NED, m/s^2).")
        .def("last_attitude_error", &CascadedController::last_attitude_error,
             "Last attitude error vector (dimensionless).")
        .def("thrust_max", &CascadedController::thrust_max,
             "Thrust saturation limit (N): 4 * mass * gravity.");

    py::class_<EKFConfig>(m, "EKFConfig")
        .def(py::init<>())
        .def_readwrite("accel_noise_density", &EKFConfig::accel_noise_density,
                       "Continuous accel noise density (m/s^2/sqrt(Hz))")
        .def_readwrite("gyro_noise_density", &EKFConfig::gyro_noise_density,
                       "Continuous gyro noise density (rad/s/sqrt(Hz))")
        .def_readwrite("accel_bias_walk", &EKFConfig::accel_bias_walk,
                       "Accel bias random walk (m/s^2/sqrt(s))")
        .def_readwrite("gyro_bias_walk", &EKFConfig::gyro_bias_walk,
                       "Gyro bias random walk (rad/s/sqrt(s))")
        .def_readwrite("gps_pos_noise", &EKFConfig::gps_pos_noise,
                       "GPS horizontal position 1-sigma (m)")
        .def_readwrite("gps_pos_noise_vertical",
                       &EKFConfig::gps_pos_noise_vertical,
                       "GPS vertical position 1-sigma (m)")
        .def_readwrite("gps_vel_noise", &EKFConfig::gps_vel_noise,
                       "GPS velocity 1-sigma (m/s)")
        .def_readwrite("innovation_gate_chi2", &EKFConfig::innovation_gate_chi2,
                       "Chi-square innovation gate (6 DOF)")
        .def_readwrite("init_pos_std", &EKFConfig::init_pos_std,
                       "Initial position std (m)")
        .def_readwrite("init_vel_std", &EKFConfig::init_vel_std,
                       "Initial velocity std (m/s)")
        .def_readwrite("init_att_std", &EKFConfig::init_att_std,
                       "Initial attitude std (rad)")
        .def_readwrite("init_gyro_bias_std", &EKFConfig::init_gyro_bias_std,
                       "Initial gyro bias std (rad/s)")
        .def_readwrite("init_accel_bias_std", &EKFConfig::init_accel_bias_std,
                       "Initial accel bias std (m/s^2)");

    py::class_<EKFNominalState>(m, "EKFNominalState")
        .def(py::init<>())
        .def_readwrite("position", &EKFNominalState::position,
                       "NED position [n, e, d] (m)")
        .def_readwrite("velocity", &EKFNominalState::velocity,
                       "NED velocity (m/s)")
        .def_readwrite("attitude", &EKFNominalState::attitude,
                       "Body-to-NED Hamilton quaternion")
        .def_readwrite("gyro_bias", &EKFNominalState::gyro_bias,
                       "Gyro bias (rad/s, body)")
        .def_readwrite("accel_bias", &EKFNominalState::accel_bias,
                       "Accel bias (m/s^2, body)")
        .def("__repr__", [](const EKFNominalState& s) {
            return "EKFNominalState(position=[" +
                   std::to_string(s.position.x()) + ", " +
                   std::to_string(s.position.y()) + ", " +
                   std::to_string(s.position.z()) + "])";
        });

    py::class_<Ekf>(m, "Ekf")
        .def(py::init<const EKFConfig&>(), py::arg("config") = EKFConfig{},
             "15-state multiplicative error-state KF (MEKF). Error state "
             "ordering [dp(3), dv(3), dtheta(3), dbg(3), dba(3)]; body-frame "
             "(right) attitude error q_true = q_est (x) dq (see "
             "cpp/include/talon/ekf.hpp).")
        .def("initialize", &Ekf::initialize, py::arg("initial"),
             "Set the nominal state and P to the configured initial "
             "covariances.")
        .def("initialize_level", &Ekf::initialize_level, py::arg("position"),
             py::arg("velocity"), py::arg("yaw") = 0.0,
             "Initialize level (roll = pitch = 0, yaw as given) from the "
             "assumed stationary specific force [0, 0, -g].")
        .def("initialize_from_accel", &Ekf::initialize_from_accel,
             py::arg("position"), py::arg("velocity"), py::arg("accel_body"),
             py::arg("yaw") = 0.0,
             "Initialize, leveling roll/pitch from a stationary "
             "body-frame specific force measurement.")
        .def("predict", &Ekf::predict, py::arg("accel_meas"),
             py::arg("gyro_meas"), py::arg("dt"),
             "IMU propagation (every DT_IMU). accel_meas is the body "
             "specific force (hover reads [0, 0, -g]); gyro_meas the body "
             "rate.")
        .def("update", &Ekf::update, py::arg("gps_position"),
             py::arg("gps_velocity"),
             "GPS position/velocity update in NED (every DT_GPS). Returns "
             "True if accepted, False if gated out (state unchanged).")
        .def("nominal", &Ekf::nominal,
             py::return_value_policy::reference_internal,
             "Nominal EKF state (reference into the filter).")
        .def("covariance", &Ekf::covariance,
             "15x15 covariance matrix as a numpy array (copy).")
        .def("last_innovation", &Ekf::last_innovation,
             "Last GPS innovation [dp(3), dv(3)] (6-vector).")
        .def("last_innovation_chi2", &Ekf::last_innovation_chi2,
             "Chi-square of the last GPS innovation.")
        .def("last_update_accepted", &Ekf::last_update_accepted,
             "Whether the last GPS update passed the innovation gate.")
        .def("mutable_nominal", &Ekf::mutable_nominal,
             py::return_value_policy::reference_internal,
             "Mutable reference to the nominal state (fault-injection hook).")
        .def("mutable_covariance", &Ekf::mutable_covariance,
             py::return_value_policy::reference_internal,
             "Mutable reference to the covariance (fault-injection hook).")
        .def("estimated_body_rates", &Ekf::estimated_body_rates,
             py::arg("gyro_meas"),
             "Bias-corrected body rates gyro_meas - gyro_bias (for the "
             "State.angular_rate view fed to the controller).")
        .def("__repr__", [](const Ekf& ekf) {
            const EKFNominalState& s = ekf.nominal();
            return "Ekf(position=[" + std::to_string(s.position.x()) + ", " +
                   std::to_string(s.position.y()) + ", " +
                   std::to_string(s.position.z()) + "])";
        });
}
