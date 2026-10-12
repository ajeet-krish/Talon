#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "talon/constants.hpp"
#include "talon/dynamics.hpp"
#include "talon/types.hpp"
#include "talon/wind.hpp"

namespace py = pybind11;

using talon::DrydenWind;
using talon::Quaternion;
using talon::State;
using talon::VehicleParams;
using talon::Vec3;

PYBIND11_MODULE(_core, m) {
    m.doc() = "Talon C++ computational kernel: rigid-body dynamics, wind, constants.";

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
}
