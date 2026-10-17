#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "talon/constants.hpp"

namespace talon {

// Small alias for Eigen::Vector3d. All 3-vectors in Talon are Eigen vectors so
// they interoperate with pybind11/numpy for free (see bindings/pytalon.cpp).
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// Hamilton quaternion, scalar-first [w, x, y, z], representing the rotation
// from body frame to NED frame: v_ned = R(q) * v_body.
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double norm() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalized() const {
        const double n = norm();
        return Quaternion{w / n, x / n, y / n, z / n};
    }

    Quaternion conjugate() const {
        return Quaternion{w, -x, -y, -z};
    }

    // Body-to-NED direction cosine matrix.
    Mat3 to_dcm() const {
        const double ww = w * w;
        const double xx = x * x;
        const double yy = y * y;
        const double zz = z * z;
        Mat3 R;
        R(0, 0) = 1.0 - 2.0 * (yy + zz);
        R(0, 1) = 2.0 * (x * y - w * z);
        R(0, 2) = 2.0 * (x * z + w * y);
        R(1, 0) = 2.0 * (x * y + w * z);
        R(1, 1) = 1.0 - 2.0 * (xx + zz);
        R(1, 2) = 2.0 * (y * z - w * x);
        R(2, 0) = 2.0 * (x * z - w * y);
        R(2, 1) = 2.0 * (y * z + w * x);
        R(2, 2) = 1.0 - 2.0 * (xx + yy);
        return R;
    }

    // Rotate a vector from body frame to NED frame.
    Vec3 rotate_vector(const Vec3& v_body) const {
        return to_dcm() * v_body;
    }
};

// Full vehicle state. Position/velocity in NED, angular rate in body frame.
struct State {
    Vec3 position = Vec3::Zero();       // NED, m
    Vec3 velocity = Vec3::Zero();       // NED, m/s
    Vec3 angular_rate = Vec3::Zero();   // body frame, rad/s
    Quaternion orientation;             // body-to-NED, Hamilton wxyz
};

// Vehicle physical parameters. Default ctor loads defaults from constants.hpp.
struct VehicleParams {
    double mass;
    Vec3 inertia_diag;      // body frame diagonal inertia, kg m^2
    double arm_length;      // m
    double drag_coeff;      // dimensionless
    double gravity;         // m/s^2
    double air_density;     // kg/m^3
    double vehicle_area;    // m^2 (frontal, for quadratic drag)

    VehicleParams()
        : mass(MASS),
          inertia_diag(IXX, IYY, IZZ),
          arm_length(ARM_LENGTH),
          drag_coeff(DRAG_COEFF),
          gravity(GRAVITY),
          air_density(AIR_DENSITY),
          vehicle_area(VEHICLE_AREA) {}
};

}  // namespace talon
