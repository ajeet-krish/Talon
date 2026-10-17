#include <cmath>

#include <gtest/gtest.h>

#include "talon/constants.hpp"
#include "talon/wind.hpp"

using talon::DrydenWind;

namespace {

// The spec'd statistical thresholds (|mean| < 0.05*sigma, variance within 20%
// of sigma^2) over 1e5 steps at Va = 5 m/s are only meaningful when the run
// spans many correlation times. With the default L_u = 200 m at 5 m/s the
// integral time scale is L/Va = 40 s, so 500 s of data holds only ~6
// independent samples and the thresholds are statistically infeasible. These
// short-scale parameters keep the exact same recursion while making 1e5 steps
// ~2500 independent samples, which the spec'd thresholds are designed for.
DrydenWind make_statistics_wind(unsigned int seed) {
    return DrydenWind(2.0, 2.0, 1.0, /*L_u=*/0.5, /*L_v=*/0.5, /*L_w=*/0.5,
                      talon::DT, seed);
}

}  // namespace

TEST(Wind, Statistics) {
    DrydenWind wind = make_statistics_wind(42u);
    const double airspeed = 5.0;
    const int steps = 100000;

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_sq = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_abs = Eigen::Vector3d::Zero();
    for (int i = 0; i < steps; ++i) {
        const Eigen::Vector3d w = wind.step(airspeed);
        sum += w;
        sum_sq += w.cwiseProduct(w);
        max_abs = max_abs.cwiseMax(w.cwiseAbs());
    }

    const Eigen::Vector3d sigma(2.0, 2.0, 1.0);
    const Eigen::Vector3d mean = sum / static_cast<double>(steps);
    const Eigen::Vector3d var =
        sum_sq / static_cast<double>(steps) - mean.cwiseProduct(mean);

    for (int i = 0; i < 3; ++i) {
        EXPECT_LT(std::abs(mean[i]), 0.05 * sigma[i]);
        EXPECT_NEAR(var[i], sigma[i] * sigma[i], 0.2 * sigma[i] * sigma[i]);
        // OU envelope: nothing near a runaway.
        EXPECT_LT(max_abs[i], 6.0 * sigma[i]);
    }
}

TEST(Wind, Reproducibility) {
    DrydenWind a1(2.0, 2.0, 1.0, 200.0, 200.0, 50.0, talon::DT, 42u);
    DrydenWind a2(2.0, 2.0, 1.0, 200.0, 200.0, 50.0, talon::DT, 42u);
    DrydenWind b(2.0, 2.0, 1.0, 200.0, 200.0, 50.0, talon::DT, 7u);

    bool identical_same_seed = true;
    bool identical_diff_seed = true;
    for (int i = 0; i < 200; ++i) {
        const Eigen::Vector3d wa = a1.step(5.0);
        const Eigen::Vector3d wb = a2.step(5.0);
        const Eigen::Vector3d wc = b.step(5.0);
        if ((wa - wb).norm() != 0.0) {
            identical_same_seed = false;
        }
        if ((wa - wc).norm() != 0.0) {
            identical_diff_seed = false;
        }
    }
    EXPECT_TRUE(identical_same_seed);
    EXPECT_FALSE(identical_diff_seed);
}

TEST(Wind, ResetRestartsSequence) {
    DrydenWind wind(2.0, 2.0, 1.0, 200.0, 200.0, 50.0, talon::DT, 42u);

    Eigen::Matrix<double, 50, 3> first_run;
    for (int i = 0; i < 50; ++i) {
        first_run.row(i) = wind.step(5.0).transpose();
    }

    // Run it off course, then reset.
    for (int i = 0; i < 137; ++i) {
        wind.step(5.0);
    }
    wind.reset();
    EXPECT_EQ(wind.wind().norm(), 0.0);

    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(wind.step(5.0), first_run.row(i).transpose());
    }
}

TEST(Wind, ZeroAirspeedFreezes) {
    DrydenWind wind(2.0, 2.0, 1.0, 200.0, 200.0, 50.0, talon::DT, 42u);
    wind.step(5.0);   // move away from the zero initial state
    const Eigen::Vector3d frozen = wind.wind();
    wind.step(0.0);   // Va = 0: alpha = 1, no noise injection
    EXPECT_EQ(wind.wind(), frozen);
}
