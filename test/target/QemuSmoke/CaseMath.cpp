// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (a) wpimath on the target task stack: Eigen (LQR: discrete ARE), Sleipnir (a template problem
// compiled in the sketch, ArmFeedforward and Ellipse2d::Nearest from the archive), spline
// trajectory generation and odometry. Expected values are upstream's unit-test values.

#include <cmath>
#include <numbers>
#include <vector>

#include <prosdk/math/controller/ArmFeedforward.hpp>
#include <prosdk/math/controller/LinearQuadraticRegulator.hpp>
#include <prosdk/math/estimator/KalmanFilter.hpp>
#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/geometry/Transform2d.hpp>
#include <prosdk/math/kinematics/DifferentialDriveOdometry.hpp>
#include <prosdk/math/shape/Ellipse2d.hpp>
#include <prosdk/math/system/DCMotor.hpp>
#include <prosdk/math/system/LinearSystem.hpp>
#include <prosdk/math/system/Models.hpp>
#include <prosdk/math/trajectory/DrivetrainSplineTrajectoryGenerator.hpp>
#include <prosdk/math/trajectory/TrajectoryConfig.hpp>
#include <prosdk/math/trajectory/TrapezoidProfile.hpp>
#include <prosdk/thirdparty/sleipnir/sleipnir/optimization/problem.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/angular_acceleration.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/mass.hpp>
#include <prosdk/units/moment_of_inertia.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

#include <esp_timer.h>

#include "Harness.hpp"

namespace qemu {

namespace pm = prosdk::math;

static bool Near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

void CaseMath(Ctx& ctx) {
  // ---- LQR (Eigen, DARE) ----
  int64_t t0 = esp_timer_get_time();
  pm::LinearSystem<2, 1, 1> elevator = [] {
    auto motors = pm::DCMotor::Vex775Pro(2);
    return pm::Models::ElevatorFromPhysicalConstants(motors, 5_kg, 0.0181864_m, 1.0).Slice(0);
  }();
  pm::Matrixd<1, 2> K = pm::LinearQuadraticRegulator<2, 1>{elevator, {0.02, 0.4}, {12.0}, 5_ms}.K();
  QCHECK_MSG(ctx, Near(K(0, 0), 522.87006795347486, 1e-6), "K0={:.12f}", K(0, 0));
  QCHECK_MSG(ctx, Near(K(0, 1), 38.239878385020411, 1e-6), "K1={:.12f}", K(0, 1));
  ctx.Note("lqr_us={}", esp_timer_get_time() - t0);

  // ---- Kalman filter (Eigen, DARE for the steady-state gain) ----
  t0 = esp_timer_get_time();
  pm::LinearSystem<1, 1, 1> flywheel =
      pm::Models::FlywheelFromPhysicalConstants(pm::DCMotor::NEO(2), 0.00032_kg_sq_m, 1.0);
  pm::KalmanFilter<1, 1, 1> kf{flywheel, {3.0}, {0.01}, 5_ms};
  for (int i = 0; i < 100; ++i) {
    kf.Predict(pm::Vectord<1>{12.0}, 5_ms);
    kf.Correct(pm::Vectord<1>{12.0}, pm::Vectord<1>{100.0});
  }
  // reference: the same loop on the host build (g++ x86_64, .build/host/libprosdk.a); the
  // estimate settles above the measured 100 rad/s because the model, driven at 12 V, keeps
  // pulling it toward the free speed
  QCHECK_MSG(ctx, Near(kf.Xhat(0), 123.98198151009245, 1e-9), "xhat={:.17g}", kf.Xhat(0));
  ctx.Note("kalman_us={}", esp_timer_get_time() - t0);

  // ---- Sleipnir: a problem compiled in the sketch (header templates) ----
  t0 = esp_timer_get_time();
  {
    slp::Problem<double> problem;
    auto x = problem.decision_variable();
    auto y = problem.decision_variable();
    problem.minimize(slp::pow(x - 3.0, 2) + slp::pow(y + 1.0, 2));
    problem.subject_to(x + y == 1.0);
    problem.subject_to(x >= 0.0);
    auto status = problem.solve();
    QCHECK_MSG(ctx, status == slp::ExitStatus::SUCCESS, "status={}", static_cast<int>(status));
    QCHECK_MSG(ctx, Near(x.value(), 2.5, 1e-6) && Near(y.value(), -1.5, 1e-6), "x={} y={}",
               x.value(), y.value());
  }
  ctx.Note("sleipnir_us={}", esp_timer_get_time() - t0);

  // ---- Sleipnir inside the archive: Ellipse2d::Nearest, ArmFeedforward ----
  t0 = esp_timer_get_time();
  constexpr pm::Pose2d center{1_m, 2_m, 270_deg};
  constexpr pm::Ellipse2d ellipse{center, 1_m, 2_m};
  auto nearest = ellipse.Nearest(pm::Translation2d{-1_m, 2.5_m});
  QCHECK_MSG(ctx, Near(nearest.X().value(), -0.8512799937611617, 1e-6), "x={:.12f}",
             nearest.X().value());
  QCHECK_MSG(ctx, Near(nearest.Y().value(), 2.378405333174535, 1e-6), "y={:.12f}",
             nearest.Y().value());
  {
    constexpr auto Ks = 0.39671_V;
    constexpr auto Kv = 2.7167_V / 1_rad_per_s;
    constexpr auto Ka = 1e-2_V / 1_rad_per_s_sq;
    constexpr auto Kg = 0.2708_V;
    pm::ArmFeedforward armFF{Ks, Kg, Kv, Ka};
    constexpr auto angle = 1_rad;
    constexpr auto v0 = 0.02_rad_per_s;
    constexpr auto v1 = 0_rad_per_s;
    double got = armFF.Calculate(angle, v0, v1).value();
    double want = (Ks + Kv * v0 + Ka * ((v1 - v0) / 20_ms) +
                   Kg * prosdk::units::math::cos(angle)).value();
    QCHECK_MSG(ctx, Near(got, want, 1e-9), "got={} want={}", got, want);
    // well-conditioned case: Newton solve of the arm dynamics (upstream: CHECK_NEAR 0.5/2.5)
    pm::ArmFeedforward arm2{0.5_V, 1_V, 1.5_V / 1_rad_per_s, 2_V / 1_rad_per_s_sq};
    double ff = arm2.Calculate(std::numbers::pi / 3 * 1_rad, 1_rad_per_s, 1.05_rad_per_s).value();
    QCHECK_MSG(ctx, std::isfinite(ff) && ff > 2.0 && ff < 8.0, "ff={}", ff);
  }
  ctx.Note("archive_sleipnir_us={}", esp_timer_get_time() - t0);

  // ---- spline trajectory (upstream TestDrivetrainSplineTrajectory, 2018 cross scale) ----
  t0 = esp_timer_get_time();
  {
    pm::TrajectoryConfig config{12_fps, 12_fps_sq};
    const pm::Pose2d sideStart{1.54_ft, 23.23_ft, 180_deg};
    const pm::Pose2d crossScale{23.7_ft, 6.8_ft, -160_deg};
    config.SetReversed(true);
    std::vector<pm::Translation2d> waypoints{
        (sideStart + pm::Transform2d{pm::Translation2d{-13_ft, 0_ft}, 0_deg}).Translation(),
        (sideStart + pm::Transform2d{pm::Translation2d{-19.5_ft, 5.0_ft}, -90_deg})
            .Translation()};
    auto trajectory =
        pm::DrivetrainSplineTrajectoryGenerator::Generate(sideStart, waypoints, crossScale, config);
    const auto& samples = trajectory.Samples();
    QCHECK_MSG(ctx, samples.size() > 10, "samples={}", samples.size());
    double duration = trajectory.Duration().value();
    QCHECK_MSG(ctx, duration > 1.0 && duration < 20.0, "duration={}", duration);
    double maxV = 0.0;
    for (const auto& s : samples) {
      maxV = std::max(maxV, std::hypot(s.velocity.vx.value(), s.velocity.vy.value()));
    }
    QCHECK_MSG(ctx, maxV <= (12_fps).value() + 1e-6, "max velocity {}", maxV);
    auto end = trajectory.End().pose;
    QCHECK_MSG(ctx, end.Translation().Distance(crossScale.Translation()).value() < 1e-3,
               "end=({}, {})", end.X().value(), end.Y().value());
    ctx.Note("traj_samples={} traj_duration_s={:.3f}", samples.size(), duration);
  }
  ctx.Note("trajectory_us={}", esp_timer_get_time() - t0);

  // ---- trapezoid profile and odometry ----
  {
    pm::TrapezoidProfile<prosdk::units::meters> profile{{2_mps, 1_mps_sq}};
    pm::TrapezoidProfile<prosdk::units::meters>::State goal{3_m, 0_mps}, state{0_m, 0_mps};
    for (int i = 0; i < 1000; ++i) {
      state = profile.Calculate(20_ms, state, goal);
    }
    QCHECK_MSG(ctx, Near(state.position.value(), 3.0, 1e-6), "pos={}", state.position.value());

    pm::DifferentialDriveOdometry odom{0_deg, 0_m, 0_m};
    for (int i = 1; i <= 100; ++i) {
      odom.Update(0_deg, 0.01_m * i, 0.01_m * i);
    }
    QCHECK_MSG(ctx, Near(odom.GetPose().X().value(), 1.0, 1e-9), "x={}",
               odom.GetPose().X().value());
  }
}

}  // namespace qemu
