// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (c) wpimath micro-benchmarks on the target task stack: geometry, odometry and kinematics, a
// swerve pose estimator, Kalman filter, LQR construction (DARE), a linear system loop, PID,
// ProfiledPID and TrapezoidProfile, spline trajectory generation, and Sleipnir: a problem compiled
// in the sketch plus the archive's two users (ArmFeedforward's Newton solve, Ellipse2d::Nearest).
// The inputs change every iteration where a constant input could be hoisted out of the loop.

#include <cmath>
#include <numbers>
#include <vector>

#include <prosdk/math/controller/ArmFeedforward.hpp>
#include <prosdk/math/controller/LinearQuadraticRegulator.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/ProfiledPIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/estimator/KalmanFilter.hpp>
#include <prosdk/math/estimator/SwerveDrivePoseEstimator.hpp>
#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/geometry/Rotation2d.hpp>
#include <prosdk/math/geometry/Transform2d.hpp>
#include <prosdk/math/geometry/Twist2d.hpp>
#include <prosdk/math/kinematics/DifferentialDriveKinematics.hpp>
#include <prosdk/math/kinematics/DifferentialDriveOdometry.hpp>
#include <prosdk/math/kinematics/SwerveDriveKinematics.hpp>
#include <prosdk/math/shape/Ellipse2d.hpp>
#include <prosdk/math/system/DCMotor.hpp>
#include <prosdk/math/system/LinearSystem.hpp>
#include <prosdk/math/system/LinearSystemLoop.hpp>
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
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

#include "Bench.hpp"

namespace qemu::bench {

namespace pm = prosdk::math;
namespace pu = prosdk::units;

namespace {

pm::LinearSystem<2, 1, 1> Elevator() {
  auto motors = pm::DCMotor::Vex775Pro(2);
  return pm::Models::ElevatorFromPhysicalConstants(motors, 5_kg, 0.0181864_m, 1.0).Slice(0);
}

void Geometry() {
  pm::Rotation2d heading{0_deg};
  const pm::Rotation2d step{0.7_deg};
  Run("rotation2d_rotateby", 20000, [&] {
    heading = heading.RotateBy(step);
    DoNotOptimize(heading);
  });

  pm::Pose2d pose{1_m, 2_m, 30_deg};
  const pm::Transform2d move{0.01_m, 0.002_m, 0.5_deg};
  Run("pose2d_transformby", 20000, [&] {
    pose = pose.TransformBy(move);
    DoNotOptimize(pose);
  });

  // exp of a twist (odometry's pose update) and log of the relative transform back
  const pm::Pose2d start{1_m, 2_m, 30_deg};
  pm::Pose2d p = start;
  pm::Twist2d twist{0.02_m, 0.001_m, 0.01_rad};
  Run("pose2d_exp_log", 10000, [&] {
    p = p + twist.Exp();
    pm::Twist2d back = pm::Transform2d{start, p}.Log();
    DoNotOptimize(back);
  });
}

void Drivetrains() {
  pm::DifferentialDriveOdometry odom{0_deg, 0_m, 0_m};
  double d = 0.0;
  Run("diff_odometry_update", 10000, [&] {
    d += 0.001;
    const pm::Pose2d& pose = odom.Update(pm::Rotation2d{pu::radian_t{d}}, pu::meter_t{d},
                                         pu::meter_t{d * 1.01});
    DoNotOptimize(pose);
  });

  const pm::DifferentialDriveKinematics diff{0.6_m};
  double v = 0.0;
  Run("diff_kinematics_to_wheels", 20000, [&] {
    v += 0.0001;
    auto wheels = diff.ToWheelVelocities({pu::meters_per_second_t{v}, 0_mps, 0.5_rad_per_s});
    DoNotOptimize(wheels);
  });

  pm::SwerveDriveKinematics<4> swerve{pm::Translation2d{0.3_m, 0.3_m},
                                      pm::Translation2d{0.3_m, -0.3_m},
                                      pm::Translation2d{-0.3_m, 0.3_m},
                                      pm::Translation2d{-0.3_m, -0.3_m}};
  Run("swerve_kinematics_to_module_velocities", 5000, [&] {
    v += 0.0001;
    auto states = swerve.ToSwerveModuleVelocities(
        {pu::meters_per_second_t{v}, 0.5_mps, 1_rad_per_s});
    DoNotOptimize(states);
  });

  // pose estimator with explicit timestamps (5 ms steps), vision measurements 100 ms in the past
  prosdk::util::array<pm::SwerveModulePosition, 4> modules{
      pm::SwerveModulePosition{}, pm::SwerveModulePosition{}, pm::SwerveModulePosition{},
      pm::SwerveModulePosition{}};
  pm::SwerveDrivePoseEstimator<4> estimator{swerve, pm::Rotation2d{}, modules, pm::Pose2d{},
                                            {0.1, 0.1, 0.1}, {0.1, 0.1, 0.1}};
  double t = 0.0;
  double dist = 0.0;
  auto step = [&] {
    t += 0.005;
    dist += 0.01;
    for (auto& m : modules) {
      m.distance = pu::meter_t{dist};
      m.angle = pm::Rotation2d{pu::radian_t{dist * 0.1}};
    }
    return estimator.UpdateWithTime(pu::second_t{t}, pm::Rotation2d{pu::radian_t{dist * 0.1}},
                                    modules);
  };
  // fill the 1.5 s pose buffer first: Update then appends and drops one sample
  for (int i = 0; i < 400; ++i) {
    step();
  }
  Run("swerve_pose_estimator_update", 2000, [&] {
    pm::Pose2d pose = step();
    DoNotOptimize(pose);
  });
  Run("swerve_pose_estimator_vision", 200, [&] {
    step();
    estimator.AddVisionMeasurement(pm::Pose2d{pu::meter_t{dist}, 0.01_m, 1_deg},
                                   pu::second_t{t - 0.1});
    pm::Pose2d pose = estimator.GetEstimatedPosition();
    DoNotOptimize(pose);
  });
}

void StateSpace() {
  pm::LinearSystem<2, 1, 1> elevator = Elevator();

  Run("lqr_construct_2x1", 100, [&] {
    pm::LinearQuadraticRegulator<2, 1> lqr{elevator, {0.02, 0.4}, {12.0}, 5_ms};
    DoNotOptimize(lqr.K());
  });

  pm::KalmanFilter<2, 1, 1> kf{elevator, {0.0508, 0.5}, {0.001}, 5_ms};
  double y = 0.0;
  Run("kalman_2state_predict_correct", 2000, [&] {
    y += 0.0001;
    kf.Predict(pm::Vectord<1>{6.0}, 5_ms);
    kf.Correct(pm::Vectord<1>{6.0}, pm::Vectord<1>{y});
    DoNotOptimize(kf.Xhat());
  });

  pm::LinearQuadraticRegulator<2, 1> lqr{elevator, {0.02, 0.4}, {12.0}, 5_ms};
  pm::KalmanFilter<2, 1, 1> observer{elevator, {0.0508, 0.5}, {0.001}, 5_ms};
  pm::LinearSystemLoop<2, 1, 1> loop{elevator, lqr, observer, 12_V, 5_ms};
  loop.SetNextR(pm::Vectord<2>{1.0, 0.0});
  Run("linear_system_loop_2state", 2000, [&] {
    y += 0.0001;
    loop.Correct(pm::Vectord<1>{y});
    loop.Predict(5_ms);
    DoNotOptimize(loop.U());
  });
}

void Controllers() {
  pm::PIDController pid{1.0, 0.1, 0.01};
  double x = 0.0;
  Run("pid_calculate", 20000, [&] {
    x += 0.0001;
    double out = pid.Calculate(x, 1.0);
    DoNotOptimize(out);
  });

  pm::SimpleMotorFeedforward<pu::meters> ff{1_V, 3_V / 1_mps, 0.2_V / 1_mps_sq};
  Run("simple_motor_ff_calculate", 20000, [&] {
    x += 0.0001;
    pu::volt_t out = ff.Calculate(pu::meters_per_second_t{x});
    DoNotOptimize(out);
  });

  pm::ProfiledPIDController<pu::meters> ppid{1.3, 0.0, 0.7, {2_mps, 1_mps_sq}, 5_ms};
  double pos = 0.0;
  Run("profiled_pid_calculate", 10000, [&] {
    double out = ppid.Calculate(pu::meter_t{pos}, 3_m);
    pos += out * 1e-4;
    DoNotOptimize(out);
  });

  pm::TrapezoidProfile<pu::meters> profile{{2_mps, 1_mps_sq}};
  pm::TrapezoidProfile<pu::meters>::State goal{3_m, 0_mps}, state{0_m, 0_mps};
  int k = 0;
  Run("trapezoid_profile_calculate", 20000, [&] {
    // restart every 1000 steps (20 s of profile): stay on the moving part of the profile
    if (++k % 1000 == 0) {
      state = {0_m, 0_mps};
    }
    state = profile.Calculate(20_ms, state, goal);
    DoNotOptimize(state);
  });
}

void Trajectory() {
  // upstream TestDrivetrainSplineTrajectory (2018 cross scale), as QemuSmoke case a
  pm::TrajectoryConfig config{12_fps, 12_fps_sq};
  const pm::Pose2d sideStart{1.54_ft, 23.23_ft, 180_deg};
  const pm::Pose2d crossScale{23.7_ft, 6.8_ft, -160_deg};
  config.SetReversed(true);
  std::vector<pm::Translation2d> waypoints{
      (sideStart + pm::Transform2d{pm::Translation2d{-13_ft, 0_ft}, 0_deg}).Translation(),
      (sideStart + pm::Transform2d{pm::Translation2d{-19.5_ft, 5.0_ft}, -90_deg}).Translation()};
  size_t samples = 0;
  Run("trajectory_generate_2waypoints", 10, [&] {
    auto trajectory =
        pm::DrivetrainSplineTrajectoryGenerator::Generate(sideStart, waypoints, crossScale, config);
    samples = trajectory.Samples().size();
    DoNotOptimize(samples);
  });
}

void Sleipnir() {
  // a problem compiled in the sketch (header templates): equality-constrained QP, 2 variables
  double sink = 0.0;
  Run("sleipnir_qp_2var", 20, [&] {
    slp::Problem<double> problem;
    auto x = problem.decision_variable();
    auto y = problem.decision_variable();
    problem.minimize(slp::pow(x - 3.0, 2) + slp::pow(y + 1.0, 2));
    problem.subject_to(x + y == 1.0);
    problem.subject_to(x >= 0.0);
    problem.solve();
    sink += x.value();
    DoNotOptimize(sink);
  });

  // in the archive: ArmFeedforward::Calculate(angle, v0, v1) solves the arm dynamics (Newton)
  pm::ArmFeedforward arm{0.5_V, 1_V, 1.5_V / 1_rad_per_s, 2_V / 1_rad_per_s_sq};
  double a = std::numbers::pi / 3;
  Run("sleipnir_arm_feedforward", 20, [&] {
    a += 0.001;
    double ff = arm.Calculate(pu::radian_t{a}, 1_rad_per_s, 1.05_rad_per_s).value();
    DoNotOptimize(ff);
  });

  constexpr pm::Pose2d center{1_m, 2_m, 270_deg};
  constexpr pm::Ellipse2d ellipse{center, 1_m, 2_m};
  double px = -1.0;
  Run("sleipnir_ellipse2d_nearest", 20, [&] {
    px += 0.001;
    auto nearest = ellipse.Nearest(pm::Translation2d{pu::meter_t{px}, 2.5_m});
    DoNotOptimize(nearest);
  });
}

}  // namespace

void BenchMath() {
  Geometry();
  Drivetrains();
  StateSpace();
  Controllers();
  Trajectory();
  Sleipnir();
}

}  // namespace qemu::bench
