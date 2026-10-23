// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SimpleDifferentialDriveSimulation: a differential drive with DifferentialDrivetrainSim
// physics that follows a trajectory with LTV unicycle control in autonomous (the trajectory
// is upstream's one-sample placeholder; replace it with a trajectory generator), ported to
// prosdk from upstream wpilibcExamples/src/main/cpp/examples/SimpleDifferentialDriveSimulation.
// The simulation part only runs in the host simulator; on the robot it is inert.
//
// Wiring (GPIO numbers; channel = GPIO):
//   PWMSparkMax (or any PWM ESC): GPIO 38 left leader, GPIO 37 left follower,
//   GPIO 36 right leader, GPIO 35 right follower
//   Encoders A/B: GPIO 39/40 left, GPIO 41/42 right
// Gamepad 0: left Y drives, right X turns.
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).
// The Driver Station's "Veri" page draws the Field2d as a field widget (probot-core #40).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <vector>

#include "Drivetrain.hpp"
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/math/controller/LTVUnicycleController.hpp>
#include <prosdk/math/filter/SlewRateLimiter.hpp>
#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/kinematics/ChassisAccelerations.hpp>
#include <prosdk/math/kinematics/ChassisVelocities.hpp>
#include <prosdk/math/trajectory/HolonomicSample.hpp>
#include <prosdk/math/trajectory/HolonomicTrajectory.hpp>
#include <prosdk/system/Timer.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/angular_acceleration.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/dimensionless.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  Robot()
      // Replace this with a call to a trajectory generator
      : trajectory{prosdk::math::HolonomicTrajectory{
            std::vector<prosdk::math::HolonomicSample>{prosdk::math::HolonomicSample{
                0_s, prosdk::math::Pose2d{0_m, 0_m, 0_rad},
                prosdk::math::ChassisVelocities{0_mps, 0_mps, 0_rad_per_s},
                prosdk::math::ChassisAccelerations{0_mps_sq, 0_mps_sq,
                                                0_rad_per_s_sq}}}}} {}

  void RobotPeriodic() override { drive.Periodic(); }

  void AutonomousInit() override {
    timer.Restart();
    drive.ResetOdometry(trajectory.Start().pose);
  }

  void AutonomousPeriodic() override {
    auto elapsed = timer.Get();
    auto reference = trajectory.SampleAt(elapsed);
    auto velocities = feedback.Calculate(drive.GetPose(), reference);
    drive.Drive(velocities.vx, velocities.omega);
  }

  void TeleopPeriodic() override {
    // Get the x velocity. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xVelocity = -velocityLimiter.Calculate(controller.GetLeftY()) *
                           Drivetrain::MAX_VELOCITY;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -rotLimiter.Calculate(controller.GetRightX()) *
               Drivetrain::MAX_ANGULAR_VELOCITY;

    drive.Drive(xVelocity, rot);
  }

  void SimulationPeriodic() override { drive.SimulationPeriodic(); }

 private:
  prosdk::Gamepad controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> velocityLimiter{3 / 1_s};
  prosdk::math::SlewRateLimiter<prosdk::units::scalar> rotLimiter{3 / 1_s};

  Drivetrain drive;
  prosdk::math::HolonomicTrajectory trajectory;
  prosdk::math::LTVUnicycleController feedback{20_ms};
  prosdk::Timer timer;
};

PROSDK_ROBOT_MAIN(Robot)
