// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/kinematics/DifferentialDriveKinematics.hpp>
#include <prosdk/math/kinematics/DifferentialDriveOdometry.hpp>
#include <prosdk/math/system/Models.hpp>
#include <prosdk/simulation/DifferentialDrivetrainSim.hpp>
#include <prosdk/simulation/EncoderSim.hpp>
#include <prosdk/smartdashboard/Field2d.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/velocity.hpp>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    imu.ResetYaw();

    leftLeader.AddFollower(leftFollower);
    rightLeader.AddFollower(rightFollower);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightLeader.SetInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * WHEEL_RADIUS /
                                    ENCODER_RESOLUTION);
    rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * WHEEL_RADIUS /
                                     ENCODER_RESOLUTION);

    leftEncoder.Reset();
    rightEncoder.Reset();

    rightLeader.SetInverted(true);
  }

  static constexpr prosdk::units::meters_per_second_t MAX_VELOCITY =
      3.0_mps;  // 3 meters per second
  static constexpr prosdk::units::radians_per_second_t MAX_ANGULAR_VELOCITY{
      std::numbers::pi};  // 1/2 rotation per second

  void SetVelocities(
      const prosdk::math::DifferentialDriveWheelVelocities& velocities);
  void Drive(prosdk::units::meters_per_second_t xVelocity,
             prosdk::units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const prosdk::math::Pose2d& pose);

  prosdk::math::Pose2d GetPose() const { return odometry.GetPose(); }

  void SimulationPeriodic();
  void Periodic();

 private:
  static constexpr prosdk::units::meter_t TRACKWIDTH = 0.381_m * 2;
  static constexpr double WHEEL_RADIUS = 0.0508;  // meters
  static constexpr int ENCODER_RESOLUTION = 4096;

  prosdk::PWMSparkMax leftLeader{38};
  prosdk::PWMSparkMax leftFollower{37};
  prosdk::PWMSparkMax rightLeader{36};
  prosdk::PWMSparkMax rightFollower{35};

  prosdk::Encoder leftEncoder{39, 40};
  prosdk::Encoder rightEncoder{41, 42};

  prosdk::math::PIDController leftPIDController{8.5, 0.0, 0.0};
  prosdk::math::PIDController rightPIDController{8.5, 0.0, 0.0};

  prosdk::MPU6050 imu{prosdk::I2C::Port::PORT_0};  // kit IMU (upstream: OnboardIMU)

  prosdk::math::DifferentialDriveKinematics kinematics{TRACKWIDTH};
  prosdk::math::DifferentialDriveOdometry odometry{
      imu.GetRotation2d(), prosdk::units::meter_t{leftEncoder.GetDistance()},
      prosdk::units::meter_t{rightEncoder.GetDistance()}};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward{
      1_V, 3_V / 1_mps};

  // Simulation classes help us simulate our robot
  prosdk::sim::EncoderSim leftEncoderSim{leftEncoder};
  prosdk::sim::EncoderSim rightEncoderSim{rightEncoder};
  prosdk::Field2d fieldSim;
  prosdk::math::LinearSystem<2, 2, 2> drivetrainSystem =
      prosdk::math::Models::DifferentialDriveFromSysId(
          1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq);
  prosdk::sim::DifferentialDrivetrainSim drivetrainSimulator{
      drivetrainSystem, TRACKWIDTH, prosdk::math::DCMotor::CIM(2), 8, 2_in};
};
