// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/fields/Field.hpp>
#include <prosdk/fields/fields.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/estimator/DifferentialDrivePoseEstimator.hpp>
#include <prosdk/math/geometry/Pose2d.hpp>
#include <prosdk/math/geometry/Pose3d.hpp>
#include <prosdk/math/geometry/Transform3d.hpp>
#include <prosdk/math/kinematics/DifferentialDriveKinematics.hpp>
#include <prosdk/math/system/Models.hpp>
#include <prosdk/simulation/DifferentialDrivetrainSim.hpp>
#include <prosdk/simulation/EncoderSim.hpp>
#include <prosdk/smartdashboard/Field2d.hpp>
#include <prosdk/units/angle.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/velocity.hpp>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain();

  static constexpr prosdk::units::meters_per_second_t MAX_VELOCITY =
      3.0_mps;  // 3 meters per second
  static constexpr prosdk::units::radians_per_second_t MAX_ANGULAR_VELOCITY{
      std::numbers::pi};  // 1/2 rotation per second

  /**
   * Sets the desired wheel velocities.
   *
   * @param velocities The desired wheel velocities.
   */
  void SetVelocities(
      const prosdk::math::DifferentialDriveWheelVelocities& velocities);

  /** Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xVelocity Linear velocity.
   * @param rot Angular Velocity.
   */
  void Drive(prosdk::units::meters_per_second_t xVelocity,
             prosdk::units::radians_per_second_t rot);

  /**
   * Updates the field-relative position.
   */
  void UpdateOdometry();

  /**
   * This function is called periodically during simulation. */
  void SimulationPeriodic();

  /** This function is called periodically, regardless of mode. */
  void Periodic();

  /**
   * Computes and publishes to an array (upstream: a networktables topic) the transformation from
   * the camera's pose to the object's pose. This function exists solely for the
   * purposes of simulation, and this would normally be handled by computer
   * vision.
   *
   * <p>The object could be a target or a fiducial marker.
   *
   * @param objectInField The object's field-relative position.
   * @param robotToCamera The transformation from the robot's pose to the
   * camera's pose.
   * @param cameraToObjectEntry The networktables entry publishing and querying
   * example computer vision measurements.
   * @param drivetrainSimulation A DifferentialDrivetrainSim modeling the
   * robot's drivetrain.
   */
  void PublishCameraToObject(
      prosdk::math::Pose3d objectInField, prosdk::math::Transform3d robotToCamera,
      std::array<double, 7>& cameraToObjectEntry,
      prosdk::sim::DifferentialDrivetrainSim drivetrainSimulator);

  /**
   * Queries the camera-to-object transformation from that array (upstream: networktables) to compute
   * the robot's field-relative pose from vision measurements.
   *
   * <p>The object could be a target or a fiducial marker.
   *
   * @param objectInField The object's field-relative position.
   * @param robotToCamera The transformation from the robot's pose to the
   * camera's pose.
   * @param cameraToObjectEntry The networktables entry publishing and querying
   * example computer vision measurements.
   */
  prosdk::math::Pose3d ObjectToRobotPose(
      prosdk::math::Pose3d objectInField, prosdk::math::Transform3d robotToCamera,
      std::array<double, 7>& cameraToObjectEntry);

 private:
  static constexpr prosdk::units::meter_t TRACKWIDTH = 0.381_m * 2;
  static constexpr prosdk::units::meter_t WHEEL_RADIUS = 0.0508_m;
  static constexpr int ENCODER_RESOLUTION = 4096;

  static constexpr std::array<double, 7> DEFAULT_VAL{0.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.0, 0.0};

  prosdk::math::Transform3d robotToCamera{
      prosdk::math::Translation3d{1_m, 1_m, 1_m},
      prosdk::math::Rotation3d{0_rad, 0_rad,
                            prosdk::units::radian_t{std::numbers::pi / 2}}};

  // prosdk has no NetworkTables: the simulated camera-to-object transform is
  // passed through this array (upstream: a "cameraToObjectTopic" NT entry).
  std::array<double, 7> cameraToObjectEntry = DEFAULT_VAL;
  std::array<double, 7>& cameraToObjectEntryRef = cameraToObjectEntry;

  prosdk::fields::Field field{
      prosdk::fields::GetField(prosdk::fields::FieldId::FRC_2024_CRESCENDO)};
  prosdk::math::Pose3d objectInField{field.GetTagPose(1).value()};

  prosdk::PWMSparkMax leftLeader{38};
  prosdk::PWMSparkMax leftFollower{37};
  prosdk::PWMSparkMax rightLeader{36};
  prosdk::PWMSparkMax rightFollower{35};

  prosdk::Encoder leftEncoder{39, 40};
  prosdk::Encoder rightEncoder{41, 42};

  prosdk::math::PIDController leftPIDController{1.0, 0.0, 0.0};
  prosdk::math::PIDController rightPIDController{1.0, 0.0, 0.0};

  prosdk::MPU6050 imu{prosdk::I2C::Port::PORT_0};  // kit IMU (upstream: OnboardIMU)

  prosdk::math::DifferentialDriveKinematics kinematics{TRACKWIDTH};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  prosdk::math::DifferentialDrivePoseEstimator poseEstimator{
      imu.GetRotation2d(),
      prosdk::units::meter_t{leftEncoder.GetDistance()},
      prosdk::units::meter_t{rightEncoder.GetDistance()},
      prosdk::math::Pose2d{},
      {0.01, 0.01, 0.01},
      {0.1, 0.1, 0.1}};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward{
      1_V, 3_V / 1_mps};

  // Simulation classes
  prosdk::sim::EncoderSim leftEncoderSim{leftEncoder};
  prosdk::sim::EncoderSim rightEncoderSim{rightEncoder};
  // No IMU sim: the MPU6050 has no sim class (upstream: OnboardIMUSim imuSim).
  prosdk::Field2d fieldSim;
  prosdk::Field2d fieldApproximation;
  prosdk::math::LinearSystem<2, 2, 2> drivetrainSystem =
      prosdk::math::Models::DifferentialDriveFromSysId(
          1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq);
  prosdk::sim::DifferentialDrivetrainSim drivetrainSimulator{
      drivetrainSystem, TRACKWIDTH, prosdk::math::DCMotor::CIM(2), 8, 2_in};
};
