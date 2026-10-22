// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.hpp"

#include <array>
#include <numbers>

#include <prosdk/math/util/ComputerVisionUtil.hpp>
#include <prosdk/system/RobotController.hpp>
#include <prosdk/system/Timer.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

Drivetrain::Drivetrain() {
  leftLeader.AddFollower(leftFollower);
  rightLeader.AddFollower(rightFollower);

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  rightLeader.SetInverted(true);

  imu.ResetYaw();

  // Set the distance per pulse for the drive encoders. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  leftEncoder.SetDistancePerPulse(
      (2 * std::numbers::pi * WHEEL_RADIUS / ENCODER_RESOLUTION).value());
  rightEncoder.SetDistancePerPulse(
      (2 * std::numbers::pi * WHEEL_RADIUS / ENCODER_RESOLUTION).value());

  leftEncoder.Reset();
  rightEncoder.Reset();
}

void Drivetrain::SetVelocities(
    const prosdk::math::DifferentialDriveWheelVelocities& velocities) {
  const auto leftFeedforward = feedforward.Calculate(velocities.left);
  const auto rightFeedforward = feedforward.Calculate(velocities.right);
  const double leftOutput = leftPIDController.Calculate(
      leftEncoder.GetRate(), velocities.left.value());
  const double rightOutput = rightPIDController.Calculate(
      rightEncoder.GetRate(), velocities.right.value());

  leftLeader.SetVoltage(prosdk::units::volt_t{leftOutput} + leftFeedforward);
  rightLeader.SetVoltage(prosdk::units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(prosdk::units::meters_per_second_t xVelocity,
                       prosdk::units::radians_per_second_t rot) {
  SetVelocities(kinematics.ToWheelVelocities({xVelocity, 0_mps, rot}));
}

void Drivetrain::PublishCameraToObject(
    prosdk::math::Pose3d objectInField, prosdk::math::Transform3d robotToCamera,
    std::array<double, 7>& cameraToObjectEntry,
    prosdk::sim::DifferentialDrivetrainSim drivetrainSimulator) {
  prosdk::math::Pose3d robotInField{drivetrainSimulator.GetPose()};
  prosdk::math::Pose3d cameraInField = robotInField + robotToCamera;
  prosdk::math::Transform3d cameraToObject{cameraInField, objectInField};

  // Publishes double array with Translation3D elements {x, y, z} and Rotation3D
  // elements {w, x, y, z} which describe the cameraToObject transformation.
  std::array<double, 7> val{cameraToObject.X().value(),
                            cameraToObject.Y().value(),
                            cameraToObject.Z().value(),
                            cameraToObject.Rotation().GetQuaternion().W(),
                            cameraToObject.Rotation().GetQuaternion().X(),
                            cameraToObject.Rotation().GetQuaternion().Y(),
                            cameraToObject.Rotation().GetQuaternion().Z()};
  cameraToObjectEntry = val;
}

prosdk::math::Pose3d Drivetrain::ObjectToRobotPose(
    prosdk::math::Pose3d objectInField, prosdk::math::Transform3d robotToCamera,
    std::array<double, 7>& cameraToObjectEntry) {
  const std::array<double, 7>& val = cameraToObjectEntry;

  // Reconstruct cameraToObject Transform3D from the array.
  prosdk::math::Translation3d translation{prosdk::units::meter_t{val[0]},
                                       prosdk::units::meter_t{val[1]},
                                       prosdk::units::meter_t{val[2]}};
  prosdk::math::Rotation3d rotation{
      prosdk::math::Quaternion{val[3], val[4], val[5], val[6]}};
  prosdk::math::Transform3d cameraToObject{translation, rotation};

  return prosdk::math::ObjectToRobotPose(objectInField, cameraToObject,
                                      robotToCamera);
}

void Drivetrain::UpdateOdometry() {
  poseEstimator.Update(imu.GetRotation2d(),
                       prosdk::units::meter_t{leftEncoder.GetDistance()},
                       prosdk::units::meter_t{rightEncoder.GetDistance()});

  // Publish cameraToObject transformation to the array --this would
  // normally be handled by the computer vision solution.
  PublishCameraToObject(objectInField, robotToCamera, cameraToObjectEntryRef,
                        drivetrainSimulator);

  // Compute the robot's field-relative position exclusively from vision
  // measurements.
  prosdk::math::Pose3d visionMeasurement3d =
      ObjectToRobotPose(objectInField, robotToCamera, cameraToObjectEntryRef);

  // Convert robot's pose from prosdk::math::Pose3d to prosdk::math::Pose2d needed to
  // apply vision measurements.
  prosdk::math::Pose2d visionMeasurement2d = visionMeasurement3d.ToPose2d();

  // Apply vision measurements. For simulation purposes only, we don't input a
  // latency delay -- on a real robot, this must be calculated based either on
  // known latency or timestamps.
  poseEstimator.AddVisionMeasurement(visionMeasurement2d,
                                     prosdk::Timer::GetTimestamp());
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro.
  drivetrainSimulator.SetInputs(prosdk::units::volt_t{leftLeader.GetThrottle()} *
                                    prosdk::RobotController::GetInputVoltage(),
                                prosdk::units::volt_t{rightLeader.GetThrottle()} *
                                    prosdk::RobotController::GetInputVoltage());
  drivetrainSimulator.Update(20_ms);

  leftEncoderSim.SetDistance(drivetrainSimulator.GetLeftPosition().value());
  leftEncoderSim.SetRate(drivetrainSimulator.GetLeftVelocity().value());
  rightEncoderSim.SetDistance(drivetrainSimulator.GetRightPosition().value());
  rightEncoderSim.SetRate(drivetrainSimulator.GetRightVelocity().value());
  // imuSim.SetYaw(-drivetrainSimulator.GetHeading().Degrees());  // no MPU6050 sim
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  fieldSim.SetRobotPose(drivetrainSimulator.GetPose());
  fieldApproximation.SetRobotPose(poseEstimator.GetEstimatedPosition());

  prosdk::telemetry::Log("Field", fieldSim);
  prosdk::telemetry::Log("Approximation", fieldApproximation);
}
