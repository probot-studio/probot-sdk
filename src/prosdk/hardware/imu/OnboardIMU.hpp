// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "prosdk/math/geometry/Quaternion.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/angular_velocity.hpp"

namespace prosdk {

/**
 * Systemcore onboard IMU
 */
class OnboardIMU {
 public:
  /**
   * A mount orientation of Systemcore
   */
  enum MountOrientation {
    /** Flat (mounted parallel to the ground). */
    FLAT,
    /** Landscape (vertically mounted with long edge of Systemcore parallel to
       the ground). */
    LANDSCAPE,
    /** Portrait (vertically mounted with the short edge of Systemcore parallel
       to the ground). */
    PORTRAIT
  };

  /**
   * Constructs a handle to the Systemcore onboard IMU.
   * @param mountOrientation the mount orientation of Systemcore to determine
   * yaw.
   */
  explicit OnboardIMU(MountOrientation mountOrientation);

  /**
   * Get the yaw value
   * @return yaw value
   * @Common This is one of the commonly used methods for this class
   */
  prosdk::units::radian_t GetYaw();

  /**
   * Reset the current yaw value to 0. Future reads of the yaw value will be
   * relative to the current orientation.
   *
   * @Common This is one of the commonly used methods for this class
   */
  void ResetYaw();

  /**
   * Get the yaw as a prosdk::math::Rotation2d.
   * @return yaw
   */
  prosdk::math::Rotation2d GetRotation2d();

  /**
   * Get the 3D orientation as a prosdk::math::Rotation3d.
   * @return 3D orientation
   */
  prosdk::math::Rotation3d GetRotation3d();

  /**
   * Get the 3D orientation as a prosdk::math::Quaternion.
   * @return 3D orientation
   */
  prosdk::math::Quaternion GetQuaternion();

  /**
   * Get the angle about the X axis of the IMU.
   * @return angle about the X axis
   */
  prosdk::units::radian_t GetAngleX();

  /**
   * Get the angle about the Y axis of the IMU.
   * @return angle about the Y axis
   */
  prosdk::units::radian_t GetAngleY();

  /**
   * Get the angle about the Z axis of the IMU.
   * @return angle about the Z axis
   */
  prosdk::units::radian_t GetAngleZ();

  /**
   * Get the angular rate about the X axis of the IMU.
   * @return angular rate about the X axis
   */
  prosdk::units::radians_per_second_t GetGyroRateX();

  /**
   * Get the angular rate about the Y axis of the IMU.
   * @return angular rate about the Y axis
   */
  prosdk::units::radians_per_second_t GetGyroRateY();

  /**
   * Get the angular rate about the Z axis of the IMU.
   * @return angular rate about the Z axis
   */
  prosdk::units::radians_per_second_t GetGyroRateZ();

  /**
   * Get the acceleration along the X axis of the IMU.
   * @return acceleration along the X axis
   */
  prosdk::units::meters_per_second_squared_t GetAccelX();

  /**
   * Get the acceleration along the Z axis of the IMU.
   * @return acceleration along the Z axis
   */
  prosdk::units::meters_per_second_squared_t GetAccelY();

  /**
   * Get the acceleration along the Z axis of the IMU.
   * @return acceleration along the Z axis
   */
  prosdk::units::meters_per_second_squared_t GetAccelZ();

 private:
  prosdk::units::radian_t GetYawNoOffset();
  MountOrientation m_mountOrientation;
  prosdk::units::radian_t m_yawOffset{0};
};
}  // namespace wpi
