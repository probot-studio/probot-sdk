// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ST LSM303AGR accelerometer and magnetometer (e-compass) over prosdk::I2C
// (design/boardoza/P3-imu.md), with the orientation getters of prosdk::OnboardIMU. There is no
// gyro: yaw is the tilt-compensated magnetic heading, and roll and pitch come from the
// accelerometer's gravity vector. Motors, magnets and steel near the chip bend the field, so the
// heading needs a hard-iron offset measured on the robot (SetHardIronOffset).

#pragma once

#include <atomic>
#include <mutex>
#include <string_view>
#include <thread>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/math/geometry/Quaternion.hpp"
#include "prosdk/math/geometry/Rotation2d.hpp"
#include "prosdk/math/geometry/Rotation3d.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/acceleration.hpp"
#include "prosdk/units/angle.hpp"
#include "prosdk/units/magnetic_field_strength.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * LSM303AGR e-compass (accelerometer at 0x19, magnetometer at 0x1E; the addresses are fixed, so
 * one chip per I2C port).
 *
 * Mounting: the chip flat, its Z axis up; X forward and Y left give the WPILib robot axes
 * (counter-clockwise positive yaw); that the chip's axes are right-handed is unverified (datasheet
 * Figure 2 is ambiguous, see design/boardoza/P3-imu.md). Accelerometer +/-4 g, high-resolution
 * mode, 100 Hz; magnetometer 100 Hz, high-resolution, offset cancellation and low-pass filter on.
 *
 * GetHeading() is the absolute heading from magnetic north (CCW-positive, -pi to pi). GetYaw()
 * is the same heading made continuous (no wrap at +/-180 degrees) and relative to ResetYaw().
 *
 * A background thread reads the sensor every updatePeriod; with an updatePeriod of 0 call
 * Update() yourself (for example every robot loop).
 */
class LSM303AGR : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Accelerometer 7-bit I2C address (fixed).
  static constexpr int kAccelAddress = 0x19;
  /// Magnetometer 7-bit I2C address (fixed).
  static constexpr int kMagAddress = 0x1E;
  /// Default period of the reader thread (both sensors output 100 Hz).
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{10};

  /**
   * Constructs the driver and switches both sensors on (they are off after power-on).
   *
   * @param port         The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param updatePeriod Period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit LSM303AGR(prosdk::I2C::Port port,
                     prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~LSM303AGR() override;

  LSM303AGR(const LSM303AGR&) = delete;
  LSM303AGR& operator=(const LSM303AGR&) = delete;

  /** Reads both sensors once (what the reader thread does). */
  void Update();

  /** True when the last read succeeded and both WHO_AM_I values matched at construction. */
  bool IsConnected() const;

  /** Tilt-compensated magnetic heading, CCW-positive from magnetic north, -pi to pi. */
  prosdk::units::radian_t GetHeading();

  /** Yaw: the continuous heading (counter-clockwise positive), relative to the last ResetYaw(). */
  prosdk::units::radian_t GetYaw();
  /** Makes the current heading yaw 0. */
  void ResetYaw();
  /** The yaw as a Rotation2d. */
  prosdk::math::Rotation2d GetRotation2d();
  /** Roll, pitch and yaw. */
  prosdk::math::Rotation3d GetRotation3d();
  /** GetRotation3d() as a quaternion. */
  prosdk::math::Quaternion GetQuaternion();

  /** Roll (rotation about X), from the accelerometer. */
  prosdk::units::radian_t GetAngleX();
  /** Pitch (rotation about Y), from the accelerometer. */
  prosdk::units::radian_t GetAngleY();
  /** Yaw (rotation about Z), same as GetYaw(). */
  prosdk::units::radian_t GetAngleZ();

  /** Accelerations (gravity included: about +9.81 m/s^2 on Z when flat). */
  prosdk::units::meters_per_second_squared_t GetAccelX();
  prosdk::units::meters_per_second_squared_t GetAccelY();
  prosdk::units::meters_per_second_squared_t GetAccelZ();

  /** Magnetic field, hard-iron offset subtracted. */
  prosdk::units::tesla_t GetMagX();
  prosdk::units::tesla_t GetMagY();
  prosdk::units::tesla_t GetMagZ();

  /**
   * Sets the hard-iron offset subtracted from every magnetometer reading (default 0). Measure it
   * on the finished robot: with the offset at 0, turn the robot through every orientation and
   * take (max + min) / 2 of GetMagX/Y/Z on each axis.
   */
  void SetHardIronOffset(prosdk::units::tesla_t x, prosdk::units::tesla_t y,
                         prosdk::units::tesla_t z);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Run();

  prosdk::I2C m_accel;
  prosdk::I2C m_mag;
  prosdk::units::millisecond_t m_updatePeriod;

  mutable std::mutex m_mutex;
  bool m_whoAmIOk = false;
  bool m_lastReadOk = false;
  double m_ax = 0.0, m_ay = 0.0, m_az = 0.0;  // m/s^2
  double m_mx = 0.0, m_my = 0.0, m_mz = 0.0;  // T, offset subtracted
  double m_offsetX = 0.0, m_offsetY = 0.0, m_offsetZ = 0.0;  // T
  bool m_haveHeading = false;
  double m_heading = 0.0;  // rad, CCW-positive, -pi to pi
  double m_yaw = 0.0;      // rad, continuous
  double m_yawOffset = 0.0;
  double m_roll = 0.0, m_pitch = 0.0;  // rad

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
