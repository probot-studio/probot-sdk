// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// InvenSense MPU-6050 6-axis IMU over prosdk::I2C (design/drivers.md, "MPU6050"), with the
// getters of prosdk::OnboardIMU. Yaw is the integrated, bias-corrected Z rate (it drifts slowly:
// the chip has no magnetometer); roll and pitch come from a complementary filter of the gyro
// rates and the accelerometer's gravity vector.

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
#include "prosdk/units/angular_velocity.hpp"
#include "prosdk/units/temperature.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * MPU-6050 gyro and accelerometer.
 *
 * Mounting: the chip flat, its Z axis up; X forward and Y left give the WPILib robot axes
 * (counter-clockwise positive yaw). Ranges: +/-1000 deg/s and +/-4 g, 44 Hz digital low-pass
 * filter, 1 kHz internal sample rate.
 *
 * A background thread reads the sensor every updatePeriod and integrates; with an updatePeriod
 * of 0 call Update() yourself (for example every robot loop).
 */
class MPU6050 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (AD0 low; 0x69 with AD0 high).
  static constexpr int kDefaultAddress = 0x68;
  /// Default period of the reader thread (200 Hz).
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{5};

  /**
   * Constructs the driver, wakes the chip and configures its ranges. Does not calibrate.
   *
   * @param port         The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address      The 7-bit I2C address.
   * @param updatePeriod Period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit MPU6050(prosdk::I2C::Port port, int address = kDefaultAddress,
                   prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~MPU6050() override;

  MPU6050(const MPU6050&) = delete;
  MPU6050& operator=(const MPU6050&) = delete;

  /**
   * Measures the gyro bias: averages `samples` readings about 2 ms apart (1 s by default). The
   * robot must not move. Blocks the caller (and the reader thread) meanwhile; call it once at
   * startup. Returns false if the sensor did not answer.
   */
  bool Calibrate(int samples = 500);

  /** Reads the sensor once and integrates (what the reader thread does). */
  void Update();

  /** True when the last read succeeded and WHO_AM_I matched at construction. */
  bool IsConnected() const;

  /** Yaw (counter-clockwise positive), relative to the last ResetYaw(). */
  prosdk::units::radian_t GetYaw();
  /** Makes the current heading yaw 0. */
  void ResetYaw();
  /** The yaw as a Rotation2d. */
  prosdk::math::Rotation2d GetRotation2d();
  /** Roll, pitch and yaw. */
  prosdk::math::Rotation3d GetRotation3d();
  /** GetRotation3d() as a quaternion. */
  prosdk::math::Quaternion GetQuaternion();

  /** Roll (rotation about X). */
  prosdk::units::radian_t GetAngleX();
  /** Pitch (rotation about Y). */
  prosdk::units::radian_t GetAngleY();
  /** Yaw (rotation about Z), same as GetYaw(). */
  prosdk::units::radian_t GetAngleZ();

  /** Bias-corrected angular rates. */
  prosdk::units::radians_per_second_t GetGyroRateX();
  prosdk::units::radians_per_second_t GetGyroRateY();
  prosdk::units::radians_per_second_t GetGyroRateZ();

  /** Accelerations (gravity included: about +9.81 m/s^2 on Z when flat). */
  prosdk::units::meters_per_second_squared_t GetAccelX();
  prosdk::units::meters_per_second_squared_t GetAccelY();
  prosdk::units::meters_per_second_squared_t GetAccelZ();

  /** Die temperature. */
  prosdk::units::celsius_t GetTemperature();

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  struct Raw {
    double ax, ay, az;  // g
    double gx, gy, gz;  // deg/s
    double tempC;
  };
  bool ReadRaw(Raw* raw);
  void Run();

  prosdk::I2C m_i2c;
  prosdk::units::millisecond_t m_updatePeriod;

  mutable std::mutex m_mutex;
  bool m_whoAmIOk = false;
  bool m_lastReadOk = false;
  bool m_haveSample = false;
  double m_lastTime = 0.0;  // s, monotonic
  double m_biasX = 0.0, m_biasY = 0.0, m_biasZ = 0.0;  // deg/s
  Raw m_raw{};
  double m_rateX = 0.0, m_rateY = 0.0, m_rateZ = 0.0;  // rad/s, bias-corrected
  double m_roll = 0.0, m_pitch = 0.0, m_yaw = 0.0;     // rad
  double m_yawOffset = 0.0;

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
