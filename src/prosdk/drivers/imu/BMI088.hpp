// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Bosch BMI088 6-axis IMU over prosdk::I2C (design/boardoza/P3-imu.md), with the getters of
// prosdk::OnboardIMU. The chip holds two I2C devices, an accelerometer and a gyroscope, each with
// its own address. As in prosdk::MPU6050, yaw is the integrated, bias-corrected Z rate (it drifts
// slowly: there is no magnetometer); roll and pitch come from a complementary filter of the gyro
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
 * BMI088 gyro and accelerometer (I2C mode: PS pin high; on the Boardoza board JP1 shorted).
 *
 * Mounting: the chip flat, its Z axis up; X forward and Y left give the WPILib robot axes
 * (counter-clockwise positive yaw). Ranges: +/-1000 deg/s (400 Hz output, 47 Hz filter) and
 * +/-6 g (100 Hz output, normal filter).
 *
 * A background thread reads the sensor every updatePeriod and integrates; with an updatePeriod
 * of 0 call Update() yourself (for example every robot loop).
 */
class BMI088 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default accelerometer address (SDO1 high; 0x18 with SDO1 low, Boardoza JP2 shorted).
  static constexpr int kDefaultAccelAddress = 0x19;
  /// Default gyroscope address (SDO2 high; 0x68 with SDO2 low, Boardoza JP2 shorted).
  static constexpr int kDefaultGyroAddress = 0x69;
  /// Default period of the reader thread (200 Hz).
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{5};

  /**
   * Constructs the driver, switches the accelerometer on and configures both ranges. Does not
   * calibrate.
   *
   * @param port         The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param accelAddress The accelerometer's 7-bit I2C address.
   * @param gyroAddress  The gyroscope's 7-bit I2C address.
   * @param updatePeriod Period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit BMI088(prosdk::I2C::Port port, int accelAddress = kDefaultAccelAddress,
                  int gyroAddress = kDefaultGyroAddress,
                  prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~BMI088() override;

  BMI088(const BMI088&) = delete;
  BMI088& operator=(const BMI088&) = delete;

  /**
   * Measures the gyro bias: averages `samples` readings about 2 ms apart (1 s by default). The
   * robot must not move. Blocks the caller (and the reader thread) meanwhile; call it once at
   * startup. Returns false if the sensor did not answer.
   */
  bool Calibrate(int samples = 500);

  /** Reads the sensor once and integrates (what the reader thread does). */
  void Update();

  /** True when the last read succeeded and both chip IDs matched at construction. */
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

  /** Die temperature (0.125 deg C resolution, updated by the chip every 1.28 s). */
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

  prosdk::I2C m_accel;
  prosdk::I2C m_gyro;
  prosdk::units::millisecond_t m_updatePeriod;

  mutable std::mutex m_mutex;
  bool m_chipIdOk = false;
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
