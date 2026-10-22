// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Bosch BNO055 absolute orientation sensor over prosdk::I2C (design/boardoza/P3-imu.md), with the
// getters of prosdk::OnboardIMU. The chip runs the sensor fusion itself: yaw is its fused heading,
// negated (the BNO055 heading turns clockwise-positive), and roll and pitch come from its fused
// gravity vector, so neither needs a host filter or a bias calibration.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
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
 * BNO055 9-axis sensor with on-chip fusion.
 *
 * Mounting: the chip flat, its Z axis up; X forward and Y left give the WPILib robot axes
 * (counter-clockwise positive yaw). The chip's own axis remap (AXIS_MAP_CONFIG) is left at its
 * default. Units on the chip: m/s^2, deg/s, degrees, deg C, Windows orientation format.
 *
 * Yaw is continuous (it does not wrap at +/-180 degrees). In the absolute modes (kNDOF,
 * kNDOFFmcOff, kCompass; datasheet Table 3-3) the first yaw is the heading from magnetic north,
 * CCW-positive; in the relative modes (kIMU, kM4G) it starts near 0. ResetYaw() makes the current
 * heading 0 in every mode.
 *
 * If the chip does not answer yet (it needs up to 650 ms after power-on), the configuration is
 * retried from Update() every 100 ms. A background thread reads the sensor every updatePeriod;
 * with an updatePeriod of 0 call Update() yourself (for example every robot loop).
 */
class BNO055 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Fusion operating modes (OPR_MODE values, datasheet 3.3).
  enum class Mode : uint8_t {
    kIMU = 0x08,         ///< accel + gyro, relative heading (no magnetometer)
    kCompass = 0x09,     ///< accel + mag, absolute heading, no gyro
    kM4G = 0x0A,         ///< accel + mag, the magnetometer replaces the gyro; relative heading
    kNDOFFmcOff = 0x0B,  ///< all three, without the fast magnetometer calibration
    kNDOF = 0x0C,        ///< all three, absolute heading
  };

  /// Calibration levels from CALIB_STAT, 0 (not calibrated) to 3 (fully calibrated).
  struct CalibrationStatus {
    int system = 0;
    int gyro = 0;
    int accel = 0;
    int mag = 0;
  };

  /// The calibration profile: registers 0x55 to 0x6A (accel, mag and gyro offsets, then the
  /// accel and mag radius), little-endian as the chip stores them.
  using Offsets = std::array<uint8_t, 22>;

  /// Default 7-bit I2C address (COM3/ADR low, as on the Boardoza board; 0x29 with ADR high).
  static constexpr int kDefaultAddress = 0x28;
  /// Default period of the reader thread (the fusion output rate is 100 Hz in kIMU and kNDOF).
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{10};

  /**
   * Constructs the driver and configures the chip. Blocks about 30 ms, or up to 1 s while the
   * chip starts the external crystal for the first time after power-on.
   *
   * @param port            The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address         The 7-bit I2C address.
   * @param mode            The fusion mode.
   * @param updatePeriod    Period of the reader thread; 0 starts no thread (call Update()).
   * @param externalCrystal Selects the 32.768 kHz crystal (CLK_SEL). The datasheet allows it only
   *                        when a crystal is fitted; the Boardoza board has one.
   */
  explicit BNO055(prosdk::I2C::Port port, int address = kDefaultAddress, Mode mode = Mode::kNDOF,
                  prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod,
                  bool externalCrystal = true);
  ~BNO055() override;

  BNO055(const BNO055&) = delete;
  BNO055& operator=(const BNO055&) = delete;

  /**
   * Reads the sensor once (what the reader thread does). While the chip is not configured it
   * retries the configuration at most every 100 ms; a retry that finds the chip blocks about
   * 30 ms (up to 1 s on the first crystal start), so with an updatePeriod of 0 it can stall the
   * caller's loop once.
   */
  void Update();

  /** True when the chip is configured and the last read succeeded. */
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

  /** Roll (rotation about X), from the fused gravity vector. */
  prosdk::units::radian_t GetAngleX();
  /** Pitch (rotation about Y), from the fused gravity vector. */
  prosdk::units::radian_t GetAngleY();
  /** Yaw (rotation about Z), same as GetYaw(). */
  prosdk::units::radian_t GetAngleZ();

  /** Angular rates (offset-compensated by the chip; 0 in kCompass and kM4G, which have no gyro). */
  prosdk::units::radians_per_second_t GetGyroRateX();
  prosdk::units::radians_per_second_t GetGyroRateY();
  prosdk::units::radians_per_second_t GetGyroRateZ();

  /** Accelerations (gravity included: about +9.81 m/s^2 on Z when flat). */
  prosdk::units::meters_per_second_squared_t GetAccelX();
  prosdk::units::meters_per_second_squared_t GetAccelY();
  prosdk::units::meters_per_second_squared_t GetAccelZ();

  /** Chip temperature (1 deg C resolution). */
  prosdk::units::celsius_t GetTemperature();

  /** The chip's calibration levels as of the last read. */
  CalibrationStatus GetCalibrationStatus();

  /**
   * Reads the calibration profile (switches to CONFIG mode for about 30 ms, then back). The
   * datasheet allows it only after a full calibration (GetCalibrationStatus() all 3). Returns
   * nullopt if the chip did not answer.
   */
  std::optional<Offsets> GetOffsets();

  /**
   * Writes a calibration profile saved earlier with GetOffsets() (CONFIG mode for about 30 ms).
   * Returns false if the chip did not answer. The chip overwrites it when it recalibrates itself.
   */
  bool SetOffsets(const Offsets& offsets);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Configure();                    // m_ioMutex held
  bool EnterMode(uint8_t mode);        // m_ioMutex held; waits the switching time
  void Run();

  prosdk::I2C m_i2c;
  Mode m_mode;
  bool m_externalCrystal;
  prosdk::units::millisecond_t m_updatePeriod;

  std::mutex m_ioMutex;  // one multi-transfer sequence (configuration, profile) at a time
  std::atomic<bool> m_configured{false};
  double m_lastAttempt = 0.0;  // s, monotonic; time of the last configuration attempt

  mutable std::mutex m_mutex;
  bool m_lastReadOk = false;
  bool m_yawInitialized = false;
  bool m_haveHeading = false;  // false: the next heading is a new reference (no yaw jump)
  double m_lastHeading = 0.0;  // rad, CCW-positive, wrapped
  double m_yaw = 0.0;          // rad, continuous
  double m_yawOffset = 0.0;
  double m_roll = 0.0, m_pitch = 0.0;  // rad
  double m_ax = 0.0, m_ay = 0.0, m_az = 0.0;  // m/s^2
  double m_rateX = 0.0, m_rateY = 0.0, m_rateZ = 0.0;  // rad/s
  double m_tempC = 0.0;
  uint8_t m_calibStat = 0;

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
