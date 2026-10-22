// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "prosdk/hal/SimDevice.hpp"
#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/struct/Struct.hpp"

namespace prosdk {

/**
 * ADXL345 Accelerometer on I2C.
 *
 * This class allows access to a Analog Devices ADXL345 3-axis accelerometer on
 * an I2C bus. This class assumes the default (not alternate) sensor address of
 * 0x1D (7-bit address).
 */
class ADXL345_I2C : public prosdk::telemetry::TelemetryLoggable {
 public:
  /**
   * Accelerometer axes.
   */
  enum class Axis {
    /// X axis.
    X = 0x00,
    /// Y axis.
    Y = 0x02,
    /// Z axis.
    Z = 0x04
  };

  /**
   * Container type for accelerations from all axes.
   */
  struct AllAxes {
    /// Acceleration along the X axis in g-forces.
    double x = 0.0;
    /// Acceleration along the Y axis in g-forces.
    double y = 0.0;
    /// Acceleration along the Z axis in g-forces.
    double z = 0.0;
  };

  /// Default I2C device address.
  static constexpr int DEFAULT_ADDRESS = 0x1D;

  /**
   * Constructs the ADXL345 Accelerometer over I2C.
   *
   * @param port          The I2C port the accelerometer is attached to
   * @param range         The range (+ or -) that the accelerometer will
   *                      measure; valid values are 2, 4, 8, or 16 Gs. The
   *                      default is 2 Gs.
   * @param deviceAddress The I2C address of the accelerometer (0x1D or 0x53)
   */
  explicit ADXL345_I2C(I2C::Port port, int range = 2,
                       int deviceAddress = DEFAULT_ADDRESS);
  ~ADXL345_I2C() override = default;

  ADXL345_I2C(ADXL345_I2C&&) = default;
  ADXL345_I2C& operator=(ADXL345_I2C&&) = default;

  I2C::Port GetI2CPort() const;
  int GetI2CDeviceAddress() const;

  /**
   * Set the measuring range of the accelerometer.
   *
   * @param range The maximum acceleration, positive or negative, that the
   *     accelerometer will measure. Valid values are 2, 4, 8, or 16 Gs.
   */
  void SetRange(int range);

  /**
   * Returns the acceleration along the X axis in g-forces.
   *
   * @return The acceleration along the X axis in g-forces.
   */
  double GetX();

  /**
   * Returns the acceleration along the Y axis in g-forces.
   *
   * @return The acceleration along the Y axis in g-forces.
   */
  double GetY();

  /**
   * Returns the acceleration along the Z axis in g-forces.
   *
   * @return The acceleration along the Z axis in g-forces.
   */
  double GetZ();

  /**
   * Get the acceleration of one axis in Gs.
   *
   * @param axis The axis to read from.
   * @return Acceleration of the ADXL345 in Gs.
   * @Common This is one of the commonly used methods for this class
   */
  virtual double GetAcceleration(Axis axis) const;

  /**
   * Get the acceleration of all axes in Gs.
   *
   * @return An object containing the acceleration measured on each axis of the
   *         ADXL345 in Gs.
   */
  virtual AllAxes GetAccelerations() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;

  std::string_view GetTelemetryType() const override;

 private:
  mutable I2C m_i2c;

  prosdk::hal::SimDevice m_simDevice;
  prosdk::hal::SimEnum m_simRange;
  prosdk::hal::SimDouble m_simX;
  prosdk::hal::SimDouble m_simY;
  prosdk::hal::SimDouble m_simZ;
};

}  // namespace wpi

template <>
struct prosdk::util::Struct<prosdk::ADXL345_I2C::AllAxes> {
  static constexpr std::string_view GetTypeName() {
    return "ADXL345_I2C.AllAxes";
  }
  static constexpr size_t GetSize() { return 8 * 3; }
  static constexpr std::string_view GetSchema() {
    return "double x;double y;double z";
  }

  static prosdk::ADXL345_I2C::AllAxes Unpack(std::span<const uint8_t> data);
  static void Pack(std::span<uint8_t> data,
                   const prosdk::ADXL345_I2C::AllAxes& value);
};

static_assert(prosdk::util::StructSerializable<prosdk::ADXL345_I2C::AllAxes>);
