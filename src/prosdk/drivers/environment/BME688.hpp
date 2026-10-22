// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Bosch BME688 gas, pressure, humidity and temperature sensor over prosdk::I2C (Boardoza BME688
// board; design/boardoza/P1-climate.md). Forced mode with one heater step: each measurement reads
// temperature, pressure and humidity, then heats the gas sensor hot plate and measures its
// resistance. The compensation is written from the formulas in the data sheet (no Bosch library
// code). Gas resistance is the raw resistance in ohms; air-quality indices (Bosch BSEC) are not
// part of this driver.

#pragma once

#include <stdint.h>

#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/impedance.hpp"
#include "prosdk/units/pressure.hpp"
#include "prosdk/units/temperature.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * BME688 environmental sensor.
 *
 * No thread: call Update() periodically (for example every robot loop). It never waits. The first
 * call checks the chip ID, reads the calibration and triggers a measurement; later calls read the
 * result when the chip flags new data and trigger the next one. With the default heater (320 C for
 * about 150 ms) a sample takes about 0.2 s. After a failure the next attempt is at least 1 s later.
 */
class BME688 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// 7-bit I2C address with SDO high (data sheet 6.2); 0x76 with SDO low.
  static constexpr int kDefaultAddress = 0x77;

  /**
   * Constructs the driver. It does not talk to the chip; the first Update() does.
   *
   * @param port    The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address The 7-bit I2C address.
   */
  explicit BME688(prosdk::I2C::Port port, int address = kDefaultAddress);

  BME688(const BME688&) = delete;
  BME688& operator=(const BME688&) = delete;

  /** Triggers a measurement or reads a finished one (non-blocking). */
  void Update();

  /**
   * Sets the gas sensor heater step used from the next measurement on.
   *
   * @param temperature The hot plate target temperature, limited to 200 to 400 C (the typical range
   *                    in data sheet 3.6).
   * @param duration    The heating time, 1 to 4032 ms; it is stored with 6 bits and a factor of 1,
   *                    4, 16 or 64, so it is rounded to the nearest step.
   */
  void SetHeater(prosdk::units::celsius_t temperature, prosdk::units::millisecond_t duration);

  /** Temperature of the last sample (0 before one). */
  prosdk::units::celsius_t GetTemperature() const;

  /** Pressure of the last sample (0 before one). */
  prosdk::units::pascal_t GetPressure() const;

  /** Relative humidity in percent of the last sample, limited to 0 to 100 (0 before one). */
  double GetHumidity() const;

  /** Gas sensor resistance of the last sample (0 before one). */
  prosdk::units::ohm_t GetGasResistance() const;

  /**
   * True when the last sample's gas measurement is valid and the heater reached its target
   * (gas_valid_r and heat_stab_r, data sheet 3.7).
   */
  bool IsGasValid() const;

  /** True once a sample exists. */
  bool HasData() const;

  /** True when the last transfer succeeded and the chip ID matched. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

  /// Calibration parameters (data sheet Tables 13 to 16).
  struct Calibration {
    uint16_t t1;
    int16_t t2;
    int8_t t3;
    uint16_t p1;
    int16_t p2;
    int8_t p3;
    int16_t p4;
    int16_t p5;
    int8_t p6;
    int8_t p7;
    int16_t p8;
    int16_t p9;
    uint8_t p10;
    uint16_t h1;
    uint16_t h2;
    int8_t h3;
    int8_t h4;
    int8_t h5;
    uint8_t h6;
    int8_t h7;
    int8_t g1;
    int16_t g2;
    int8_t g3;
    uint8_t resHeatRange;
    int8_t resHeatVal;
  };

  /** Compensated temperature in C (data sheet 3.6, floating point); sets tFine for the others. */
  static double CompensateTemperature(const Calibration& cal, uint32_t adc, double& tFine);

  /** Compensated pressure in Pa (data sheet 3.6, floating point). */
  static double CompensatePressure(const Calibration& cal, uint32_t adc, double tFine);

  /** Compensated relative humidity in percent, not limited (data sheet 3.6, floating point). */
  static double CompensateHumidity(const Calibration& cal, uint32_t adc, double temperature);

  /** Gas resistance in ohms (data sheet 3.7). */
  static double GasResistance(uint32_t adc, int range);

  /** res_heat_x register code for a target and ambient temperature in C (data sheet 3.6). */
  static uint8_t HeaterCode(const Calibration& cal, double target, double ambient);

 private:
  enum class State { kStart, kMeasuring };

  bool Trigger(double now);
  void Fail(double now);

  prosdk::I2C m_i2c;
  int m_address;
  mutable std::mutex m_mutex;
  State m_state = State::kStart;
  double m_stateTime = 0.0;  // s, monotonic; when the current state began
  bool m_everTried = false;
  bool m_warnedId = false;
  bool m_connected = false;
  bool m_hasData = false;
  Calibration m_cal{};
  double m_heaterTarget = 320.0;  // C
  uint8_t m_gasWait = 0;          // gas_wait_0 code
  int m_heaterCode = -1;          // res_heat_0 last written; -1: write on the next trigger
  bool m_gasWaitDirty = true;
  double m_resultTimeout = 0.0;   // s after a trigger without new data before the next trigger
  double m_temperature = 0.0;     // C
  double m_pressure = 0.0;        // Pa
  double m_humidity = 0.0;        // %RH
  double m_gasResistance = 0.0;   // ohm
  bool m_gasValid = false;
};

}  // namespace prosdk
