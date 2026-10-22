// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Analog current sensor with a linear output, I = (Vout - Vzero) / sensitivity, read through a
// prosdk::AnalogInput (ADC1 on the ESP32-S3: GPIO 1..10, about 0..3.1 V). Presets for the TI
// INA250A1 and the Allegro ACS758LCB-050B (Boardoza breakout boards;
// design/boardoza/P14-existing.md).
// An optional resistive divider in front of the ADC pin is undone in software.

#pragma once

#include <string_view>

#include "prosdk/hardware/discrete/AnalogInput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/current.hpp"
#include "prosdk/units/voltage.hpp"

namespace prosdk {

/**
 * Linear analog current sensor.
 *
 * Current = (sensor output volts - zero volts) / volts per amp, where the sensor output volts are
 * the ADC pin volts times the divider ratio. A negative voltsPerAmp reverses the sign (a sensor
 * wired the other way round).
 *
 * The ESP32-S3 pins are not 5 V tolerant and ADC1 reads about 0 to 3.1 V: a sensor whose output
 * can exceed that (ACS758 at VCC = 5 V, INA250 with VS above 3.3 V) needs a divider, whose ratio
 * is passed here.
 */
class AnalogCurrentSensor : public prosdk::telemetry::TelemetryLoggable {
 public:
  /**
   * TI INA250A1 (integrated 2 mOhm shunt): VOUT = ILOAD x 200 mV/A + VREF (TI SBOS511C, section
   * 7.4.1 equation 1; gain from Table 7-1). Positive current flows IN+ -> IN-.
   *
   * The Boardoza board routes REF only to header J1 pin 2 (no on-board tie, from its schematic):
   * wire it to GND (unidirectional, refVolts 0) or to a low-impedance reference (bidirectional).
   * The datasheet asks for a buffered source on REF; a bare resistive divider is not enough.
   *
   * OUT cannot swing below GND + 25 mV typ, 50 mV max (SBOS511C 6.5, RL = 10 kOhm): with REF
   * grounded, currents below about 0.125-0.25 A read high and negative currents read about 0 A.
   * Do not call CalibrateZero() with REF grounded: it would take that swing limit as the zero.
   *
   * @param adcGpio      ADC1 GPIO wired to OUT.
   * @param refVolts     voltage on the REF pin (0 when REF is grounded).
   * @param dividerRatio sensor volts per ADC pin volt (1 without a divider; needed only when VS
   *                     is above 3.3 V).
   */
  static AnalogCurrentSensor INA250A1(int adcGpio, double refVolts, double dividerRatio = 1.0);

  /**
   * Allegro ACS758LCB-050B (+/-50 A, bidirectional): sensitivity 40 mV/A and zero at VCC/2, both
   * ratiometric to VCC (Allegro ACS758 datasheet, X050B performance characteristics and
   * VIOUT(QBI)). Positive current flows IP+ -> IP- (terminal 4 -> 5: output above VCC/2).
   * Ratiometry is specified for VCC = 4.5 to 5.5 V only; for other supplies the preset scales
   * 40 mV/A x VCC / 5 V, which is unverified: call CalibrateZero() and measure the sensitivity
   * (then use the general constructor). A vccVolts outside the 3.0 to 5.5 V supply range prints a
   * warning (volts, not millivolts).
   *
   * @param adcGpio      ADC1 GPIO wired to VIOUT.
   * @param dividerRatio sensor volts per ADC pin volt (at VCC = 5 V the output spans 0.5 to
   *                     4.5 V: use a divider, for example 10 kOhm over 20 kOhm = 1.5).
   * @param vccVolts     sensor supply voltage.
   */
  static AnalogCurrentSensor ACS758LCB050B(int adcGpio, double dividerRatio,
                                           double vccVolts = 5.0);

  /**
   * Constructs a sensor from its transfer function.
   *
   * @param adcGpio      ADC1 GPIO wired to the sensor output (an analog channel on the host sim).
   * @param voltsPerAmp  sensitivity at the sensor output, volts per ampere (not 0).
   * @param zeroVolts    sensor output voltage at 0 A.
   * @param dividerRatio sensor volts per ADC pin volt, (Rtop + Rbottom) / Rbottom; 1 without a
   *                     divider. Values below 1 are not a resistive divider and are refused.
   */
  AnalogCurrentSensor(int adcGpio, double voltsPerAmp, double zeroVolts,
                      double dividerRatio = 1.0);

  AnalogCurrentSensor(AnalogCurrentSensor&&) = default;
  AnalogCurrentSensor& operator=(AnalogCurrentSensor&&) = default;

  /** The current from one ADC reading (0 A when the sensitivity is invalid). */
  prosdk::units::ampere_t GetCurrent() const;

  /** The sensor output voltage from one ADC reading (the divider undone). */
  prosdk::units::volt_t GetVoltage() const;

  /**
   * Sets the zero to the average of `samples` readings (at least 1). Call it while no current
   * flows; it blocks the caller for the reads (back to back, no delay).
   */
  void CalibrateZero(int samples = 100);

  /** The sensor output voltage taken as 0 A (the constructor value or the last CalibrateZero). */
  prosdk::units::volt_t GetZeroVoltage() const;

  /** The analog channel (GPIO on the ESP32-S3). */
  int GetChannel() const;

  /** Logs "Value" (amperes) and "Voltage" (sensor output volts) from one ADC reading. */
  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  /** "Current Sensor". */
  std::string_view GetTelemetryType() const override;

 private:
  double AmpsFrom(double sensorVolts) const;

  AnalogInput m_input;
  double m_voltsPerAmp;
  double m_zeroVolts;
  double m_dividerRatio;
};

}  // namespace prosdk
