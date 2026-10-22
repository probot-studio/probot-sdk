// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Presets from the manufacturer datasheets: TI INA250 (SBOS511C, Sept 2023) and Allegro
// ACS758xCB. The Boardoza INA250A1 example scales raw ADC counts by a 3.3 V full scale (the
// 12 dB range is about 3.1 V); this class uses the HAL's calibrated volts instead.

#include "prosdk/drivers/analog/AnalogCurrentSensor.hpp"

#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
// INA250A1 gain, V/A (SBOS511C Table 7-1; includes the integrated 2 mOhm shunt).
constexpr double kIna250A1VoltsPerAmp = 0.2;
// ACS758LCB-050B sensitivity at VCC = 5 V, V/A (SensTA, X050B performance characteristics).
constexpr double kAcs758050BVoltsPerAmp = 0.040;
// VCC at which the ACS758 is trimmed; Sens and VIOUT(Q) scale with VCC / kAcs758TrimVcc.
constexpr double kAcs758TrimVcc = 5.0;
// ACS758 supply range, V (common operating characteristics, VCC 3 to 5.5 V).
constexpr double kAcs758MinVcc = 3.0;
constexpr double kAcs758MaxVcc = 5.5;
}  // namespace

AnalogCurrentSensor AnalogCurrentSensor::INA250A1(int adcGpio, double refVolts,
                                                  double dividerRatio) {
  return AnalogCurrentSensor{adcGpio, kIna250A1VoltsPerAmp, refVolts, dividerRatio};
}

AnalogCurrentSensor AnalogCurrentSensor::ACS758LCB050B(int adcGpio, double dividerRatio,
                                                       double vccVolts) {
  // Bidirectional variant: VIOUT(QBI) = VCC / 2. Ratiometric scaling below 4.5 V is unverified.
  if (!(vccVolts >= kAcs758MinVcc && vccVolts <= kAcs758MaxVcc)) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] AnalogCurrentSensor {}: ACS758 VCC {} V is outside "
                           "3.0-5.5 V\n",
                           adcGpio, vccVolts);
  }
  return AnalogCurrentSensor{adcGpio, kAcs758050BVoltsPerAmp * vccVolts / kAcs758TrimVcc,
                             vccVolts / 2.0, dividerRatio};
}

AnalogCurrentSensor::AnalogCurrentSensor(int adcGpio, double voltsPerAmp, double zeroVolts,
                                         double dividerRatio)
    : m_input{adcGpio},
      m_voltsPerAmp{voltsPerAmp},
      m_zeroVolts{std::isfinite(zeroVolts) ? zeroVolts : 0.0},
      m_dividerRatio{dividerRatio} {
  if (!std::isfinite(voltsPerAmp) || voltsPerAmp == 0.0) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] AnalogCurrentSensor {}: invalid sensitivity {} V/A; "
                           "reads 0 A\n",
                           adcGpio, voltsPerAmp);
    m_voltsPerAmp = 0.0;
  }
  if (!std::isfinite(dividerRatio) || dividerRatio < 1.0) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] AnalogCurrentSensor {}: divider ratio {} is not sensor volts "
                           "per ADC volt (>= 1); using 1\n",
                           adcGpio, dividerRatio);
    m_dividerRatio = 1.0;
  }
  util::ReportUsage("IO", adcGpio, "AnalogCurrentSensor");
}

units::volt_t AnalogCurrentSensor::GetVoltage() const {
  return units::volt_t{m_input.GetVoltage() * m_dividerRatio};
}

double AnalogCurrentSensor::AmpsFrom(double sensorVolts) const {
  return m_voltsPerAmp == 0.0 ? 0.0 : (sensorVolts - m_zeroVolts) / m_voltsPerAmp;
}

units::ampere_t AnalogCurrentSensor::GetCurrent() const {
  return units::ampere_t{AmpsFrom(GetVoltage().value())};
}

void AnalogCurrentSensor::CalibrateZero(int samples) {
  if (samples < 1) {
    samples = 1;
  }
  double sum = 0.0;
  for (int i = 0; i < samples; ++i) {
    sum += m_input.GetVoltage();
  }
  m_zeroVolts = sum / samples * m_dividerRatio;
}

units::volt_t AnalogCurrentSensor::GetZeroVoltage() const {
  return units::volt_t{m_zeroVolts};
}

int AnalogCurrentSensor::GetChannel() const {
  return m_input.GetChannel();
}

void AnalogCurrentSensor::LogTo(telemetry::TelemetryTable& table) const {
  double volts = GetVoltage().value();
  table.Log("Value", AmpsFrom(volts));
  table.Log("Voltage", volts);
}

std::string_view AnalogCurrentSensor::GetTelemetryType() const {
  return "Current Sensor";
}
