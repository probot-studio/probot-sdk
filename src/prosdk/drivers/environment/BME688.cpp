// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// BME688 driver (BME688.hpp), written from the Bosch Sensortec BME688 data sheet
// BST-BME688-DS000-03 revision 1.3 (no Bosch library code): 3.4 (modes; a mode write during a
// measurement waits for its end), 3.5 (forced mode quick start and its default settings), 3.6
// (floating point compensation of temperature, pressure and humidity, heater resistance code;
// Tables 13 to 16 give the calibration addresses), 3.7 (gas resistance, Table 17; new_data,
// gas_valid_r and heat_stab_r), 5.2 (memory map), 5.3 (register fields) and 6.2 (I2C address).
// Two things the data sheet leaves open: the signedness of each calibration parameter is
// unverified against the data sheet (taken from the parameter types declared in Bosch's public
// BME68x API, used as a reference only, no code copied), and the pressure formula's var1_p /
// var2_p / var3_p are read as var1 / var2 / var3 (the data sheet's integer formula gives the same
// result to within 2 Pa on the test vector).

#include "prosdk/drivers/environment/BME688.hpp"

#include <algorithm>
#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegResHeatVal = 0x00;    // signed
constexpr int kRegResHeatRange = 0x02;  // bits 5:4
constexpr int kRegField0 = 0x1D;        // meas_status_0 .. 0x2D, 17 bytes
constexpr int kRegResHeat0 = 0x5A;
constexpr int kRegGasWait0 = 0x64;
constexpr int kRegCtrlGas0 = 0x70;  // bit 3 heat_off
constexpr int kRegCtrlGas1 = 0x71;  // bit 5 run_gas, bits 3:0 nb_conv
constexpr int kRegCtrlHum = 0x72;   // bits 2:0 osrs_h
constexpr int kRegCtrlMeas = 0x74;  // osrs_t 7:5, osrs_p 4:2, mode 1:0
constexpr int kRegConfig = 0x75;    // bits 4:2 filter
constexpr int kRegCalA = 0x8A;      // 0x8A..0xA0, 23 bytes
constexpr int kRegChipId = 0xD0;
constexpr int kRegCalB = 0xE1;  // 0xE1..0xEE, 14 bytes
constexpr int kRegVariantId = 0xF0;

constexpr uint8_t kChipId = 0x61;
constexpr uint8_t kVariantBme688 = 0x01;

// data sheet 3.5 quick start: humidity 1x, temperature 2x, pressure 16x, IIR filter off
constexpr uint8_t kCtrlHum = 0x01;
constexpr uint8_t kCtrlMeasForced = (0x2 << 5) | (0x5 << 2) | 0x01;  // 0x55, mode 01 = forced
constexpr uint8_t kConfig = 0x00;
constexpr uint8_t kCtrlGas0 = 0x00;  // heater on
constexpr uint8_t kCtrlGas1 = 0x20;  // run_gas, heater step 0

constexpr uint8_t kNewData = 0x80;
constexpr uint8_t kGasValid = 0x20;
constexpr uint8_t kHeatStab = 0x10;

constexpr double kRetryTime = 1.0;    // s between attempts after a failure
constexpr double kResultMargin = 1.0;  // s past the heating time without new data: trigger again
constexpr double kDefaultAmbient = 25.0;  // C, for the heater code before the first sample

// gas_wait_x: 6-bit count of 1 ms steps times 1, 4, 16 or 64 (data sheet 5.3.4.3)
uint8_t EncodeGasWait(double ms) {
  ms = std::clamp(ms, 1.0, 63.0 * 64.0);
  for (int factor = 0; factor < 4; ++factor) {
    double step = 1 << (2 * factor);
    if (ms / step <= 63.0) {
      int count = std::clamp(static_cast<int>(std::lround(ms / step)), 1, 63);
      return static_cast<uint8_t>((factor << 6) | count);
    }
  }
  return 0xFF;
}

// gas_wait_x code back to milliseconds (the time the chip actually heats)
double DecodeGasWait(uint8_t code) {
  return (code & 0x3F) * static_cast<double>(1 << (2 * (code >> 6)));
}

uint16_t U16(const uint8_t* lsb) {
  return static_cast<uint16_t>(lsb[0] | (lsb[1] << 8));
}

}  // namespace

BME688::BME688(I2C::Port port, int address)
    : m_i2c{port, address}, m_address{address}, m_gasWait{EncodeGasWait(150.0)} {
  util::ReportUsage("BME688", address, "");
}

double BME688::CompensateTemperature(const Calibration& cal, uint32_t adc, double& tFine) {
  double var1 = (adc / 16384.0 - cal.t1 / 1024.0) * cal.t2;
  double x = adc / 131072.0 - cal.t1 / 8192.0;
  double var2 = x * x * (cal.t3 * 16.0);
  tFine = var1 + var2;
  return tFine / 5120.0;
}

double BME688::CompensatePressure(const Calibration& cal, uint32_t adc, double tFine) {
  double var1 = tFine / 2.0 - 64000.0;
  double var2 = var1 * var1 * (cal.p6 / 131072.0);
  var2 = var2 + var1 * cal.p5 * 2.0;
  var2 = var2 / 4.0 + cal.p4 * 65536.0;
  var1 = (cal.p3 * var1 * var1 / 16384.0 + cal.p2 * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * cal.p1;
  if (var1 == 0.0) {
    return 0.0;  // no calibration (all zero): avoid dividing by zero
  }
  double p = 1048576.0 - adc;
  p = (p - var2 / 4096.0) * 6250.0 / var1;
  var1 = cal.p9 * p * p / 2147483648.0;
  var2 = p * (cal.p8 / 32768.0);
  double scaled = p / 256.0;
  double var3 = scaled * scaled * scaled * (cal.p10 / 131072.0);
  return p + (var1 + var2 + var3 + cal.p7 * 128.0) / 16.0;
}

double BME688::CompensateHumidity(const Calibration& cal, uint32_t adc, double temperature) {
  double var1 = adc - (cal.h1 * 16.0 + (cal.h3 / 2.0) * temperature);
  double var2 = var1 * ((cal.h2 / 262144.0) * (1.0 + (cal.h4 / 16384.0) * temperature +
                                               (cal.h5 / 1048576.0) * temperature * temperature));
  double var3 = cal.h6 / 16384.0;
  double var4 = cal.h7 / 2097152.0;
  return var2 + (var3 + var4 * temperature) * var2 * var2;
}

double BME688::GasResistance(uint32_t adc, int range) {
  double var1 = static_cast<double>(262144u >> range);
  double var2 = 4096.0 + 3.0 * (static_cast<double>(adc) - 512.0);
  return 1000000.0 * var1 / var2;
}

uint8_t BME688::HeaterCode(const Calibration& cal, double target, double ambient) {
  double var1 = cal.g1 / 16.0 + 49.0;
  double var2 = (cal.g2 / 32768.0) * 0.0005 + 0.00235;
  double var3 = cal.g3 / 1024.0;
  double var4 = var1 * (1.0 + var2 * target);
  double var5 = var4 + var3 * ambient;
  double code = 3.4 * (var5 * (4.0 / (4.0 + cal.resHeatRange)) *
                           (1.0 / (1.0 + cal.resHeatVal * 0.002)) -
                       25.0);
  return static_cast<uint8_t>(std::clamp(static_cast<int>(code), 0, 255));
}

void BME688::SetHeater(units::celsius_t temperature, units::millisecond_t duration) {
  std::scoped_lock lock{m_mutex};
  m_heaterTarget = std::clamp(temperature.value(), 200.0, 400.0);
  m_gasWait = EncodeGasWait(duration.value());
  m_gasWaitDirty = true;
  m_heaterCode = -1;
}

// m_mutex held.
void BME688::Fail(double now) {
  m_connected = false;
  m_state = State::kStart;
  m_stateTime = now;
}

// m_mutex held. Writes changed heater settings and starts one forced measurement; true = failed.
bool BME688::Trigger(double now) {
  double ambient = m_hasData ? m_temperature : kDefaultAmbient;
  int code = HeaterCode(m_cal, m_heaterTarget, ambient);
  if (m_gasWaitDirty) {
    if (m_i2c.Write(kRegGasWait0, m_gasWait)) {
      return true;
    }
    m_gasWaitDirty = false;
  }
  if (code != m_heaterCode) {
    if (m_i2c.Write(kRegResHeat0, static_cast<uint8_t>(code))) {
      return true;
    }
    m_heaterCode = code;
  }
  if (m_i2c.Write(kRegCtrlMeas, kCtrlMeasForced)) {
    return true;
  }
  // a write during a running measurement is delayed to its end (data sheet 3.4), so a re-trigger
  // must wait at least for the heating time of the measurement just started
  m_resultTimeout = DecodeGasWait(m_gasWait) / 1000.0 + kResultMargin;
  m_state = State::kMeasuring;
  m_stateTime = now;
  return false;
}

void BME688::Update() {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  if (m_state == State::kStart) {
    if (m_everTried && now - m_stateTime < kRetryTime) {
      return;
    }
    m_everTried = true;
    uint8_t chipId;
    uint8_t variant;
    if (m_i2c.Read(kRegChipId, 1, &chipId) || m_i2c.Read(kRegVariantId, 1, &variant)) {
      Fail(now);
      return;
    }
    if (chipId != kChipId || variant != kVariantBme688) {
      if (!m_warnedId) {
        m_warnedId = true;
        platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                               "[prosdk] BME688 0x{:02X} on I2C port {}: chip ID 0x{:02X} variant "
                               "0x{:02X}, expected 0x{:02X} 0x{:02X}\n",
                               m_address, static_cast<int>(m_i2c.GetPort()), chipId, variant,
                               kChipId, kVariantBme688);
      }
      Fail(now);
      return;
    }
    uint8_t a[23];
    uint8_t b[14];
    uint8_t heatVal;
    uint8_t heatRange;
    if (m_i2c.Read(kRegCalA, sizeof(a), a) || m_i2c.Read(kRegCalB, sizeof(b), b) ||
        m_i2c.Read(kRegResHeatVal, 1, &heatVal) || m_i2c.Read(kRegResHeatRange, 1, &heatRange)) {
      Fail(now);
      return;
    }
    // a[i] is register 0x8A + i, b[i] is register 0xE1 + i (Tables 13 to 16)
    Calibration& c = m_cal;
    c.t1 = U16(&b[0xE9 - 0xE1]);
    c.t2 = static_cast<int16_t>(U16(&a[0]));
    c.t3 = static_cast<int8_t>(a[0x8C - 0x8A]);
    c.p1 = U16(&a[0x8E - 0x8A]);
    c.p2 = static_cast<int16_t>(U16(&a[0x90 - 0x8A]));
    c.p3 = static_cast<int8_t>(a[0x92 - 0x8A]);
    c.p4 = static_cast<int16_t>(U16(&a[0x94 - 0x8A]));
    c.p5 = static_cast<int16_t>(U16(&a[0x96 - 0x8A]));
    c.p6 = static_cast<int8_t>(a[0x99 - 0x8A]);
    c.p7 = static_cast<int8_t>(a[0x98 - 0x8A]);
    c.p8 = static_cast<int16_t>(U16(&a[0x9C - 0x8A]));
    c.p9 = static_cast<int16_t>(U16(&a[0x9E - 0x8A]));
    c.p10 = a[0xA0 - 0x8A];
    // h1: 0xE3 (bits 11:4) and 0xE2 bits 3:0; h2: 0xE1 (bits 11:4) and 0xE2 bits 7:4
    c.h1 = static_cast<uint16_t>((b[0xE3 - 0xE1] << 4) | (b[0xE2 - 0xE1] & 0x0F));
    c.h2 = static_cast<uint16_t>((b[0xE1 - 0xE1] << 4) | (b[0xE2 - 0xE1] >> 4));
    c.h3 = static_cast<int8_t>(b[0xE4 - 0xE1]);
    c.h4 = static_cast<int8_t>(b[0xE5 - 0xE1]);
    c.h5 = static_cast<int8_t>(b[0xE6 - 0xE1]);
    c.h6 = b[0xE7 - 0xE1];
    c.h7 = static_cast<int8_t>(b[0xE8 - 0xE1]);
    c.g1 = static_cast<int8_t>(b[0xED - 0xE1]);
    c.g2 = static_cast<int16_t>(U16(&b[0xEB - 0xE1]));
    c.g3 = static_cast<int8_t>(b[0xEE - 0xE1]);
    c.resHeatRange = static_cast<uint8_t>((heatRange >> 4) & 0x03);
    c.resHeatVal = static_cast<int8_t>(heatVal);
    // ctrl_hum takes effect with the following ctrl_meas write (the trigger)
    m_gasWaitDirty = true;
    m_heaterCode = -1;
    if (m_i2c.Write(kRegCtrlHum, kCtrlHum) || m_i2c.Write(kRegConfig, kConfig) ||
        m_i2c.Write(kRegCtrlGas0, kCtrlGas0) || m_i2c.Write(kRegCtrlGas1, kCtrlGas1) ||
        Trigger(now)) {
      Fail(now);
      return;
    }
    m_connected = true;
    return;
  }

  // meas_status_0 first; the whole field only when it has new data
  uint8_t d[17];  // field 0, 0x1D..0x2D; d[i] is register 0x1D + i
  if (m_i2c.Read(kRegField0, 1, d)) {
    Fail(now);
    return;
  }
  m_connected = true;
  if ((d[0] & kNewData) == 0) {
    if (now - m_stateTime > m_resultTimeout && Trigger(now)) {
      Fail(now);
    }
    return;
  }
  if (m_i2c.Read(kRegField0, sizeof(d), d)) {
    Fail(now);
    return;
  }
  uint32_t pressAdc = (d[0x1F - 0x1D] << 12) | (d[0x20 - 0x1D] << 4) | (d[0x21 - 0x1D] >> 4);
  uint32_t tempAdc = (d[0x22 - 0x1D] << 12) | (d[0x23 - 0x1D] << 4) | (d[0x24 - 0x1D] >> 4);
  uint32_t humAdc = (d[0x25 - 0x1D] << 8) | d[0x26 - 0x1D];
  uint8_t gasLsb = d[0x2D - 0x1D];
  uint32_t gasAdc = (d[0x2C - 0x1D] << 2) | (gasLsb >> 6);
  double tFine = 0.0;
  m_temperature = CompensateTemperature(m_cal, tempAdc, tFine);
  m_pressure = CompensatePressure(m_cal, pressAdc, tFine);
  m_humidity = std::clamp(CompensateHumidity(m_cal, humAdc, m_temperature), 0.0, 100.0);
  m_gasResistance = GasResistance(gasAdc, gasLsb & 0x0F);
  m_gasValid = (gasLsb & kGasValid) && (gasLsb & kHeatStab);
  m_hasData = true;
  if (Trigger(now)) {
    Fail(now);
  }
}

units::celsius_t BME688::GetTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_temperature};
}

units::pascal_t BME688::GetPressure() const {
  std::scoped_lock lock{m_mutex};
  return units::pascal_t{m_pressure};
}

double BME688::GetHumidity() const {
  std::scoped_lock lock{m_mutex};
  return m_humidity;
}

units::ohm_t BME688::GetGasResistance() const {
  std::scoped_lock lock{m_mutex};
  return units::ohm_t{m_gasResistance};
}

bool BME688::IsGasValid() const {
  std::scoped_lock lock{m_mutex};
  return m_gasValid;
}

bool BME688::HasData() const {
  std::scoped_lock lock{m_mutex};
  return m_hasData;
}

bool BME688::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void BME688::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Temperature", m_temperature);
  table.Log("Pressure", m_pressure);
  table.Log("Humidity", m_humidity);
  table.Log("Gas Resistance", m_gasResistance);
  table.Log("Gas Valid", m_gasValid);
  table.Log("Connected", m_connected);
}

std::string_view BME688::GetTelemetryType() const {
  return "Environmental Sensor";
}
