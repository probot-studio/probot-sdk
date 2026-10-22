// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// DPS310 driver (DPS310.hpp). HP303B data sheet V1.0 (2016-09-05): Table 8 (SENSOR_RDY within
// 12 ms, COEF_RDY within 40 ms), 4.4 (rate * time sum below 1 s), 4.9.1 and 4.9.2
// (compensation), Table 9 (scale factors), 6.1 (I2C address), Table 15 (register map), 8.3 to 8.6
// (PRS_CFG, TMP_CFG, MEAS_CFG, CFG_REG), Table 16 (measurement time per oversampling), 8.10 (ID),
// 8.11 / Table 18 (coefficients) and 8.12 (COEF_SRCE: use the temperature sensor the
// coefficients were made with).

#include "prosdk/drivers/environment/DPS310.hpp"

#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegPsr = 0x00;  // PSR_B2..B0 then TMP_B2..B0, 24-bit two's complement, MSB first
constexpr int kRegPrsCfg = 0x06;
constexpr int kRegTmpCfg = 0x07;
constexpr int kRegMeasCfg = 0x08;
constexpr int kRegCfgReg = 0x09;
constexpr int kRegId = 0x0D;
constexpr int kRegCoef = 0x10;  // 18 bytes, 0x10..0x21
constexpr int kRegCoefSrce = 0x28;

constexpr uint8_t kCoefRdy = 0x80;
constexpr uint8_t kSensorRdy = 0x40;
constexpr uint8_t kTmpRdy = 0x20;
constexpr uint8_t kPrsRdy = 0x10;
constexpr uint8_t kTmpExt = 0x80;  // TMP_CFG bit 7 and COEF_SRCE bit 7

// 8 measurements per second (rate 011) at 8 times oversampling (0011) for both: 14.8 ms each
// (Table 16), 8 * 2 * 14.8 ms = 237 ms of every second. No result shift is needed up to 8 times
// (Table 9), so CFG_REG stays 0.
constexpr uint8_t kCfg8Hz8x = 0x33;
constexpr uint8_t kMeasContinuous = 0x07;  // MEAS_CTRL 111: continuous pressure and temperature
constexpr double kScale8x = 7864320.0;     // kP = kT for 8 times (Table 9)

constexpr double kRetryTime = 1.0;   // s between attempts after a failure
constexpr double kReadyTime = 0.1;   // s; COEF_RDY within 40 ms (Table 8)

// Not applied: the undocumented temperature fix (registers 0x0E, 0x0F, 0x62) that Infineon's
// DPS310 library uses for some lots; the HP303B data sheet does not mention it (unverified).

int32_t SignExtend(uint32_t value, int bits) {
  uint32_t sign = 1u << (bits - 1);
  return static_cast<int32_t>((value ^ sign) - sign);
}

}  // namespace

DPS310::DPS310(I2C::Port port, int address) : m_i2c{port, address}, m_address{address} {
  util::ReportUsage("DPS310", address, "");
}

// m_mutex held.
void DPS310::Fail(double now) {
  m_connected = false;
  m_state = State::kStart;
  m_stateTime = now;
}

// m_mutex held. Reads the coefficients and starts continuous measurement; true = failed.
bool DPS310::Configure() {
  uint8_t c[18];
  uint8_t source;
  if (m_i2c.Read(kRegCoef, sizeof(c), c) || m_i2c.Read(kRegCoefSrce, 1, &source)) {
    return true;
  }
  // Table 18: c0 and c1 12 bits, c00 and c10 20 bits, the rest 16 bits, two's complement
  m_c0 = SignExtend((c[0] << 4) | (c[1] >> 4), 12);
  m_c1 = SignExtend(((c[1] & 0x0F) << 8) | c[2], 12);
  m_c00 = SignExtend((c[3] << 12) | (c[4] << 4) | (c[5] >> 4), 20);
  m_c10 = SignExtend(((c[5] & 0x0F) << 16) | (c[6] << 8) | c[7], 20);
  m_c01 = SignExtend((c[8] << 8) | c[9], 16);
  m_c11 = SignExtend((c[10] << 8) | c[11], 16);
  m_c20 = SignExtend((c[12] << 8) | c[13], 16);
  m_c21 = SignExtend((c[14] << 8) | c[15], 16);
  m_c30 = SignExtend((c[16] << 8) | c[17], 16);
  uint8_t tmpCfg = static_cast<uint8_t>((source & kTmpExt) | kCfg8Hz8x);
  return m_i2c.Write(kRegPrsCfg, kCfg8Hz8x) || m_i2c.Write(kRegTmpCfg, tmpCfg) ||
         m_i2c.Write(kRegCfgReg, 0x00) || m_i2c.Write(kRegMeasCfg, kMeasContinuous);
}

void DPS310::Update() {
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  switch (m_state) {
    case State::kStart: {
      if (m_everTried && now - m_stateTime < kRetryTime) {
        return;
      }
      m_everTried = true;
      uint8_t id;
      if (m_i2c.Read(kRegId, 1, &id)) {
        Fail(now);
        return;
      }
      if ((id & 0x0F) != 0x00) {  // PROD_ID 0 (8.10); REV_ID in bits 7:4 may vary
        if (!m_warnedId) {
          m_warnedId = true;
          platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                                 "[prosdk] DPS310 0x{:02X} on I2C port {}: product ID 0x{:02X} is "
                                 "not a DPS310\n",
                                 m_address, static_cast<int>(m_i2c.GetPort()), id);
        }
        Fail(now);
        return;
      }
      m_connected = true;
      m_state = State::kWaitReady;
      m_stateTime = now;
      return;
    }
    case State::kWaitReady: {
      uint8_t meas;
      if (m_i2c.Read(kRegMeasCfg, 1, &meas)) {
        Fail(now);
        return;
      }
      m_connected = true;
      if ((meas & (kCoefRdy | kSensorRdy)) != (kCoefRdy | kSensorRdy)) {
        if (now - m_stateTime > kReadyTime) {
          Fail(now);
        }
        return;
      }
      if (Configure()) {
        Fail(now);
        return;
      }
      m_state = State::kRunning;
      m_stateTime = now;
      return;
    }
    case State::kRunning: {
      uint8_t meas;
      if (m_i2c.Read(kRegMeasCfg, 1, &meas)) {
        Fail(now);
        return;
      }
      m_connected = true;
      if ((meas & (kTmpRdy | kPrsRdy)) == 0) {
        return;
      }
      // reading the results clears TMP_RDY and PRS_RDY (8.5)
      uint8_t buf[6];
      if (m_i2c.Read(kRegPsr, sizeof(buf), buf)) {
        Fail(now);
        return;
      }
      int32_t pRaw = SignExtend((buf[0] << 16) | (buf[1] << 8) | buf[2], 24);
      int32_t tRaw = SignExtend((buf[3] << 16) | (buf[4] << 8) | buf[5], 24);
      if (meas & kTmpRdy) {
        m_tempScaled = tRaw / kScale8x;
        m_temperature = m_c0 * 0.5 + m_c1 * m_tempScaled;  // 4.9.2
        m_hasTemperature = true;
      }
      if ((meas & kPrsRdy) && m_hasTemperature) {
        double p = pRaw / kScale8x;
        double t = m_tempScaled;
        m_pressure = m_c00 + p * (m_c10 + p * (m_c20 + p * m_c30)) + t * m_c01 +
                     t * p * (m_c11 + p * m_c21);  // 4.9.1
        m_hasData = true;
      }
      return;
    }
  }
}

units::pascal_t DPS310::GetPressure() const {
  std::scoped_lock lock{m_mutex};
  return units::pascal_t{m_pressure};
}

units::celsius_t DPS310::GetTemperature() const {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_temperature};
}

units::meter_t DPS310::GetAltitude(units::pascal_t seaLevel) const {
  std::scoped_lock lock{m_mutex};
  if (!m_hasData) {
    return units::meter_t{0.0};  // not 44330 m from a pressure of 0
  }
  return units::meter_t{44330.0 * (1.0 - std::pow(m_pressure / seaLevel.value(), 1.0 / 5.255))};
}

bool DPS310::HasData() const {
  std::scoped_lock lock{m_mutex};
  return m_hasData;
}

bool DPS310::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_connected;
}

void DPS310::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Pressure", m_pressure);
  table.Log("Temperature", m_temperature);
  table.Log("Connected", m_connected);
}

std::string_view DPS310::GetTelemetryType() const {
  return "Barometer";
}
