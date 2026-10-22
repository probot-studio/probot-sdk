// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// BNO055 driver (BNO055.hpp). Registers and timing: Bosch BST-BNO055-DS000-18 rev 1.8, sections
// 3.3 (operating modes, Table 3-6 switching times), 3.6 (units, data scaling, Table 3-13 rotation
// conventions), 3.11.5 (calibration profile), 4.3 (register map) and 5.5 (external crystal).

#include "prosdk/drivers/imu/BNO055.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// Page 0 registers (4.3); the driver never selects page 1.
constexpr int kRegChipId = 0x00;
constexpr int kRegPageId = 0x07;
constexpr int kRegAccDataX = 0x08;  // burst start: data 0x08-0x33, then TEMP .. SYS_STATUS
constexpr int kBurstLength = 0x39 - kRegAccDataX + 1;  // 50 bytes, up to SYS_STATUS
constexpr int kRegUnitSel = 0x3B;
constexpr int kRegOprMode = 0x3D;
constexpr int kRegPwrMode = 0x3E;
constexpr int kRegSysTrigger = 0x3F;
constexpr int kRegSysClkStatus = 0x38;
constexpr int kRegOffsets = 0x55;  // ACC_OFFSET_X_LSB; 22 bytes up to MAG_RADIUS_MSB 0x6A

// Offsets of the burst fields from kRegAccDataX.
constexpr int kAcc = 0x08 - kRegAccDataX;
constexpr int kGyr = 0x14 - kRegAccDataX;
constexpr int kEulHeading = 0x1A - kRegAccDataX;
constexpr int kGrv = 0x2E - kRegAccDataX;
constexpr int kTemp = 0x34 - kRegAccDataX;
constexpr int kCalibStat = 0x35 - kRegAccDataX;
constexpr int kSysStatus = 0x39 - kRegAccDataX;

constexpr uint8_t kChipId = 0xA0;
constexpr uint8_t kModeConfig = 0x00;
constexpr uint8_t kPowerNormal = 0x00;
constexpr uint8_t kUnitSel = 0x00;    // m/s^2, deg/s, degrees, deg C, Windows format (3.6.1)
constexpr uint8_t kClkSel = 0x80;     // SYS_TRIGGER bit 7: external crystal
constexpr uint8_t kStMainClk = 0x01;  // SYS_CLK_STATUS bit 0: clock source being configured
constexpr uint8_t kFusionRunning = 5;  // SYS_STATUS: sensor fusion algorithm running

// Table 3-6 switching times (19 ms and 7 ms), with 1 ms margin.
constexpr std::chrono::milliseconds kToConfigTime{20};
constexpr std::chrono::milliseconds kFromConfigTime{8};
// 5.5: the crystal takes about 600 ms to start; ST_MAIN_CLK is polled for at most 1 s.
constexpr std::chrono::milliseconds kClockPollInterval{10};
constexpr int kClockPollCount = 100;

constexpr double kRetryPeriod = 0.1;     // s, configuration retry while the chip is missing
constexpr double kAccelLsbPerMps2 = 100.0;  // 3.6.5.1 (also the gravity vector, 3.6.5.7)
constexpr double kGyroLsbPerDps = 16.0;     // 3.6.5.3
constexpr double kEulerLsbPerDegree = 16.0;  // 3.6.5.4
constexpr double kDegreesToRadians = std::numbers::pi / 180.0;

int16_t LittleEndian(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

double Wrap(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);  // [-pi, pi]
}

}  // namespace

BNO055::BNO055(I2C::Port port, int address, Mode mode, units::millisecond_t updatePeriod,
               bool externalCrystal)
    : m_i2c{port, address},
      m_mode{mode},
      m_externalCrystal{externalCrystal},
      m_updatePeriod{updatePeriod} {
  {
    std::scoped_lock io{m_ioMutex};
    m_lastAttempt = Timer::GetMonotonicTimestamp().value();
    m_configured = Configure();
  }
  if (!m_configured) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] BNO055 0x{:02X} on I2C port {}: no answer or not ready "
                           "(CHIP_ID 0xA0 expected); retrying\n",
                           address, static_cast<int>(port));
  }
  util::ReportUsage("BNO055", address, "");
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "bno055"}};
    m_thread = std::thread([this] { Run(); });
  }
}

BNO055::~BNO055() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

bool BNO055::EnterMode(uint8_t mode) {
  if (m_i2c.Write(kRegOprMode, mode)) {
    return false;
  }
  std::this_thread::sleep_for(mode == kModeConfig ? kToConfigTime : kFromConfigTime);
  return true;
}

bool BNO055::Configure() {
  // CHIP_ID reads 0xA0 once the chip is out of its power-on reset (up to 650 ms, Table 0-2)
  uint8_t id = 0;
  if (m_i2c.Read(kRegChipId, 1, &id) || id != kChipId) {
    return false;
  }
  bool aborted = m_i2c.Write(kRegPageId, 0);
  aborted = aborted || !EnterMode(kModeConfig);  // the registers below are writable only here
  aborted = aborted || m_i2c.Write(kRegPwrMode, kPowerNormal);
  aborted = aborted || m_i2c.Write(kRegUnitSel, kUnitSel);
  if (!aborted && m_externalCrystal) {
    uint8_t trigger = 0;
    aborted = m_i2c.Read(kRegSysTrigger, 1, &trigger);
    if (!aborted && (trigger & kClkSel) == 0) {  // not selected yet (first time after power-on)
      aborted = m_i2c.Write(kRegSysTrigger, kClkSel);
      // wait before each poll: ST_MAIN_CLK is not read in the same instant the switch starts
      // (when the chip sets it after the write is not stated: unverified)
      for (int i = 0; !aborted && i < kClockPollCount; ++i) {
        std::this_thread::sleep_for(kClockPollInterval);
        uint8_t clock = 0;
        aborted = m_i2c.Read(kRegSysClkStatus, 1, &clock);
        if (aborted || (clock & kStMainClk) == 0) {
          break;
        }
      }
      // without a working crystal the chip clears CLK_SEL and keeps its internal oscillator
    }
  }
  aborted = aborted || !EnterMode(static_cast<uint8_t>(m_mode));
  return !aborted;
}

void BNO055::Update() {
  uint8_t buf[kBurstLength];
  bool ok = false;
  bool reconfigured = false;
  {
    std::scoped_lock io{m_ioMutex};
    if (!m_configured) {
      double now = Timer::GetMonotonicTimestamp().value();
      if (now - m_lastAttempt >= kRetryPeriod) {
        m_lastAttempt = now;
        m_configured = Configure();
        reconfigured = m_configured;
      }
    }
    if (m_configured) {
      ok = !m_i2c.Read(kRegAccDataX, kBurstLength, buf);
      if (!ok) {
        m_configured = false;  // a chip that reset is back in CONFIG mode: configure it again
      }
    }
  }
  std::scoped_lock lock{m_mutex};
  if (reconfigured) {
    m_haveHeading = false;  // in the relative modes the chip's heading restarts after a mode change
  }
  m_lastReadOk = ok;
  if (!ok) {
    return;
  }
  m_ax = LittleEndian(&buf[kAcc]) / kAccelLsbPerMps2;
  m_ay = LittleEndian(&buf[kAcc + 2]) / kAccelLsbPerMps2;
  m_az = LittleEndian(&buf[kAcc + 4]) / kAccelLsbPerMps2;
  m_rateX = LittleEndian(&buf[kGyr]) / kGyroLsbPerDps * kDegreesToRadians;
  m_rateY = LittleEndian(&buf[kGyr + 2]) / kGyroLsbPerDps * kDegreesToRadians;
  m_rateZ = LittleEndian(&buf[kGyr + 4]) / kGyroLsbPerDps * kDegreesToRadians;
  m_tempC = static_cast<int8_t>(buf[kTemp]);
  m_calibStat = buf[kCalibStat];
  if (buf[kSysStatus] != kFusionRunning) {
    return;  // fusion outputs are not valid yet: keep the last orientation
  }
  // heading 0-360 degrees, clockwise increasing (Table 3-13): negate for CCW-positive yaw
  double heading =
      Wrap(-LittleEndian(&buf[kEulHeading]) / kEulerLsbPerDegree * kDegreesToRadians);
  if (!m_yawInitialized) {
    m_yaw = heading;
    m_yawInitialized = true;
  } else if (m_haveHeading) {
    m_yaw += Wrap(heading - m_lastHeading);
  }
  m_lastHeading = heading;
  m_haveHeading = true;
  double gx = LittleEndian(&buf[kGrv]) / kAccelLsbPerMps2;
  double gy = LittleEndian(&buf[kGrv + 2]) / kAccelLsbPerMps2;
  double gz = LittleEndian(&buf[kGrv + 4]) / kAccelLsbPerMps2;
  m_roll = std::atan2(gy, gz);
  m_pitch = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));
}

void BNO055::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

bool BNO055::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_configured && m_lastReadOk;
}

units::radian_t BNO055::GetYaw() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_yaw - m_yawOffset};
}

void BNO055::ResetYaw() {
  std::scoped_lock lock{m_mutex};
  m_yawOffset = m_yaw;
}

math::Rotation2d BNO055::GetRotation2d() {
  return math::Rotation2d{GetYaw()};
}

math::Rotation3d BNO055::GetRotation3d() {
  std::scoped_lock lock{m_mutex};
  return math::Rotation3d{units::radian_t{m_roll}, units::radian_t{m_pitch},
                          units::radian_t{m_yaw - m_yawOffset}};
}

math::Quaternion BNO055::GetQuaternion() {
  return GetRotation3d().GetQuaternion();
}

units::radian_t BNO055::GetAngleX() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_roll};
}

units::radian_t BNO055::GetAngleY() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_pitch};
}

units::radian_t BNO055::GetAngleZ() {
  return GetYaw();
}

units::radians_per_second_t BNO055::GetGyroRateX() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateX};
}

units::radians_per_second_t BNO055::GetGyroRateY() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateY};
}

units::radians_per_second_t BNO055::GetGyroRateZ() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateZ};
}

units::meters_per_second_squared_t BNO055::GetAccelX() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_ax};
}

units::meters_per_second_squared_t BNO055::GetAccelY() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_ay};
}

units::meters_per_second_squared_t BNO055::GetAccelZ() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_az};
}

units::celsius_t BNO055::GetTemperature() {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_tempC};
}

BNO055::CalibrationStatus BNO055::GetCalibrationStatus() {
  std::scoped_lock lock{m_mutex};
  // CALIB_STAT 0x35 (4.3.54): SYS bits 7:6, GYR 5:4, ACC 3:2, MAG 1:0
  return {.system = (m_calibStat >> 6) & 0x03,
          .gyro = (m_calibStat >> 4) & 0x03,
          .accel = (m_calibStat >> 2) & 0x03,
          .mag = m_calibStat & 0x03};
}

std::optional<BNO055::Offsets> BNO055::GetOffsets() {
  Offsets offsets{};
  bool ok;
  {
    std::scoped_lock io{m_ioMutex};
    if (!m_configured) {
      return std::nullopt;
    }
    // 3.11.5: the profile is read in CONFIG mode
    ok = EnterMode(kModeConfig) &&
         !m_i2c.Read(kRegOffsets, static_cast<int>(offsets.size()), offsets.data());
    ok = EnterMode(static_cast<uint8_t>(m_mode)) && ok;
    m_configured = ok;
  }
  std::scoped_lock lock{m_mutex};
  m_haveHeading = false;
  if (!ok) {
    return std::nullopt;
  }
  return offsets;
}

bool BNO055::SetOffsets(const Offsets& offsets) {
  bool ok;
  {
    std::scoped_lock io{m_ioMutex};
    if (!m_configured) {
      return false;
    }
    // 3.11.5: CONFIG mode, write the profile, back to the fusion mode. Single-byte writes in
    // address order; each block takes effect when its last (MSB) byte is written (3.6.4).
    ok = EnterMode(kModeConfig);
    for (std::size_t i = 0; ok && i < offsets.size(); ++i) {
      ok = !m_i2c.Write(kRegOffsets + static_cast<int>(i), offsets[i]);
    }
    ok = EnterMode(static_cast<uint8_t>(m_mode)) && ok;
    m_configured = ok;
  }
  std::scoped_lock lock{m_mutex};
  m_haveHeading = false;
  return ok;
}

void BNO055::LogTo(telemetry::TelemetryTable& table) const {
  double yaw, roll, pitch, rateZ;
  {
    std::scoped_lock lock{m_mutex};
    yaw = m_yaw - m_yawOffset;
    roll = m_roll;
    pitch = m_pitch;
    rateZ = m_rateZ;
  }
  constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;
  table.Log("Value", yaw * kRadiansToDegrees);  // dashboard gyro widget: degrees
  table.Log("Roll", roll * kRadiansToDegrees);
  table.Log("Pitch", pitch * kRadiansToDegrees);
  table.Log("Rate", rateZ * kRadiansToDegrees);
}

std::string_view BNO055::GetTelemetryType() const {
  return "Gyro";
}
