// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// MPU-6050 driver (MPU6050.hpp). Registers: InvenSense RM-MPU-6000A-00 rev. 4.2.

#include "prosdk/drivers/imu/MPU6050.hpp"

#include <chrono>
#include <cmath>
#include <numbers>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Timer.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr int kRegSampleRateDivider = 0x19;
constexpr int kRegConfig = 0x1A;
constexpr int kRegGyroConfig = 0x1B;
constexpr int kRegAccelConfig = 0x1C;
constexpr int kRegAccelXOutH = 0x3B;  // 14 bytes: accel XYZ, temperature, gyro XYZ
constexpr int kRegPowerManagement1 = 0x6B;
constexpr int kRegWhoAmI = 0x75;

constexpr uint8_t kClockPllGyroX = 0x01;
constexpr uint8_t kDlpf44Hz = 0x03;
constexpr uint8_t kGyro1000Dps = 0x10;
constexpr uint8_t kAccel4G = 0x08;

constexpr double kGyroLsbPerDps = 32.8;    // +/-1000 deg/s
constexpr double kAccelLsbPerG = 8192.0;   // +/-4 g
constexpr double kGravity = 9.80665;       // m/s^2 per g
constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
constexpr double kFilterAlpha = 0.98;      // complementary filter: gyro weight
constexpr double kMaxIntegrationStep = 0.5;  // s; longer gaps are not integrated

int16_t BigEndian(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

MPU6050::MPU6050(I2C::Port port, int address, units::millisecond_t updatePeriod)
    : m_i2c{port, address}, m_updatePeriod{updatePeriod} {
  uint8_t who = 0;
  bool aborted = m_i2c.Read(kRegWhoAmI, 1, &who);
  // MPU-6050 answers 0x68; the register-compatible MPU-6500/9250 family 0x70/0x71/0x73
  m_whoAmIOk = !aborted && (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73);
  aborted = aborted || m_i2c.Write(kRegPowerManagement1, kClockPllGyroX);
  aborted = aborted || m_i2c.Write(kRegSampleRateDivider, 0);
  aborted = aborted || m_i2c.Write(kRegConfig, kDlpf44Hz);
  aborted = aborted || m_i2c.Write(kRegGyroConfig, kGyro1000Dps);
  aborted = aborted || m_i2c.Write(kRegAccelConfig, kAccel4G);
  if (aborted || !m_whoAmIOk) {
    platform::ConsolePrint(
        platform::ConsoleLevel::kWarning, "[prosdk] MPU6050 0x{:02X} on I2C port {}: {} (WHO_AM_I 0x{:02X})\n", address,
        static_cast<int>(port), aborted ? "no answer" : "unexpected chip", who);
  }
  util::ReportUsage("MPU6050", address, "");
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "mpu6050"}};
    m_thread = std::thread([this] { Run(); });
  }
}

MPU6050::~MPU6050() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

bool MPU6050::ReadRaw(Raw* raw) {
  uint8_t buf[14];
  if (m_i2c.Read(kRegAccelXOutH, sizeof(buf), buf)) {
    return false;  // aborted
  }
  raw->ax = BigEndian(&buf[0]) / kAccelLsbPerG;
  raw->ay = BigEndian(&buf[2]) / kAccelLsbPerG;
  raw->az = BigEndian(&buf[4]) / kAccelLsbPerG;
  raw->tempC = BigEndian(&buf[6]) / 340.0 + 36.53;
  raw->gx = BigEndian(&buf[8]) / kGyroLsbPerDps;
  raw->gy = BigEndian(&buf[10]) / kGyroLsbPerDps;
  raw->gz = BigEndian(&buf[12]) / kGyroLsbPerDps;
  return true;
}

bool MPU6050::Calibrate(int samples) {
  std::scoped_lock lock{m_mutex};
  double sx = 0.0, sy = 0.0, sz = 0.0;
  int n = 0;
  for (int i = 0; i < samples; ++i) {
    Raw r;
    if (ReadRaw(&r)) {
      sx += r.gx;
      sy += r.gy;
      sz += r.gz;
      ++n;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  m_lastReadOk = n > 0;
  if (n == 0) {
    return false;
  }
  m_biasX = sx / n;
  m_biasY = sy / n;
  m_biasZ = sz / n;
  m_haveSample = false;  // restart integration after the pause
  return true;
}

void MPU6050::Update() {
  Raw r;
  bool ok = ReadRaw(&r);
  double now = Timer::GetMonotonicTimestamp().value();
  std::scoped_lock lock{m_mutex};
  m_lastReadOk = ok;
  if (!ok) {
    return;
  }
  m_raw = r;
  m_rateX = (r.gx - m_biasX) * kDegreesToRadians;
  m_rateY = (r.gy - m_biasY) * kDegreesToRadians;
  m_rateZ = (r.gz - m_biasZ) * kDegreesToRadians;
  double accelRoll = std::atan2(r.ay, r.az);
  double accelPitch = std::atan2(-r.ax, std::sqrt(r.ay * r.ay + r.az * r.az));
  if (!m_haveSample) {
    m_roll = accelRoll;
    m_pitch = accelPitch;
  } else {
    double dt = now - m_lastTime;
    if (dt > 0.0 && dt < kMaxIntegrationStep) {
      m_yaw += m_rateZ * dt;
      m_roll = kFilterAlpha * (m_roll + m_rateX * dt) + (1.0 - kFilterAlpha) * accelRoll;
      m_pitch = kFilterAlpha * (m_pitch + m_rateY * dt) + (1.0 - kFilterAlpha) * accelPitch;
    }
  }
  m_haveSample = true;
  m_lastTime = now;
}

void MPU6050::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

bool MPU6050::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_whoAmIOk && m_lastReadOk;
}

units::radian_t MPU6050::GetYaw() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_yaw - m_yawOffset};
}

void MPU6050::ResetYaw() {
  std::scoped_lock lock{m_mutex};
  m_yawOffset = m_yaw;
}

math::Rotation2d MPU6050::GetRotation2d() {
  return math::Rotation2d{GetYaw()};
}

math::Rotation3d MPU6050::GetRotation3d() {
  std::scoped_lock lock{m_mutex};
  return math::Rotation3d{units::radian_t{m_roll}, units::radian_t{m_pitch},
                          units::radian_t{m_yaw - m_yawOffset}};
}

math::Quaternion MPU6050::GetQuaternion() {
  return GetRotation3d().GetQuaternion();
}

units::radian_t MPU6050::GetAngleX() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_roll};
}

units::radian_t MPU6050::GetAngleY() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_pitch};
}

units::radian_t MPU6050::GetAngleZ() {
  return GetYaw();
}

units::radians_per_second_t MPU6050::GetGyroRateX() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateX};
}

units::radians_per_second_t MPU6050::GetGyroRateY() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateY};
}

units::radians_per_second_t MPU6050::GetGyroRateZ() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateZ};
}

units::meters_per_second_squared_t MPU6050::GetAccelX() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.ax * kGravity};
}

units::meters_per_second_squared_t MPU6050::GetAccelY() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.ay * kGravity};
}

units::meters_per_second_squared_t MPU6050::GetAccelZ() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.az * kGravity};
}

units::celsius_t MPU6050::GetTemperature() {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_raw.tempC};
}

void MPU6050::LogTo(telemetry::TelemetryTable& table) const {
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

std::string_view MPU6050::GetTelemetryType() const {
  return "Gyro";
}
