// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// LSM303AGR driver (LSM303AGR.hpp). Registers: ST LSM303AGR datasheet DocID027765 Rev 11,
// Table 3 (sensitivities), Table 14 (accelerometer modes), section 6.1.1 (I2C sub-address
// auto-increment), Table 26 (register map) and section 8 (register descriptions).

#include "prosdk/drivers/imu/LSM303AGR.hpp"

#include <chrono>
#include <cmath>
#include <numbers>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// Accelerometer, address 0x19
constexpr int kRegWhoAmIA = 0x0F;
constexpr int kRegCtrlReg1A = 0x20;
constexpr int kRegCtrlReg4A = 0x23;
constexpr int kRegOutXLA = 0x28;  // 6 bytes: X, Y, Z, LSB first
constexpr int kAutoIncrement = 0x80;  // 6.1.1: MSb of the sub-address enables auto-increment
// Magnetometer, address 0x1E
constexpr int kRegWhoAmIM = 0x4F;
constexpr int kRegCfgRegAM = 0x60;
constexpr int kRegCfgRegBM = 0x61;
constexpr int kRegCfgRegCM = 0x62;
constexpr int kRegOutXLM = 0x68;  // 6 bytes: X, Y, Z, LSB first

constexpr uint8_t kWhoAmIA = 0x33;
constexpr uint8_t kWhoAmIM = 0x40;
constexpr uint8_t kCtrl1A100HzXyz = 0x57;  // ODR 0101 = 100 Hz, LPen 0, Z/Y/X enabled (8.6)
constexpr uint8_t kCtrl4ABduHr4G = 0x98;   // BDU, FS 01 = +/-4 g, HR (8.9)
// COMP_TEMP_EN (must be 1, Table 92 note), high resolution, ODR 11 = 100 Hz, continuous mode
constexpr uint8_t kCfgAM = 0x8C;
constexpr uint8_t kCfgBMOffCancLpf = 0x03;  // OFF_CANC, LPF (8.40)
constexpr uint8_t kCfgCMBdu = 0x10;         // BDU (8.41)

constexpr double kAccelGPerLsb = 1.95e-3;  // +/-4 g, high-resolution mode, 12-bit (Table 3)
constexpr double kMagTeslaPerLsb = 1.5e-7;  // 1.5 mgauss/LSB (Table 3)
constexpr double kGravity = 9.80665;        // m/s^2 per g

int16_t LittleEndian(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

double Wrap(double angle) {
  return std::remainder(angle, 2.0 * std::numbers::pi);  // [-pi, pi]
}

struct Vec {
  double x, y, z;
};

Vec Cross(const Vec& a, const Vec& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

}  // namespace

LSM303AGR::LSM303AGR(I2C::Port port, units::millisecond_t updatePeriod)
    : m_accel{port, kAccelAddress}, m_mag{port, kMagAddress}, m_updatePeriod{updatePeriod} {
  uint8_t whoA = 0;
  uint8_t whoM = 0;
  bool accAborted = m_accel.Read(kRegWhoAmIA, 1, &whoA);
  bool magAborted = m_mag.Read(kRegWhoAmIM, 1, &whoM);
  m_whoAmIOk = !accAborted && !magAborted && whoA == kWhoAmIA && whoM == kWhoAmIM;
  // Both sensors are off after power-on: CTRL_REG1_A ODR 0000 (power-down), CFG_REG_A_M MD 11
  // (idle) (Table 26 defaults).
  magAborted = magAborted || m_mag.Write(kRegCfgRegAM, kCfgAM);
  magAborted = magAborted || m_mag.Write(kRegCfgRegBM, kCfgBMOffCancLpf);
  magAborted = magAborted || m_mag.Write(kRegCfgRegCM, kCfgCMBdu);
  accAborted = accAborted || m_accel.Write(kRegCtrlReg1A, kCtrl1A100HzXyz);
  accAborted = accAborted || m_accel.Write(kRegCtrlReg4A, kCtrl4ABduHr4G);
  if (accAborted || magAborted || !m_whoAmIOk) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] LSM303AGR on I2C port {}: accel {} (WHO_AM_I 0x{:02X}), "
                           "mag {} (WHO_AM_I 0x{:02X})\n",
                           static_cast<int>(port), accAborted ? "no answer" : "answered", whoA,
                           magAborted ? "no answer" : "answered", whoM);
  }
  util::ReportUsage("LSM303AGR", kAccelAddress, "");
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "lsm303agr"}};
    m_thread = std::thread([this] { Run(); });
  }
}

LSM303AGR::~LSM303AGR() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void LSM303AGR::Update() {
  uint8_t acc[6];
  uint8_t mag[6];
  // The magnetometer is read with the plain sub-address, as ST's own driver does; whether it
  // also honours the auto-increment bit is not stated (unverified).
  bool ok = !m_accel.Read(kRegOutXLA | kAutoIncrement, sizeof(acc), acc) &&
            !m_mag.Read(kRegOutXLM, sizeof(mag), mag);
  std::scoped_lock lock{m_mutex};
  m_lastReadOk = ok;
  if (!ok) {
    return;
  }
  // high-resolution mode: 12-bit data, left-justified in 16 bits (Table 14)
  m_ax = (LittleEndian(&acc[0]) >> 4) * kAccelGPerLsb * kGravity;
  m_ay = (LittleEndian(&acc[2]) >> 4) * kAccelGPerLsb * kGravity;
  m_az = (LittleEndian(&acc[4]) >> 4) * kAccelGPerLsb * kGravity;
  m_mx = LittleEndian(&mag[0]) * kMagTeslaPerLsb - m_offsetX;
  m_my = LittleEndian(&mag[2]) * kMagTeslaPerLsb - m_offsetY;
  m_mz = LittleEndian(&mag[4]) * kMagTeslaPerLsb - m_offsetZ;

  m_roll = std::atan2(m_ay, m_az);
  m_pitch = std::atan2(-m_ax, std::sqrt(m_ay * m_ay + m_az * m_az));

  // Tilt-compensated heading: with Down = -accel (normalized), East = Down x Mag and
  // North = East x Down are the horizontal directions in the sensor frame (the vertical part of
  // the field drops out). Compass heading of the X axis = atan2(East.x, North.x), clockwise;
  // yaw is its negation.
  double g = std::sqrt(m_ax * m_ax + m_ay * m_ay + m_az * m_az);
  if (g <= 0.0) {
    return;
  }
  Vec down{-m_ax / g, -m_ay / g, -m_az / g};
  Vec east = Cross(down, Vec{m_mx, m_my, m_mz});
  Vec north = Cross(east, down);
  if (east.x == 0.0 && north.x == 0.0) {
    return;  // no horizontal field (or no reading)
  }
  double heading = std::atan2(-east.x, north.x);
  if (m_haveHeading) {
    m_yaw += Wrap(heading - m_heading);
  } else {
    m_yaw = heading;
    m_haveHeading = true;
  }
  m_heading = heading;
}

void LSM303AGR::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

bool LSM303AGR::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_whoAmIOk && m_lastReadOk;
}

units::radian_t LSM303AGR::GetHeading() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_heading};
}

units::radian_t LSM303AGR::GetYaw() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_yaw - m_yawOffset};
}

void LSM303AGR::ResetYaw() {
  std::scoped_lock lock{m_mutex};
  m_yawOffset = m_yaw;
}

math::Rotation2d LSM303AGR::GetRotation2d() {
  return math::Rotation2d{GetYaw()};
}

math::Rotation3d LSM303AGR::GetRotation3d() {
  std::scoped_lock lock{m_mutex};
  return math::Rotation3d{units::radian_t{m_roll}, units::radian_t{m_pitch},
                          units::radian_t{m_yaw - m_yawOffset}};
}

math::Quaternion LSM303AGR::GetQuaternion() {
  return GetRotation3d().GetQuaternion();
}

units::radian_t LSM303AGR::GetAngleX() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_roll};
}

units::radian_t LSM303AGR::GetAngleY() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_pitch};
}

units::radian_t LSM303AGR::GetAngleZ() {
  return GetYaw();
}

units::meters_per_second_squared_t LSM303AGR::GetAccelX() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_ax};
}

units::meters_per_second_squared_t LSM303AGR::GetAccelY() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_ay};
}

units::meters_per_second_squared_t LSM303AGR::GetAccelZ() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_az};
}

units::tesla_t LSM303AGR::GetMagX() {
  std::scoped_lock lock{m_mutex};
  return units::tesla_t{m_mx};
}

units::tesla_t LSM303AGR::GetMagY() {
  std::scoped_lock lock{m_mutex};
  return units::tesla_t{m_my};
}

units::tesla_t LSM303AGR::GetMagZ() {
  std::scoped_lock lock{m_mutex};
  return units::tesla_t{m_mz};
}

void LSM303AGR::SetHardIronOffset(units::tesla_t x, units::tesla_t y, units::tesla_t z) {
  std::scoped_lock lock{m_mutex};
  m_offsetX = x.value();
  m_offsetY = y.value();
  m_offsetZ = z.value();
}

void LSM303AGR::LogTo(telemetry::TelemetryTable& table) const {
  double yaw, roll, pitch, heading;
  {
    std::scoped_lock lock{m_mutex};
    yaw = m_yaw - m_yawOffset;
    roll = m_roll;
    pitch = m_pitch;
    heading = m_heading;
  }
  constexpr double kRadiansToDegrees = 180.0 / std::numbers::pi;
  table.Log("Value", yaw * kRadiansToDegrees);  // dashboard gyro widget: degrees
  table.Log("Roll", roll * kRadiansToDegrees);
  table.Log("Pitch", pitch * kRadiansToDegrees);
  table.Log("Heading", heading * kRadiansToDegrees);
}

std::string_view LSM303AGR::GetTelemetryType() const {
  return "Gyro";
}
