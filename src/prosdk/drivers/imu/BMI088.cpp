// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// BMI088 driver (BMI088.hpp). Registers and timing: Bosch BST-BMI088-DS000-19 rev 1.9, sections
// 3 (device initialization), 4.1 (power modes), 5.3 (accelerometer registers), 5.5 (gyroscope
// registers), 6.2 (I2C addresses and write idle times) and 8.3 (sensing axes).

#include "prosdk/drivers/imu/BMI088.hpp"

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

// Accelerometer (5.3)
constexpr int kRegAccChipId = 0x00;
constexpr int kRegAccData = 0x12;  // 6 bytes: X, Y, Z, LSB first
constexpr int kRegTempMsb = 0x22;  // TEMP_MSB, then TEMP_LSB
constexpr int kRegAccConf = 0x40;
constexpr int kRegAccRange = 0x41;
constexpr int kRegAccPwrConf = 0x7C;
constexpr int kRegAccPwrCtrl = 0x7D;
// Gyroscope (5.5)
constexpr int kRegGyroChipId = 0x00;
constexpr int kRegGyroData = 0x02;  // 6 bytes: X, Y, Z, LSB first
constexpr int kRegGyroRange = 0x0F;
constexpr int kRegGyroBandwidth = 0x10;

constexpr uint8_t kAccChipId = 0x1E;
constexpr uint8_t kGyroChipId = 0x0F;
constexpr uint8_t kAccEnable = 0x04;          // ACC_PWR_CTRL: accelerometer on
constexpr uint8_t kAccActive = 0x00;          // ACC_PWR_CONF: active (reset 0x03, suspend)
constexpr uint8_t kAccConfNormal100Hz = 0xA8;  // acc_bwp normal, acc_odr 100 Hz (40 Hz cut-off)
constexpr uint8_t kAccRange6G = 0x01;
constexpr uint8_t kGyroRange1000Dps = 0x01;
constexpr uint8_t kGyroOdr400Bw47 = 0x03;

// 3: wait 450 us after ACC_PWR_CTRL; 6.2: 450 us between writes in suspend mode (2 us in normal
// mode). The same wait is used after every configuration write.
constexpr std::chrono::microseconds kWriteIdle{450};

constexpr double kGyroLsbPerDps = 32.768;              // +/-1000 deg/s (5.5.5)
constexpr double kAccelLsbPerG = 32768.0 / (1.5 * 4.0);  // 5.3.4: 1.5 * 2^(range + 1) g, range 1
constexpr double kGravity = 9.80665;                    // m/s^2 per g
constexpr double kDegreesToRadians = std::numbers::pi / 180.0;
constexpr double kFilterAlpha = 0.98;        // complementary filter: gyro weight
constexpr double kMaxIntegrationStep = 0.5;  // s; longer gaps are not integrated

int16_t LittleEndian(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

// Writes one register and waits the idle time before the next access.
bool WriteAndWait(I2C& i2c, int reg, uint8_t value) {
  bool aborted = i2c.Write(reg, value);
  std::this_thread::sleep_for(kWriteIdle);
  return aborted;
}

}  // namespace

BMI088::BMI088(I2C::Port port, int accelAddress, int gyroAddress,
               units::millisecond_t updatePeriod)
    : m_accel{port, accelAddress}, m_gyro{port, gyroAddress}, m_updatePeriod{updatePeriod} {
  uint8_t accId = 0;
  uint8_t gyroId = 0;
  // The accelerometer starts in I2C mode, in suspend (section 3): no SPI dummy read here.
  bool accAborted = m_accel.Read(kRegAccChipId, 1, &accId);
  bool gyroAborted = m_gyro.Read(kRegGyroChipId, 1, &gyroId);
  m_chipIdOk = !accAborted && !gyroAborted && accId == kAccChipId && gyroId == kGyroChipId;
  // Section 3 / 4.1.1: ACC_PWR_CTRL = 4 (on), wait 450 us; then ACC_PWR_CONF = 0 (active).
  accAborted = accAborted || WriteAndWait(m_accel, kRegAccPwrCtrl, kAccEnable);
  accAborted = accAborted || WriteAndWait(m_accel, kRegAccPwrConf, kAccActive);
  accAborted = accAborted || WriteAndWait(m_accel, kRegAccConf, kAccConfNormal100Hz);
  accAborted = accAborted || WriteAndWait(m_accel, kRegAccRange, kAccRange6G);
  // The gyroscope is in normal mode after power-on (4.1.2); GYRO_LPM1 is left at its reset value.
  gyroAborted = gyroAborted || WriteAndWait(m_gyro, kRegGyroRange, kGyroRange1000Dps);
  gyroAborted = gyroAborted || WriteAndWait(m_gyro, kRegGyroBandwidth, kGyroOdr400Bw47);
  if (accAborted || gyroAborted || !m_chipIdOk) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] BMI088 0x{:02X}/0x{:02X} on I2C port {}: accel {} (ID "
                           "0x{:02X}), gyro {} (ID 0x{:02X})\n",
                           accelAddress, gyroAddress, static_cast<int>(port),
                           accAborted ? "no answer" : "answered", accId,
                           gyroAborted ? "no answer" : "answered", gyroId);
  }
  util::ReportUsage("BMI088", accelAddress, "");
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "bmi088"}};
    m_thread = std::thread([this] { Run(); });
  }
}

BMI088::~BMI088() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

bool BMI088::ReadRaw(Raw* raw) {
  uint8_t acc[6];
  uint8_t gyro[6];
  uint8_t temp[2];
  // burst reads: the data registers of one part are locked during a burst (4.2)
  if (m_accel.Read(kRegAccData, sizeof(acc), acc) ||
      m_gyro.Read(kRegGyroData, sizeof(gyro), gyro) ||
      m_accel.Read(kRegTempMsb, sizeof(temp), temp)) {
    return false;  // aborted
  }
  raw->ax = LittleEndian(&acc[0]) / kAccelLsbPerG;
  raw->ay = LittleEndian(&acc[2]) / kAccelLsbPerG;
  raw->az = LittleEndian(&acc[4]) / kAccelLsbPerG;
  raw->gx = LittleEndian(&gyro[0]) / kGyroLsbPerDps;
  raw->gy = LittleEndian(&gyro[2]) / kGyroLsbPerDps;
  raw->gz = LittleEndian(&gyro[4]) / kGyroLsbPerDps;
  // 5.3.7: 11-bit two's complement, TEMP_MSB * 8 + TEMP_LSB / 32, 0.125 deg C/LSB + 23 deg C
  int t = temp[0] * 8 + temp[1] / 32;
  if (t > 1023) {
    t -= 2048;
  }
  raw->tempC = t * 0.125 + 23.0;
  return true;
}

bool BMI088::Calibrate(int samples) {
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

void BMI088::Update() {
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

void BMI088::Run() {
  auto period = std::chrono::microseconds{
      static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())};
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(period);
  }
}

bool BMI088::IsConnected() const {
  std::scoped_lock lock{m_mutex};
  return m_chipIdOk && m_lastReadOk;
}

units::radian_t BMI088::GetYaw() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_yaw - m_yawOffset};
}

void BMI088::ResetYaw() {
  std::scoped_lock lock{m_mutex};
  m_yawOffset = m_yaw;
}

math::Rotation2d BMI088::GetRotation2d() {
  return math::Rotation2d{GetYaw()};
}

math::Rotation3d BMI088::GetRotation3d() {
  std::scoped_lock lock{m_mutex};
  return math::Rotation3d{units::radian_t{m_roll}, units::radian_t{m_pitch},
                          units::radian_t{m_yaw - m_yawOffset}};
}

math::Quaternion BMI088::GetQuaternion() {
  return GetRotation3d().GetQuaternion();
}

units::radian_t BMI088::GetAngleX() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_roll};
}

units::radian_t BMI088::GetAngleY() {
  std::scoped_lock lock{m_mutex};
  return units::radian_t{m_pitch};
}

units::radian_t BMI088::GetAngleZ() {
  return GetYaw();
}

units::radians_per_second_t BMI088::GetGyroRateX() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateX};
}

units::radians_per_second_t BMI088::GetGyroRateY() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateY};
}

units::radians_per_second_t BMI088::GetGyroRateZ() {
  std::scoped_lock lock{m_mutex};
  return units::radians_per_second_t{m_rateZ};
}

units::meters_per_second_squared_t BMI088::GetAccelX() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.ax * kGravity};
}

units::meters_per_second_squared_t BMI088::GetAccelY() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.ay * kGravity};
}

units::meters_per_second_squared_t BMI088::GetAccelZ() {
  std::scoped_lock lock{m_mutex};
  return units::meters_per_second_squared_t{m_raw.az * kGravity};
}

units::celsius_t BMI088::GetTemperature() {
  std::scoped_lock lock{m_mutex};
  return units::celsius_t{m_raw.tempC};
}

void BMI088::LogTo(telemetry::TelemetryTable& table) const {
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

std::string_view BMI088::GetTelemetryType() const {
  return "Gyro";
}
