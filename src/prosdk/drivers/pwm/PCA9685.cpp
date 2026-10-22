// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// PCA9685 driver (PCA9685.hpp). Register map and timing: NXP PCA9685 datasheet rev. 4, sections
// 7.3 (registers) and 7.3.5 (PRE_SCALE).

#include "prosdk/drivers/pwm/PCA9685.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>

#include "prosdk/hal/DriverStation.h"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kRegMode1 = 0x00;
constexpr uint8_t kRegMode2 = 0x01;
constexpr uint8_t kRegLed0OnL = 0x06;
constexpr uint8_t kRegAllLedOnL = 0xFA;
constexpr uint8_t kRegPrescale = 0xFE;

constexpr uint8_t kMode1AutoIncrement = 0x20;
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1AllCall = 0x01;
constexpr uint8_t kMode2TotemPole = 0x04;
constexpr uint8_t kFullBit = 0x10;  // bit 4 of LEDn_ON_H / LEDn_OFF_H: full on / full off

constexpr double kOscillatorHz = 25e6;
constexpr double kMinFrequency = 24.0;    // prescale 255
constexpr double kMaxFrequency = 1526.0;  // prescale 3

uint8_t PrescaleFor(double hz) {
  double pre = std::round(kOscillatorHz / (4096.0 * hz)) - 1.0;
  return static_cast<uint8_t>(std::clamp(pre, 3.0, 255.0));
}

// The output gate of the HAL outputs (Esp32Internal.hpp), as far as it is visible to a driver.
bool GateOpen() {
  return HAL_GetOutputsEnabled() && !platform::EmergencyStopped() &&
         !platform::RobotProgramHalted();
}

}  // namespace

PCA9685::PCA9685(I2C::Port port, int address, units::hertz_t frequency,
                 units::millisecond_t updatePeriod)
    : m_i2c{port, address},
      m_address{address},
      m_prescale{0},
      m_updatePeriod{updatePeriod} {
  double hz = frequency.value();
  if (!(hz >= kMinFrequency && hz <= kMaxFrequency)) {
    throw WPILIB_MakeError(err::ParameterOutOfRange,
                           "PCA9685 frequency {} Hz (valid: {} to {} Hz)", hz, kMinFrequency,
                           kMaxFrequency);
  }
  m_prescale = PrescaleFor(hz);
  m_dirty = (uint32_t{1} << kNumChannels) - 1;
  {
    std::scoped_lock io{m_ioMutex};
    m_configured = Configure();
  }
  if (!m_configured) {
    platform::ConsolePrint("[prosdk] PCA9685 0x{:02X}: no answer on I2C port {}; retrying\n",
                           address, static_cast<int>(port));
  }
  util::ReportUsage("PCA9685", address, "");
  if (m_updatePeriod > 0_ms) {
    platform::ScopedThreadConfig threadConfig{{.name = "pca9685"}};
    m_thread = std::thread([this] { Run(); });
  }
}

PCA9685::~PCA9685() {
  m_stop = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
  std::scoped_lock io{m_ioMutex};
  if (m_configured) {
    WriteAllOff();
  }
}

void PCA9685::CheckChannel(int channel) const {
  if (channel < 0 || channel >= kNumChannels) {
    throw WPILIB_MakeError(err::ChannelIndexOutOfRange, "PCA9685 channel {} (valid: 0 to {})",
                           channel, kNumChannels - 1);
  }
}

void PCA9685::SetDutyCycle(int channel, double dutyCycle) {
  CheckChannel(channel);
  if (!(dutyCycle > 0.0)) {
    dutyCycle = 0.0;  // also NaN
  } else if (dutyCycle > 1.0) {
    dutyCycle = 1.0;
  }
  auto counts = static_cast<uint16_t>(std::lround(dutyCycle * kResolution));
  std::scoped_lock lock{m_mutex};
  if (m_counts[channel] != counts) {
    m_counts[channel] = counts;
    m_dirty |= uint32_t{1} << channel;
  }
}

void PCA9685::SetPulseWidth(int channel, units::microsecond_t pulse) {
  SetDutyCycle(channel, (pulse / GetPeriod()).value());
}

void PCA9685::SetOff(int channel) {
  SetDutyCycle(channel, 0.0);
}

double PCA9685::GetDutyCycle(int channel) const {
  CheckChannel(channel);
  std::scoped_lock lock{m_mutex};
  return static_cast<double>(m_counts[channel]) / kResolution;
}

units::hertz_t PCA9685::GetFrequency() const {
  return units::hertz_t{kOscillatorHz / (4096.0 * (m_prescale + 1))};
}

units::microsecond_t PCA9685::GetPeriod() const {
  return units::microsecond_t{4096.0 * (m_prescale + 1) * 1e6 / kOscillatorHz};
}

bool PCA9685::IsConnected() const {
  return m_connected;
}

bool PCA9685::OutputsEnabled() const {
  return m_enabled;
}

int PCA9685::GetAddress() const {
  return m_address;
}

// m_ioMutex held. WPILib I2C calls return true when the transfer was aborted.
bool PCA9685::Configure() {
  bool aborted = m_i2c.Write(kRegMode1, kMode1Sleep | kMode1AutoIncrement | kMode1AllCall);
  aborted = aborted || m_i2c.Write(kRegPrescale, m_prescale);
  aborted = aborted || m_i2c.Write(kRegMode2, kMode2TotemPole);
  // all channels fully off before the oscillator starts
  aborted = aborted || !WriteAllOff();
  aborted = aborted || m_i2c.Write(kRegMode1, kMode1AutoIncrement | kMode1AllCall);
  m_connected = !aborted;
  if (!aborted) {
    // the oscillator needs 500 us after leaving sleep; outputs stay off meanwhile
    std::this_thread::sleep_for(std::chrono::microseconds{500});
  }
  return !aborted;
}

// m_ioMutex held.
bool PCA9685::WriteAllOff() {
  uint8_t buf[] = {kRegAllLedOnL, 0, 0, 0, kFullBit};
  bool ok = !m_i2c.WriteBulk(buf, sizeof(buf));
  m_connected = ok;
  if (ok) {
    m_outputsOff = true;
  }
  return ok;
}

// m_ioMutex held.
bool PCA9685::WriteChannel(int channel, uint16_t counts) {
  uint8_t buf[5] = {static_cast<uint8_t>(kRegLed0OnL + 4 * channel), 0, 0, 0, 0};
  if (counts == 0) {
    buf[4] = kFullBit;
  } else if (counts >= kResolution) {
    buf[2] = kFullBit;
  } else {
    buf[3] = static_cast<uint8_t>(counts & 0xFF);
    buf[4] = static_cast<uint8_t>(counts >> 8);
  }
  bool ok = !m_i2c.WriteBulk(buf, sizeof(buf));
  m_connected = ok;
  return ok;
}

void PCA9685::Update() {
  std::scoped_lock io{m_ioMutex};
  if (!m_configured) {
    m_configured = Configure();
    if (!m_configured) {
      return;
    }
  }
  bool open = GateOpen();
  m_enabled = open;
  if (!open) {
    if (!m_outputsOff) {
      WriteAllOff();
    }
    return;
  }
  uint32_t dirty;
  std::array<uint16_t, kNumChannels> counts;
  {
    std::scoped_lock lock{m_mutex};
    if (m_outputsOff) {
      m_dirty = (uint32_t{1} << kNumChannels) - 1;  // ALL_LED_OFF overwrote every channel
    }
    dirty = m_dirty;
    counts = m_counts;
    m_dirty = 0;
  }
  uint32_t failed = 0;
  for (int ch = 0; ch < kNumChannels; ++ch) {
    if ((dirty & (uint32_t{1} << ch)) != 0 && !WriteChannel(ch, counts[ch])) {
      failed |= uint32_t{1} << ch;
    }
  }
  if (failed != 0) {
    std::scoped_lock lock{m_mutex};
    m_dirty |= failed;
  } else {
    m_outputsOff = false;
  }
}

void PCA9685::Run() {
  while (!m_stop) {
    Update();
    std::this_thread::sleep_for(std::chrono::microseconds{
        static_cast<int64_t>(units::microsecond_t{m_updatePeriod}.value())});
  }
}
