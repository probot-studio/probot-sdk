// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// DRV2605L driver (DRV2605L.hpp). Registers: TI SLOS854D (rev. D, March 2018), section 8.6
// (register map, table 3; DEVICE_ID, table 4); real-time playback format: figures 26 (open loop)
// and 28 (closed loop, bidirectional input, the LRA default).

#include "prosdk/drivers/haptic/DRV2605L.hpp"

#include <cmath>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

constexpr uint8_t kRegStatus = 0x00;    // DEVICE_ID[7:5], DIAG_RESULT, OVER_TEMP, OC_DETECT
constexpr uint8_t kRegMode = 0x01;      // DEV_RESET, STANDBY, MODE[2:0]
constexpr uint8_t kRegRtpInput = 0x02;
constexpr uint8_t kRegLibrary = 0x03;   // HI_Z, LIBRARY_SEL[2:0]
constexpr uint8_t kRegWaveSeq1 = 0x04;  // 0x04 to 0x0B
constexpr uint8_t kRegGo = 0x0C;
constexpr uint8_t kRegFeedback = 0x1A;  // N_ERM_LRA in bit 7

constexpr uint8_t kDeviceIdDrv2605L = 7;  // table 4
constexpr uint8_t kModeInternalTrigger = 0x00;
constexpr uint8_t kModeRealtime = 0x05;
constexpr uint8_t kModeStandby = 0x40;    // STANDBY bit, internal trigger
constexpr uint8_t kFeedbackLra = 0x80;
constexpr uint8_t kFeedbackDefault = 0x36;  // reset value
constexpr int kLibraryErmA = 1;
constexpr int kLibraryLra = 6;

}  // namespace

DRV2605L::DRV2605L(I2C::Port port, Actuator actuator)
    : m_i2c{port, kAddress}, m_actuator{actuator} {
  uint8_t status = 0;
  bool configured;
  {
    std::scoped_lock lock{m_mutex};
    configured = Configure(&status);
  }
  if (!configured || !m_idOk) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] DRV2605L 0x{:02X} on I2C port {}: {} (STATUS 0x{:02X})\n",
                           kAddress, static_cast<int>(port),
                           configured ? "unexpected chip" : "no answer; retrying", status);
  }
  util::ReportUsage("DRV2605L", kAddress, "");
}

DRV2605L::~DRV2605L() {
  std::scoped_lock lock{m_mutex};
  WriteRegister(kRegGo, 0);
  WriteRegister(kRegRtpInput, 0);
  WriteRegister(kRegMode, kModeStandby);  // standby stops any playback at once (8.4.1.4)
}

// m_mutex held. Checks DEVICE_ID, leaves standby, sets the actuator and its library, clears the
// RTP input and the sequencer. Returns true when every transfer was acknowledged.
bool DRV2605L::Configure(uint8_t* statusOut) {
  uint8_t status = 0;
  bool aborted = m_i2c.Read(kRegStatus, 1, &status);
  m_idOk = !aborted && (status >> 5) == kDeviceIdDrv2605L;
  if (statusOut) {
    *statusOut = status;
  }
  uint8_t feedback = kFeedbackDefault;
  aborted = aborted || m_i2c.Read(kRegFeedback, 1, &feedback);
  feedback = static_cast<uint8_t>(m_actuator == Actuator::kLRA ? (feedback | kFeedbackLra)
                                                               : (feedback & ~kFeedbackLra));
  // leave standby first: the datasheet's sequences (8.5.6, 8.5.8.2) write MODE before the rest
  aborted = aborted || m_i2c.Write(kRegMode, kModeInternalTrigger);
  aborted = aborted || m_i2c.Write(kRegFeedback, feedback);
  aborted = aborted || m_i2c.Write(kRegLibrary,
                                   m_actuator == Actuator::kLRA ? kLibraryLra : kLibraryErmA);
  aborted = aborted || m_i2c.Write(kRegRtpInput, 0);
  uint8_t sequence[1 + kNumSlots] = {kRegWaveSeq1};  // every slot 0: nothing to play
  aborted = aborted || m_i2c.WriteBulk(sequence, sizeof(sequence));
  m_connected = !aborted;
  m_configured = !aborted;
  m_realtime = false;
  m_value = 0.0;
  return !aborted;
}

// m_mutex held. Retries the configuration when it failed before (the chip did not answer).
bool DRV2605L::EnsureConfigured() {
  return m_configured || Configure();
}

// m_mutex held. WPILib I2C calls return true when the transfer was aborted.
bool DRV2605L::WriteRegister(uint8_t reg, uint8_t value) {
  bool ok = !m_i2c.Write(reg, value);
  m_connected = ok;
  return ok;
}

// m_mutex held: leaves real-time playback (internal trigger mode) before a sequencer command.
void DRV2605L::CheckIdle() {
  if (m_realtime && WriteRegister(kRegMode, kModeInternalTrigger)) {
    m_realtime = false;
    m_value = 0.0;
  }
}

void DRV2605L::SetLibrary(int library) {
  if (library < 0 || library > 7) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "DRV2605L library {} (valid: 0 to 7)",
                           library);
  }
  std::scoped_lock lock{m_mutex};
  if (EnsureConfigured()) {
    WriteRegister(kRegLibrary, static_cast<uint8_t>(library));  // HI_Z stays 0
  }
}

void DRV2605L::SetWaveform(int slot, int effect) {
  if (slot < 0 || slot >= kNumSlots) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "DRV2605L slot {} (valid: 0 to {})", slot,
                           kNumSlots - 1);
  }
  if (effect < 0 || effect > kMaxEffect) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "DRV2605L effect {} (valid: 0 to {})",
                           effect, kMaxEffect);
  }
  std::scoped_lock lock{m_mutex};
  if (EnsureConfigured()) {
    WriteRegister(static_cast<uint8_t>(kRegWaveSeq1 + slot), static_cast<uint8_t>(effect));
  }
}

void DRV2605L::Go() {
  std::scoped_lock lock{m_mutex};
  if (!EnsureConfigured()) {
    return;
  }
  CheckIdle();
  WriteRegister(kRegGo, 1);
}

void DRV2605L::Stop() {
  std::scoped_lock lock{m_mutex};
  WriteRegister(kRegGo, 0);  // clearing GO cancels the sequence (8.4.3)
  if (m_realtime) {
    WriteRegister(kRegRtpInput, 0);
    CheckIdle();
  }
}

bool DRV2605L::IsPlaying() {
  std::scoped_lock lock{m_mutex};
  uint8_t go = 0;
  bool ok = !m_i2c.Read(kRegGo, 1, &go);
  m_connected = ok;
  return ok && (go & 0x01) != 0;
}

void DRV2605L::SetRealtime(double value) {
  if (!(value > -1.0)) {
    value = std::isnan(value) ? 0.0 : -1.0;
  } else if (value > 1.0) {
    value = 1.0;
  }
  // signed format: 0x81 (-127) .. 0x00 .. 0x7F (127)
  auto code = static_cast<int8_t>(std::lround(value * 127.0));
  std::scoped_lock lock{m_mutex};
  if (!EnsureConfigured() || !WriteRegister(kRegRtpInput, static_cast<uint8_t>(code))) {
    return;
  }
  m_value = value;
  if (!m_realtime && WriteRegister(kRegMode, kModeRealtime)) {
    m_realtime = true;
  }
}

DRV2605L::Actuator DRV2605L::GetActuator() const {
  return m_actuator;
}

bool DRV2605L::IsConnected() const {
  return m_idOk && m_connected;
}

void DRV2605L::LogTo(telemetry::TelemetryTable& table) const {
  table.Log("Value", m_value.load());
  table.Log("Connected", IsConnected());
}

std::string_view DRV2605L::GetTelemetryType() const {
  return "Haptic Driver";
}
