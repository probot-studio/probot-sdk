// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// PYD1598 driver (PYD1598.hpp). Source: Excelitas "PYD 1588 / 7656 low power DigiPyro" product
// specification (issued 20/03/2018, revised 06/04/2021): Table 4 (SERIN timing), sections 2.4
// (motion detection unit, clearing DL), 2.5 (Table 5, configuration register), 2.6 (Serial In
// interface) and 2.7 (wake-up mode). The PYD 1598 document itself could not be downloaded
// (excelitas.com refused it); Excelitas' low-power DigiPyro brochure lists PYD 1588 and PYD 1598
// with the same electrical data and the same configurable functions, differing in window size
// and field of view, so the PYD 1588 register map and timing are used here (unverified on a
// PYD 1598).

#include "prosdk/drivers/sensor/PYD1598.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <thread>
#include <vector>

#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// Table 5 fields (bit positions) and fixed values for wake-up mode
constexpr int kThresholdShift = 17;    // [24:17]
constexpr int kBlindTimeShift = 13;    // [16:13] 0.5 s + n * 0.5 s
constexpr int kPulseCountShift = 11;   // [12:11] 1 + n
constexpr int kWindowTimeShift = 9;    // [10:9]  2 s + n * 2 s
constexpr int kModeShift = 7;          // [8:7]   2 = wake up
constexpr int kSourceShift = 5;        // [6:5]   0 = PIR (BPF)
constexpr int kReservedShift = 3;      // [4:3]   must be 2
constexpr int kHpfShift = 2;           // [2]     0 = 0.4 Hz, 1 = 0.2 Hz
                                       // [1]     reserved, must be 0
constexpr int kCountModeShift = 0;     // [0]     1 = count without BPF sign change
constexpr uint32_t kModeWakeUp = 2;
constexpr uint32_t kSourcePirBpf = 0;
constexpr uint32_t kReservedValue = 2;
constexpr int kWordBits = 25;

// SERIN timing (Table 4, section 2.6), us: each bit is a low-to-high transition (tSL and tSH,
// 0.2-2 us each) followed by the data level held for tSHD (80-150 us); after the 25th bit SERIN
// stays low for more than tSLT (650 us) to latch the word, and a new word is accepted only after
// SERIN was low for tSLT.
constexpr uint32_t kClockLowUs = 1;
constexpr uint32_t kClockHighUs = 1;
constexpr uint32_t kHoldUs = 100;
constexpr uint32_t kLoadUs = 700;

constexpr units::second_t kSendTimeout{0.1};

void Append(std::vector<Pulse>& train, bool level, uint32_t us) {
  if (!train.empty() && train.back().level == level) {
    train.back().us += us;
  } else {
    train.push_back({level, us});
  }
}

std::vector<Pulse> SerinTrain(uint32_t word) {
  std::vector<Pulse> train;
  Append(train, false, kLoadUs);
  for (int bit = kWordBits - 1; bit >= 0; --bit) {  // MSB (bit 24) first
    Append(train, false, kClockLowUs);
    Append(train, true, kClockHighUs);
    Append(train, ((word >> bit) & 1u) != 0, kHoldUs);
  }
  Append(train, false, kLoadUs);
  return train;
}

// Register value of a time setting `first + n * step` (n = 0 .. maxN); rounds to the nearest step.
uint32_t TimeField(units::second_t time, double first, double step, uint32_t maxN,
                   const char* name) {
  double s = time.value();
  double last = first + maxN * step;
  if (!(s >= first - 1e-9 && s <= last + 1e-9)) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "PYD1598 {} {} s (valid: {} to {} s)", name,
                           s, first, last);
  }
  return static_cast<uint32_t>(std::lround((s - first) / step));
}

}  // namespace

PYD1598::PYD1598(int serinChannel, int dlChannel)
    : PYD1598(serinChannel, dlChannel, Config{}) {}

PYD1598::PYD1598(int serinChannel, int dlChannel, const Config& config)
    : m_dlChannel{dlChannel},
      m_config{config},
      m_word{EncodeConfig(config)},
      m_serin{serinChannel} {
  m_dl.emplace(dlChannel);
  util::ReportUsage("PYD1598", serinChannel, "");
  Configure();
}

uint32_t PYD1598::EncodeConfig(const Config& config) {
  if (config.threshold < 0 || config.threshold > 255) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "PYD1598 threshold {} (valid: 0 to 255)",
                           config.threshold);
  }
  if (config.pulseCount < 1 || config.pulseCount > 4) {
    throw WPILIB_MakeError(err::ParameterOutOfRange, "PYD1598 pulse count {} (valid: 1 to 4)",
                           config.pulseCount);
  }
  uint32_t blind = TimeField(config.blindTime, 0.5, 0.5, 15, "blind time");
  uint32_t window = TimeField(config.windowTime, 2.0, 2.0, 3, "window time");
  return (static_cast<uint32_t>(config.threshold) << kThresholdShift) |
         (blind << kBlindTimeShift) |
         (static_cast<uint32_t>(config.pulseCount - 1) << kPulseCountShift) |
         (window << kWindowTimeShift) | (kModeWakeUp << kModeShift) |
         (kSourcePirBpf << kSourceShift) | (kReservedValue << kReservedShift) |
         ((config.lowHighPassCutoff ? 1u : 0u) << kHpfShift) |
         ((config.countWithoutSignChange ? 1u : 0u) << kCountModeShift);
}

bool PYD1598::Configure() {
  bool ok = HoldDlLow(m_word, true);
  if (!ok) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] PYD1598 on SERIN GPIO {}: configuration not sent\n",
                           m_serin.GetChannel());
  }
  std::scoped_lock lock{m_mutex};
  m_configured = ok;
  return ok;
}

bool PYD1598::Configure(const Config& config) {
  uint32_t word = EncodeConfig(config);
  m_config = config;
  m_word = word;
  return Configure();
}

const PYD1598::Config& PYD1598::GetConfig() const {
  return m_config;
}

bool PYD1598::Poll() {
  bool motion = m_dl && m_dl->Get();
  std::scoped_lock lock{m_mutex};
  m_motion = motion;
  return motion;
}

void PYD1598::Clear() {
  HoldDlLow(m_word, false);
  std::scoped_lock lock{m_mutex};
  m_motion = false;
}

bool PYD1598::HoldDlLow(uint32_t word, bool send) {
  // DL must be low while the word is clocked in (section 2.6) and low for at least 160 us to
  // clear an event (section 2.4); the host input must be high impedance otherwise (2.7)
  bool ok = true;
  m_dl.reset();
  try {
    DigitalOutput dl{m_dlChannel};
    dl.Set(false);
    if (send) {
      std::vector<Pulse> train = SerinTrain(word);
      ok = m_serin.Send(train) && m_serin.WaitSent(kSendTimeout);
    }
    std::this_thread::sleep_for(
        std::chrono::microseconds{static_cast<int64_t>(kClearTime.value())});
  } catch (const std::exception& e) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] PYD1598: DL GPIO {} could not be driven low: {}\n",
                           m_dlChannel, e.what());
    ok = false;
  }
  m_dl.emplace(m_dlChannel);
  if (m_dl->Get() && !m_warnedFloating) {
    // motion detection is stopped for the blind time after a clear, so DL should read low here
    // unless the sensor leaves it floating against the input's internal pull-up (unverified)
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] PYD1598: DL GPIO {} reads high right after a clear; see "
                           "design/boardoza/P12-pulse.md (DL pull-up)\n",
                           m_dlChannel);
    m_warnedFloating = true;
  }
  return ok;
}

void PYD1598::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Motion", m_motion);
  table.Log("Configured", m_configured);
}

std::string_view PYD1598::GetTelemetryType() const {
  return "Motion Sensor";
}
