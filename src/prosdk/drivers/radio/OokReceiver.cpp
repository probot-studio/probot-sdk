// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// OokReceiver (OokReceiver.hpp). Frame format: EV1527 data sheet (Shenzhen Silvan, Chinese,
// section 2.3): sync, C0-C19, D0-D3; data 1 = 12 LCK high + 4 LCK low, data 0 = 4 LCK high +
// 12 LCK low, 1 LCK = 8 oscillator clocks, 16 LCK = 230 us-3.32 ms (oscillator table). The sync's
// low length is drawn with a break in that figure; about 124 LCK (31 short pulses) is the
// rc-switch "protocol 1" practice, and PT2262 (12 tri-state digits of two such bits each, then a
// sync of 4 alpha high + 124 alpha low) was taken from a secondary copy of its data sheet: both
// are unverified. The decoder therefore only requires the gap to be much longer than the sync
// high (at least 8 short pulses) and checks every bit's 1:3 / 3:1 shape. CMT2210LH data sheet
// v0.5: DATA is the demodulated OOK output (high while RF is received is assumed, unverified).

#include "prosdk/drivers/radio/OokReceiver.hpp"

#include <algorithm>

#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// The longest captured level; anything longer arrives as a 0 us end-of-burst marker.
constexpr units::microsecond_t kIdleTime{static_cast<double>(platform::hal::kMaxRxIdleUs)};
// A low this long, and at least kGapRatio times the high before it, is a sync gap.
constexpr uint32_t kMinGapUs = 1500;
constexpr uint32_t kGapRatio = 8;
// Bit shape limits, in short pulses: high + low (nominal 4) and long/short (nominal 3).
constexpr double kMinPeriod = 2.5;
constexpr double kMaxPeriod = 5.5;
constexpr double kMinRatio = 1.6;
// The sync high, in short pulses (nominal 1).
constexpr double kMaxSyncHigh = 2.5;
constexpr size_t kMaxQueued = 16;

}  // namespace

OokReceiver::OokReceiver(int channel)
    : m_rx{channel, kIdleTime,
           units::microsecond_t{static_cast<double>(platform::hal::kMaxRxGlitchUs)}} {
  util::ReportUsage("OokReceiver", channel, "");
}

std::optional<OokCode> OokReceiver::Poll() {
  std::array<Pulse, 64> buf;
  for (int n; (n = m_rx.Read(buf)) > 0;) {
    {
      std::scoped_lock lock{m_mutex};
      for (int i = 0; i < n; ++i) {
        m_recent[m_recentNext] = buf[i];
        m_recentNext = (m_recentNext + 1) % kRecentPulses;
        m_recentCount = std::min(m_recentCount + 1, kRecentPulses);
      }
    }
    for (int i = 0; i < n; ++i) {
      Feed(buf[i]);
    }
  }
  if (m_codes.empty()) {
    return std::nullopt;
  }
  OokCode code = m_codes.front();
  m_codes.pop_front();
  return code;
}

void OokReceiver::Feed(const Pulse& p) {
  if (p.us == 0) {
    // the line stayed at one level for the whole idle time: low ends a frame like a sync gap
    // (PT2262 sends its sync after the word), high is no frame of this format
    if (!p.level && !m_frame.empty() && m_frame.back().level) {
      OnGap(0);
    } else {
      BreakRun();
      m_frame.clear();
      m_framed = false;
    }
    return;
  }
  if (!p.level && p.us >= kMinGapUs && !m_frame.empty() && m_frame.back().level &&
      p.us >= kGapRatio * m_frame.back().us) {
    OnGap(p.us);
    return;
  }
  m_frame.push_back(p);
  if (m_frame.size() > static_cast<size_t>(2 * kMaxBits + 1)) {
    // too long for a frame: noise (keep the last pulse, it may be the next sync high)
    if (m_framed) {
      BreakRun();
    }
    m_frame.erase(m_frame.begin(), m_frame.end() - 1);
    m_framed = false;
  }
}

void OokReceiver::OnGap(uint32_t gapUs) {
  if (m_framed) {
    if (auto code = DecodeFrame(gapUs)) {
      uint64_t key = code->value | (static_cast<uint64_t>(code->bits) << 32);
      if (m_prevKey == key) {
        // confirmed by the frame before it
        code->repeat = m_reported;
        m_reported = true;
        if (m_codes.size() >= kMaxQueued) {
          m_codes.pop_front();
        }
        m_codes.push_back(*code);
        std::scoped_lock lock{m_mutex};
        m_lastCode = *code;
        ++m_codesReceived;
      } else {
        m_prevKey = key;
        m_reported = false;
      }
    } else {
      BreakRun();
    }
  }
  m_frame.clear();
  m_framed = true;
  m_leadingGapUs = gapUs;
}

std::optional<OokCode> OokReceiver::DecodeFrame(uint32_t trailingGapUs) const {
  // m_frame: high, low, ..., high, low, then the sync high of the gap that ends it
  if (m_frame.size() % 2 != 1) {
    return std::nullopt;
  }
  int bits = static_cast<int>(m_frame.size() / 2);
  if (bits < kMinBits || bits > kMaxBits) {
    return std::nullopt;
  }
  double total = 0.0;
  for (int i = 0; i < 2 * bits; ++i) {
    total += m_frame[i].us;
  }
  double unit = total / (4.0 * bits);  // the short pulse: 4 per bit
  if (unit < kMinPulseLength.value() || unit > kMaxPulseLength.value()) {
    return std::nullopt;
  }
  if (m_frame.back().us > kMaxSyncHigh * unit) {
    return std::nullopt;
  }
  // a 0 us gap lasted the idle time (32.767 ms) or longer
  for (uint32_t gap : {m_leadingGapUs, trailingGapUs}) {
    if (gap != 0 && gap < kGapRatio * unit) {
      return std::nullopt;
    }
  }
  uint32_t value = 0;
  for (int bit = 0; bit < bits; ++bit) {
    const Pulse& high = m_frame[2 * bit];
    const Pulse& low = m_frame[2 * bit + 1];
    if (!high.level || low.level) {
      return std::nullopt;
    }
    double period = static_cast<double>(high.us) + low.us;
    double longer = std::max(high.us, low.us);
    double shorter = std::min(high.us, low.us);
    if (period < kMinPeriod * unit || period > kMaxPeriod * unit ||
        longer < kMinRatio * shorter) {
      return std::nullopt;
    }
    value = (value << 1) | (high.us > low.us ? 1u : 0u);
  }
  OokCode code;
  code.value = value;
  code.bits = bits;
  code.pulseLength = units::microsecond_t{unit};
  return code;
}

void OokReceiver::BreakRun() {
  m_prevKey.reset();
  m_reported = false;
}

std::vector<Pulse> OokReceiver::GetRecentPulses() const {
  std::scoped_lock lock{m_mutex};
  std::vector<Pulse> out;
  out.reserve(m_recentCount);
  int start = (m_recentNext - m_recentCount + kRecentPulses) % kRecentPulses;
  for (int i = 0; i < m_recentCount; ++i) {
    out.push_back(m_recent[(start + i) % kRecentPulses]);
  }
  return out;
}

int OokReceiver::GetChannel() const {
  return m_rx.GetChannel();
}

void OokReceiver::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Code", static_cast<int64_t>(m_lastCode.value));
  table.Log("Bits", static_cast<int32_t>(m_lastCode.bits));
  table.Log("Codes", m_codesReceived);
}

std::string_view OokReceiver::GetTelemetryType() const {
  return "OOK Receiver";
}
