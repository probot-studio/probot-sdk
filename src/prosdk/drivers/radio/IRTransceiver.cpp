// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// IRTransceiver (IRTransceiver.hpp). Receiver: Everlight IRM-H6xxT data sheet DMO-0000008 Rev. 3
// (output active low while the carrier is present; output pulse width 400-800 us for a 600 us
// burst, so the decoder allows about +-250 us on every level). NEC format: 9 ms leader burst,
// 4.5 ms space, 32 bits LSB first (address, inverted address or high address byte, command,
// inverted command), each bit a 562.5 us burst followed by a 562.5 us (0) or 1687.5 us (1)
// space, then a 562.5 us stop burst; a repeat code is a 9 ms burst, a 2.25 ms space and a
// 562.5 us burst. These NEC values are the format as commonly documented (NEC uPD6121 family);
// no NEC document was obtained, so they are unverified.

#include "prosdk/drivers/radio/IRTransceiver.hpp"

#include <array>

#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {

// NEC timing, us (rounded to the 1 us pulse resolution)
constexpr uint32_t kLeaderMark = 9000;
constexpr uint32_t kLeaderSpace = 4500;
constexpr uint32_t kRepeatSpace = 2250;
constexpr uint32_t kBitMark = 562;
constexpr uint32_t kZeroSpace = 562;
constexpr uint32_t kOneSpace = 1687;

constexpr size_t kFramePulses = 2 + 64 + 1;  // leader, 32 bits, stop burst
constexpr size_t kRepeatPulses = 3;
constexpr size_t kMaxBurst = 200;            // longer bursts are noise for this decoder
constexpr size_t kMaxQueued = 16;

// Accepted ranges, us: the nominal value with the receiver's distortion and some margin
constexpr bool InRange(uint32_t us, uint32_t lo, uint32_t hi) {
  return us >= lo && us <= hi;
}
constexpr bool IsLeaderMark(uint32_t us) {
  return InRange(us, 7500, 10500);
}
constexpr bool IsLeaderSpace(uint32_t us) {
  return InRange(us, 3700, 5300);
}
constexpr bool IsRepeatSpace(uint32_t us) {
  return InRange(us, 1700, 2800);
}
constexpr bool IsBitMark(uint32_t us) {
  return InRange(us, 250, 1000);
}
constexpr bool IsZeroSpace(uint32_t us) {
  return InRange(us, 250, 950);
}
constexpr bool IsOneSpace(uint32_t us) {
  return InRange(us, 1300, 2200);
}

}  // namespace

IRTransceiver::IRTransceiver(int txChannel, int rxChannel) {
  if (txChannel >= 0) {
    m_tx.emplace(txChannel, kCarrier, kCarrierDuty);
  }
  if (rxChannel >= 0) {
    // the receiver output is clean (no RF noise): only the shortest glitches are filtered
    m_rx.emplace(rxChannel, kIdleTime,
                 units::microsecond_t{static_cast<double>(platform::hal::kMaxRxGlitchUs)});
  }
  util::ReportUsage("IRTransceiver", txChannel >= 0 ? txChannel : rxChannel, "");
}

bool IRTransceiver::SendNec(uint8_t address, uint8_t command) {
  std::array<Pulse, kFramePulses> train;
  size_t n = 0;
  train[n++] = {true, kLeaderMark};
  train[n++] = {false, kLeaderSpace};
  uint32_t word = address | (static_cast<uint32_t>(static_cast<uint8_t>(~address)) << 8) |
                  (static_cast<uint32_t>(command) << 16) |
                  (static_cast<uint32_t>(static_cast<uint8_t>(~command)) << 24);
  for (int bit = 0; bit < 32; ++bit) {  // LSB first
    train[n++] = {true, kBitMark};
    train[n++] = {false, ((word >> bit) & 1u) != 0 ? kOneSpace : kZeroSpace};
  }
  train[n++] = {true, kBitMark};
  return SendRaw(train);
}

bool IRTransceiver::SendNecRepeat() {
  std::array<Pulse, kRepeatPulses> train{
      {{true, kLeaderMark}, {false, kRepeatSpace}, {true, kBitMark}}};
  return SendRaw(train);
}

bool IRTransceiver::SendRaw(std::span<const Pulse> pulses) {
  if (!m_tx) {
    platform::ConsolePrint(platform::ConsoleLevel::kWarning,
                           "[prosdk] IRTransceiver: no transmit pin (txChannel -1)\n");
    return false;
  }
  return m_tx->Send(pulses);
}

std::optional<IrFrame> IRTransceiver::Poll() {
  if (m_rx) {
    std::array<Pulse, 64> buf;
    for (int n; (n = m_rx->Read(buf)) > 0;) {
      for (int i = 0; i < n; ++i) {
        // RXD is low while the carrier is received: invert to carrier levels
        Pulse p{!buf[i].level, buf[i].us};
        if (p.us == 0) {  // the line idled: the burst is complete
          if (!m_burst.empty()) {
            Decode(m_burst);
            m_burst.clear();
          }
        } else if (m_burst.size() < kMaxBurst) {
          m_burst.push_back(p);
        }
      }
    }
  }
  if (m_frames.empty()) {
    return std::nullopt;
  }
  IrFrame frame = m_frames.front();
  m_frames.pop_front();
  return frame;
}

void IRTransceiver::Decode(std::span<const Pulse> burst) {
  {
    std::scoped_lock lock{m_mutex};
    m_lastPulses.assign(burst.begin(), burst.end());
  }
  std::optional<IrFrame> frame;
  // every burst starts with a carrier level (the first edge after the idle line)
  if (burst.size() == kRepeatPulses && burst[0].level && IsLeaderMark(burst[0].us) &&
      IsRepeatSpace(burst[1].us) && IsBitMark(burst[2].us)) {
    if (m_last) {
      frame = *m_last;
      frame->repeat = true;
    }
  } else if (burst.size() == kFramePulses && burst[0].level && IsLeaderMark(burst[0].us) &&
             IsLeaderSpace(burst[1].us) && IsBitMark(burst.back().us)) {
    uint32_t word = 0;
    bool ok = true;
    for (int bit = 0; bit < 32 && ok; ++bit) {
      const Pulse& mark = burst[2 + 2 * bit];
      const Pulse& space = burst[3 + 2 * bit];
      if (!IsBitMark(mark.us)) {
        ok = false;
      } else if (IsOneSpace(space.us)) {
        word |= uint32_t{1} << bit;
      } else if (!IsZeroSpace(space.us)) {
        ok = false;
      }
    }
    auto a0 = static_cast<uint8_t>(word);
    auto a1 = static_cast<uint8_t>(word >> 8);
    auto cmd = static_cast<uint8_t>(word >> 16);
    auto cmdInv = static_cast<uint8_t>(word >> 24);
    if (ok && static_cast<uint8_t>(~cmd) == cmdInv) {
      IrFrame f;
      f.command = cmd;
      if (static_cast<uint8_t>(~a0) == a1) {
        f.protocol = IrProtocol::kNec;
        f.address = a0;
      } else {
        f.protocol = IrProtocol::kNecExtended;
        f.address = static_cast<uint16_t>(a0 | (a1 << 8));
      }
      frame = f;
    }
  }

  if (!frame) {
    m_last.reset();  // anything else in between ends a run of repeats
    return;
  }
  m_last = *frame;
  m_last->repeat = false;
  if (m_frames.size() >= kMaxQueued) {
    m_frames.pop_front();
  }
  m_frames.push_back(*frame);
  std::scoped_lock lock{m_mutex};
  m_lastFrame = *frame;
  ++m_framesReceived;
}

std::vector<Pulse> IRTransceiver::GetLastPulses() const {
  std::scoped_lock lock{m_mutex};
  return m_lastPulses;
}

void IRTransceiver::LogTo(telemetry::TelemetryTable& table) const {
  std::scoped_lock lock{m_mutex};
  table.Log("Address", static_cast<int32_t>(m_lastFrame.address));
  table.Log("Command", static_cast<int32_t>(m_lastFrame.command));
  table.Log("Frames", m_framesReceived);
}

std::string_view IRTransceiver::GetTelemetryType() const {
  return "IR Transceiver";
}
