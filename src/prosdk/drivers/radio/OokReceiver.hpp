// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Fixed-code decoder for 433 MHz OOK remote controls (design/boardoza/P12-pulse.md): the
// demodulated DATA output of an OOK receiver, captured by a prosdk::PulseCapture, decoded as the
// EV1527 / PT2262 family of keyfob encoders send it. Written for the Boardoza CMT2210LH RF
// Receiver board (CMOSTEK CMT2210LH, 433.92 MHz, no configuration interface); any other OOK
// receiver with a 3.3 V data output works the same way.
//
// Frame: a sync (one short high, then a long low of about 31 short pulses), then N bits, each a
// high and a low pulse with the lengths 1:3 (bit 0) or 3:1 (bit 1) in units of the short pulse.
// Remotes send the frame repeatedly while a key is held. With no transmitter in range the
// receiver's AGC makes DATA toggle at random, so a code is only accepted after two identical
// frames in a row.
//
// Safety: OOK fixed codes have no authentication; anyone with a matching remote can replay
// them. Never use a code as an enable or an emergency stop.

#pragma once

#include <stdint.h>

#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <string_view>
#include <vector>

#include "prosdk/drivers/bus/PulseCapture.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** One received fixed code (OokReceiver::Poll). */
struct OokCode {
  /// The bits, the first received one most significant (EV1527: 20-bit ID, then the 4 data
  /// bits D0-D3; PT2262: two bits per tri-state digit, 00 = '0', 11 = '1', 01 = 'F').
  uint32_t value = 0;
  /// Number of bits, 8 to 32 (24 for EV1527 and PT2262).
  int bits = 0;
  /// The short pulse length measured over the frame (the encoder's clock; 4 LCK on an EV1527).
  prosdk::units::microsecond_t pulseLength{0};
  /// True when this frame directly follows a frame with the same code that was already
  /// returned: the key is still held.
  bool repeat = false;
};

/**
 * Decoder for EV1527 / PT2262-style OOK fixed codes on one GPIO.
 *
 * Poll() reads the captured pulses and returns one code per call: the first time when two
 * identical frames arrived in a row, then once per further frame (repeat true) while the key is
 * held. Up to 16 codes are queued; call it every loop (the capture buffer holds 256 pulses,
 * about 5 frames of 24 bits: a quarter of a second with a 350 us short pulse). The last frame of
 * a transmission counts only when a sync follows it (PT2262 sends the sync after each word,
 * EV1527 before it).
 *
 * Not thread-safe except the telemetry and GetRecentPulses(): poll from one thread.
 *
 * Resources: one of the 4 RMT RX channels.
 */
class OokReceiver : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Shortest and longest frame, in bits.
  static constexpr int kMinBits = 8;
  static constexpr int kMaxBits = 32;
  /// Short pulse lengths accepted (EV1527: 16 LCK per bit is 230 us to 3.32 ms in its
  /// oscillator table, 47 kOhm to 680 kOhm at 3 to 13 V, so a short pulse is 58 to 830 us).
  static constexpr prosdk::units::microsecond_t kMinPulseLength{50};
  static constexpr prosdk::units::microsecond_t kMaxPulseLength{1000};
  /// Pulses kept for GetRecentPulses().
  static constexpr int kRecentPulses = 256;

  /**
   * Opens the capture on the receiver's data pin.
   *
   * @param channel The GPIO of DATA.
   * @throws prosdk::RuntimeError when the pin is in use or no RMT RX channel is free.
   */
  explicit OokReceiver(int channel);

  OokReceiver(const OokReceiver&) = delete;
  OokReceiver& operator=(const OokReceiver&) = delete;

  /**
   * Reads the receiver and returns the oldest code not returned yet.
   *
   * @return Nothing when no code is waiting.
   */
  std::optional<OokCode> Poll();

  /**
   * The last pulses Poll() read (up to kRecentPulses, oldest first; true = high, a pulse of
   * 0 us means the line stayed at that level for 32.767 ms or longer). For protocols this class
   * does not decode.
   */
  std::vector<Pulse> GetRecentPulses() const;

  /** The GPIO. */
  int GetChannel() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Feed(const Pulse& p);
  void OnGap(uint32_t gapUs);
  std::optional<OokCode> DecodeFrame(uint32_t trailingGapUs) const;
  void BreakRun();

  PulseCapture m_rx;

  std::vector<Pulse> m_frame;      // pulses since the last sync gap (the sync high last)
  bool m_framed = false;           // m_frame started at a sync gap
  uint32_t m_leadingGapUs = 0;     // that gap
  std::optional<uint64_t> m_prevKey;  // code of the previous frame of the current run
  bool m_reported = false;         // the run's code was returned
  std::deque<OokCode> m_codes;     // accepted, not returned yet

  mutable std::mutex m_mutex;      // the members below (telemetry and GetRecentPulses)
  std::array<Pulse, kRecentPulses> m_recent{};
  int m_recentNext = 0;
  int m_recentCount = 0;
  OokCode m_lastCode;
  int64_t m_codesReceived = 0;
};

}  // namespace prosdk
