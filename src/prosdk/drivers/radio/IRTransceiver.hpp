// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Consumer-IR link on a 38 kHz carrier (design/boardoza/P12-pulse.md): an IR LED driven by a
// prosdk::PulseTransmitter on its own carrier, and a demodulating receiver module (output low
// while it sees the carrier) read by a prosdk::PulseCapture. Written for the Boardoza IrDA
// Transceiver board (Everlight IRM-H638T receiver, Vishay VSMB10940 LED; despite the name it is
// not an IrDA SIR transceiver). Decodes the NEC remote-control format (8-bit and 16-bit
// addresses, repeat codes); any other format is available as raw pulses.
//
// Board hazard: TXD high turns the LED on through 33 ohm only, so the line must only carry
// carrier bursts (the transmitter idles low and sends high levels as 33 % carrier bursts), and
// at VCC = 5 V the RXD output swings to 5 V, which an ESP32-S3 pin does not tolerate: power the
// board from 3.3 V.

#pragma once

#include <stdint.h>

#include <deque>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "prosdk/drivers/bus/PulseCapture.hpp"
#include "prosdk/drivers/bus/PulseTransmitter.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** Format of a decoded IR frame. */
enum class IrProtocol {
  /// NEC: 8-bit address sent with its inverse.
  kNec,
  /// Extended NEC: 16-bit address (the second address byte is not the inverse of the first).
  kNecExtended
};

/** One decoded IR frame (IRTransceiver::Poll). */
struct IrFrame {
  IrProtocol protocol = IrProtocol::kNec;
  /// 0-255 for kNec; 16 bits for kNecExtended (first byte sent in the low byte).
  uint16_t address = 0;
  uint8_t command = 0;
  /// A NEC repeat code: the key is still held. Address and command are those of the frame it
  /// repeats.
  bool repeat = false;

  bool operator==(const IrFrame&) const = default;
};

/**
 * IR transmitter and receiver for 38 kHz consumer-IR remotes and short robot-to-robot links.
 *
 * Poll() reads the captured pulses, decodes complete bursts and returns one frame per call (up
 * to 16 are queued; call it every loop). The receiver also sees this board's own LED, so a
 * frame sent with SendNec() normally comes back through Poll() (unverified on the board).
 *
 * Not thread-safe except the telemetry and GetLastPulses(): send and poll from one thread.
 *
 * Resources: one of the 3 RMT TX channels (with a transmit pin) and one of the 4 RMT RX
 * channels (with a receive pin).
 */
class IRTransceiver : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Carrier frequency (the IRM-H638T's band).
  static constexpr prosdk::units::hertz_t kCarrier{38000};
  /// Carrier duty cycle; keeps the LED's average current under its rating at either VCC.
  static constexpr double kCarrierDuty = 0.33;
  /// A level at least this long ends a received burst (longer than the 9 ms NEC leader).
  static constexpr prosdk::units::microsecond_t kIdleTime{10000};
  /// Period of NEC repeat codes while a key is held, measured from frame start to frame start.
  static constexpr prosdk::units::millisecond_t kNecRepeatPeriod{108};

  /**
   * Opens the transmitter and the receiver.
   *
   * @param txChannel GPIO of TXD (the LED), or -1 for receive only.
   * @param rxChannel GPIO of RXD (the receiver output), or -1 for transmit only.
   * @throws prosdk::RuntimeError when a pin is in use or no RMT channel is free.
   */
  IRTransceiver(int txChannel, int rxChannel);

  IRTransceiver(const IRTransceiver&) = delete;
  IRTransceiver& operator=(const IRTransceiver&) = delete;

  /**
   * Sends one NEC frame (address, inverted address, command, inverted command; about 68 ms).
   * Returns without waiting for it.
   *
   * @return False without a transmit pin or after a transmitter error (reported).
   */
  bool SendNec(uint8_t address, uint8_t command);

  /**
   * Sends a NEC repeat code (about 12 ms). A held key sends one every kNecRepeatPeriod after
   * the frame; timing them is the caller's job.
   */
  bool SendNecRepeat();

  /**
   * Sends any pulse train: high levels are carrier bursts, low levels silence; each 1 to
   * 32767 us, at most 1024. Keep the bursts short (the LED's limits: P12-pulse.md).
   */
  bool SendRaw(std::span<const Pulse> pulses);

  /**
   * Reads the receiver and returns the oldest decoded frame not returned yet. Bursts that are
   * not NEC frames are dropped (GetLastPulses() shows the last one). A repeat code is returned
   * only when it follows a frame or repeat code with no other burst in between.
   *
   * @return Nothing without a receive pin or when no frame is waiting.
   */
  std::optional<IrFrame> Poll();

  /**
   * The last complete burst Poll() saw, decoded or not, as levels of the carrier: true while
   * the carrier was received. The idle level after the burst is not included, so the result
   * can be passed to SendRaw() to replay it.
   */
  std::vector<Pulse> GetLastPulses() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Decode(std::span<const Pulse> burst);

  std::optional<PulseTransmitter> m_tx;
  std::optional<PulseCapture> m_rx;

  std::vector<Pulse> m_burst;      // the burst being received (carrier levels)
  std::deque<IrFrame> m_frames;    // decoded, not returned yet
  std::optional<IrFrame> m_last;   // last frame or repeat of the current run, for repeats

  mutable std::mutex m_mutex;      // the members below (telemetry and GetLastPulses)
  std::vector<Pulse> m_lastPulses;
  IrFrame m_lastFrame;
  int64_t m_framesReceived = 0;
};

}  // namespace prosdk
