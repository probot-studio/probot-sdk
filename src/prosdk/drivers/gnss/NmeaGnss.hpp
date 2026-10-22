// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Any GNSS receiver that streams NMEA 0183 over a UART (Ai-Thinker GP-02 on the Boardoza GP-02
// board, and u-blox receivers left at their defaults), read through prosdk::SerialPort
// (design/boardoza/P7-gnss.md). Listen-only: nothing is sent to the receiver. The fix is built
// from RMC (time, date, status, position, speed, course), GGA (quality, satellites, HDOP,
// altitude) and GSA (2D/3D).

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>

#include "prosdk/drivers/gnss/GnssFix.hpp"
#include "prosdk/drivers/gnss/Nmea.hpp"
#include "prosdk/hardware/bus/SerialPort.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * NMEA GNSS receiver on a serial port.
 *
 * fixType comes from the latest GSA sentence; a receiver that sends no GSA reports k2D for any
 * valid fix. hAcc is not available from NMEA (NaN); hdop comes from GGA.
 */
class NmeaGnss : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default baud rate: unverified (the Boardoza GP-02 example uses it; the GP-02 data sheet
  /// gives only "max support 256000bps").
  static constexpr int kDefaultBaudRate = 9600;
  /// Default period of the reader thread (the 1 KiB UART buffer holds about 1 s at 9600 baud).
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{20};
  /// A receiver without a valid sentence for this long is reported as not connected.
  static constexpr prosdk::units::millisecond_t kTimeout{2000};

  /**
   * Opens a serial port and starts the reader thread.
   *
   * @param port         The serial port (prosdk::platform::SetSerialPins sets its pins).
   * @param baudRate     The receiver's baud rate.
   * @param updatePeriod Period of the reader thread; 0 opens no port and starts no thread (feed
   *                     bytes with Feed(), for tests and other transports).
   *
   * The thread prints one console warning when no valid sentence arrived within 5 s.
   */
  explicit NmeaGnss(prosdk::SerialPort::Port port, int baudRate = kDefaultBaudRate,
                    prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~NmeaGnss() override;

  NmeaGnss(const NmeaGnss&) = delete;
  NmeaGnss& operator=(const NmeaGnss&) = delete;

  /** The latest solution (valid false before the first fix). */
  GnssFix GetFix() const;

  /** True when a sentence with a good checksum arrived within kTimeout. */
  bool IsConnected() const;

  /** Parses received bytes (the reader thread calls this). Sentences may span calls. */
  void Feed(std::span<const uint8_t> bytes);

  /** Sentences that passed the checksum. */
  uint32_t GetSentenceCount() const;
  /** Sentences dropped (bad or missing checksum, overlong, stray control bytes). */
  uint32_t GetChecksumErrors() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void Run();
  void Handle(std::string_view body, double now);

  std::optional<prosdk::SerialPort> m_serial;
  std::string m_name;  // for console messages
  prosdk::units::millisecond_t m_updatePeriod;

  mutable std::mutex m_parseMutex;  // m_reader
  prosdk::nmea::Reader m_reader;

  mutable std::mutex m_mutex;  // everything below
  GnssFix m_fix;
  int m_gsaMode = 0;             // navMode of the latest GSA (0: none seen)
  bool m_deadReckoning = false;  // latest GGA quality 6
  double m_lastSentence = -1.0;  // s, monotonic (-1: none yet)

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
