// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// u-blox M8 / M10 GNSS receiver (SAM-M8Q, SAM-M10Q and other u-blox modules) over prosdk::SerialPort
// or prosdk::I2C (DDC), reading UBX NAV-PVT (design/boardoza/P7-gnss.md). A thread asks the
// receiver for its version (UBX-MON-VER), configures periodic NAV-PVT output with the legacy
// messages on u-blox 8 (protocol up to 23.01) or with UBX-CFG-VALSET on newer receivers (M10:
// protocol 34), turns NMEA output off on the port in use, and parses the stream.
//
// I2C (M8 Receiver Description UBX-13003221 R28, section 11.5): registers 0xFD (high byte) and
// 0xFE (low byte) hold the number of bytes waiting, register 0xFF streams them; writes are raw UBX
// frames of at least 2 bytes. The M10 Interface Description does not describe these registers; the
// driver assumes the same model on the M10 (unverified). Default address 0x42 (M8 appendix
// C.15.4; M10 CFG-I2C-ADDRESS default 0x84, read as the 8-bit form of 0x42).

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "prosdk/drivers/gnss/GnssFix.hpp"
#include "prosdk/drivers/gnss/Ubx.hpp"
#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/hardware/bus/SerialPort.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * u-blox GNSS receiver reading UBX NAV-PVT.
 *
 * The configuration is written to the receiver's RAM only (no backup battery on the Boardoza
 * boards), so the driver repeats it whenever NAV-PVT stops arriving, for example after a power
 * cycle of the receiver. Construction never blocks; GetFix() reports no fix until the receiver
 * answers and sees the sky (a cold start takes about 25 s outdoors).
 */
class UbloxGnss : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// UART default of the u-blox receivers (SAM-M8Q data sheet table 9, SAM-M10Q table 18).
  static constexpr int kDefaultBaudRate = 9600;
  /// Default 7-bit I2C (DDC) address.
  static constexpr int kDefaultAddress = 0x42;
  /// Default period of the reader thread.
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{20};
  /// Navigation rate set at start (the receivers' default).
  static constexpr prosdk::units::hertz_t kDefaultRate{1};

  /**
   * Opens a serial port and starts the reader thread.
   *
   * @param port         The serial port (prosdk::platform::SetSerialPins sets its pins).
   * @param baudRate     The receiver's UART baud rate (the driver does not change it).
   * @param updatePeriod Period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit UbloxGnss(prosdk::SerialPort::Port port, int baudRate = kDefaultBaudRate,
                     prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);

  /**
   * Uses an I2C port and starts the reader thread. The SAM-M10Q allows at most 320 kHz
   * (data sheet section 5.2): set a lower clock with prosdk::platform::SetI2CPins.
   *
   * @param port         The I2C port.
   * @param address      The receiver's 7-bit address.
   * @param updatePeriod Period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit UbloxGnss(prosdk::I2C::Port port, int address = kDefaultAddress,
                     prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~UbloxGnss() override;

  UbloxGnss(const UbloxGnss&) = delete;
  UbloxGnss& operator=(const UbloxGnss&) = delete;

  /** The latest NAV-PVT solution (valid false before the first fix). */
  GnssFix GetFix() const;

  /**
   * Sets the navigation rate (UBX measurement period, 50 ms to 65.535 s). The receivers' limits
   * apply: a u-blox M8 does up to 10 Hz with GPS + GLONASS, the SAM-M10Q 4 Hz with its default
   * four constellations; a rate the receiver refuses is reported once on the console. At 9600 baud
   * one NAV-PVT frame (100 bytes) takes about 104 ms on the wire: stay at 5 Hz or below there.
   */
  void SetRate(prosdk::units::hertz_t rate);

  /** True when a UBX message arrived within the timeout (2 s, or 3 navigation periods). */
  bool IsConnected() const;

  /** True when the receiver acknowledged the NAV-PVT configuration. */
  bool IsConfigured() const;

  /** Reads what the receiver sent, parses it and advances the configuration (the thread's work). */
  void Update();

  /** Parses received bytes (Update() calls this). Frames may span calls. */
  void Feed(std::span<const uint8_t> bytes);

  /** UBX frames dropped because of a checksum mismatch or a bad length. */
  uint32_t GetChecksumErrors() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  enum class Stage { kProbe, kConfigure, kRunning };

  void Start(prosdk::units::millisecond_t updatePeriod);
  void Run();
  void ReadSerial();
  void ReadI2C();
  void HandleMessage(const prosdk::ubx::Parser::Message& message, double now);
  std::vector<std::vector<uint8_t>> Step(double now);
  void Send(const std::vector<uint8_t>& frame);
  double TimeoutSeconds() const;

  std::optional<prosdk::SerialPort> m_serial;
  std::optional<prosdk::I2C> m_i2c;
  int m_baudRate = 0;
  int m_address = 0;
  std::string m_name;  // for console messages
  prosdk::units::millisecond_t m_updatePeriod{0};

  mutable std::mutex m_parseMutex;  // m_parser
  prosdk::ubx::Parser m_parser;

  mutable std::mutex m_mutex;  // everything below
  Stage m_stage = Stage::kProbe;
  bool m_configInterface = false;  // receiver takes UBX-CFG-VALSET (protocol 24 and later)
  int m_periodMs = 1000;
  uint8_t m_pendingAcks = 0;       // configuration messages not yet acknowledged
  bool m_configSent = false;
  int m_attempts = 0;
  double m_lastSend = 0.0;         // s, monotonic
  double m_lastFrame = -1.0;       // s, any UBX frame (-1: none yet)
  double m_lastPvt = 0.0;          // s, NAV-PVT (or the end of the configuration)
  bool m_warnedSilent = false;
  bool m_warnedNak = false;
  GnssFix m_fix;

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
