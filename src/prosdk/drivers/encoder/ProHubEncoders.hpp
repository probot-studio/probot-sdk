// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ProHub encoder stream over prosdk::SerialPort (design/drivers.md, "ProHub encoders"). The
// ProHub co-processor counts 8 quadrature encoders and sends a 108-byte frame (wire format of
// Probot Lib 0.2.6, probot/prohub/encoder_frame.hpp):
//
//   AA 55 | seq u8 | timestamp_us u32 | dt_us u32 | 8 x total ticks i64 | 8 x delta ticks i32 |
//   crc u8      (little endian; crc = XOR of the 107 bytes before it, header included)

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <thread>

#include "prosdk/hardware/bus/SerialPort.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** Reader of the ProHub encoder stream (one per serial port). */
class ProHubEncoders {
 public:
  /// Encoder channels per frame.
  static constexpr int kNumEncoders = 8;
  /// Frame size in bytes, header and CRC included.
  static constexpr int kFrameSize = 108;
  /// Default baud rate of the stream.
  static constexpr int kDefaultBaudRate = 921600;
  /// Default period of the reader thread (the 1 KiB UART buffer holds about 11 ms at 921600).
  static constexpr prosdk::units::millisecond_t kDefaultPollPeriod{2};
  /// A stream without a valid frame for this long is reported as not connected.
  static constexpr prosdk::units::millisecond_t kTimeout{100};

  /** One decoded frame. */
  struct Frame {
    uint8_t seq = 0;            ///< sequence number (wraps at 256)
    uint32_t timestampUs = 0;   ///< co-processor time of the sample
    uint32_t dtUs = 0;          ///< time since the previous frame
    std::array<int64_t, kNumEncoders> totalTicks{};  ///< counts since co-processor start
    std::array<int32_t, kNumEncoders> deltaTicks{};  ///< counts since the previous frame
  };

  /**
   * Opens the serial port and starts the reader thread.
   *
   * @param port       The serial port (prosdk::platform::SetSerialPins sets its pins;
   *                   prosdk::prohub::ConfigurePins sets the ProHub's).
   * @param baudRate   The stream's baud rate.
   * @param pollPeriod Period of the reader thread; 0 opens no port and starts no thread (feed
   *                   bytes with Feed(), for tests and other transports).
   */
  explicit ProHubEncoders(prosdk::SerialPort::Port port = prosdk::SerialPort::Port::ONBOARD,
                          int baudRate = kDefaultBaudRate,
                          prosdk::units::millisecond_t pollPeriod = kDefaultPollPeriod);
  ~ProHubEncoders();

  ProHubEncoders(const ProHubEncoders&) = delete;
  ProHubEncoders& operator=(const ProHubEncoders&) = delete;

  /** Parses received bytes (the reader thread calls this). Frames may span calls. */
  void Feed(std::span<const uint8_t> bytes);

  /** The latest valid frame, if any. */
  std::optional<Frame> GetFrame() const;

  /** Total ticks of an encoder in the latest frame (0 before the first frame). */
  int64_t GetTicks(int index) const;

  /** Ticks per second of an encoder over the latest frame interval (0 before two frames). */
  double GetTicksPerSecond(int index) const;

  /** True when a valid frame arrived within kTimeout. */
  bool IsConnected() const;

  /** Valid frames received. */
  uint32_t GetFrameCount() const;
  /** Frames dropped because of a CRC mismatch. */
  uint32_t GetCrcErrors() const;
  /** Bytes received. */
  uint32_t GetBytesReceived() const;

  /** Builds a frame's wire bytes (CRC included); for tests and simulators. */
  static std::array<uint8_t, kFrameSize> Encode(const Frame& frame);

 private:
  void CheckIndex(int index) const;
  void Run();

  std::optional<prosdk::SerialPort> m_serial;
  prosdk::units::millisecond_t m_pollPeriod;

  // parser (Feed callers: the reader thread or the user; m_parseMutex)
  std::mutex m_parseMutex;
  std::array<uint8_t, kFrameSize> m_buf{};
  int m_len = 0;

  mutable std::mutex m_mutex;  // m_frame, m_frameTime
  std::optional<Frame> m_frame;
  double m_frameTime = 0.0;  // s, monotonic

  std::atomic<uint32_t> m_frames{0};
  std::atomic<uint32_t> m_crcErrors{0};
  std::atomic<uint32_t> m_bytes{0};

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

/**
 * One encoder of the ProHub stream, with the distance API of prosdk::Encoder. Reset() zeroes it
 * locally (the co-processor count keeps running).
 */
class ProHubEncoder : public prosdk::telemetry::TelemetryLoggable {
 public:
  /**
   * @param encoders The stream reader (must outlive this object).
   * @param index    Encoder channel, 0 to 7 (motor port index + 1 = channel + 1).
   * @throws prosdk::RuntimeError (ChannelIndexOutOfRange) for an invalid index.
   */
  ProHubEncoder(ProHubEncoders& encoders, int index);

  /** Ticks since construction or Reset() (sign follows SetReverseDirection). */
  int64_t Get() const;
  /** Zeroes Get() and GetDistance() at the current count. */
  void Reset();
  /** Distance since Reset(), in the units of SetDistancePerPulse(). */
  double GetDistance() const;
  /** Rate over the latest frame interval, distance units per second. */
  double GetRate() const;
  /** Distance per tick (default 1). */
  void SetDistancePerPulse(double distancePerPulse);
  double GetDistancePerPulse() const;
  /** Inverts the counting direction. */
  void SetReverseDirection(bool reverseDirection);
  /** The encoder channel, 0 to 7. */
  int GetIndex() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  ProHubEncoders& m_encoders;
  int m_index;
  int64_t m_offset = 0;
  double m_distancePerPulse = 1.0;
  bool m_reverse = false;
};

}  // namespace prosdk
