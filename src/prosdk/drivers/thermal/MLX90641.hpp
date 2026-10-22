// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Melexis MLX90641 16x12 thermal camera over prosdk::I2C (Boardoza MLX90641 breakout;
// design/boardoza/P2-light-thermal.md). The calibration and the temperature math are the Melexis
// API (melexis/MLX90641_API.cpp, Apache-2.0, licenses/melexis/LICENSE). Each refresh period the
// chip measures one subpage, and each subpage holds all 192 pixels (datasheet rev 4, section 10.4).

#pragma once

#include <stdint.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <thread>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/temperature.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/**
 * MLX90641 thermal camera.
 *
 * A reader thread polls the chip every updatePeriod. On the first poll it reads the 832-word
 * calibration EEPROM (Hamming-coded; a corrected single-bit error is accepted) and sets the
 * refresh rate (retried once per second until it succeeds). Each new subpage is read (about
 * 500 bytes), converted to degrees Celsius and published as a frame for GetFrame(). With an
 * updatePeriod of 0 no thread starts: call Update() yourself from a thread with at least 4 KiB of
 * stack.
 *
 * Use 400 kHz on the I2C port: the datasheet limits EEPROM reads to 400 kHz (rev 4, section 8,
 * note 5). At 400 kHz a subpage takes about 13 ms of bus time.
 */
class MLX90641 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit I2C address (datasheet rev 4, section 8; stored in EEPROM 0x240F).
  static constexpr int kDefaultAddress = 0x33;
  static constexpr int kWidth = 16;
  static constexpr int kHeight = 12;
  static constexpr int kPixelCount = kWidth * kHeight;
  /// Default poll period of the reader thread.
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{10};

  /// Subpage refresh rate, control register 0x800D bits 9:7 (datasheet rev 4, section 10.6.1,
  /// Figure 11). One frame per period.
  enum class RefreshRate : uint8_t {
    k0_5Hz = 0,
    k1Hz = 1,
    k2Hz = 2,
    k4Hz = 3,
    k8Hz = 4,
    k16Hz = 5,
    k32Hz = 6,
    k64Hz = 7,
  };

  /**
   * Constructs the driver and starts the reader thread. No bus access happens here.
   *
   * @param port         The I2C port (prosdk::platform::SetI2CPins sets its pins and clock).
   * @param address      The 7-bit I2C address.
   * @param rate         The subpage refresh rate written to the chip (RAM only, not EEPROM).
   * @param updatePeriod Poll period of the reader thread; 0 starts no thread (call Update()).
   */
  explicit MLX90641(prosdk::I2C::Port port, int address = kDefaultAddress,
                    RefreshRate rate = RefreshRate::k8Hz,
                    prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);
  ~MLX90641() override;

  MLX90641(const MLX90641&) = delete;
  MLX90641& operator=(const MLX90641&) = delete;

  /**
   * Initializes the chip if needed, then reads a new subpage if one is ready (the thread's job).
   * A failed transfer makes the next call initialize again (EEPROM and refresh rate), at most once
   * per second, because a camera that lost power is back at its EEPROM refresh rate.
   */
  void Update();

  /**
   * Copies the latest frame, row by row (index = row * kWidth + column), in degrees Celsius.
   *
   * @return false (and out unchanged) before the first frame.
   */
  bool GetFrame(std::span<float, kPixelCount> out) const;

  /** The chip's own temperature from the latest subpage (0 before the first one). */
  prosdk::units::celsius_t GetAmbientTemperature() const;

  /** Number of frames published so far. */
  uint32_t GetFrameCount() const;

  /** Object emissivity used for the conversion, 0 < emissivity <= 1 (default 0.95). */
  void SetEmissivity(double emissivity);

  /** True when the calibration was read and the last transfer succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  struct Impl;  // the Melexis buffers, about 5 KiB, on the heap

  bool Init();
  void Run();

  prosdk::I2C m_i2c;
  RefreshRate m_rate;
  prosdk::units::millisecond_t m_updatePeriod;
  std::unique_ptr<Impl> m_impl;  // used by Update() only
  std::mutex m_updateMutex;      // one Update() at a time (the thread's and the user's)
  mutable std::mutex m_mutex;    // guards the members below and m_impl->published
  bool m_initialized = false;
  bool m_connected = false;
  bool m_warned = false;
  double m_lastInitAttempt = -1.0;  // s, monotonic
  double m_emissivity = 0.95;
  double m_ambient = 0.0;
  uint32_t m_frameCount = 0;
  float m_minimum = 0.0f;
  float m_maximum = 0.0f;
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
