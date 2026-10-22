// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Nuvoton NAU7802 24-bit bridge ADC over prosdk::I2C (Boardoza NAU7802 breakout;
// design/boardoza/P6-loadcell.md): two differential inputs (one converted at a time), a PGA of
// x1 to x128 and an on-chip LDO that excites the bridge. The driver runs the chip from its
// internal LDO at 3.0 V (DVDD, here 3.3 V, must be 0.3 V above it; at least 10 mA, enough for one
// 350 ohm bridge) and its internal oscillator.
// For load cells, feed GetRaw() counts into a prosdk::LoadCell.

#pragma once

#include <stdint.h>

#include <mutex>
#include <optional>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * NAU7802 24-bit ADC.
 *
 * No thread: call Update() periodically (for example every robot loop). It does not wait for a
 * conversion: it reads the status register and, when a conversion is ready, the 24-bit result
 * (only while the chip needs setting up again does it wait, up to about 10 ms once per second,
 * for the power-up). The chip converts at
 * 10 to 320 samples per second (80 by default); a slower loop gets the latest conversion.
 *
 * Defaults: gain x128, 80 samples per second, channel 1. After a gain, rate or channel change the
 * next 6 conversions are dropped while the input settles. Call CalibrateInternal() once at
 * startup and after each such change (the data sheet recommends it, section 8.6).
 */
class NAU7802 : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// The 7-bit I2C address, fixed in silicon (data sheet 8.3.2, Figure 4: 0101010).
  static constexpr int kAddress = 0x2A;

  /**
   * Constructs the driver: resets the chip, powers it up and configures it (data sheet 9.1).
   * Waits up to about 10 ms for the power-up. Does not calibrate. A missing chip prints one
   * console line; Update() then retries the setup at most once per second.
   *
   * @param port The I2C port (prosdk::platform::SetI2CPins sets its pins).
   */
  explicit NAU7802(prosdk::I2C::Port port);

  NAU7802(const NAU7802&) = delete;
  NAU7802& operator=(const NAU7802&) = delete;

  /**
   * Sets the PGA gain.
   *
   * @param gain 1, 2, 4, 8, 16, 32, 64 or 128.
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for other values.
   */
  void SetGain(int gain);

  /**
   * Sets the conversion rate.
   *
   * @param samplesPerSecond 10, 20, 40, 80 or 320.
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for other values.
   */
  void SetSampleRate(int samplesPerSecond);

  /**
   * Selects the input: 1 (VIN1, board connector J1) or 2 (VIN2, J4). The offset and gain
   * calibration is kept per channel, so calibrate each channel once after selecting it.
   *
   * @throws prosdk::RuntimeError (ChannelIndexOutOfRange) for other values.
   */
  void SetChannel(int channel);

  /**
   * Internal offset calibration of the selected channel (CALMOD = 00, data sheet 8.6). Blocks
   * until the chip finishes, at most 2 s; call it at startup, not from a periodic method (2 s is
   * probot-core's loop stall deadline).
   *
   * @return True when the calibration finished without CAL_ERR.
   */
  bool CalibrateInternal();

  /** Reads the status register: true when a new conversion is ready (CR). */
  bool IsReady();

  /**
   * Reads a new conversion if one is ready (non-blocking).
   *
   * @return The new signed 24-bit result (-8388608 to 8388607), or nothing when no new
   *         conversion is ready, while settling, or when the transfer failed.
   */
  std::optional<int32_t> Update();

  /** The last result Update() returned (0 before the first). */
  int32_t GetRaw() const;

  /**
   * True when the last transfer succeeded and the chip reported power-up ready. A chip that
   * reset (a supply dip) is set up again by Update(), without its calibration.
   */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Setup();
  bool WriteCtrl1();
  bool WriteCtrl2(uint8_t extra);

  prosdk::I2C m_i2c;
  mutable std::mutex m_mutex;
  uint8_t m_gainCode = 7;   // CTRL1 GAINS: x128
  uint8_t m_rateCode = 3;   // CTRL2 CRS: 80 SPS
  int m_channel = 1;
  int m_discard = 0;        // conversions still to drop after a change
  bool m_connected = false;
  double m_lastSetupTime = 0.0;  // s, monotonic
  int32_t m_raw = 0;
};

}  // namespace prosdk
