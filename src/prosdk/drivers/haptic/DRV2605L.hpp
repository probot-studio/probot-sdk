// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// TI DRV2605L haptic driver (ERM or LRA vibration motor) over prosdk::I2C, for example the
// Boardoza DRV2605L breakout (design/boardoza/P5-i2c-misc.md). It plays effects of the chip's
// licensed ROM libraries through the 8-slot waveform sequencer, or drives the motor with a
// host-set amplitude (real-time playback). No auto-calibration.
//
// The chip is an I2C device and is not gated by the HAL: an effect or a real-time amplitude keeps
// playing while the robot is disabled or e-stopped, until Stop() or the destructor. It drives a
// small vibration motor only (supply = board VCC, at most 5.2 V).

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * TI DRV2605L haptic driver (datasheet SLOS854D).
 *
 * The constructor takes the chip out of standby in internal-trigger mode (MODE 0), selects the
 * actuator type (FEEDBACK N_ERM_LRA) and its library (ERM: TS2200 library A; LRA: the LRA
 * library) and clears the sequencer. Other registers keep their reset values: ERM runs open loop
 * (CONTROL3 ERM_OPEN_LOOP) with OD_CLAMP 0x8C as full scale (21.59 mV x 140, about 3.0 V;
 * datasheet 8.5.2.1, equation 6), and RTP data is signed. An LRA runs closed loop, bidirectional
 * input (CONTROL2 BIDIR_INPUT = 1), with the reset compensation values, without the datasheet's
 * auto-calibration (8.5.6); how well that suits a given LRA is unverified.
 *
 * Playback is asynchronous: Go() returns at once, IsPlaying() polls the GO bit. When the chip
 * did not answer at construction, the configuration is retried by the next call.
 */
class DRV2605L : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Actuator type (FEEDBACK register bit N_ERM_LRA).
  enum class Actuator { kERM, kLRA };

  /// 7-bit I2C address (fixed, datasheet 8.5.3.1).
  static constexpr int kAddress = 0x5A;
  /// Waveform sequencer slots (WAV_FRM_SEQ1 to 8, registers 0x04 to 0x0B).
  static constexpr int kNumSlots = 8;
  /// Highest ROM effect id (datasheet 8.5.8.2.5: ids 1 to 123).
  static constexpr int kMaxEffect = 123;

  /**
   * Constructs the driver and configures the chip (see the class comment). A missing chip prints
   * one console line; the constructor does not throw for it.
   *
   * @param port     The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param actuator The motor type on OUT+/OUT-.
   */
  explicit DRV2605L(prosdk::I2C::Port port, Actuator actuator = Actuator::kERM);

  /** Stops playback and puts the chip in standby (STANDBY bit). */
  ~DRV2605L() override;

  DRV2605L(const DRV2605L&) = delete;
  DRV2605L& operator=(const DRV2605L&) = delete;

  /**
   * Selects the ROM library (LIBRARY_SEL): 0 empty, 1 to 5 TS2200 libraries A to E, 6 LRA,
   * 7 TS2200 library F. Libraries 1 to 5 and 7 are for ERM motors, 6 for LRA.
   */
  void SetLibrary(int library);

  /**
   * Sets one sequencer slot (0 to 7). effect 1 to 123 plays that ROM effect; 0 ends the sequence
   * there (later slots are not played).
   */
  void SetWaveform(int slot, int effect);

  /** Starts the sequence (GO bit), leaving real-time playback first if it is active. */
  void Go();

  /** Stops playback: clears GO, and ends real-time playback (amplitude 0, internal trigger). */
  void Stop();

  /** True while the sequence plays (the GO bit; false when the chip does not answer). */
  bool IsPlaying();

  /**
   * Drives the motor continuously with an amplitude from -1 to 1 (real-time playback, MODE 5,
   * signed RTP data): 1 is full scale (0x7F), 0 no drive. Negative values drive an ERM backwards
   * (open loop: counter-rotation, braking; figure 26); for an LRA (closed loop, bidirectional
   * input) they are the same as 0, no drive (figure 28). NaN gives 0; values outside -1 to 1 are
   * clamped. Stop() ends it.
   */
  void SetRealtime(double value);

  /** The actuator type set at construction. */
  Actuator GetActuator() const;

  /** True when the last I2C transfer succeeded and the device id matched at configuration. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  bool Configure(uint8_t* statusOut = nullptr);
  bool EnsureConfigured();
  bool WriteRegister(uint8_t reg, uint8_t value);
  void CheckIdle();

  prosdk::I2C m_i2c;
  Actuator m_actuator;

  std::mutex m_mutex;                // the chip, m_realtime, m_configured
  bool m_realtime = false;           // MODE 5 is active
  bool m_configured = false;         // the configuration sequence was acknowledged
  std::atomic<bool> m_idOk{false};
  std::atomic<bool> m_connected{false};
  std::atomic<double> m_value{0.0};  // last real-time amplitude (0 when not in real time)
};

}  // namespace prosdk
