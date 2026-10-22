// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// NXP PCA9685 16-channel 12-bit PWM driver over prosdk::I2C (design/drivers.md, "PCA9685").
// The ProHub board carries one (0x40, a second one at 0x41 is planned): motor ports drive its
// channel pairs as an H-bridge (ProHubMotor), channels 12-15 drive servos (Servo).
//
// Writes are buffered: Set*() stores the channel value and a background thread (or Update())
// sends the changed channels. The same thread applies the output gate: while the robot is not
// enabled, after an emergency stop and after the robot program halted, every output is held
// fully off (ALL_LED_OFF). This is not done from probot-core's emergency-stop callback, which must
// be lock-free and cannot use the I2C bus; see design/drivers.md for the resulting limitation.

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** NXP PCA9685 16-channel PWM controller on an I2C bus. */
class PCA9685 {
 public:
  /// Default 7-bit I2C address (all address pins low).
  static constexpr int kDefaultAddress = 0x40;
  /// Number of outputs.
  static constexpr int kNumChannels = 16;
  /// Duty-cycle resolution (counts per period).
  static constexpr int kResolution = 4096;
  /// Default output frequency (analog servos).
  static constexpr prosdk::units::hertz_t kDefaultFrequency{50};
  /// Default period of the output thread.
  static constexpr prosdk::units::millisecond_t kDefaultUpdatePeriod{10};

  /**
   * Constructs the driver, configures the chip (all outputs off, totem-pole outputs, the given
   * frequency) and starts the output thread.
   *
   * @param port         The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address      The 7-bit I2C address.
   * @param frequency    The output frequency, 24 to 1526 Hz (internal 25 MHz oscillator). The
   *                     chip has one frequency for all 16 channels.
   * @param updatePeriod Period of the output thread; 0 starts no thread (call Update()).
   */
  explicit PCA9685(prosdk::I2C::Port port, int address = kDefaultAddress,
                   prosdk::units::hertz_t frequency = kDefaultFrequency,
                   prosdk::units::millisecond_t updatePeriod = kDefaultUpdatePeriod);

  /** Stops the thread and turns every output off. */
  ~PCA9685();

  PCA9685(const PCA9685&) = delete;
  PCA9685& operator=(const PCA9685&) = delete;

  /**
   * Sets a channel's duty cycle: 0 is fully off, 1 is fully on, values in between are rounded
   * to 1/4096. NaN and values outside 0 to 1 are clamped (NaN to 0).
   */
  void SetDutyCycle(int channel, double dutyCycle);

  /** Sets a channel's high time per period (clamped to 0 .. the period). */
  void SetPulseWidth(int channel, prosdk::units::microsecond_t pulse);

  /** Turns a channel fully off. */
  void SetOff(int channel);

  /** The duty cycle last set on a channel (0 to 1). */
  double GetDutyCycle(int channel) const;

  /** The actual output frequency (the prescaler rounds the requested one). */
  prosdk::units::hertz_t GetFrequency() const;

  /** The output period (1 / GetFrequency()). */
  prosdk::units::microsecond_t GetPeriod() const;

  /** True when the last I2C transfer to the chip succeeded. */
  bool IsConnected() const;

  /** True while the outputs are released (the robot is enabled and not e-stopped). */
  bool OutputsEnabled() const;

  /** The 7-bit I2C address. */
  int GetAddress() const;

  /**
   * Applies the output gate and sends the changed channels now (what the output thread does
   * every period). Call it periodically when updatePeriod is 0.
   */
  void Update();

 private:
  bool Configure();
  bool WriteChannel(int channel, uint16_t counts);
  bool WriteAllOff();
  void CheckChannel(int channel) const;
  void Run();

  prosdk::I2C m_i2c;
  int m_address;
  uint8_t m_prescale;
  prosdk::units::millisecond_t m_updatePeriod;

  mutable std::mutex m_mutex;          // m_counts, m_dirty
  std::array<uint16_t, kNumChannels> m_counts{};  // 0 = full off, 4096 = full on
  uint32_t m_dirty = 0;                // bit n: channel n changed

  std::mutex m_ioMutex;                // the chip (Update from the thread and the user)
  bool m_configured = false;
  bool m_outputsOff = true;            // the chip holds ALL_LED_OFF (gate closed)
  std::atomic<bool> m_connected{false};
  std::atomic<bool> m_enabled{false};

  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
