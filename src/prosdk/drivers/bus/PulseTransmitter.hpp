// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Timed pulse trains on a GPIO (design/boardoza/F3-timed-io.md): consumer-IR frames modulated on
// a carrier (38 kHz for the usual IR receivers), or plain level sequences that are too long for
// interrupt-masked bit-banging, such as a configuration word clocked out at about 100 us per
// bit. The ESP32 backend sends them on an RMT TX channel without CPU involvement (1 us
// resolution); on the host prosdk/simulation/PulseIOSim.hpp shows the trains to tests.

#pragma once

#include <stdint.h>

#include <span>

#include "prosdk/platform/hal/PulseIO.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/util/Handle.hpp"

namespace prosdk {

/// One level of a pulse train: `level` for `us` microseconds (prosdk/platform/hal/PulseIO.hpp).
using Pulse = prosdk::platform::hal::Pulse;

/**
 * Pulse-train output on one GPIO. The line idles low; each train is a list of levels with their
 * lengths (1 to 32767 us each, at most 1024 per train), sent in the background. With a carrier,
 * high levels are sent as carrier bursts and low levels as silence.
 *
 * Resources: one of the 3 RMT TX channels that AddressableLED outputs also use. Safety: the line
 * is a HAL output, so the emergency stop drives it low and no train starts after it; it is not
 * gated while the robot is merely disabled (an IR remote command or a sensor configuration is
 * not an actuator output).
 */
class PulseTransmitter {
 public:
  /// Carrier duty cycle used when none is given (the high third of each carrier period).
  static constexpr double kDefaultCarrierDuty = 0.33;

  /**
   * Opens the transmitter.
   *
   * @param channel     The GPIO.
   * @param carrier     Carrier frequency: 0 (none, the default) or 2 kHz to 1 MHz.
   * @param carrierDuty Carrier duty cycle, 0.1 to 0.9 (ignored without a carrier).
   * @throws prosdk::RuntimeError when the pin is in use, all RMT TX channels are taken or an
   *         argument is out of range. After an emergency stop the error is only reported (a
   *         warning status, as for DigitalOutput) and every Send() fails.
   */
  explicit PulseTransmitter(int channel,
                            prosdk::units::hertz_t carrier = prosdk::units::hertz_t{0},
                            double carrierDuty = kDefaultCarrierDuty);

  PulseTransmitter(PulseTransmitter&&) = default;
  PulseTransmitter& operator=(PulseTransmitter&&) = default;

  /**
   * Starts sending a train and returns without waiting for it to finish. A train still being sent
   * is waited for first (up to its length plus 100 ms).
   *
   * @return False after an error (reported to the console: a pulse out of range, too many
   *         pulses, the previous train stuck, an emergency stop).
   */
  bool Send(std::span<const Pulse> pulses);

  /**
   * Waits until the last train is sent (0 s: only checks).
   *
   * @return True when no train is being sent.
   */
  bool WaitSent(prosdk::units::second_t timeout);

  /** The GPIO. */
  int GetChannel() const;

  /** The carrier frequency (0 Hz: none). */
  prosdk::units::hertz_t GetCarrier() const;

 private:
  int m_channel;
  prosdk::units::hertz_t m_carrier;
  prosdk::util::Handle<int32_t, prosdk::platform::hal::ClosePulse,
                       prosdk::platform::hal::kInvalidPulseHandle>
      m_handle;
};

}  // namespace prosdk
