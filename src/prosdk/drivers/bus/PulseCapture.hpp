// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Pulse-width capture on a GPIO (design/boardoza/F3-timed-io.md): the demodulated output of an
// IR receiver or an OOK radio receiver, recorded as a list of levels with their lengths for a
// protocol decoder. The ESP32 backend records on an RMT RX channel in the background (1 us
// resolution); on the host tests feed waveforms through prosdk/simulation/PulseIOSim.hpp.

#pragma once

#include <stdint.h>

#include <span>

#include "prosdk/drivers/bus/PulseTransmitter.hpp"
#include "prosdk/platform/hal/PulseIO.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/util/Handle.hpp"

namespace prosdk {

/**
 * Pulse capture on one GPIO (input with the internal pull-up, unverified). The line is recorded
 * in bursts: a burst starts at the first edge after an idle line, and a level that lasts the idle
 * time or longer ends it with a Pulse of 0 us at that level. Read() hands out the pulses in order;
 * the buffer holds 256 pulses, more are dropped (GetDropped()), so read at least every loop.
 *
 * Resources: one of the 4 RMT RX channels.
 */
class PulseCapture {
 public:
  /// Idle time used when none is given.
  static constexpr prosdk::units::microsecond_t kDefaultIdleTime{10000};

  /**
   * Opens the capture.
   *
   * @param channel      The GPIO.
   * @param idleTime     Burst end: 10 us to 32.767 ms, longer than any pulse inside a burst.
   * @param glitchFilter Pulses shorter than this are ignored: 0 to 3 us (the RMT filter limit;
   *                     a decoder filters longer noise itself).
   * @throws prosdk::RuntimeError when the pin is in use, all RMT RX channels are taken or an
   *         argument is out of range.
   */
  explicit PulseCapture(int channel, prosdk::units::microsecond_t idleTime = kDefaultIdleTime,
                        prosdk::units::microsecond_t glitchFilter =
                            prosdk::units::microsecond_t{0});

  PulseCapture(PulseCapture&&) = default;
  PulseCapture& operator=(PulseCapture&&) = default;

  /**
   * Moves up to out.size() captured pulses, oldest first, into `out`. Does not block.
   *
   * @return How many pulses were written.
   */
  int Read(std::span<Pulse> out);

  /** Pulses dropped because the buffer was full, since construction. */
  int64_t GetDropped() const;

  /** The GPIO. */
  int GetChannel() const;

 private:
  int m_channel;
  prosdk::util::Handle<int32_t, prosdk::platform::hal::ClosePulse,
                       prosdk::platform::hal::kInvalidPulseHandle>
      m_handle;
};

}  // namespace prosdk
