// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Pulse-train I/O on any GPIO (design/boardoza/F3-timed-io.md): transmit a list of timed levels,
// optionally modulated on a carrier (consumer IR at 38 kHz), and capture the pulse widths on an
// input (OOK radio receivers, IR receivers). A prosdk extension like CoreJoystick and SPI: plain
// C++ functions, not a HAL_* C API. Users: prosdk::PulseTransmitter and prosdk::PulseCapture.
//
// ESP32: esp32/PulseIO.cpp on the RMT peripheral at 1 MHz (1 us per tick). TX takes one of the 3
// RMT TX channels it shares with AddressableLED outputs; RX one of the 4 RMT RX channels. A pulse
// TX pin is a HAL output: the emergency stop drives it low, and after it OpenPulseTx and
// SendPulses fail with HAL_INCOMPATIBLE_STATE. It is not gated while the robot is merely
// disabled (like LED strips). Host: PulseIOSim.cpp with the test hooks of
// prosdk/simulation/PulseIOSim.hpp.

#pragma once

#include <stdint.h>

#include <span>

namespace prosdk::platform::hal {

/**
 * One level of a pulse train: the line is at `level` for `us` microseconds. With a carrier, a
 * high level is sent as carrier bursts and a low level as silence.
 *
 * In captured trains a pulse with `us` 0 ends a burst: the line then stayed at `level` for at
 * least the idle time given to OpenPulseRx.
 */
struct Pulse {
  bool level = false;
  uint32_t us = 0;

  bool operator==(const Pulse&) const = default;
};

/** Handle value of no pulse channel. */
inline constexpr int32_t kInvalidPulseHandle = 0;

/** Longest pulse SendPulses accepts, in us (one RMT symbol half at 1 MHz). Split longer levels
 * into several pulses of the same level. */
inline constexpr uint32_t kMaxPulseUs = 32767;
/** Most pulses in one SendPulses. */
inline constexpr int32_t kMaxPulsesPerSend = 1024;
/** Carrier frequency range of OpenPulseTx (0 = no carrier). The RMT carrier counts its high and
 * low times in group-clock ticks, at most 65536 each (rmt_ll_tx_set_carrier_high_low_ticks
 * asserts it and aborts otherwise; ESP-IDF 5.5 hal/esp32s3 rmt_ll.h). With the group clock at
 * most 80 MHz (APB; that it is undivided is unverified) and a 0.9 duty, 1.1 kHz is the lowest
 * safe carrier, so 2 kHz is the floor. */
inline constexpr uint32_t kMinCarrierHz = 2000;
inline constexpr uint32_t kMaxCarrierHz = 1000000;
/** Carrier duty cycle range (fraction of the carrier period that is high). */
inline constexpr double kMinCarrierDuty = 0.1;
inline constexpr double kMaxCarrierDuty = 0.9;
/** Longest glitch filter of OpenPulseRx, in us (the RMT filter: 255 cycles of 80 MHz). */
inline constexpr uint32_t kMaxRxGlitchUs = 3;
/** Idle time range of OpenPulseRx, in us (the RMT idle threshold is 15 bits at 1 MHz). */
inline constexpr uint32_t kMinRxIdleUs = 10;
inline constexpr uint32_t kMaxRxIdleUs = 32767;
/** Pulses buffered per capture channel; more are dropped (GetDroppedPulses). */
inline constexpr int32_t kRxBufferPulses = 256;

/**
 * Opens a pulse transmitter on a GPIO. The line idles low.
 *
 * @param gpio               Any free GPIO.
 * @param carrierHz          Carrier frequency, 0 (none) or kMinCarrierHz to kMaxCarrierHz.
 * @param carrierDuty        Carrier duty cycle, kMinCarrierDuty to kMaxCarrierDuty (ignored
 *                           without a carrier).
 * @param allocationLocation Shown when the pin is allocated twice (may be nullptr).
 * @param status             HAL status: HAL_PARAMETER_OUT_OF_RANGE, HAL_RESOURCE_IS_ALLOCATED,
 *                           HAL_NO_AVAILABLE_RESOURCES (all 3 RMT TX channels in use, LED
 *                           outputs included), HAL_INCOMPATIBLE_STATE after an emergency stop.
 * @return The handle, or kInvalidPulseHandle.
 */
int32_t OpenPulseTx(int32_t gpio, uint32_t carrierHz, double carrierDuty,
                    const char* allocationLocation, int32_t* status);

/**
 * Starts sending a pulse train and returns without waiting for it. A train still being sent is
 * waited for first (up to its remaining length plus 100 ms). Each pulse is 1 to kMaxPulseUs us;
 * at most kMaxPulsesPerSend pulses. After the train the line returns low.
 */
void SendPulses(int32_t handle, std::span<const Pulse> pulses, int32_t* status);

/**
 * Waits until the last train is sent. Returns false on timeout (`timeoutMs`, 0 = just check) or
 * an invalid handle.
 */
bool WaitPulsesSent(int32_t handle, int32_t timeoutMs, int32_t* status);

/**
 * Opens a pulse capture on a GPIO (input; the ESP32 backend enables the internal pull-up,
 * unverified that it stays on). Capture runs in the background into a buffer of kRxBufferPulses
 * pulses; read it with ReadPulses.
 *
 * A burst starts at the first edge after an idle line. Pulses shorter than `glitchUs` are
 * ignored (the line keeps its previous level). A level that lasts `idleUs` or longer ends the
 * burst with a pulse of 0 us at that level.
 *
 * @param gpio               Any free GPIO.
 * @param glitchUs           Glitch filter, 0 to kMaxRxGlitchUs.
 * @param idleUs             Burst end, kMinRxIdleUs to kMaxRxIdleUs; longer than any pulse
 *                           inside a burst.
 * @param allocationLocation Shown when the pin is allocated twice (may be nullptr).
 * @param status             HAL status (HAL_NO_AVAILABLE_RESOURCES: all 4 RMT RX channels in
 *                           use).
 * @return The handle, or kInvalidPulseHandle.
 */
int32_t OpenPulseRx(int32_t gpio, uint32_t glitchUs, uint32_t idleUs,
                    const char* allocationLocation, int32_t* status);

/** Moves up to out.size() captured pulses, oldest first, into `out`; returns how many. Does not
 * block. */
int32_t ReadPulses(int32_t handle, std::span<Pulse> out, int32_t* status);

/** Pulses dropped because the capture buffer was full, since the channel opened. */
int64_t GetDroppedPulses(int32_t handle, int32_t* status);

/** Closes a transmitter (after its train ends, at most 100 ms) or a capture; frees the pin. */
void ClosePulse(int32_t handle);

namespace detail {
// Argument checks shared by both backends: nullptr when valid, else the reason.
inline const char* CheckPulseTx(uint32_t carrierHz, double carrierDuty) {
  if (carrierHz != 0 && (carrierHz < kMinCarrierHz || carrierHz > kMaxCarrierHz)) {
    return "pulse TX: the carrier must be 0 (none) or 2 kHz to 1 MHz";
  }
  if (carrierHz != 0 && !(carrierDuty >= kMinCarrierDuty && carrierDuty <= kMaxCarrierDuty)) {
    return "pulse TX: the carrier duty cycle must be 0.1 to 0.9";
  }
  return nullptr;
}

inline const char* CheckPulseTrain(std::span<const Pulse> pulses) {
  if (pulses.size() > static_cast<size_t>(kMaxPulsesPerSend)) {
    return "pulse TX: more than 1024 pulses in one train";
  }
  for (const Pulse& p : pulses) {
    if (p.us < 1 || p.us > kMaxPulseUs) {
      return "pulse TX: every pulse must be 1 to 32767 us";
    }
  }
  return nullptr;
}

inline const char* CheckPulseRx(uint32_t glitchUs, uint32_t idleUs) {
  if (glitchUs > kMaxRxGlitchUs) {
    return "pulse RX: the glitch filter must be 0 to 3 us";
  }
  if (idleUs < kMinRxIdleUs || idleUs > kMaxRxIdleUs) {
    return "pulse RX: the idle time must be 10 to 32767 us";
  }
  return nullptr;
}
}  // namespace detail

}  // namespace prosdk::platform::hal
