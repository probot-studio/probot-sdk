// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Timed DIO bit-banging (design/boardoza/F3-timed-io.md): clocked serial reads and short fixed
// waveforms on DIO pins with sub-microsecond timing that a FreeRTOS preemption between two
// DigitalOutput::Set() calls would break. A prosdk extension (WPILib has no such HAL function),
// after the CoreJoystick precedent: plain C++ functions, not a HAL_* C API.
//
// Pins are GPIO numbers of DIOs the caller already allocated (a prosdk DigitalOutput or
// DigitalInput); wpilibc does not expose DIO handles, so the backends look the pins up by
// channel. ESP32: src/prosdk/platform/hal/esp32/BitBang.cpp (interrupts masked per clock-high
// phase, direct GPIO register access). Host: BitBangSim.cpp (the DIO sim values, so tests
// emulate the chip with DIO sim callbacks; no real time passes).

#pragma once

#include <stdint.h>

#include <cstddef>
#include <span>

namespace prosdk::platform::hal {

/**
 * Longest window, in ns, during which the ESP32 backend masks interrupts: one clock-high phase
 * of ShiftIn, or a whole WriteWaveform. 20 us keeps WiFi and probot-core's loop unaffected
 * (unverified on hardware: design/boardoza/F3-timed-io.md).
 */
inline constexpr uint32_t kMaxMaskedNs = 20000;

/** Longest clock-low phase of ShiftIn, in ns (a busy wait with interrupts enabled). */
inline constexpr uint32_t kMaxShiftLowNs = 1000000;

/** Most bits one ShiftIn call clocks. */
inline constexpr int kMaxShiftBits = 64;

/** Most spans in one WriteWaveform. */
inline constexpr size_t kMaxWaveformSpans = 64;

/**
 * Clocks `bits` bits in, MSB first, as the Avia HX711 expects (datasheet Fig. 2 and its timing
 * table: data valid at most 0.1 us after the PD_SCK rising edge, T3 high time 0.2 to 50 us, T4
 * low time at least 0.2 us with no maximum). Per bit: clock high, wait highNs, sample the data
 * pin, clock low, wait lowNs. The clock idles low. On the ESP32 interrupts are masked only during
 * the high phase; the low phase may be preempted (no maximum low time).
 *
 * @param clockGpio Allocated DIO output that drives the clock.
 * @param dataGpio  Allocated DIO (input, normally) that is sampled.
 * @param bits      1 to kMaxShiftBits.
 * @param highNs    Clock-high time, 1 to kMaxMaskedNs.
 * @param lowNs     Clock-low time after each bit, 0 to kMaxShiftLowNs.
 * @param out       Receives the bits (first bit in the most significant position of `bits`).
 * @param status    HAL status: HAL_PARAMETER_OUT_OF_RANGE for bad arguments,
 *                  HAL_INCOMPATIBLE_STATE when a pin is not an allocated DIO (clock: output) or
 *                  after an emergency stop (the clock stays low).
 * @return True when all bits were clocked.
 */
bool ShiftIn(int32_t clockGpio, int32_t dataGpio, int bits, uint32_t highNs, uint32_t lowNs,
             uint64_t* out, int32_t* status);

/** One level of a WriteWaveform: the pin is driven to `level` for `ns` nanoseconds. */
struct LevelSpan {
  bool level = false;
  uint32_t ns = 0;
};

/**
 * Drives one DIO output through a fixed waveform with interrupts masked for its whole length
 * (the sum of the spans must be at most kMaxMaskedNs). After the last span the pin keeps that
 * span's level. Longer waveforms (for example the PYD1598 SERIN word) belong on the RMT pulse
 * transmitter (PulseIO.hpp), which needs no interrupt masking.
 *
 * @param gpio   Allocated DIO output.
 * @param spans  1 to kMaxWaveformSpans spans.
 * @param status As for ShiftIn; after an emergency stop nothing is written.
 * @return True when the waveform was written.
 */
bool WriteWaveform(int32_t gpio, std::span<const LevelSpan> spans, int32_t* status);

namespace detail {
// Argument checks shared by both backends: nullptr when valid, else the reason.
inline const char* CheckShiftIn(int bits, uint32_t highNs, uint32_t lowNs) {
  if (bits < 1 || bits > kMaxShiftBits) {
    return "ShiftIn: bits must be 1 to 64";
  }
  if (highNs < 1 || highNs > kMaxMaskedNs) {
    return "ShiftIn: the clock-high time must be 1 ns to 20 us";
  }
  if (lowNs > kMaxShiftLowNs) {
    return "ShiftIn: the clock-low time must be at most 1 ms";
  }
  return nullptr;
}

inline const char* CheckWaveform(std::span<const LevelSpan> spans) {
  if (spans.empty() || spans.size() > kMaxWaveformSpans) {
    return "WriteWaveform: 1 to 64 spans";
  }
  uint64_t total = 0;
  for (const LevelSpan& s : spans) {
    total += s.ns;
  }
  if (total > kMaxMaskedNs) {
    return "WriteWaveform: the waveform is longer than 20 us (use a PulseTransmitter)";
  }
  return nullptr;
}
}  // namespace detail

}  // namespace prosdk::platform::hal
