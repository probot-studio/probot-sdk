// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: PCNT pulse counters shared by Counter and Encoder
// (design/hal-esp32.md, "Counters and encoders").
//
// The ESP32-S3 has 4 PCNT units; each counter or encoder takes one. Counts are 32-bit (the 16-bit
// hardware counter is extended on overflow by the driver). A glitch filter drops pulses shorter
// than 1 us. Rates come from an esp_timer that samples every unit every 5 ms; the rate is the
// count change over the rate window (default 50 ms, 5..300 ms).

#pragma once

#include <stdint.h>

#include <memory>

#include "Esp32Internal.hpp"

namespace prosdk::hal::esp32 {

class PulseCounter final : public PinDriver {
 public:
  /**
   * Starts a counter. Quadrature (x4) when bGpio >= 0: counts up when A leads B. Otherwise one
   * input counting rising or falling edges. Inputs get the internal pull-up. Returns a HAL status
   * (with a message) on failure.
   */
  static int32_t Create(int aGpio, int bGpio, bool risingEdge, std::unique_ptr<PulseCounter>* out);

  ~PulseCounter() override;

  int32_t GetCount(int32_t* count);
  /** Counts per second over the rate window. */
  int32_t GetRate(double* countsPerSecond);
  int32_t SetRateWindow(int32_t windowMs);
  /** Single-input counters only. */
  int32_t SetEdge(bool risingEdge);
  /** Sets the count to 0. */
  int32_t Reset();

  struct State;

 private:
  PulseCounter() = default;
  State* m_state = nullptr;
};

}  // namespace prosdk::hal::esp32
