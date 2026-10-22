// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: timed DIO bit-banging (prosdk/platform/hal/BitBang.hpp,
// design/boardoza/F3-timed-io.md). The pins are DIOs allocated through DIO.cpp; this file only
// looks them up by GPIO in smartIoHandles and drives them with direct GPIO register writes
// (gpio_ll) and busy waits on the CPU cycle counter.
//
// Interrupt masking: a spinlock critical section (portENTER_CRITICAL) covers one clock-high phase
// of ShiftIn or one whole WriteWaveform, at most kMaxMaskedNs (20 us); the clock-low phases run
// with interrupts enabled. The HX711 allows this: its PD_SCK low time has no maximum, and only a
// high time over 60 us powers it down (Avia HX711 datasheet, timing table T3/T4 and Fig. 3).
//
// Emergency stop: nothing is written once EStopped() is true, and a high level that raced the
// e-stop is undone at the end of the bit or waveform (as HAL_SetDIO does), so the pin stays low.
//
// Unverified on hardware (design/boardoza/F3-timed-io.md): the real high and low times (cycle
// counter waits plus GPIO register latency) and that WiFi and probot-core tolerate the windows.

#include <esp_cpu.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <hal/gpio_ll.h>

#include <format>
#include <memory>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/platform/hal/BitBang.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

constexpr const char* kEStopped =
    "bit-bang: the robot is emergency stopped; outputs stay off until reboot";

// The DIO on `gpio` (an output if `output`), held for the transfer so it cannot be freed
// meanwhile. The target does not check handle versions (HandlesInternal.hpp), so the handle is
// rebuilt from the GPIO; the mode check makes sure the pin really is a DIO.
std::shared_ptr<SmartIo> GetDio(int32_t gpio, bool output, const char* what, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  if (IsValidGpio(gpio)) {
    port = smartIoHandles->Get(createHandle(static_cast<int16_t>(gpio), HAL_HandleEnum::DIO, 0),
                               HAL_HandleEnum::DIO);
  }
  bool ok = port != nullptr && (port->mode == PinMode::kDigitalOutput ||
                                (!output && port->mode == PinMode::kDigitalInput));
  if (!ok) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        std::format("{}: GPIO {} is not an allocated DIO{}", what, gpio,
                                    output ? " output" : ""));
    return nullptr;
  }
  *status = HAL_SUCCESS;
  return port;
}

uint32_t NsToCycles(uint32_t ns) {
  return static_cast<uint32_t>(
      (static_cast<uint64_t>(ns) * esp_rom_get_cpu_ticks_per_us() + 999) / 1000);
}

inline void WaitCycles(esp_cpu_cycle_count_t start, uint32_t cycles) {
  while (static_cast<uint32_t>(esp_cpu_get_cycle_count() - start) < cycles) {
  }
}

}  // namespace

namespace prosdk::platform::hal {

bool ShiftIn(int32_t clockGpio, int32_t dataGpio, int bits, uint32_t highNs, uint32_t lowNs,
             uint64_t* out, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (const char* error = detail::CheckShiftIn(bits, highNs, lowNs)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return false;
  }
  auto clock = GetDio(clockGpio, true, "ShiftIn clock", status);
  if (clock == nullptr) {
    return false;
  }
  auto data = GetDio(dataGpio, false, "ShiftIn data", status);
  if (data == nullptr) {
    return false;
  }
  const uint32_t clk = static_cast<uint32_t>(clockGpio);
  const uint32_t dat = static_cast<uint32_t>(dataGpio);
  const uint32_t highCycles = NsToCycles(highNs);
  const uint32_t lowCycles = NsToCycles(lowNs);
  uint64_t value = 0;
  for (int i = 0; i < bits; ++i) {
    if (EStopped()) {
      *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
      return false;
    }
    portENTER_CRITICAL(&gMux);
    esp_cpu_cycle_count_t start = esp_cpu_get_cycle_count();
    gpio_ll_set_level(&GPIO, clk, 1);
    WaitCycles(start, highCycles);
    int bit = gpio_ll_get_level(&GPIO, dat);
    gpio_ll_set_level(&GPIO, clk, 0);
    portEXIT_CRITICAL(&gMux);
    value = (value << 1) | static_cast<uint64_t>(bit);
    WaitCycles(esp_cpu_get_cycle_count(), lowCycles);
  }
  if (EStopped()) {
    // the clock ended low; an e-stop during the transfer may have cut it short
    gpio_ll_set_level(&GPIO, clk, 0);
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
    return false;
  }
  *out = value;
  return true;
}

bool WriteWaveform(int32_t gpio, std::span<const LevelSpan> spans, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (const char* error = detail::CheckWaveform(spans)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return false;
  }
  auto pin = GetDio(gpio, true, "WriteWaveform", status);
  if (pin == nullptr) {
    return false;
  }
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
    return false;
  }
  // cycle counts first: nothing but register writes and busy waits inside the critical section
  uint32_t cycles[kMaxWaveformSpans];
  for (size_t i = 0; i < spans.size(); ++i) {
    cycles[i] = NsToCycles(spans[i].ns);
  }
  const uint32_t p = static_cast<uint32_t>(gpio);
  portENTER_CRITICAL(&gMux);
  for (size_t i = 0; i < spans.size(); ++i) {
    esp_cpu_cycle_count_t start = esp_cpu_get_cycle_count();
    gpio_ll_set_level(&GPIO, p, spans[i].level ? 1 : 0);
    WaitCycles(start, cycles[i]);
  }
  portEXIT_CRITICAL(&gMux);
  if (EStopped()) {
    gpio_ll_set_level(&GPIO, p, 0);  // undo a level that raced the emergency stop
    *status = MakeError(HAL_INCOMPATIBLE_STATE, kEStopped);
    return false;
  }
  return true;
}

}  // namespace prosdk::platform::hal
