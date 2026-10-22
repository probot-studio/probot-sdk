// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of timed DIO bit-banging (prosdk/platform/hal/BitBang.hpp). The pins are the
// DIO sim channels: every clock edge and waveform level is a HALSIM_SetDIOValue (DIO sim value
// callbacks run synchronously, so a test emulates the chip from them) and the data pin is read
// with HALSIM_GetDIOValue. The argument checks are the ESP32 backend's; no real or simulated
// time passes. Compiled into both builds by the platform/hal glob; empty on the robot.

#include "prosdk/platform/config.hpp"

#if !PROSDK_TARGET_HW

#include <format>

#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/Ports.h"
#include "prosdk/hal/simulation/DIOData.h"
#include "prosdk/platform/hal/BitBang.hpp"

namespace {

using prosdk::hal::MakeError;

// HAL_SUCCESS when `gpio` is an initialized DIO sim channel (an output if `output`).
int32_t CheckPin(int32_t gpio, bool output, const char* what) {
  if (gpio < 0 || gpio >= HAL_GetNumDigitalChannels() || !HALSIM_GetDIOInitialized(gpio)) {
    return MakeError(HAL_INCOMPATIBLE_STATE,
                     std::format("{}: channel {} is not an allocated DIO", what, gpio));
  }
  if (output && HALSIM_GetDIOIsInput(gpio)) {
    return MakeError(HAL_INCOMPATIBLE_STATE,
                     std::format("{}: channel {} is not a DIO output", what, gpio));
  }
  return HAL_SUCCESS;
}

}  // namespace

namespace prosdk::platform::hal {

bool ShiftIn(int32_t clockGpio, int32_t dataGpio, int bits, uint32_t highNs, uint32_t lowNs,
             uint64_t* out, int32_t* status) {
  if (const char* error = detail::CheckShiftIn(bits, highNs, lowNs)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return false;
  }
  *status = CheckPin(clockGpio, true, "ShiftIn clock");
  if (*status == HAL_SUCCESS) {
    *status = CheckPin(dataGpio, false, "ShiftIn data");
  }
  if (*status != HAL_SUCCESS) {
    return false;
  }
  uint64_t value = 0;
  for (int i = 0; i < bits; ++i) {
    HALSIM_SetDIOValue(clockGpio, true);
    value = (value << 1) | (HALSIM_GetDIOValue(dataGpio) ? 1u : 0u);
    HALSIM_SetDIOValue(clockGpio, false);
  }
  *out = value;
  return true;
}

bool WriteWaveform(int32_t gpio, std::span<const LevelSpan> spans, int32_t* status) {
  if (const char* error = detail::CheckWaveform(spans)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return false;
  }
  *status = CheckPin(gpio, true, "WriteWaveform");
  if (*status != HAL_SUCCESS) {
    return false;
  }
  for (const LevelSpan& s : spans) {
    HALSIM_SetDIOValue(gpio, s.level);
  }
  return true;
}

}  // namespace prosdk::platform::hal

#endif  // !PROSDK_TARGET_HW
