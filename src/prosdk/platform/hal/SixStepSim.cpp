// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of six-step commutation (prosdk/platform/hal/SixStep.hpp). The pins are DIO
// sim channels (0 to HAL_GetNumDigitalChannels() - 1). A DIO sim value callback on each Hall
// channel commutates synchronously, as the ESP32 Hall interrupt does: a test sets the Hall levels
// with DIOSim and reads the outputs with DIOSim. INLx is the enable level; INHx reads high while
// that phase gets the PWM (the duty itself is not simulated on the pin) and low otherwise. Step
// times come from HAL_GetMonotonicTime, so sim::PauseTiming/StepTiming control the speed.
//
// It applies the ESP32 backend's argument and table checks, its slot count and one owner per
// pin among the motors, and refuses pins that are allocated DIOs; it does not see other HAL
// users. There is no output gate and no emergency stop. Compiled into both builds by the
// platform/hal glob; empty on the robot.
//
// Threads: a DIO sim callback runs under its value's lock and HallCallback then takes gMutex and
// reads the other Hall values, so set the Hall levels and call the motor from one thread (the
// robot loop and its simulation periodic, or a test). Hall changes from two threads at once, or
// from one thread while another opens or closes a motor, can deadlock.

#include "prosdk/platform/config.hpp"

#if !PROSDK_TARGET_HW

#include <array>
#include <cmath>
#include <format>
#include <mutex>
#include <string>

#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/HAL.h"
#include "prosdk/hal/Ports.h"
#include "prosdk/hal/Value.h"
#include "prosdk/hal/simulation/DIOData.h"
#include "prosdk/platform/hal/SixStep.hpp"

namespace {

using prosdk::hal::MakeError;
using namespace prosdk::platform::hal;
using detail::SixStepMode;

struct Motor {
  bool open = false;
  SixStepPins pins;
  uint32_t periodTicks = 0;
  std::string allocation;
  SixStepTable table = kDefaultSixStepTable;
  SixStepMode mode = SixStepMode::kCoast;
  std::array<int32_t, 3> hallCallbacks{-1, -1, -1};
  SixStepFeedback feedback;
  int lastStep = -1;  // table step index of the last valid Hall state
};

// recursive: Apply sets DIO sim values, and a test's callback on an output may set a Hall level,
// which re-enters through HallCallback
std::recursive_mutex gMutex;
std::array<Motor, kMaxSixStepMotors> gMotors;

Motor* Find(int32_t handle) {
  int slot = handle - 1;
  return slot >= 0 && slot < kMaxSixStepMotors && gMotors[slot].open ? &gMotors[slot] : nullptr;
}

int ReadHalls(const SixStepPins& pins) {
  int state = 0;
  for (int i = 0; i < 3; ++i) {
    if (HALSIM_GetDIOValue(pins.hall[i])) {
      state |= 1 << i;
    }
  }
  return state;
}

// gMutex held. Writes the outputs for the current mode and Hall state.
void Apply(Motor& m) {
  m.feedback.phases = detail::PhasesFor(m.table, m.feedback.hallState, m.mode);
  for (int i = 0; i < 3; ++i) {
    SixStepPhase p = m.feedback.phases[i];
    HALSIM_SetDIOValue(m.pins.inl[i], p != SixStepPhase::kFloat);
    HALSIM_SetDIOValue(m.pins.inh[i], p == SixStepPhase::kPwm);
  }
}

// gMutex held. A new Hall state: count the step, then commutate.
void OnHallState(Motor& m, int state) {
  int64_t now = HAL_GetMonotonicTime();
  int step = detail::StepOf(m.table[static_cast<size_t>(state)]);
  if (step < 0) {
    ++m.feedback.badStates;
  } else if (m.lastStep >= 0 && step != m.lastStep) {
    int delta = detail::StepDelta(m.lastStep, step);
    if (delta == 3) {
      ++m.feedback.badStates;
    } else {
      m.feedback.periodUs = (now - m.feedback.lastStepUs) / delta;
      m.feedback.steps += delta;
      m.feedback.lastStepUs = now;
    }
  }
  if (step >= 0) {
    if (m.lastStep < 0) {
      m.feedback.lastStepUs = now;  // first valid state: the step clock starts here
    }
    m.lastStep = step;
  }
  m.feedback.hallState = state;
  Apply(m);
}

void HallCallback(const char* /*name*/, void* param, const HAL_Value* /*value*/) {
  std::scoped_lock lock(gMutex);
  auto* m = static_cast<Motor*>(param);
  if (!m->open) {
    return;
  }
  int state = ReadHalls(m->pins);
  if (state != m->feedback.hallState) {
    OnHallState(*m, state);
  }
}

// gMutex held. HAL_SUCCESS when all 9 pins are distinct DIO sim channels that no motor and no
// DIO uses.
int32_t CheckPins(const SixStepPins& pins) {
  std::array<int32_t, 9> all{};
  for (int i = 0; i < 3; ++i) {
    all[i] = pins.inh[i];
    all[3 + i] = pins.inl[i];
    all[6 + i] = pins.hall[i];
  }
  int32_t channels = HAL_GetNumDigitalChannels();
  for (size_t i = 0; i < all.size(); ++i) {
    int32_t pin = all[i];
    if (pin < 0 || pin >= channels) {
      return prosdk::hal::MakeErrorIndexOutOfRange(HAL_RESOURCE_OUT_OF_RANGE,
                                                   "Invalid Index for Six-step", 0,
                                                   channels - 1, pin);
    }
    for (size_t j = 0; j < i; ++j) {
      if (all[j] == pin) {
        return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                         std::format("six-step: GPIO {} is given twice", pin));
      }
    }
    if (HALSIM_GetDIOInitialized(pin)) {
      return MakeError(HAL_RESOURCE_IS_ALLOCATED,
                       std::format("six-step: GPIO {} is an allocated DIO", pin));
    }
    for (const Motor& m : gMotors) {
      if (!m.open) {
        continue;
      }
      for (int k = 0; k < 3; ++k) {
        if (m.pins.inh[k] == pin || m.pins.inl[k] == pin || m.pins.hall[k] == pin) {
          return prosdk::hal::MakeErrorPreviouslyAllocated(HAL_RESOURCE_IS_ALLOCATED,
                                                           "Six-step", pin, m.allocation);
        }
      }
    }
  }
  return HAL_SUCCESS;
}

}  // namespace

namespace prosdk::platform::hal {

int32_t OpenSixStep(const SixStepPins& pins, uint32_t pwmHz, const char* allocationLocation,
                    int32_t* status) {
  if (pwmHz < kMinSixStepPwmHz || pwmHz > kMaxSixStepPwmHz) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        "six-step: the PWM frequency must be 1 kHz to 100 kHz");
    return kInvalidSixStepHandle;
  }
  std::scoped_lock lock(gMutex);
  *status = CheckPins(pins);
  if (*status != HAL_SUCCESS) {
    return kInvalidSixStepHandle;
  }
  for (int slot = 0; slot < kMaxSixStepMotors; ++slot) {
    Motor& m = gMotors[slot];
    if (m.open) {
      continue;
    }
    m = Motor{};
    m.open = true;
    m.pins = pins;
    m.periodTicks = kSixStepResolutionHz / pwmHz;
    m.allocation = allocationLocation ? allocationLocation : "";
    m.feedback.hallState = ReadHalls(pins);
    m.lastStep = detail::StepOf(m.table[static_cast<size_t>(m.feedback.hallState)]);
    m.feedback.lastStepUs = HAL_GetMonotonicTime();
    for (int i = 0; i < 3; ++i) {
      m.hallCallbacks[i] =
          HALSIM_RegisterDIOValueCallback(pins.hall[i], &HallCallback, &m, false);
    }
    Apply(m);
    return slot + 1;
  }
  *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                      std::format("six-step: all {} motor slots are in use", kMaxSixStepMotors));
  return kInvalidSixStepHandle;
}

void SetSixStepTable(int32_t handle, const SixStepTable& table, int32_t* status) {
  if (const char* error = detail::CheckSixStepTable(table)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE, error);
    return;
  }
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  m->table = table;
  m->lastStep = detail::StepOf(table[static_cast<size_t>(m->feedback.hallState)]);
  Apply(*m);
  *status = HAL_SUCCESS;
}

void SetSixStepOutput(int32_t handle, double duty, bool brake, int32_t* status) {
  SixStepPins pins;
  {
    std::scoped_lock lock(gMutex);
    Motor* m = Find(handle);
    if (m == nullptr) {
      *status = HAL_HANDLE_ERROR;
      return;
    }
    pins = m->pins;
  }
  // the Hall levels are read without the lock (a Hall callback takes the Hall value's lock first)
  int state = ReadHalls(pins);
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (std::isnan(duty)) {
    duty = 0.0;
  }
  if (detail::DutyTicks(duty, m->periodTicks) == 0) {
    m->mode = brake ? SixStepMode::kBrake : SixStepMode::kCoast;
  } else {
    m->mode = duty > 0.0 ? SixStepMode::kForward : SixStepMode::kReverse;
  }
  // as on the robot, every call re-reads the Halls (recovers a missed edge at standstill)
  if (state != m->feedback.hallState) {
    OnHallState(*m, state);
  } else {
    Apply(*m);
  }
  *status = HAL_SUCCESS;
}

bool GetSixStepFeedback(int32_t handle, SixStepFeedback* out, int32_t* status) {
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  *out = m->feedback;
  out->nowUs = HAL_GetMonotonicTime();
  *status = HAL_SUCCESS;
  return true;
}

void CloseSixStep(int32_t handle) {
  std::scoped_lock lock(gMutex);
  Motor* m = Find(handle);
  if (m == nullptr) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    HALSIM_CancelDIOValueCallback(m->pins.hall[i], m->hallCallbacks[i]);
  }
  m->mode = SixStepMode::kCoast;
  Apply(*m);
  *m = Motor{};
}

}  // namespace prosdk::platform::hal

#endif  // !PROSDK_TARGET_HW
