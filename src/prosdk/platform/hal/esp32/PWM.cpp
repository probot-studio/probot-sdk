// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/PWM.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: servo/ESC PWM outputs on the MCPWM peripheral (design/hal-esp32.md,
// "PWM").
//
// Hardware: 2 MCPWM groups x 3 timers; each timer drives one operator with 2 comparators and
// 2 generators, so 12 outputs on any GPIO, and the two outputs of one timer share its period.
// Timers run at 1 MHz (1 tick = 1 us), counting up: the generator goes high at zero and low at
// the comparator (the pulse width in us). A period change moves the output to a timer that has
// that period (or a free one); HAL_NO_AVAILABLE_RESOURCES when there is none.
//
// Output gate: while probot-core's outputsEnabled() is false (or after an emergency stop) every
// output is forced low (no pulses: ESCs and servos see "no signal"). Pulse 0 also forces low and
// 0xFFFF forces high, like systemcore.

#include "prosdk/hal/PWM.h"

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>

#include <format>
#include <memory>
#include <mutex>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr uint32_t kResolutionHz = 1000000;  // 1 tick = 1 us
constexpr int kNumGroups = 2;
constexpr int kNumTimers = kNumPwmOutputs / 2;
constexpr uint32_t kDefaultPeriodUs = 20000;

struct PwmTimer {
  mcpwm_timer_handle_t timer = nullptr;
  mcpwm_oper_handle_t oper = nullptr;
  uint32_t periodUs = 0;
  int users = 0;
};

struct PwmOutput;

std::mutex gMutex;
PwmTimer gTimers[kNumTimers];
PwmOutput* gOutputs[kNumPwmOutputs] = {};
bool gGate = false;  // gate last applied to all outputs

struct PwmOutput final : PinDriver {
  int gpio = -1;
  int timerIndex = -1;
  mcpwm_cmpr_handle_t cmpr = nullptr;
  mcpwm_gen_handle_t gen = nullptr;
  int32_t pulseUs = kPwmDisabled;
  uint32_t periodUs = kDefaultPeriodUs;

  ~PwmOutput() override;
};

void DeleteTimerIfUnused(int index) {
  PwmTimer& t = gTimers[index];
  if (t.users > 0 || t.timer == nullptr) {
    return;
  }
  if (t.oper != nullptr) {
    mcpwm_del_operator(t.oper);
    t.oper = nullptr;
  }
  mcpwm_timer_start_stop(t.timer, MCPWM_TIMER_STOP_EMPTY);
  mcpwm_timer_disable(t.timer);
  mcpwm_del_timer(t.timer);
  t = PwmTimer{};
}

// Finds a timer with this period and a free generator, or creates one. -1 if none is left.
int AcquireTimer(uint32_t periodUs) {
  for (int i = 0; i < kNumTimers; ++i) {
    if (gTimers[i].timer != nullptr && gTimers[i].periodUs == periodUs && gTimers[i].users < 2) {
      return i;
    }
  }
  for (int i = 0; i < kNumTimers; ++i) {
    PwmTimer& t = gTimers[i];
    if (t.timer != nullptr) {
      continue;
    }
    for (int group = 0; group < kNumGroups; ++group) {
      mcpwm_timer_config_t tc{};
      tc.group_id = group;
      tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
      tc.resolution_hz = kResolutionHz;
      tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
      tc.period_ticks = periodUs;
      if (mcpwm_new_timer(&tc, &t.timer) != ESP_OK) {
        t.timer = nullptr;
        continue;
      }
      mcpwm_operator_config_t oc{};
      oc.group_id = group;
      if (mcpwm_new_operator(&oc, &t.oper) != ESP_OK) {
        t.oper = nullptr;
        mcpwm_del_timer(t.timer);
        t.timer = nullptr;
        continue;
      }
      if (mcpwm_operator_connect_timer(t.oper, t.timer) != ESP_OK ||
          mcpwm_timer_enable(t.timer) != ESP_OK ||
          mcpwm_timer_start_stop(t.timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
        mcpwm_del_operator(t.oper);
        mcpwm_del_timer(t.timer);
        t = PwmTimer{};
        continue;
      }
      t.periodUs = periodUs;
      t.users = 0;
      return i;
    }
    return -1;  // no group has a free timer/operator
  }
  return -1;
}

// Drives one output from its pulse and the gate. gMutex held. EStopped() is checked here, after
// any generator (re)creation: a generator created after the e-stop handler rerouted the pin takes
// the pin back from plain GPIO, and gGate may still hold the value from before the e-stop.
void ApplyOutput(PwmOutput& out, bool enabled) {
  if (out.gen == nullptr) {
    return;
  }
  if (!enabled || EStopped() || out.pulseUs == kPwmDisabled) {
    mcpwm_generator_set_force_level(out.gen, 0, true);
  } else if (out.pulseUs == kPwmAlwaysHigh) {
    mcpwm_generator_set_force_level(out.gen, 1, true);
  } else {
    mcpwm_comparator_set_compare_value(out.cmpr, static_cast<uint32_t>(out.pulseUs));
    mcpwm_generator_set_force_level(out.gen, -1, true);
  }
}

// gMutex held. Applies a gate change to every output.
void ApplyGateLocked(bool enabled) {
  if (enabled == gGate) {
    return;
  }
  gGate = enabled;
  for (PwmOutput* out : gOutputs) {
    if (out != nullptr) {
      ApplyOutput(*out, enabled);
    }
  }
}

void Detach(PwmOutput& out) {
  if (out.gen != nullptr) {
    mcpwm_generator_set_force_level(out.gen, 0, true);
    mcpwm_del_generator(out.gen);
    out.gen = nullptr;
  }
  if (out.cmpr != nullptr) {
    mcpwm_del_comparator(out.cmpr);
    out.cmpr = nullptr;
  }
  if (out.timerIndex >= 0) {
    --gTimers[out.timerIndex].users;
    DeleteTimerIfUnused(out.timerIndex);
    out.timerIndex = -1;
  }
}

// gMutex held. Puts the output on a timer with this period. The output stays forced low until
// ApplyOutput.
HAL_Status Attach(PwmOutput& out, uint32_t periodUs) {
  int index = AcquireTimer(periodUs);
  if (index < 0) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("PWM: no MCPWM timer left for a {} ms period on GPIO {} (12 "
                                 "outputs; the 2 outputs of a timer share one period)",
                                 periodUs / 1000, out.gpio));
  }
  PwmTimer& t = gTimers[index];
  ++t.users;
  out.timerIndex = index;
  mcpwm_comparator_config_t cc{};
  cc.flags.update_cmp_on_tez = 1;
  esp_err_t err = mcpwm_new_comparator(t.oper, &cc, &out.cmpr);
  if (err == ESP_OK) {
    mcpwm_generator_config_t gc{};
    gc.gen_gpio_num = out.gpio;
    err = mcpwm_new_generator(t.oper, &gc, &out.gen);
  }
  if (err == ESP_OK) {
    mcpwm_generator_set_force_level(out.gen, 0, true);
    mcpwm_gen_timer_event_action_t onEmpty{};
    onEmpty.direction = MCPWM_TIMER_DIRECTION_UP;
    onEmpty.event = MCPWM_TIMER_EVENT_EMPTY;
    onEmpty.action = MCPWM_GEN_ACTION_HIGH;
    mcpwm_gen_compare_event_action_t onCompare{};
    onCompare.direction = MCPWM_TIMER_DIRECTION_UP;
    onCompare.comparator = out.cmpr;
    onCompare.action = MCPWM_GEN_ACTION_LOW;
    err = mcpwm_generator_set_action_on_timer_event(out.gen, onEmpty);
    if (err == ESP_OK) {
      err = mcpwm_generator_set_action_on_compare_event(out.gen, onCompare);
    }
    if (err == ESP_OK) {
      err = mcpwm_comparator_set_compare_value(out.cmpr, 0);
    }
  }
  if (err != ESP_OK) {
    Detach(out);
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("PWM: MCPWM setup failed on GPIO {} ({})", out.gpio,
                                 EspErrName(err)));
  }
  out.periodUs = periodUs;
  return HAL_SUCCESS;
}

PwmOutput::~PwmOutput() {
  {
    std::scoped_lock lock(gMutex);
    for (PwmOutput*& slot : gOutputs) {
      if (slot == this) {
        slot = nullptr;
      }
    }
    Detach(*this);
  }
  SetOutputPin(gpio, false);
  if (IsValidGpio(gpio) && !EStopped()) {
    gpio_reset_pin(static_cast<gpio_num_t>(gpio));
  }
}

PwmOutput* GetOutput(const std::shared_ptr<SmartIo>& port) {
  return static_cast<PwmOutput*>(port->driver.get());
}

}  // namespace

namespace prosdk::hal::esp32::pwm {
bool ApplyGate(bool enabled, bool blocking) {
  std::unique_lock lock(gMutex, std::defer_lock);
  if (blocking) {
    lock.lock();
  } else if (!lock.try_lock()) {
    return false;
  }
  ApplyGateLocked(enabled);
  return true;
}
}  // namespace prosdk::hal::esp32::pwm

namespace prosdk::hal::init {
void InitializePWM() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_DigitalHandle HAL_InitializePWMPort(int32_t channel, const char* allocationLocation,
                                        int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "PWM: the robot is emergency stopped; outputs stay off until reboot");
    return HAL_INVALID_HANDLE;
  }
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::PWM, "PWM");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  auto out = std::make_unique<PwmOutput>();
  out->gpio = channel;
  {
    std::scoped_lock lock(gMutex);
    int slot = -1;
    for (int i = 0; i < kNumPwmOutputs; ++i) {
      if (gOutputs[i] == nullptr) {
        slot = i;
        break;
      }
    }
    *status = slot < 0 ? MakeError(HAL_NO_AVAILABLE_RESOURCES,
                                   "PWM: all 12 MCPWM outputs are in use")
                       : Attach(*out, kDefaultPeriodUs);
    if (*status == HAL_SUCCESS) {
      gOutputs[slot] = out.get();
      ApplyOutput(*out, gGate);  // pulse 0: forced low
    }
  }
  if (*status != HAL_SUCCESS) {
    out.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::PWM);
    return HAL_INVALID_HANDLE;
  }
  SetOutputPin(channel, true);
  port->mode = PinMode::kPwm;
  port->driver = std::move(out);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

void HAL_FreePWMPort(HAL_DigitalHandle pwmPortHandle) {
  auto port = smartIoHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(pwmPortHandle, HAL_HandleEnum::PWM, std::move(port), "PWM");
}

void HAL_SetPWMSimDevice(HAL_DigitalHandle handle, HAL_SimDeviceHandle device) {}

HAL_Bool HAL_CheckPWMChannel(int32_t channel) {
  return IsValidGpio(channel);
}

void HAL_SetPWMPulseTimeMicroseconds(HAL_DigitalHandle pwmPortHandle,
                                     int32_t microsecondPulseTime, int32_t* status) {
  auto port = smartIoHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (microsecondPulseTime < 0 ||
      (microsecondPulseTime != kPwmAlwaysHigh && microsecondPulseTime >= kMaxPwmPulseUs)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("Pulse time {} out of range. Expect [0-4096) or 0xFFFF",
                                    microsecondPulseTime));
    return;
  }
  bool enabled = OutputsEnabled();
  std::scoped_lock lock(gMutex);
  PwmOutput* out = GetOutput(port);
  out->pulseUs = microsecondPulseTime;
  ApplyGateLocked(enabled);
  ApplyOutput(*out, gGate);
  *status = HAL_SUCCESS;
}

int32_t HAL_GetPWMPulseTimeMicroseconds(HAL_DigitalHandle pwmPortHandle, int32_t* status) {
  auto port = smartIoHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  std::scoped_lock lock(gMutex);
  *status = HAL_SUCCESS;
  return GetOutput(port)->pulseUs;
}

void HAL_SetPWMOutputPeriod(HAL_DigitalHandle pwmPortHandle, int32_t period, int32_t* status) {
  auto port = smartIoHandles->Get(pwmPortHandle, HAL_HandleEnum::PWM);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  uint32_t periodUs = 0;
  switch (period) {
    case 0:
      periodUs = 5000;
      break;
    case 1:
    case 2:
      periodUs = 10000;
      break;
    case 3:
      periodUs = 20000;
      break;
    default:
      *status = HAL_PARAMETER_OUT_OF_RANGE;
      return;
  }
  if (EStopped()) {
    // a period change recreates the MCPWM generator, which would reconnect the pin
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "PWM: the robot is emergency stopped; outputs stay off until reboot");
    return;
  }
  bool enabled = OutputsEnabled();
  std::scoped_lock lock(gMutex);
  PwmOutput* out = GetOutput(port);
  *status = HAL_SUCCESS;
  if (out->periodUs == periodUs && out->gen != nullptr) {
    return;
  }
  uint32_t oldPeriod = out->periodUs;
  Detach(*out);
  *status = Attach(*out, periodUs);
  if (*status != HAL_SUCCESS) {
    // keep the old period; its timer slot was just released, so this succeeds
    int32_t ignored = Attach(*out, oldPeriod);
    (void)ignored;
  }
  ApplyGateLocked(enabled);
  ApplyOutput(*out, gGate);
}

}  // extern "C"
