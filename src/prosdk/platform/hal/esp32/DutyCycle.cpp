// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/DutyCycle.cpp (WPILib BSD
// license).
//
// ESP32-S3 HAL backend: duty-cycle (PWM) inputs on the MCPWM capture channels
// (design/hal-esp32.md, "Duty cycle"). 2 groups x 3 channels = 6 inputs on any GPIO, timed by the
// 80 MHz capture timer of the group. Both edges are captured in an interrupt; the period is
// rising edge to rising edge and the high time rising to falling edge. Without an edge for
// 100 ms the signal is treated as constant: frequency 0, output 1.0 or 0.0 from the pin level.

#include "prosdk/hal/DutyCycle.h"

#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include <atomic>
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

constexpr int kNumGroups = 2;
constexpr uint32_t kStaleMs = 100;

std::mutex gMutex;
mcpwm_cap_timer_handle_t gCaptureTimers[kNumGroups] = {};
uint32_t gResolutionHz[kNumGroups] = {};

uint32_t NowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

struct DutyInput final : PinDriver {
  int gpio = -1;
  int group = -1;
  mcpwm_cap_channel_handle_t channel = nullptr;
  // written by the capture interrupt
  std::atomic<uint32_t> lastRise{0};
  std::atomic<uint32_t> periodTicks{0};
  std::atomic<uint32_t> highTicks{0};
  std::atomic<uint32_t> lastEdgeMs{0};
  std::atomic<bool> seenRise{false};

  ~DutyInput() override {
    if (channel != nullptr) {
      mcpwm_capture_channel_disable(channel);
      mcpwm_del_capture_channel(channel);
    }
    if (IsValidGpio(gpio)) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
  }

  bool Fresh() const {
    return NowMs() - lastEdgeMs.load(std::memory_order_relaxed) < kStaleMs &&
           periodTicks.load(std::memory_order_relaxed) != 0;
  }
};

bool OnCapture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t* edata,
               void* context) {
  auto* in = static_cast<DutyInput*>(context);
  uint32_t value = edata->cap_value;
  if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
    if (in->seenRise.load(std::memory_order_relaxed)) {
      in->periodTicks.store(value - in->lastRise.load(std::memory_order_relaxed),
                            std::memory_order_relaxed);
    }
    in->lastRise.store(value, std::memory_order_relaxed);
    in->seenRise.store(true, std::memory_order_relaxed);
  } else if (in->seenRise.load(std::memory_order_relaxed)) {
    in->highTicks.store(value - in->lastRise.load(std::memory_order_relaxed),
                        std::memory_order_relaxed);
  }
  in->lastEdgeMs.store(NowMs(), std::memory_order_relaxed);
  return false;
}

// gMutex held. Creates and starts the group's capture timer once.
esp_err_t EnsureCaptureTimer(int group) {
  if (gCaptureTimers[group] != nullptr) {
    return ESP_OK;
  }
  mcpwm_capture_timer_config_t tc{};
  tc.group_id = group;
  tc.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  mcpwm_cap_timer_handle_t timer = nullptr;
  esp_err_t err = mcpwm_new_capture_timer(&tc, &timer);
  if (err != ESP_OK) {
    return err;
  }
  err = mcpwm_capture_timer_get_resolution(timer, &gResolutionHz[group]);
  if (err == ESP_OK) {
    err = mcpwm_capture_timer_enable(timer);
  }
  if (err == ESP_OK) {
    err = mcpwm_capture_timer_start(timer);
  }
  if (err != ESP_OK) {
    mcpwm_capture_timer_disable(timer);
    mcpwm_del_capture_timer(timer);
    return err;
  }
  gCaptureTimers[group] = timer;
  return ESP_OK;
}

HAL_Status StartCapture(DutyInput& in) {
  std::scoped_lock lock(gMutex);
  esp_err_t err = ESP_ERR_NOT_FOUND;
  for (int group = 0; group < kNumGroups; ++group) {
    err = EnsureCaptureTimer(group);
    if (err != ESP_OK) {
      continue;
    }
    mcpwm_capture_channel_config_t cc{};
    cc.gpio_num = in.gpio;
    cc.prescale = 1;
    cc.flags.pos_edge = 1;
    cc.flags.neg_edge = 1;
    cc.flags.pull_up = 1;
    err = mcpwm_new_capture_channel(gCaptureTimers[group], &cc, &in.channel);
    if (err != ESP_OK) {
      in.channel = nullptr;
      continue;
    }
    mcpwm_capture_event_callbacks_t cbs{};
    cbs.on_cap = &OnCapture;
    err = mcpwm_capture_channel_register_event_callbacks(in.channel, &cbs, &in);
    if (err == ESP_OK) {
      err = mcpwm_capture_channel_enable(in.channel);
    }
    if (err != ESP_OK) {
      mcpwm_del_capture_channel(in.channel);
      in.channel = nullptr;
      break;
    }
    in.group = group;
    return HAL_SUCCESS;
  }
  return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                   std::format("DutyCycle: no MCPWM capture channel left for GPIO {} (6 "
                               "inputs) ({})",
                               in.gpio, EspErrName(err)));
}

DutyInput* GetInput(HAL_DutyCycleHandle handle, std::shared_ptr<SmartIo>* port,
                    int32_t* status) {
  *port = smartIoHandles->Get(handle, HAL_HandleEnum::DUTY_CYCLE);
  if (*port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return nullptr;
  }
  *status = HAL_SUCCESS;
  return static_cast<DutyInput*>((*port)->driver.get());
}

}  // namespace

namespace prosdk::hal::init {
void InitializeDutyCycle() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_DutyCycleHandle HAL_InitializeDutyCycle(int32_t channel, const char* allocationLocation,
                                            int32_t* status) {
  prosdk::hal::init::CheckInit();
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::DUTY_CYCLE, "DutyCycle");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  auto input = std::make_unique<DutyInput>();
  input->gpio = channel;
  *status = StartCapture(*input);
  if (*status != HAL_SUCCESS) {
    input.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::DUTY_CYCLE);
    return HAL_INVALID_HANDLE;
  }
  port->mode = PinMode::kDutyCycle;
  port->driver = std::move(input);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

void HAL_FreeDutyCycle(HAL_DutyCycleHandle dutyCycleHandle) {
  auto port = smartIoHandles->Get(dutyCycleHandle, HAL_HandleEnum::DUTY_CYCLE);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(dutyCycleHandle, HAL_HandleEnum::DUTY_CYCLE, std::move(port), "DutyCycle");
}

void HAL_SetDutyCycleSimDevice(HAL_DutyCycleHandle handle, HAL_SimDeviceHandle device) {}

double HAL_GetDutyCycleFrequency(HAL_DutyCycleHandle dutyCycleHandle, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  DutyInput* in = GetInput(dutyCycleHandle, &port, status);
  if (in == nullptr || !in->Fresh()) {
    return 0.0;
  }
  return static_cast<double>(gResolutionHz[in->group]) /
         static_cast<double>(in->periodTicks.load(std::memory_order_relaxed));
}

double HAL_GetDutyCycleOutput(HAL_DutyCycleHandle dutyCycleHandle, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  DutyInput* in = GetInput(dutyCycleHandle, &port, status);
  if (in == nullptr) {
    return 0.0;
  }
  if (!in->Fresh()) {
    return gpio_get_level(static_cast<gpio_num_t>(in->gpio)) != 0 ? 1.0 : 0.0;
  }
  uint32_t period = in->periodTicks.load(std::memory_order_relaxed);
  uint32_t high = in->highTicks.load(std::memory_order_relaxed);
  if (high >= period) {
    return 1.0;
  }
  return static_cast<double>(high) / static_cast<double>(period);
}

// Nanoseconds, like systemcore.
int32_t HAL_GetDutyCycleHighTime(HAL_DutyCycleHandle dutyCycleHandle, int32_t* status) {
  std::shared_ptr<SmartIo> port;
  DutyInput* in = GetInput(dutyCycleHandle, &port, status);
  if (in == nullptr || !in->Fresh()) {
    return 0;
  }
  uint64_t high = in->highTicks.load(std::memory_order_relaxed);
  return static_cast<int32_t>(high * 1000000000ULL / gResolutionHz[in->group]);
}

}  // extern "C"
