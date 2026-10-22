// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/DIO.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: digital I/O on any free GPIO (design/hal-esp32.md, "DIO").
// Inputs have the internal pull-up enabled (an open switch reads true, like the roboRIO and
// systemcore). Outputs start low and can be read back. On an emergency stop every DIO output is
// driven low and further writes are ignored. Digital PWM generators and pulses are not
// implemented (systemcore does not implement them either): they report HAL_HANDLE_ERROR.

#include "prosdk/hal/DIO.h"

#include <driver/gpio.h>

#include <memory>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

struct DioPin final : PinDriver {
  int gpio = -1;
  ~DioPin() override {
    SetOutputPin(gpio, false);
    if (IsValidGpio(gpio) && !EStopped()) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
  }
};

esp_err_t ConfigureDirection(int gpio, bool input) {
  auto pin = static_cast<gpio_num_t>(gpio);
  if (input) {
    SetOutputPin(gpio, false);
    esp_err_t err = gpio_set_direction(pin, GPIO_MODE_INPUT);
    if (err == ESP_OK) {
      err = gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    }
    return err;
  }
  gpio_set_level(pin, 0);
  esp_err_t err = gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
  if (err == ESP_OK) {
    err = gpio_set_pull_mode(pin, GPIO_FLOATING);
  }
  SetOutputPin(gpio, true);
  return err;
}

constexpr const char* kNoDigitalPwm =
    "Digital PWM generators are not supported by the ESP32-S3 HAL (use PWM)";
constexpr const char* kNoPulse = "DIO pulses are not supported by the ESP32-S3 HAL";

}  // namespace

namespace prosdk::hal::init {
void InitializeDIO() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_DigitalHandle HAL_InitializeDIOPort(int32_t channel, HAL_Bool input,
                                        const char* allocationLocation, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (!input && EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "DIO: the robot is emergency stopped; outputs stay off until reboot");
    return HAL_INVALID_HANDLE;
  }
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::DIO, "DIO");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  auto pin = std::make_unique<DioPin>();
  pin->gpio = channel;
  gpio_reset_pin(static_cast<gpio_num_t>(channel));
  esp_err_t err = ConfigureDirection(channel, input);
  if (err != ESP_OK) {
    pin.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::DIO);
    *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                        std::string("DIO: GPIO setup failed (") + EspErrName(err) + ")");
    return HAL_INVALID_HANDLE;
  }
  port->mode = input ? PinMode::kDigitalInput : PinMode::kDigitalOutput;
  port->driver = std::move(pin);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  *status = HAL_SUCCESS;
  return handle;
}

HAL_Bool HAL_CheckDIOChannel(int32_t channel) {
  return IsValidGpio(channel);
}

void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle) {
  auto port = smartIoHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(dioPortHandle, HAL_HandleEnum::DIO, std::move(port), "DIO");
}

void HAL_SetDIOSimDevice(HAL_DigitalHandle handle, HAL_SimDeviceHandle device) {}

HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoDigitalPwm);
  return HAL_INVALID_HANDLE;
}

void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator) {}

void HAL_SetDigitalPWMRate(double rate, int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoDigitalPwm);
}

void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                                int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoDigitalPwm);
}

void HAL_SetDigitalPWMPPS(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                          int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoDigitalPwm);
}

void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator, int32_t channel,
                                    int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoDigitalPwm);
}

void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value, int32_t* status) {
  auto port = smartIoHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (port->mode != PinMode::kDigitalOutput) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE, "DIO: the channel is an input");
    return;
  }
  *status = HAL_SUCCESS;
  if (EStopped()) {
    return;  // held low by the emergency stop
  }
  gpio_set_level(static_cast<gpio_num_t>(port->channel), value ? 1 : 0);
}

void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input, int32_t* status) {
  auto port = smartIoHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  if (!input && EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "DIO: the robot is emergency stopped; outputs stay off until reboot");
    return;
  }
  esp_err_t err = ConfigureDirection(port->channel, input);
  if (err != ESP_OK) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        std::string("DIO: GPIO direction change failed (") + EspErrName(err) +
                            ")");
    return;
  }
  port->mode = input ? PinMode::kDigitalInput : PinMode::kDigitalOutput;
  *status = HAL_SUCCESS;
}

HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto port = smartIoHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  *status = HAL_SUCCESS;
  return gpio_get_level(static_cast<gpio_num_t>(port->channel)) != 0;
}

HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  auto port = smartIoHandles->Get(dioPortHandle, HAL_HandleEnum::DIO);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }
  *status = HAL_SUCCESS;
  return port->mode == PinMode::kDigitalInput;
}

void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLength, int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoPulse);
}

void HAL_PulseMultiple(uint32_t channelMask, double pulseLength, int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoPulse);
}

HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoPulse);
  return false;
}

HAL_Bool HAL_IsAnyPulsing(int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoPulse);
  return false;
}

}  // extern "C"
