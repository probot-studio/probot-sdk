// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/DIO.cpp (WPILib BSD license).
//
// ESP32-S3 HAL backend: digital I/O on any free GPIO (design/hal-esp32.md, "DIO").
// Inputs have the internal pull-up enabled (an open switch reads true, like the roboRIO and
// systemcore). Outputs start low and can be read back. On an emergency stop every DIO output is
// driven low and further writes are ignored.
//
// Digital PWM generators (DigitalOutput::EnablePWM/UpdateDutyCycle/SetPWMRate) run on the LEDC
// peripheral through the Arduino LEDC API: 6 generators, one rate for all of them (5 Hz to
// 40 kHz, default 500 Hz), 14 to 10 bits of duty resolution. Unlike the roboRIO they are gated:
// the duty is forced to 0 while the output gate is closed (Esp32Internal.hpp), and the emergency
// stop drives their pins low like every DIO output. PPS mode and pulses are not implemented
// (systemcore does not implement them either): they report HAL_HANDLE_ERROR.

#include "prosdk/hal/DIO.h"

#include <driver/gpio.h>
#include <esp32-hal-ledc.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

#include <array>
#include <atomic>
#include <cmath>
#include <format>
#include <memory>
#include <mutex>

#include "Esp32Internal.hpp"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"
#include "prosdk/hal/handles/LimitedHandleResource.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr uint64_t Bit(int gpio) {
  return uint64_t{1} << gpio;
}

// GPIOs currently allocated as DIO outputs (a digital PWM generator may only drive one of them).
constinit std::atomic<uint64_t> gDioOutputs{0};

// ---- digital PWM generators (LEDC) ----

constexpr double kDpwmMinRate = 5.0;       // LEDC divider limit at 14 bits (80 MHz APB)
constexpr double kDpwmMaxRate = 40000.0;   // keeps at least 10 bits of duty resolution
constexpr double kDpwmDefaultRate = 500.0;
constexpr double kLedcClockHz = 80e6;
constexpr int kLedcMaxBits = 14;

struct DigitalPwm {
  int gpio = -1;          // output channel, -1 = none
  double duty = 0.0;      // requested duty cycle [0, 1]
  bool attached = false;  // the pin is routed to an LEDC channel
};

LimitedHandleResource<HAL_DigitalPWMHandle, uint8_t, kNumDigitalPwmOutputs,
                      HAL_HandleEnum::DIGITAL_PWM>* gDpwmHandles;
std::mutex gDpwmMutex;
std::array<DigitalPwm, kNumDigitalPwmOutputs> gDpwm;
uint32_t gDpwmRate = static_cast<uint32_t>(kDpwmDefaultRate);
uint8_t gDpwmBits = 0;
bool gDpwmGate = false;  // gate last applied to the generators

uint8_t BitsForRate(uint32_t rate) {
  int bits = static_cast<int>(std::floor(std::log2(kLedcClockHz / rate)));
  if (bits > kLedcMaxBits) {
    bits = kLedcMaxBits;
  }
  return static_cast<uint8_t>(bits < 1 ? 1 : bits);
}

// Routes a pin back to the plain GPIO output, driven low (same as the emergency stop).
void ForceLow(int gpio) {
  gpio_set_level(static_cast<gpio_num_t>(gpio), 0);
  esp_rom_gpio_connect_out_signal(gpio, SIG_GPIO_OUT_IDX, false, false);
}

// gDpwmMutex held. Writes the gated duty of an attached generator.
void WriteDutyLocked(const DigitalPwm& g) {
  if (!g.attached || EStopped()) {
    return;
  }
  uint32_t full = uint32_t{1} << gDpwmBits;
  uint32_t counts = 0;
  if (gDpwmGate) {
    counts = static_cast<uint32_t>(std::lround(g.duty * full));
    if (counts > full) {
      counts = full;
    }
  }
  // ledcWrite turns (full - 1) into full on; full itself is accepted by the LEDC driver
  ledcWrite(static_cast<uint8_t>(g.gpio), counts);
}

// gDpwmMutex held. Routes g.gpio to a new LEDC channel at the current rate.
bool AttachLocked(DigitalPwm& g) {
  if (gDpwmBits == 0) {
    gDpwmBits = BitsForRate(gDpwmRate);
  }
  if (!ledcAttach(static_cast<uint8_t>(g.gpio), gDpwmRate, gDpwmBits)) {
    return false;
  }
  g.attached = true;
  WriteDutyLocked(g);
  // the emergency stop may have run while the pin was being routed to LEDC: undo the routing
  if (EStopped()) {
    ForceLow(g.gpio);
  }
  return true;
}

// gDpwmMutex held. Returns the pin to a plain DIO output (low) when restore is set.
void DetachLocked(DigitalPwm& g, bool restore) {
  if (!g.attached) {
    return;
  }
  ledcDetach(static_cast<uint8_t>(g.gpio));
  g.attached = false;
  auto pin = static_cast<gpio_num_t>(g.gpio);
  gpio_set_level(pin, 0);
  if (restore && !EStopped()) {
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
  }
}

// Stops any generator that drives this GPIO (the DIO is being freed or turned into an input).
void ReleaseDigitalPwmPin(int gpio) {
  std::scoped_lock lock(gDpwmMutex);
  for (DigitalPwm& g : gDpwm) {
    if (g.gpio == gpio) {
      DetachLocked(g, false);
      g.gpio = -1;
    }
  }
}

std::shared_ptr<uint8_t> GetGenerator(HAL_DigitalPWMHandle handle, int32_t* status) {
  auto id = gDpwmHandles == nullptr ? nullptr : gDpwmHandles->Get(handle);
  if (id == nullptr) {
    *status = HAL_HANDLE_ERROR;
  }
  return id;
}

struct DioPin final : PinDriver {
  int gpio = -1;
  ~DioPin() override {
    ReleaseDigitalPwmPin(gpio);
    gDioOutputs.fetch_and(~Bit(gpio), std::memory_order_acq_rel);
    SetOutputPin(gpio, false);
    if (IsValidGpio(gpio) && !EStopped()) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
  }
};

esp_err_t ConfigureDirection(int gpio, bool input) {
  auto pin = static_cast<gpio_num_t>(gpio);
  if (input) {
    ReleaseDigitalPwmPin(gpio);
    gDioOutputs.fetch_and(~Bit(gpio), std::memory_order_acq_rel);
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
  gDioOutputs.fetch_or(Bit(gpio), std::memory_order_acq_rel);
  return err;
}

constexpr const char* kNoPps =
    "Digital PWM PPS mode is not supported by the ESP32-S3 HAL (use EnablePWM)";
constexpr const char* kNoPulse = "DIO pulses are not supported by the ESP32-S3 HAL";

}  // namespace

namespace prosdk::hal::esp32::dio {
bool ApplyGate(bool enabled, bool blocking) {
  std::unique_lock lock(gDpwmMutex, std::defer_lock);
  if (blocking) {
    lock.lock();
  } else if (!lock.try_lock()) {
    return false;
  }
  if (enabled != gDpwmGate) {
    gDpwmGate = enabled;
    for (const DigitalPwm& g : gDpwm) {
      WriteDutyLocked(g);
    }
  }
  return true;
}
}  // namespace prosdk::hal::esp32::dio

namespace prosdk::hal::init {
void InitializeDIO() {
  static LimitedHandleResource<HAL_DigitalPWMHandle, uint8_t, kNumDigitalPwmOutputs,
                               HAL_HandleEnum::DIGITAL_PWM>
      handles;
  gDpwmHandles = &handles;
}
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
  prosdk::hal::init::CheckInit();
  auto handle = gDpwmHandles->Allocate();
  if (handle == HAL_INVALID_HANDLE) {
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("Digital PWM: all {} generators are in use",
                                    kNumDigitalPwmOutputs));
    return HAL_INVALID_HANDLE;
  }
  auto id = gDpwmHandles->Get(handle);
  if (id == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return HAL_INVALID_HANDLE;
  }
  *id = static_cast<uint8_t>(getHandleIndex(handle));
  {
    std::scoped_lock lock(gDpwmMutex);
    gDpwm[*id] = DigitalPwm{};
  }
  *status = HAL_SUCCESS;
  return handle;
}

void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator) {
  int32_t status = 0;
  auto id = GetGenerator(pwmGenerator, &status);
  if (id == nullptr) {
    return;
  }
  {
    std::scoped_lock lock(gDpwmMutex);
    DetachLocked(gDpwm[*id], true);
    gDpwm[*id] = DigitalPwm{};
  }
  gDpwmHandles->Free(pwmGenerator);
}

void HAL_SetDigitalPWMRate(double rate, int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (!(rate >= kDpwmMinRate && rate <= kDpwmMaxRate)) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("Digital PWM: rate {} Hz is outside {} to {} Hz", rate,
                                    kDpwmMinRate, kDpwmMaxRate));
    return;
  }
  std::scoped_lock lock(gDpwmMutex);
  auto hz = static_cast<uint32_t>(std::lround(rate));
  if (hz == gDpwmRate && gDpwmBits != 0) {
    *status = HAL_SUCCESS;
    return;
  }
  // re-route every attached generator to a channel at the new rate (detach all first, so the
  // old LEDC timer is free and no other LEDC user's timer is reconfigured)
  std::array<bool, kNumDigitalPwmOutputs> wasAttached{};
  for (size_t i = 0; i < gDpwm.size(); ++i) {
    wasAttached[i] = gDpwm[i].attached;
    DetachLocked(gDpwm[i], false);
  }
  gDpwmRate = hz;
  gDpwmBits = BitsForRate(hz);
  *status = HAL_SUCCESS;
  for (size_t i = 0; i < gDpwm.size(); ++i) {
    if (wasAttached[i] && !AttachLocked(gDpwm[i])) {
      gpio_set_direction(static_cast<gpio_num_t>(gDpwm[i].gpio), GPIO_MODE_INPUT_OUTPUT);
      *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                          std::format("Digital PWM: no LEDC channel/timer for GPIO {} at {} Hz",
                                      gDpwm[i].gpio, hz));
    }
  }
}

void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                                int32_t* status) {
  auto id = GetGenerator(pwmGenerator, status);
  if (id == nullptr) {
    return;
  }
  if (!(dutyCycle > 0.0)) {
    dutyCycle = 0.0;  // also NaN
  } else if (dutyCycle > 1.0) {
    dutyCycle = 1.0;
  }
  std::scoped_lock lock(gDpwmMutex);
  gDpwm[*id].duty = dutyCycle;
  WriteDutyLocked(gDpwm[*id]);
  *status = HAL_SUCCESS;
}

void HAL_SetDigitalPWMPPS(HAL_DigitalPWMHandle pwmGenerator, double dutyCycle,
                          int32_t* status) {
  *status = MakeError(HAL_HANDLE_ERROR, kNoPps);
}

void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator, int32_t channel,
                                    int32_t* status) {
  auto id = GetGenerator(pwmGenerator, status);
  if (id == nullptr) {
    return;
  }
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "Digital PWM: the robot is emergency stopped; outputs stay off until "
                        "reboot");
    return;
  }
  if (!IsValidGpio(channel) ||
      (gDioOutputs.load(std::memory_order_acquire) & Bit(channel)) == 0) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        std::format("Digital PWM: channel {} is not an allocated digital output",
                                    channel));
    return;
  }
  std::scoped_lock lock(gDpwmMutex);
  DigitalPwm& g = gDpwm[*id];
  for (const DigitalPwm& other : gDpwm) {
    if (&other != &g && other.gpio == channel) {
      *status = MakeError(HAL_RESOURCE_IS_ALLOCATED,
                          std::format("Digital PWM: GPIO {} already has a generator", channel));
      return;
    }
  }
  if (g.attached && g.gpio == channel) {
    *status = HAL_SUCCESS;
    return;
  }
  DetachLocked(g, true);
  g.gpio = channel;
  if (!AttachLocked(g)) {
    g.gpio = -1;
    *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                        std::format("Digital PWM: no LEDC channel/timer for GPIO {} at {} Hz (8 "
                                    "LEDC channels and 4 timers are shared with analogWrite and "
                                    "other Arduino LEDC users)",
                                    channel, gDpwmRate));
    return;
  }
  *status = HAL_SUCCESS;
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
  auto pin = static_cast<gpio_num_t>(port->channel);
  gpio_set_level(pin, value ? 1 : 0);
  // the e-stop handler may have driven the pin low between the check and the write (it sets the
  // flag before touching pins): undo a high write
  if (value && EStopped()) {
    gpio_set_level(pin, 0);
  }
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
