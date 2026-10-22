// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/AnalogInput.cpp (WPILib BSD
// license).
//
// ESP32-S3 HAL backend: analog inputs on ADC1 (design/hal-esp32.md, "Analog inputs").
// Channel = GPIO 1..10 (ADC1 channels 0..9). ADC2 (GPIO 11..20) is not offered: it cannot be read
// while WiFi is on, and the Driver Station link is WiFi. Readings go through the Arduino one-shot
// ADC driver (12 bits, 12 dB attenuation: about 0..3.1 V); volts use the chip's eFuse curve
// fitting calibration.

#include "prosdk/hal/AnalogInput.h"

#include <esp32-hal-adc.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

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

constexpr int32_t kMaxRaw = 4095;
constexpr double kUncalibratedFullScale = 3.1;  // volts at kMaxRaw without calibration

std::once_flag gCaliOnce;
adc_cali_handle_t gCali = nullptr;

adc_cali_handle_t Calibration() {
  std::call_once(gCaliOnce, [] {
    adc_cali_curve_fitting_config_t config{};
    config.unit_id = ADC_UNIT_1;
    config.chan = ADC_CHANNEL_0;
    config.atten = ADC_ATTEN_DB_12;
    config.bitwidth = ADC_BITWIDTH_12;
    if (adc_cali_create_scheme_curve_fitting(&config, &gCali) != ESP_OK) {
      gCali = nullptr;
      Log("AnalogInput: no ADC calibration in eFuse; volts are approximate",
          prosdk::platform::ConsoleLevel::kWarning);
    }
  });
  return gCali;
}

double RawToVolts(int32_t raw) {
  if (raw < 0) {
    raw = 0;
  } else if (raw > kMaxRaw) {
    raw = kMaxRaw;
  }
  adc_cali_handle_t cali = Calibration();
  int mv = 0;
  if (cali != nullptr && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
    return mv / 1000.0;
  }
  return raw * kUncalibratedFullScale / kMaxRaw;
}

struct AnalogPin final : PinDriver {
  int gpio = -1;
  ~AnalogPin() override { ReleaseArduinoPin(gpio); }
};

std::shared_ptr<SmartIo> GetPort(HAL_AnalogInputHandle handle, int32_t* status) {
  auto port = smartIoHandles->Get(handle, HAL_HandleEnum::ANALOG_INPUT);
  if (port == nullptr) {
    *status = HAL_HANDLE_ERROR;
  } else {
    *status = HAL_SUCCESS;
  }
  return port;
}

}  // namespace

namespace prosdk::hal::init {
void InitializeAnalogInput() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_AnalogInputHandle HAL_InitializeAnalogInputPort(int32_t channel,
                                                    const char* allocationLocation,
                                                    int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (!HAL_CheckAnalogInputChannel(channel)) {
    *status = MakeErrorIndexOutOfRange(
        HAL_RESOURCE_OUT_OF_RANGE,
        "Invalid Index for Analog (GPIO 1-10, ADC1; ADC2 cannot be used with WiFi)", 1,
        kNumAnalogInputs - 1, channel);
    return HAL_INVALID_HANDLE;
  }
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::ANALOG_INPUT, "Analog");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  auto pin = std::make_unique<AnalogPin>();
  pin->gpio = channel;
  analogSetPinAttenuation(static_cast<uint8_t>(channel), ADC_11db);
  analogRead(static_cast<uint8_t>(channel));  // attaches the one-shot ADC to the pin
  port->mode = PinMode::kAnalogInput;
  port->driver = std::move(pin);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  *status = HAL_SUCCESS;
  return handle;
}

void HAL_FreeAnalogInputPort(HAL_AnalogInputHandle analogPortHandle) {
  auto port = smartIoHandles->Get(analogPortHandle, HAL_HandleEnum::ANALOG_INPUT);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(analogPortHandle, HAL_HandleEnum::ANALOG_INPUT, std::move(port), "Analog");
}

HAL_Bool HAL_CheckAnalogModule(int32_t module) {
  return module == 1;
}

HAL_Bool HAL_CheckAnalogInputChannel(int32_t channel) {
  return channel >= 1 && channel < kNumAnalogInputs;
}

void HAL_SetAnalogInputSimDevice(HAL_AnalogInputHandle handle, HAL_SimDeviceHandle device) {}

int32_t HAL_GetAnalogValue(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
  auto port = GetPort(analogPortHandle, status);
  if (port == nullptr) {
    return 0;
  }
  return analogRead(port->channel);
}

int32_t HAL_GetAnalogVoltsToValue(HAL_AnalogInputHandle analogPortHandle, double voltage,
                                  int32_t* status) {
  *status = HAL_SUCCESS;
  if (voltage <= RawToVolts(0)) {
    return 0;
  }
  if (voltage >= RawToVolts(kMaxRaw)) {
    return kMaxRaw;
  }
  // the calibration curve is monotonic: smallest raw value reaching the voltage
  int32_t lo = 0;
  int32_t hi = kMaxRaw;
  while (lo < hi) {
    int32_t mid = (lo + hi) / 2;
    if (RawToVolts(mid) < voltage) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

double HAL_GetAnalogVoltage(HAL_AnalogInputHandle analogPortHandle, int32_t* status) {
  auto port = GetPort(analogPortHandle, status);
  if (port == nullptr) {
    return 0;
  }
  return analogReadMilliVolts(port->channel) / 1000.0;
}

double HAL_GetAnalogValueToVolts(HAL_AnalogInputHandle analogPortHandle, int32_t rawValue,
                                 int32_t* status) {
  *status = HAL_SUCCESS;
  return RawToVolts(rawValue);
}

}  // extern "C"
