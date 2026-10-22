// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_* semantics follow allwpilib hal/src/main/native/systemcore/AddressableLED.cpp (WPILib BSD
// license).
//
// ESP32-S3 HAL backend: WS2812-style LED strips on RMT TX channels (design/hal-esp32.md,
// "Addressable LEDs"). As on systemcore, all outputs share one buffer of
// HAL_ADDRESSABLE_LED_MAX_LEN (1024) LEDs; each output sends its own [start, start + length)
// slice. HAL_SetAddressableLEDData converts the colors into the strip's wire order, then sends
// every output with a non-zero length (800 kHz; about 30 us per LED, a full 1024-LED buffer takes
// about 31 ms). Up to 3 outputs (the ESP32-S3 has 4 RMT TX channels; probot-core's status LED
// uses one); the 3 are shared with the pulse transmitters (PulseIO.cpp, rmt::AcquireTxChannel in
// Esp32Internal.hpp). LED outputs are not gated by the robot state; an emergency stop drives the
// data line low (the strip keeps its last colors).

#include "prosdk/hal/AddressableLED.h"

#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include <cstring>
#include <format>
#include <memory>
#include <mutex>

#include "Esp32Internal.hpp"
#include "prosdk/hal/AddressableLEDTypes.h"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/HandlesInternal.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr uint32_t kResolutionHz = 10000000;  // 0.1 us per tick
constexpr int kTxTimeoutMs = 100;

struct LedOutput;

std::mutex gMutex;
uint8_t gBuffer[HAL_ADDRESSABLE_LED_MAX_LEN * 3];
LedOutput* gOutputs[kNumAddressableLeds] = {};

struct LedOutput final : PinDriver {
  int gpio = -1;
  rmt_channel_handle_t channel = nullptr;
  rmt_encoder_handle_t encoder = nullptr;
  int32_t start = 0;
  int32_t length = 0;
  std::unique_ptr<uint8_t[]> tx;  // bytes in flight (the RMT driver reads them asynchronously)
  int32_t txCapacity = 0;
  bool rmtReserved = false;  // holds one of the shared RMT TX channels (rmt::AcquireTxChannel)

  ~LedOutput() override {
    {
      std::scoped_lock lock(gMutex);
      for (LedOutput*& slot : gOutputs) {
        if (slot == this) {
          slot = nullptr;
        }
      }
    }
    if (channel != nullptr) {
      rmt_tx_wait_all_done(channel, kTxTimeoutMs);
      rmt_disable(channel);
      rmt_del_channel(channel);
    }
    if (encoder != nullptr) {
      rmt_del_encoder(encoder);
    }
    if (rmtReserved) {
      rmt::ReleaseTxChannel();
    }
    if (IsValidGpio(gpio)) {
      SetOutputPin(gpio, false);
      if (!EStopped()) {
        gpio_reset_pin(static_cast<gpio_num_t>(gpio));
      }
    }
  }
};

HAL_Status StartOutput(LedOutput& out) {
  rmt_tx_channel_config_t cc{};
  cc.gpio_num = static_cast<gpio_num_t>(out.gpio);
  cc.clk_src = RMT_CLK_SRC_DEFAULT;
  cc.resolution_hz = kResolutionHz;
  cc.mem_block_symbols = 48;
  cc.trans_queue_depth = 1;
  esp_err_t err = rmt_new_tx_channel(&cc, &out.channel);
  if (err != ESP_OK) {
    out.channel = nullptr;
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("AddressableLED: no RMT TX channel left for GPIO {} ({})",
                                 out.gpio, EspErrName(err)));
  }
  rmt_bytes_encoder_config_t ec{};
  // WS2812: 0 = 0.3 us high + 0.9 us low, 1 = 0.9 us high + 0.3 us low
  ec.bit0.level0 = 1;
  ec.bit0.duration0 = 3;
  ec.bit0.level1 = 0;
  ec.bit0.duration1 = 9;
  ec.bit1.level0 = 1;
  ec.bit1.duration0 = 9;
  ec.bit1.level1 = 0;
  ec.bit1.duration1 = 3;
  ec.flags.msb_first = 1;
  err = rmt_new_bytes_encoder(&ec, &out.encoder);
  if (err == ESP_OK) {
    err = rmt_enable(out.channel);
  } else {
    out.encoder = nullptr;
  }
  if (err != ESP_OK) {
    return MakeError(HAL_NO_AVAILABLE_RESOURCES,
                     std::format("AddressableLED: RMT setup failed on GPIO {} ({})", out.gpio,
                                 EspErrName(err)));
  }
  return HAL_SUCCESS;
}

// gMutex held. Sends the output's slice of the shared buffer.
HAL_Status Send(LedOutput& out) {
  if (out.length <= 0) {
    return HAL_SUCCESS;
  }
  int32_t bytes = out.length * 3;
  if (rmt_tx_wait_all_done(out.channel, kTxTimeoutMs) != ESP_OK) {
    return MakeError(HAL_INCOMPATIBLE_STATE,
                     std::format("AddressableLED: GPIO {} transmit timeout", out.gpio));
  }
  if (out.txCapacity < bytes) {
    out.tx.reset(new uint8_t[bytes]);
    out.txCapacity = bytes;
  }
  std::memcpy(out.tx.get(), &gBuffer[out.start * 3], bytes);
  rmt_transmit_config_t tc{};
  esp_err_t err = rmt_transmit(out.channel, out.encoder, out.tx.get(), bytes, &tc);
  if (err != ESP_OK) {
    return MakeError(HAL_INCOMPATIBLE_STATE,
                     std::format("AddressableLED: GPIO {} transmit failed ({})", out.gpio,
                                 EspErrName(err)));
  }
  return HAL_SUCCESS;
}

// RGB input to the strip's byte order.
void ConvertPixels(uint8_t* dst, const HAL_AddressableLEDData* src, int32_t len,
                   HAL_AddressableLEDColorOrder order) {
  for (int32_t i = 0; i < len; ++i) {
    const uint8_t r = src[i].r;
    const uint8_t g = src[i].g;
    const uint8_t b = src[i].b;
    uint8_t* p = dst + i * 3;
    switch (order) {
      case HAL_ALED_RGB:
        p[0] = r, p[1] = g, p[2] = b;
        break;
      case HAL_ALED_RBG:
        p[0] = r, p[1] = b, p[2] = g;
        break;
      case HAL_ALED_BGR:
        p[0] = b, p[1] = g, p[2] = r;
        break;
      case HAL_ALED_BRG:
        p[0] = b, p[1] = r, p[2] = g;
        break;
      case HAL_ALED_GBR:
        p[0] = g, p[1] = b, p[2] = r;
        break;
      case HAL_ALED_GRB:
      default:
        p[0] = g, p[1] = r, p[2] = b;
        break;
    }
  }
}

LedOutput* GetOutput(HAL_AddressableLEDHandle handle, std::shared_ptr<SmartIo>* port,
                     int32_t* status) {
  *port = smartIoHandles->Get(handle, HAL_HandleEnum::ADDRESSABLE_LED);
  if (*port == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return nullptr;
  }
  return static_cast<LedOutput*>((*port)->driver.get());
}

}  // namespace

static_assert(sizeof(HAL_AddressableLEDData) == 3, "LED Data must be 3 bytes");

namespace prosdk::hal::init {
void InitializeAddressableLED() {}
}  // namespace prosdk::hal::init

extern "C" {

HAL_AddressableLEDHandle HAL_InitializeAddressableLED(int32_t channel,
                                                      const char* allocationLocation,
                                                      int32_t* status) {
  prosdk::hal::init::CheckInit();
  if (EStopped()) {
    *status = MakeError(HAL_INCOMPATIBLE_STATE,
                        "AddressableLED: the robot is emergency stopped; outputs stay off until "
                        "reboot");
    return HAL_INVALID_HANDLE;
  }
  auto resource = AllocateSmartIo(channel, HAL_HandleEnum::ADDRESSABLE_LED, "AddressableLED");
  if (!resource) {
    *status = resource.error();
    return HAL_INVALID_HANDLE;
  }
  auto [handle, port] = *resource;
  auto out = std::make_unique<LedOutput>();
  out->gpio = channel;
  {
    std::scoped_lock lock(gMutex);
    int slot = -1;
    for (int i = 0; i < kNumAddressableLeds; ++i) {
      if (gOutputs[i] == nullptr) {
        slot = i;
        break;
      }
    }
    if (slot < 0 || !rmt::AcquireTxChannel()) {
      *status = MakeError(HAL_NO_AVAILABLE_RESOURCES,
                          std::format("AddressableLED: all {} RMT TX channels are in use (LED "
                                      "outputs and pulse transmitters share them)",
                                      kNumRmtTxChannels));
    } else {
      out->rmtReserved = true;
      *status = StartOutput(*out);
    }
    if (*status == HAL_SUCCESS) {
      gOutputs[slot] = out.get();
    }
  }
  if (*status != HAL_SUCCESS) {
    out.reset();
    smartIoHandles->Free(handle, HAL_HandleEnum::ADDRESSABLE_LED);
    return HAL_INVALID_HANDLE;
  }
  SetOutputPin(channel, true);  // the emergency stop drives the data line low (LEDs freeze)
  port->mode = PinMode::kAddressableLed;
  port->driver = std::move(out);
  port->previousAllocation = allocationLocation ? allocationLocation : "";
  return handle;
}

void HAL_FreeAddressableLED(HAL_AddressableLEDHandle handle) {
  auto port = smartIoHandles->Get(handle, HAL_HandleEnum::ADDRESSABLE_LED);
  if (port == nullptr) {
    return;
  }
  FreeSmartIo(handle, HAL_HandleEnum::ADDRESSABLE_LED, std::move(port), "AddressableLED");
}

void HAL_SetAddressableLEDStart(HAL_AddressableLEDHandle handle, int32_t start,
                                int32_t* status) {
  std::shared_ptr<SmartIo> port;
  LedOutput* out = GetOutput(handle, &port, status);
  if (out == nullptr) {
    return;
  }
  std::scoped_lock lock(gMutex);
  if (start < 0 || start + out->length > HAL_ADDRESSABLE_LED_MAX_LEN) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("LED start {} out of range (start + length <= {})", start,
                                    HAL_ADDRESSABLE_LED_MAX_LEN));
    return;
  }
  out->start = start;
  *status = HAL_SUCCESS;
}

void HAL_SetAddressableLEDLength(HAL_AddressableLEDHandle handle, int32_t length,
                                 int32_t* status) {
  std::shared_ptr<SmartIo> port;
  LedOutput* out = GetOutput(handle, &port, status);
  if (out == nullptr) {
    return;
  }
  std::scoped_lock lock(gMutex);
  if (length < 0 || out->start + length > HAL_ADDRESSABLE_LED_MAX_LEN) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        std::format("LED length {} out of range (start + length <= {})", length,
                                    HAL_ADDRESSABLE_LED_MAX_LEN));
    return;
  }
  out->length = length;
  *status = HAL_SUCCESS;
}

void HAL_SetAddressableLEDData(int32_t start, int32_t length,
                               HAL_AddressableLEDColorOrder colorOrder,
                               const struct HAL_AddressableLEDData* data, int32_t* status) {
  if (start < 0 || start >= HAL_ADDRESSABLE_LED_MAX_LEN || length < 0 ||
      (start + length) >= HAL_ADDRESSABLE_LED_MAX_LEN) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  std::scoped_lock lock(gMutex);
  ConvertPixels(&gBuffer[start * 3], data, length, colorOrder);
  *status = HAL_SUCCESS;
  for (LedOutput* out : gOutputs) {
    if (out != nullptr) {
      int32_t s = Send(*out);
      if (*status == HAL_SUCCESS) {
        *status = s;
      }
    }
  }
}

}  // extern "C"
