// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// HAL_CAN_* semantics follow allwpilib hal/src/main/native/systemcore/CAN.cpp (WPILib BSD
// license).
//
// ESP32-S3 HAL backend: CAN bus 0 on the TWAI controller (design/hal-esp32.md, "CAN").
// Needs an external transceiver and prosdk::platform::SetCanConfig(tx, rx, bitrate) before the
// first CAN call; the bus starts on first use. Classic CAN only (8 data bytes; FD frames are
// rejected). One "prosdk-can" thread receives frames (latest frame per id, plus stream sessions)
// and sends periodic frames. No FRC heartbeat is sent, so FRC CAN motor controllers stay
// disabled. After an emergency stop periodic frames stop and new sends are refused (driving the
// TX pin low would hold the bus dominant and jam it, so CAN TX is not forced low).
// The TX/RX GPIOs are claimed in the HAL pin table when the bus starts (DIO/PWM/... on them fail);
// the bus is never stopped, so the claim lasts until reboot.

#include "prosdk/hal/CAN.h"

#include <driver/gpio.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <deque>
#include <format>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "Esp32Internal.hpp"
#include "prosdk/hal/CANAPITypes.h"
#include "prosdk/hal/ErrorHandling.hpp"
#include "prosdk/hal/Errors.h"
#include "prosdk/hal/handles/UnlimitedHandleResource.hpp"
#include "prosdk/platform/HalConfig.hpp"
#include "prosdk/platform/Thread.hpp"
#include "prosdk/util/timestamp.hpp"

using namespace prosdk::hal;
using namespace prosdk::hal::esp32;

namespace {

constexpr uint32_t kIdMask = 0x1FFFFFFF;
constexpr TickType_t kRxWaitTicks = pdMS_TO_TICKS(2);
constexpr TickType_t kTxWaitTicks = pdMS_TO_TICKS(5);

struct StreamStorage {
  StreamStorage(uint32_t maxMessages, uint32_t mask, uint32_t messageId)
      : maxMessages{maxMessages}, mask{mask}, messageId{messageId} {}
  uint32_t maxMessages;
  uint32_t mask;
  uint32_t messageId;
  std::deque<HAL_CANStreamMessage> received;
  bool didOverflow = false;

  void CheckFrame(const HAL_CANStreamMessage& message) {
    if ((message.messageId & mask) != (messageId & mask)) {
      return;
    }
    if (received.size() >= maxMessages) {
      didOverflow = true;
      received.pop_front();
    }
    received.push_back(message);
  }
};

struct Periodic {
  uint32_t messageId;
  twai_message_t frame;
  int64_t periodUs;
  int64_t nextUs;
};

std::mutex gStartMutex;
std::atomic<bool> gRunning{false};
std::atomic<bool> gEStopped{false};
std::atomic<uint32_t> gBusOffCount{0};
std::atomic<uint32_t> gTxFullCount{0};

std::mutex gReadMutex;  // gFrames, gStreams
std::map<uint32_t, HAL_CANStreamMessage> gFrames;
std::vector<StreamStorage*> gStreams;

std::mutex gPeriodicMutex;
std::vector<Periodic> gPeriodic;

UnlimitedHandleResource<HAL_CANStreamHandle, StreamStorage, HAL_HandleEnum::CAN_STREAM>*
    streamHandles;

int64_t NowUs() {
  return prosdk::util::Now() / 1000;
}

uint32_t ToHalId(const twai_message_t& frame) {
  uint32_t id = frame.identifier & kIdMask;
  if (!frame.extd) {
    id |= HAL_CAN_IS_FRAME_11BIT;
  }
  if (frame.rtr) {
    id |= HAL_CAN_IS_FRAME_REMOTE;
  }
  return id;
}

twai_message_t ToFrame(uint32_t messageId, const HAL_CANMessage& message) {
  twai_message_t frame{};
  const bool is11 = (messageId & HAL_CAN_IS_FRAME_11BIT) != 0;
  frame.extd = is11 ? 0 : 1;
  frame.rtr = (messageId & HAL_CAN_IS_FRAME_REMOTE) != 0 ? 1 : 0;
  frame.identifier = messageId & (is11 ? 0x7FF : kIdMask);
  frame.data_length_code = std::min<uint8_t>(message.dataSize, 8);
  std::memcpy(frame.data, message.data, frame.data_length_code);
  return frame;
}

void OnFrame(const twai_message_t& frame) {
  HAL_CANStreamMessage msg{};
  msg.messageId = ToHalId(frame);
  msg.message.timeStamp = prosdk::util::Now();
  msg.message.message.flags = HAL_CAN_NO_FLAGS;
  msg.message.message.dataSize = frame.data_length_code > 8 ? 8 : frame.data_length_code;
  std::memcpy(msg.message.message.data, frame.data, msg.message.message.dataSize);
  std::scoped_lock lock(gReadMutex);
  gFrames[msg.messageId] = msg;
  for (StreamStorage* stream : gStreams) {
    stream->CheckFrame(msg);
  }
}

void SendDuePeriodic() {
  std::scoped_lock lock(gPeriodicMutex);
  if (gEStopped.load(std::memory_order_acquire)) {
    gPeriodic.clear();
    return;
  }
  int64_t now = NowUs();
  for (Periodic& p : gPeriodic) {
    if (now >= p.nextUs) {
      if (twai_transmit(&p.frame, 0) != ESP_OK) {
        gTxFullCount.fetch_add(1, std::memory_order_relaxed);
      }
      p.nextUs += p.periodUs;
      if (p.nextUs < now) {
        p.nextUs = now + p.periodUs;
      }
    }
  }
}

void CanThreadMain() {
  twai_state_t lastState = TWAI_STATE_RUNNING;
  for (;;) {
    twai_message_t frame{};
    while (twai_receive(&frame, kRxWaitTicks) == ESP_OK) {
      OnFrame(frame);
      SendDuePeriodic();
    }
    SendDuePeriodic();
    twai_status_info_t info{};
    if (twai_get_status_info(&info) == ESP_OK) {
      if (info.state == TWAI_STATE_BUS_OFF && lastState != TWAI_STATE_BUS_OFF) {
        gBusOffCount.fetch_add(1, std::memory_order_relaxed);
        twai_initiate_recovery();
      } else if (info.state == TWAI_STATE_STOPPED && lastState == TWAI_STATE_RECOVERING) {
        twai_start();  // recovery finished
      }
      lastState = info.state;
    }
  }
}

// Starts the TWAI controller and the CAN thread once.
int32_t EnsureStarted() {
  if (gRunning.load(std::memory_order_acquire)) {
    return HAL_SUCCESS;
  }
  std::scoped_lock lock(gStartMutex);
  if (gRunning.load(std::memory_order_relaxed)) {
    return HAL_SUCCESS;
  }
  prosdk::platform::CanConfig config = prosdk::platform::GetCanConfig();
  if (config.tx < 0 || config.rx < 0) {
    return MakeError(HAL_ERR_CANSessionMux_NotInitialized,
                     "CAN has no pins: call prosdk::platform::SetCanConfig(tx, rx, bitrate) "
                     "first (and connect a CAN transceiver)");
  }
  HAL_DigitalHandle claims[2] = {HAL_INVALID_HANDLE, HAL_INVALID_HANDLE};
  auto releaseClaims = [&] {
    for (HAL_DigitalHandle& claim : claims) {
      ReleasePin(claim, HAL_HandleEnum::CAN);
      claim = HAL_INVALID_HANDLE;
    }
  };
  for (int i = 0; i < 2; ++i) {
    auto claimed = ClaimPin(i == 0 ? config.tx : config.rx, HAL_HandleEnum::CAN, "CAN",
                            i == 0 ? "CAN bus 0 (TWAI TX)" : "CAN bus 0 (TWAI RX)");
    if (!claimed) {
      releaseClaims();
      return claimed.error();
    }
    claims[i] = *claimed;
  }
  twai_timing_config_t timing;
  switch (config.bitrate) {
    case 125000:
      timing = TWAI_TIMING_CONFIG_125KBITS();
      break;
    case 250000:
      timing = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 500000:
      timing = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 1000000:
      timing = TWAI_TIMING_CONFIG_1MBITS();
      break;
    default:  // SetCanConfig accepts only the rates above
      releaseClaims();
      return MakeError(HAL_ERR_CANSessionMux_NotInitialized,
                       std::format("CAN: unsupported bit rate {}", config.bitrate));
  }
  twai_general_config_t general{};
  general.controller_id = 0;
  general.mode = TWAI_MODE_NORMAL;
  general.tx_io = static_cast<gpio_num_t>(config.tx);
  general.rx_io = static_cast<gpio_num_t>(config.rx);
  general.clkout_io = TWAI_IO_UNUSED;
  general.bus_off_io = TWAI_IO_UNUSED;
  general.tx_queue_len = 16;
  general.rx_queue_len = 32;
  general.alerts_enabled = TWAI_ALERT_NONE;
  general.clkout_divider = 0;
  general.intr_flags = ESP_INTR_FLAG_LEVEL1;
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  if (err == ESP_OK) {
    err = twai_start();
    if (err != ESP_OK) {
      twai_driver_uninstall();
    }
  }
  if (err != ESP_OK) {
    for (int gpio : {config.tx, config.rx}) {
      gpio_reset_pin(static_cast<gpio_num_t>(gpio));
    }
    releaseClaims();
    return MakeError(HAL_ERR_CANSessionMux_NotInitialized,
                     std::format("CAN could not start on TX {} / RX {} ({})", config.tx,
                                 config.rx, EspErrName(err)));
  }
  {
    prosdk::platform::ThreadConfig tc;
    tc.name = "prosdk-can";
    prosdk::platform::ScopedThreadConfig scoped{tc};
    std::thread(CanThreadMain).detach();
  }
  gRunning.store(true, std::memory_order_release);
  return HAL_SUCCESS;
}

void RemovePeriodic(uint32_t messageId) {
  std::scoped_lock lock(gPeriodicMutex);
  std::erase_if(gPeriodic, [&](const Periodic& p) { return p.messageId == messageId; });
}

}  // namespace

namespace prosdk::hal::esp32::can {
void OnEmergencyStop() {
  gEStopped.store(true, std::memory_order_release);
}
}  // namespace prosdk::hal::esp32::can

namespace prosdk::hal::init {
void InitializeCAN() {
  static UnlimitedHandleResource<HAL_CANStreamHandle, StreamStorage, HAL_HandleEnum::CAN_STREAM>
      handles;
  streamHandles = &handles;
}
}  // namespace prosdk::hal::init

extern "C" {

void HAL_CAN_SendMessage(int32_t busId, uint32_t messageId, const struct HAL_CANMessage* message,
                         int32_t periodMs, int32_t* status) {
  *status = HAL_SUCCESS;
  if (busId != 0 || message == nullptr || periodMs < HAL_CAN_SEND_PERIOD_STOP_REPEATING) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  if (periodMs == HAL_CAN_SEND_PERIOD_STOP_REPEATING) {
    RemovePeriodic(messageId);
    return;
  }
  if ((message->flags & (HAL_CAN_FD_DATALENGTH | HAL_CAN_FD_BITRATESWITCH)) != 0 ||
      message->dataSize > 8) {
    *status = MakeError(HAL_PARAMETER_OUT_OF_RANGE,
                        "CAN FD frames are not supported by the ESP32-S3 (classic CAN, 8 bytes)");
    return;
  }
  if (gEStopped.load(std::memory_order_acquire)) {
    *status = MakeError(HAL_ERR_CANSessionMux_NotAllowed,
                        "CAN: the robot is emergency stopped; sends are refused until reboot");
    return;
  }
  *status = EnsureStarted();
  if (*status != HAL_SUCCESS) {
    return;
  }
  twai_message_t frame = ToFrame(messageId, *message);
  RemovePeriodic(messageId);
  if (twai_transmit(&frame, kTxWaitTicks) != ESP_OK) {
    gTxFullCount.fetch_add(1, std::memory_order_relaxed);
    *status = HAL_WARN_CANSessionMux_TxQueueFull;
    return;
  }
  if (periodMs > 0) {
    std::scoped_lock lock(gPeriodicMutex);
    int64_t periodUs = static_cast<int64_t>(periodMs) * 1000;
    gPeriodic.push_back(Periodic{messageId, frame, periodUs, NowUs() + periodUs});
  }
}

void HAL_CAN_ReceiveMessage(int32_t busId, uint32_t messageId,
                            struct HAL_CANReceiveMessage* message, int32_t* status) {
  if (busId != 0) {
    message->message.dataSize = 0;
    message->timeStamp = 0;
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  *status = EnsureStarted();
  if (*status != HAL_SUCCESS) {
    message->message.dataSize = 0;
    message->timeStamp = 0;
    return;
  }
  std::scoped_lock lock(gReadMutex);
  auto it = gFrames.find(messageId);
  if (it == gFrames.end() || it->second.message.timeStamp == 0) {
    message->message.dataSize = 0;
    message->timeStamp = 0;
    *status = HAL_ERR_CANSessionMux_MessageNotFound;
    return;
  }
  *message = it->second.message;
  it->second.message.timeStamp = 0;  // each frame is returned once
  *status = HAL_SUCCESS;
}

HAL_CANStreamHandle HAL_CAN_OpenStreamSession(int32_t busId, uint32_t messageId,
                                              uint32_t messageIDMask, uint32_t maxMessages,
                                              int32_t* status) {
  if (busId != 0 || maxMessages == 0) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return HAL_INVALID_HANDLE;
  }
  *status = EnsureStarted();
  if (*status != HAL_SUCCESS) {
    return HAL_INVALID_HANDLE;
  }
  auto stream = std::make_shared<StreamStorage>(maxMessages, messageIDMask, messageId);
  auto handle = streamHandles->Allocate(stream);
  if (handle == HAL_INVALID_HANDLE) {
    *status = HAL_NO_AVAILABLE_RESOURCES;
    return HAL_INVALID_HANDLE;
  }
  std::scoped_lock lock(gReadMutex);
  gStreams.push_back(stream.get());
  return handle;
}

void HAL_CAN_CloseStreamSession(HAL_CANStreamHandle sessionHandle) {
  auto stream = streamHandles->Free(sessionHandle);
  if (stream == nullptr) {
    return;
  }
  std::scoped_lock lock(gReadMutex);
  std::erase(gStreams, stream.get());
}

void HAL_CAN_ReadStreamSession(HAL_CANStreamHandle sessionHandle,
                               struct HAL_CANStreamMessage* messages, uint32_t messagesToRead,
                               uint32_t* messagesRead, int32_t* status) {
  *status = HAL_SUCCESS;
  if (messages == nullptr || messagesRead == nullptr) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  auto stream = streamHandles->Get(sessionHandle);
  if (!stream) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  std::scoped_lock lock(gReadMutex);
  size_t count = std::min(static_cast<size_t>(messagesToRead), stream->received.size());
  for (size_t i = 0; i < count; ++i) {
    messages[i] = stream->received.front();
    stream->received.pop_front();
  }
  *messagesRead = static_cast<uint32_t>(count);
  if (stream->didOverflow) {
    stream->didOverflow = false;
    *status = HAL_ERR_CANSessionMux_SessionOverrun;
  }
}

void HAL_CAN_GetCANStatus(int32_t busId, float* percentBusUtilization, uint32_t* busOffCount,
                          uint32_t* txFullCount, uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount, int32_t* status) {
  *percentBusUtilization = 0;  // not measured
  *busOffCount = gBusOffCount.load(std::memory_order_relaxed);
  *txFullCount = gTxFullCount.load(std::memory_order_relaxed);
  *receiveErrorCount = 0;
  *transmitErrorCount = 0;
  if (busId != 0) {
    *status = HAL_PARAMETER_OUT_OF_RANGE;
    return;
  }
  if (!gRunning.load(std::memory_order_acquire)) {
    *status = HAL_ERR_CANSessionMux_NotInitialized;
    return;
  }
  twai_status_info_t info{};
  if (twai_get_status_info(&info) != ESP_OK) {
    *status = HAL_ERR_CANSessionMux_NotInitialized;
    return;
  }
  *receiveErrorCount = info.rx_error_counter;
  *transmitErrorCount = info.tx_error_counter;
  *status = HAL_SUCCESS;
}

}  // extern "C"
