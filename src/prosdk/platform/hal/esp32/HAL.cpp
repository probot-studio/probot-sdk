// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Adapted for prosdk (Probot Studio) from allwpilib hal/src/main/native/systemcore/HAL.cpp and
// HALInitializer.cpp: the ESP32-S3 backend's HAL core (initialization, error messages, time,
// identity). Not vendored (hal.py never vendors the systemcore backend).

#include "prosdk/hal/HAL.h"

#include <esp_mac.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <format>
#include <utility>
#include <vector>

#include "Esp32Internal.hpp"
#include "prosdk/hal/CAN.h"
#include "prosdk/hal/Errors.h"
#include "prosdk/util/mutex.hpp"
#include "prosdk/util/string.h"
#include "prosdk/util/timestamp.hpp"

using namespace prosdk::hal;

static int64_t dsStartTime;
static prosdk::util::mutex gOnShutdownMutex;
static std::vector<std::pair<void*, void (*)(void*)>> gOnShutdown;

namespace prosdk::hal {
void InitializeDriverStation();
void WaitForInitialPacket();
namespace init {
std::atomic_bool HAL_IsInitialized{false};
void RunInitialize() {
  HAL_Initialize();
}

void InitializeHAL() {
  InitializeSmartIo();
  InitializeAddressableLED();
  InitializeAnalogInput();
  InitializeCAN();
  InitializeCANAPI();
  InitializeCounter();
  InitializeDIO();
  InitializeDutyCycle();
  InitializeEncoder();
  InitializeI2C();
  InitializeMain();
  InitializeNotifier();
  InitializePWM();
  InitializeSerialPort();
  InitializeDriverStationPump();
}
}  // namespace init

int64_t GetDSInitializeTime() {
  return dsStartTime;
}
}  // namespace prosdk::hal

extern "C" {

const char* HAL_GetErrorMessage(int32_t code) {
  switch (code) {
    case 0:
      return "";
    case HAL_VOLTAGE_OUT_OF_RANGE:
      return HAL_VOLTAGE_OUT_OF_RANGE_MESSAGE;
    case HAL_INCOMPATIBLE_STATE:
      return HAL_INCOMPATIBLE_STATE_MESSAGE;
    case HAL_NO_AVAILABLE_RESOURCES:
      return HAL_NO_AVAILABLE_RESOURCES_MESSAGE;
    case HAL_RESOURCE_IS_ALLOCATED:
      return HAL_RESOURCE_IS_ALLOCATED_MESSAGE;
    case HAL_RESOURCE_OUT_OF_RANGE:
      return HAL_RESOURCE_OUT_OF_RANGE_MESSAGE;
    case HAL_HANDLE_ERROR:
      return HAL_HANDLE_ERROR_MESSAGE;
    case HAL_NULL_PARAMETER:
      return HAL_NULL_PARAMETER_MESSAGE;
    case HAL_PARAMETER_OUT_OF_RANGE:
      return HAL_PARAMETER_OUT_OF_RANGE_MESSAGE;
    case HAL_COUNTER_NOT_SUPPORTED:
      return HAL_COUNTER_NOT_SUPPORTED_MESSAGE;
    case HAL_ERR_CANSessionMux_InvalidBuffer:
      return HAL_ERR_CANSessionMux_InvalidBuffer_MESSAGE;
    case HAL_ERR_CANSessionMux_MessageNotFound:
      return HAL_ERR_CANSessionMux_MessageNotFound_MESSAGE;
    case HAL_WARN_CANSessionMux_NoToken:
      return HAL_WARN_CANSessionMux_NoToken_MESSAGE;
    case HAL_ERR_CANSessionMux_NotAllowed:
      return HAL_ERR_CANSessionMux_NotAllowed_MESSAGE;
    case HAL_ERR_CANSessionMux_NotInitialized:
      return HAL_ERR_CANSessionMux_NotInitialized_MESSAGE;
    case HAL_WARN_CANSessionMux_TxQueueFull:
      return HAL_WARN_CANSessionMux_TxQueueFull_MESSAGE;
    case HAL_WARN_CANSessionMux_SocketBufferFull:
      return HAL_WARN_CANSessionMux_SocketBufferFull_MESSAGE;
    case HAL_SERIAL_PORT_NOT_FOUND:
      return HAL_SERIAL_PORT_NOT_FOUND_MESSAGE;
    case HAL_THREAD_PRIORITY_ERROR:
      return HAL_THREAD_PRIORITY_ERROR_MESSAGE;
    case HAL_THREAD_PRIORITY_RANGE_ERROR:
      return HAL_THREAD_PRIORITY_RANGE_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_OPEN_ERROR:
      return HAL_SERIAL_PORT_OPEN_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_ERROR:
      return HAL_SERIAL_PORT_ERROR_MESSAGE;
    case HAL_CAN_TIMEOUT:
      return HAL_CAN_TIMEOUT_MESSAGE;
    case HAL_CAN_BUFFER_OVERRUN:
      return HAL_CAN_BUFFER_OVERRUN_MESSAGE;
    case HAL_USE_LAST_ERROR:
      return HAL_USE_LAST_ERROR_MESSAGE;
    default:
      return "Unknown error status";
  }
}

// SYSTEMCORE: the robot runtime, so RobotBase::IsReal() is true on the ESP32-S3.
HAL_RuntimeType HAL_GetRuntimeType(void) {
  return HAL_RUNTIME_SYSTEMCORE;
}

// The factory MAC address (eFuse), as 12 hex digits.
void HAL_GetSerialNumber(struct WPI_String* serialNumber) {
  uint8_t mac[6] = {};
  esp_efuse_mac_get_default(mac);
  char text[13];
  std::snprintf(text, sizeof(text), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3],
                mac[4], mac[5]);
  size_t len = std::strlen(text);
  auto write = WPI_AllocateString(serialNumber, len);
  std::memcpy(write, text, len);
}

void HAL_GetComments(struct WPI_String* comments) {
  comments->len = 0;
  comments->str = nullptr;
}

int32_t HAL_GetTeamNumber(void) {
  return 0;
}

int64_t HAL_GetMonotonicTime(void) {
  return prosdk::util::NowDefault();
}

// True while outputs are enabled (the roboRIO meaning: the FPGA drives outputs).
HAL_Bool HAL_GetSystemActive(int32_t* status) {
  prosdk::hal::init::CheckInit();
  *status = HAL_SUCCESS;
  return esp32::OutputsEnabled();
}

int32_t HAL_GetCommsDisableCount(int32_t* status) {
  prosdk::hal::init::CheckInit();
  *status = HAL_HANDLE_ERROR;
  return 0;
}

HAL_Bool HAL_GetRSLState(int32_t* status) {
  prosdk::hal::init::CheckInit();
  *status = HAL_HANDLE_ERROR;
  return false;
}

HAL_Bool HAL_Initialize(void) {
  static std::atomic_bool initialized{false};
  static prosdk::util::mutex initializeMutex;
  // Initial check, as if it's true initialization has finished
  if (initialized) {
    return true;
  }

  std::scoped_lock lock(initializeMutex);
  // Second check in case another thread was waiting
  if (initialized) {
    return true;
  }

  prosdk::hal::init::InitializeHAL();

  prosdk::hal::init::HAL_IsInitialized.store(true);

  prosdk::hal::InitializeDriverStation();

  dsStartTime = HAL_GetMonotonicTime();

  prosdk::hal::WaitForInitialPacket();

  initialized = true;
  return true;
}

void HAL_Shutdown(void) {
  std::vector<std::pair<void*, void (*)(void*)>> funcs;
  {
    std::scoped_lock lock(gOnShutdownMutex);
    funcs.swap(gOnShutdown);
  }
  for (auto&& func : funcs) {
    func.second(func.first);
  }
}

void HAL_OnShutdown(void* param, void (*func)(void*)) {
  std::scoped_lock lock(gOnShutdownMutex);
  gOnShutdown.emplace_back(param, func);
}

void HAL_SimPeriodicBefore(void) {}

void HAL_SimPeriodicAfter(void) {}

}  // extern "C"
