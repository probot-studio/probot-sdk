// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: resource counts (design/hal-esp32.md, "Capacities"). "Channels" are
// channel-number spaces (channel = GPIO, so 49 including the GPIOs that do not exist); the other
// counts are hardware units that the pins share.

#include "prosdk/hal/Ports.h"

#include "Esp32Internal.hpp"

using namespace prosdk::hal::esp32;

extern "C" {

int32_t HAL_GetNumCanBuses(void) {
  return kNumCanBuses;
}
int32_t HAL_GetNumSmartIo(void) {
  return kNumSmartIo;
}
int32_t HAL_GetNumAnalogInputs(void) {
  return kNumAnalogInputs;
}
int32_t HAL_GetNumCounters(void) {
  return kNumPulseCounters;
}
int32_t HAL_GetNumDigitalChannels(void) {
  return kNumSmartIo;
}
int32_t HAL_GetNumPWMChannels(void) {
  return kNumSmartIo;
}
int32_t HAL_GetNumDigitalPWMOutputs(void) {
  return kNumDigitalPwmOutputs;
}
int32_t HAL_GetNumEncoders(void) {
  return kNumPulseCounters;
}
int32_t HAL_GetNumInterrupts(void) {
  return 0;
}
// CAN device id spaces (the WPILib values); the devices themselves are not supported.
int32_t HAL_GetNumCTREPCMModules(void) {
  return 63;
}
int32_t HAL_GetNumCTRESolenoidChannels(void) {
  return 8;
}
int32_t HAL_GetNumCTREPDPModules(void) {
  return 63;
}
int32_t HAL_GetNumCTREPDPChannels(void) {
  return 16;
}
int32_t HAL_GetNumREVPDHModules(void) {
  return 63;
}
int32_t HAL_GetNumREVPDHChannels(void) {
  return 24;
}
int32_t HAL_GetNumREVPHModules(void) {
  return 63;
}
int32_t HAL_GetNumREVPHChannels(void) {
  return 16;
}
int32_t HAL_GetNumDutyCycles(void) {
  return kNumDutyCycleInputs;
}
int32_t HAL_GetNumAddressableLEDs(void) {
  return kNumAddressableLeds;
}

}  // extern "C"
