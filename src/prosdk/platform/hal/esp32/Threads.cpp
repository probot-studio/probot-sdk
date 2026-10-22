// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ESP32-S3 HAL backend: thread priorities. WPILib uses Linux real-time priorities (0 = normal,
// 1..99 = SCHED_RR). On the ESP32-S3 every thread is a FreeRTOS task and only a few priorities
// are safe next to probot-core (user task 1, system loop 4, WiFi/lwIP higher), so the range is
// mapped (design/hal-esp32.md, "Threads"):
//   WPILib 0 -> FreeRTOS 1 (core's user task level), 1..39 -> 2, 40..99 -> 3.
// Reading back maps FreeRTOS <= 1 -> 0, 2 -> 1, >= 3 -> 40.
// A handle is a pointer to a pthread_t (std::thread::native_handle()), as on systemcore.

#include "prosdk/hal/Threads.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pthread.h>
#include <sched.h>

#include "prosdk/hal/Errors.h"

namespace {

int ToFreeRtos(int32_t priority) {
  if (priority <= 0) {
    return 1;
  }
  if (priority < 40) {
    return 2;
  }
  return 3;
}

int32_t ToWpilib(int freeRtos) {
  if (freeRtos <= 1) {
    return 0;
  }
  if (freeRtos == 2) {
    return 1;
  }
  return 40;
}

}  // namespace

extern "C" {

HAL_Status HAL_GetThreadPriority(NativeThreadHandle handle, int32_t* priority) {
  if (handle == nullptr || priority == nullptr) {
    return HAL_NULL_PARAMETER;
  }
  sched_param sch{};
  int policy = 0;
  if (pthread_getschedparam(*reinterpret_cast<const pthread_t*>(handle), &policy, &sch) != 0) {
    return HAL_THREAD_PRIORITY_ERROR;
  }
  *priority = ToWpilib(sch.sched_priority);
  return HAL_SUCCESS;
}

// The current thread may be a plain FreeRTOS task (core's user task), not a pthread.
HAL_Status HAL_GetCurrentThreadPriority(int32_t* priority) {
  if (priority == nullptr) {
    return HAL_NULL_PARAMETER;
  }
  *priority = ToWpilib(static_cast<int>(uxTaskPriorityGet(nullptr)));
  return HAL_SUCCESS;
}

HAL_Status HAL_SetThreadPriority(NativeThreadHandle handle, int32_t priority) {
  if (handle == nullptr) {
    return HAL_NULL_PARAMETER;
  }
  if (priority < 0 || priority > 99) {
    return HAL_THREAD_PRIORITY_RANGE_ERROR;
  }
  const pthread_t thread = *reinterpret_cast<const pthread_t*>(handle);
  sched_param sch{};
  int policy = 0;
  if (pthread_getschedparam(thread, &policy, &sch) != 0) {
    return HAL_THREAD_PRIORITY_ERROR;
  }
  sch.sched_priority = ToFreeRtos(priority);
  if (pthread_setschedparam(thread, policy, &sch) != 0) {
    return HAL_THREAD_PRIORITY_ERROR;
  }
  return HAL_SUCCESS;
}

HAL_Status HAL_SetCurrentThreadPriority(int32_t priority) {
  if (priority < 0 || priority > 99) {
    return HAL_THREAD_PRIORITY_RANGE_ERROR;
  }
  vTaskPrioritySet(nullptr, static_cast<UBaseType_t>(ToFreeRtos(priority)));
  return HAL_SUCCESS;
}

}  // extern "C"
