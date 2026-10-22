// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// FreeRTOS side of the QEMU sketches (QemuSmoke, QemuBench): the runner task, the user task that
// each smoke case or benchmark group runs on (probot-core's user-task parameters), and the TASK
// report. Header-only, shared through -I<repo>/test/target/shared.

#pragma once

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "QemuUart.hpp"

namespace qemu {

/** The stack of every user task: prosdk's PROBOT_USER_STACK (prosdk.h). */
inline constexpr uint32_t kUserStack = 24576;

/** Free 8-bit heap in bytes (heap_caps_get_free_size(MALLOC_CAP_8BIT)). */
inline size_t HeapFree() {
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

/** What RunUserTask() saw. stackFreeMin, threw and what are set only for kDone. */
struct UserTaskResult {
  enum Status { kNotCreated, kTimeout, kDone };
  Status status = kNotCreated;
  uint32_t stackFreeMin = 0;  // bytes (ESP-IDF: StackType_t is uint8_t)
  bool threw = false;
  std::string what;
};

namespace detail {

struct UserTaskRun {
  void (*fn)(void*);
  void* arg;
  TaskHandle_t waiter;
  UserTaskResult result;
};

inline void UserTaskEntry(void* p) {
  auto* run = static_cast<UserTaskRun*>(p);
  try {
    run->fn(run->arg);
  } catch (const std::exception& e) {
    run->result.threw = true;
    run->result.what = e.what();
  } catch (...) {
    run->result.threw = true;
    run->result.what = "unknown exception";
  }
  run->result.stackFreeMin = uxTaskGetStackHighWaterMark(nullptr);
  xTaskNotifyGive(run->waiter);
  vTaskDelete(nullptr);
}

}  // namespace detail

/**
 * Runs fn(arg) on a new task with probot-core's user-task parameters (priority 1, core 1,
 * kUserStack bytes of stack, name "user") and waits up to timeoutMs for it to return. After a
 * timeout the task may still be running: its bookkeeping is leaked and arg must stay valid.
 */
inline UserTaskResult RunUserTask(void (*fn)(void*), void* arg, uint32_t timeoutMs) {
  auto* run = new detail::UserTaskRun{fn, arg, xTaskGetCurrentTaskHandle(), {}};
  if (xTaskCreatePinnedToCore(detail::UserTaskEntry, "user", kUserStack, run, 1, nullptr, 1) !=
      pdPASS) {
    delete run;
    return {};
  }
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeoutMs)) == 0) {
    UserTaskResult timedOut;
    timedOut.status = UserTaskResult::kTimeout;
    return timedOut;  // leaks run; the task may still use it
  }
  vTaskDelay(pdMS_TO_TICKS(20));  // let the idle task free the user task's stack
  UserTaskResult result = std::move(run->result);
  result.status = UserTaskResult::kDone;
  delete run;
  return result;
}

/**
 * Prints "TASK <name> prio=<n> core=<n> stack_free_min=<bytes>" for every FreeRTOS task (core -1:
 * no affinity) and returns their states.
 */
inline std::vector<TaskStatus_t> ReportTasks() {
  UBaseType_t n = uxTaskGetNumberOfTasks();
  std::vector<TaskStatus_t> tasks(n + 4);
  n = uxTaskGetSystemState(tasks.data(), tasks.size(), nullptr);
  tasks.resize(n);
  for (const TaskStatus_t& t : tasks) {
    int core = t.xCoreID == tskNO_AFFINITY ? -1 : static_cast<int>(t.xCoreID);
    Linef("TASK {} prio={} core={} stack_free_min={}", t.pcTaskName,
          static_cast<unsigned>(t.uxCurrentPriority), core,
          static_cast<unsigned>(t.usStackHighWaterMark));
  }
  return tasks;
}

/** Starts the runner task (16 KB, priority 2, core 0); it only starts and waits for user tasks. */
inline void StartRunner(void (*fn)(void*)) {
  xTaskCreatePinnedToCore(fn, "runner", 16384, nullptr, 2, nullptr, 0);
}

}  // namespace qemu
