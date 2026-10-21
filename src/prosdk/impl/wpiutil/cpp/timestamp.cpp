// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/util/timestamp.hpp"

#include <atomic>
#include <chrono>

#include "prosdk/util/timestamp.h"
#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW
#include <esp_timer.h>
#endif

static int64_t timestamp() noexcept {
#if PROSDK_TARGET_HW
  // esp_timer: monotonic microseconds since boot; 1-ns intervals like upstream
  return esp_timer_get_time() * 1000;
#else
  // 1-ns intervals
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
#endif
}

static const int64_t original_program_start_time = timestamp();
static std::atomic<int64_t> program_start_time{original_program_start_time};

int64_t prosdk::util::NowDefault() {
  return timestamp();
}

static std::atomic<int64_t (*)()> now_impl{prosdk::util::NowDefault};

void prosdk::util::SetNowImpl(int64_t (*func)(void)) {
  if (!func) {
    now_impl = prosdk::util::NowDefault;
    program_start_time = original_program_start_time;
  } else {
    now_impl = func;
    program_start_time = func();
  }
}

int64_t prosdk::util::Now() {
  return (now_impl.load())();
}

int64_t prosdk::util::GetProgramStartTime() {
  return program_start_time;
}

int64_t prosdk::util::GetSystemTime() {
  // 1-ns intervals
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

extern "C" {

int64_t WPI_NowDefault(void) {
  return prosdk::util::NowDefault();
}

void WPI_SetNowImpl(int64_t (*func)(void)) {
  prosdk::util::SetNowImpl(func);
}

int64_t WPI_Now(void) {
  return prosdk::util::Now();
}

int64_t WPI_GetProgramStartTime(void) {
  return prosdk::util::GetProgramStartTime();
}

int64_t WPI_GetSystemTime(void) {
  return prosdk::util::GetSystemTime();
}

}  // extern "C"
