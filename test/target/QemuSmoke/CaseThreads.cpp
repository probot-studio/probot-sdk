// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (c) std::thread through prosdk::platform::ScopedThreadConfig (name, stack, priority, core on
// FreeRTOS), std::mutex / std::condition_variable, prosdk::Notifier periodic and one-shot timing
// (HAL notifier thread on esp_timer time), and prosdk time against esp_timer.

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <prosdk/platform/Thread.hpp>
#include <prosdk/system/Notifier.hpp>
#include <prosdk/system/RobotController.hpp>
#include <prosdk/system/Timer.hpp>
#include <prosdk/units/time.hpp>

#include "Harness.hpp"

namespace qemu {

void CaseThreads(Ctx& ctx) {
  // ---- ScopedThreadConfig ----
  struct Seen {
    char name[16] = {};
    unsigned prio = 99;
    int core = -2;
    unsigned stackFree = 0;
  } seen;
  std::mutex m;
  std::condition_variable cv;
  bool ready = false;
  int value = 0;
  {
    prosdk::platform::ThreadConfig config;
    config.name = "qemu-worker";
    config.stackSize = 6144;
    config.priority = 2;
    config.core = 1;
    prosdk::platform::ScopedThreadConfig scoped{config};
    std::thread worker([&] {
      std::strncpy(seen.name, pcTaskGetName(nullptr), sizeof(seen.name) - 1);
      seen.prio = uxTaskPriorityGet(nullptr);
      seen.core = xPortGetCoreID();
      std::unique_lock lock{m};
      cv.wait(lock, [&] { return ready; });
      value = 42;
      seen.stackFree = uxTaskGetStackHighWaterMark(nullptr);
    });
    {
      std::scoped_lock lock{m};
      ready = true;
    }
    cv.notify_one();
    worker.join();
  }
  QCHECK_MSG(ctx, std::strcmp(seen.name, "qemu-worker") == 0, "name '{}'", seen.name);
  QCHECK_MSG(ctx, seen.prio == 2, "prio {}", seen.prio);
  QCHECK_MSG(ctx, seen.core == 1, "core {}", seen.core);
  QCHECK_MSG(ctx, seen.stackFree > 0 && seen.stackFree < 6144, "stack free {}", seen.stackFree);
  QCHECK(ctx, value == 42);
  ctx.Note("worker_stack_used={}/6144", 6144 - seen.stackFree);

  // ---- prosdk time vs esp_timer ----
  int64_t esp0 = esp_timer_get_time();
  int64_t mono0 = prosdk::RobotController::GetMonotonicTime() / 1000;  // ns -> us
  double ts0 = prosdk::Timer::GetMonotonicTimestamp().value();
  vTaskDelay(pdMS_TO_TICKS(200));
  int64_t esp1 = esp_timer_get_time();
  int64_t mono1 = prosdk::RobotController::GetMonotonicTime() / 1000;
  double ts1 = prosdk::Timer::GetMonotonicTimestamp().value();
  int64_t dEsp = esp1 - esp0;
  int64_t dMono = mono1 - mono0;
  QCHECK_MSG(ctx, std::abs(dMono - dEsp) < 2000, "esp {} us, monotonic {} us", dEsp, dMono);
  QCHECK_MSG(ctx, std::abs((ts1 - ts0) * 1e6 - dEsp) < 2000, "Timer {} s", ts1 - ts0);

  // ---- Notifier: periodic 10 ms for 1 s ----
  std::vector<int64_t> stamps;
  stamps.reserve(160);
  std::atomic<unsigned> cbPrio{99};
  std::mutex sm;
  prosdk::Notifier periodic{[&] {
    int64_t now = esp_timer_get_time();
    cbPrio.store(uxTaskPriorityGet(nullptr));
    std::scoped_lock lock{sm};
    if (stamps.size() < 150) {
      stamps.push_back(now);
    }
  }};
  periodic.SetName("qemu-periodic");
  int64_t start = esp_timer_get_time();
  periodic.StartPeriodic(10_ms);
  vTaskDelay(pdMS_TO_TICKS(1000));
  periodic.Stop();
  int64_t elapsed = esp_timer_get_time() - start;
  std::vector<int64_t> s;
  {
    std::scoped_lock lock{sm};
    s = stamps;
  }
  size_t expected = static_cast<size_t>(elapsed / 10000);
  QCHECK_MSG(ctx, s.size() + 3 >= expected && s.size() <= expected + 1, "{} callbacks in {} us",
             s.size(), elapsed);
  int64_t minGap = INT64_MAX, maxGap = 0;
  for (size_t i = 1; i < s.size(); ++i) {
    minGap = std::min(minGap, s[i] - s[i - 1]);
    maxGap = std::max(maxGap, s[i] - s[i - 1]);
  }
  double mean = s.size() > 1 ? double(s.back() - s.front()) / double(s.size() - 1) : 0.0;
  QCHECK_MSG(ctx, mean > 9500 && mean < 10500, "mean period {} us", mean);
  QCHECK_MSG(ctx, cbPrio.load() < 4, "notifier callback priority {}", cbPrio.load());
  ctx.Note("notifier_n={} mean_us={:.0f} min_us={} max_us={} cb_prio={}", s.size(), mean,
           minGap, maxGap, cbPrio.load());

  // ---- Notifier: one-shot 50 ms ----
  std::atomic<int64_t> fired{0};
  prosdk::Notifier single{[&] { fired.store(esp_timer_get_time()); }};
  int64_t t0 = esp_timer_get_time();
  single.StartSingle(50_ms);
  vTaskDelay(pdMS_TO_TICKS(150));
  int64_t latency = fired.load() - t0;
  QCHECK_MSG(ctx, fired.load() != 0 && latency >= 49000 && latency < 60000, "one-shot {} us",
             latency);
  ctx.Note("oneshot_us={}", latency);
}

}  // namespace qemu
