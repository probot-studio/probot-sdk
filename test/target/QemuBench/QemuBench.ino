// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// QemuBench: prosdk on-target speed benchmark for Espressif's QEMU (machine esp32s3).
// Run it with tools/qemu/run.sh --bench; see design/qemu-bench.md and test/target/README.md.
//
// Same setup as QemuSmoke (no probot-core; FakeCore.hpp from test/target/shared installs a fake
// CoreOps table; the archive, Xtensa CPU, FreeRTOS, newlib and ESP-IDF are real). QEMU runs with
// -icount shift=2: guest time advances 4 ns per executed guest instruction, so the numbers are
// deterministic instruction counts, not ESP32-S3 wall time (design/qemu-bench.md, caveats).
//
// UART0 protocol (parsed by tools/qemu/bench.py):
//   QEMU-BENCH BEGIN boot=<n>
//   CLOCK ccount_mhz_busy=<..> ccount_mhz_idle=<..> source=ccount|esp_timer
//   BENCH <name> iters=<n> total_us=<..> per_op_us=<..> instr_per_op=<..> [key=value ...]
//   MEM <where> heap_free=<bytes> heap_min_free=<bytes> heap_largest_block=<bytes>
//   STACK <group> used=<bytes>/<bytes>
//   TASK <name> prio=<n> core=<n> stack_free_min=<bytes>
//   QEMU-BENCH END boot=<n> groups=<n> failed=<n>

#include <esp_cpu.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <string>
#include <vector>

#include "Bench.hpp"
#include "FakeCore.hpp"

namespace qemu::bench {
namespace {

uint32_t gCcountMhz = 0;  // 0: esp_timer

// esp_timer is good for at most this long between two reads of the 32-bit CCOUNT (wraps after
// 2^32 / 240 MHz = 17.9 s); longer intervals use esp_timer
constexpr int64_t kCcountMaxUs = 15'000'000;

}  // namespace

Stopwatch::Stopwatch() : m_cycles{esp_cpu_get_cycle_count()}, m_us{esp_timer_get_time()} {}

uint64_t Stopwatch::ElapsedNs() const {
  const uint32_t cycles = esp_cpu_get_cycle_count() - m_cycles;
  const int64_t us = esp_timer_get_time() - m_us;
  if (gCcountMhz != 0 && us < kCcountMaxUs) {
    return uint64_t{cycles} * 1000 / gCcountMhz;
  }
  return static_cast<uint64_t>(us) * 1000;
}

void CalibrateClock() {
  // busy: spin on esp_timer for 20 ms of guest time
  uint32_t c0 = esp_cpu_get_cycle_count();
  int64_t t0 = esp_timer_get_time();
  int64_t t1 = t0;
  while (t1 - t0 < 20000) {
    t1 = esp_timer_get_time();
  }
  uint32_t c1 = esp_cpu_get_cycle_count();
  const double busy = double(c1 - c0) / double(t1 - t0);
  // idle: both cores wait, QEMU (sleep=off) jumps to the next timer deadline
  c0 = esp_cpu_get_cycle_count();
  t0 = esp_timer_get_time();
  vTaskDelay(pdMS_TO_TICKS(100));
  c1 = esp_cpu_get_cycle_count();
  t1 = esp_timer_get_time();
  const double idle = double(c1 - c0) / double(t1 - t0);
  const double mhz = std::round(busy);
  const bool ok = mhz >= 1.0 && std::abs(busy - mhz) < 0.005 * mhz &&
                  std::abs(idle - mhz) < 0.005 * mhz;
  gCcountMhz = ok ? static_cast<uint32_t>(mhz) : 0;
  Linef("CLOCK ccount_mhz_busy={:.3f} ccount_mhz_idle={:.3f} source={}", busy, idle,
        ok ? "ccount" : "esp_timer");
}

size_t HeapFree() {
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

void Report(std::string_view name, uint64_t iters, uint64_t totalNs, std::string_view extra) {
  const double n = iters != 0 ? double(iters) : 1.0;
  Linef("BENCH {} iters={} total_us={:.1f} per_op_us={:.3f} instr_per_op={:.1f}{}{}", name, iters,
        double(totalNs) / 1000.0, double(totalNs) / 1000.0 / n, double(totalNs) / 4.0 / n,
        extra.empty() ? "" : " ", extra);
}

}  // namespace qemu::bench

namespace {

namespace qb = qemu::bench;

struct Group {
  const char* name;
  void (*fn)();
};

// robot first: its boot initializes the HAL, which the scheduler group needs
const Group kGroups[] = {
    {"robot", &qb::BenchRobot},
    {"scheduler", &qb::BenchScheduler},
    {"math", &qb::BenchMath},
    {"drivers", &qb::BenchDrivers},
};

struct GroupRun {
  const Group* group;
  TaskHandle_t waiter;
  uint32_t stackFreeMin;
  bool threw;
  std::string what;
};

void GroupTask(void* arg) {
  auto* run = static_cast<GroupRun*>(arg);
  try {
    run->group->fn();
  } catch (const std::exception& e) {
    run->threw = true;
    run->what = e.what();
  } catch (...) {
    run->threw = true;
    run->what = "unknown exception";
  }
  // ESP-IDF: StackType_t is uint8_t, so the high-water mark is in bytes
  run->stackFreeMin = uxTaskGetStackHighWaterMark(nullptr);
  xTaskNotifyGive(run->waiter);
  vTaskDelete(nullptr);
}

void Mem(const char* where) {
  qemu::Linef("MEM {} heap_free={} heap_min_free={} heap_largest_block={}", where,
              heap_caps_get_free_size(MALLOC_CAP_8BIT),
              heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
              heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

void ReportTasks() {
  UBaseType_t n = uxTaskGetNumberOfTasks();
  std::vector<TaskStatus_t> tasks(n + 4);
  n = uxTaskGetSystemState(tasks.data(), tasks.size(), nullptr);
  for (UBaseType_t i = 0; i < n; ++i) {
    const TaskStatus_t& t = tasks[i];
    int core = t.xCoreID == tskNO_AFFINITY ? -1 : static_cast<int>(t.xCoreID);
    qemu::Linef("TASK {} prio={} core={} stack_free_min={}", t.pcTaskName,
                static_cast<unsigned>(t.uxCurrentPriority), core,
                static_cast<unsigned>(t.usStackHighWaterMark));
  }
}

void RunnerTask(void*) {
  // fixed boot number: every bench boot starts from the same flash image (run.sh --bench)
  qemu::Linef("QEMU-BENCH BEGIN boot=1 fake_core={}", qemu::fakecore::Installed() ? 1 : 0);
  qb::CalibrateClock();
  Mem("start");
  int failed = 0;
  for (const Group& group : kGroups) {
    auto* run = new GroupRun{&group, xTaskGetCurrentTaskHandle(), 0, false, {}};
    TaskHandle_t task = nullptr;
    // probot-core's user task: priority 1, core 1, PROBOT_USER_STACK
    if (xTaskCreatePinnedToCore(GroupTask, "user", qb::kUserStack, run, 1, &task, 1) != pdPASS) {
      qemu::Linef("BENCH-FAIL {} could not create the group task", group.name);
      ++failed;
      delete run;
      continue;
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(300000)) == 0) {
      qemu::Linef("BENCH-FAIL {} timeout", group.name);
      ++failed;
      continue;  // leaks run; the task may still use it
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // let the idle task free the group task's stack
    if (run->threw) {
      qemu::Linef("BENCH-FAIL {} exception: {}", group.name, run->what);
      ++failed;
    }
    qemu::Linef("STACK {} used={}/{}", group.name, qb::kUserStack - run->stackFreeMin,
                qb::kUserStack);
    Mem((std::string{"after-"} + group.name).c_str());
    delete run;
  }
  ReportTasks();
  qemu::Linef("QEMU-BENCH END boot=1 groups={} failed={}", std::size(kGroups), failed);
  vTaskDelete(nullptr);
}

}  // namespace

void setup() {
  qemu::Line("QEMU-BENCH setup");
  // the runner only starts and waits for the group tasks (core 0 stays idle meanwhile)
  xTaskCreatePinnedToCore(RunnerTask, "runner", 16384, nullptr, 2, nullptr, 0);
}

void loop() {
  vTaskDelete(nullptr);
}
