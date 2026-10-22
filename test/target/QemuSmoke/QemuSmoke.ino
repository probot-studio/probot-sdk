// QemuSmoke: prosdk on-target smoke test for Espressif's QEMU (machine esp32s3).
// Run it with tools/qemu/run.sh; see test/target/README.md and design/PORT.md section 11.
//
// No probot-core: QEMU has no WiFi, so the sketch does not include <prosdk.h>/<probot.h>.
// FakeCore.cpp installs a fake CoreOps table (as the host tests do) and CaseRobot.cpp drives
// core's framework hooks. Everything else is the real precompiled archive on the real Xtensa CPU,
// FreeRTOS, newlib and ESP-IDF (esp_timer, NVS, LittleFS).
//
// UART0 protocol (parsed by tools/qemu/run.sh):
//   QEMU-SMOKE BEGIN boot=<n>
//   CASE <id> <name> PASS|FAIL checks=<n> failed=<n> ms=<n> stack_used=<bytes>/<bytes>
//        heap_free=<bytes> heap_delta=<bytes> [notes]
//   TASK <name> prio=<n> core=<n> stack_free_min=<bytes>
//   QEMU-SMOKE END boot=<n> pass=<n> fail=<n>

#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>
#include <vector>

#include "FakeCore.hpp"
#include "Harness.hpp"

// linker symbols bounding the static TLS template (.tdata + .tbss)
extern "C" char _thread_local_start[];
extern "C" char _thread_local_end[];

namespace {

struct CaseDef {
  const char* id;
  const char* name;
  void (*fn)(qemu::Ctx&);
};

// (d) runs last: it ends with a probot-core e-stop, which latches.
const CaseDef kCases[] = {
    {"a", "math-eigen-sleipnir-lqr-trajectory", &qemu::CaseMath},
    {"b", "exceptions-format-print", &qemu::CaseExceptions},
    {"c", "threads-notifier", &qemu::CaseThreads},
    {"e", "preferences-nvs", &qemu::CasePreferences},
    {"f", "datalog-littlefs", &qemu::CaseDataLog},
    {"g", "command-scheduler", &qemu::CaseCommands},
    {"d", "timed-robot-hosted-loop", &qemu::CaseRobot},
};

struct CaseRun {
  const CaseDef* def;
  qemu::Ctx ctx;
  TaskHandle_t waiter;
  uint32_t stackFreeMin;
  bool threw;
  std::string what;
};

void CaseTask(void* arg) {
  auto* run = static_cast<CaseRun*>(arg);
  try {
    run->def->fn(run->ctx);
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

size_t HeapFree() { return heap_caps_get_free_size(MALLOC_CAP_8BIT); }

void ReportTasks(int& pass, int& fail) {
  // (h) every task: stack high-water marks; prosdk threads must run below priority 4 and
  // no thread may have been created with the default pthread configuration
  UBaseType_t n = uxTaskGetNumberOfTasks();
  std::vector<TaskStatus_t> tasks(n + 4);
  n = uxTaskGetSystemState(tasks.data(), tasks.size(), nullptr);
  qemu::Ctx ctx;
  int prosdkTasks = 0;
  for (UBaseType_t i = 0; i < n; ++i) {
    const TaskStatus_t& t = tasks[i];
    int core = t.xCoreID == tskNO_AFFINITY ? -1 : static_cast<int>(t.xCoreID);
    qemu::Linef("TASK {} prio={} core={} stack_free_min={}", t.pcTaskName,
                static_cast<unsigned>(t.uxCurrentPriority), core,
                static_cast<unsigned>(t.usStackHighWaterMark));
    const char* name = t.pcTaskName;
    bool isProsdk = std::strcmp(name, "prosdk") == 0 || std::strcmp(name, "notifier") == 0 ||
                    std::strcmp(name, "datalog") == 0 || std::strcmp(name, "watchdog") == 0 ||
                    std::strcmp(name, "dsmode") == 0 || std::strcmp(name, "qemu-worker") == 0;
    if (isProsdk) {
      ++prosdkTasks;
      QCHECK_MSG(ctx, t.uxCurrentPriority < 4, "task {} priority {}", name,
                 static_cast<unsigned>(t.uxCurrentPriority));
      QCHECK_MSG(ctx, t.usStackHighWaterMark >= 512, "task {} stack_free_min {}", name,
                 static_cast<unsigned>(t.usStackHighWaterMark));
    }
    QCHECK_MSG(ctx, std::strcmp(name, "pthread") != 0,
               "a thread was created with the default pthread config");
  }
  ctx.Note("tasks={} prosdk_tasks={}", static_cast<unsigned>(n), prosdkTasks);
  // ESP-IDF copies the static TLS block (every thread_local of the image) into the top of EVERY
  // task stack, the 1 KB IPC tasks included: a large thread_local overflows them at boot
  const size_t tls = static_cast<size_t>(_thread_local_end - _thread_local_start);
  ctx.Note("static_tls={}", tls);
  QCHECK_MSG(ctx, tls <= 64, "static TLS block {} bytes (thread_local objects)", tls);
  ctx.Note("heap_free={} heap_min_free={} heap_largest_block={}", HeapFree(),
           heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
           heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  bool ok = ctx.failures == 0;
  (ok ? pass : fail)++;
  qemu::Linef("CASE h heap-stack-report {} checks={} failed={}{}", ok ? "PASS" : "FAIL",
              ctx.checks, ctx.failures, ctx.notes);
}

void RunnerTask(void*) {
  qemu::Line("QEMU-SMOKE runner started");
  // LittleFS on the huge_app "spiffs" data partition (formatted on the first boot)
  bool fsMounted = LittleFS.begin(true);
  int boot = qemu::BootNumber();
  qemu::Linef("QEMU-SMOKE BEGIN boot={} fake_core={} littlefs={} heap_free={}", boot,
              qemu::fakecore::Installed() ? 1 : 0, fsMounted ? 1 : 0, HeapFree());
  int pass = 0;
  int fail = 0;
  for (const CaseDef& def : kCases) {
    auto* run = new CaseRun{&def, {}, xTaskGetCurrentTaskHandle(), 0, false, {}};
    size_t heapBefore = HeapFree();
    int64_t t0 = esp_timer_get_time();
    TaskHandle_t task = nullptr;
    // probot-core's user task: priority 1, core 1, PROBOT_USER_STACK
    if (xTaskCreatePinnedToCore(CaseTask, "user", qemu::kUserStack, run, 1, &task, 1) != pdPASS) {
      qemu::Linef("CASE {} {} FAIL could not create the case task", def.id, def.name);
      ++fail;
      delete run;
      continue;
    }
    // generous: the slowest case (robot) takes a few seconds of guest time
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(120000)) == 0) {
      qemu::Linef("CASE {} {} FAIL timeout", def.id, def.name);
      ++fail;
      continue;  // leaks run; the task may still use it
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // let the idle task free the case task's stack
    int64_t ms = (esp_timer_get_time() - t0) / 1000;
    if (run->threw) {
      run->ctx.Check(false, "uncaught exception", __FILE__, __LINE__, run->what);
    }
    bool ok = run->ctx.failures == 0 && run->ctx.checks > 0;
    (ok ? pass : fail)++;
    long delta = static_cast<long>(HeapFree()) - static_cast<long>(heapBefore);
    qemu::Linef("CASE {} {} {} checks={} failed={} ms={} stack_used={}/{} heap_free={} "
                "heap_delta={}{}",
                def.id, def.name, ok ? "PASS" : "FAIL", run->ctx.checks, run->ctx.failures, ms,
                qemu::kUserStack - run->stackFreeMin, qemu::kUserStack, HeapFree(), delta,
                run->ctx.notes);
    delete run;
  }
  ReportTasks(pass, fail);
  qemu::Linef("QEMU-SMOKE END boot={} pass={} fail={}", boot, pass, fail);
  vTaskDelete(nullptr);
}

}  // namespace

namespace qemu {

void Line(std::string_view text) {
  std::fwrite(text.data(), 1, text.size(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

void Ctx::Check(bool ok, const char* expr, const char* file, int line, std::string_view extra) {
  ++checks;
  if (ok) {
    return;
  }
  ++failures;
  const char* base = std::strrchr(file, '/');
  base = base != nullptr ? base + 1 : file;
  Linef("  CHECK FAILED {}:{}: {}{}{}", base, line, expr, extra.empty() ? "" : " -- ", extra);
}

}  // namespace qemu

void setup() {
  qemu::Line("QEMU-SMOKE setup");
  // the runner only starts and waits for the case tasks
  xTaskCreatePinnedToCore(RunnerTask, "runner", 16384, nullptr, 2, nullptr, 0);
}

void loop() {
  vTaskDelete(nullptr);
}
