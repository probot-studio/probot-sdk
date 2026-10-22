// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// QemuSmoke harness: shared declarations of the on-target smoke test (test/target/README.md).
// The sketch runs on the real Xtensa CPU, FreeRTOS and newlib of an ESP32-S3 (Espressif QEMU),
// without probot-core: FakeCore.cpp installs a fake CoreOps table, like the host tests.

#pragma once

#include <stdint.h>

#include <cstdio>
#include <format>
#include <string>
#include <string_view>
#include <utility>

namespace qemu {

/** The stack every case task gets: prosdk's PROBOT_USER_STACK (prosdk.h). */
inline constexpr uint32_t kUserStack = 24576;

/** Writes one line to UART0 (stdout) and flushes it. */
void Line(std::string_view text);

template <class... Args>
void Linef(std::format_string<Args...> fmt, Args&&... args) {
  Line(std::format(fmt, std::forward<Args>(args)...));
}

/** Result collector of one case. */
struct Ctx {
  int checks = 0;
  int failures = 0;
  std::string notes;  // " key=value" pairs for the CASE line

  void Check(bool ok, const char* expr, const char* file, int line, std::string_view extra = {});
  template <class... Args>
  void Note(std::format_string<Args...> fmt, Args&&... args) {
    notes += ' ';
    notes += std::format(fmt, std::forward<Args>(args)...);
  }
};

#define QCHECK(ctx, cond) (ctx).Check(static_cast<bool>(cond), #cond, __FILE__, __LINE__)
#define QCHECK_MSG(ctx, cond, ...) \
  (ctx).Check(static_cast<bool>(cond), #cond, __FILE__, __LINE__, std::format(__VA_ARGS__))

/** Boot number of this flash image (1 on a fresh image), from NVS (CasePreferences.cpp). */
int BootNumber();

// The cases (one file each). Every case runs on its own FreeRTOS task with kUserStack bytes of
// stack, priority 1, core 1 (probot-core's user task), except where noted.
void CaseMath(Ctx& ctx);         // (a) Eigen / Sleipnir / LQR / trajectory
void CaseExceptions(Ctx& ctx);   // (b) exceptions across the archive, std::format / print
void CaseThreads(Ctx& ctx);      // (c) ScopedThreadConfig threads, Notifier timing
void CasePreferences(Ctx& ctx);  // (e) Preferences on NVS, persisted across a restart
void CaseDataLog(Ctx& ctx);      // (f) DataLogManager on LittleFS
void CaseCommands(Ctx& ctx);     // (g) CommandScheduler
void CaseRobot(Ctx& ctx);        // (d) hosted TimedRobot through core's hooks (last: e-stop)

}  // namespace qemu
