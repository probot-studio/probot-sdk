// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// QemuBench harness: timing, optimizer barriers and the BENCH line of the on-target speed
// benchmark (design/qemu-bench.md, test/target/README.md). Guest time under QEMU's -icount
// shift=2 is instruction-count time: 4 ns per executed guest instruction, so instr = ns / 4.

#pragma once

#include <stdint.h>

#include <string>
#include <string_view>

#include "QemuUart.hpp"  // Line, Linef (test/target/shared)

namespace qemu::bench {

/** The stack every group task gets: prosdk's PROBOT_USER_STACK (prosdk.h), as QemuSmoke. */
inline constexpr uint32_t kUserStack = 24576;

/**
 * Measures the clock sources against each other and picks one (prints a CLOCK line): the
 * CPU cycle counter (CCOUNT) when it advances at a whole number of cycles per esp_timer
 * microsecond both while busy and across an idle wait, else esp_timer (1 us resolution).
 */
void CalibrateClock();

/** Guest time since construction, from the clock CalibrateClock() picked. Same core only. */
class Stopwatch {
 public:
  Stopwatch();
  uint64_t ElapsedNs() const;

 private:
  uint32_t m_cycles;
  int64_t m_us;
};

/** Keeps the value (and everything it was computed from) alive for the optimizer. */
template <class T>
inline void DoNotOptimize(T& value) {
  asm volatile("" : "+m"(value) : : "memory");
}
template <class T>
inline void DoNotOptimize(const T& value) {
  asm volatile("" : : "m"(value) : "memory");
}
/** Forces pending stores to memory: the compiler may not move work across it. */
inline void ClobberMemory() {
  asm volatile("" : : : "memory");
}

/**
 * Prints "BENCH <name> iters=<n> total_us=<..> per_op_us=<..> instr_per_op=<..>[ extra]".
 * instr_per_op = total ns / 4 / iters (guest instructions per operation under -icount shift=2).
 */
void Report(std::string_view name, uint64_t iters, uint64_t totalNs, std::string_view extra = {});

/** Runs fn once (first-call allocations), then iters times between two clock reads. */
template <class Fn>
void Run(std::string_view name, uint32_t iters, Fn&& fn, std::string_view extra = {}) {
  fn();
  Stopwatch watch;
  for (uint32_t i = 0; i < iters; ++i) {
    fn();
    ClobberMemory();
  }
  Report(name, iters, watch.ElapsedNs(), extra);
}

/** Free 8-bit heap in bytes (heap_caps_get_free_size(MALLOC_CAP_8BIT)). */
size_t HeapFree();

// The groups (one file each). Each runs on its own FreeRTOS task with kUserStack bytes of stack,
// priority 1, core 1 (probot-core's user task), like the QemuSmoke cases.
void BenchRobot();      // (a) hosted TimedRobot under load at 20/10/5/2 ms through core's hooks
void BenchScheduler();  // (b) CommandScheduler::Run() with 0..200 commands, subsystems
void BenchMath();       // (c) wpimath micro-benchmarks, Sleipnir
void BenchDrivers();    // (d) Boardoza driver decode paths (pure computation)

}  // namespace qemu::bench
