// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Steady-state heap traffic of the CommandScheduler's watchdog epochs (wpilibc patch 0014,
// commandsv2 patch 0005): a loop that adds the same epochs as the previous one allocates nothing,
// a loop that adds new epochs never holds more heap blocks than upstream does, and
// Tracer::PrintEpochs() still prints only the epochs added since the last ClearEpochs().
// Upstream allocates a std::string and a map node (and a key string for names longer than 15
// characters) per scheduled command per Run().

#include <atomic>
#include <cstdlib>
#include <new>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/commands2/CommandScheduler.hpp"
#include "prosdk/commands2/Commands.hpp"
#include "prosdk/simulation/SimHooks.hpp"
#include "prosdk/system/Tracer.hpp"
#include "prosdk/util/SmallString.hpp"
#include "prosdk/util/raw_ostream.hpp"

// operator new / delete calls of this executable while counting is armed
namespace {
std::atomic<bool> g_counting{false};
std::atomic<int> g_allocations{0};
std::atomic<int> g_live{0};      // blocks allocated minus blocks freed
std::atomic<int> g_peakLive{0};  // highest g_live

class CountAllocations {
 public:
  CountAllocations() {
    g_allocations = 0;
    g_live = 0;
    g_peakLive = 0;
    g_counting = true;
  }
  ~CountAllocations() { g_counting = false; }
  int Count() const { return g_allocations; }
  // most heap blocks held at once beyond those held when counting started
  int PeakLive() const { return g_peakLive; }
};

void CountFree(void* p) {
  if (p != nullptr && g_counting) {
    --g_live;
  }
}
}  // namespace

void* operator new(std::size_t size) {
  if (g_counting) {
    ++g_allocations;
    const int live = ++g_live;
    if (live > g_peakLive) {
      g_peakLive = live;
    }
  }
  if (void* p = std::malloc(size ? size : 1)) {
    return p;
  }
  throw std::bad_alloc{};
}

void operator delete(void* p) noexcept {
  CountFree(p);
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  CountFree(p);
  std::free(p);
}

namespace cmd = prosdk::cmd;

TEST_CASE("Tracer re-adding last loop's epochs allocates nothing", "[commands2][tracer]") {
  // names longer than the 15-character small-string buffer: each new entry allocates its key
  constexpr std::string_view kA = "FirstLongEpochName.Execute()";
  constexpr std::string_view kB = "SecondLongEpochName.Execute()";
  prosdk::Tracer tracer;
  tracer.AddEpoch(kA);
  tracer.AddEpoch(kB);

  tracer.ClearEpochs();
  {
    CountAllocations count;
    tracer.AddEpoch(kA);
    tracer.AddEpoch(kB);
    CHECK(count.Count() == 0);
  }

  // an epoch not added for a whole loop is dropped: adding it again allocates a new entry
  tracer.ClearEpochs();
  tracer.AddEpoch(kA);
  tracer.ClearEpochs();
  {
    CountAllocations count;
    tracer.AddEpoch(kB);
    CHECK(count.Count() > 0);
  }
}

TEST_CASE("Tracer holds no more heap than upstream when a loop adds new epochs",
          "[commands2][tracer]") {
  // A loop of new epochs after a loop of others (a burst of Schedule() calls, a mode change):
  // upstream frees the old entries in ClearEpochs() before the new ones are allocated. Names
  // longer than the small-string buffer: two blocks per entry.
  constexpr int kEpochs = 20;
  std::vector<std::string> oldNames;
  std::vector<std::string> newNames;
  for (int i = 0; i < kEpochs; ++i) {
    oldNames.push_back("OldCommandNumber" + std::to_string(i) + ".Initialize()");
    newNames.push_back("NewCommandNumber" + std::to_string(i) + ".Execute()");
  }
  prosdk::Tracer tracer;
  for (const auto& name : oldNames) {
    tracer.AddEpoch(name);
  }
  {
    CountAllocations count;
    tracer.ClearEpochs();
    for (const auto& name : newNames) {
      tracer.AddEpoch(name);
    }
    CHECK(count.PeakLive() == 0);
  }
  // the next loop adds them again: nothing allocated
  tracer.ClearEpochs();
  {
    CountAllocations count;
    for (const auto& name : newNames) {
      tracer.AddEpoch(name);
    }
    CHECK(count.Count() == 0);
  }
}

TEST_CASE("Tracer prints only the epochs added since the last clear", "[commands2][tracer]") {
  prosdk::util::SmallString<128> buf;
  prosdk::util::raw_svector_ostream os(buf);

  prosdk::sim::PauseTiming();
  prosdk::Tracer tracer;
  tracer.AddEpoch("stale");
  tracer.AddEpoch("kept");
  tracer.ClearEpochs();
  tracer.AddEpoch("kept");
  tracer.AddEpoch("new");
  // PrintEpochs() prints at most once per second
  prosdk::sim::StepTiming(prosdk::units::second_t{1.5});
  tracer.PrintEpochs(os);
  prosdk::sim::ResumeTiming();

  CHECK(std::string{os.str()} == "\tkept: 0.000000s\n\tnew: 0.000000s\n");
}

TEST_CASE("CommandScheduler Run() with the same commands allocates nothing",
          "[commands2][scheduler]") {
  auto& scheduler = cmd::CommandScheduler::GetInstance();
  scheduler.CancelAll();
  scheduler.Enable();
  int executes = 0;
  std::vector<cmd::CommandPtr> commands;
  for (int i = 0; i < 10; ++i) {
    // "Command<i>.Execute()" is longer than the small-string buffer
    commands.push_back(cmd::Run([&executes] { ++executes; })
                           .IgnoringDisable(true)
                           .WithName("Command" + std::to_string(i)));
    scheduler.Schedule(commands.back());
  }
  scheduler.Run();  // first loop: the epoch entries and the name buffer are created
  scheduler.Run();  // the first loop that repeats the previous one's epochs

  executes = 0;
  {
    CountAllocations count;
    scheduler.Run();
    scheduler.Run();
    CHECK(count.Count() == 0);
  }
  CHECK(executes == 20);
  scheduler.CancelAll();
}
