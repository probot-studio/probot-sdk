// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DataLogManager offers its log folder on the Driver Station (wpilibc/0015, CoreOps ABI 8
// filesShare/filesUnshare, probot-core R11). Own executable: it installs a fake core that records
// share()/unshare() and the thread that called them (CoreOps is installed once per process). When
// the probot-core clone is present at configure time (PROSDK_TEST_PROBOT_CORE_FILES), the fake
// passes each call on to core's own probot::files::share(name, vfsDir) / unshare(name), so core's
// rules decide (4 shares, 95-byte absolute paths, no "." or ".." segments); otherwise a reduced
// rule (an absolute path) stands in. Built a second time as prosdk_files_legacy_tests with
// PROSDK_FAKE_CORE_NO_FILES_SHARE: both fields null, as CoreBridge.hpp leaves them for a core
// without R11; nothing is shared and nothing is printed.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/system/DataLogManager.hpp"

#if PROSDK_TEST_PROBOT_CORE_FILES
#include <probot/files/files.hpp>
#endif

namespace pf = prosdk::platform;

namespace {

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

std::mutex gMutex;
// "share <name> <dir>" / "unshare <name>", in call order, and the thread of each call
std::vector<std::string> gCalls;
std::vector<std::thread::id> gShareThreads;
std::vector<std::thread::id> gUnshareThreads;
std::string gConsole;
// how long the fake share() blocks, as core's does while a transfer of a replaced share stops
std::atomic<int> gShareDelayMs{0};

#if !PROSDK_FAKE_CORE_NO_FILES_SHARE
bool FakeShare(const char* name, const char* dir) {
  {
    std::scoped_lock lock{gMutex};
    gCalls.push_back(std::string{"share "} + name + " " + dir);
    gShareThreads.push_back(std::this_thread::get_id());
  }
  std::this_thread::sleep_for(milliseconds(gShareDelayMs.load()));
#if PROSDK_TEST_PROBOT_CORE_FILES
  return probot::files::share(name, dir);
#else
  return dir[0] == '/';
#endif
}
bool FakeUnshare(const char* name) {
  std::scoped_lock lock{gMutex};
  gCalls.push_back(std::string{"unshare "} + name);
  gUnshareThreads.push_back(std::this_thread::get_id());
#if PROSDK_TEST_PROBOT_CORE_FILES
  return probot::files::unshare(name);
#else
  return true;
#endif
}
#endif
void FakeConsoleLevel(uint8_t, const char* text, size_t len) {
  std::scoped_lock lock{gMutex};
  gConsole.append(text, len);
}

void InstallFakeCore() {
  static const bool installed = [] {
    pf::CoreOps ops{};
    ops.abiVersion = pf::kCoreOpsAbiVersion;
    ops.size = sizeof(pf::CoreOps);
    ops.consoleWriteLevel = FakeConsoleLevel;
#if !PROSDK_FAKE_CORE_NO_FILES_SHARE
    ops.filesShare = FakeShare;
    ops.filesUnshare = FakeUnshare;
#endif
    return pf::InstallCoreOps(&ops);
  }();
  REQUIRE(installed);
}

std::vector<std::string> TakeCalls() {
  std::scoped_lock lock{gMutex};
  return std::exchange(gCalls, {});
}

// Each case starts with an empty console capture, no recorded calls and (with core) no shares
// (cases run in random order).
void ResetCase() {
  std::scoped_lock lock{gMutex};
  gConsole.clear();
  gCalls.clear();
  gShareThreads.clear();
  gUnshareThreads.clear();
  gShareDelayMs = 0;
#if PROSDK_TEST_PROBOT_CORE_FILES
  probot::files::detail::resetForTest();
#endif
}

std::string Console() {
  std::scoped_lock lock{gMutex};
  return gConsole;
}

// Whether every share() / unshare() so far ran on another thread than `self` (share) and on
// `self` (unshare: the caller of Stop()).
bool SharedOffThread(std::thread::id self) {
  std::scoped_lock lock{gMutex};
  for (auto id : gShareThreads) {
    if (id == self) {
      return false;
    }
  }
  return !gShareThreads.empty();
}
bool UnsharedOn(std::thread::id self) {
  std::scoped_lock lock{gMutex};
  for (auto id : gUnshareThreads) {
    if (id != self) {
      return false;
    }
  }
  return !gUnshareThreads.empty();
}

// Waits until the fake core saw at least `n` calls (the manager's thread shares), at most `limit`.
bool WaitForCalls(size_t n, milliseconds limit) {
  auto end = Clock::now() + limit;
  while (Clock::now() < end) {
    {
      std::scoped_lock lock{gMutex};
      if (gCalls.size() >= n) {
        return true;
      }
    }
    std::this_thread::sleep_for(milliseconds(10));
  }
  return false;
}
bool WaitForCall(milliseconds limit) {
  return WaitForCalls(1, limit);
}

size_t Count(std::string_view text, std::string_view what) {
  size_t n = 0;
  for (size_t at = text.find(what); at != std::string_view::npos; at = text.find(what, at + 1)) {
    ++n;
  }
  return n;
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
}

std::filesystem::path TempDir(std::string_view tag) {
  return std::filesystem::temp_directory_path() /
         ("prosdk-dlm-share-" + std::to_string(::getpid()) + "-" + std::string{tag});
}

constexpr std::string_view kRefused = "Driver Station cannot offer the logs";

}  // namespace

#if !PROSDK_FAKE_CORE_NO_FILES_SHARE

TEST_CASE("DataLogManager shares its log folder with the Driver Station",
          "[prosdk][datalog][files]") {
  InstallFakeCore();
  ResetCase();
  const auto self = std::this_thread::get_id();
  REQUIRE(pf::FileSharingSupported());
  auto a = TempDir("a");
  auto b = TempDir("b");
  std::filesystem::remove_all(a);
  std::filesystem::remove_all(b);
  std::filesystem::create_directories(a);

  // the folder exists: shared as "logs" by the manager's thread, not by the caller of Start()
  prosdk::DataLogManager::Start(a.string(), "share.wpilog", 0.05);
  REQUIRE(WaitForCall(milliseconds(2000)));
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + a.string()});
  CHECK(SharedOffThread(self));
  // Stop() takes it back before it returns, on its own thread
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"unshare logs"});
  CHECK(UnsharedOn(self));

  // another folder after a restart, created only after Start(): shared once it exists, under the
  // same name
  prosdk::DataLogManager::Start(b.string(), "share.wpilog", 0.05);
  CHECK_FALSE(WaitForCall(milliseconds(400)));
  std::filesystem::create_directories(b);
  REQUIRE(WaitForCall(milliseconds(3000)));
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + b.string()});
  // shared once: no second share while running
  std::this_thread::sleep_for(milliseconds(1200));
  CHECK(TakeCalls().empty());
#if PROSDK_TEST_PROBOT_CORE_FILES
  probot::files::detail::Share sh{};
  REQUIRE(probot::files::detail::findShare("logs", sh));
  CHECK(std::string{sh.dir} == b.string());
#endif
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"unshare logs"});
  CHECK(SharedOffThread(self));
  CHECK(UnsharedOn(self));
  CHECK(Console().find(kRefused) == std::string::npos);
#if PROSDK_TEST_PROBOT_CORE_FILES
  CHECK(probot::files::detail::count() == 0);
#endif

  std::filesystem::remove_all(a);
  std::filesystem::remove_all(b);
}

TEST_CASE("DataLogManager keeps logging while share() blocks", "[prosdk][datalog][files]") {
  InstallFakeCore();
  ResetCase();
  auto a = TempDir("slow");
  std::filesystem::remove_all(a);
  std::filesystem::create_directories(a);
  // core's share() waits up to 1 s for a transfer of a replaced "logs" share to stop
  gShareDelayMs = 1000;

  auto t0 = Clock::now();
  prosdk::DataLogManager::Start(a.string(), "slow.wpilog", 0.05);
  CHECK(Clock::now() - t0 < milliseconds(500));
  REQUIRE(WaitForCall(milliseconds(2000)));  // share() entered, blocked on the manager's thread
  auto t1 = Clock::now();
  prosdk::DataLogManager::Log("logged during share");
  CHECK(Clock::now() - t1 < milliseconds(200));
  // the DataLog writer is its own thread: the message reaches the file before share() returns
  std::string data;
  auto end = Clock::now() + milliseconds(700);
  while (Clock::now() < end && data.find("logged during share") == std::string::npos) {
    std::this_thread::sleep_for(milliseconds(20));
    data = ReadFile(a / "slow.wpilog");
  }
  CHECK(data.find("logged during share") != std::string::npos);
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + a.string(), "unshare logs"});
  gShareDelayMs = 0;

  std::filesystem::remove_all(a);
}

TEST_CASE("DataLogManager reports a folder core refuses once", "[prosdk][datalog][files]") {
  InstallFakeCore();
  ResetCase();
  // a relative folder (refused by core, and by the reduced fake rule)
  std::string rel = "prosdk-dlm-share-rel-" + std::to_string(::getpid());
  std::filesystem::remove_all(rel);
  std::filesystem::create_directories(rel);

  prosdk::DataLogManager::Start(rel, "share.wpilog", 0.05);
  REQUIRE(WaitForCall(milliseconds(2000)));
  // tried again once a second, reported once
  REQUIRE(WaitForCalls(2, milliseconds(2500)));
  prosdk::DataLogManager::Stop();
  auto calls = TakeCalls();
  for (const auto& call : calls) {
    CHECK(call == "share logs " + rel);  // nothing to take back
  }

  std::string console = Console();
  INFO(console);
  CHECK(Count(console, kRefused) == 1);
  CHECK(console.find(rel) != std::string::npos);

  std::filesystem::remove_all(rel);
}

#if PROSDK_TEST_PROBOT_CORE_FILES

TEST_CASE("DataLogManager gets a share slot once the sketch frees one",
          "[prosdk][datalog][files][core]") {
  InstallFakeCore();
  ResetCase();
  auto a = TempDir("slot");
  std::filesystem::remove_all(a);
  std::filesystem::create_directories(a);
  // the sketch offers 4 folders of its own first
  for (const char* name : {"s0", "s1", "s2", "s3"}) {
    REQUIRE(probot::files::share(name, "/tmp"));
  }

  prosdk::DataLogManager::Start(a.string(), "slot.wpilog", 0.05);
  REQUIRE(WaitForCalls(2, milliseconds(3000)));  // refused, tried again
  probot::files::detail::Share sh{};
  CHECK_FALSE(probot::files::detail::findShare("logs", sh));
  // a slot frees up: the next try takes it
  REQUIRE(probot::files::unshare("s3"));
  auto end = Clock::now() + milliseconds(3000);
  while (Clock::now() < end && !probot::files::detail::findShare("logs", sh)) {
    std::this_thread::sleep_for(milliseconds(20));
  }
  REQUIRE(probot::files::detail::findShare("logs", sh));
  CHECK(std::string{sh.dir} == a.string());
  TakeCalls();
  std::this_thread::sleep_for(milliseconds(1200));
  CHECK(TakeCalls().empty());  // shared: no more tries

  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"unshare logs"});
  CHECK_FALSE(probot::files::detail::findShare("logs", sh));
  CHECK(probot::files::detail::count() == 3);  // the sketch's own shares stay
  std::string console = Console();
  INFO(console);
  CHECK(Count(console, kRefused) == 1);

  std::filesystem::remove_all(a);
}

TEST_CASE("DataLogManager follows core's path rules", "[prosdk][datalog][files][core]") {
  InstallFakeCore();
  ResetCase();
  auto base = TempDir("rules");
  std::filesystem::remove_all(base);
  // an absolute folder longer than 95 bytes, and one reached through ".."
  std::string longDir = (base / std::string(100, 'x')).string();
  REQUIRE(longDir.size() > probot::files::kMaxDir);
  std::filesystem::create_directories(longDir);
  std::filesystem::create_directories(base / "sub");
  std::string dotDot = (base / "sub" / "..").string();

  for (const std::string& dir : {longDir, dotDot}) {
    ResetCase();
    prosdk::DataLogManager::Start(dir, "rules.wpilog", 0.05);
    REQUIRE(WaitForCall(milliseconds(2000)));
    prosdk::DataLogManager::Stop();
    probot::files::detail::Share sh{};
    CHECK_FALSE(probot::files::detail::findShare("logs", sh));
    auto calls = TakeCalls();
    REQUIRE_FALSE(calls.empty());
    for (const auto& call : calls) {
      CHECK(call == "share logs " + dir);  // never shared, so never unshared
    }
    std::string console = Console();
    INFO(console);
    CHECK(Count(console, kRefused) == 1);
  }

  // the same folder without "..": accepted
  ResetCase();
  prosdk::DataLogManager::Start(base.string(), "rules.wpilog", 0.05);
  REQUIRE(WaitForCall(milliseconds(2000)));
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + base.string(), "unshare logs"});
  CHECK(Console().find(kRefused) == std::string::npos);

  std::filesystem::remove_all(base);
}

#endif  // PROSDK_TEST_PROBOT_CORE_FILES

#else

TEST_CASE("DataLogManager on a core without file sharing", "[prosdk][datalog][files]") {
  InstallFakeCore();
  ResetCase();
  CHECK(pf::HasCoreOps());
  CHECK_FALSE(pf::FileSharingSupported());
  CHECK_FALSE(pf::ShareFiles("logs", "/littlefs/logs"));
  CHECK_FALSE(pf::UnshareFiles("logs"));
  auto a = TempDir("legacy");
  std::filesystem::remove_all(a);
  std::filesystem::create_directories(a);

  // logs as before, without a share and without a warning
  prosdk::DataLogManager::Start(a.string(), "legacy.wpilog", 0.05);
  prosdk::DataLogManager::Log("hello");
  std::this_thread::sleep_for(milliseconds(1300));
  prosdk::DataLogManager::Stop();
  CHECK(std::filesystem::exists(a / "legacy.wpilog"));
  std::string console = Console();
  INFO(console);
  CHECK(console.find(kRefused) == std::string::npos);
  CHECK(console.find("hello") != std::string::npos);

  std::filesystem::remove_all(a);
}

#endif
