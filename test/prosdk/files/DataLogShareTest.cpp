// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DataLogManager offers its log folder on the Driver Station (wpilibc/0015, CoreOps ABI 8
// filesShare/filesUnshare, probot-core R11). Own executable: it installs a fake core that records
// share()/unshare() (CoreOps is installed once per process). Built a second time as
// prosdk_files_legacy_tests with PROSDK_FAKE_CORE_NO_FILES_SHARE: both fields null, as
// CoreBridge.hpp leaves them for a core without R11; nothing is shared and nothing is printed.

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/system/DataLogManager.hpp"

namespace pf = prosdk::platform;

namespace {

std::mutex gMutex;
// "share <name> <dir>" / "unshare <name>", in call order
std::vector<std::string> gCalls;
std::string gConsole;

#if !PROSDK_FAKE_CORE_NO_FILES_SHARE
// core's rule, reduced to what the test needs: an absolute folder
bool FakeShare(const char* name, const char* dir) {
  std::scoped_lock lock{gMutex};
  gCalls.push_back(std::string{"share "} + name + " " + dir);
  return dir[0] == '/';
}
bool FakeUnshare(const char* name) {
  std::scoped_lock lock{gMutex};
  gCalls.push_back(std::string{"unshare "} + name);
  return true;
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

// Each case starts with an empty console capture (cases run in random order).
void ClearConsole() {
  std::scoped_lock lock{gMutex};
  gConsole.clear();
}

std::string Console() {
  std::scoped_lock lock{gMutex};
  return gConsole;
}

// Waits until the fake core saw a call (the manager's thread shares), at most `limit`.
bool WaitForCall(std::chrono::milliseconds limit) {
  auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    {
      std::scoped_lock lock{gMutex};
      if (!gCalls.empty()) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
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
  ClearConsole();
  REQUIRE(pf::FileSharingSupported());
  auto a = TempDir("a");
  auto b = TempDir("b");
  std::filesystem::remove_all(a);
  std::filesystem::remove_all(b);
  std::filesystem::create_directories(a);
  TakeCalls();

  // the folder exists: shared by the manager's thread as "logs"
  prosdk::DataLogManager::Start(a.string(), "share.wpilog", 0.05);
  REQUIRE(WaitForCall(std::chrono::seconds(2)));
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + a.string()});
  // Stop() takes it back before it returns
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"unshare logs"});

  // another folder after a restart, created only after Start(): shared once it exists, under the
  // same name
  prosdk::DataLogManager::Start(b.string(), "share.wpilog", 0.05);
  CHECK_FALSE(WaitForCall(std::chrono::milliseconds(400)));
  std::filesystem::create_directories(b);
  REQUIRE(WaitForCall(std::chrono::seconds(3)));
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + b.string()});
  // tried once: no second share while running
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  CHECK(TakeCalls().empty());
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls() == std::vector<std::string>{"unshare logs"});
  CHECK(Console().find(kRefused) == std::string::npos);

  std::filesystem::remove_all(a);
  std::filesystem::remove_all(b);
}

TEST_CASE("DataLogManager reports a folder core refuses once", "[prosdk][datalog][files]") {
  InstallFakeCore();
  ClearConsole();
  // a relative folder (the fake refuses it, as core does)
  std::string rel = "prosdk-dlm-share-rel-" + std::to_string(::getpid());
  std::filesystem::remove_all(rel);
  std::filesystem::create_directories(rel);
  TakeCalls();

  prosdk::DataLogManager::Start(rel, "share.wpilog", 0.05);
  REQUIRE(WaitForCall(std::chrono::seconds(2)));
  CHECK(TakeCalls() == std::vector<std::string>{"share logs " + rel});
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  CHECK(TakeCalls().empty());  // not retried
  prosdk::DataLogManager::Stop();
  CHECK(TakeCalls().empty());  // nothing to take back

  std::string console = Console();
  INFO(console);
  size_t at = console.find(kRefused);
  REQUIRE(at != std::string::npos);
  CHECK(console.find(kRefused, at + 1) == std::string::npos);
  CHECK(console.find(rel) != std::string::npos);

  std::filesystem::remove_all(rel);
}

#else

TEST_CASE("DataLogManager on a core without file sharing", "[prosdk][datalog][files]") {
  InstallFakeCore();
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
  std::this_thread::sleep_for(std::chrono::milliseconds(1300));
  prosdk::DataLogManager::Stop();
  CHECK(std::filesystem::exists(a / "legacy.wpilog"));
  std::string console = Console();
  INFO(console);
  CHECK(console.find(kRefused) == std::string::npos);
  CHECK(console.find("hello") != std::string::npos);

  std::filesystem::remove_all(a);
}

#endif
