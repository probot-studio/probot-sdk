// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Console tee (prosdk/platform/Console.hpp), DataLogManager's console log (wpilibc/0012) and the
// host branch of GetStorageSpace (prosdk/platform/Storage.hpp).

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/datalog/DataLog.hpp"
#include "prosdk/datalog/DataLogReader.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/Storage.hpp"
#include "prosdk/platform/WpilibcConfig.hpp"
#include "prosdk/system/DataLogManager.hpp"
#include "prosdk/util/MemoryBuffer.hpp"

namespace {

std::string gSink;
std::string gTee;
void Sink(std::string_view text) { gSink += text; }
void Tee(std::string_view text) { gTee += text; }
void NestedTee(std::string_view text) {
  gTee += text;
  prosdk::platform::ConsoleWrite("nested\n");  // reaches the sink, not the tee again
}

/** String records of one entry of a .wpilog file. */
std::vector<std::string> ReadStrings(const std::string& file, std::string_view name) {
  auto buffer = prosdk::util::MemoryBuffer::GetFile(file);
  REQUIRE(buffer);
  prosdk::log::DataLogReader reader{std::move(*buffer)};
  REQUIRE(reader.IsValid());
  int id = -1;
  std::vector<std::string> out;
  for (const auto& record : reader) {
    if (record.IsStart()) {
      prosdk::log::StartRecordData start;
      REQUIRE(record.GetStartData(&start));
      if (start.name == name) {
        CHECK(start.type == "string");
        id = start.entry;
      }
    } else if (!record.IsControl() && record.GetEntry() == id) {
      std::string_view value;
      REQUIRE(record.GetString(&value));
      out.emplace_back(value);
    }
  }
  return out;
}

}  // namespace

TEST_CASE("Console tee receives every write after the sink", "[prosdk][console]") {
  gSink.clear();
  gTee.clear();
  prosdk::platform::SetConsoleSink(Sink);
  prosdk::platform::SetConsoleTee(Tee);
  prosdk::platform::ConsoleWrite("one\n");
  prosdk::platform::ConsoleWrite("two\n");
  CHECK(gSink == "one\ntwo\n");
  CHECK(gTee == "one\ntwo\n");

  gSink.clear();
  gTee.clear();
  prosdk::platform::SetConsoleTee(NestedTee);
  prosdk::platform::ConsoleWrite("outer\n");
  CHECK(gSink == "outer\nnested\n");
  CHECK(gTee == "outer\n");

  gTee.clear();
  prosdk::platform::SetConsoleTee(nullptr);
  prosdk::platform::ConsoleWrite("after\n");
  CHECK(gTee.empty());
  prosdk::platform::SetConsoleSink(nullptr);
}

TEST_CASE("GetStorageSpace reports the host file system", "[prosdk][storage]") {
  auto space = prosdk::platform::GetStorageSpace(std::filesystem::temp_directory_path().string());
  REQUIRE(space);
  CHECK(space->capacity > 0);
  CHECK(space->available <= space->capacity);
  CHECK_FALSE(prosdk::platform::GetStorageSpace("/nonexistent/prosdk/storage/test"));
}

TEST_CASE("DataLogManager logs the console when asked", "[prosdk][datalog]") {
  auto dir = std::filesystem::temp_directory_path() /
             ("prosdk-dlm-console-" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  gSink.clear();
  prosdk::platform::SetConsoleSink(Sink);
  prosdk::DataLogManager::Start(dir.string(), "console.wpilog", 0.05);
  // let the writer thread open its file (a Stop() before that leaves no log)
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // host: off by default (as upstream, which logs the console only on a real robot)
  prosdk::platform::ConsoleWrite("before\n");
  prosdk::DataLogManager::LogConsoleOutput(true);
  prosdk::platform::ConsoleWrite("first line\nsecond ");
  prosdk::platform::ConsoleWrite("line\npartial");
  // more than the buffer holds: the rest is counted as dropped
  prosdk::platform::ConsoleWrite(std::string(PROSDK_DATALOGMANAGER_CONSOLE_BUFFER, 'x'));
  prosdk::DataLogManager::LogConsoleOutput(false);
  prosdk::platform::ConsoleWrite("after\n");
  // Stop() discards what the writer has not written yet: flush and let it write
  prosdk::DataLogManager::GetLog().Flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  prosdk::DataLogManager::Stop();
  prosdk::platform::SetConsoleSink(nullptr);

  // the sink got everything
  CHECK(gSink.find("before\n") != std::string::npos);
  CHECK(gSink.find("after\n") != std::string::npos);

  auto lines = ReadStrings((dir / "console.wpilog").string(), "console");
  REQUIRE_FALSE(lines.empty());
  std::string all;
  for (auto& line : lines) {
    all += line;
    all += '\n';
  }
  INFO(all);
  CHECK(all.find("before") == std::string::npos);
  CHECK(all.find("after") == std::string::npos);
  CHECK(all.find("first line\nsecond line\npartial") != std::string::npos);
  // the x's that did not fit (how many depends on what the thread drained before) are counted
  constexpr std::string_view kDropped = "[DataLogManager: ";
  size_t at = all.find(kDropped);
  REQUIRE(at != std::string::npos);
  size_t dropped = std::stoul(all.substr(at + kDropped.size()));
  CHECK(all.find(" bytes of console output dropped]", at) != std::string::npos);
  size_t kept = static_cast<size_t>(std::count(all.begin(), all.end(), 'x'));
  CHECK(dropped > 0);
  CHECK(kept + dropped == PROSDK_DATALOGMANAGER_CONSOLE_BUFFER);

  std::filesystem::remove_all(dir);
}
