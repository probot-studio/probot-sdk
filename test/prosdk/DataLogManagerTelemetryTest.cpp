// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DataLogManager's telemetry copy (wpilibc/0016, prosdk/platform/TelemetryLog.hpp): with a default
// telemetry backend set, LogNetworkTables (on by default) also writes telemetry into the log as
// "NT:/Telemetry/<path>". Switching the copy keeps the default backend's entries.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/datalog/DataLog.hpp"
#include "prosdk/datalog/DataLogBackgroundWriter.hpp"
#include "prosdk/datalog/DataLogReader.hpp"
#include "prosdk/platform/TelemetryLog.hpp"
#include "prosdk/system/DataLogManager.hpp"
#include "prosdk/telemetry/MockTelemetryBackend.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/util/MemoryBuffer.hpp"

namespace {

// counts the entries the registry removes from the default backend
class CountingBackend : public prosdk::telemetry::MockTelemetryBackend {
 public:
  void RemoveEntry(std::string_view path) override {
    ++removed;
    MockTelemetryBackend::RemoveEntry(path);
  }
  std::atomic<int> removed{0};
};

struct LoggedEntry {
  std::string type;
  std::vector<double> doubles;
  std::vector<std::string> strings;
};

std::unordered_map<std::string, LoggedEntry> ReadLog(const std::string& file) {
  auto buffer = prosdk::util::MemoryBuffer::GetFile(file);
  REQUIRE(buffer);
  prosdk::log::DataLogReader reader{std::move(*buffer)};
  REQUIRE(reader.IsValid());
  std::unordered_map<int, std::string> names;
  std::unordered_map<std::string, LoggedEntry> out;
  for (const auto& record : reader) {
    if (record.IsStart()) {
      prosdk::log::StartRecordData start;
      REQUIRE(record.GetStartData(&start));
      names[start.entry] = start.name;
      out[std::string{start.name}].type = start.type;
    } else if (!record.IsControl()) {
      auto it = names.find(record.GetEntry());
      if (it == names.end()) {
        continue;
      }
      auto& entry = out[it->second];
      if (entry.type == "double") {
        double value;
        REQUIRE(record.GetDouble(&value));
        entry.doubles.push_back(value);
      } else if (entry.type == "string") {
        std::string_view value;
        REQUIRE(record.GetString(&value));
        entry.strings.emplace_back(value);
      }
    }
  }
  return out;
}

}  // namespace

TEST_CASE("DataLogManager copies telemetry into the log", "[prosdk][datalog][telemetry]") {
  namespace fs = std::filesystem;
  auto dir = fs::temp_directory_path() / ("prosdk-dlm-telemetry-" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);

  prosdk::telemetry::TelemetryRegistry::Reset();
  auto mock = std::make_shared<CountingBackend>();
  prosdk::platform::SetDefaultTelemetryBackend(mock);
  // the registered backend forwards to the default backend
  CHECK(prosdk::telemetry::TelemetryRegistry::GetBackend("/Drive/speed")->GetEntry("/Drive/speed") ==
        mock->GetEntry("/Drive/speed"));
  // logged once before the copy starts (an alert, a static display, a value set in init)
  prosdk::telemetry::Log("Drive/once", 7.0);
  auto once = mock->GetEntry("/Drive/once");
  REQUIRE_FALSE(once->IsDiscard());

  prosdk::DataLogManager::Start(dir.string(), "telemetry.wpilog", 0.05);
  // the default backend is now the pair (dashboard + log)
  auto pair = prosdk::telemetry::TelemetryRegistry::GetBackend("/Drive/speed");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  prosdk::telemetry::Log("Drive/speed", 1.5);
  prosdk::telemetry::Log("Drive/speed", 1.5);  // equal: written once
  prosdk::telemetry::Log("Drive/speed", 2.5);
  prosdk::telemetry::Log("Drive/state", std::string_view{"auto"});
  CHECK(mock->GetLastValue<double>("/Drive/speed") == 2.5);

  // off: only the default backend gets values
  prosdk::DataLogManager::LogNetworkTables(false);
  CHECK(prosdk::telemetry::TelemetryRegistry::GetBackend("/Drive/speed") != pair);
  prosdk::telemetry::Log("Drive/speed", 3.5);
  CHECK(mock->GetLastValue<double>("/Drive/speed") == 3.5);

  // on again: a new log entry for the same path
  prosdk::DataLogManager::LogNetworkTables(true);
  prosdk::telemetry::Log("Drive/speed", 4.5);
  CHECK(mock->GetLastValue<double>("/Drive/speed") == 4.5);

  prosdk::DataLogManager::GetLog().Flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  prosdk::DataLogManager::Stop();
  // the swaps kept the default backend's entries: nothing was removed or unpublished
  CHECK(mock->removed == 0);
  CHECK(mock->GetEntry("/Drive/once") == once);
  CHECK_FALSE(once->IsDiscard());
  CHECK(mock->GetLastValue<double>("/Drive/once") == 7.0);

  auto entries = ReadLog((dir / "telemetry.wpilog").string());
  REQUIRE(entries.contains("NT:/Telemetry/Drive/speed"));
  const auto& speed = entries["NT:/Telemetry/Drive/speed"];
  CHECK(speed.type == "double");
  CHECK(speed.doubles == std::vector<double>{1.5, 2.5, 4.5});
  REQUIRE(entries.contains("NT:/Telemetry/Drive/state"));
  CHECK(entries["NT:/Telemetry/Drive/state"].strings == std::vector<std::string>{"auto"});
  // logged before the copy started and not since
  CHECK_FALSE(entries.contains("NT:/Telemetry/Drive/once"));

  // clearing the default backend: neither it nor a log copy gets values
  prosdk::log::DataLogBackgroundWriter log{[](std::span<const uint8_t>) {}, 0.25};
  prosdk::platform::StartTelemetryLog(log);
  prosdk::telemetry::Log("Drive/late", 1.0);
  auto late = mock->GetEntry("/Drive/late");
  prosdk::platform::SetDefaultTelemetryBackend(nullptr);
  CHECK(mock->removed == 1);
  CHECK(late->IsDiscard());
  CHECK(prosdk::telemetry::TelemetryRegistry::GetBackend("/Drive/speed")
            ->GetEntry("/Drive/speed")
            ->IsDiscard());
  prosdk::telemetry::Log("Drive/speed", 5.5);
  CHECK(mock->GetLastValue<double>("/Drive/speed") == 4.5);
  prosdk::platform::StopTelemetryLog(log);
  prosdk::telemetry::TelemetryRegistry::Reset();
  fs::remove_all(dir);
}
