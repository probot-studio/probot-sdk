// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// DataLogManager with a valid system clock (core request R10): once a Driver Station is attached
// for a second it renames the log to WPILIB_<UTC date>_<time>.wpilog, and every ~250 Driver Station
// updates it appends a systemTime entry. On the robot the clock is valid once probot-core's
// ds::systemTimeValid() is true (HAL_GetSystemTimeValid, CoreOps systemTimeValid); the host sim
// HAL always reports a valid clock, which stands in for a Driver Station that set it.

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/datalog/DataLog.hpp"
#include "prosdk/datalog/DataLogReader.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/HAL.h"
#include "prosdk/simulation/DriverStationSim.hpp"
#include "prosdk/system/DataLogManager.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/MemoryBuffer.hpp"

namespace fs = std::filesystem;

namespace {

int64_t NowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// "WPILIB_YYYYMMDD_HHMMSS.wpilog" -> seconds since the epoch (UTC), or -1.
int64_t ParseLogName(const std::string& name) {
  int y, mo, d, h, mi, s;
  char tail[16] = {};
  if (std::sscanf(name.c_str(), "WPILIB_%4d%2d%2d_%2d%2d%2d%15s", &y, &mo, &d, &h, &mi, &s,
                  tail) != 7 ||
      std::string_view{tail} != ".wpilog") {
    return -1;
  }
  std::tm tm{};
  tm.tm_year = y - 1900;
  tm.tm_mon = mo - 1;
  tm.tm_mday = d;
  tm.tm_hour = h;
  tm.tm_min = mi;
  tm.tm_sec = s;
  return static_cast<int64_t>(timegm(&tm));
}

/** Integer records of one entry of a .wpilog file. */
std::vector<int64_t> ReadIntegers(const std::string& file, std::string_view name) {
  auto buffer = prosdk::util::MemoryBuffer::GetFile(file);
  REQUIRE(buffer);
  prosdk::log::DataLogReader reader{std::move(*buffer)};
  REQUIRE(reader.IsValid());
  int id = -1;
  std::vector<int64_t> out;
  for (const auto& record : reader) {
    if (record.IsStart()) {
      prosdk::log::StartRecordData start;
      REQUIRE(record.GetStartData(&start));
      if (start.name == name) {
        CHECK(start.type == "int64");
        id = start.entry;
      }
    } else if (!record.IsControl() && record.GetEntry() == id) {
      int64_t value;
      REQUIRE(record.GetInteger(&value));
      out.push_back(value);
    }
  }
  return out;
}

}  // namespace

TEST_CASE("DataLogManager names the log and logs systemTime with a valid clock",
          "[prosdk][datalog]") {
  REQUIRE(HAL_Initialize());
  REQUIRE(prosdk::RobotController::IsSystemTimeValid());

  auto dir = fs::temp_directory_path() / ("prosdk-dlm-time-" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);

  const int64_t startUs = NowUs();
  prosdk::DataLogManager::Start(dir.string(), "", 0.05);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // A Driver Station attaches and sends updates: the manager's thread wakes on every refresh.
  // More than 50 updates rename the log, 250 write one systemTime entry.
  prosdk::sim::DriverStationSim::SetDsAttached(true);
  for (int i = 0; i < 400; ++i) {
    prosdk::sim::DriverStationSim::NotifyNewData();
    prosdk::internal::DriverStationBackend::RefreshData();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
  prosdk::DataLogManager::GetLog().Flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  prosdk::DataLogManager::Stop();
  const int64_t endUs = NowUs();
  prosdk::sim::DriverStationSim::ResetData();

  std::vector<std::string> names;
  for (const auto& entry : fs::directory_iterator{dir}) {
    names.push_back(entry.path().filename().string());
  }
  REQUIRE(names.size() == 1);
  INFO(names[0]);
  // a real name, not the WPILIB_TBD_ placeholder: the UTC time of the rename
  int64_t named = ParseLogName(names[0]);
  REQUIRE(named > 0);
  CHECK(named >= startUs / 1000000 - 1);
  CHECK(named <= endUs / 1000000 + 1);

  auto times = ReadIntegers((dir / names[0]).string(), "systemTime");
  REQUIRE_FALSE(times.empty());
  for (int64_t t : times) {
    CHECK(t >= startUs);
    CHECK(t <= endUs);
  }

  fs::remove_all(dir);
}
