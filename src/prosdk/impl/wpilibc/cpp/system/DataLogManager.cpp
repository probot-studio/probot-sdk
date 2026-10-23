// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/DataLogManager.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <format>
#include <mutex>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "prosdk/datalog/DataLog.hpp"
#include "prosdk/datalog/DataLogBackgroundWriter.hpp"
#include "prosdk/driverstation/MatchState.hpp"
#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/framework/RobotBase.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/Random.hpp"
#include "prosdk/platform/Storage.hpp"
#include "prosdk/platform/TelemetryLog.hpp"
#include "prosdk/platform/WpilibcConfig.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/Filesystem.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/SafeThread.hpp"
#include "prosdk/util/StringExtras.hpp"
#include "prosdk/util/UsageReporting.hpp"
#include "prosdk/util/fs.hpp"
#include "prosdk/util/print.hpp"
#include "prosdk/util/timestamp.hpp"

using namespace prosdk;

namespace {

// prosdk: the log file name's time stamp without std::format's chrono formatter, which links
// about 47 KB of flash into every sketch that logs (design/size.md). Same text as upstream's
// "{:%Y%m%d_%H%M%OS}" of a system_clock time point: UTC, %OS two-digit seconds.
std::string FileTimestamp(std::chrono::system_clock::time_point now) {
  std::time_t t = std::chrono::system_clock::to_time_t(
      std::chrono::floor<std::chrono::seconds>(now));
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900,
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

// prosdk: name of the log folder's share on the Driver Station ("Dosyalar" card, probot-core R11)
constexpr const char* kLogShareName = "logs";

struct Thread final : public prosdk::util::SafeThread {
  Thread(std::string_view dir, std::string_view filename, double period);
  ~Thread() override;

  void Main() final;

  void StartNTLog();
  void StopNTLog();
  // prosdk: upstream starts the console logger only on a real robot, also when
  // LogConsoleOutput(true) asks; here an explicit request (force) works on the host too
  void StartConsoleLog(bool force = false);
  void StopConsoleLog();
  // prosdk: moves the complete console lines captured so far into the "console" entry
  void DrainConsoleLog(bool all);
  // prosdk: offers m_logDir on the Driver Station once it exists (this thread only)
  void ShareLogDir();

  std::string m_logDir;
  bool m_filenameOverride;
  prosdk::log::DataLogBackgroundWriter m_log;
  // prosdk: no NetworkTables entry/connection loggers (ntcore is not ported; StartNTLog copies
  // telemetry into the log instead). Upstream's
  // console logger tails /home/systemcore/WPILIB_UserProgram.log; here the console tee
  // (prosdk::platform::SetConsoleTee) copies every prosdk console write into a bounded buffer
  // that this thread drains into the "console" entry (created on the first
  // StartConsoleLog; guarded by m_consoleLogMutex, taken before the buffer's).
  std::mutex m_consoleLogMutex;
  prosdk::log::StringLogEntry m_consoleLog;
  prosdk::log::StringLogEntry m_messageLog;
  // prosdk: the log folder's Driver Station share. Written by this thread, read by ~Thread after
  // Main() returned. kWaiting: the folder is missing or core refused it (tried again once a
  // second; a refusal is reported once).
  enum class LogShare { kWaiting, kShared, kOff };
  LogShare m_logShare = LogShare::kWaiting;
  bool m_logShareRefused = false;
};

struct Instance {
  Instance(std::string_view dir, std::string_view filename, double period);
  prosdk::util::SafeThreadOwner<Thread> owner;
};

}  // namespace

// if less than this much free space, delete log files until there is this much
// free space OR there are this many files remaining.
// prosdk: scaled to the robot's flash (prosdk/platform/WpilibcConfig.hpp): on the
// ESP32 the threshold is PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD bytes, at
// most half the file system, and fewer files are kept; the host keeps upstream's
// 50 MB / 10 files.
static constexpr uintmax_t FREE_SPACE_THRESHOLD =
    PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD;
static constexpr int FILE_COUNT_THRESHOLD =
    PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD;
#if PROSDK_TARGET_HW
static constexpr uintmax_t SPACE_UNIT = 1024;
static constexpr const char* SPACE_UNIT_NAME = "KiB";
#else
static constexpr uintmax_t SPACE_UNIT = 1000000;
static constexpr const char* SPACE_UNIT_NAME = "MB";
#endif

namespace {

// prosdk: console capture (see Thread::m_consoleLog). The tee is a plain
// function, so the buffer is global; it is bounded (no allocation while
// capturing) and counts what it had to drop.
struct ConsoleCapture {
  std::mutex mutex;
  bool enabled = false;
  std::string buf;
  size_t dropped = 0;
};

ConsoleCapture& GetConsoleCapture() {
  static ConsoleCapture capture;
  return capture;
}

void ConsoleCaptureTee(std::string_view text) {
  auto& capture = GetConsoleCapture();
  std::scoped_lock lock{capture.mutex};
  if (!capture.enabled) {
    return;
  }
  size_t room = PROSDK_DATALOGMANAGER_CONSOLE_BUFFER - capture.buf.size();
  if (text.size() > room) {
    capture.dropped += text.size() - room;
    text = text.substr(0, room);
  }
  capture.buf.append(text);
}

}  // namespace

// prosdk: the default directory comes from prosdk::platform::GetLogDirectory()
// (prosdk/platform/Storage.hpp); "" means no storage and selects a discarding log.
static std::string MakeLogDir(std::string_view dir) {
  if (!dir.empty()) {
    return std::string{dir};
  }
  std::string logDir = prosdk::platform::GetLogDirectory();
  if (logDir.empty()) {
    prosdk::platform::ConsoleWrite(
        "DataLogManager: no log storage (LittleFS is not mounted); log data is discarded\n",
        prosdk::platform::ConsoleLevel::kWarning);
    return logDir;
  }
  std::error_code ec;
  prosdk::fs::create_directory(logDir, ec);
  return logDir;
}

static std::string MakeLogFilename(std::string_view filenameOverride) {
  if (!filenameOverride.empty()) {
    return std::string{filenameOverride};
  }
  // prosdk: the shared platform engine (no std::random_device / mt19937 on the stack)
  prosdk::platform::RandomEngine rng;
  std::uniform_int_distribution<int> dist(0, 15);
  const char* v = "0123456789abcdef";
  std::string filename = "WPILIB_TBD_";
  for (int i = 0; i < 16; i++) {
    filename += v[dist(rng)];
  }
  filename += ".wpilog";
  return filename;
}

// prosdk: without a log directory the writer discards its output (same API, no file)
static prosdk::log::DataLogBackgroundWriter MakeWriter(std::string_view dir,
                                                       std::string_view filename,
                                                       double period) {
  if (dir.empty()) {
    return prosdk::log::DataLogBackgroundWriter{
        [](std::span<const uint8_t>) {}, period};
  }
  return prosdk::log::DataLogBackgroundWriter{dir, MakeLogFilename(filename),
                                              period};
}

Thread::Thread(std::string_view dir, std::string_view filename, double period)
    : m_logDir{dir},
      m_filenameOverride{!filename.empty()},
      m_log{MakeWriter(dir, filename, period)},
      m_messageLog{m_log, "messages"} {
  // prosdk: per-thread stack and name (wpiutil SafeThread, read by Start());
  // the date/time std::format of the rename needs more than the default
  m_stackSize = PROSDK_DATALOGMANAGER_THREAD_STACK_SIZE;
  m_threadName = "datalogmgr";
  // prosdk: the telemetry copy is off by default on the robot (PROSDK_DATALOGMANAGER_LOG_TELEMETRY)
  if (PROSDK_DATALOGMANAGER_LOG_TELEMETRY) {
    StartNTLog();
  }
  StartConsoleLog();
}

Thread::~Thread() {
  StopNTLog();
  StopConsoleLog();
  // prosdk: take the folder off the Driver Station. Core waits at most 1 s for a running download
  // to stop; a download stalled on a slow browser can keep a file open longer.
  if (m_logShare == LogShare::kShared) {
    prosdk::platform::UnshareFiles(kLogShareName);
  }
}

void Thread::ShareLogDir() {
  if (m_logShare != LogShare::kWaiting) {
    return;
  }
  // no folder, or a probot-core without file sharing: nothing to offer, nothing to report
  if (m_logDir.empty() || !prosdk::platform::FileSharingSupported()) {
    m_logShare = LogShare::kOff;
    return;
  }
  std::error_code ec;
  if (!prosdk::fs::is_directory(m_logDir, ec)) {
    return;  // tried again later
  }
  if (prosdk::platform::ShareFiles(kLogShareName, m_logDir.c_str())) {
    m_logShare = LogShare::kShared;
  } else if (!m_logShareRefused) {
    // tried again later too: with 4 shares, a slot may free up when the sketch unshares one
    m_logShareRefused = true;
    prosdk::platform::ConsolePrint(
        prosdk::platform::ConsoleLevel::kWarning,
        "DataLogManager: the Driver Station cannot offer the logs in {} (probot-core shares an "
        "absolute path of at most 95 bytes without . or .., and at most 4 folders); trying again "
        "once a second\n",
        m_logDir);
  }
}

void Thread::Main() {
  // based on free disk space, scan for "old" WPILIB_*.wpilog files and remove
  {
    std::error_code ec;
    uintmax_t freeSpace;
    uintmax_t freeSpaceThreshold = FREE_SPACE_THRESHOLD;
    // prosdk: the ESP32 has no statvfs (fs::space fails there); the platform
    // asks esp_littlefs for the LittleFS partition (prosdk/platform/Storage.hpp)
    auto freeSpaceInfo = prosdk::platform::GetStorageSpace(m_logDir);
    if (freeSpaceInfo) {
      freeSpace = freeSpaceInfo->available;
#if PROSDK_TARGET_HW
      // never aim for more than half of a small partition
      freeSpaceThreshold =
          std::min(freeSpaceThreshold, freeSpaceInfo->capacity / 2);
#endif
    } else {
      freeSpace = UINTMAX_MAX;
    }
    if (!m_logDir.empty() && freeSpace < freeSpaceThreshold) {
      // Delete oldest WPILIB_*.wpilog files (ignore WPILIB_TBD_*.wpilog as we
      // just created one)
      std::vector<prosdk::fs::directory_entry> entries;
      for (auto&& entry : prosdk::fs::directory_iterator{m_logDir, ec}) {
        auto stem = entry.path().stem().string();
        if (prosdk::util::starts_with(stem, "WPILIB_") &&
            entry.path().extension() == ".wpilog" &&
            !prosdk::util::starts_with(stem, "WPILIB_TBD_")) {
          entries.emplace_back(entry);
        }
      }
      // prosdk: ties (a file system without modification times) are broken by
      // name, which is the date/time for renamed logs
      std::sort(entries.begin(), entries.end(),
                [](const auto& a, const auto& b) {
                  std::error_code aec, bec;
                  auto at = a.last_write_time(aec);
                  auto bt = b.last_write_time(bec);
                  if (!aec && !bec && at != bt) {
                    return at < bt;
                  }
                  return a.path() < b.path();
                });

      int count = entries.size();
      for (auto&& entry : entries) {
        --count;
        if (count < FILE_COUNT_THRESHOLD) {
          break;
        }
        auto size = entry.file_size();
        if (prosdk::fs::remove(entry.path(), ec)) {
          WPILIB_ReportWarning("DataLogManager: Deleted {}",
                               entry.path().string());
          freeSpace += size;
          if (freeSpace >= freeSpaceThreshold) {
            break;
          }
        } else {
          prosdk::platform::ConsolePrint(prosdk::platform::ConsoleLevel::kWarning,
                                         "DataLogManager: could not delete {}\n",
                                         entry.path().string());
        }
      }
    } else if (!m_logDir.empty() && freeSpace < 2 * freeSpaceThreshold) {
      WPILIB_ReportError(
          warn::Warning,
          "DataLogManager: Log storage device has {} {} of free space "
          "remaining! Logs will get deleted below {} {} of free space. "
          "Consider deleting logs off the storage device.",
          freeSpace / SPACE_UNIT, SPACE_UNIT_NAME,
          freeSpaceThreshold / SPACE_UNIT, SPACE_UNIT_NAME);
    }
  }

  // prosdk: offer the folder on the Driver Station; while it does not exist or core refuses it,
  // try again once a second
  ShareLogDir();
  uint64_t shareTry = prosdk::util::Now();

  int timeoutCount = 0;
  bool paused = false;
  int dsAttachCount = 0;
  int fmsAttachCount = 0;
  bool dsRenamed = m_filenameOverride;
  bool fmsRenamed = m_filenameOverride;
  int sysTimeCount = 0;
  prosdk::log::IntegerLogEntry sysTimeEntry{
      m_log, "systemTime",
      "{\"source\":\"DataLogManager\",\"format\":\"time_t_us\"}"};

  prosdk::util::Event newDataEvent;
  prosdk::internal::DriverStationBackend::ProvideRefreshedDataEventHandle(
      newDataEvent.GetHandle());

  for (;;) {
    bool timedOut = false;
    bool newData =
        prosdk::util::WaitForObject(newDataEvent.GetHandle(), 0.25, &timedOut);
    if (!m_active) {
      break;
    }
    DrainConsoleLog(false);
    if (m_logShare == LogShare::kWaiting && prosdk::util::Now() - shareTry >= 1000000) {
      shareTry = prosdk::util::Now();
      ShareLogDir();
    }
    if (!newData) {
      ++timeoutCount;
      // pause logging after being disconnected for 10 seconds
      if (timeoutCount > 40 && !paused) {
        timeoutCount = 0;
        paused = true;
        m_log.Pause();
      }
      continue;
    }
    // when we connect to the DS, resume logging
    timeoutCount = 0;
    if (paused) {
      paused = false;
      m_log.Resume();
    }

    if (!dsRenamed) {
      // track DS attach
      if (RobotState::IsDSAttached()) {
        ++dsAttachCount;
      } else {
        dsAttachCount = 0;
      }
      if (dsAttachCount > 50) {  // 1 second
        if (RobotController::IsSystemTimeValid()) {
          auto now = std::chrono::system_clock::now();
          m_log.SetFilename(
              std::format("WPILIB_{}.wpilog", FileTimestamp(now)));
          dsRenamed = true;
        } else {
          dsAttachCount = 0;  // wait a bit and try again
        }
      }
    }

    if (!fmsRenamed) {
      // track FMS attach
      if (RobotState::IsFMSAttached()) {
        ++fmsAttachCount;
      } else {
        fmsAttachCount = 0;
      }
      if (fmsAttachCount > 250) {  // 5 seconds
        // match info comes through TCP, so we need to double-check we've
        // actually received it
        auto matchType = MatchState::GetMatchType();
        if (matchType != prosdk::MatchType::NONE) {
          // rename per match info
          char matchTypeChar;
          switch (matchType) {
            case prosdk::MatchType::PRACTICE:
              matchTypeChar = 'P';
              break;
            case prosdk::MatchType::QUALIFICATION:
              matchTypeChar = 'Q';
              break;
            case prosdk::MatchType::ELIMINATION:
              matchTypeChar = 'E';
              break;
            default:
              matchTypeChar = '_';
              break;
          }
          auto now = std::chrono::system_clock::now();
          m_log.SetFilename(
              std::format("WPILIB_{}_{}_{}{}.wpilog", FileTimestamp(now),
                          MatchState::GetEventName(), matchTypeChar,
                          MatchState::GetMatchNumber()));
          fmsRenamed = true;
          dsRenamed = true;  // don't override FMS rename
        }
      }
    }

    // Write system time every ~5 seconds
    ++sysTimeCount;
    if (sysTimeCount >= 250) {
      sysTimeCount = 0;
      if (RobotController::IsSystemTimeValid()) {
        sysTimeEntry.Append(prosdk::util::GetSystemTime() / 1000,
                            prosdk::util::Now());
      }
    }
  }
  prosdk::internal::DriverStationBackend::RemoveRefreshedDataEventHandle(
      newDataEvent.GetHandle());
  DrainConsoleLog(true);
}

// prosdk: no ntcore. What upstream's NetworkTables entry logger records of the robot's own
// data is its telemetry ("NT:/Telemetry/..."); here the telemetry backend registered for ""
// also writes into this log under that name (prosdk/platform/TelemetryLog.hpp). Without a log
// directory the writer discards its output, so nothing is copied.
void Thread::StartNTLog() {
  if (!m_logDir.empty()) {
    prosdk::platform::StartTelemetryLog(m_log);
  }
}

void Thread::StopNTLog() {
  prosdk::platform::StopTelemetryLog(m_log);
}

void Thread::StartConsoleLog(bool force) {
  if (!force && !RobotBase::IsReal()) {
    return;
  }
  auto& capture = GetConsoleCapture();
  {
    std::scoped_lock lock{capture.mutex};
    if (capture.enabled) {
      return;
    }
    capture.enabled = true;
    capture.buf.clear();
    capture.buf.reserve(PROSDK_DATALOGMANAGER_CONSOLE_BUFFER);
    capture.dropped = 0;
  }
  {
    std::scoped_lock lock{m_consoleLogMutex};
    if (!m_consoleLog) {
      m_consoleLog = prosdk::log::StringLogEntry{m_log, "console"};
    }
  }
  prosdk::platform::SetConsoleTee(ConsoleCaptureTee);
}

void Thread::StopConsoleLog() {
  auto& capture = GetConsoleCapture();
  {
    std::scoped_lock lock{capture.mutex};
    if (!capture.enabled) {
      return;
    }
    capture.enabled = false;
  }
  prosdk::platform::SetConsoleTee(nullptr);
  DrainConsoleLog(true);
}

void Thread::DrainConsoleLog(bool all) {
  std::scoped_lock logLock{m_consoleLogMutex};
  std::string text;
  size_t dropped;
  {
    auto& capture = GetConsoleCapture();
    std::scoped_lock lock{capture.mutex};
    // upstream's FileLogger::Buffer: whole lines, without the last newline; a
    // full buffer without a newline is flushed as is
    size_t n = capture.buf.size();
    if (!all) {
      size_t newline = capture.buf.rfind('\n');
      if (newline != std::string::npos) {
        n = newline + 1;
      } else if (n < PROSDK_DATALOGMANAGER_CONSOLE_BUFFER) {
        n = 0;
      }
    }
    text.assign(capture.buf, 0, n);
    capture.buf.erase(0, n);
    dropped = std::exchange(capture.dropped, 0);
  }
  if (!m_consoleLog) {
    return;
  }
  if (!text.empty()) {
    if (text.back() == '\n') {
      text.pop_back();
    }
    m_consoleLog.Append(text);
  }
  if (dropped != 0) {
    m_consoleLog.Append(std::format(
        "[DataLogManager: {} bytes of console output dropped]", dropped));
  }
}

Instance::Instance(std::string_view dir, std::string_view filename,
                   double period) {
  // Delete all previously existing WPILIB_TBD_*.wpilog files. These only exist
  // when the robot never connects to the DS, so they are very unlikely to
  // have useful data and just clutter the filesystem.
  auto logDir = MakeLogDir(dir);
  std::error_code ec;
  for (auto&& entry : prosdk::fs::directory_iterator{logDir, ec}) {
    if (prosdk::util::starts_with(entry.path().stem().string(), "WPILIB_TBD_") &&
        entry.path().extension() == ".wpilog") {
      if (!prosdk::fs::remove(entry, ec)) {
        prosdk::platform::ConsolePrint(prosdk::platform::ConsoleLevel::kWarning,
                                       "DataLogManager: could not delete {}\n",
                                       entry.path().string());
      }
    }
  }

  owner.Start(logDir, filename, period);
}

static Instance& GetInstance(std::string_view dir = "",
                             std::string_view filename = "",
                             double period = 0.25) {
  static Instance instance(dir, filename, period);
  if (!instance.owner) {
    instance.owner.Start(MakeLogDir(dir), filename, period);
  }
  return instance;
}

void DataLogManager::Start(std::string_view dir, std::string_view filename,
                           double period) {
  GetInstance(dir, filename, period);
}

void DataLogManager::Stop() {
  auto& inst = GetInstance();
  inst.owner.GetThread()->m_log.Stop();
  inst.owner.Join();
}

void DataLogManager::Log(std::string_view message) {
  GetInstance().owner.GetThread()->m_messageLog.Append(message);
  prosdk::platform::ConsolePrint("{}\n", message);
}

prosdk::log::DataLog& DataLogManager::GetLog() {
  return GetInstance().owner.GetThread()->m_log;
}

std::string DataLogManager::GetLogDir() {
  return GetInstance().owner.GetThread()->m_logDir;
}

void DataLogManager::LogNetworkTables(bool enabled) {
  if (auto thr = GetInstance().owner.GetThread()) {
    if (enabled) {
      thr->StartNTLog();
    } else if (!enabled) {
      thr->StopNTLog();
    }
  }
}

void DataLogManager::LogConsoleOutput(bool enabled) {
  if (auto thr = GetInstance().owner.GetThread()) {
    if (enabled) {
      thr->StartConsoleLog(true);
    } else if (!enabled) {
      thr->StopConsoleLog();
    }
  }
}
