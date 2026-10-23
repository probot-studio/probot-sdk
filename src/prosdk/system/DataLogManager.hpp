// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>

namespace prosdk::log {
class DataLog;
}  // namespace prosdk::log

namespace prosdk {

/**
 * Centralized data log that provides automatic data log file management. It
 * automatically cleans up old files when disk space is low and renames the file
 * based either on current date/time or (if available) competition match number.
 * prosdk: the data file is saved to prosdk::platform::GetLogDirectory()
 * (prosdk/platform/Storage.hpp): /littlefs/logs on the robot when the sketch has
 * mounted LittleFS, otherwise the log is kept but its data is discarded (and
 * GetLogDir() returns ""); <operating directory>/logs on the host.
 *
 * Log files are initially named "WPILIB_TBD_{random}.wpilog" until the DS
 * connects. After the DS connects, the log file is renamed to
 * "WPILIB_yyyyMMdd_HHmmss.wpilog" (where the date/time is UTC). If the FMS is
 * connected and provides a match number, the log file is renamed to
 * "WPILIB_yyyyMMdd_HHmmss_{event}_{match}.wpilog".
 *
 * On startup, all existing WPILIB_TBD log files are deleted. If there is less
 * than 50 MB of free space on the target storage, WPILIB_ log files are deleted
 * (oldest to newest) until there is 50 MB free OR there are 10 files remaining.
 * prosdk: on the robot the limits are scaled to the LittleFS partition
 * (prosdk/platform/WpilibcConfig.hpp): PROSDK_DATALOGMANAGER_FREE_SPACE_THRESHOLD
 * (128 KiB, at most half the partition) and
 * PROSDK_DATALOGMANAGER_FILE_COUNT_THRESHOLD (3 files); a warning is printed
 * below twice the threshold. Free space comes from esp_littlefs (only for logs
 * under /littlefs; elsewhere, e.g. an SD card, nothing is deleted).
 *
 * prosdk: NetworkTables is not ported; LogNetworkTables() has no effect.
 * The console log ("console" entry) holds the prosdk console output
 * (prosdk::platform::ConsoleWrite: errors, warnings, DataLogManager::Log and
 * other library messages), not Serial or stdout. It is copied through a
 * PROSDK_DATALOGMANAGER_CONSOLE_BUFFER byte buffer that the manager's thread
 * drains every loop (whole lines, stamped when drained, up to 250 ms late);
 * overflow is counted in a "bytes dropped" line. As upstream it starts by
 * default only on the robot (RobotBase::IsReal()); LogConsoleOutput(true) also
 * starts it on the host.
 *
 * prosdk: with a probot-core that shares files (R11), the log directory is
 * offered on the Driver Station as "logs" once it exists (Logs page,
 * "Dosyalar" card: listed and downloaded while the robot is idle). A share
 * named "logs" made by the sketch is replaced. A share core refuses (a path
 * against its rules, see prosdk::platform::ShareFiles, or 4 other shares
 * already offered) is reported once and tried again once a second, so it
 * succeeds once the sketch unshares a folder. Stop() takes the share back;
 * core waits at most 1 s for a running download to stop, and a download
 * stalled on a slow browser can keep a file open longer (each chunk may
 * take 2 s or more), so a sketch that unmounts the file system after Stop()
 * first waits until probot::files::busy() is false (probot-core R17, in #38
 * from ba2500b on; an earlier #38 draft has no busy(), there wait a few
 * seconds). prosdk itself never unmounts.
 * With an older core nothing is shared.
 */
class DataLogManager final {
 public:
  DataLogManager() = delete;

  /**
   * Start data log manager. The parameters have no effect if the data log
   * manager was already started (e.g. by calling another static function).
   *
   * @param dir if not empty, directory to use for data log storage
   * @param filename filename to use; if none provided, the filename is
   *     automatically generated
   * @param period time between automatic flushes to disk, in seconds;
   *               this is a time/storage tradeoff
   */
  static void Start(std::string_view dir = "", std::string_view filename = "",
                    double period = 0.25);

  /**
   * Stop data log manager.
   * prosdk: also stops offering the log directory on the Driver Station (see
   * the class comment); this may wait up to 1 s for a running download.
   */
  static void Stop();

  /**
   * Log a message to the "messages" entry. The message is also printed to
   * standard output (followed by a newline).
   *
   * @param message message
   */
  static void Log(std::string_view message);

  /**
   * Get the managed data log (for custom logging). Starts the data log manager
   * if not already started.
   *
   * @return data log
   */
  static prosdk::log::DataLog& GetLog();

  /**
   * Get the log directory.
   *
   * @return log directory
   */
  static std::string GetLogDir();

  /**
   * Enable or disable logging of NetworkTables data. Note that unlike the
   * network interface for NetworkTables, this will capture every value change.
   * Defaults to enabled.
   *
   * @param enabled true to enable, false to disable
   */
  static void LogNetworkTables(bool enabled);

  /**
   * Enable or disable logging of the console output. Defaults to enabled.
   * prosdk: defaults to enabled on the robot only; true enables it on the host
   * too (see the class comment).
   * @param enabled  true to enable, false to disable
   */
  static void LogConsoleOutput(bool enabled);
};

}  // namespace wpi
