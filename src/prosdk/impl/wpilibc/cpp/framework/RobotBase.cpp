// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/framework/RobotBase.hpp"

#include <stdint.h>

#include <atomic>
#include <cstdio>
#include <format>
#include <memory>
#include <mutex>
#include <prosdk/cxx/print.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/driverstation/internal/DriverStationBackend.hpp"
#include "prosdk/hal/HAL.h"
#include "prosdk/hal/UsageReporting.hpp"
#include "prosdk/math/util/MathShared.hpp"
#include "prosdk/platform/ConsoleFormat.hpp"
#include "prosdk/platform/RobotHost.hpp"
#include "prosdk/system/Errors.hpp"
#include "prosdk/system/WPILibVersion.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/util/Alert.hpp"
#include "prosdk/util/UsageReporting.hpp"
#include "prosdk/util/print.hpp"
#include "prosdk/util/timestamp.hpp"

static_assert(prosdk::RuntimeType::SYSTEMCORE ==
              static_cast<prosdk::RuntimeType>(HAL_RUNTIME_SYSTEMCORE));
static_assert(prosdk::RuntimeType::SIMULATION ==
              static_cast<prosdk::RuntimeType>(HAL_RUNTIME_SIMULATION));

using namespace prosdk;

int prosdk::RunHALInitialization() {
  if (!HAL_Initialize()) {
    prosdk::platform::ConsoleWrite("FATAL ERROR: HAL could not be initialized\n");
    return -1;
  }
  prosdk::util::SetReportUsageImpl(
      static_cast<WPI_ReportUsageImpl>(HAL_ReportUsage));
  prosdk::internal::DriverStationBackend::RefreshData();
  prosdk::util::ReportUsage("Language", "C++");
  prosdk::util::ReportUsage("WPILibVersion", GetWPILibVersion());
  HAL_PublishWpilibVersion(std::format("{} (C++)", GetWPILibVersion()));

  prosdk::platform::ConsoleWrite("\n********** Robot program starting **********\n");
  return 0;
}

std::thread::id RobotBase::m_threadId;

namespace {
std::atomic<uint32_t> s_nextWarningReporterId{0};

void PrintDefaultTelemetryWarning(std::string_view path, std::string_view msg) {
  prosdk::platform::ConsolePrint("Telemetry '{}': warning: {}\n", path, msg);
}

void PrintDefaultTunableWarning(std::string_view msg) {
  prosdk::platform::ConsolePrint("Tunable warning: {}\n", msg);
}

class WPILibMathShared : public prosdk::math::MathShared {
 public:
  void ReportErrorV(std::string_view format, std::format_args args) override {
    prosdk::ReportErrorV(err::Error, __FILE__, __LINE__, __FUNCTION__, format,
                      args);
  }

  void ReportWarningV(std::string_view format, std::format_args args) override {
    prosdk::ReportErrorV(warn::Warning, __FILE__, __LINE__, __FUNCTION__, format,
                      args);
  }

  prosdk::units::second_t GetTimestamp() override {
    return prosdk::units::second_t{prosdk::util::Now() * 1.0e-9};
  }
};
}  // namespace

struct prosdk::RobotBase::WarningReporter {
  WarningReporter()
      : m_alertIdPrefix{std::format("warning{}_", s_nextWarningReporterId++)} {}

  void ReportTelemetry(std::string_view path, std::string_view msg) {
    std::string key{path};
    key.push_back('\n');
    key.append(msg.data(), msg.size());
    Report(m_telemetryAlerts, "Telemetry", key,
           std::format("Telemetry '{}': warning: {}", path, msg));
  }

  void ReportTunable(std::string_view msg) {
    Report(m_tunableAlerts, "Tunables", msg,
           std::format("Tunable warning: {}", msg));
  }

 private:
  void Report(std::unordered_map<std::string, prosdk::util::Alert>& alerts,
              std::string_view group, std::string_view key,
              std::string_view text) {
    std::scoped_lock lock{m_mutex};
    auto [it, inserted] = alerts.try_emplace(std::string{key});
    if (inserted || !it->second) {
      it->second = prosdk::util::Alert{
          group, std::format("{}{}", m_alertIdPrefix, m_nextAlertId++), text,
          prosdk::util::Alert::Level::kMedium};
      if (!it->second) {
        alerts.erase(it);
        return;
      }
    }

    it->second.SetText(text);
    it->second.Set(true);
  }

  prosdk::util::mutex m_mutex;
  std::string m_alertIdPrefix;
  uint32_t m_nextAlertId = 0;
  std::unordered_map<std::string, prosdk::util::Alert> m_telemetryAlerts;
  std::unordered_map<std::string, prosdk::util::Alert> m_tunableAlerts;
};

static void SetupMathShared() {
  prosdk::math::MathSharedStore::SetMathShared(
      std::make_unique<WPILibMathShared>());
}

bool RobotBase::IsEnabled() {
  return RobotState::IsEnabled();
}

bool RobotBase::IsDisabled() {
  return RobotState::IsDisabled();
}

bool RobotBase::IsAutonomous() {
  return RobotState::IsAutonomous();
}

bool RobotBase::IsAutonomousEnabled() {
  return RobotState::IsAutonomousEnabled();
}

bool RobotBase::IsTeleop() {
  return RobotState::IsTeleop();
}

bool RobotBase::IsTeleopEnabled() {
  return RobotState::IsTeleopEnabled();
}

bool RobotBase::IsUtility() {
  return RobotState::IsUtility();
}

bool RobotBase::IsUtilityEnabled() {
  return RobotState::IsUtilityEnabled();
}

int64_t RobotBase::GetOpModeId() {
  return RobotState::GetOpModeId();
}

std::string RobotBase::GetOpMode() {
  return RobotState::GetOpMode();
}

std::thread::id RobotBase::GetThreadId() {
  return m_threadId;
}

RuntimeType RobotBase::GetRuntimeType() {
  return static_cast<RuntimeType>(HAL_GetRuntimeType());
}

RobotBase::RobotBase() {
  m_threadId = std::this_thread::get_id();

  SetupMathShared();
  m_warningReporter = std::make_shared<WarningReporter>();
  std::weak_ptr<WarningReporter> warningReporter{m_warningReporter};

  // prosdk: no NetworkTables server (ntcore is not ported)

  prosdk::telemetry::TelemetryRegistry::SetReportWarning(
      [warningReporter](std::string_view path, std::string_view msg) {
        if (auto reporter = warningReporter.lock()) {
          reporter->ReportTelemetry(path, msg);
        } else {
          PrintDefaultTelemetryWarning(path, msg);
        }
      });
  prosdk::tunables::TunableRegistry::SetReportWarning(
      [warningReporter](std::string_view msg) {
        if (auto reporter = warningReporter.lock()) {
          reporter->ReportTunable(msg);
        } else {
          PrintDefaultTunableWarning(msg);
        }
      });

  // prosdk: upstream registers the NetworkTables telemetry and tunable backends, waits for the
  // NT server, publishes /Robot/ProgramStartTime and reports NT connections here. The robot host
  // (prosdk/platform/RobotHost.hpp) installs its backends instead.
  prosdk::platform::OnRobotBaseConstructed();

  // Call prosdk::internal::DriverStationBackend::RefreshData() to kick things off
  prosdk::internal::DriverStationBackend::RefreshData();
}

RobotBase::~RobotBase() {
  m_warningReporter.reset();
}
