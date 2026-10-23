// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/Dashboard.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/TelemetryLog.hpp"
#include "prosdk/platform/dashboard/DashboardAlerts.hpp"
#include "prosdk/platform/dashboard/DashboardDisplay.hpp"
#include "prosdk/platform/dashboard/DashboardPreferences.hpp"
#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/platform/dashboard/ProbotTelemetryBackend.hpp"
#include "prosdk/platform/dashboard/ProbotTunableBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"

namespace prosdk::platform {

namespace {

// core's user task, before the loop iteration
void DashboardPreCycle(void*) {
  dashboard::DrainWrites();
  dashboard::FlushPending();
}

// core's user task, after the loop iteration
void DashboardPostCycle(void*) {
  dashboard::PublishAlerts();
}

}  // namespace

void RegisterDashboardBackends() {
  static std::once_flag once;
  std::call_once(once, [] {
    // registered for "" with DataLogManager's log copy while it runs (prosdk/platform/TelemetryLog.hpp);
    // the display screen goes to the Driver Station only (upstream does not log the display)
    auto telemetry = std::make_shared<prosdk::backend::ProbotTelemetryBackend>("/Telemetry");
    prosdk::telemetry::TelemetryRegistry::RegisterBackend(
        std::string{"/"} + std::string{dashboard::kDisplayTable}, telemetry);
    SetDefaultTelemetryBackend(std::move(telemetry));
    prosdk::tunables::TunableRegistry::RegisterBackend(
        "", std::make_shared<prosdk::backend::ProbotTunableBackend>("/Tunables"));
    dashboard::TrackPreferences();
    dashboard::TrackAlerts();
    if (!AddCycleCallback(CyclePhase::kPre, DashboardPreCycle, nullptr) ||
        !AddCycleCallback(CyclePhase::kPost, DashboardPostCycle, nullptr)) {
      ConsoleWrite(
          "[prosdk] dashboard: core cycle callback table full; Driver Station tunable writes "
          "and alerts are not processed\n",
          ConsoleLevel::kError);
    }
  });
}

bool ClearPersistedTunables() {
  return dashboard::ClearPersisted();
}

}  // namespace prosdk::platform
