// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Alerts on the Driver Station page (the WPILib "Alerts" widget layout under
// /Telemetry/Alerts/<group>) and the RegisterDashboardBackends() entry point.

#include <stdint.h>

#include <span>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/datalog/DataLogBackgroundWriter.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/TelemetryLog.hpp"
#include "prosdk/platform/dashboard/Dashboard.hpp"
#include "prosdk/platform/dashboard/DashboardAlerts.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/Alert.hpp"
#include "prosdk/util/timestamp.hpp"

using dashboard_test::DashboardFixture;
using Level = prosdk::util::Alert::Level;
namespace dash = prosdk::platform::dashboard;
using Strings = std::vector<std::string>;

namespace {

int64_t gNow = 1'000'000;

int64_t FakeNow() {
  return gNow;
}

class AlertsFixture : public DashboardFixture {
 public:
  AlertsFixture() {
    dash::TrackAlerts();
    dash::ResetAlertsForTest();
    prosdk::util::SetNowImpl(FakeNow);
    gNow = 1'000'000;
  }
  ~AlertsFixture() { prosdk::util::SetNowImpl(nullptr); }

  static void Tick() { gNow += 1'000; }
};

}  // namespace

TEST_CASE_METHOD(AlertsFixture, "Dashboard alerts use the Alerts widget layout",
                 "[prosdk][dashboard][alerts]") {
  prosdk::util::Alert error{"Drive", "left", "left motor stalled", Level::kHigh};
  prosdk::util::Alert warning{"Drive", "battery", "battery low", Level::kMedium};
  prosdk::util::Alert info{"Drive", "auto", "auto selected", Level::kLow};
  prosdk::util::Alert inactive{"Drive", "idle", "never shown", Level::kHigh};
  error.Set(true);
  warning.Set(true);
  info.Set(true);
  dash::PublishAlerts();

  auto type = fake::Find("/Telemetry/Alerts/Drive/.type");
  REQUIRE(type);
  CHECK(fake::GetString("/Telemetry/Alerts/Drive/.type") == "Alerts");
  CHECK(fake::Find("/Telemetry/Alerts/Drive/errors")->typeString == "string[]");
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Drive/errors") == Strings{"left motor stalled"});
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Drive/warnings") == Strings{"battery low"});
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Drive/infos") == Strings{"auto selected"});
}

TEST_CASE_METHOD(AlertsFixture, "Dashboard alerts are newest first per level",
                 "[prosdk][dashboard][alerts]") {
  prosdk::util::Alert first{"Arm", "a", "first", Level::kHigh};
  prosdk::util::Alert second{"Arm", "b", "second", Level::kHigh};
  prosdk::util::Alert third{"Arm", "c", "third", Level::kHigh};
  first.Set(true);
  Tick();
  second.Set(true);
  Tick();
  third.Set(true);
  dash::PublishAlerts();
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Arm/errors") ==
        (Strings{"third", "second", "first"}));

  // re-activating moves an alert to the front
  Tick();
  first.Set(false);
  first.Set(true);
  dash::PublishAlerts();
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Arm/errors") ==
        (Strings{"first", "third", "second"}));
}

TEST_CASE_METHOD(AlertsFixture, "Dashboard alerts use the default group and separate groups",
                 "[prosdk][dashboard][alerts]") {
  prosdk::util::Alert plain{"plain", "default group", Level::kLow};
  prosdk::util::Alert vision{"Vision", "cam", "camera lost", Level::kMedium};
  plain.Set(true);
  vision.Set(true);
  dash::PublishAlerts();
  CHECK(fake::GetString("/Telemetry/Alerts/Alerts/.type") == "Alerts");
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Alerts/infos") == Strings{"default group"});
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Vision/warnings") == Strings{"camera lost"});
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Vision/errors") == Strings{});
}

TEST_CASE_METHOD(AlertsFixture, "Dashboard alerts clear and follow text changes",
                 "[prosdk][dashboard][alerts]") {
  {
    prosdk::util::Alert alert{"Intake", "jam", "intake jammed", Level::kHigh};
    alert.Set(true);
    dash::PublishAlerts();
    CHECK(fake::GetStringArray("/Telemetry/Alerts/Intake/errors") == Strings{"intake jammed"});

    alert.SetText("intake jammed (2)");
    dash::PublishAlerts();
    CHECK(fake::GetStringArray("/Telemetry/Alerts/Intake/errors") ==
          Strings{"intake jammed (2)"});

    alert.Set(false);
    dash::PublishAlerts();
    CHECK(fake::GetStringArray("/Telemetry/Alerts/Intake/errors") == Strings{});
    alert.Set(true);
  }
  // destroyed while active: the group stays with empty arrays
  dash::PublishAlerts();
  CHECK(fake::GetString("/Telemetry/Alerts/Intake/.type") == "Alerts");
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Intake/errors") == Strings{});
}

TEST_CASE_METHOD(AlertsFixture, "Dashboard alerts publish only after a change",
                 "[prosdk][dashboard][alerts]") {
  prosdk::util::Alert alert{"Loop", "slow", "loop overrun", Level::kMedium};
  alert.Set(true);
  dash::PublishAlerts();
  REQUIRE(fake::Find("/Telemetry/Alerts/Loop/warnings"));
  int sets = fake::Find("/Telemetry/Alerts/Loop/warnings")->sets;

  // Set(true) every loop is not a change
  for (int i = 0; i < 5; ++i) {
    Tick();
    alert.Set(true);
    dash::PublishAlerts();
  }
  CHECK(fake::Find("/Telemetry/Alerts/Loop/warnings")->sets == sets);
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Loop/warnings") == Strings{"loop overrun"});
}

TEST_CASE_METHOD(DashboardFixture, "RegisterDashboardBackends is idempotent",
                 "[prosdk][dashboard]") {
  prosdk::telemetry::TelemetryRegistry::Reset();
  prosdk::tunables::TunableRegistry::Reset();
  prosdk::platform::RegisterDashboardBackends();
  auto telemetryBackend = prosdk::telemetry::TelemetryRegistry::GetBackend("");
  auto tunableBackend = prosdk::tunables::TunableRegistry::GetBackend("");
  REQUIRE(telemetryBackend);
  REQUIRE(tunableBackend);

  prosdk::platform::RegisterDashboardBackends();
  CHECK(prosdk::telemetry::TelemetryRegistry::GetBackend("") == telemetryBackend);
  CHECK(prosdk::tunables::TunableRegistry::GetBackend("") == tunableBackend);

  // the cycle callbacks: the pre-cycle one drains Driver Station writes, the post-cycle one
  // publishes alerts
  prosdk::tunables::TunableDouble value{1.0};
  prosdk::tunables::Publish("registered", value);
  CHECK(fake::GetDouble("/Tunables/registered") == 1.0);
  fake::PushWrite("/Tunables/registered", static_cast<uint8_t>(dash::ValueType::kDouble),
                  fake::EncodeDouble(2.0));
  prosdk::platform::detail::PreCycle();
  prosdk::tunables::TunableRegistry::Update();
  CHECK(value.Get() == 2.0);

  dash::ResetAlertsForTest();
  prosdk::util::Alert alert{"Registered", "x", "visible", Level::kHigh};
  alert.Set(true);
  prosdk::platform::detail::PostCycle();
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Registered/errors") == Strings{"visible"});
  CHECK(fake::Rejects().empty());

  // DataLogManager's log copy (prosdk/platform/TelemetryLog.hpp): switching it on and off keeps
  // the published topics, also the alert that is logged again only when it changes; the display
  // table goes to the Driver Station backend without the copy
  using prosdk::telemetry::TelemetryRegistry;
  auto displayBackend = TelemetryRegistry::GetBackend("/DriverStationDisplay/text");
  CHECK(displayBackend != telemetryBackend);
  prosdk::log::DataLogBackgroundWriter log{[](std::span<const uint8_t>) {}, 0.25};
  prosdk::platform::StartTelemetryLog(log);
  CHECK(TelemetryRegistry::GetBackend("/Alerts/Registered/errors") != telemetryBackend);
  CHECK(TelemetryRegistry::GetBackend("/DriverStationDisplay/text") == displayBackend);
  prosdk::platform::detail::PostCycle();
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Registered/errors") == Strings{"visible"});
  prosdk::platform::StopTelemetryLog(log);
  CHECK(TelemetryRegistry::GetBackend("") == telemetryBackend);
  prosdk::platform::detail::PostCycle();
  CHECK(fake::GetStringArray("/Telemetry/Alerts/Registered/errors") == Strings{"visible"});
}
