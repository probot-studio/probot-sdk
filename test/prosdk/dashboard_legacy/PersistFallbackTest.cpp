// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Tunable persistence with a probot-core that has no dashboard persistence (CoreOps ABI 6
// dashClearPersisted null, as CoreBridge.hpp leaves it for a core without
// PROBOT_HAS_DASH_PERSIST). Own executable: the fake store is built with
// PROSDK_FAKE_CORE_NO_PERSIST (CoreOps is installed once per process). The opt-in is a no-op with
// one console warning per path (not repeated on a republish).

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/platform/dashboard/Dashboard.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/Tunables.hpp"

using dashboard_test::CountOf;
using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;
namespace tunables = prosdk::tunables;

TEST_CASE_METHOD(DashboardFixture, "Persistent tunable on a core without persistence",
                 "[prosdk][dashboard][tunable][persist]") {
  CHECK(prosdk::platform::HasCoreOps());
  CHECK_FALSE(prosdk::platform::DashPersistenceSupported());
  CHECK_FALSE(dash::PersistenceSupported());
  CHECK_FALSE(prosdk::platform::GetCoreOps().dashClearPersisted());
  CHECK_FALSE(prosdk::platform::ClearPersistedTunables());

  fake::SetPersisted("/Tunables/gain", static_cast<uint8_t>(dash::ValueType::kDouble), "double",
                     fake::EncodeDouble(7.5));
  tunables::TunableConfig config;
  config.properties["persistent"] = true;
  int onTune = 0;
  config.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) { ++onTune; };
  tunables::TunableDouble gain{1.0, config};
  tunables::TunableDouble other{2.0, config};
  REQUIRE(tunables::Publish("gain", gain));
  REQUIRE(tunables::Publish("other", other));
  DashboardFixture::Cycle();

  CHECK(gain.Get() == 1.0);
  CHECK(onTune == 0);
  auto topic = fake::Find("/Tunables/gain");
  REQUIRE(topic);
  CHECK(topic->flags == (dash::kFlagTunable | dash::kFlagMutable));
  CHECK(topic->properties == R"({"mutable":true,"persistent":true})");
  // one warning per persistent tunable
  CHECK(CountOf(fake::Console(), "does not keep dashboard values across reboots") == 2);
  CHECK(CountOf(fake::Console(), "/Tunables/gain") == 1);

  // Driver Station tunes still work
  fake::PushWrite("/Tunables/gain", static_cast<uint8_t>(dash::ValueType::kDouble),
                  fake::EncodeDouble(3.0));
  DashboardFixture::Cycle();
  CHECK(gain.Get() == 3.0);
  CHECK(onTune == 1);

  // published again (an OpMode change): no second warning for the same path
  tunables::Remove("gain");
  REQUIRE(tunables::Publish("gain", gain));
  {
    tunables::TunableDouble replacement{4.0, config};
    tunables::Remove("other");
    REQUIRE(tunables::Publish("other", replacement));
  }
  DashboardFixture::Cycle();
  CHECK(CountOf(fake::Console(), "does not keep dashboard values across reboots") == 2);
}
