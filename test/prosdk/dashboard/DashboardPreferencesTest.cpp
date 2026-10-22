// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Preferences on the Driver Station (prosdk/platform/dashboard/DashboardPreferences.hpp) against
// the fake core store: each preference is a robust tunable /Preferences/<key>; store changes
// (added, changed, removed keys, changed types) appear at the next registry update, Driver Station
// writes go back to the store.

#include <limits>
#include <string>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/platform/PreferencesStore.hpp"
#include "prosdk/platform/dashboard/DashboardPreferences.hpp"
#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/Tunables.hpp"

using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;
namespace prefs = prosdk::platform::prefs;
namespace tunables = prosdk::tunables;

namespace {

constexpr uint8_t kPrefFlags = dash::kFlagTunable | dash::kFlagMutable | dash::kFlagRobust;

uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}

/** Dashboard fixture with an empty in-memory preferences store, mirrored. */
class PreferencesFixture : public DashboardFixture {
 public:
  PreferencesFixture() {
    prefs::SetPreferencesFile("");
    prefs::Reload();
    dash::TrackPreferences();
  }
  ~PreferencesFixture() {
    dash::ResetPreferencesForTest();
    prefs::Reload();
  }
};

}  // namespace

TEST_CASE_METHOD(PreferencesFixture, "Preferences are published under /Preferences",
                 "[prosdk][dashboard][preferences]") {
  prefs::Set("flag", prefs::Value{true});
  prefs::Set("count", prefs::Value{int64_t{7}});
  prefs::Set("gain", prefs::Value{0.5f});
  prefs::Set("kP", prefs::Value{1.25});
  prefs::Set("name", prefs::Value{std::string{"probot"}});
  CHECK_FALSE(fake::Find("/Preferences/kP"));  // nothing is published outside the update
  Cycle();

  for (std::string_view key : {"flag", "count", "gain", "kP", "name"}) {
    auto topic = fake::Find(std::string{"/Preferences/"} + std::string{key});
    REQUIRE(topic);
    CHECK(topic->flags == kPrefFlags);
    CHECK_FALSE(fake::Find(std::string{"/Tunables/Preferences/"} + std::string{key}));
  }
  CHECK(fake::GetBoolean("/Preferences/flag") == true);
  CHECK(fake::GetInt("/Preferences/count") == 7);
  CHECK(fake::GetFloat("/Preferences/gain") == 0.5f);
  CHECK(fake::GetDouble("/Preferences/kP") == 1.25);
  CHECK(fake::GetString("/Preferences/name") == "probot");
}

TEST_CASE_METHOD(PreferencesFixture, "Preferences already stored are published on tracking",
                 "[prosdk][dashboard][preferences]") {
  dash::ResetPreferencesForTest();
  prefs::Set("early", prefs::Value{2.0});
  dash::TrackPreferences();
  Cycle();
  CHECK(fake::GetDouble("/Preferences/early") == 2.0);
}

TEST_CASE_METHOD(PreferencesFixture, "A Driver Station edit is written to the preferences store",
                 "[prosdk][dashboard][preferences]") {
  prefs::Set("kP", prefs::Value{1.0});
  prefs::Set("name", prefs::Value{std::string{"a"}});
  Cycle();
  auto revision = prefs::Revision();

  fake::PushWrite("/Preferences/kP", VT(dash::ValueType::kDouble), fake::EncodeDouble(3.5));
  fake::PushWrite("/Preferences/name", VT(dash::ValueType::kString), fake::EncodeString("b"));
  Cycle();
  CHECK(prefs::Get("kP") == prefs::Value{3.5});
  CHECK(prefs::Get("name") == prefs::Value{std::string{"b"}});
  CHECK(prefs::Revision() > revision);
  CHECK(fake::GetDouble("/Preferences/kP") == 3.5);

  // the next sync finds store and tunable equal: no republish
  int sets = fake::Find("/Preferences/kP")->sets;
  Cycle();
  Cycle();
  CHECK(fake::Find("/Preferences/kP")->sets == sets);
  CHECK(prefs::Get("kP") == prefs::Value{3.5});
}

TEST_CASE_METHOD(PreferencesFixture, "Store changes reach the Driver Station",
                 "[prosdk][dashboard][preferences]") {
  prefs::Set("kP", prefs::Value{1.0});
  prefs::Set("old", prefs::Value{true});
  Cycle();

  SECTION("changed value") {
    prefs::Set("kP", prefs::Value{2.0});
    Cycle();
    CHECK(fake::GetDouble("/Preferences/kP") == 2.0);
  }
  SECTION("added key") {
    prefs::Init("added", prefs::Value{int64_t{3}});
    Cycle();
    CHECK(fake::GetInt("/Preferences/added") == 3);
    CHECK(fake::Find("/Preferences/added")->flags == kPrefFlags);
  }
  SECTION("removed key") {
    prefs::Remove("old");
    Cycle();
    CHECK_FALSE(fake::Find("/Preferences/old"));
    CHECK(fake::Find("/Preferences/kP"));
  }
  SECTION("changed type") {
    prefs::Set("kP", prefs::Value{std::string{"text"}});
    Cycle();
    auto topic = fake::Find("/Preferences/kP");
    REQUIRE(topic);
    CHECK(topic->valueType == VT(dash::ValueType::kString));
    CHECK(fake::GetString("/Preferences/kP") == "text");
  }
  SECTION("reload") {
    prefs::Reload();  // in memory: the reload finds nothing
    Cycle();
    CHECK_FALSE(fake::Find("/Preferences/kP"));
    CHECK_FALSE(fake::Find("/Preferences/old"));
  }
}

TEST_CASE_METHOD(PreferencesFixture, "A NaN preference is not republished every update",
                 "[prosdk][dashboard][preferences]") {
  prefs::Set("nan", prefs::Value{std::numeric_limits<double>::quiet_NaN()});
  Cycle();
  auto topic = fake::Find("/Preferences/nan");
  REQUIRE(topic);
  int sets = topic->sets;
  prefs::Set("other", prefs::Value{1.0});  // bumps the revision: a full sync runs
  Cycle();
  CHECK(fake::Find("/Preferences/nan")->sets == sets);
}

TEST_CASE_METHOD(PreferencesFixture, "Stopping the mirror unpublishes the preferences",
                 "[prosdk][dashboard][preferences]") {
  prefs::Set("kP", prefs::Value{1.0});
  Cycle();
  REQUIRE(fake::Find("/Preferences/kP"));
  dash::ResetPreferencesForTest();
  Cycle();
  CHECK_FALSE(fake::Find("/Preferences/kP"));
  prefs::Set("kP", prefs::Value{2.0});
  Cycle();
  CHECK_FALSE(fake::Find("/Preferences/kP"));
}
