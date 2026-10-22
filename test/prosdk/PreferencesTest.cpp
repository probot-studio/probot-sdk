// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk::Preferences on the platform store (prosdk/platform/PreferencesStore.hpp; upstream has
// no C++ Preferences test) and the platform storage locations (prosdk/platform/Storage.hpp).

#include <cstdio>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/platform/PreferencesStore.hpp"
#include "prosdk/platform/Storage.hpp"
#include "prosdk/preferences/Preferences.hpp"
#include "prosdk/system/Filesystem.hpp"

using prosdk::Preferences;

namespace {
struct PreferencesFixture {
  PreferencesFixture() {
    prosdk::platform::prefs::SetPreferencesFile("");
    prosdk::platform::prefs::Reload();
  }
  ~PreferencesFixture() {
    prosdk::platform::prefs::SetPreferencesFile("");
    prosdk::platform::prefs::Reload();
  }
};
}  // namespace

TEST_CASE_METHOD(PreferencesFixture, "Preferences typed values and defaults", "[prosdk]") {
  CHECK(Preferences::GetKeys().empty());
  CHECK(Preferences::GetDouble("missing", 1.5) == 1.5);
  CHECK_FALSE(Preferences::ContainsKey("missing"));

  Preferences::SetString("name", "probot");
  Preferences::SetBoolean("flag", true);
  Preferences::SetInt("count", 7);
  Preferences::SetLong("big", 1LL << 40);
  Preferences::SetDouble("kP", 0.25);
  Preferences::SetFloat("kI", 0.5f);

  CHECK(Preferences::GetString("name") == "probot");
  CHECK(Preferences::GetBoolean("flag") == true);
  CHECK(Preferences::GetInt("count") == 7);
  CHECK(Preferences::GetLong("big") == (1LL << 40));
  CHECK(Preferences::GetDouble("kP") == 0.25);
  CHECK(Preferences::GetFloat("kI") == 0.5f);
  CHECK(Preferences::GetKeys().size() == 6);

  // numeric getters convert between integer, float and double (NetworkTables semantics)
  CHECK(Preferences::GetDouble("count") == 7.0);
  CHECK(Preferences::GetInt("kP", 99) == 0);
  CHECK(Preferences::GetFloat("kP") == 0.25f);
  // other types yield the default
  CHECK(Preferences::GetString("count", "def") == "def");
  CHECK(Preferences::GetBoolean("count", false) == false);
  CHECK(Preferences::GetDouble("name", -1.0) == -1.0);

  // Init only sets absent keys
  Preferences::InitDouble("kP", 9.0);
  CHECK(Preferences::GetDouble("kP") == 0.25);
  Preferences::InitDouble("kD", 2.0);
  CHECK(Preferences::GetDouble("kD") == 2.0);

  // Set replaces value and type
  Preferences::SetString("count", "seven");
  CHECK(Preferences::GetString("count") == "seven");
  CHECK(Preferences::GetInt("count", -1) == -1);

  Preferences::Remove("kD");
  CHECK_FALSE(Preferences::ContainsKey("kD"));
  Preferences::RemoveAll();
  CHECK(Preferences::GetKeys().empty());
}

TEST_CASE_METHOD(PreferencesFixture, "Preferences persist to the host JSON file", "[prosdk]") {
  const std::string path = "prosdk_preferences_test.json";
  std::remove(path.c_str());
  prosdk::platform::prefs::SetPreferencesFile(path);
  prosdk::platform::prefs::Reload();

  Preferences::SetDouble("kP", 0.125);
  Preferences::SetFloat("kF", 1.5f);
  Preferences::SetLong("id", -42);
  Preferences::SetBoolean("on", false);
  Preferences::SetString("text", "a \"quoted\" value");

  // memory dropped: the values come back from the file with their types
  prosdk::platform::prefs::Reload();
  CHECK(Preferences::GetDouble("kP") == 0.125);
  CHECK(Preferences::GetFloat("kF") == 1.5f);
  CHECK(Preferences::GetLong("id") == -42);
  CHECK(Preferences::GetBoolean("on", true) == false);
  CHECK(Preferences::GetString("text") == "a \"quoted\" value");
  CHECK(Preferences::GetKeys().size() == 5);

  Preferences::Remove("kP");
  prosdk::platform::prefs::Reload();
  CHECK_FALSE(Preferences::ContainsKey("kP"));

  std::remove(path.c_str());
}

TEST_CASE("Platform storage locations on the host", "[prosdk]") {
  CHECK_FALSE(prosdk::platform::IsLittleFsMounted());
  auto launch = prosdk::filesystem::GetLaunchDirectory();
  CHECK(prosdk::filesystem::GetOperatingDirectory() == launch);
  CHECK(prosdk::filesystem::GetDeployDirectory() == launch + "/src/main/deploy");
  CHECK(prosdk::platform::GetLogDirectory() == launch + "/logs");

  prosdk::platform::SetDeployDirectory("/tmp/deploy");
  prosdk::platform::SetLogDirectory("/tmp/logs");
  CHECK(prosdk::filesystem::GetDeployDirectory() == "/tmp/deploy");
  CHECK(prosdk::platform::GetLogDirectory() == "/tmp/logs");
  prosdk::platform::SetDeployDirectory("");
  prosdk::platform::SetLogDirectory("");
  CHECK(prosdk::filesystem::GetDeployDirectory() == launch + "/src/main/deploy");
}
