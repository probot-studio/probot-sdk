// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Tunable persistence on probot-core's dashboard store (core R12 part 2, CoreOps ABI 6), a prosdk
// addition: a mutable tunable with TunableConfig::properties {"persistent": true} is published
// with core's PERSISTENT flag; core queues its stored value when the topic is created, and the
// backend applies it before Publish() returns (published before the store is ready: when it is
// published). Robust alone is not persistence. Published again in the same run, a tunable gets
// the last value set on its path (core stores the topic's current value, whoever set it). The fake
// store plays core's flash (fake::SetPersisted / PersistValues) and restore.

#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/geometry/struct/Translation2dStruct.hpp"
#include "prosdk/platform/dashboard/Dashboard.hpp"
#include "prosdk/tunables/Selectable.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/struct/Struct.hpp"

using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;
namespace tunables = prosdk::tunables;

namespace {

uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}

tunables::TunableConfig PersistentConfig(int* onTuneCalls = nullptr) {
  tunables::TunableConfig config;
  config.properties["persistent"] = true;
  if (onTuneCalls != nullptr) {
    config.onTune = [onTuneCalls](tunables::detail::TunableBase&, tunables::ComplexTunable*) {
      ++*onTuneCalls;
    };
  }
  return config;
}

void StoreDouble(std::string_view path, double value) {
  fake::SetPersisted(path, VT(dash::ValueType::kDouble), "double", fake::EncodeDouble(value));
}

// A robot part that publishes its gain in its constructor and reads it right after (a restored
// value must already be there, as upstream applies a robust tunable's retained value in Publish).
struct Arm {
  explicit Arm(int* onTuneCalls) : gain{1.0, PersistentConfig(onTuneCalls)} {
    published = tunables::Publish("arm/gain", gain);
    gainSeenInConstructor = gain.Get();
  }
  tunables::TunableDouble gain;
  bool published = false;
  double gainSeenInConstructor = 0.0;
};

}  // namespace

TEST_CASE_METHOD(DashboardFixture,
                 "Persistent tunable gets its stored value before Publish returns",
                 "[prosdk][dashboard][tunable][persist]") {
  StoreDouble("/Tunables/arm/gain", 7.5);
  int onTune = 0;
  Arm arm{&onTune};

  REQUIRE(arm.published);
  CHECK(arm.gainSeenInConstructor == 7.5);
  CHECK(onTune == 1);
  CHECK(tunables::TunableRegistry::GetTuneRevision(arm.gain) == 1);
  auto topic = fake::Find("/Tunables/arm/gain");
  REQUIRE(topic);
  CHECK(topic->flags == (dash::kFlagTunable | dash::kFlagMutable | dash::kFlagPersistent));
  CHECK(topic->properties == R"({"mutable":true,"persistent":true})");
  CHECK(fake::GetDouble("/Tunables/arm/gain") == 7.5);  // the initial publish sends it
  CHECK(fake::Rejects().empty());

  // nothing is applied a second time by the loop
  DashboardFixture::Cycle();
  CHECK(onTune == 1);
  CHECK(tunables::TunableRegistry::GetTuneRevision(arm.gain) == 1);

  // a Driver Station tune is stored by core (here: its idle save) and comes back after a reboot
  fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble), fake::EncodeDouble(9.0));
  DashboardFixture::Cycle();
  CHECK(arm.gain.Get() == 9.0);
  fake::PersistValues();
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(9.0));
}

TEST_CASE_METHOD(DashboardFixture, "Persistent tunable without a stored value keeps its code value",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  Arm arm{&onTune};
  CHECK(arm.gainSeenInConstructor == 1.0);
  CHECK(onTune == 0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(arm.gain) == 0);
  CHECK(fake::GetDouble("/Tunables/arm/gain") == 1.0);

  // a stored value of another type is not restored (core checks value type and type string)
  tunables::Remove("arm/gain");
  fake::SetPersisted("/Tunables/other", VT(dash::ValueType::kInt), "int", fake::EncodeInt(3));
  tunables::TunableDouble other{2.0, PersistentConfig(&onTune)};
  REQUIRE(tunables::Publish("other", other));
  CHECK(other.Get() == 2.0);
  CHECK(onTune == 0);
}

TEST_CASE_METHOD(DashboardFixture, "A stored value that does not decode is refused and erased",
                 "[prosdk][dashboard][tunable][persist]") {
  const prosdk::math::Translation2d initial{1.25_m, 2.5_m};
  fake::SetPersisted("/Tunables/target", VT(dash::ValueType::kRaw), "struct:Translation2d",
                     std::vector<uint8_t>{1, 2, 3});
  int onTune = 0;
  tunables::Tunable<prosdk::math::Translation2d> target{PersistentConfig(&onTune), initial};
  REQUIRE(tunables::Publish("target", target));

  CHECK(target.Get().X() == initial.X());
  CHECK(onTune == 0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(target) == 0);
  // core's rejectWrite of a restored value: record erased, no 'K' frame to the Driver Station
  CHECK(fake::RestoreErases() == 1);
  CHECK_FALSE(fake::GetPersisted("/Tunables/target"));
  CHECK(fake::Rejects().empty());
}

TEST_CASE_METHOD(DashboardFixture, "Robust tunables are not persistent",
                 "[prosdk][dashboard][tunable][persist]") {
  // stored values for every robust topic: none may come back
  StoreDouble("/Tunables/robust", 5.0);
  fake::SetPersisted("/Tunables/auto/selected", VT(dash::ValueType::kString), "string",
                     fake::EncodeString("two"));

  tunables::TunableConfig robust;
  robust.robust = true;
  tunables::TunableDouble value{1.0, robust};
  REQUIRE(tunables::Publish("robust", value));
  tunables::Selectable<int> chooser;
  chooser.AddDefault("one", 1);
  chooser.Add("two", 2);
  REQUIRE(tunables::Publish("auto", chooser));
  DashboardFixture::Cycle();

  CHECK(fake::Find("/Tunables/robust")->flags ==
        (dash::kFlagTunable | dash::kFlagMutable | dash::kFlagRobust));
  CHECK(value.Get() == 1.0);
  auto selected = fake::Find("/Tunables/auto/selected");
  REQUIRE(selected);
  CHECK((selected->flags & dash::kFlagRobust) != 0);
  CHECK((selected->flags & dash::kFlagPersistent) == 0);
  CHECK(chooser.GetSelected() == 1);

  // an immutable tunable is never persistent (a restored write could not be applied)
  auto immutable = PersistentConfig();
  immutable.isMutable = false;
  tunables::TunableDouble fixed{3.0, immutable};
  REQUIRE(tunables::Publish("fixed", fixed));
  CHECK((fake::Find("/Tunables/fixed")->flags & dash::kFlagPersistent) == 0);

  fake::PersistValues();
  CHECK_FALSE(fake::GetPersisted("/Tunables/fixed"));
  CHECK(fake::GetPersisted("/Tunables/robust") == fake::EncodeDouble(5.0));  // untouched
}

TEST_CASE_METHOD(DashboardFixture,
                 "The restore drain keeps the other topics' Driver Station writes",
                 "[prosdk][dashboard][tunable][persist]") {
  tunables::TunableDouble other{1.0};
  REQUIRE(tunables::Publish("other", other));
  tunables::TunableDouble robust{1.0, [] {
    tunables::TunableConfig config;
    config.robust = true;
    return config;
  }()};
  REQUIRE(tunables::Publish("robust", robust));
  fake::PushWrite("/Tunables/other", VT(dash::ValueType::kDouble), fake::EncodeDouble(3.0));
  fake::PushWrite("/Tunables/robust", VT(dash::ValueType::kDouble), fake::EncodeDouble(4.0));

  StoreDouble("/Tunables/arm/gain", 7.5);
  int onTune = 0;
  Arm arm{&onTune};
  CHECK(arm.gainSeenInConstructor == 7.5);
  // drained with the restore, applied by the next update as usual
  CHECK(other.Get() == 1.0);
  CHECK(robust.Get() == 1.0);
  tunables::TunableRegistry::Update();
  CHECK(other.Get() == 3.0);
  CHECK(robust.Get() == 4.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(other) == 1);
  CHECK(fake::Rejects().empty());
  CHECK(onTune == 1);
}

TEST_CASE_METHOD(DashboardFixture,
                 "Persistent tunable published before the store is ready restores when published",
                 "[prosdk][dashboard][tunable][persist]") {
  StoreDouble("/Tunables/early", 6.5);
  StoreDouble("/Tunables/late", 8.5);
  fake::SetReady(false);
  int onTune = 0;
  tunables::TunableDouble early{1.0, PersistentConfig(&onTune)};
  tunables::TunableDouble late{1.0, PersistentConfig(&onTune)};
  REQUIRE(tunables::Publish("early", early));
  REQUIRE(tunables::Publish("late", late));
  tunables::TunableRegistry::Update();
  CHECK(early.Get() == 1.0);
  CHECK(fake::LiveTopics() == 0);

  SECTION("by the pre-cycle flush, before the robot loop") {
    fake::SetReady(true);
    dash::FlushPending();
  }
  SECTION("by the registry update") {
    fake::SetReady(true);
    tunables::TunableRegistry::Update();
  }
  CHECK(early.Get() == 6.5);
  CHECK(late.Get() == 8.5);
  CHECK(onTune == 2);
  CHECK(tunables::TunableRegistry::GetTuneRevision(early) == 1);
  CHECK(fake::GetDouble("/Tunables/early") == 6.5);
  CHECK((fake::Find("/Tunables/early")->flags & dash::kFlagPersistent) != 0);
  tunables::TunableRegistry::Update();
  CHECK(onTune == 2);
  CHECK(fake::Rejects().empty());
}

TEST_CASE_METHOD(DashboardFixture,
                 "Persistent tunable published again keeps the value tuned in this run",
                 "[prosdk][dashboard][tunable][persist]") {
  StoreDouble("/Tunables/arm/gain", 7.5);
  int onTune = 0;
  {
    Arm arm{&onTune};
    CHECK(arm.gain.Get() == 7.5);
    // tuned while running: core does not store it yet (it saves only while idle)
    fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble),
                    fake::EncodeDouble(9.0));
    DashboardFixture::Cycle();
    CHECK(arm.gain.Get() == 9.0);

    // republished: core restores its older record on the new topic; the run's value wins
    tunables::Remove("arm/gain");
    REQUIRE(tunables::Publish("arm/gain", arm.gain));
    CHECK(arm.gain.Get() == 9.0);
    CHECK(fake::GetDouble("/Tunables/arm/gain") == 9.0);
    tunables::TunableRegistry::Update();
    CHECK(arm.gain.Get() == 9.0);
  }
  // a new object on the same path in the same run gets it too
  Arm again{&onTune};
  CHECK(again.gainSeenInConstructor == 9.0);
  // core's record was not refused (refusing would erase it)
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(7.5));
  CHECK(fake::Rejects().empty());
  CHECK(fake::RestoreErases() == 0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "Persistent tunable republished with its own last value is not tuned",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  tunables::TunableDouble value{1.0, PersistentConfig(&onTune)};
  REQUIRE(tunables::Publish("value", value));
  value.Set(2.0);
  tunables::TunableRegistry::Update();
  fake::PersistValues();  // core stored the robot's value
  value.Set(3.0);
  tunables::TunableRegistry::Update();

  // the last value set on the path is the tunable's own: nothing to apply, no onTune
  tunables::Remove("value");
  REQUIRE(tunables::Publish("value", value));
  CHECK(value.Get() == 3.0);
  CHECK(onTune == 0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);
  CHECK(fake::GetPersisted("/Tunables/value") == fake::EncodeDouble(2.0));  // not dropped
}

TEST_CASE_METHOD(DashboardFixture,
                 "Persistent tunable published again gets the last value set on its path",
                 "[prosdk][dashboard][tunable][persist]") {
  StoreDouble("/Tunables/arm/gain", 7.5);
  int onTune = 0;
  {
    Arm arm{&onTune};
    CHECK(arm.gain.Get() == 7.5);
    fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble),
                    fake::EncodeDouble(9.0));
    DashboardFixture::Cycle();
    CHECK(arm.gain.Get() == 9.0);
    // the robot program sets it after the tune; the update publishes it (core's topic value)
    arm.gain.Set(4.0);
    tunables::TunableRegistry::Update();
    CHECK(fake::GetDouble("/Tunables/arm/gain") == 4.0);
    CHECK(onTune == 2);
  }
  // the next OpMode's object, before core saved (2 s debounce, idle only): the robot's value,
  // not the earlier tune and not core's older record
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(7.5));
  Arm again{&onTune};
  CHECK(again.gainSeenInConstructor == 4.0);
  CHECK(onTune == 3);
  CHECK(tunables::TunableRegistry::GetTuneRevision(again.gain) == 1);
  CHECK(fake::GetDouble("/Tunables/arm/gain") == 4.0);
  // what core then stores, i.e. what a reboot restores, is the same value
  fake::PersistValues();
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(4.0));
  CHECK(fake::Rejects().empty());
  CHECK(fake::RestoreErases() == 0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "An always-get persistent tunable remembers the value its update publishes",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  auto config = PersistentConfig(&onTune);
  config.polling = tunables::TunableConfig::Polling::kAlwaysGet;
  {
    tunables::TunableDouble value{1.0, config};
    REQUIRE(tunables::Publish("value", value));
    value.Set(6.0);
    tunables::TunableRegistry::Update();
    CHECK(fake::GetDouble("/Tunables/value") == 6.0);
  }
  tunables::TunableDouble next{1.0, config};
  REQUIRE(tunables::Publish("value", next));
  CHECK(next.Get() == 6.0);
  CHECK(onTune == 1);
}

TEST_CASE_METHOD(DashboardFixture,
                 "A Set() the update has not published yet is not on the path",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  tunables::TunableDouble value{1.0, PersistentConfig(&onTune)};
  REQUIRE(tunables::Publish("value", value));
  value.Set(2.0);
  tunables::TunableRegistry::Update();
  // removed before an update published it: core's topic still holds 2.0, so does the republish
  value.Set(3.0);
  tunables::Remove("value");
  REQUIRE(tunables::Publish("value", value));
  CHECK(value.Get() == 2.0);
  CHECK(onTune == 1);
  fake::PersistValues();
  CHECK(fake::GetPersisted("/Tunables/value") == fake::EncodeDouble(2.0));
}

TEST_CASE_METHOD(DashboardFixture, "ClearPersistedTunables erases the stored values",
                 "[prosdk][dashboard][tunable][persist]") {
  CHECK(dash::PersistenceSupported());
  StoreDouble("/Tunables/arm/gain", 7.5);
  CHECK(prosdk::platform::ClearPersistedTunables());
  CHECK_FALSE(fake::GetPersisted("/Tunables/arm/gain"));
  int onTune = 0;
  Arm arm{&onTune};
  CHECK(arm.gainSeenInConstructor == 1.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "Published again after ClearPersistedTunables, a tunable starts from its code",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  {
    Arm arm{&onTune};
    fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble),
                    fake::EncodeDouble(9.0));
    DashboardFixture::Cycle();
    CHECK(arm.gain.Get() == 9.0);
    fake::PersistValues();
    CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(9.0));
    CHECK(prosdk::platform::ClearPersistedTunables());
    CHECK_FALSE(fake::GetPersisted("/Tunables/arm/gain"));
  }
  // the next OpMode's object: the value tuned before the clear does not come back
  Arm again{&onTune};
  CHECK(again.gainSeenInConstructor == 1.0);
  CHECK(onTune == 1);
  CHECK(fake::GetDouble("/Tunables/arm/gain") == 1.0);
  fake::PersistValues();
  CHECK(fake::GetPersisted("/Tunables/arm/gain") != fake::EncodeDouble(9.0));

  // a tune after the clear is remembered for the next republish again
  fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble), fake::EncodeDouble(4.0));
  DashboardFixture::Cycle();
  tunables::Remove("arm/gain");
  REQUIRE(tunables::Publish("arm/gain", again.gain));
  CHECK(again.gain.Get() == 4.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "A value set after ClearPersistedTunables is what the next object gets",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  {
    Arm arm{&onTune};
    fake::PushWrite("/Tunables/arm/gain", VT(dash::ValueType::kDouble),
                    fake::EncodeDouble(9.0));
    DashboardFixture::Cycle();
    CHECK(arm.gain.Get() == 9.0);
    CHECK(prosdk::platform::ClearPersistedTunables());
    // core stores a topic again only once its value changes
    fake::PersistValues();
    CHECK_FALSE(fake::GetPersisted("/Tunables/arm/gain"));
    // set after the clear (the update that publishes it sees the clear first), never republished
    // before the object goes away
    arm.gain.Set(5.0);
    tunables::TunableRegistry::Update();
    CHECK(fake::GetDouble("/Tunables/arm/gain") == 5.0);
  }
  Arm again{&onTune};
  CHECK(again.gainSeenInConstructor == 5.0);
  CHECK(onTune == 2);
  fake::PersistValues();
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(5.0));
}

TEST_CASE_METHOD(DashboardFixture, "A stored value equal to the code value is not a tune",
                 "[prosdk][dashboard][tunable][persist]") {
  StoreDouble("/Tunables/arm/gain", 1.0);
  int onTune = 0;
  Arm arm{&onTune};
  CHECK(arm.gainSeenInConstructor == 1.0);
  CHECK(onTune == 0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(arm.gain) == 0);
  // dropped, not refused: the record stays
  CHECK(fake::GetPersisted("/Tunables/arm/gain") == fake::EncodeDouble(1.0));
  CHECK(fake::Rejects().empty());
  CHECK(fake::RestoreErases() == 0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "A stored value that does not decode is erased when a waiting topic publishes",
                 "[prosdk][dashboard][tunable][persist]") {
  const prosdk::math::Translation2d initial{1.25_m, 2.5_m};
  fake::SetPersisted("/Tunables/target", VT(dash::ValueType::kRaw), "struct:Translation2d",
                     std::vector<uint8_t>{1, 2, 3});
  fake::SetReady(false);
  int onTune = 0;
  tunables::Tunable<prosdk::math::Translation2d> target{PersistentConfig(&onTune), initial};
  REQUIRE(tunables::Publish("target", target));
  CHECK(fake::RestoreErases() == 0);

  fake::SetReady(true);
  dash::FlushPending();
  REQUIRE(fake::Find("/Tunables/target"));
  CHECK(target.Get().X() == initial.X());
  CHECK(onTune == 0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(target) == 0);
  CHECK(fake::RestoreErases() == 1);
  CHECK_FALSE(fake::GetPersisted("/Tunables/target"));
  CHECK(fake::Rejects().empty());
}

TEST_CASE_METHOD(DashboardFixture,
                 "A tunable of another type on a republished path takes core's record",
                 "[prosdk][dashboard][tunable][persist]") {
  int onTune = 0;
  {
    tunables::TunableDouble value{1.0, PersistentConfig(&onTune)};
    REQUIRE(tunables::Publish("value", value));
    fake::PushWrite("/Tunables/value", VT(dash::ValueType::kDouble), fake::EncodeDouble(9.0));
    DashboardFixture::Cycle();
    CHECK(value.Get() == 9.0);
  }
  // the remembered double does not apply to an int tunable on the same path
  fake::SetPersisted("/Tunables/value", VT(dash::ValueType::kInt), "int", fake::EncodeInt(3));
  tunables::TunableInt64 count{1, PersistentConfig(&onTune)};
  REQUIRE(tunables::Publish("value", count));
  CHECK(count.Get() == 3);
  CHECK(onTune == 2);
  CHECK(fake::Rejects().empty());
  CHECK(fake::RestoreErases() == 0);
}
