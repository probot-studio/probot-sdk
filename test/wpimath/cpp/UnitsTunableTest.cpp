// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/tunables/ComplexTunable.hpp"
#include "prosdk/tunables/MockTunableBackend.hpp"
#include "prosdk/tunables/TunableBackend.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/TunableTable.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/units/tunable.hpp"

using namespace prosdk;
using namespace prosdk::tunables;

static_assert(
    std::same_as<
        decltype(std::declval<prosdk::tunables::Tunable<prosdk::units::meter_t>&>()
                     .GetInnerTunable()),
        prosdk::tunables::Tunable<double>&>);
static_assert(std::same_as<decltype(std::declval<const prosdk::tunables::Tunable<
                                        prosdk::units::meter_t>&>()
                                        .GetInnerTunable()),
                           const prosdk::tunables::Tunable<double>&>);
static_assert(
    std::same_as<decltype(prosdk::tunables::TunableRegistry::GetTuneRevision(
                     std::declval<const prosdk::tunables::Tunable<
                         prosdk::units::meter_t>&>())),
                 uint64_t>);

namespace {
class RecordingDirtyBackend : public TunableBackend {
 public:
  bool Publish(std::string_view, uint32_t, detail::TunableBase&,
               const TunableConfig*, detail::TunableTypeValue) override {
    return true;
  }

  void MarkDirty(uint32_t uid) override { dirtyUids.emplace_back(uid); }

  void Remove(std::string_view) override {}

  std::vector<PublishedTunable> RemovePrefix(std::string_view) override {
    return {};
  }

  void UnregisterTunable(uint32_t) override {}

  void Update() override {}

  std::vector<uint32_t> dirtyUids;
};

struct UnitsTunableTest {
  UnitsTunableTest() { TunableRegistry::RegisterBackend("", backend); }

  ~UnitsTunableTest() { TunableRegistry::Reset(); }

  std::shared_ptr<MockTunableBackend> backend =
      std::make_shared<MockTunableBackend>();
};

struct UnitMemberComplex : public ComplexTunable {
  prosdk::units::meter_t distance{1.0};

  void PublishTunable(TunableTable& table) override {
    table.Publish("distance", this, &UnitMemberComplex::distance);
  }
};
}  // namespace

TEST_CASE_METHOD(UnitsTunableTest, "UnitsTunableTest RevisionForUnitTunable",
                 "[tunable]") {
  prosdk::tunables::Tunable<prosdk::units::meter_t> distance{
      prosdk::units::meter_t{1.0}};
  const auto& constDistance = distance;

  auto revision = prosdk::tunables::TunableRegistry::GetTuneRevision(distance);
  CHECK(revision == 0);
  CHECK(prosdk::tunables::TunableRegistry::GetTuneRevision(constDistance) == 0);

  distance = prosdk::units::meter_t{2.0};
  CHECK(distance.Get().to<double>() == 2.0);
  CHECK(prosdk::tunables::TunableRegistry::GetTuneRevision(distance) == 0);

  Publish("distance", distance);
  backend->SetDouble("/distance", 3.0);
  TunableRegistry::Update();

  CHECK(distance.Get().to<double>() == 3.0);
  CHECK(prosdk::tunables::TunableRegistry::GetTuneRevision(distance) == 1);
  CHECK(prosdk::tunables::TunableRegistry::GetTuneRevision(constDistance) == 1);
}

TEST_CASE_METHOD(UnitsTunableTest,
                 "UnitsTunableTest MemberRemoteTuneMarksDirty", "[tunable]") {
  auto recordingBackend = std::make_shared<RecordingDirtyBackend>();
  TunableRegistry::RegisterBackend("/recording", recordingBackend);

  UnitMemberComplex complex;
  Publish("unit", complex);
  auto uid = backend->GetUid("/unit/distance");
  REQUIRE(uid);
  CHECK(TunableRegistry::GetTuneRevision(complex) == 0);

  backend->SetDouble("/unit/distance", 2.0);
  TunableRegistry::Update();

  CHECK(complex.distance.to<double>() == 2.0);
  CHECK(TunableRegistry::GetTuneRevision(complex) == 1);
  CHECK(std::find(recordingBackend->dirtyUids.begin(),
                  recordingBackend->dirtyUids.end(),
                  *uid) != recordingBackend->dirtyUids.end());
}
