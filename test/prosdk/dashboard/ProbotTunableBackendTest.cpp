// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// ProbotTunableBackend on the fake core store. The cases mirror upstream
// NetworkTablesTunableBackendTest.cpp (WPILib 2027); a Driver Station tune is a write queued in
// the store (fake::PushWrite) instead of a NetworkTables publisher, and a robust tunable is one
// topic instead of "<path>/value" + "<path>/tune".

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/math/controller/PIDController.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/geometry/proto/Translation2dProto.hpp"
#include "prosdk/math/geometry/struct/Translation2dStruct.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/tunables/ComplexTunable.hpp"
#include "prosdk/tunables/Selectable.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableTable.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/util/json.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/struct/Struct.hpp"

using dashboard_test::CountOf;
using dashboard_test::DashboardFixture;
using dashboard_test::HasWarning;
using prosdk::backend::ProbotTunableBackendTestAccess;
namespace dash = prosdk::platform::dashboard;
namespace tunables = prosdk::tunables;

struct ThrowingPackStruct {
  int16_t value;
};

struct ThrowingPackStructState {
  static inline bool removeDuringPack = false;
  static inline bool throwDuringPack = false;
  static inline bool throwDuringUnpack = false;
};

template <>
struct prosdk::util::Struct<ThrowingPackStruct> {
  static constexpr std::string_view GetTypeName() { return "ThrowingPackStruct"; }
  static constexpr size_t GetSize() { return 2; }
  static constexpr std::string_view GetSchema() { return "int16 value"; }
  static ThrowingPackStruct Unpack(std::span<const uint8_t> data) {
    if (ThrowingPackStructState::throwDuringUnpack) {
      ThrowingPackStructState::throwDuringUnpack = false;
      throw std::runtime_error{"broken unpack"};
    }
    return {prosdk::util::UnpackStruct<int16_t, 0>(data)};
  }
  static void Pack(std::span<uint8_t> data, ThrowingPackStruct value) {
    if (ThrowingPackStructState::removeDuringPack) {
      ThrowingPackStructState::removeDuringPack = false;
      tunables::Remove("removeMe");
    }
    if (ThrowingPackStructState::throwDuringPack) {
      throw std::runtime_error{"broken pack"};
    }
    prosdk::util::PackStruct<0>(data, value.value);
  }
};

namespace {

uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}

tunables::TunableConfig RobustConfig() {
  tunables::TunableConfig config;
  config.robust = true;
  return config;
}

void Tune(std::string_view path, dash::ValueType vt, const std::vector<uint8_t>& data) {
  fake::PushWrite(path, VT(vt), data);
}

template <typename T>
std::vector<uint8_t> Le(std::initializer_list<T> values) {
  std::vector<uint8_t> out;
  for (T v : values) {
    auto p = reinterpret_cast<const uint8_t*>(&v);
    out.insert(out.end(), p, p + sizeof(T));
  }
  return out;
}

class MutatingComplexTunable final : public tunables::ComplexTunable {
 public:
  explicit MutatingComplexTunable(tunables::TunableDouble& published) : m_published{published} {}
  std::string_view GetTunableType() const override { return "Mutating"; }
  void PublishTunable(tunables::TunableTable&) override {}
  void UpdateTunable() const override {
    if (m_updates++ != 0) {
      return;
    }
    tunables::Remove("removeMe");
    tunables::Publish("publishedFromComplex", m_published);
  }
  int GetUpdates() const { return m_updates; }

 private:
  tunables::TunableDouble& m_published;
  mutable int m_updates = 0;
};

class CountingComplexTunable final : public tunables::ComplexTunable {
 public:
  std::string_view GetTunableType() const override { return "Counting"; }
  void PublishTunable(tunables::TunableTable&) override {}
  void UpdateTunable() const override { ++m_updates; }
  int GetUpdates() const { return m_updates; }

 private:
  mutable int m_updates = 0;
};

class StructMemberComplexTunable final : public tunables::ComplexTunable {
 public:
  explicit StructMemberComplexTunable(const tunables::TunableConfig& config) : m_config{config} {}
  std::string_view GetTunableType() const override { return "StructMember"; }
  void PublishTunable(tunables::TunableTable& table) override {
    table.Publish("member", this, &StructMemberComplexTunable::value, m_config);
  }
  ThrowingPackStruct value{1};

 private:
  tunables::TunableConfig m_config;
};

}  // namespace

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes and tunes scalar data types",
                 "[prosdk][dashboard][tunable]") {
  auto config = RobustConfig();
  tunables::TunableBool boolean{true, config};
  tunables::TunableInt32 intValue{1, config};
  tunables::TunableInt64 longValue{2, config};
  tunables::TunableFloat floatValue{3.25f, config};
  tunables::TunableDouble doubleValue{4.5, config};
  tunables::TunableString stringValue{"ready", config};

  tunables::Publish("boolean", boolean);
  tunables::Publish("int", intValue);
  tunables::Publish("long", longValue);
  tunables::Publish("float", floatValue);
  tunables::Publish("double", doubleValue);
  tunables::Publish("string", stringValue);

  CHECK(fake::GetBoolean("/Tunables/boolean") == true);
  CHECK(fake::GetInt("/Tunables/int") == 1);
  CHECK(fake::GetInt("/Tunables/long") == 2);
  CHECK(fake::GetFloat("/Tunables/float") == 3.25f);
  CHECK(fake::GetDouble("/Tunables/double") == 4.5);
  CHECK(fake::GetString("/Tunables/string") == "ready");
  CHECK(fake::Find("/Tunables/int")->typeString == "int");
  CHECK(fake::Find("/Tunables/int")->valueType == VT(dash::ValueType::kInt));
  CHECK(fake::Find("/Tunables/float")->typeString == "float");
  CHECK(fake::Find("/Tunables/string")->typeString == "string");

  Tune("/Tunables/boolean", dash::ValueType::kBoolean, fake::EncodeBoolean(false));
  Tune("/Tunables/int", dash::ValueType::kInt, fake::EncodeInt(11));
  Tune("/Tunables/long", dash::ValueType::kInt, fake::EncodeInt(12));
  Tune("/Tunables/float", dash::ValueType::kFloat, fake::EncodeFloat(13.5f));
  Tune("/Tunables/double", dash::ValueType::kDouble, fake::EncodeDouble(14.75));
  Tune("/Tunables/string", dash::ValueType::kString, fake::EncodeString("tuned"));
  tunables::TunableRegistry::Update();

  CHECK_FALSE(boolean.Get());
  CHECK(11 == intValue.Get());
  CHECK(12 == longValue.Get());
  CHECK(13.5f == floatValue.Get());
  CHECK(14.75 == doubleValue.Get());
  CHECK("tuned" == stringValue.Get());
  CHECK(fake::GetDouble("/Tunables/double") == 14.75);
  CHECK(fake::Rejects().empty());
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend publishes and tunes array and raw data types",
                 "[prosdk][dashboard][tunable]") {
  auto config = RobustConfig();
  tunables::TunableRaw raw{std::vector<uint8_t>{1, 2}, config};
  tunables::TunableBoolVector booleans{std::vector<bool>{true, false}, config};
  tunables::TunableInt32Vector ints{std::vector<int32_t>{3, 4}, config};
  tunables::TunableInt64Vector longs{std::vector<int64_t>{5, 6}, config};
  tunables::TunableFloatVector floats{std::vector<float>{7.25f, 8.5f}, config};
  tunables::TunableDoubleVector doubles{std::vector<double>{9.25, 10.5}, config};
  tunables::Tunable<std::vector<std::string>> strings{std::vector<std::string>{"a", "b"},
                                                      config};

  tunables::Publish("raw", raw);
  tunables::Publish("booleans", booleans);
  tunables::Publish("ints", ints);
  tunables::Publish("longs", longs);
  tunables::Publish("floats", floats);
  tunables::Publish("doubles", doubles);
  tunables::Publish("strings", strings);

  CHECK(fake::GetRaw("/Tunables/raw") == std::vector<uint8_t>{1, 2});
  CHECK(fake::GetBooleanArray("/Tunables/booleans") == std::vector<bool>{true, false});
  CHECK(fake::GetIntArray("/Tunables/ints") == std::vector<int64_t>{3, 4});
  CHECK(fake::GetIntArray("/Tunables/longs") == std::vector<int64_t>{5, 6});
  CHECK(fake::GetFloatArray("/Tunables/floats") == std::vector<float>{7.25f, 8.5f});
  CHECK(fake::GetDoubleArray("/Tunables/doubles") == std::vector<double>{9.25, 10.5});
  CHECK(fake::GetStringArray("/Tunables/strings") == std::vector<std::string>{"a", "b"});
  CHECK(fake::Find("/Tunables/raw")->typeString == "raw");
  CHECK(fake::Find("/Tunables/booleans")->typeString == "boolean[]");
  CHECK(fake::Find("/Tunables/ints")->typeString == "int[]");
  CHECK(fake::Find("/Tunables/floats")->typeString == "float[]");
  CHECK(fake::Find("/Tunables/doubles")->typeString == "double[]");
  CHECK(fake::Find("/Tunables/strings")->typeString == "string[]");

  const std::vector<std::string> tunedStrings{"c", "d"};
  Tune("/Tunables/raw", dash::ValueType::kRaw, {21, 22});
  Tune("/Tunables/booleans", dash::ValueType::kBooleanArray, {0, 1});
  Tune("/Tunables/ints", dash::ValueType::kIntArray, Le<int64_t>({23, 24}));
  Tune("/Tunables/longs", dash::ValueType::kIntArray, Le<int64_t>({25, 26}));
  Tune("/Tunables/floats", dash::ValueType::kFloatArray, Le<float>({27.25f, 28.5f}));
  Tune("/Tunables/doubles", dash::ValueType::kDoubleArray, Le<double>({29.25, 30.5}));
  Tune("/Tunables/strings", dash::ValueType::kStringArray, fake::EncodeStringArray(tunedStrings));
  tunables::TunableRegistry::Update();

  CHECK((std::vector<uint8_t>{21, 22}) == raw.Get());
  CHECK((std::vector<bool>{false, true}) == booleans.Get());
  CHECK((std::vector<int32_t>{23, 24}) == ints.Get());
  CHECK((std::vector<int64_t>{25, 26}) == longs.Get());
  CHECK((std::vector<float>{27.25f, 28.5f}) == floats.Get());
  CHECK((std::vector<double>{29.25, 30.5}) == doubles.Get());
  CHECK(tunedStrings == strings.Get());
  CHECK(fake::GetStringArray("/Tunables/strings") == tunedStrings);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes and tunes double",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableConfig config;
  tunables::TunableDouble value{1.0, config};
  tunables::Publish("foo", value);

  auto topic = fake::Find("/Tunables/foo");
  REQUIRE(topic);
  CHECK(fake::GetDouble("/Tunables/foo") == 1.0);
  CHECK(topic->flags == (dash::kFlagTunable | dash::kFlagMutable));
  CHECK(topic->properties == R"({"mutable":true})");

  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 2.0);
  CHECK(fake::GetDouble("/Tunables/foo") == 2.0);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend tunables without config are mutable",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0};
  tunables::Publish("foo", value);

  CHECK(fake::Find("/Tunables/foo")->properties == R"({"mutable":true})");
  CHECK(fake::Find("/Tunables/foo")->flags == (dash::kFlagTunable | dash::kFlagMutable));

  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 2.0);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend tune revision tracks remote updates",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0};
  tunables::Publish("revision", value);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);

  Tune("/Tunables/revision", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 2.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 1);
  tunables::TunableRegistry::Update();
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 1);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend tune revision ignores local publishes",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0, RobustConfig()};
  tunables::Publish("localRevision", value);

  value.Set(2.0);
  tunables::TunableRegistry::Update();
  CHECK(fake::GetDouble("/Tunables/localRevision") == 2.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);

  Tune("/Tunables/localRevision", dash::ValueType::kDouble, fake::EncodeDouble(3.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 3.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 1);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes a robust tunable as one topic",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0, RobustConfig()};
  tunables::Publish("foo", value);

  auto topic = fake::Find("/Tunables/foo");
  REQUIRE(topic);
  CHECK(topic->flags == (dash::kFlagTunable | dash::kFlagMutable | dash::kFlagRobust));
  CHECK(topic->properties == R"({"mutable":true,"robust":true})");
  CHECK_FALSE(fake::Find("/Tunables/foo/value"));
  CHECK_FALSE(fake::Find("/Tunables/foo/tune"));
  CHECK(fake::LiveTopics() == 1);
  CHECK(fake::GetDouble("/Tunables/foo") == 1.0);

  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 2.0);
  CHECK(fake::GetDouble("/Tunables/foo") == 2.0);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend applies the last of several writes",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0};
  tunables::Publish("foo", value);
  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  dash::DrainWrites();
  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(3.0));
  dash::DrainWrites();
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 3.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 1);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend selectable publishes option changes",
                 "[prosdk][dashboard][tunable]") {
  tunables::Selectable<int> chooser;
  chooser.Add("one", 1);
  tunables::Publish("chooser", chooser);

  CHECK(fake::GetString("/Tunables/chooser/.type") == "Selectable");
  CHECK(fake::GetStringArray("/Tunables/chooser/options") == std::vector<std::string>{"one"});

  chooser.Add("two", 2);
  tunables::TunableRegistry::Update();
  CHECK(fake::GetStringArray("/Tunables/chooser/options") ==
        (std::vector<std::string>{"one", "two"}));

  chooser.Add("one", 11);
  tunables::TunableRegistry::Update();
  CHECK(fake::GetStringArray("/Tunables/chooser/options") ==
        (std::vector<std::string>{"two", "one"}));

  chooser.Clear();
  tunables::TunableRegistry::Update();
  CHECK(fake::GetStringArray("/Tunables/chooser/options") == std::vector<std::string>{});
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend selectable has the Driver Station widget layout",
                 "[prosdk][dashboard][tunable]") {
  tunables::Selectable<int> chooser;
  chooser.AddDefault("one", 1);
  chooser.Add("two", 2);
  tunables::Publish("auto", chooser);

  auto type = fake::Find("/Tunables/auto/.type");
  REQUIRE(type);
  CHECK(type->valueType == VT(dash::ValueType::kString));
  CHECK(type->flags == dash::kFlagTunable);  // read-only
  CHECK(type->properties == R"({"mutable":false})");
  auto selected = fake::Find("/Tunables/auto/selected");
  REQUIRE(selected);
  CHECK(selected->valueType == VT(dash::ValueType::kString));
  CHECK((selected->flags & (dash::kFlagTunable | dash::kFlagMutable)) ==
        (dash::kFlagTunable | dash::kFlagMutable));
  auto options = fake::Find("/Tunables/auto/options");
  REQUIRE(options);
  CHECK(options->valueType == VT(dash::ValueType::kStringArray));
  CHECK(fake::GetString("/Tunables/auto/default") == "one");
  CHECK(1 == chooser.GetSelected());

  Tune("/Tunables/auto/selected", dash::ValueType::kString, fake::EncodeString("two"));
  DashboardFixture::Cycle();
  CHECK(2 == chooser.GetSelected());
  CHECK(fake::GetString("/Tunables/auto/selected") == "two");

  // the .type topic is not writable
  Tune("/Tunables/auto/.type", dash::ValueType::kString, fake::EncodeString("Other"));
  DashboardFixture::Cycle();
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].first == "/Tunables/auto/.type");
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kUnknownId));
  CHECK(fake::GetString("/Tunables/auto/.type") == "Selectable");
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes a PIDController",
                 "[prosdk][dashboard][tunable]") {
  prosdk::math::PIDController pid{1.0, 0.0, 0.25};
  REQUIRE(tunables::Publish("pid", pid));

  CHECK(fake::GetString("/Tunables/pid/.type") == "PIDController");
  size_t children = 0;
  for (std::string_view name : {"p", "i", "d"}) {
    if (fake::Find(std::string{"/Tunables/pid/"} + std::string{name})) {
      ++children;
    }
  }
  CHECK(children == 3);
  CHECK(fake::GetDouble("/Tunables/pid/p") == 1.0);
  CHECK(fake::GetDouble("/Tunables/pid/d") == 0.25);

  Tune("/Tunables/pid/p", dash::ValueType::kDouble, fake::EncodeDouble(2.5));
  DashboardFixture::Cycle();
  CHECK(pid.GetP() == 2.5);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes and tunes struct",
                 "[prosdk][dashboard][tunable]") {
  const prosdk::math::Translation2d initial{1.25_m, 2.5_m};
  const prosdk::math::Translation2d tuned{3.75_m, 4.5_m};
  tunables::Tunable<prosdk::math::Translation2d> value{RobustConfig(), initial};
  tunables::Publish("translation", value);

  auto topic = fake::Find("/Tunables/translation");
  REQUIRE(topic);
  CHECK(topic->typeString == "struct:Translation2d");
  CHECK(topic->valueType == VT(dash::ValueType::kRaw));
  auto logged = prosdk::util::UnpackStruct<prosdk::math::Translation2d>(
      *fake::GetRaw("/Tunables/translation"));
  CHECK(initial.X() == logged.X());
  CHECK(initial.Y() == logged.Y());
  REQUIRE(fake::FindSchema("struct:Translation2d"));
  CHECK(fake::FindSchema("struct:Translation2d")->type == "structschema");

  std::vector<uint8_t> data(prosdk::util::GetStructSize<prosdk::math::Translation2d>());
  prosdk::util::PackStruct(std::span<uint8_t>{data}, tuned);
  Tune("/Tunables/translation", dash::ValueType::kRaw, data);
  tunables::TunableRegistry::Update();

  CHECK(tuned.X() == value.Get().X());
  CHECK(tuned.Y() == value.Get().Y());
  CHECK(fake::GetRaw("/Tunables/translation") == data);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend rejects invalid struct tune payloads",
                 "[prosdk][dashboard][tunable]") {
  const prosdk::math::Translation2d initial{1.25_m, 2.5_m};
  auto config = RobustConfig();
  int calls = 0;
  config.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) { ++calls; };
  tunables::Tunable<prosdk::math::Translation2d> value{config, initial};
  tunables::Publish("translation", value);
  int sets = fake::Find("/Tunables/translation")->sets;

  Tune("/Tunables/translation", dash::ValueType::kRaw, {1, 2, 3});
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  std::vector<uint8_t> oversized(prosdk::util::GetStructSize<prosdk::math::Translation2d>() + 1);
  Tune("/Tunables/translation", dash::ValueType::kRaw, oversized);
  CHECK_NOTHROW(tunables::TunableRegistry::Update());

  CHECK(initial.X() == value.Get().X());
  CHECK(initial.Y() == value.Get().Y());
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);
  CHECK(0 == calls);
  CHECK(fake::Find("/Tunables/translation")->sets == sets);
  REQUIRE(fake::Rejects().size() == 2);
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kDecodeError));
  CHECK(fake::Rejects()[1].second == static_cast<uint8_t>(dash::WriteStatus::kDecodeError));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend publishes and tunes protobuf",
                 "[prosdk][dashboard][tunable]") {
  const prosdk::math::Translation2d initial{5.25_m, 6.5_m};
  const prosdk::math::Translation2d tuned{7.75_m, 8.5_m};
  // geometry2d.proto's descriptor is ~1.3 KB: within core's default PROBOT_DASH_MAX_SCHEMA_BYTES
  // (4096, core R13), so core's defaults store it
  tunables::detail::TunableProtobuf<prosdk::math::Translation2d> value{RobustConfig(), initial};
  tunables::Publish("translation", value);

  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> message;
  auto topic = fake::Find("/Tunables/translation");
  REQUIRE(topic);
  CHECK(topic->typeString == message.GetTypeString());
  auto logged = message.Unpack(*fake::GetRaw("/Tunables/translation"));
  REQUIRE(logged.has_value());
  CHECK(initial.X() == logged->X());
  size_t descriptors = 0;
  message.ForEachProtobufDescriptor(
      [&](std::string_view filename) {
        auto schema = fake::FindSchema(filename);
        descriptors += schema && schema->type == "proto:FileDescriptorProto" ? 1 : 0;
        return true;
      },
      [](std::string_view, std::span<const uint8_t>) {});
  CHECK(descriptors > 0);

  std::vector<uint8_t> data;
  REQUIRE(message.Pack(data, tuned));
  Tune("/Tunables/translation", dash::ValueType::kRaw, data);
  tunables::TunableRegistry::Update();
  CHECK(tuned.X() == value.Get().X());
  CHECK(tuned.Y() == value.Get().Y());
  logged = message.Unpack(*fake::GetRaw("/Tunables/translation"));
  REQUIRE(logged.has_value());
  CHECK(tuned.X() == logged->X());
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend rejects malformed protobuf tune payloads",
                 "[prosdk][dashboard][tunable]") {
  const prosdk::math::Translation2d initial{5.25_m, 6.5_m};
  const prosdk::math::Translation2d partial{9.25_m, 10.5_m};
  int calls = 0;
  auto config = RobustConfig();
  config.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) { ++calls; };
  tunables::detail::TunableProtobuf<prosdk::math::Translation2d> value{config, initial};
  tunables::Publish("malformedTranslation", value);

  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> message;
  std::vector<uint8_t> malformed;
  REQUIRE(message.Pack(malformed, partial));
  REQUIRE(malformed.size() > 10);
  malformed.resize(10);

  std::vector<std::string> warnings;
  tunables::TunableRegistry::SetReportWarning(
      [&](std::string_view msg) { warnings.emplace_back(msg); });
  Tune("/Tunables/malformedTranslation", dash::ValueType::kRaw, malformed);
  CHECK_NOTHROW(tunables::TunableRegistry::Update());

  CHECK(initial.X() == value.Get().X());
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);
  CHECK(0 == calls);
  CHECK(HasWarning(warnings, "/Tunables/malformedTranslation", "rejected protobuf tune payload"));
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kDecodeError));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend applies backend config options",
                 "[prosdk][dashboard][tunable]") {
  auto config = RobustConfig();
  config.typeString = "json";
  config.properties = prosdk::util::json::object();
  config.properties["min"] = 0;
  config.properties["max"] = 10;
  tunables::TunableString value{"1", config};
  tunables::Publish("configured", value);

  auto topic = fake::Find("/Tunables/configured");
  REQUIRE(topic);
  CHECK(topic->typeString == "json");
  CHECK(topic->properties == R"({"max":10,"min":0,"mutable":true,"robust":true})");
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend runs onTune for mutable remote updates",
                 "[prosdk][dashboard][tunable]") {
  int calls = 0;
  uint64_t callbackRevision = 0;
  tunables::TunableDouble* valuePtr = nullptr;
  auto config = RobustConfig();
  config.onTune = [&](tunables::detail::TunableBase& tunable, tunables::ComplexTunable*) {
    callbackRevision = tunables::TunableRegistry::GetTuneRevision(tunable);
    ++calls;
    valuePtr->Set(4.0);
  };
  tunables::TunableDouble value{1.0, config};
  valuePtr = &value;
  tunables::Publish("mutable", value);

  Tune("/Tunables/mutable", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(4.0 == value.Get());
  CHECK(1 == calls);
  CHECK(callbackRevision == 1);

  tunables::TunableRegistry::Update();
  CHECK(1 == calls);
  CHECK(fake::GetDouble("/Tunables/mutable") == 4.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend remote set callback can canonicalize the echo",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble* valuePtr = nullptr;
  auto config = RobustConfig();
  config.onRemoteSet = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) {
    if (valuePtr->Get() > 5.0) {
      valuePtr->Set(5.0);
    }
  };
  tunables::TunableDouble value{1.0, config};
  valuePtr = &value;
  tunables::Publish("clamped", value);

  Tune("/Tunables/clamped", dash::ValueType::kDouble, fake::EncodeDouble(10.0));
  tunables::TunableRegistry::Update();
  CHECK(5.0 == value.Get());
  CHECK(fake::GetDouble("/Tunables/clamped") == 5.0);
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 1);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend rejects struct tunes that throw during unpack",
                 "[prosdk][dashboard][tunable]") {
  auto standaloneConfig = RobustConfig();
  int standaloneCalls = 0;
  standaloneConfig.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) {
    ++standaloneCalls;
  };
  tunables::Tunable<ThrowingPackStruct> throwing{standaloneConfig, ThrowingPackStruct{1}};
  tunables::Publish("throwing", throwing);

  auto memberConfig = RobustConfig();
  int memberCalls = 0;
  memberConfig.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) {
    ++memberCalls;
  };
  StructMemberComplexTunable complex{memberConfig};
  tunables::Publish("complex", complex);
  CHECK(fake::Find("/Tunables/complex/member")->typeString == "struct:ThrowingPackStruct");

  std::vector<std::string> warnings;
  tunables::TunableRegistry::SetReportWarning(
      [&](std::string_view msg) { warnings.emplace_back(msg); });

  ThrowingPackStructState::throwDuringUnpack = true;
  Tune("/Tunables/throwing", dash::ValueType::kRaw, Le<int16_t>({2}));
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  CHECK(1 == throwing.Get().value);
  CHECK(0 == standaloneCalls);
  CHECK(HasWarning(warnings, "/Tunables/throwing", "rejected struct tune payload"));

  ThrowingPackStructState::throwDuringUnpack = true;
  Tune("/Tunables/complex/member", dash::ValueType::kRaw, Le<int16_t>({3}));
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  CHECK(1 == complex.value.value);
  CHECK(0 == memberCalls);
  CHECK(HasWarning(warnings, "/Tunables/complex/member", "rejected struct tune payload"));

  Tune("/Tunables/throwing", dash::ValueType::kRaw, Le<int16_t>({4}));
  Tune("/Tunables/complex/member", dash::ValueType::kRaw, Le<int16_t>({5}));
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  CHECK(4 == throwing.Get().value);
  CHECK(5 == complex.value.value);
  CHECK(1 == standaloneCalls);
  CHECK(1 == memberCalls);
  ThrowingPackStructState::throwDuringUnpack = false;
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend onTune can publish and remove tunables",
                 "[prosdk][dashboard][tunable]") {
  bool callbackRan = false;
  tunables::TunableDouble published{3.0};
  tunables::TunableDouble removeMe{4.0};
  tunables::Publish("removeMe", removeMe);

  auto config = RobustConfig();
  config.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) {
    if (callbackRan) {
      return;
    }
    callbackRan = true;
    tunables::Remove("removeMe");
    tunables::Publish("publishedFromOnTune", published);
  };
  tunables::TunableDouble value{1.0, config};
  tunables::Publish("mutable", value);

  Tune("/Tunables/mutable", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();

  CHECK(callbackRan);
  CHECK(2.0 == value.Get());
  CHECK(fake::GetDouble("/Tunables/publishedFromOnTune") == 3.0);
  CHECK_FALSE(fake::Find("/Tunables/removeMe"));

  tunables::TunableDouble replacement{5.0};
  CHECK_NOTHROW(tunables::Publish("removeMe", replacement));
  CHECK(fake::GetDouble("/Tunables/removeMe") == 5.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend failure during update applies deferred erases",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble removeMe{4.0};
  tunables::Publish("removeMe", removeMe);
  tunables::Tunable<ThrowingPackStruct> throwingStruct{ThrowingPackStruct{1}};
  tunables::Publish("throwingStruct", throwingStruct);

  ThrowingPackStructState::removeDuringPack = true;
  ThrowingPackStructState::throwDuringPack = true;
  throwingStruct.Set({2});

  std::vector<std::string> warnings;
  tunables::TunableRegistry::SetReportWarning(
      [&](std::string_view msg) { warnings.emplace_back(msg); });
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  ThrowingPackStructState::throwDuringPack = false;
  CHECK(HasWarning(warnings, "Dashboard tunable backend update failed", "broken pack"));
  CHECK(0 == ProbotTunableBackendTestAccess::GetUpdateDepth(*tunables));
  CHECK(0u == ProbotTunableBackendTestAccess::GetDeferredEraseCount(*tunables));

  tunables::TunableDouble replacement{5.0};
  CHECK_NOTHROW(tunables::Publish("removeMe", replacement));
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend initial network update failure rolls back the entry",
                 "[prosdk][dashboard][tunable]") {
  auto config = tunables::TunableConfig::AlwaysGet();
  tunables::Tunable<ThrowingPackStruct> throwingStruct{config, ThrowingPackStruct{1}};

  std::vector<std::string> warnings;
  tunables::TunableRegistry::SetReportWarning(
      [&](std::string_view msg) { warnings.emplace_back(msg); });

  ThrowingPackStructState::throwDuringPack = true;
  CHECK_NOTHROW(tunables::Publish("throwingStruct", throwingStruct));
  ThrowingPackStructState::throwDuringPack = false;
  CHECK(HasWarning(warnings, "/Tunables/throwingStruct", "failed during initial publish"));
  CHECK(0u == ProbotTunableBackendTestAccess::GetEntryCount(*tunables));
  CHECK(0u == ProbotTunableBackendTestAccess::GetUidPathCount(*tunables));
  CHECK(0u == ProbotTunableBackendTestAccess::GetAlwaysGetEntryCount(*tunables));
  CHECK(0u == ProbotTunableBackendTestAccess::GetTopicCount(*tunables));
  CHECK_FALSE(fake::Find("/Tunables/throwingStruct"));

  CHECK_NOTHROW(tunables::Publish("throwingStruct", throwingStruct));
  CHECK(1u == ProbotTunableBackendTestAccess::GetEntryCount(*tunables));
  CHECK(1u == ProbotTunableBackendTestAccess::GetUidPathCount(*tunables));
  CHECK(1u == ProbotTunableBackendTestAccess::GetAlwaysGetEntryCount(*tunables));
  CHECK(fake::Find("/Tunables/throwingStruct"));
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend complex update can publish and remove tunables",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble published{3.0};
  tunables::TunableDouble removeMe{4.0};
  tunables::Publish("removeMe", removeMe);

  MutatingComplexTunable complex{published};
  tunables::Publish("complex", complex);
  CountingComplexTunable after;
  tunables::Publish("z", after);
  CHECK(fake::GetString("/Tunables/complex/.type") == "Mutating");

  tunables::TunableRegistry::Update();
  CHECK(1 == complex.GetUpdates());
  CHECK(1 == after.GetUpdates());
  CHECK(fake::GetDouble("/Tunables/publishedFromComplex") == 3.0);

  tunables::TunableDouble replacement{5.0};
  CHECK_NOTHROW(tunables::Publish("removeMe", replacement));
  CHECK(fake::GetDouble("/Tunables/removeMe") == 5.0);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend programmatic changes queue dirty entries",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble changed{1.0};
  tunables::TunableDouble unchanged{2.0};
  tunables::TunableDouble alwaysGet{3.0, tunables::TunableConfig::AlwaysGet()};
  tunables::Publish("changed", changed);
  tunables::Publish("unchanged", unchanged);
  tunables::Publish("alwaysGet", alwaysGet);

  CHECK(0u == ProbotTunableBackendTestAccess::GetDirtyEntryCount(*tunables));
  CHECK(1u == ProbotTunableBackendTestAccess::GetAlwaysGetEntryCount(*tunables));

  changed.Set(4.0);
  changed.Set(5.0);
  alwaysGet.Set(6.0);
  CHECK(1u == ProbotTunableBackendTestAccess::GetDirtyEntryCount(*tunables));

  tunables::TunableRegistry::Update();
  CHECK(0u == ProbotTunableBackendTestAccess::GetDirtyEntryCount(*tunables));
  CHECK(fake::GetDouble("/Tunables/changed") == 5.0);
  CHECK(fake::GetDouble("/Tunables/unchanged") == 2.0);
  CHECK(fake::GetDouble("/Tunables/alwaysGet") == 6.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend failure during dirty update clears dirty entries",
                 "[prosdk][dashboard][tunable]") {
  tunables::Tunable<ThrowingPackStruct> throwingStruct{ThrowingPackStruct{1}};
  tunables::TunableDouble after{2.0};
  tunables::Publish("throwingStruct", throwingStruct);
  tunables::Publish("after", after);

  throwingStruct.Set({3});
  after.Set(4.0);
  CHECK(2u == ProbotTunableBackendTestAccess::GetDirtyEntryCount(*tunables));

  std::vector<std::string> warnings;
  tunables::TunableRegistry::SetReportWarning(
      [&](std::string_view msg) { warnings.emplace_back(msg); });
  ThrowingPackStructState::throwDuringPack = true;
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
  ThrowingPackStructState::throwDuringPack = false;
  CHECK(HasWarning(warnings, "Dashboard tunable backend update failed", "broken pack"));
  CHECK(0 == ProbotTunableBackendTestAccess::GetUpdateDepth(*tunables));
  CHECK(0u == ProbotTunableBackendTestAccess::GetDirtyEntryCount(*tunables));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend immutable tunables refuse remote updates",
                 "[prosdk][dashboard][tunable]") {
  int calls = 0;
  auto config = RobustConfig();
  config.isMutable = false;
  config.onTune = [&](tunables::detail::TunableBase&, tunables::ComplexTunable*) { ++calls; };
  tunables::TunableDouble value{1.0, config};
  tunables::Publish("immutable", value);

  auto topic = fake::Find("/Tunables/immutable");
  REQUIRE(topic);
  CHECK(topic->properties == R"({"mutable":false,"robust":true})");
  CHECK(topic->flags == (dash::kFlagTunable | dash::kFlagRobust));

  Tune("/Tunables/immutable", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  CHECK(1.0 == value.Get());
  CHECK(tunables::TunableRegistry::GetTuneRevision(value) == 0);
  CHECK(0 == calls);
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kImmutable));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend refuses writes of another type",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0};
  tunables::Publish("typed", value);
  Tune("/Tunables/typed", dash::ValueType::kInt, fake::EncodeInt(7));
  tunables::TunableRegistry::Update();
  CHECK(1.0 == value.Get());
  REQUIRE(fake::Rejects().size() == 1);
  CHECK(fake::Rejects()[0].first == "/Tunables/typed");
  CHECK(fake::Rejects()[0].second == static_cast<uint8_t>(dash::WriteStatus::kTypeMismatch));

  // a write to a telemetry topic belongs to nobody
  prosdk::telemetry::Log("readOnly", 1.0);
  Tune("/Telemetry/readOnly", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  tunables::TunableRegistry::Update();
  REQUIRE(fake::Rejects().size() == 2);
  CHECK(fake::Rejects()[1].second == static_cast<uint8_t>(dash::WriteStatus::kUnknownId));
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend publishes tunables once the store is ready",
                 "[prosdk][dashboard][tunable]") {
  fake::SetReady(false);
  tunables::TunableDouble value{1.0};
  tunables::Selectable<int> chooser;
  chooser.AddDefault("one", 1);
  REQUIRE(tunables::Publish("early", value));
  REQUIRE(tunables::Publish("chooser", chooser));
  value.Set(2.0);
  tunables::TunableRegistry::Update();
  CHECK(fake::LiveTopics() == 0);

  fake::SetReady(true);
  dash::FlushPending();  // what the pre-cycle callback does
  CHECK(fake::GetDouble("/Tunables/early") == 2.0);
  CHECK(fake::GetString("/Tunables/chooser/.type") == "Selectable");
  CHECK(fake::GetStringArray("/Tunables/chooser/options") == std::vector<std::string>{"one"});

  Tune("/Tunables/early", dash::ValueType::kDouble, fake::EncodeDouble(3.0));
  tunables::TunableRegistry::Update();
  CHECK(value.Get() == 3.0);
  CHECK(fake::Console().empty());
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend publishes waiting tunables from Update too",
                 "[prosdk][dashboard][tunable]") {
  fake::SetReady(false);
  tunables::TunableDouble value{1.0};
  REQUIRE(tunables::Publish("early", value));
  fake::SetReady(true);
  tunables::TunableRegistry::Update();
  CHECK(fake::GetDouble("/Tunables/early") == 1.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend reports a full topic table once and refuses the tunable",
                 "[prosdk][dashboard][tunable]") {
  fake::SetMaxTopics(2);
  tunables::TunableDouble a{1.0};
  tunables::TunableDouble b{2.0};
  tunables::TunableDouble c{3.0};
  tunables::TunableDouble d{4.0};
  CHECK(tunables::Publish("a", a));
  CHECK(tunables::Publish("b", b));
  CHECK_FALSE(tunables::Publish("c", c));
  CHECK_FALSE(tunables::Publish("d", d));
  CHECK(fake::LiveTopics() == 2);
  CHECK(CountOf(fake::Console(), "topic table is full") == 1);
  CHECK(1u * 2 == ProbotTunableBackendTestAccess::GetEntryCount(*tunables));

  a.Set(5.0);
  tunables::TunableRegistry::Update();
  CHECK(fake::GetDouble("/Tunables/a") == 5.0);

  // space again: the refused tunable can be published later
  tunables::Remove("b");
  CHECK(tunables::Publish("c", c));
  CHECK(fake::GetDouble("/Tunables/c") == 3.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTunableBackend drops a tunable the ready store refuses later",
                 "[prosdk][dashboard][tunable]") {
  fake::SetReady(false);
  fake::SetMaxTopics(1);
  tunables::TunableDouble a{1.0};
  tunables::TunableDouble b{2.0};
  REQUIRE(tunables::Publish("a", a));
  REQUIRE(tunables::Publish("b", b));
  fake::SetReady(true);
  dash::FlushPending();
  CHECK(fake::LiveTopics() == 1);
  CHECK(CountOf(fake::Console(), "topic table is full") == 1);
  CHECK(1u == ProbotTunableBackendTestAccess::GetEntryCount(*tunables));
  CHECK_NOTHROW(tunables::TunableRegistry::Update());
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTunableBackend retire unpublishes and stops writes",
                 "[prosdk][dashboard][tunable]") {
  tunables::TunableDouble value{1.0};
  tunables::Publish("foo", value);
  Tune("/Tunables/foo", dash::ValueType::kDouble, fake::EncodeDouble(2.0));
  dash::DrainWrites();
  tunables->Retire();
  CHECK_FALSE(fake::Find("/Tunables/foo"));
  tunables::TunableRegistry::Update();
  CHECK(1.0 == value.Get());
}
