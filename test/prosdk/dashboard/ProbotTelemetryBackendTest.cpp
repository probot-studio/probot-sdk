// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// ProbotTelemetryBackend on the fake core store. The cases mirror upstream
// NetworkTablesTelemetryBackendTest.cpp (WPILib 2027) with the store in place of NetworkTables,
// plus the store-specific behaviour (not-ready store, limits).

#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "DashboardFixture.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/geometry/proto/Translation2dProto.hpp"
#include "prosdk/math/geometry/struct/Translation2dStruct.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryEntry.hpp"
#include "prosdk/util/SmallVector.hpp"
#include "prosdk/util/json.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/struct/Struct.hpp"

using dashboard_test::CountOf;
using dashboard_test::DashboardFixture;
namespace dash = prosdk::platform::dashboard;

namespace {
uint8_t VT(dash::ValueType vt) {
  return static_cast<uint8_t>(vt);
}
}  // namespace

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend publishes scalar data types",
                 "[prosdk][dashboard][telemetry]") {
  prosdk::telemetry::Log("boolean", true);
  prosdk::telemetry::Log("byte", int8_t{2});
  prosdk::telemetry::Log("short", int16_t{3});
  prosdk::telemetry::Log("int", int32_t{4});
  prosdk::telemetry::Log("long", int64_t{5});
  prosdk::telemetry::Log("float", 6.25f);
  prosdk::telemetry::Log("double", 7.5);
  prosdk::telemetry::Log("string", "ready");
  prosdk::telemetry::Log("json", std::string_view{"{\"ok\":true}"}, std::string_view{"json"});

  CHECK(fake::GetBoolean("/Telemetry/boolean") == true);
  CHECK(fake::GetInt("/Telemetry/byte") == 2);
  CHECK(fake::GetInt("/Telemetry/short") == 3);
  CHECK(fake::GetInt("/Telemetry/int") == 4);
  CHECK(fake::GetInt("/Telemetry/long") == 5);
  CHECK(fake::GetFloat("/Telemetry/float") == 6.25f);
  CHECK(fake::GetDouble("/Telemetry/double") == 7.5);
  CHECK(fake::GetString("/Telemetry/string") == "ready");
  CHECK(fake::GetString("/Telemetry/json") == "{\"ok\":true}");

  REQUIRE(fake::Find("/Telemetry/double"));
  CHECK(fake::Find("/Telemetry/boolean")->valueType == VT(dash::ValueType::kBoolean));
  CHECK(fake::Find("/Telemetry/boolean")->typeString == "boolean");
  CHECK(fake::Find("/Telemetry/long")->valueType == VT(dash::ValueType::kInt));
  CHECK(fake::Find("/Telemetry/long")->typeString == "int");
  CHECK(fake::Find("/Telemetry/float")->valueType == VT(dash::ValueType::kFloat));
  CHECK(fake::Find("/Telemetry/double")->valueType == VT(dash::ValueType::kDouble));
  CHECK(fake::Find("/Telemetry/double")->typeString == "double");
  CHECK(fake::Find("/Telemetry/string")->valueType == VT(dash::ValueType::kString));
  CHECK(fake::Find("/Telemetry/json")->typeString == "json");
  CHECK(fake::Find("/Telemetry/double")->flags == dash::kFlagTelemetry);
  CHECK(fake::Find("/Telemetry/double")->properties == "");
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend ignores timestamps",
                 "[prosdk][dashboard][telemetry]") {
  telemetry->GetEntry("/timestamped")->LogDouble(2.5, 123456789);
  CHECK(fake::GetDouble("/Telemetry/timestamped") == 2.5);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend publishes array and raw data types",
                 "[prosdk][dashboard][telemetry]") {
  const bool boolValues[] = {true, false};
  const int16_t shortValues[] = {1, 2};
  const int32_t intValues[] = {3, 4};
  const int64_t longValues[] = {5, 6};
  const float floatValues[] = {7.25f, 8.5f};
  const double doubleValues[] = {9.25, 10.5};
  const std::string stringValues[] = {"a", "b"};
  const std::string_view stringViewValues[] = {"c", "d"};
  const uint8_t rawValues[] = {11, 12, 13};
  const uint8_t customRawValues[] = {14, 15};

  prosdk::telemetry::Log("booleans", boolValues);
  prosdk::telemetry::Log("shorts", shortValues);
  prosdk::telemetry::Log("ints", intValues);
  prosdk::telemetry::Log("longs", longValues);
  prosdk::telemetry::Log("floats", floatValues);
  prosdk::telemetry::Log("doubles", doubleValues);
  prosdk::telemetry::Log("strings", stringValues);
  prosdk::telemetry::Log("stringViews", stringViewValues);
  prosdk::telemetry::Log("raw", rawValues);
  prosdk::telemetry::Log("customRaw", std::span<const uint8_t>{customRawValues}, "custom");
  const int intBools[] = {0, 1};
  telemetry->GetEntry("/intBools")->LogBooleanArray(std::span<const int>{intBools}, 0);

  CHECK(fake::GetBooleanArray("/Telemetry/booleans") == std::vector<bool>{true, false});
  CHECK(fake::GetIntArray("/Telemetry/shorts") == std::vector<int64_t>{1, 2});
  CHECK(fake::GetIntArray("/Telemetry/ints") == std::vector<int64_t>{3, 4});
  CHECK(fake::GetIntArray("/Telemetry/longs") == std::vector<int64_t>{5, 6});
  CHECK(fake::GetFloatArray("/Telemetry/floats") == std::vector<float>{7.25f, 8.5f});
  CHECK(fake::GetDoubleArray("/Telemetry/doubles") == std::vector<double>{9.25, 10.5});
  CHECK(fake::GetStringArray("/Telemetry/strings") == std::vector<std::string>{"a", "b"});
  CHECK(fake::GetStringArray("/Telemetry/stringViews") == std::vector<std::string>{"c", "d"});
  CHECK(fake::GetRaw("/Telemetry/raw") == std::vector<uint8_t>{11, 12, 13});
  CHECK(fake::GetRaw("/Telemetry/customRaw") == std::vector<uint8_t>{14, 15});
  CHECK(fake::GetBooleanArray("/Telemetry/intBools") == std::vector<bool>{false, true});

  CHECK(fake::Find("/Telemetry/booleans")->typeString == "boolean[]");
  CHECK(fake::Find("/Telemetry/booleans")->valueType == VT(dash::ValueType::kBooleanArray));
  CHECK(fake::Find("/Telemetry/shorts")->typeString == "int[]");
  CHECK(fake::Find("/Telemetry/shorts")->valueType == VT(dash::ValueType::kIntArray));
  CHECK(fake::Find("/Telemetry/floats")->typeString == "float[]");
  CHECK(fake::Find("/Telemetry/doubles")->typeString == "double[]");
  CHECK(fake::Find("/Telemetry/strings")->typeString == "string[]");
  CHECK(fake::Find("/Telemetry/strings")->valueType == VT(dash::ValueType::kStringArray));
  CHECK(fake::Find("/Telemetry/raw")->typeString == "raw");
  CHECK(fake::Find("/Telemetry/raw")->valueType == VT(dash::ValueType::kRaw));
  CHECK(fake::Find("/Telemetry/customRaw")->typeString == "custom");
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend publishes struct and its schema",
                 "[prosdk][dashboard][telemetry]") {
  const prosdk::math::Translation2d value{1.25_m, 2.5_m};
  prosdk::telemetry::Log("translation", value);

  auto topic = fake::Find("/Telemetry/translation");
  REQUIRE(topic);
  CHECK(topic->typeString == "struct:Translation2d");
  CHECK(topic->valueType == VT(dash::ValueType::kRaw));
  auto raw = fake::GetRaw("/Telemetry/translation");
  REQUIRE(raw);
  auto logged = prosdk::util::UnpackStruct<prosdk::math::Translation2d>(*raw);
  CHECK(logged.X() == value.X());
  CHECK(logged.Y() == value.Y());

  auto schema = fake::FindSchema("struct:Translation2d");
  REQUIRE(schema);
  CHECK(schema->type == "structschema");
  CHECK(std::string(schema->data.begin(), schema->data.end()) ==
        prosdk::util::GetStructSchema<prosdk::math::Translation2d>());
  CHECK(telemetry->HasSchema("struct:Translation2d"));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend publishes struct array",
                 "[prosdk][dashboard][telemetry]") {
  const prosdk::math::Translation2d values[] = {{1.25_m, 2.5_m}, {3.75_m, 4.5_m}};
  prosdk::telemetry::Log("translations", std::span<const prosdk::math::Translation2d>{values});

  auto topic = fake::Find("/Telemetry/translations");
  REQUIRE(topic);
  CHECK(topic->typeString == "struct:Translation2d[]");
  auto raw = fake::GetRaw("/Telemetry/translations");
  REQUIRE(raw);
  constexpr size_t size = prosdk::util::GetStructSize<prosdk::math::Translation2d>();
  REQUIRE(raw->size() == 2 * size);
  auto second = prosdk::util::UnpackStruct<prosdk::math::Translation2d>(
      std::span<const uint8_t>{*raw}.subspan(size, size));
  CHECK(second.X() == values[1].X());
  CHECK(second.Y() == values[1].Y());
  CHECK(fake::FindSchema("struct:Translation2d"));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend publishes protobuf",
                 "[prosdk][dashboard][telemetry]") {
  const prosdk::math::Translation2d value{3.75_m, 4.5_m};
  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> message;
  prosdk::util::SmallVector<uint8_t, 128> buf;
  REQUIRE(message.Pack(buf, value));

  prosdk::telemetry::Log("translation", std::span<const uint8_t>{buf}, message.GetTypeString());

  auto topic = fake::Find("/Telemetry/translation");
  REQUIRE(topic);
  CHECK(topic->typeString == message.GetTypeString());
  auto raw = fake::GetRaw("/Telemetry/translation");
  REQUIRE(raw);
  auto logged = message.Unpack(*raw);
  REQUIRE(logged.has_value());
  CHECK(logged->X() == value.X());
  CHECK(logged->Y() == value.Y());
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend adds protobuf descriptors",
                 "[prosdk][dashboard][telemetry]") {
  // geometry2d.proto's descriptor is ~1.3 KB: over core's default PROBOT_DASH_MAX_VALUE (1024)
  fake::SetMaxValue(4096);
  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> message;
  message.ForEachProtobufDescriptor(
      [&](std::string_view filename) { return telemetry->HasSchema(filename); },
      [&](std::string_view filename, std::span<const uint8_t> descriptor) {
        telemetry->AddSchema(filename, "proto:FileDescriptorProto", descriptor);
      });
  bool any = false;
  message.ForEachProtobufDescriptor(
      [&](std::string_view filename) {
        auto schema = fake::FindSchema(filename);
        CHECK(schema);
        if (schema) {
          CHECK(schema->type == "proto:FileDescriptorProto");
          any = true;
        }
        return true;
      },
      [](std::string_view, std::span<const uint8_t>) {});
  CHECK(any);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend applies telemetry properties",
                 "[prosdk][dashboard][telemetry]") {
  prosdk::telemetry::SetProperty("speed", "min", "0");
  prosdk::telemetry::SetProperty("speed", "unit", "\"mps\"");

  prosdk::telemetry::Log("speed", 4.0);
  REQUIRE(fake::Find("/Telemetry/speed"));
  CHECK(fake::Find("/Telemetry/speed")->properties == R"({"min":0,"unit":"mps"})");
  prosdk::telemetry::SetProperty("speed", "max", "10");
  CHECK(fake::Find("/Telemetry/speed")->properties == R"({"max":10,"min":0,"unit":"mps"})");

  std::vector<std::string> warnings;
  prosdk::telemetry::TelemetryRegistry::SetReportWarning(
      [&](std::string_view path, std::string_view msg) {
        warnings.emplace_back(std::string{path} + ": " + std::string{msg});
      });
  prosdk::telemetry::SetProperty("speed", "bad", "{not json");
  CHECK(dashboard_test::HasWarning(warnings, "/Telemetry/speed", "invalid property JSON"));
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend keeps duplicates after publishing",
                 "[prosdk][dashboard][telemetry]") {
  prosdk::telemetry::Log("duplicates", 1.0);
  prosdk::telemetry::Log("duplicates", 1.0);
  REQUIRE(fake::Find("/Telemetry/duplicates"));
  CHECK(fake::Find("/Telemetry/duplicates")->sets == 1);

  prosdk::telemetry::KeepDuplicates("duplicates");
  CHECK(fake::Find("/Telemetry/duplicates")->flags ==
        (dash::kFlagTelemetry | dash::kFlagKeepDuplicates));
  prosdk::telemetry::Log("duplicates", 1.0);
  CHECK(fake::Find("/Telemetry/duplicates")->sets == 2);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend removed entry does not recreate the topic",
                 "[prosdk][dashboard][telemetry]") {
  auto staleEntry = telemetry->GetEntry("/stale");
  staleEntry->LogDouble(0.5, 0);
  CHECK(fake::Find("/Telemetry/stale"));
  telemetry->RemoveEntry("/stale");
  CHECK_FALSE(fake::Find("/Telemetry/stale"));
  CHECK(staleEntry->IsDiscard());

  staleEntry->LogDouble(1.25, 0);
  CHECK_FALSE(fake::Find("/Telemetry/stale"));

  telemetry->GetEntry("/stale")->LogDouble(2.5, 0);
  CHECK(fake::GetDouble("/Telemetry/stale") == 2.5);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend type mismatch warning allows same entry reentry",
                 "[prosdk][dashboard][telemetry]") {
  std::vector<std::string> warnings;
  bool nested = false;
  prosdk::telemetry::TelemetryRegistry::SetReportWarning(
      [&](std::string_view path, std::string_view msg) {
        CHECK("/Telemetry/value" == path);
        CHECK("type mismatch" == msg);
        warnings.emplace_back(path);
        if (!nested) {
          nested = true;
          prosdk::telemetry::Log("value", "again");
        }
      });

  prosdk::telemetry::Log("value", int64_t{1});
  prosdk::telemetry::Log("value", true);

  CHECK((std::vector<std::string>{"/Telemetry/value", "/Telemetry/value"}) == warnings);
  CHECK(fake::GetInt("/Telemetry/value") == 1);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend holds topics and schemas until the store is ready",
                 "[prosdk][dashboard][telemetry]") {
  fake::SetReady(false);
  prosdk::telemetry::SetProperty("early", "unit", "\"V\"");
  prosdk::telemetry::Log("early", 1.0);
  prosdk::telemetry::Log("early", 2.0);
  prosdk::telemetry::Log("pose", prosdk::math::Translation2d{1_m, 2_m});
  CHECK(fake::LiveTopics() == 0);
  CHECK_FALSE(fake::FindSchema("struct:Translation2d"));
  CHECK(telemetry->HasSchema("struct:Translation2d"));  // pending: not added twice

  dash::FlushPending();  // still not ready: nothing happens
  CHECK(fake::LiveTopics() == 0);

  fake::SetReady(true);
  dash::FlushPending();
  CHECK(fake::GetDouble("/Telemetry/early") == 2.0);
  CHECK(fake::Find("/Telemetry/early")->properties == R"({"unit":"V"})");
  CHECK(fake::Find("/Telemetry/pose")->typeString == "struct:Translation2d");
  CHECK(fake::FindSchema("struct:Translation2d"));
  CHECK(fake::Console().empty());

  prosdk::telemetry::Log("early", 3.0);
  CHECK(fake::GetDouble("/Telemetry/early") == 3.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend reports a full topic table once and keeps running",
                 "[prosdk][dashboard][telemetry]") {
  fake::SetMaxTopics(3);
  for (int i = 0; i < 6; ++i) {
    prosdk::telemetry::Log(std::string{"t"} + std::to_string(i), static_cast<double>(i));
  }
  CHECK(fake::LiveTopics() == 3);
  CHECK(CountOf(fake::Console(), "topic table is full") == 1);
  CHECK(CountOf(fake::Console(), "[prosdk]") == 1);
  CHECK(CountOf(fake::Console(), "'/Telemetry/t3'") == 1);
  CHECK(telemetry->GetEntry("/t4")->IsDiscard());
  CHECK_FALSE(telemetry->GetEntry("/t0")->IsDiscard());

  prosdk::telemetry::Log("t0", 10.0);
  CHECK(fake::GetDouble("/Telemetry/t0") == 10.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend reports oversized values once and keeps the topic",
                 "[prosdk][dashboard][telemetry]") {
  std::vector<uint8_t> big(1025, 7);
  std::vector<uint8_t> small(1024, 8);
  prosdk::telemetry::Log("blob", std::span<const uint8_t>{small});
  prosdk::telemetry::Log("blob", std::span<const uint8_t>{big});
  prosdk::telemetry::Log("blob", std::span<const uint8_t>{big});
  CHECK(fake::GetRaw("/Telemetry/blob") == small);
  CHECK(CountOf(fake::Console(), "value not stored") == 1);

  small[0] = 9;
  prosdk::telemetry::Log("blob", std::span<const uint8_t>{small});
  CHECK(fake::GetRaw("/Telemetry/blob") == small);
  CHECK(CountOf(fake::Console(), "[prosdk]") == 1);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend reports a full schema table once",
                 "[prosdk][dashboard][telemetry]") {
  fake::SetMaxSchemas(0);
  prosdk::telemetry::Log("pose", prosdk::math::Translation2d{1_m, 2_m});
  prosdk::telemetry::Log("pose2", prosdk::math::Translation2d{1_m, 2_m});
  CHECK(fake::Find("/Telemetry/pose"));  // the value is still published
  CHECK(CountOf(fake::Console(), "schema not stored") == 1);
}

TEST_CASE_METHOD(DashboardFixture, "ProbotTelemetryBackend reports overlong names once",
                 "[prosdk][dashboard][telemetry]") {
  std::string name(300, 'n');
  prosdk::telemetry::Log(name, 1.0);
  prosdk::telemetry::Log(name + "x", 1.0);
  CHECK(fake::LiveTopics() == 0);
  CHECK(CountOf(fake::Console(), "longer than 255 bytes") == 1);
  prosdk::telemetry::Log("short", 1.0);
  CHECK(fake::GetDouble("/Telemetry/short") == 1.0);
}

TEST_CASE_METHOD(DashboardFixture,
                 "ProbotTelemetryBackend reports a descriptor over the value limit once",
                 "[prosdk][dashboard][telemetry]") {
  // core's default limit: the descriptor is refused and reported once; logging goes on
  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> message;
  for (int i = 0; i < 2; ++i) {
    message.ForEachProtobufDescriptor(
        [&](std::string_view filename) { return telemetry->HasSchema(filename); },
        [&](std::string_view filename, std::span<const uint8_t> descriptor) {
          telemetry->AddSchema(filename, "proto:FileDescriptorProto", descriptor);
        });
  }
  bool stored = false;
  message.ForEachProtobufDescriptor(
      [&](std::string_view filename) {
        stored = stored || fake::FindSchema(filename) != nullptr;
        return true;
      },
      [](std::string_view, std::span<const uint8_t>) {});
  CHECK_FALSE(stored);
  CHECK(CountOf(fake::Console(), "dashboard schema not stored") == 1);
  prosdk::telemetry::Log("after", 1.0);
  CHECK(fake::GetDouble("/Telemetry/after") == 1.0);
}
