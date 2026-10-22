// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/backend/DataLogTelemetryBackend.hpp"

#include <stdint.h>

#include <array>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/datalog/DataLogReader.hpp"
#include "prosdk/datalog/DataLogWriter.hpp"
#include "prosdk/math/geometry/Translation2d.hpp"
#include "prosdk/math/geometry/proto/Translation2dProto.hpp"
#include "prosdk/math/geometry/struct/Translation2dStruct.hpp"
#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryEntry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/units/length.hpp"
#include "prosdk/util/Logger.hpp"
#include "prosdk/util/MemoryBuffer.hpp"
#include "prosdk/util/json.hpp"
#include "prosdk/util/protobuf/Protobuf.hpp"
#include "prosdk/util/raw_ostream.hpp"
#include "prosdk/util/struct/Struct.hpp"

class DataLogTelemetryBackendTest {
 public:
  struct EntryData {
    std::string type;
    std::string metadata;
    std::vector<std::vector<uint8_t>> records;
    std::vector<int64_t> startTimestamps;
    std::vector<int64_t> timestamps;
  };

  struct LogSnapshot {
    std::unordered_map<std::string, EntryData> entries;
  };

  DataLogTelemetryBackendTest()
      : log{msglog, std::make_unique<prosdk::util::raw_uvector_ostream>(data)},
        backend{std::make_shared<prosdk::backend::DataLogTelemetryBackend>(
            log, "/Telemetry")} {
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", backend);
  }

  ~DataLogTelemetryBackendTest() {
    prosdk::telemetry::TelemetryRegistry::SetReportWarning(nullptr);
    prosdk::telemetry::TelemetryRegistry::Reset();
  }

  LogSnapshot ReadSnapshot() {
    log.Flush();

    prosdk::log::DataLogReader reader{
        prosdk::util::MemoryBuffer::GetMemBufferCopy(data, "test")};
    CHECK(reader.IsValid());

    std::unordered_map<int, std::string> names;
    LogSnapshot snapshot;
    for (const auto& record : reader) {
      if (record.IsStart()) {
        prosdk::log::StartRecordData start;
        CHECK(record.GetStartData(&start));
        names.emplace(start.entry, start.name);
        auto& entry = snapshot.entries[std::string{start.name}];
        entry.type = start.type;
        entry.metadata = start.metadata;
        entry.startTimestamps.emplace_back(record.GetTimestamp());
      } else if (record.IsSetMetadata()) {
        prosdk::log::MetadataRecordData metadata;
        CHECK(record.GetSetMetadataData(&metadata));
        auto it = names.find(metadata.entry);
        if (it != names.end()) {
          snapshot.entries[it->second].metadata = metadata.metadata;
        }
      } else if (!record.IsControl()) {
        auto it = names.find(record.GetEntry());
        if (it != names.end()) {
          auto raw = record.GetRaw();
          snapshot.entries[it->second].records.emplace_back(raw.begin(),
                                                            raw.end());
          snapshot.entries[it->second].timestamps.emplace_back(
              record.GetTimestamp());
        }
      }
    }
    return snapshot;
  }

  static const EntryData& Entry(const LogSnapshot& snapshot,
                                std::string_view name) {
    return snapshot.entries.at(std::string{"/Telemetry/"} + std::string{name});
  }

  static const std::vector<uint8_t>& Last(const EntryData& entry) {
    REQUIRE_FALSE(entry.records.empty());
    return entry.records.back();
  }

  static void CheckStartAndLastTimestamp(const EntryData& entry,
                                         int64_t timestamp) {
    CHECK((std::vector<int64_t>{timestamp}) == entry.startTimestamps);
    REQUIRE_FALSE(entry.timestamps.empty());
    CHECK(timestamp == entry.timestamps.back());
  }

  static bool HasEntryWithType(const LogSnapshot& snapshot,
                               std::string_view type) {
    for (const auto& item : snapshot.entries) {
      if (item.second.type == type) {
        return true;
      }
    }
    return false;
  }

  static bool DecodeBoolean(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    bool value = false;
    CHECK(record.GetBoolean(&value));
    return value;
  }

  static int64_t DecodeInteger(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    int64_t value = 0;
    CHECK(record.GetInteger(&value));
    return value;
  }

  static float DecodeFloat(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    float value = 0.0f;
    CHECK(record.GetFloat(&value));
    return value;
  }

  static double DecodeDouble(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    double value = 0.0;
    CHECK(record.GetDouble(&value));
    return value;
  }

  static std::string DecodeString(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::string_view value;
    CHECK(record.GetString(&value));
    return std::string{value};
  }

  static std::vector<int> DecodeBooleanArray(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::vector<int> value;
    CHECK(record.GetBooleanArray(&value));
    return value;
  }

  static std::vector<int64_t> DecodeIntegerArray(
      const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::vector<int64_t> value;
    CHECK(record.GetIntegerArray(&value));
    return value;
  }

  static std::vector<float> DecodeFloatArray(const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::vector<float> value;
    CHECK(record.GetFloatArray(&value));
    return value;
  }

  static std::vector<double> DecodeDoubleArray(
      const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::vector<double> value;
    CHECK(record.GetDoubleArray(&value));
    return value;
  }

  static std::vector<std::string> DecodeStringArray(
      const std::vector<uint8_t>& raw) {
    prosdk::log::DataLogRecord record{1, 0, raw};
    std::vector<std::string_view> views;
    CHECK(record.GetStringArray(&views));
    return {views.begin(), views.end()};
  }

  static prosdk::math::Translation2d DecodeTranslation(
      const std::vector<uint8_t>& raw) {
    return prosdk::util::UnpackStruct<prosdk::math::Translation2d>(
        std::span<const uint8_t>{raw.data(), raw.size()});
  }

  static std::vector<prosdk::math::Translation2d> DecodeTranslationArray(
      const std::vector<uint8_t>& raw) {
    constexpr size_t STRUCT_SIZE =
        prosdk::util::Struct<prosdk::math::Translation2d>::GetSize();
    REQUIRE(0u == raw.size() % STRUCT_SIZE);
    std::vector<prosdk::math::Translation2d> values;
    for (size_t offset = 0; offset < raw.size(); offset += STRUCT_SIZE) {
      values.emplace_back(prosdk::util::UnpackStruct<prosdk::math::Translation2d>(
          std::span<const uint8_t>{raw.data() + offset, STRUCT_SIZE}));
    }
    return values;
  }

  static prosdk::math::Translation2d DecodeTranslationProto(
      const std::vector<uint8_t>& raw) {
    prosdk::util::ProtobufMessage<prosdk::math::Translation2d> msg;
    auto value = msg.Unpack(raw);
    REQUIRE(value.has_value());
    return value.value_or(prosdk::math::Translation2d{});
  }

  static void ExpectTranslationEq(const prosdk::math::Translation2d& expected,
                                  const prosdk::math::Translation2d& actual) {
    CHECK(expected.X().value() == actual.X().value());
    CHECK(expected.Y().value() == actual.Y().value());
  }

  prosdk::util::Logger msglog;
  std::vector<uint8_t> data;
  prosdk::log::DataLogWriter log;
  std::shared_ptr<prosdk::backend::DataLogTelemetryBackend> backend;
};

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest LogsScalarDataTypes",
                 "[wpilibc][telemetry]") {
  prosdk::telemetry::Log("boolean", true);
  prosdk::telemetry::Log("byte", int8_t{2});
  prosdk::telemetry::Log("short", int16_t{3});
  prosdk::telemetry::Log("int", int32_t{4});
  prosdk::telemetry::Log("long", int64_t{5});
  prosdk::telemetry::Log("float", 6.25f);
  prosdk::telemetry::Log("double", 7.5);
  prosdk::telemetry::Log("string", "ready");
  prosdk::telemetry::Log("json", std::string_view{"{\"ok\":true}"},
                      std::string_view{"json"});

  auto snapshot = ReadSnapshot();

  CHECK("boolean" == Entry(snapshot, "boolean").type);
  CHECK(DecodeBoolean(Last(Entry(snapshot, "boolean"))));
  CHECK("int64" == Entry(snapshot, "byte").type);
  CHECK(2 == DecodeInteger(Last(Entry(snapshot, "byte"))));
  CHECK("int64" == Entry(snapshot, "short").type);
  CHECK(3 == DecodeInteger(Last(Entry(snapshot, "short"))));
  CHECK("int64" == Entry(snapshot, "int").type);
  CHECK(4 == DecodeInteger(Last(Entry(snapshot, "int"))));
  CHECK("int64" == Entry(snapshot, "long").type);
  CHECK(5 == DecodeInteger(Last(Entry(snapshot, "long"))));
  CHECK("float" == Entry(snapshot, "float").type);
  CHECK(6.25f == DecodeFloat(Last(Entry(snapshot, "float"))));
  CHECK("double" == Entry(snapshot, "double").type);
  CHECK(7.5 == DecodeDouble(Last(Entry(snapshot, "double"))));
  CHECK("string" == Entry(snapshot, "string").type);
  CHECK("ready" == DecodeString(Last(Entry(snapshot, "string"))));
  CHECK("json" == Entry(snapshot, "json").type);
  CHECK("{\"ok\":true}" == DecodeString(Last(Entry(snapshot, "json"))));
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest LogsExplicitTimestamp",
                 "[wpilibc][telemetry]") {
  constexpr int64_t timestamp = 123456000;
  const uint8_t rawValue[] = {1, 2, 3};

  backend->GetEntry("/timestamped")->LogDouble(2.5, timestamp);
  backend->GetEntry("/timestampedRaw")
      ->LogRaw(std::span<const uint8_t>{rawValue}, "custom", timestamp);

  auto snapshot = ReadSnapshot();
  const auto& timestamped = Entry(snapshot, "timestamped");
  CheckStartAndLastTimestamp(timestamped, timestamp);
  CHECK(2.5 == DecodeDouble(Last(timestamped)));

  const auto& timestampedRaw = Entry(snapshot, "timestampedRaw");
  CheckStartAndLastTimestamp(timestampedRaw, timestamp);
  CHECK((std::vector<uint8_t>{1, 2, 3}) == Last(timestampedRaw));
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest LogsArrayAndRawDataTypes",
                 "[wpilibc][telemetry]") {
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
  prosdk::telemetry::Log("customRaw", std::span<const uint8_t>{customRawValues},
                      "custom");

  auto snapshot = ReadSnapshot();

  CHECK("boolean[]" == Entry(snapshot, "booleans").type);
  CHECK((std::vector<int>{1, 0}) ==
        DecodeBooleanArray(Last(Entry(snapshot, "booleans"))));
  CHECK("int64[]" == Entry(snapshot, "shorts").type);
  CHECK((std::vector<int64_t>{1, 2}) ==
        DecodeIntegerArray(Last(Entry(snapshot, "shorts"))));
  CHECK("int64[]" == Entry(snapshot, "ints").type);
  CHECK((std::vector<int64_t>{3, 4}) ==
        DecodeIntegerArray(Last(Entry(snapshot, "ints"))));
  CHECK("int64[]" == Entry(snapshot, "longs").type);
  CHECK((std::vector<int64_t>{5, 6}) ==
        DecodeIntegerArray(Last(Entry(snapshot, "longs"))));
  CHECK("float[]" == Entry(snapshot, "floats").type);
  CHECK((std::vector<float>{7.25f, 8.5f}) ==
        DecodeFloatArray(Last(Entry(snapshot, "floats"))));
  CHECK("double[]" == Entry(snapshot, "doubles").type);
  CHECK((std::vector<double>{9.25, 10.5}) ==
        DecodeDoubleArray(Last(Entry(snapshot, "doubles"))));
  CHECK("string[]" == Entry(snapshot, "strings").type);
  CHECK((std::vector<std::string>{"a", "b"}) ==
        DecodeStringArray(Last(Entry(snapshot, "strings"))));
  CHECK("string[]" == Entry(snapshot, "stringViews").type);
  CHECK((std::vector<std::string>{"c", "d"}) ==
        DecodeStringArray(Last(Entry(snapshot, "stringViews"))));
  CHECK("raw" == Entry(snapshot, "raw").type);
  CHECK((std::vector<uint8_t>{11, 12, 13}) == Last(Entry(snapshot, "raw")));
  CHECK("custom" == Entry(snapshot, "customRaw").type);
  CHECK((std::vector<uint8_t>{14, 15}) == Last(Entry(snapshot, "customRaw")));
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest LogsStructAndProtobufDataTypes",
                 "[wpilibc][telemetry]") {
  const prosdk::math::Translation2d value{prosdk::units::meter_t{1.25},
                                       prosdk::units::meter_t{2.5}};
  const std::array<prosdk::math::Translation2d, 2> array{
      value, prosdk::math::Translation2d{prosdk::units::meter_t{3.75},
                                      prosdk::units::meter_t{4.5}}};
  prosdk::util::ProtobufMessage<prosdk::math::Translation2d> msg;
  const std::string structType{std::string_view{
      prosdk::util::GetStructTypeString<prosdk::math::Translation2d>()}};
  const std::string protoType = msg.GetTypeString();

  prosdk::telemetry::Log("translation", value);
  prosdk::telemetry::Log("translations", std::span<const prosdk::math::Translation2d>{
                                          array.data(), array.size()});
  prosdk::telemetry::Log("translationProto", value,
                      prosdk::util::ProtobufMessage<prosdk::math::Translation2d>{});

  auto snapshot = ReadSnapshot();

  CHECK(structType == Entry(snapshot, "translation").type);
  ExpectTranslationEq(value,
                      DecodeTranslation(Last(Entry(snapshot, "translation"))));
  CHECK(structType + "[]" == Entry(snapshot, "translations").type);
  auto decodedArray =
      DecodeTranslationArray(Last(Entry(snapshot, "translations")));
  REQUIRE(2u == decodedArray.size());
  ExpectTranslationEq(array[0], decodedArray[0]);
  ExpectTranslationEq(array[1], decodedArray[1]);
  CHECK(protoType == Entry(snapshot, "translationProto").type);
  ExpectTranslationEq(
      value, DecodeTranslationProto(Last(Entry(snapshot, "translationProto"))));

  CHECK("structschema" == snapshot.entries.at("/.schema/" + structType).type);
  CHECK(HasEntryWithType(snapshot, "proto:FileDescriptorProto"));
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest AppliesTelemetryProperties",
                 "[wpilibc][telemetry]") {
  prosdk::telemetry::SetProperty("speed", "min", "0");
  prosdk::telemetry::SetProperty("speed", "unit", "\"m/s\"");
  prosdk::telemetry::Log("speed", 4.0);
  prosdk::telemetry::SetProperty("speed", "max", "10");

  auto snapshot = ReadSnapshot();
  const auto& speed = Entry(snapshot, "speed");

  auto metadata = prosdk::util::json::parse(speed.metadata);
  REQUIRE(metadata);
  REQUIRE((*metadata)["min"].is_number());
  REQUIRE((*metadata)["max"].is_number());
  REQUIRE((*metadata)["unit"].is_string());
  CHECK(0.0 == (*metadata)["min"].get_number());
  CHECK(10.0 == (*metadata)["max"].get_number());
  CHECK("m/s" == (*metadata)["unit"].get_string());
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest HonorsKeepDuplicates",
                 "[wpilibc][telemetry]") {
  prosdk::telemetry::Log("normal", int64_t{1});
  prosdk::telemetry::Log("normal", int64_t{1});
  prosdk::telemetry::KeepDuplicates("duplicates");
  prosdk::telemetry::Log("duplicates", int64_t{2});
  prosdk::telemetry::Log("duplicates", int64_t{2});

  auto snapshot = ReadSnapshot();

  CHECK(1u == Entry(snapshot, "normal").records.size());
  CHECK(2u == Entry(snapshot, "duplicates").records.size());
}

TEST_CASE_METHOD(DataLogTelemetryBackendTest,
                 "DataLogTelemetryBackendTest RemovedEntryDoesNotRecreateEntry",
                 "[wpilibc][telemetry]") {
  auto staleEntry = backend->GetEntry("/stale");
  backend->RemoveEntry("/stale");

  staleEntry->LogDouble(1.25, 0);
  backend->GetEntry("/stale")->LogDouble(2.5, 0);

  auto snapshot = ReadSnapshot();
  const auto& stale = Entry(snapshot, "stale");
  CHECK(1u == stale.records.size());
  CHECK(2.5 == DecodeDouble(Last(stale)));
}

TEST_CASE_METHOD(
    DataLogTelemetryBackendTest,
    "DataLogTelemetryBackendTest TypeMismatchWarningAllowsSameEntryReentry",
    "[wpilibc][telemetry]") {
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

  CHECK((std::vector<std::string>{"/Telemetry/value", "/Telemetry/value"}) ==
        warnings);
}
