// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// An in-memory stand-in for probot-core's dashboard topic store, installed as the CoreOps table
// of the test process (only the dash* and console entries; the rest keep their defaults). It
// follows core's dashboard.hpp semantics: publish is idempotent for the same path and type and
// fails for another type, a full table, a name/type over 255 bytes or properties over the value
// limit; set copies at most the value limit and drops values equal to the last one unless the
// topic has KEEP_DUPLICATES; schemas are idempotent per name and limited in number and size
// (core R13: PROBOT_DASH_MAX_SCHEMA_BYTES, 4096; SetMaxSchemaBytes(0) plays a core without R13,
// whose schemas are limited by the value limit).

#pragma once

#include <stdint.h>

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fake {

struct Topic {
  std::string path;
  uint8_t valueType = 0;
  std::string typeString;
  std::string properties;
  uint8_t flags = 0;
  std::vector<uint8_t> value;
  bool hasValue = false;
  int sets = 0;  // values stored (duplicates dropped by the store are not counted)
  bool live = false;
};

struct Schema {
  std::string type;
  std::vector<uint8_t> data;
};

/** Installs the fake table (once per process) and clears the store; limits back to core's. */
void Reset();

/** Store configured (core runtime_setup ran). */
void SetReady(bool ready);
void SetMaxTopics(size_t maxTopics);
void SetMaxSchemas(size_t maxSchemas);
void SetMaxValue(size_t maxValue);
/** Per-schema byte limit (core R13, default 4096); 0: a core without R13 (the value limit). */
void SetMaxSchemaBytes(size_t maxSchemaBytes);

/** The live topic at path, or nullptr. */
const Topic* Find(std::string_view path);
size_t LiveTopics();
const Schema* FindSchema(std::string_view name);

/** Queues a Driver Station write to the live topic at path (like core's pushWrite, unchecked). */
void PushWrite(std::string_view path, uint8_t valueType, std::span<const uint8_t> data);

/** Writes refused through dashRejectWrite: (topic path, reason). */
const std::vector<std::pair<std::string, uint8_t>>& Rejects();

/** Text written to the console (CoreOps consoleWrite and the prosdk console sink). */
std::string& Console();

// ---- value helpers (store encoding) ----
std::optional<double> GetDouble(std::string_view path);
std::optional<float> GetFloat(std::string_view path);
std::optional<int64_t> GetInt(std::string_view path);
std::optional<bool> GetBoolean(std::string_view path);
std::optional<std::string> GetString(std::string_view path);
std::optional<std::vector<uint8_t>> GetRaw(std::string_view path);
std::optional<std::vector<int64_t>> GetIntArray(std::string_view path);
std::optional<std::vector<double>> GetDoubleArray(std::string_view path);
std::optional<std::vector<float>> GetFloatArray(std::string_view path);
std::optional<std::vector<bool>> GetBooleanArray(std::string_view path);
std::optional<std::vector<std::string>> GetStringArray(std::string_view path);

std::vector<uint8_t> EncodeDouble(double value);
std::vector<uint8_t> EncodeFloat(float value);
std::vector<uint8_t> EncodeInt(int64_t value);
std::vector<uint8_t> EncodeBoolean(bool value);
std::vector<uint8_t> EncodeString(std::string_view value);
std::vector<uint8_t> EncodeStringArray(std::span<const std::string> value);

}  // namespace fake
