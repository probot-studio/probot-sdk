// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Storage behind prosdk::Preferences (upstream: the persistent NetworkTables table
// "/Preferences"; ntcore is not ported).
//
// The store holds typed values (boolean, integer, float, double, string, the NetworkTables types
// Preferences uses) with the NetworkTables read semantics: GetInt/GetLong/GetFloat/GetDouble
// convert between the numeric types, a value of another type yields the default. A Set replaces
// the value and its type.
//
// Persistence (all keys are persistent, as upstream marks every Preferences topic persistent):
//   ESP32  one NVS blob (namespace "prosdk", key "prefs"; serialized as JSON) in the default NVS
//          partition. It is rewritten on every change; a Set with the value already stored
//          writes nothing. Do not call Set with a changing value every loop (flash wear).
//   Host   memory only, unless SetPreferencesFile() names a JSON file, which is then loaded on
//          first use and rewritten on every change.
// The store is loaded on first use. Thread-safe.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace prosdk::platform::prefs {

/** One preference value (index order is the type tag). */
using Value = std::variant<bool, int64_t, float, double, std::string>;

/** All keys, sorted. */
std::vector<std::string> Keys();

/** The stored value, or nullopt. */
std::optional<Value> Get(std::string_view key);

/** Stores a value (replacing value and type) and persists it if it changed. */
void Set(std::string_view key, Value value);

/** Stores a value only if the key does not exist yet. */
void Init(std::string_view key, Value value);

bool Contains(std::string_view key);

/** Removes one key (and persists the removal). */
void Remove(std::string_view key);

/**
 * Host only (no-op on ESP32): the JSON file that backs the store; "" (the default) keeps it in
 * memory. Takes effect at the next load: call it before the first Preferences access, or call
 * Reload() afterwards.
 */
void SetPreferencesFile(std::string_view path);

/** Drops the in-memory copy; the next access loads the persisted state again. */
void Reload();

}  // namespace prosdk::platform::prefs
