// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/preferences/Preferences.hpp"

#include <optional>
#include <string>
#include <variant>
#include <utility>
#include <vector>

#include "prosdk/platform/PreferencesStore.hpp"
#include "prosdk/platform/config.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

// prosdk: upstream keeps the preferences in the persistent NetworkTables table "/Preferences";
// ntcore is not ported, so they live in the platform store (prosdk/platform/PreferencesStore.hpp:
// NVS on the ESP32, memory or a JSON file on the host). Reads follow the NetworkTables entry
// semantics: the numeric getters convert between integer, float and double values, and a value of
// another type yields the default.
namespace prefs = prosdk::platform::prefs;

namespace {
void ReportUsageOnce() {
  static bool reported = [] {
    prosdk::util::ReportUsage("Preferences", "");
    return true;
  }();
  (void)reported;
}

std::optional<prefs::Value> Lookup(std::string_view key) {
  ReportUsageOnce();
  return prefs::Get(key);
}

// numeric value of an integer, float or double preference
std::optional<double> NumberOf(const std::optional<prefs::Value>& v) {
  if (!v) {
    return std::nullopt;
  }
  if (auto i = std::get_if<int64_t>(&*v)) {
    return static_cast<double>(*i);
  }
  if (auto f = std::get_if<float>(&*v)) {
    return static_cast<double>(*f);
  }
  if (auto d = std::get_if<double>(&*v)) {
    return *d;
  }
  return std::nullopt;
}

int64_t IntegerOf(const std::optional<prefs::Value>& v, int64_t defaultValue) {
  if (v) {
    if (auto i = std::get_if<int64_t>(&*v)) {
      return *i;
    }
  }
  auto n = NumberOf(v);
  return n ? static_cast<int64_t>(*n) : defaultValue;
}

void Store(std::string_view key, prefs::Value value) {
  ReportUsageOnce();
  prefs::Set(key, std::move(value));
}

void Init(std::string_view key, prefs::Value value) {
  ReportUsageOnce();
  prefs::Init(key, std::move(value));
}
}  // namespace

#if !PROSDK_TARGET_HW
namespace prosdk::impl {
void ResetPreferencesInstance() {
  prefs::Reload();
}
}  // namespace prosdk::impl
#endif

std::vector<std::string> Preferences::GetKeys() {
  ReportUsageOnce();
  return prefs::Keys();
}

std::string Preferences::GetString(std::string_view key,
                                   std::string_view defaultValue) {
  auto v = Lookup(key);
  if (v) {
    if (auto str = std::get_if<std::string>(&*v)) {
      return *str;
    }
  }
  return std::string{defaultValue};
}

int Preferences::GetInt(std::string_view key, int defaultValue) {
  return static_cast<int>(IntegerOf(Lookup(key), defaultValue));
}

double Preferences::GetDouble(std::string_view key, double defaultValue) {
  return NumberOf(Lookup(key)).value_or(defaultValue);
}

float Preferences::GetFloat(std::string_view key, float defaultValue) {
  auto n = NumberOf(Lookup(key));
  return n ? static_cast<float>(*n) : defaultValue;
}

bool Preferences::GetBoolean(std::string_view key, bool defaultValue) {
  auto v = Lookup(key);
  if (v) {
    if (auto b = std::get_if<bool>(&*v)) {
      return *b;
    }
  }
  return defaultValue;
}

int64_t Preferences::GetLong(std::string_view key, int64_t defaultValue) {
  return IntegerOf(Lookup(key), defaultValue);
}

void Preferences::SetString(std::string_view key, std::string_view value) {
  Store(key, std::string{value});
}

void Preferences::InitString(std::string_view key, std::string_view value) {
  Init(key, std::string{value});
}

void Preferences::SetInt(std::string_view key, int value) {
  Store(key, static_cast<int64_t>(value));
}

void Preferences::InitInt(std::string_view key, int value) {
  Init(key, static_cast<int64_t>(value));
}

void Preferences::SetDouble(std::string_view key, double value) {
  Store(key, value);
}

void Preferences::InitDouble(std::string_view key, double value) {
  Init(key, value);
}

void Preferences::SetFloat(std::string_view key, float value) {
  Store(key, value);
}

void Preferences::InitFloat(std::string_view key, float value) {
  Init(key, value);
}

void Preferences::SetBoolean(std::string_view key, bool value) {
  Store(key, value);
}

void Preferences::InitBoolean(std::string_view key, bool value) {
  Init(key, value);
}

void Preferences::SetLong(std::string_view key, int64_t value) {
  Store(key, value);
}

void Preferences::InitLong(std::string_view key, int64_t value) {
  Init(key, value);
}

bool Preferences::ContainsKey(std::string_view key) {
  ReportUsageOnce();
  return prefs::Contains(key);
}

void Preferences::Remove(std::string_view key) {
  ReportUsageOnce();
  prefs::Remove(key);
}

void Preferences::RemoveAll() {
  for (auto preference : GetKeys()) {
    Remove(preference);
  }
}
