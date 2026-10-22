// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/PreferencesStore.hpp"

#include <atomic>
#include <cstdio>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/config.hpp"
#include "prosdk/util/json.hpp"

#if PROSDK_TARGET_HW
#include <nvs.h>
#include <nvs_flash.h>
#endif

namespace prosdk::platform::prefs {
namespace {

// type names of the persisted format (the NetworkTables type strings)
constexpr const char* kTypeNames[] = {"boolean", "int", "float", "double", "string"};

struct Store {
  std::mutex mutex;
  bool loaded = false;
  std::map<std::string, Value, std::less<>> values;
  std::string file;  // host only
};

Store& GetStore() {
  static Store store;
  return store;
}

std::atomic<uint64_t> gRevision{0};

void Changed() {
  gRevision.fetch_add(1, std::memory_order_acq_rel);
}

std::string Serialize(const std::map<std::string, Value, std::less<>>& values) {
  prosdk::util::json arr = prosdk::util::json::array();
  for (auto& [key, value] : values) {
    prosdk::util::json item = prosdk::util::json::object();
    item["name"] = prosdk::util::json{std::string_view{key}};
    item["type"] = prosdk::util::json{kTypeNames[value.index()]};
    switch (value.index()) {
      case 0:
        item["value"] = prosdk::util::json{std::get<bool>(value)};
        break;
      case 1:
        item["value"] = prosdk::util::json{static_cast<long long>(std::get<int64_t>(value))};
        break;
      case 2:
        item["value"] = prosdk::util::json{std::get<float>(value)};
        break;
      case 3:
        item["value"] = prosdk::util::json{std::get<double>(value)};
        break;
      default:
        item["value"] = prosdk::util::json{std::string_view{std::get<std::string>(value)}};
        break;
    }
    arr.emplace_back(std::move(item));
  }
  return arr.to_string();
}

void Deserialize(std::string_view text, std::map<std::string, Value, std::less<>>& values) {
  auto parsed = prosdk::util::json::parse(text);
  if (!parsed || !parsed->is_array()) {
    ConsoleWrite("Preferences: stored data is not readable; starting empty\n", ConsoleLevel::kWarning);
    return;
  }
  for (const auto& item : parsed->get_array()) {
    if (!item.is_object() || !item.contains("name") || !item.contains("type") ||
        !item.contains("value")) {
      continue;
    }
    const auto& name = item.at("name");
    const auto& type = item.at("type");
    const auto& value = item.at("value");
    if (!name.is_string() || !type.is_string()) {
      continue;
    }
    const std::string& t = type.get_string();
    if ((t == "float" || t == "double") && value.is_null()) {
      // NaN is written as null (JSON has no NaN); infinities as +-1e5000, which parse back
      values[name.get_string()] = t == "float" ? Value{std::numeric_limits<float>::quiet_NaN()}
                                               : Value{std::numeric_limits<double>::quiet_NaN()};
    } else if (t == "boolean" && value.is_bool()) {
      values[name.get_string()] = value.get_bool();
    } else if (t == "int" && value.is_number()) {
      values[name.get_string()] = static_cast<int64_t>(
          value.is_int() ? value.get_int() : static_cast<long long>(value.get_number()));
    } else if (t == "float" && value.is_number()) {
      values[name.get_string()] = static_cast<float>(value.get_number());
    } else if (t == "double" && value.is_number()) {
      values[name.get_string()] = value.get_number();
    } else if (t == "string" && value.is_string()) {
      values[name.get_string()] = value.get_string();
    }
  }
}

#if PROSDK_TARGET_HW
constexpr const char* kNvsNamespace = "prosdk";
constexpr const char* kNvsKey = "prefs";

bool NvsOpen(nvs_open_mode_t mode, nvs_handle_t* handle) {
  esp_err_t err = nvs_open(kNvsNamespace, mode, handle);
  if (err == ESP_ERR_NVS_NOT_INITIALIZED && nvs_flash_init() == ESP_OK) {
    err = nvs_open(kNvsNamespace, mode, handle);
  }
  return err == ESP_OK;
}

std::string LoadPersisted(Store&) {
  nvs_handle_t handle;
  if (!NvsOpen(NVS_READONLY, &handle)) {
    return {};  // namespace not created yet: nothing stored
  }
  std::string data;
  size_t len = 0;
  if (nvs_get_blob(handle, kNvsKey, nullptr, &len) == ESP_OK && len > 0) {
    data.resize(len);
    if (nvs_get_blob(handle, kNvsKey, data.data(), &len) != ESP_OK) {
      data.clear();
    }
    data.resize(len);
  }
  nvs_close(handle);
  return data;
}

void SavePersisted(Store& store) {
  std::string data = Serialize(store.values);
  nvs_handle_t handle;
  if (!NvsOpen(NVS_READWRITE, &handle)) {
    ConsoleWrite("Preferences: cannot open NVS; the change is not persisted\n", ConsoleLevel::kWarning);
    return;
  }
  esp_err_t err = nvs_set_blob(handle, kNvsKey, data.data(), data.size());
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  if (err != ESP_OK) {
    ConsoleWrite("Preferences: writing NVS failed; the change is not persisted\n", ConsoleLevel::kWarning);
  }
}
#else
std::string LoadPersisted(Store& store) {
  if (store.file.empty()) {
    return {};
  }
  std::FILE* f = std::fopen(store.file.c_str(), "rb");
  if (f == nullptr) {
    return {};
  }
  std::string data;
  char buf[512];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
    data.append(buf, n);
  }
  std::fclose(f);
  return data;
}

void SavePersisted(Store& store) {
  if (store.file.empty()) {
    return;
  }
  std::string data = Serialize(store.values);
  std::string tmp = store.file + ".tmp";
  std::FILE* f = std::fopen(tmp.c_str(), "wb");
  bool ok = f != nullptr && std::fwrite(data.data(), 1, data.size(), f) == data.size();
  if (f != nullptr) {
    ok = std::fclose(f) == 0 && ok;
  }
  if (!ok || std::rename(tmp.c_str(), store.file.c_str()) != 0) {
    std::remove(tmp.c_str());
    ConsoleWrite("Preferences: writing the preferences file failed\n", ConsoleLevel::kWarning);
  }
}
#endif

// equal values (NaN equals NaN here, so setting NaN again does not rewrite flash)
bool SameValue(const Value& a, const Value& b) {
  if (a.index() != b.index()) {
    return false;
  }
  if (auto f = std::get_if<float>(&a)) {
    float g = std::get<float>(b);
    return *f == g || (*f != *f && g != g);
  }
  if (auto d = std::get_if<double>(&a)) {
    double e = std::get<double>(b);
    return *d == e || (*d != *d && e != e);
  }
  return a == b;
}

// returns the loaded store; the caller holds store.mutex
Store& Loaded(Store& store) {
  if (!store.loaded) {
    store.loaded = true;
    store.values.clear();
    std::string data = LoadPersisted(store);
    if (!data.empty()) {
      Deserialize(data, store.values);
    }
  }
  return store;
}

}  // namespace

std::vector<std::string> Keys() {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  std::vector<std::string> keys;
  for (auto& [key, value] : Loaded(store).values) {
    keys.push_back(key);
  }
  return keys;
}

std::optional<Value> Get(std::string_view key) {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  auto it = values.find(key);
  if (it == values.end()) {
    return std::nullopt;
  }
  return it->second;
}

void Set(std::string_view key, Value value) {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  auto it = values.find(key);
  if (it != values.end()) {
    if (SameValue(it->second, value)) {
      return;
    }
    it->second = std::move(value);
  } else {
    values.emplace(std::string{key}, std::move(value));
  }
  Changed();
  SavePersisted(store);
}

void Init(std::string_view key, Value value) {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  if (values.find(key) != values.end()) {
    return;
  }
  values.emplace(std::string{key}, std::move(value));
  Changed();
  SavePersisted(store);
}

bool Contains(std::string_view key) {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  return values.find(key) != values.end();
}

void Remove(std::string_view key) {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  auto it = values.find(key);
  if (it == values.end()) {
    return;
  }
  values.erase(it);
  Changed();
  SavePersisted(store);
}

void SetPreferencesFile([[maybe_unused]] std::string_view path) {
#if !PROSDK_TARGET_HW
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  store.file.assign(path.data(), path.size());
#endif
}

void Reload() {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  store.loaded = false;
  store.values.clear();
  Changed();
}

uint64_t Revision() {
  return gRevision.load(std::memory_order_acquire);
}

std::vector<std::pair<std::string, Value>> Entries() {
  auto& store = GetStore();
  std::scoped_lock lock{store.mutex};
  auto& values = Loaded(store).values;
  return {values.begin(), values.end()};
}

}  // namespace prosdk::platform::prefs
