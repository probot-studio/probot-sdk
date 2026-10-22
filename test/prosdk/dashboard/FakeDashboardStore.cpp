// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#include "FakeDashboardStore.hpp"

#include <cstring>
#include <map>
#include <mutex>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"

namespace fake {

namespace {

constexpr uint16_t kInvalid = 0xFFFF;
constexpr uint8_t kKeepDuplicates = 16;

struct Write {
  uint16_t id;
  uint8_t valueType;
  std::vector<uint8_t> data;
};

struct Store {
  std::recursive_mutex mutex;
  bool ready = true;
  size_t maxTopics = 128;
  size_t maxSchemas = 32;
  size_t maxValue = 1024;
  size_t maxSchemaBytes = 4096;  // core R13 default; 0: a core without R13 (maxValue applies)
  std::vector<Topic> topics;  // id = index
  std::map<std::string, Schema, std::less<>> schemas;
  std::vector<Write> writes;
  std::vector<std::pair<std::string, uint8_t>> rejects;
  std::string console;
};

Store& S() {
  static Store* store = new Store;  // never destroyed: backends may outlive static destruction
  return *store;
}

size_t CountLive() {
  size_t n = 0;
  for (auto& t : S().topics) {
    n += t.live ? 1 : 0;
  }
  return n;
}

Topic* LiveTopic(uint16_t id) {
  auto& s = S();
  if (id >= s.topics.size() || !s.topics[id].live) {
    return nullptr;
  }
  return &s.topics[id];
}

uint16_t Publish(const char* path, uint8_t valueType, const char* typeString,
                 const char* properties, uint8_t flags) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  if (!s.ready) {
    return kInvalid;
  }
  std::string_view p{path};
  std::string_view t{typeString ? typeString : ""};
  std::string_view props{properties ? properties : ""};
  if (p.size() > 255 || t.size() > 255 || props.size() > s.maxValue) {
    return kInvalid;
  }
  for (size_t i = 0; i < s.topics.size(); ++i) {
    auto& topic = s.topics[i];
    if (topic.live && topic.path == p) {
      if (topic.valueType == valueType && topic.typeString == t) {
        return static_cast<uint16_t>(i);
      }
      return kInvalid;
    }
  }
  if (CountLive() >= s.maxTopics) {
    return kInvalid;
  }
  Topic topic;
  topic.path = p;
  topic.valueType = valueType;
  topic.typeString = t;
  topic.properties = props;
  topic.flags = flags;
  topic.live = true;
  for (size_t i = 0; i < s.topics.size(); ++i) {
    if (!s.topics[i].live) {
      s.topics[i] = std::move(topic);
      return static_cast<uint16_t>(i);
    }
  }
  s.topics.push_back(std::move(topic));
  return static_cast<uint16_t>(s.topics.size() - 1);
}

void Unpublish(uint16_t id) {
  std::scoped_lock lock{S().mutex};
  if (auto t = LiveTopic(id)) {
    *t = Topic{};
  }
}

bool Set(uint16_t id, const void* data, size_t len) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  auto t = LiveTopic(id);
  if (!t || len > s.maxValue) {
    return false;
  }
  auto bytes = static_cast<const uint8_t*>(data);
  std::vector<uint8_t> value{bytes, bytes + len};
  if (t->hasValue && t->value == value && (t->flags & kKeepDuplicates) == 0) {
    return true;
  }
  t->value = std::move(value);
  t->hasValue = true;
  ++t->sets;
  return true;
}

bool SetProperties(uint16_t id, const char* properties) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  auto t = LiveTopic(id);
  if (!t || std::strlen(properties) > s.maxValue) {
    return false;
  }
  t->properties = properties;
  return true;
}

bool SetFlags(uint16_t id, uint8_t flags) {
  std::scoped_lock lock{S().mutex};
  auto t = LiveTopic(id);
  if (!t) {
    return false;
  }
  t->flags = flags;
  return true;
}

bool HasSchema(const char* name) {
  std::scoped_lock lock{S().mutex};
  return S().schemas.find(std::string_view{name}) != S().schemas.end();
}

bool AddSchema(const char* name, const char* type, const void* data, size_t len) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  if (!s.ready) {
    return false;
  }
  if (s.schemas.find(std::string_view{name}) != s.schemas.end()) {
    return true;
  }
  const size_t limit = s.maxSchemaBytes != 0 ? s.maxSchemaBytes : s.maxValue;
  if (s.schemas.size() >= s.maxSchemas || len > limit) {
    return false;
  }
  auto bytes = static_cast<const uint8_t*>(data);
  s.schemas.emplace(name, Schema{type, {bytes, bytes + len}});
  return true;
}

size_t DrainWrites(void (*fn)(const prosdk::platform::CoreDashboardWrite& write, void* ctx),
                   void* ctx) {
  std::vector<Write> writes;
  {
    std::scoped_lock lock{S().mutex};
    writes.swap(S().writes);
  }
  for (auto& w : writes) {
    prosdk::platform::CoreDashboardWrite write{w.id, w.valueType,
                                               static_cast<uint16_t>(w.data.size()),
                                               w.data.data()};
    fn(write, ctx);
  }
  return writes.size();
}

void RejectWrite(uint16_t id, uint8_t reason) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  auto t = LiveTopic(id);
  s.rejects.emplace_back(t ? t->path : std::string{"<unknown>"}, reason);
}

void Stats(prosdk::platform::CoreDashboardStats* out) {
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  *out = {};
  if (!s.ready) {
    return;
  }
  out->topics = static_cast<uint16_t>(CountLive());
  out->maxTopics = static_cast<uint16_t>(s.maxTopics);
}

uint32_t MaxSchemaBytes() {
  std::scoped_lock lock{S().mutex};
  return static_cast<uint32_t>(S().maxSchemaBytes);
}

void ConsoleWriteFn(const char* text, size_t len) {
  std::scoped_lock lock{S().mutex};
  S().console.append(text, len);
}

void CaptureConsole(std::string_view text) {
  std::scoped_lock lock{S().mutex};
  S().console.append(text);
}

void InstallOnce() {
  static bool installed = [] {
    prosdk::platform::CoreOps ops{};
    ops.abiVersion = prosdk::platform::kCoreOpsAbiVersion;
    ops.size = sizeof(prosdk::platform::CoreOps);
    ops.consoleWrite = ConsoleWriteFn;
    ops.dashPublish = Publish;
    ops.dashUnpublish = Unpublish;
    ops.dashSet = Set;
    ops.dashSetProperties = SetProperties;
    ops.dashSetFlags = SetFlags;
    ops.dashHasSchema = HasSchema;
    ops.dashAddSchema = AddSchema;
    ops.dashDrainWrites = DrainWrites;
    ops.dashRejectWrite = RejectWrite;
    ops.dashStats = Stats;
    ops.dashMaxSchemaBytes = MaxSchemaBytes;
    return prosdk::platform::InstallCoreOps(&ops);
  }();
  (void)installed;
}

template <typename T>
std::optional<T> Scalar(std::string_view path) {
  auto raw = GetRaw(path);
  if (!raw || raw->size() != sizeof(T)) {
    return std::nullopt;
  }
  T value;
  std::memcpy(&value, raw->data(), sizeof(T));
  return value;
}

template <typename T>
std::optional<std::vector<T>> Array(std::string_view path) {
  auto raw = GetRaw(path);
  if (!raw || raw->size() % sizeof(T) != 0) {
    return std::nullopt;
  }
  std::vector<T> values(raw->size() / sizeof(T));
  std::memcpy(values.data(), raw->data(), raw->size());
  return values;
}

template <typename T>
std::vector<uint8_t> Bytes(T value) {
  std::vector<uint8_t> out(sizeof(T));
  std::memcpy(out.data(), &value, sizeof(T));
  return out;
}

}  // namespace

void Reset() {
  InstallOnce();
  auto& s = S();
  std::scoped_lock lock{s.mutex};
  s.ready = true;
  s.maxTopics = 128;
  s.maxSchemas = 32;
  s.maxValue = 1024;
  s.maxSchemaBytes = 4096;
  s.topics.clear();
  s.schemas.clear();
  s.writes.clear();
  s.rejects.clear();
  s.console.clear();
  prosdk::platform::SetConsoleSink(CaptureConsole);
}

void SetReady(bool ready) {
  std::scoped_lock lock{S().mutex};
  S().ready = ready;
}

void SetMaxTopics(size_t maxTopics) {
  std::scoped_lock lock{S().mutex};
  S().maxTopics = maxTopics;
}

void SetMaxSchemas(size_t maxSchemas) {
  std::scoped_lock lock{S().mutex};
  S().maxSchemas = maxSchemas;
}

void SetMaxValue(size_t maxValue) {
  std::scoped_lock lock{S().mutex};
  S().maxValue = maxValue;
}

void SetMaxSchemaBytes(size_t maxSchemaBytes) {
  std::scoped_lock lock{S().mutex};
  S().maxSchemaBytes = maxSchemaBytes;
}

const Topic* Find(std::string_view path) {
  std::scoped_lock lock{S().mutex};
  for (auto& t : S().topics) {
    if (t.live && t.path == path) {
      return &t;
    }
  }
  return nullptr;
}

size_t LiveTopics() {
  std::scoped_lock lock{S().mutex};
  return CountLive();
}

const Schema* FindSchema(std::string_view name) {
  std::scoped_lock lock{S().mutex};
  auto it = S().schemas.find(name);
  return it == S().schemas.end() ? nullptr : &it->second;
}

void PushWrite(std::string_view path, uint8_t valueType, std::span<const uint8_t> data) {
  std::scoped_lock lock{S().mutex};
  auto& s = S();
  for (size_t i = 0; i < s.topics.size(); ++i) {
    if (s.topics[i].live && s.topics[i].path == path) {
      s.writes.push_back({static_cast<uint16_t>(i), valueType, {data.begin(), data.end()}});
      return;
    }
  }
}

const std::vector<std::pair<std::string, uint8_t>>& Rejects() {
  return S().rejects;
}

std::string& Console() {
  return S().console;
}

std::optional<std::vector<uint8_t>> GetRaw(std::string_view path) {
  auto t = Find(path);
  if (!t || !t->hasValue) {
    return std::nullopt;
  }
  return t->value;
}

std::optional<double> GetDouble(std::string_view path) {
  return Scalar<double>(path);
}

std::optional<float> GetFloat(std::string_view path) {
  return Scalar<float>(path);
}

std::optional<int64_t> GetInt(std::string_view path) {
  return Scalar<int64_t>(path);
}

std::optional<bool> GetBoolean(std::string_view path) {
  auto v = Scalar<uint8_t>(path);
  if (!v) {
    return std::nullopt;
  }
  return *v != 0;
}

std::optional<std::string> GetString(std::string_view path) {
  auto raw = GetRaw(path);
  if (!raw) {
    return std::nullopt;
  }
  return std::string{raw->begin(), raw->end()};
}

std::optional<std::vector<int64_t>> GetIntArray(std::string_view path) {
  return Array<int64_t>(path);
}

std::optional<std::vector<double>> GetDoubleArray(std::string_view path) {
  return Array<double>(path);
}

std::optional<std::vector<float>> GetFloatArray(std::string_view path) {
  return Array<float>(path);
}

std::optional<std::vector<bool>> GetBooleanArray(std::string_view path) {
  auto raw = GetRaw(path);
  if (!raw) {
    return std::nullopt;
  }
  std::vector<bool> values;
  for (uint8_t b : *raw) {
    values.push_back(b != 0);
  }
  return values;
}

std::optional<std::vector<std::string>> GetStringArray(std::string_view path) {
  auto raw = GetRaw(path);
  if (!raw) {
    return std::nullopt;
  }
  std::vector<std::string> values;
  size_t p = 0;
  while (p < raw->size()) {
    if (raw->size() - p < 2) {
      return std::nullopt;
    }
    uint16_t n;
    std::memcpy(&n, raw->data() + p, 2);
    p += 2;
    if (raw->size() - p < n) {
      return std::nullopt;
    }
    values.emplace_back(reinterpret_cast<const char*>(raw->data() + p), n);
    p += n;
  }
  return values;
}

std::vector<uint8_t> EncodeDouble(double value) {
  return Bytes(value);
}

std::vector<uint8_t> EncodeFloat(float value) {
  return Bytes(value);
}

std::vector<uint8_t> EncodeInt(int64_t value) {
  return Bytes(value);
}

std::vector<uint8_t> EncodeBoolean(bool value) {
  return {static_cast<uint8_t>(value ? 1 : 0)};
}

std::vector<uint8_t> EncodeString(std::string_view value) {
  return {value.begin(), value.end()};
}

std::vector<uint8_t> EncodeStringArray(std::span<const std::string> value) {
  std::vector<uint8_t> out;
  for (auto& s : value) {
    uint16_t n = static_cast<uint16_t>(s.size());
    out.push_back(static_cast<uint8_t>(n & 0xFF));
    out.push_back(static_cast<uint8_t>(n >> 8));
    out.insert(out.end(), s.begin(), s.end());
  }
  return out;
}

}  // namespace fake
