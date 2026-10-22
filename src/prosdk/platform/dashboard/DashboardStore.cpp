// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/DashboardStore.hpp"

#include <algorithm>
#include <atomic>
#include <format>
#include <mutex>
#include <string>
#include <vector>

#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/CoreOps.hpp"
#include "prosdk/util/StringMap.hpp"
#include "prosdk/util/mutex.hpp"

namespace prosdk::platform::dashboard {

namespace {

constexpr size_t kMaxName = 255;  // core: name and type strings are at most 255 bytes

std::atomic<bool> gReported[static_cast<size_t>(Limit::kCount)];
std::atomic<uint32_t> gPersistClears{0};

struct SchemaState {
  struct Pending {
    std::string name;
    std::string type;
    std::vector<uint8_t> data;
  };
  prosdk::util::mutex mutex;
  prosdk::util::StringMap<bool> known;  // name -> stored in core (false: pending or refused)
  std::vector<Pending> pending;
  std::atomic<bool> hasPending{false};
};

SchemaState& Schemas() {
  static SchemaState state;
  return state;
}

struct Router {
  // recursive: a participant's FlushPending() may drain (a persistent tunable's restored value)
  prosdk::util::recursive_mutex mutex;
  std::vector<Participant*> participants;
};

Router& GetRouter() {
  static Router router;
  return router;
}

const char* LimitText(Limit limit) {
  switch (limit) {
    case Limit::kTopics:
      return "dashboard topic table is full (PROBOT_DASH_MAX_TOPICS); this and later new topics "
             "are not shown";
    case Limit::kName:
      return "dashboard topic name or type string is longer than 255 bytes; topic not shown";
    case Limit::kStore:
      return "dashboard store refused the topic (PROBOT_DASH_POOL_BYTES exhausted, properties "
             "longer than PROBOT_DASH_MAX_VALUE, or the path is already published with another "
             "type); later refusals are not reported";
    case Limit::kValue:
      return "dashboard value not stored (larger than PROBOT_DASH_MAX_VALUE or "
             "PROBOT_DASH_POOL_BYTES exhausted); later values are tried again, further failures "
             "are not reported";
    case Limit::kSchemas:
      return "dashboard schema not stored (PROBOT_DASH_MAX_SCHEMAS reached or "
             "PROBOT_DASH_POOL_BYTES exhausted); values of this type cannot be decoded on the "
             "Driver Station";
    case Limit::kProperties:
      return "dashboard topic properties not updated (longer than PROBOT_DASH_MAX_VALUE or pool "
             "exhausted)";
    default:
      return "dashboard limit reached";
  }
}

void DrainOne(const CoreDashboardWrite& write, void* ctx) {
  auto& participants = *static_cast<std::vector<Participant*>*>(ctx);
  std::span<const uint8_t> data{write.data, write.len};
  auto vt = static_cast<ValueType>(write.valueType);
  for (Participant* p : participants) {
    std::optional<WriteStatus> status;
    try {
      status = p->OfferWrite(write.id, vt, data);
    } catch (...) {
      status = WriteStatus::kQueueFull;  // out of memory while queueing
    }
    if (status) {
      if (*status != WriteStatus::kOk) {
        RejectWrite(write.id, *status);
      }
      return;
    }
  }
  RejectWrite(write.id, WriteStatus::kUnknownId);
}

// A schema the store refused, reported once (Limit::kSchemas) with the reason that applies. With
// probot-core R13 the store says its per-schema limit (PROBOT_DASH_MAX_SCHEMA_BYTES); an older
// core limits schemas by PROBOT_DASH_MAX_VALUE (1024 by default, less than a protobuf descriptor
// such as geometry2d.proto's, about 1.3 KB).
void ReportSchemaRefused(std::string_view name, size_t size) noexcept {
  auto index = static_cast<size_t>(Limit::kSchemas);
  if (gReported[index].exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  try {
    const uint32_t maxSchema = GetCoreOps().dashMaxSchemaBytes();
    std::string text;
    if (maxSchema != 0 && size > maxSchema) {
      text = std::format(
          "[prosdk] dashboard schema not stored: '{}' has {} bytes, more than "
          "PROBOT_DASH_MAX_SCHEMA_BYTES ({}); define a larger PROBOT_DASH_MAX_SCHEMA_BYTES (at "
          "most 32768) before #include <prosdk.h>. Values of this type cannot be decoded on the "
          "Driver Station\n",
          name, size, maxSchema);
    } else if (maxSchema != 0) {
      text = std::format("[prosdk] {}: '{}'\n", LimitText(Limit::kSchemas), name);
    } else {
      text = std::format(
          "[prosdk] dashboard schema not stored: '{}' ({} bytes): PROBOT_DASH_MAX_SCHEMAS "
          "reached, PROBOT_DASH_POOL_BYTES exhausted, or larger than PROBOT_DASH_MAX_VALUE "
          "(this probot-core limits schemas by the value size, 1024 bytes by default: "
          "#define PROBOT_DASH_MAX_VALUE {} or more before #include <prosdk.h>, or use a "
          "probot-core with PROBOT_DASH_MAX_SCHEMA_BYTES). Values of this type cannot be decoded "
          "on the Driver Station\n",
          name, size, size);
    }
    ConsoleWrite(text, ConsoleLevel::kWarning);
  } catch (...) {
    ConsoleWrite("[prosdk] dashboard schema not stored\n", ConsoleLevel::kWarning);
  }
}

}  // namespace

bool StoreReady() noexcept {
  CoreDashboardStats stats{};
  GetCoreOps().dashStats(&stats);
  return stats.maxTopics != 0;
}

void ReportLimitOnce(Limit limit, std::string_view path) noexcept {
  auto index = static_cast<size_t>(limit);
  if (index >= static_cast<size_t>(Limit::kCount) ||
      gReported[index].exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  try {
    ConsoleWrite(std::format("[prosdk] {}: '{}'\n", LimitText(limit), path),
                 ConsoleLevel::kWarning);
  } catch (...) {
    ConsoleWrite("[prosdk] dashboard limit reached\n", ConsoleLevel::kWarning);
  }
}

PublishResult PublishTopic(const std::string& path, ValueType valueType,
                           const std::string& typeString, const std::string& properties,
                           uint8_t flags) noexcept {
  PublishResult result;
  result.id = GetCoreOps().dashPublish(path.c_str(), static_cast<uint8_t>(valueType),
                                       typeString.c_str(), properties.c_str(), flags);
  if (result.id != kInvalidTopic) {
    return result;
  }
  CoreDashboardStats stats{};
  GetCoreOps().dashStats(&stats);
  if (stats.maxTopics == 0) {
    result.notReady = true;
  } else if (path.size() > kMaxName || typeString.size() > kMaxName) {
    ReportLimitOnce(Limit::kName, path);
  } else if (stats.topics >= stats.maxTopics) {
    ReportLimitOnce(Limit::kTopics, path);
  } else {
    ReportLimitOnce(Limit::kStore, path);
  }
  return result;
}

void UnpublishTopic(uint16_t id) noexcept {
  if (id != kInvalidTopic) {
    GetCoreOps().dashUnpublish(id);
  }
}

bool SetTopicValue(uint16_t id, std::span<const uint8_t> data, std::string_view path) noexcept {
  if (id == kInvalidTopic) {
    return false;
  }
  if (!GetCoreOps().dashSet(id, data.data(), data.size())) {
    ReportLimitOnce(Limit::kValue, path);
    return false;
  }
  return true;
}

bool SetTopicProperties(uint16_t id, const std::string& properties,
                        std::string_view path) noexcept {
  if (id == kInvalidTopic) {
    return false;
  }
  if (!GetCoreOps().dashSetProperties(id, properties.c_str())) {
    ReportLimitOnce(Limit::kProperties, path);
    return false;
  }
  return true;
}

bool SetTopicFlags(uint16_t id, uint8_t flags) noexcept {
  return id != kInvalidTopic && GetCoreOps().dashSetFlags(id, flags);
}

void RejectWrite(uint16_t id, WriteStatus status) noexcept {
  GetCoreOps().dashRejectWrite(id, static_cast<uint8_t>(status));
}

bool PersistenceSupported() noexcept {
  return DashPersistenceSupported();
}

bool ClearPersisted() noexcept {
  bool cleared = GetCoreOps().dashClearPersisted();
  gPersistClears.fetch_add(1, std::memory_order_acq_rel);
  return cleared;
}

uint32_t PersistClearCount() noexcept {
  return gPersistClears.load(std::memory_order_acquire);
}

bool HasSchema(std::string_view name) noexcept {
  auto& s = Schemas();
  std::scoped_lock lock{s.mutex};
  return s.known.find(name) != s.known.end();
}

void AddSchema(std::string_view name, std::string_view type,
               std::span<const uint8_t> data) noexcept {
  auto& s = Schemas();
  try {
    std::string nameStr{name};
    std::string typeStr{type};
    {
      std::scoped_lock lock{s.mutex};
      if (s.known.find(name) != s.known.end()) {
        return;
      }
      s.known.emplace(nameStr, false);
    }
    if (GetCoreOps().dashAddSchema(nameStr.c_str(), typeStr.c_str(), data.data(),
                                   data.size())) {
      std::scoped_lock lock{s.mutex};
      s.known[nameStr] = true;
      return;
    }
    if (!StoreReady()) {
      std::scoped_lock lock{s.mutex};
      s.pending.push_back({std::move(nameStr), std::move(typeStr), {data.begin(), data.end()}});
      s.hasPending.store(true, std::memory_order_release);
      return;
    }
    ReportSchemaRefused(name, data.size());
  } catch (...) {
    ReportLimitOnce(Limit::kSchemas, name);
  }
}

void AddParticipant(Participant* participant) noexcept {
  if (participant == nullptr) {
    return;
  }
  auto& r = GetRouter();
  std::scoped_lock lock{r.mutex};
  try {
    if (std::find(r.participants.begin(), r.participants.end(), participant) ==
        r.participants.end()) {
      r.participants.push_back(participant);
    }
  } catch (...) {
    ConsoleWrite("[prosdk] dashboard: out of memory registering a backend\n",
                 ConsoleLevel::kError);
  }
}

void RemoveParticipant(Participant* participant) noexcept {
  auto& r = GetRouter();
  std::scoped_lock lock{r.mutex};
  std::erase(r.participants, participant);
}

size_t DrainWrites() noexcept {
  auto& r = GetRouter();
  std::scoped_lock lock{r.mutex};
  return GetCoreOps().dashDrainWrites(&DrainOne, &r.participants);
}

void FlushPending() noexcept {
  auto& s = Schemas();
  if (s.hasPending.load(std::memory_order_acquire) && StoreReady()) {
    std::vector<SchemaState::Pending> pending;
    {
      std::scoped_lock lock{s.mutex};
      pending.swap(s.pending);
      s.hasPending.store(false, std::memory_order_release);
    }
    for (auto& p : pending) {
      bool stored =
          GetCoreOps().dashAddSchema(p.name.c_str(), p.type.c_str(), p.data.data(), p.data.size());
      if (stored) {
        std::scoped_lock lock{s.mutex};
        s.known[p.name] = true;
      } else {
        ReportSchemaRefused(p.name, p.data.size());
      }
    }
  }
  auto& r = GetRouter();
  std::scoped_lock lock{r.mutex};
  if (r.participants.empty() || !StoreReady()) {
    return;
  }
  // a copy: a flush may run tunable callbacks (a restored value's onTune) that register or
  // remove participants on this task
  std::vector<Participant*> participants;
  try {
    participants = r.participants;
  } catch (...) {
    return;  // out of memory: retried next cycle
  }
  for (Participant* p : participants) {
    if (std::find(r.participants.begin(), r.participants.end(), p) == r.participants.end()) {
      continue;  // removed by an earlier flush
    }
    try {
      p->FlushPending();
    } catch (...) {
      ConsoleWrite("[prosdk] dashboard: publishing pending topics failed\n",
                   ConsoleLevel::kError);
    }
  }
}

void ResetForTest() noexcept {
  for (auto& r : gReported) {
    r.store(false, std::memory_order_release);
  }
  auto& s = Schemas();
  std::scoped_lock lock{s.mutex};
  s.known.clear();
  s.pending.clear();
  s.hasPending.store(false, std::memory_order_release);
}

}  // namespace prosdk::platform::dashboard
