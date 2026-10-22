// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Mirrors upstream NetworkTablesTunableBackend.cpp (WPILib 2027): the same entry kinds (dirty,
// always-get, complex), the same re-entrancy rules (entries are updated with the backend mutex
// released; erases and untracks requested during an update are deferred; entries inserted during
// an update wait for the next one), the same remote-tune sequence and warnings. The transport is
// probot-core's dashboard store instead of NetworkTables publishers and a listener poller.

#include "prosdk/platform/dashboard/ProbotTunableBackend.hpp"

#include <stdint.h>

#include <algorithm>
#include <cstring>
#include <exception>
#include <format>
#include <mutex>
#include <utility>

#include "prosdk/tunables/ComplexTunable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/detail/PathUtil.hpp"
#include "prosdk/tunables/detail/TunableBase.hpp"
#include "prosdk/tunables/detail/TunableDetail.hpp"
#include "prosdk/tunables/detail/TunableMember.hpp"
#include "prosdk/tunables/detail/TunableTypeValue.hpp"
#include "prosdk/util/json.hpp"

using namespace prosdk::backend;
using namespace prosdk::tunables;
namespace dash = prosdk::platform::dashboard;
using dash::ValueType;
using dash::WriteStatus;

namespace {

std::string_view GetTypeString(std::string_view defaultType, const TunableConfig* config) {
  if (config && config->typeString) {
    return *config->typeString;
  }
  return defaultType;
}

prosdk::util::json GetProperties(const TunableConfig* config) {
  if (!config) {
    prosdk::util::json properties = prosdk::util::json::object();
    properties["mutable"] = true;
    return properties;
  }

  prosdk::util::json properties = config->properties;
  if (config->robust) {
    properties["robust"] = true;
  }
  properties["mutable"] = config->isMutable;
  return properties;
}

uint8_t GetFlags(const TunableConfig* config) {
  uint8_t flags = dash::kFlagTunable;
  if (!config || config->isMutable) {
    flags |= dash::kFlagMutable;
  }
  if (config && config->robust) {
    flags |= dash::kFlagRobust;
  }
  return flags;
}

// prosdk addition: properties {"persistent": true} (the NetworkTables topic property of the same
// name) keeps a mutable tunable's value across reboots in probot-core's store
bool IsPersistent(const TunableConfig* config) {
  if (!config || !config->isMutable) {
    return false;
  }
  const prosdk::util::json* persistent = config->properties.lookup("persistent");
  return persistent != nullptr && persistent->is_bool() && persistent->get_bool();
}

bool IsAlwaysGet(const TunableConfig* config) {
  return config && config->polling == TunableConfig::Polling::kAlwaysGet;
}

void NotifyOnTune(uint32_t uid, TunableRegistry::TunableInfo& info) {
  TunableRegistry::ResetChangedAfterUpdate(uid);
  if (auto config = info.config) {
    if (config->onTune) {
      TunableRegistry::RunAfterUpdate([uid] {
        auto info = TunableRegistry::GetTunable(uid);
        if (auto config = info.config) {
          if (auto&& onTune = config->onTune) {
            onTune(*info.tunable, config->parent);
          }
        }
      });
    }
  }
}

void NotifyRemoteSet(TunableRegistry::TunableInfo& info) {
  if (auto config = info.config) {
    if (config->onRemoteSet) {
      config->onRemoteSet(*info.tunable, config->parent);
    }
  }
}

bool ShouldUpdateNetwork(const TunableRegistry::TunableInfo& info) {
  return (info.config && info.config->polling == TunableConfig::Polling::kAlwaysGet) ||
         info.IsChanged();
}

template <typename T>
const T& GetValue(const TunableRegistry::TunableInfo& info) {
  if (info.type == detail::GetTunableTypeValue<T, false>()) {
    return static_cast<detail::TunableValueBase<T>*>(info.tunable)->Get();
  }
  return static_cast<detail::TunableMemberValueBase<T>*>(info.tunable)->Get(info.config->parent);
}

template <typename T>
void SetValue(const TunableRegistry::TunableInfo& info, T value) {
  if (info.type == detail::GetTunableTypeValue<T, false>()) {
    static_cast<detail::TunableValueBase<T>*>(info.tunable)->Set(std::move(value));
  } else {
    static_cast<detail::TunableMemberValueBase<T>*>(info.tunable)
        ->Set(info.config->parent, std::move(value));
  }
}

// ---- store encoding of the tunable value types ----

template <typename T>
struct Codec;

template <typename T>
  requires std::is_arithmetic_v<T>
struct ScalarCodec {
  // bool -> u8, integers -> i64, float -> f32, double -> f64
  using Wire = std::conditional_t<std::is_same_v<T, bool>, uint8_t,
                                  std::conditional_t<std::is_integral_v<T>, int64_t, T>>;
  static void Encode(const T& value, std::vector<uint8_t>& out) {
    dash::Append<Wire>(out, static_cast<Wire>(value));
  }
  static bool Decode(std::span<const uint8_t> data, T& value) {
    if (data.size() != sizeof(Wire)) {
      return false;
    }
    if constexpr (std::is_same_v<T, bool>) {
      value = data[0] != 0;
    } else {
      value = static_cast<T>(dash::Read<Wire>(data, 0));
    }
    return true;
  }
};

template <>
struct Codec<bool> : ScalarCodec<bool> {};
template <>
struct Codec<int32_t> : ScalarCodec<int32_t> {};
template <>
struct Codec<int64_t> : ScalarCodec<int64_t> {};
template <>
struct Codec<float> : ScalarCodec<float> {};
template <>
struct Codec<double> : ScalarCodec<double> {};

template <>
struct Codec<std::string> {
  static void Encode(const std::string& value, std::vector<uint8_t>& out) {
    out.assign(value.begin(), value.end());
  }
  static bool Decode(std::span<const uint8_t> data, std::string& value) {
    value.assign(reinterpret_cast<const char*>(data.data()), data.size());
    return true;
  }
};

template <typename U>
struct Codec<std::vector<U>> {
  using Element = ScalarCodec<U>;
  using Wire = typename Element::Wire;
  static void Encode(const std::vector<U>& value, std::vector<uint8_t>& out) {
    out.reserve(value.size() * sizeof(Wire));
    for (U v : value) {
      Element::Encode(v, out);
    }
  }
  static bool Decode(std::span<const uint8_t> data, std::vector<U>& value) {
    if (data.size() % sizeof(Wire) != 0) {
      return false;
    }
    value.clear();
    value.reserve(data.size() / sizeof(Wire));
    for (size_t i = 0; i < data.size(); i += sizeof(Wire)) {
      U v{};
      Element::Decode(data.subspan(i, sizeof(Wire)), v);
      value.push_back(v);
    }
    return true;
  }
};

template <>
struct Codec<std::vector<uint8_t>> {  // raw
  static void Encode(const std::vector<uint8_t>& value, std::vector<uint8_t>& out) {
    out.assign(value.begin(), value.end());
  }
  static bool Decode(std::span<const uint8_t> data, std::vector<uint8_t>& value) {
    value.assign(data.begin(), data.end());
    return true;
  }
};

template <>
struct Codec<std::vector<std::string>> {
  static void Encode(const std::vector<std::string>& value, std::vector<uint8_t>& out) {
    for (const auto& v : value) {
      // an element over 64 KiB cannot be stored anyway (PROBOT_DASH_MAX_VALUE)
      dash::AppendStringElement(out, std::string_view{v}.substr(0, 0xFFFF));
    }
  }
  static bool Decode(std::span<const uint8_t> data, std::vector<std::string>& value) {
    return dash::DecodeStringArray(data, value);
  }
};

}  // namespace

class ProbotTunableBackend::Entry {
 public:
  enum class UpdateKind { DIRTY, ALWAYS_GET, COMPLEX };

  Entry(std::string key, std::string path, uint32_t uid, ValueType valueType,
        std::string typeString, std::string properties, uint8_t flags, bool persistent)
      : m_key{std::move(key)},
        m_path{std::move(path)},
        m_uid{uid},
        m_valueType{valueType},
        m_typeString{std::move(typeString)},
        m_properties{std::move(properties)},
        m_flags{flags},
        m_persistent{persistent} {}

  virtual ~Entry() { Close(); }

  Entry(const Entry&) = delete;
  Entry& operator=(const Entry&) = delete;

  uint32_t GetUid() const { return m_uid; }
  const std::string& GetKey() const { return m_key; }
  ValueType GetValueType() const { return m_valueType; }
  const std::string& GetTypeString() const { return m_typeString; }
  bool IsMutable() const { return (m_flags & dash::kFlagMutable) != 0; }
  /** Opted in to persistence (properties.persistent on a mutable value tunable). */
  bool WantsPersistence() const { return m_persistent; }
  /** Published with core's PERSISTENT flag (core may have queued its stored value). */
  bool IsCorePersistent() const { return m_corePersistent; }
  /**
   * Publishes the tunable's value to the store topic if it changed (or force); returns the value
   * the store took (valid until the next call), or nullptr when nothing was set.
   */
  virtual const std::vector<uint8_t>* UpdateNetwork(bool force) = 0;
  virtual ValueEntry* AsValueEntry() { return nullptr; }

  /** Publishes the store topic (caller holds the backend mutex). */
  dash::PublishResult Open() {
    // PERSISTENT only for an opt-in tunable and a core that keeps the values (CoreOps ABI 6)
    bool persist = m_persistent && dash::PersistenceSupported();
    auto result = dash::PublishTopic(m_path, m_valueType, m_typeString, m_properties,
                                     persist ? m_flags | dash::kFlagPersistent : m_flags);
    m_id.store(result.id, std::memory_order_release);
    m_corePersistent = persist && result.id != dash::kInvalidTopic;
    return result;
  }

  /** Unpublishes the store topic. */
  void Close() { dash::UnpublishTopic(m_id.exchange(dash::kInvalidTopic)); }

  uint16_t GetTopicId() const { return m_id.load(std::memory_order_acquire); }
  bool IsPublished() const { return GetTopicId() != dash::kInvalidTopic; }

  void SetInsertEpoch(uint64_t epoch) { m_insertEpoch = epoch; }
  uint64_t GetInsertEpoch() const { return m_insertEpoch; }
  void SetUpdateKind(UpdateKind kind) { m_updateKind = kind; }
  UpdateKind GetUpdateKind() const { return m_updateKind; }
  void SetDirtyQueued(bool queued) { m_dirtyQueued = queued; }
  bool IsDirtyQueued() const { return m_dirtyQueued; }
  void MarkRemoved() { m_removed = true; }
  bool IsRemoved() const { return m_removed; }

 protected:
  bool SetStoreValue(std::span<const uint8_t> data) {
    return dash::SetTopicValue(GetTopicId(), data, m_path);
  }

  void ReportWarning(std::string_view msg) {
    TunableRegistry::ReportWarning(std::format("Dashboard tunable '{}': {}", m_path, msg));
  }

  void ReportWarning(std::string_view msg, const std::exception& e) {
    ReportWarning(std::format("{}: {}", msg, e.what()));
  }

 private:
  std::string m_key;   // path relative to the backend prefix (m_entries key)
  std::string m_path;  // store topic path
  uint32_t m_uid;
  ValueType m_valueType;
  std::string m_typeString;
  std::string m_properties;
  uint8_t m_flags;
  bool m_persistent;              // opt-in (properties.persistent), mutable value tunable
  bool m_corePersistent = false;  // the open topic has core's PERSISTENT flag
  std::atomic<uint16_t> m_id{dash::kInvalidTopic};
  uint64_t m_insertEpoch = 0;
  UpdateKind m_updateKind = UpdateKind::DIRTY;
  bool m_dirtyQueued = false;
  bool m_removed = false;
};

class ProbotTunableBackend::ValueEntry : public Entry {
 public:
  using Entry::Entry;

  ValueEntry* AsValueEntry() override { return this; }

  /** Applies a Driver Station write; false if the payload does not decode. */
  bool UpdateTunable(std::span<const uint8_t> data) {
    auto info = TunableRegistry::GetTunable(GetUid());
    if (!info) {
      return false;
    }
    if (!DoUpdateTunable(info, data)) {
      return false;
    }
    TunableRegistry::RecordTuneApplied(GetUid());
    NotifyRemoteSet(info);
    m_forceUpdate = true;
    return true;
  }

  /** The tunable's value encodes to data (an equal value needs no apply). */
  bool HoldsValue(std::span<const uint8_t> data) {
    auto info = TunableRegistry::GetTunable(GetUid());
    if (!info) {
      return false;
    }
    m_data.clear();
    return Encode(info, m_data) && std::ranges::equal(m_data, data);
  }

  const std::vector<uint8_t>* UpdateNetwork(bool force) override {
    if (!IsPublished()) {
      return nullptr;  // published (and forced) once the store is ready
    }
    auto info = TunableRegistry::GetTunable(GetUid());
    if (!info) {
      return nullptr;
    }
    bool forcePublish = force || m_forceUpdate;
    if (!forcePublish && !ShouldUpdateNetwork(info)) {
      return nullptr;
    }
    bool stored = false;
    m_data.clear();
    if (Encode(info, m_data) && (forcePublish || m_data != m_lastData)) {
      stored = SetStoreValue(m_data);
      m_lastData = m_data;
    }
    TunableRegistry::ResetChangedAfterUpdate(GetUid());
    m_forceUpdate = false;
    return stored ? &m_lastData : nullptr;
  }

 protected:
  virtual bool Encode(const TunableRegistry::TunableInfo& info, std::vector<uint8_t>& out) = 0;
  virtual bool DoUpdateTunable(const TunableRegistry::TunableInfo& info,
                               std::span<const uint8_t> data) = 0;

 private:
  std::vector<uint8_t> m_data;
  std::vector<uint8_t> m_lastData;
  bool m_forceUpdate = false;
};

namespace {

template <typename T>
class ValueTunableEntry final : public ProbotTunableBackend::ValueEntry {
 public:
  using ValueEntry::ValueEntry;

 private:
  bool Encode(const TunableRegistry::TunableInfo& info, std::vector<uint8_t>& out) override {
    Codec<T>::Encode(GetValue<T>(info), out);
    return true;
  }

  bool DoUpdateTunable(const TunableRegistry::TunableInfo& info,
                       std::span<const uint8_t> data) override {
    T value{};
    if (!Codec<T>::Decode(data, value)) {
      return false;
    }
    SetValue<T>(info, std::move(value));
    return true;
  }
};

class StructTunableEntry final : public ProbotTunableBackend::ValueEntry {
 public:
  using ValueEntry::ValueEntry;

 private:
  bool Encode(const TunableRegistry::TunableInfo& info, std::vector<uint8_t>& out) override {
    if (info.type == detail::TunableTypeValue::STRUCT) {
      auto tunable = static_cast<detail::TunableStructBase*>(info.tunable);
      PublishSchemas(*tunable);
      out.resize(tunable->GetStructSize());
      tunable->PackStruct(out);
    } else {
      auto tunable = static_cast<detail::TunableMemberStructBase*>(info.tunable);
      PublishSchemas(*tunable);
      out.resize(tunable->GetStructSize(info.config->parent));
      tunable->PackStruct(info.config->parent, out);
    }
    return true;
  }

  bool DoUpdateTunable(const TunableRegistry::TunableInfo& info,
                       std::span<const uint8_t> data) override {
    try {
      if (info.type == detail::TunableTypeValue::STRUCT) {
        return static_cast<detail::TunableStructBase*>(info.tunable)->UnpackStruct(data);
      } else {
        return static_cast<detail::TunableMemberStructBase*>(info.tunable)
            ->UnpackStruct(info.config->parent, data);
      }
    } catch (const std::exception& e) {
      ReportWarning("rejected struct tune payload", e);
      return false;
    } catch (...) {
      ReportWarning("rejected struct tune payload");
      return false;
    }
  }

  template <typename B>
  void PublishSchemas(B& tunable) {
    if (m_schemaPublished) {
      return;
    }
    tunable.ForEachStructSchema([](std::string_view typeName, std::string_view schema) {
      dash::AddSchema(typeName, "structschema",
                      {reinterpret_cast<const uint8_t*>(schema.data()), schema.size()});
    });
    m_schemaPublished = true;
  }

  bool m_schemaPublished = false;
};

class ProtobufTunableEntry final : public ProbotTunableBackend::ValueEntry {
 public:
  using ValueEntry::ValueEntry;

 private:
  bool Encode(const TunableRegistry::TunableInfo& info, std::vector<uint8_t>& out) override {
    if (info.type == detail::TunableTypeValue::PROTOBUF) {
      auto tunable = static_cast<detail::TunableProtobufBase*>(info.tunable);
      PublishSchemas(*tunable);
      return tunable->PackProtobuf(out);
    }
    auto tunable = static_cast<detail::TunableMemberProtobufBase*>(info.tunable);
    PublishSchemas(*tunable);
    return tunable->PackProtobuf(info.config->parent, out);
  }

  bool DoUpdateTunable(const TunableRegistry::TunableInfo& info,
                       std::span<const uint8_t> data) override {
    bool updated = false;
    try {
      if (info.type == detail::TunableTypeValue::PROTOBUF) {
        updated = static_cast<detail::TunableProtobufBase*>(info.tunable)->UnpackProtobuf(data);
      } else {
        updated = static_cast<detail::TunableMemberProtobufBase*>(info.tunable)
                      ->UnpackProtobuf(info.config->parent, data);
      }
    } catch (const std::exception& e) {
      ReportWarning("rejected protobuf tune payload", e);
      return false;
    } catch (...) {
      ReportWarning("rejected protobuf tune payload");
      return false;
    }
    if (!updated) {
      ReportWarning("rejected protobuf tune payload");
    }
    return updated;
  }

  template <typename B>
  void PublishSchemas(B& tunable) {
    if (m_schemaPublished) {
      return;
    }
    tunable.ForEachProtobufDescriptor(
        [](std::string_view filename) { return dash::HasSchema(filename); },
        [](std::string_view filename, std::string_view descriptor) {
          dash::AddSchema(filename, "proto:FileDescriptorProto",
                          {reinterpret_cast<const uint8_t*>(descriptor.data()),
                           descriptor.size()});
        });
    m_schemaPublished = true;
  }

  bool m_schemaPublished = false;
};

class ComplexTunableEntry final : public ProbotTunableBackend::Entry {
 public:
  ComplexTunableEntry(std::string key, std::string path, uint32_t uid, std::string tunableType)
      : Entry{std::move(key),
              std::move(path),
              uid,
              ValueType::kString,
              "string",
              R"({"mutable":false})",
              dash::kFlagTunable,
              false},
        m_tunableType{std::move(tunableType)} {}

  const std::vector<uint8_t>* UpdateNetwork(bool force) override {
    if (force && IsPublished()) {
      SetStoreValue({reinterpret_cast<const uint8_t*>(m_tunableType.data()),
                     m_tunableType.size()});
    }
    TunableRegistry::ResetChangedAfterUpdate(GetUid());
    return nullptr;  // never persistent
  }

 private:
  std::string m_tunableType;
};

template <typename T>
std::shared_ptr<ProbotTunableBackend::Entry> MakeValueEntry(std::string_view key,
                                                            std::string path, uint32_t uid,
                                                            const TunableConfig* config,
                                                            ValueType valueType,
                                                            std::string_view typeString) {
  return std::make_shared<T>(std::string{key}, std::move(path), uid, valueType,
                             std::string{GetTypeString(typeString, config)},
                             GetProperties(config).to_string(), GetFlags(config),
                             IsPersistent(config));
}

}  // namespace

ProbotTunableBackend::ProbotTunableBackend(std::string_view prefix) : m_prefix{prefix} {
  dash::AddParticipant(this);
}

void ProbotTunableBackend::TrackEntry(const std::shared_ptr<Entry>& entry,
                                      const TunableConfig* config,
                                      detail::TunableTypeValue type) {
  using enum detail::TunableTypeValue;
  if (type == COMPLEX || type == MEMBER_COMPLEX) {
    entry->SetUpdateKind(Entry::UpdateKind::COMPLEX);
    m_complexEntries.emplace_back(entry);
  } else if (IsAlwaysGet(config)) {
    entry->SetUpdateKind(Entry::UpdateKind::ALWAYS_GET);
    m_alwaysGetEntries.emplace_back(entry);
  } else {
    entry->SetUpdateKind(Entry::UpdateKind::DIRTY);
  }
}

void ProbotTunableBackend::UntrackEntry(const std::shared_ptr<Entry>& entry) {
  if (m_updateDepth > 0) {
    m_deferredUntracks.emplace_back(entry);
    return;
  }
  UntrackEntryNow(entry);
}

void ProbotTunableBackend::UntrackEntryNow(const std::shared_ptr<Entry>& entry) {
  entry->SetDirtyQueued(false);
  std::erase(m_dirtyEntries, entry);
  std::erase(m_alwaysGetEntries, entry);
  std::erase(m_complexEntries, entry);
}

void ProbotTunableBackend::EnqueueDirtyEntry(const std::shared_ptr<Entry>& entry) {
  if (entry->IsRemoved() || entry->GetUpdateKind() != Entry::UpdateKind::DIRTY ||
      entry->IsDirtyQueued()) {
    return;
  }
  entry->SetDirtyQueued(true);
  m_dirtyEntries.emplace_back(entry);
}

void ProbotTunableBackend::RetireEntry(const std::shared_ptr<Entry>& entry) {
  if (entry->IsRemoved()) {
    return;
  }
  UntrackEntry(entry);
  entry->MarkRemoved();
  if (auto it = m_topics.find(entry->GetTopicId());
      it != m_topics.end() && it->second.get() == entry->AsValueEntry()) {
    m_topics.erase(it);
  }
  std::erase(m_waiting, entry);
  entry->Close();
}

void ProbotTunableBackend::RemovePublishedEntry(std::string path,
                                                const std::shared_ptr<Entry>& entry) {
  auto uid = entry->GetUid();
  RetireEntry(entry);
  if (auto uidIt = m_uids.find(uid); uidIt != m_uids.end()) {
    std::erase(uidIt->second, path);
    if (uidIt->second.empty()) {
      m_uids.erase(uidIt);
    }
  }
  EraseEntry(std::move(path), entry);
}

void ProbotTunableBackend::EraseEntry(std::string path, const std::shared_ptr<Entry>& entry) {
  if (m_updateDepth > 0) {
    m_deferredErases.emplace_back(std::move(path), entry);
    return;
  }
  auto it = m_entries.find(path);
  if (it != m_entries.end() && it->second == entry) {
    m_entries.erase(it);
  }
}

void ProbotTunableBackend::ApplyDeferredErases() {
  if (m_clearTrackedEntries) {
    ClearTrackedEntries();
    m_deferredErases.clear();
    m_deferredUntracks.clear();
    m_clearTrackedEntries = false;
    return;
  }

  for (auto&& pending : m_deferredErases) {
    auto it = m_entries.find(pending.path);
    if (it != m_entries.end() && it->second == pending.entry) {
      m_entries.erase(it);
    }
  }
  m_deferredErases.clear();

  for (auto&& entry : m_deferredUntracks) {
    UntrackEntryNow(entry);
  }
  m_deferredUntracks.clear();
}

void ProbotTunableBackend::ClearTrackedEntries() {
  for (auto&& entry : m_dirtyEntries) {
    entry->SetDirtyQueued(false);
  }
  m_dirtyEntries.clear();
  m_alwaysGetEntries.clear();
  m_complexEntries.clear();
}

ProbotTunableBackend::~ProbotTunableBackend() {
  Retire();
}

void ProbotTunableBackend::Retire() {
  // no more Driver Station writes or flushes reach this backend (takes the routing lock, so it
  // is never called with m_mutex held)
  dash::RemoveParticipant(this);
  std::scoped_lock lock{m_mutex};
  if (m_retired) {
    return;
  }
  m_retired = true;
  for (auto&& [path, entry] : m_entries) {
    RetireEntry(entry);
  }
  m_entries.clear();
  m_uids.clear();
  m_topics.clear();
  m_pendingWrites.clear();
  m_persisted.clear();
  m_persistWarned.clear();
  m_waiting.clear();
  m_hasWaiting.store(false);
  m_deferredErases.clear();
  if (m_updateDepth > 0) {
    m_deferredUntracks.clear();
    m_clearTrackedEntries = true;
  } else {
    ClearTrackedEntries();
  }
}

bool ProbotTunableBackend::Publish(std::string_view path, uint32_t uid,
                                   detail::TunableBase& tunable, const TunableConfig* config,
                                   detail::TunableTypeValue type) {
  std::shared_ptr<Entry> entry;
  bool restore = false;
  {
    std::scoped_lock lock{m_mutex};
    if (m_retired) {
      return false;
    }
    if (auto it = m_entries.find(path); it != m_entries.end() && !it->second->IsRemoved()) {
      TunableRegistry::ReportWarning(
          std::format("Dashboard tunable '{}{}' already exists", m_prefix, path));
      return false;
    }

    std::string storePath = std::format("{}{}", m_prefix, path);
    using enum detail::TunableTypeValue;
    switch (type) {
      case BOOLEAN:
      case MEMBER_BOOLEAN:
        entry = MakeValueEntry<ValueTunableEntry<bool>>(path, std::move(storePath), uid, config,
                                                        ValueType::kBoolean, "boolean");
        break;
      case INT32:
      case MEMBER_INT32:
        entry = MakeValueEntry<ValueTunableEntry<int32_t>>(path, std::move(storePath), uid,
                                                           config, ValueType::kInt, "int");
        break;
      case INT64:
      case MEMBER_INT64:
        entry = MakeValueEntry<ValueTunableEntry<int64_t>>(path, std::move(storePath), uid,
                                                           config, ValueType::kInt, "int");
        break;
      case FLOAT:
      case MEMBER_FLOAT:
        entry = MakeValueEntry<ValueTunableEntry<float>>(path, std::move(storePath), uid, config,
                                                         ValueType::kFloat, "float");
        break;
      case DOUBLE:
      case MEMBER_DOUBLE:
        entry = MakeValueEntry<ValueTunableEntry<double>>(path, std::move(storePath), uid,
                                                          config, ValueType::kDouble, "double");
        break;
      case STRING:
      case MEMBER_STRING:
        entry = MakeValueEntry<ValueTunableEntry<std::string>>(
            path, std::move(storePath), uid, config, ValueType::kString, "string");
        break;
      case RAW:
      case MEMBER_RAW:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<uint8_t>>>(
            path, std::move(storePath), uid, config, ValueType::kRaw, "raw");
        break;
      case BOOLEAN_ARRAY:
      case MEMBER_BOOLEAN_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<bool>>>(
            path, std::move(storePath), uid, config, ValueType::kBooleanArray, "boolean[]");
        break;
      case INT32_ARRAY:
      case MEMBER_INT32_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<int32_t>>>(
            path, std::move(storePath), uid, config, ValueType::kIntArray, "int[]");
        break;
      case INT64_ARRAY:
      case MEMBER_INT64_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<int64_t>>>(
            path, std::move(storePath), uid, config, ValueType::kIntArray, "int[]");
        break;
      case FLOAT_ARRAY:
      case MEMBER_FLOAT_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<float>>>(
            path, std::move(storePath), uid, config, ValueType::kFloatArray, "float[]");
        break;
      case DOUBLE_ARRAY:
      case MEMBER_DOUBLE_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<double>>>(
            path, std::move(storePath), uid, config, ValueType::kDoubleArray, "double[]");
        break;
      case STRING_ARRAY:
      case MEMBER_STRING_ARRAY:
        entry = MakeValueEntry<ValueTunableEntry<std::vector<std::string>>>(
            path, std::move(storePath), uid, config, ValueType::kStringArray, "string[]");
        break;
      case STRUCT:
        entry = MakeValueEntry<StructTunableEntry>(
            path, std::move(storePath), uid, config, ValueType::kRaw,
            std::format("struct:{}",
                        static_cast<detail::TunableStructBase&>(tunable).GetStructTypeName()));
        break;
      case MEMBER_STRUCT:
        entry = MakeValueEntry<StructTunableEntry>(
            path, std::move(storePath), uid, config, ValueType::kRaw,
            std::format(
                "struct:{}",
                static_cast<detail::TunableMemberStructBase&>(tunable).GetStructTypeName()));
        break;
      case PROTOBUF:
        entry = MakeValueEntry<ProtobufTunableEntry>(
            path, std::move(storePath), uid, config, ValueType::kRaw,
            static_cast<detail::TunableProtobufBase&>(tunable).GetProtobufTypeString());
        break;
      case MEMBER_PROTOBUF:
        entry = MakeValueEntry<ProtobufTunableEntry>(
            path, std::move(storePath), uid, config, ValueType::kRaw,
            static_cast<detail::TunableMemberProtobufBase&>(tunable).GetProtobufTypeString());
        break;
      case COMPLEX:
      case MEMBER_COMPLEX:
        entry = std::make_shared<ComplexTunableEntry>(
            std::string{path}, std::format("{}/.type", storePath), uid,
            std::string{static_cast<ComplexTunable&>(tunable).GetTunableType()});
        break;
      default:
        TunableRegistry::ReportWarning(std::format("Unsupported tunable type for path {}", path));
        return false;
    }

    auto result = entry->Open();
    if (result.id == dash::kInvalidTopic && !result.notReady) {
      return false;  // the store refused the topic (reported once by PublishTopic)
    }
    restore = entry->IsCorePersistent();
    WarnIfNotPersistent(*entry);

    entry->SetInsertEpoch(++m_nextInsertEpoch);
    m_entries[path] = entry;
    if (auto valueEntry = entry->AsValueEntry(); valueEntry && entry->IsPublished()) {
      m_topics[entry->GetTopicId()] = std::static_pointer_cast<ValueEntry>(entry);
    }
    if (result.notReady) {
      m_waiting.emplace_back(entry);
      m_hasWaiting.store(true, std::memory_order_release);
    }
    m_uids[uid].emplace_back(path);
    TrackEntry(entry, config, type);
  }

  bool tunedInitialValue = false;
  try {
    // a persistent tunable takes its stored value now, before Publish returns (upstream applies a
    // robust tunable's retained value here the same way)
    if (restore) {
      dash::DrainWrites();
      tunedInitialValue = ApplyRestoredValue(entry);
    }
    if (auto stored = entry->UpdateNetwork(true)) {
      RememberValue(*entry, *stored);
    }
  } catch (...) {
    {
      std::scoped_lock lock{m_mutex};
      if (!m_retired) {
        RemovePublishedEntry(std::string{path}, entry);
      }
    }
    try {
      throw;
    } catch (const std::exception& e) {
      TunableRegistry::ReportWarning(
          std::format("Dashboard tunable '{}{}' failed during initial publish: {}", m_prefix,
                      path, e.what()));
    } catch (...) {
      TunableRegistry::ReportWarning(std::format(
          "Dashboard tunable '{}{}' failed during initial publish", m_prefix, path));
    }
    return false;
  }
  if (tunedInitialValue) {
    auto info = TunableRegistry::GetTunable(entry->GetUid());
    if (info) {
      NotifyOnTune(entry->GetUid(), info);
    }
  }
  return true;
}

bool ProbotTunableBackend::ApplyRestoredValue(const std::shared_ptr<Entry>& entry) {
  auto valueEntry = entry->AsValueEntry();
  const uint16_t id = entry->GetTopicId();
  std::optional<std::vector<uint8_t>> value;
  bool fromCore = false;
  {
    std::scoped_lock lock{m_mutex};
    if (m_retired || entry->IsRemoved() || !valueEntry) {
      return false;
    }
    // what core restored: the write for the new topic in the drain that followed its publish
    std::optional<std::vector<uint8_t>> restored;
    for (auto it = m_pendingWrites.begin(); it != m_pendingWrites.end(); ++it) {
      if (it->id == id) {
        restored = std::move(it->data);
        m_pendingWrites.erase(it);
        break;
      }
    }
    SyncPersistClears();
    auto [memo, first] = m_persisted.try_emplace(entry->GetKey());
    if (!first && memo->second.valueType == entry->GetValueType() &&
        memo->second.typeString == entry->GetTypeString()) {
      // published again in this run: the last value set on the path is what core stores (its
      // topic's current value, whoever set it), but core's record may not have it yet (2 s
      // debounce, saved only while the robot is idle), so the remembered value wins and core's
      // restore is dropped (not refused: that would erase the record)
      value = memo->second.lastValue;
    } else {
      memo->second = PersistedTunable{entry->GetValueType(), entry->GetTypeString(), {}};
      value = std::move(restored);
      fromCore = true;
    }
  }
  if (!value) {
    return false;
  }
  if (valueEntry->HoldsValue(*value)) {
    // the stored or remembered value is the one it already has (e.g. its own last set value or
    // its code default): nothing to tune, no onTune (core's restore is dropped, not refused)
    return false;
  }
  // the Driver Station write path: decode, set, RecordTuneApplied, onRemoteSet (the store gets
  // the value from the UpdateNetwork that follows, which remembers it)
  if (valueEntry->UpdateTunable(*value)) {
    return true;
  }
  if (fromCore) {
    // the stored value does not decode any more: core erases the record (no frame to the DS)
    dash::RejectWrite(id, WriteStatus::kDecodeError);
  }
  return false;
}

void ProbotTunableBackend::RememberValue(const Entry& entry, std::span<const uint8_t> data) {
  if (!entry.IsCorePersistent()) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  RememberValueLocked(entry, data);
}

void ProbotTunableBackend::RememberValueLocked(const Entry& entry,
                                               std::span<const uint8_t> data) {
  // the value core now holds for the path (set by the robot program, a getter, a Driver Station
  // write echoed back or a restore); an entry removed meanwhile no longer owns the path
  if (!entry.IsCorePersistent() || entry.IsRemoved()) {
    return;
  }
  SyncPersistClears();
  if (auto it = m_persisted.find(entry.GetKey());
      it != m_persisted.end() && it->second.valueType == entry.GetValueType() &&
      it->second.typeString == entry.GetTypeString()) {
    // assigned in place: a getter that changes every loop reuses the buffer
    auto& last = it->second.lastValue;
    if (last) {
      last->assign(data.begin(), data.end());
    } else {
      last.emplace(data.begin(), data.end());
    }
  }
}

void ProbotTunableBackend::SyncPersistClears() {
  const uint32_t clears = dash::PersistClearCount();
  if (clears == m_persistClears) {
    return;
  }
  m_persistClears = clears;
  // ClearPersistedTunables() erased core's records and core stores a topic again only once its
  // value changes: a value set before the clear must not come back on a republish either (the
  // path then keeps the new tunable's own value, which its publish sets and core then stores)
  for (auto&& [key, memo] : m_persisted) {
    memo.lastValue.reset();
  }
}

void ProbotTunableBackend::WarnIfNotPersistent(const Entry& entry) {
  if (!entry.WantsPersistence() || entry.IsCorePersistent() || !entry.IsPublished() ||
      !m_persistWarned.try_emplace(entry.GetKey(), true).second) {
    return;
  }
  // once per path: a republish (OpMode change) does not repeat it
  TunableRegistry::ReportWarning(std::format(
      "Dashboard tunable '{}{}': properties.persistent: this probot-core does not keep dashboard "
      "values across reboots; the tunable starts from its code value",
      m_prefix, entry.GetKey()));
}

void ProbotTunableBackend::PublishWaiting() {
  if (!m_hasWaiting.load(std::memory_order_acquire) || !dash::StoreReady()) {
    return;
  }
  std::vector<std::shared_ptr<Entry>> opened;
  bool restore = false;
  {
    std::scoped_lock lock{m_mutex};
    if (m_retired) {
      return;
    }
    m_hasWaiting.store(false, std::memory_order_release);
    std::vector<std::shared_ptr<Entry>> waiting;
    waiting.swap(m_waiting);
    for (auto&& entry : waiting) {
      if (entry->IsRemoved()) {
        continue;
      }
      auto result = entry->Open();
      if (result.id != dash::kInvalidTopic) {
        if (entry->AsValueEntry()) {
          m_topics[result.id] = std::static_pointer_cast<ValueEntry>(entry);
        }
        restore = restore || entry->IsCorePersistent();
        WarnIfNotPersistent(*entry);
        opened.emplace_back(entry);
      } else if (result.notReady) {
        m_waiting.emplace_back(entry);
        m_hasWaiting.store(true, std::memory_order_release);
      } else {
        // refused (reported once): the tunable stays registered but is not shown
        RemovePublishedEntry(entry->GetKey(), entry);
      }
    }
  }
  // persistent tunables take their stored values before their first publish, as in Publish()
  if (restore) {
    dash::DrainWrites();
  }
  std::vector<uint32_t> tunedUids;
  for (auto&& entry : opened) {
    try {
      if (entry->IsCorePersistent() && ApplyRestoredValue(entry)) {
        tunedUids.emplace_back(entry->GetUid());
      }
      if (auto stored = entry->UpdateNetwork(true)) {
        RememberValue(*entry, *stored);
      }
    } catch (const std::exception& e) {
      TunableRegistry::ReportWarning(std::format(
          "Dashboard tunable '{}{}' failed during initial publish: {}", m_prefix,
          entry->GetKey(), e.what()));
    } catch (...) {
      TunableRegistry::ReportWarning(std::format(
          "Dashboard tunable '{}{}' failed during initial publish", m_prefix, entry->GetKey()));
    }
  }
  for (auto uid : tunedUids) {
    auto info = TunableRegistry::GetTunable(uid);
    if (info) {
      NotifyOnTune(uid, info);
    }
  }
}

void ProbotTunableBackend::MarkDirty(uint32_t uid) {
  std::scoped_lock lock{m_mutex};
  if (m_retired) {
    return;
  }
  auto uidIt = m_uids.find(uid);
  if (uidIt == m_uids.end()) {
    return;
  }
  for (auto&& path : uidIt->second) {
    if (auto entryIt = m_entries.find(path);
        entryIt != m_entries.end() && !entryIt->second->IsRemoved()) {
      EnqueueDirtyEntry(entryIt->second);
    }
  }
}

void ProbotTunableBackend::Remove(std::string_view path) {
  std::scoped_lock lock{m_mutex};
  if (m_retired) {
    return;
  }
  auto it = m_entries.find(path);
  if (it != m_entries.end() && !it->second->IsRemoved()) {
    RemovePublishedEntry(std::string{it->first}, it->second);
  }
}

std::vector<TunableBackend::PublishedTunable> ProbotTunableBackend::RemovePrefix(
    std::string_view prefix) {
  std::string prefixBuf;
  prefix = prosdk::tunables::detail::NormalizePrefix(prefix, prefixBuf);
  std::scoped_lock lock{m_mutex};
  std::vector<PublishedTunable> removed;
  if (m_retired) {
    return removed;
  }
  for (auto it = m_entries.begin(); it != m_entries.end();) {
    if (it->second->IsRemoved() ||
        !prosdk::tunables::detail::IsPathOrDescendant(it->first, prefix)) {
      ++it;
      continue;
    }
    std::string path{it->first};
    auto entry = it->second;
    ++it;
    removed.push_back({path, entry->GetUid()});
    RemovePublishedEntry(std::move(path), entry);
  }
  return removed;
}

void ProbotTunableBackend::UnregisterTunable(uint32_t uid) {
  std::scoped_lock lock{m_mutex};
  if (m_retired) {
    return;
  }
  auto it = m_uids.find(uid);
  if (it != m_uids.end()) {
    for (auto&& path : it->second) {
      if (auto entryIt = m_entries.find(path);
          entryIt != m_entries.end() && !entryIt->second->IsRemoved()) {
        auto entry = entryIt->second;
        RetireEntry(entry);
        EraseEntry(path, entry);
      }
    }
    m_uids.erase(it);
  }
}

std::optional<WriteStatus> ProbotTunableBackend::OfferWrite(uint16_t id, ValueType valueType,
                                                            std::span<const uint8_t> data) {
  std::scoped_lock lock{m_mutex};
  if (m_retired) {
    return std::nullopt;
  }
  auto it = m_topics.find(id);
  if (it == m_topics.end()) {
    return std::nullopt;
  }
  auto& entry = it->second;
  if (!entry->IsMutable()) {
    return WriteStatus::kImmutable;
  }
  if (entry->GetValueType() != valueType) {
    return WriteStatus::kTypeMismatch;
  }
  // last write per topic wins until the next Update()
  for (auto&& pending : m_pendingWrites) {
    if (pending.id == id) {
      pending.entry = entry;
      pending.data.assign(data.begin(), data.end());
      return WriteStatus::kOk;
    }
  }
  m_pendingWrites.push_back({entry, id, {data.begin(), data.end()}});
  return WriteStatus::kOk;
}

void ProbotTunableBackend::FlushPending() {
  if (!m_hasWaiting.load(std::memory_order_acquire)) {
    return;
  }
  // tunable values are only read under the registry's update mutex; skip this cycle if busy
  std::unique_lock updateLock{TunableRegistry::GetUpdateMutex(), std::try_to_lock};
  if (updateLock.owns_lock()) {
    PublishWaiting();
  }
}

void ProbotTunableBackend::Update() {
  dash::DrainWrites();
  PublishWaiting();

  std::vector<PendingWrite> writes;
  {
    std::scoped_lock lock{m_mutex};
    if (m_retired) {
      return;
    }
    writes.swap(m_pendingWrites);
  }

  std::vector<uint32_t> onTuneUids;
  for (auto&& write : writes) {
    auto entry = write.entry.lock();
    if (!entry || entry->IsRemoved()) {
      dash::RejectWrite(write.id, WriteStatus::kUnknownId);
      continue;
    }
    if (entry->UpdateTunable(write.data)) {
      // remembered for a republish once the update below echoes it to the store, as core stores
      // the topic's value (a Driver Station write alone does not change it)
      onTuneUids.emplace_back(entry->GetUid());
    } else {
      dash::RejectWrite(write.id, WriteStatus::kDecodeError);
    }
  }

  std::unique_lock lock{m_mutex};
  ++m_updateDepth;
  auto finishUpdate = [&] {
    if (!lock.owns_lock()) {
      lock.lock();
    }
    --m_updateDepth;
    if (m_updateDepth == 0) {
      ApplyDeferredErases();
    }
  };
  try {
    uint64_t updateEpoch = m_nextInsertEpoch;
    auto updateEntry = [&](const std::shared_ptr<Entry>& entry,
                           bool requeueIfInsertedDuringUpdate) {
      if (entry->IsRemoved() || entry->GetInsertEpoch() > updateEpoch) {
        if (!entry->IsRemoved() && requeueIfInsertedDuringUpdate) {
          EnqueueDirtyEntry(entry);
        }
        return;
      }

      lock.unlock();
      auto stored = entry->UpdateNetwork(false);
      lock.lock();
      if (stored) {
        RememberValueLocked(*entry, *stored);
      }
    };

    auto updateEntries = [&](auto& entries, bool requeueIfInsertedDuringUpdate) {
      size_t size = entries.size();
      for (size_t i = 0; i < size; ++i) {
        auto entry = entries[i];
        updateEntry(entry, requeueIfInsertedDuringUpdate);
      }
    };

    updateEntries(m_complexEntries, false);
    updateEntries(m_alwaysGetEntries, false);

    std::vector<std::shared_ptr<Entry>> dirtyEntries;
    dirtyEntries.swap(m_dirtyEntries);
    for (auto&& entry : dirtyEntries) {
      entry->SetDirtyQueued(false);
    }
    for (auto&& entry : dirtyEntries) {
      updateEntry(entry, true);
    }
  } catch (...) {
    if (!lock.owns_lock()) {
      lock.lock();
    }
    finishUpdate();
    lock.unlock();
    try {
      throw;
    } catch (const std::exception& e) {
      TunableRegistry::ReportWarning(
          std::format("Dashboard tunable backend update failed: {}", e.what()));
    } catch (...) {
      TunableRegistry::ReportWarning("Dashboard tunable backend update failed");
    }
    return;
  }
  finishUpdate();
  lock.unlock();

  // onTune callbacks can publish or remove tunables, which re-enters this backend. Run them after
  // releasing m_mutex so those mutations are safe.
  for (auto uid : onTuneUids) {
    auto info = TunableRegistry::GetTunable(uid);
    if (info) {
      NotifyOnTune(uid, info);
    }
  }
}
