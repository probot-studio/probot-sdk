// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Tunable backend on probot-core's dashboard topic store (the Driver Station page). The
// counterpart of upstream wpi::backend::NetworkTablesTunableBackend: every tunable becomes one
// store topic <prefix><path> (RegisterDashboardBackends() uses "/Tunables") with flag TUNABLE,
// MUTABLE when TunableConfig::isMutable (the default) and ROBUST when TunableConfig::robust, and
// the properties the NetworkTables backend sets ("mutable", "robust", TunableConfig::properties).
// A ComplexTunable publishes "<path>/.type" (its GetTunableType()) so the Driver Station picks the
// widget (PIDController, Selectable, Command, ...); its members are tunables of their own.
//
// Driver Station writes reach the backend through prosdk::platform::dashboard::DrainWrites()
// (run by the pre-cycle callback RegisterDashboardBackends() installs, and by Update()); they are
// applied in Update() (TunableRegistry::Update(), core's user task) exactly as the NetworkTables
// backend applies its tune events: type check, value set, RecordTuneApplied, onRemoteSet, onTune
// after the update. A refused write (type mismatch, immutable tunable, struct/protobuf payload
// that does not decode) is answered with dashRejectWrite.
//
// Differences from the NetworkTables backend (design/API-DIFFERENCES.md):
//   - a robust tunable is ONE topic with the ROBUST flag (no "<path>/value" + "<path>/tune" pair);
//     the store keeps no value across reboots, so a robust tunable starts from its code value;
//   - before probot-core's runtime_setup() ran, Publish() succeeds and the topic waits; it is
//     published by the next Update() or FlushPending();
//   - a topic the store refuses (topic table, pool, name length) makes Publish() return false
//     (the registry rolls the tunable back) and is reported once through the console;
//   - the "<path>/.type" topic of a ComplexTunable is read-only (no MUTABLE flag, properties
//     {"mutable":false}); NetworkTables publishes it with {"mutable":true} but never reads it.
//
// Threading: like every tunable backend, Publish/Remove/Update run under
// TunableRegistry::GetUpdateMutex() (the registry takes it); FlushPending() only works when it
// can take that mutex without waiting.

#pragma once

#include <stdint.h>

#include <atomic>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/tunables/TunableBackend.hpp"
#include "prosdk/util/StringMap.hpp"
#include "prosdk/util/mutex.hpp"

namespace prosdk::backend {

class ProbotTunableBackendTestAccess;

/** Tunable backend that publishes to probot-core's dashboard store. */
class ProbotTunableBackend final : public prosdk::tunables::TunableBackend,
                                   private prosdk::platform::dashboard::Participant {
  friend class ProbotTunableBackendTestAccess;

 public:
  class Entry;
  class ValueEntry;

  /**
   * Creates the backend.
   *
   * @param prefix topic path prefix (e.g. "/Tunables"); tunable paths start with '/'
   */
  explicit ProbotTunableBackend(std::string_view prefix);
  ~ProbotTunableBackend() override;

  void Retire() override;
  bool Publish(std::string_view path, uint32_t uid, prosdk::tunables::detail::TunableBase& tunable,
               const prosdk::tunables::TunableConfig* config,
               prosdk::tunables::detail::TunableTypeValue type) override;
  void MarkDirty(uint32_t uid) override;
  void Remove(std::string_view path) override;
  std::vector<PublishedTunable> RemovePrefix(std::string_view prefix) override;
  void UnregisterTunable(uint32_t uid) override;
  void Update() override;

 private:
  struct PendingErase {
    std::string path;
    std::shared_ptr<Entry> entry;
  };
  struct PendingWrite {
    std::weak_ptr<ValueEntry> entry;
    uint16_t id;
    std::vector<uint8_t> data;
  };

  std::optional<prosdk::platform::dashboard::WriteStatus> OfferWrite(
      uint16_t id, prosdk::platform::dashboard::ValueType valueType,
      std::span<const uint8_t> data) override;
  void FlushPending() override;

  void PublishWaiting();
  void TrackEntry(const std::shared_ptr<Entry>& entry,
                  const prosdk::tunables::TunableConfig* config,
                  prosdk::tunables::detail::TunableTypeValue type);
  void UntrackEntry(const std::shared_ptr<Entry>& entry);
  void EnqueueDirtyEntry(const std::shared_ptr<Entry>& entry);
  void UntrackEntryNow(const std::shared_ptr<Entry>& entry);
  void RetireEntry(const std::shared_ptr<Entry>& entry);
  void RemovePublishedEntry(std::string path, const std::shared_ptr<Entry>& entry);
  void EraseEntry(std::string path, const std::shared_ptr<Entry>& entry);
  void ClearTrackedEntries();
  void ApplyDeferredErases();

  std::string m_prefix;
  prosdk::util::mutex m_mutex;
  prosdk::util::StringMap<std::shared_ptr<Entry>> m_entries;
  std::unordered_map<uint32_t, std::vector<std::string>> m_uids;
  std::unordered_map<uint16_t, std::shared_ptr<ValueEntry>> m_topics;  // store id -> entry
  std::vector<PendingWrite> m_pendingWrites;
  std::vector<std::shared_ptr<Entry>> m_waiting;  // not published yet (store not ready)
  std::atomic<bool> m_hasWaiting{false};
  std::vector<std::shared_ptr<Entry>> m_dirtyEntries;
  std::vector<std::shared_ptr<Entry>> m_alwaysGetEntries;
  std::vector<std::shared_ptr<Entry>> m_complexEntries;
  std::vector<PendingErase> m_deferredErases;
  std::vector<std::shared_ptr<Entry>> m_deferredUntracks;
  int m_updateDepth = 0;
  uint64_t m_nextInsertEpoch = 0;
  bool m_clearTrackedEntries = false;
  bool m_retired = false;
};

}  // namespace prosdk::backend
