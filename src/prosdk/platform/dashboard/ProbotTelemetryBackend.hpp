// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Telemetry backend on probot-core's dashboard topic store (the Driver Station page). The
// counterpart of upstream wpi::backend::NetworkTablesTelemetryBackend: every telemetry path
// becomes one store topic <prefix><path> (RegisterDashboardBackends() uses "/Telemetry"), with
// the WPILib type string ("double", "int[]", "struct:Pose2d", "proto:...") and the entry's
// properties JSON. Schemas go to the store's schema table.
//
// Differences from the NetworkTables backend (design/API-DIFFERENCES.md):
//   - timestamps are ignored (the store keeps the last value only);
//   - before probot-core's runtime_setup() ran (static initialization), topics and the last value
//     of each entry wait here and are published by the next FlushPending();
//   - a topic the store refuses (topic table, pool, name length) is reported once through the
//     console and its entry becomes a discard entry; a value the store refuses (over
//     PROBOT_DASH_MAX_VALUE) is reported once and dropped.
//
// Thread-safe: entries may be logged from any task. Not for emergency-stop callbacks.

#pragma once

#include <stdint.h>

#include <atomic>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/telemetry/TelemetryBackend.hpp"
#include "prosdk/util/StringMap.hpp"
#include "prosdk/util/mutex.hpp"

namespace prosdk::backend {

/** Telemetry backend that publishes to probot-core's dashboard store. */
class ProbotTelemetryBackend final : public prosdk::telemetry::TelemetryBackend,
                                     private prosdk::platform::dashboard::Participant {
 public:
  class Entry;

  /**
   * Creates the backend.
   *
   * @param prefix topic path prefix (e.g. "/Telemetry"); telemetry paths start with '/'
   */
  explicit ProbotTelemetryBackend(std::string_view prefix);
  ~ProbotTelemetryBackend() override;

  std::shared_ptr<prosdk::telemetry::TelemetryEntry> GetEntry(std::string_view path) override;
  void RemoveEntry(std::string_view path) override;
  bool HasSchema(std::string_view schemaName) const override;
  void AddSchema(std::string_view schemaName, std::string_view type,
                 std::span<const uint8_t> schema) override;
  void AddSchema(std::string_view schemaName, std::string_view type,
                 std::string_view schema) override;

 private:
  void FlushPending() override;

  std::string m_prefix;
  prosdk::util::mutex m_mutex;
  prosdk::util::StringMap<std::shared_ptr<Entry>> m_entries;
  std::shared_ptr<std::atomic<bool>> m_anyPending;
};

}  // namespace prosdk::backend
