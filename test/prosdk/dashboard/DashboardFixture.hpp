// Copyright (c) Probot Studio. Hand-written; see LICENSE.

#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "FakeDashboardStore.hpp"
#include "prosdk/platform/Console.hpp"
#include "prosdk/platform/dashboard/DashboardStore.hpp"
#include "prosdk/platform/dashboard/ProbotTelemetryBackend.hpp"
#include "prosdk/platform/dashboard/ProbotTunableBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"

namespace prosdk::backend {
class ProbotTunableBackendTestAccess {
 public:
  static int GetUpdateDepth(const ProbotTunableBackend& backend) { return backend.m_updateDepth; }
  static size_t GetEntryCount(const ProbotTunableBackend& backend) {
    return backend.m_entries.size();
  }
  static size_t GetUidPathCount(const ProbotTunableBackend& backend) {
    size_t count = 0;
    for (const auto& entry : backend.m_uids) {
      count += entry.second.size();
    }
    return count;
  }
  static size_t GetDeferredEraseCount(const ProbotTunableBackend& backend) {
    return backend.m_deferredErases.size();
  }
  static size_t GetDirtyEntryCount(const ProbotTunableBackend& backend) {
    return backend.m_dirtyEntries.size();
  }
  static size_t GetAlwaysGetEntryCount(const ProbotTunableBackend& backend) {
    return backend.m_alwaysGetEntries.size();
  }
  static size_t GetTopicCount(const ProbotTunableBackend& backend) {
    return backend.m_topics.size();
  }
};
}  // namespace prosdk::backend

namespace dashboard_test {

inline bool HasWarning(const std::vector<std::string>& warnings, std::string_view path,
                       std::string_view msg) {
  for (auto&& warning : warnings) {
    if (warning.find(path) != std::string::npos && warning.find(msg) != std::string::npos) {
      return true;
    }
  }
  return false;
}

inline size_t CountOf(std::string_view text, std::string_view needle) {
  size_t n = 0;
  for (size_t p = text.find(needle); p != std::string_view::npos;
       p = text.find(needle, p + needle.size())) {
    ++n;
  }
  return n;
}

/** Fresh fake store, registries and both backends registered for "". */
class DashboardFixture {
 public:
  DashboardFixture() {
    fake::Reset();
    prosdk::platform::dashboard::ResetForTest();
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::telemetry::TelemetryRegistry::SetReportWarning(nullptr);
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::tunables::TunableRegistry::SetReportWarning(nullptr);
    telemetry = std::make_shared<prosdk::backend::ProbotTelemetryBackend>("/Telemetry");
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", telemetry);
    tunables = std::make_shared<prosdk::backend::ProbotTunableBackend>("/Tunables");
    prosdk::tunables::TunableRegistry::RegisterBackend("", tunables);
  }

  ~DashboardFixture() {
    prosdk::telemetry::TelemetryRegistry::SetReportWarning(nullptr);
    prosdk::telemetry::TelemetryRegistry::Reset();
    prosdk::tunables::TunableRegistry::SetReportWarning(nullptr);
    prosdk::tunables::TunableRegistry::Reset();
    prosdk::platform::SetConsoleSink(nullptr);
  }

  /** What core's pre-cycle callback and the robot loop do: drain, flush, update tunables. */
  static void Cycle() {
    prosdk::platform::dashboard::DrainWrites();
    prosdk::platform::dashboard::FlushPending();
    prosdk::tunables::TunableRegistry::Update();
  }

  std::shared_ptr<prosdk::backend::ProbotTelemetryBackend> telemetry;
  std::shared_ptr<prosdk::backend::ProbotTunableBackend> tunables;
};

}  // namespace dashboard_test
