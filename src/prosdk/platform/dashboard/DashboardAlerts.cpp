// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/DashboardAlerts.hpp"

#include <stdint.h>

#include <algorithm>
#include <atomic>
#include <format>
#include <map>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "prosdk/telemetry/Telemetry.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/util/Alert.h"
#include "prosdk/util/mutex.hpp"

namespace prosdk::platform::dashboard {

namespace {

std::atomic<const WPI_AlertBackend*> gInner{nullptr};
std::atomic<bool> gChanged{true};
std::atomic<bool> gInstalled{false};

struct Published {
  prosdk::util::mutex mutex;
  std::map<std::string, bool, std::less<>> groups;  // groups published at least once
};

Published& GetPublished() {
  static Published published;
  return published;
}

const WPI_AlertBackend& Inner() {
  return *gInner.load(std::memory_order_acquire);
}

void MarkChanged() {
  gChanged.store(true, std::memory_order_release);
}

int32_t CreateAlert(const WPI_String* group, const WPI_String* id, const WPI_String* text,
                    int32_t level, WPI_AlertHandle* handle) {
  int32_t status = Inner().createAlert(group, id, text, level, handle);
  MarkChanged();
  return status;
}

void DestroyAlert(WPI_AlertHandle handle) {
  Inner().destroyAlert(handle);
  MarkChanged();
}

int32_t SetAlertActive(WPI_AlertHandle handle, int32_t active) {
  // Alert::Set() usually runs every loop: only a change of state is a change
  int32_t wasActive = 0;
  bool known = Inner().isAlertActive(handle, &wasActive) == 0;
  int32_t status = Inner().setAlertActive(handle, active);
  if (!known || (wasActive != 0) != (active != 0)) {
    MarkChanged();
  }
  return status;
}

int32_t IsAlertActive(WPI_AlertHandle handle, int32_t* active) {
  return Inner().isAlertActive(handle, active);
}

int32_t SetAlertText(WPI_AlertHandle handle, const WPI_String* text) {
  int32_t status = Inner().setAlertText(handle, text);
  MarkChanged();
  return status;
}

int32_t GetAlertText(WPI_AlertHandle handle, WPI_String* text) {
  return Inner().getAlertText(handle, text);
}

int32_t GetAlertLevel(WPI_AlertHandle handle, int32_t* level) {
  return Inner().getAlertLevel(handle, level);
}

int32_t GetNumAlerts() {
  return Inner().getNumAlerts();
}

int32_t GetAlerts(WPI_AlertInfo* arr, int32_t length) {
  return Inner().getAlerts(arr, length);
}

void FreeAlerts(WPI_AlertInfo* arr, int32_t length) {
  Inner().freeAlerts(arr, length);
}

void ResetAlertData() {
  Inner().resetAlertData();
  MarkChanged();
}

constexpr WPI_AlertBackend kTrackingBackend{
    CreateAlert,   DestroyAlert,  SetAlertActive, IsAlertActive, SetAlertText, GetAlertText,
    GetAlertLevel, GetNumAlerts,  GetAlerts,      FreeAlerts,    ResetAlertData};

std::string_view View(const WPI_String& s) {
  return {s.str ? s.str : "", s.len};
}

struct GroupAlerts {
  // (activeStartTime, text) per level
  std::vector<std::pair<int64_t, std::string>> levels[3];
};

void LogGroup(std::string_view group, GroupAlerts& alerts) {
  static constexpr std::string_view kNames[3] = {"errors", "warnings", "infos"};
  auto& table = prosdk::telemetry::GetTable("Alerts").GetTable(group);
  table.SetType("Alerts");
  for (int level = 0; level < 3; ++level) {
    auto& list = alerts.levels[level];
    // newest first; equal start times keep the backend's order
    std::stable_sort(list.begin(), list.end(),
                     [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<std::string> texts;
    texts.reserve(list.size());
    for (auto& [time, text] : list) {
      texts.emplace_back(std::move(text));
    }
    table.Log(kNames[level], std::span<const std::string>{texts});
  }
}

}  // namespace

void TrackAlerts() noexcept {
  if (gInstalled.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  const WPI_AlertBackend* inner = WPI_GetAlertBackend();
  if (inner == &kTrackingBackend || inner == nullptr) {
    return;
  }
  gInner.store(inner, std::memory_order_release);
  WPI_SetAlertBackend(&kTrackingBackend);
  MarkChanged();
}

void PublishAlerts() noexcept {
  if (!gChanged.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  try {
    std::vector<WPI_AlertInfo> infos;
    int32_t count = WPI_GetNumAlerts();
    if (count < 0) {
      return;
    }
    for (;;) {
      infos.resize(static_cast<size_t>(count));
      int32_t total = WPI_GetAlerts(infos.data(), count);
      if (total < 0) {
        return;
      }
      if (total <= count) {
        infos.resize(static_cast<size_t>(total));
        break;
      }
      WPI_FreeAlerts(infos.data(), count);  // grew meanwhile: read again
      count = total;
    }

    std::map<std::string, GroupAlerts, std::less<>> groups;
    for (const auto& info : infos) {
      auto& group = groups[std::string{View(info.group)}];
      if (info.activeStartTime != 0 && info.level >= 0 && info.level < 3) {
        group.levels[info.level].emplace_back(info.activeStartTime, std::string{View(info.text)});
      }
    }
    WPI_FreeAlerts(infos.data(), static_cast<int32_t>(infos.size()));

    auto& published = GetPublished();
    std::scoped_lock lock{published.mutex};
    for (auto& [name, alerts] : groups) {
      published.groups[name] = true;
      LogGroup(name, alerts);
    }
    // groups whose alerts are all gone: publish them empty
    for (auto& [name, seen] : published.groups) {
      if (groups.find(name) == groups.end()) {
        GroupAlerts empty;
        LogGroup(name, empty);
      }
    }
  } catch (...) {
    MarkChanged();  // out of memory: try again next cycle
  }
}

void ResetAlertsForTest() noexcept {
  auto& published = GetPublished();
  std::scoped_lock lock{published.mutex};
  published.groups.clear();
  MarkChanged();
}

}  // namespace prosdk::platform::dashboard
