// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/dashboard/DashboardPreferences.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "prosdk/platform/PreferencesStore.hpp"
#include "prosdk/platform/dashboard/ProbotTunableBackend.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableConfig.hpp"
#include "prosdk/tunables/TunableRegistry.hpp"
#include "prosdk/tunables/Tunables.hpp"

namespace prosdk::platform::dashboard {
namespace {

namespace prefs = prosdk::platform::prefs;
namespace tunables = prosdk::tunables;

// the upstream NetworkTables table of the preferences
constexpr const char* kTable = "/Preferences";

std::string PathOf(const std::string& key) {
  return std::string{kTable} + "/" + key;
}

// equal values; NaN equals NaN (no republish, no flash write for a NaN preference)
template <typename T>
bool Same(const T& a, const T& b) {
  if constexpr (std::is_floating_point_v<T>) {
    return a == b || (a != a && b != b);
  } else {
    return a == b;
  }
}

// one preference as a tunable
class Mirror {
 public:
  virtual ~Mirror() = default;
  /** Index of the prefs::Value alternative this mirror holds. */
  virtual size_t Index() const = 0;
  /** Sets the tunable to the store's value (same alternative) if it differs. */
  virtual void Update(const prefs::Value& value) = 0;
};

template <typename T>
class MirrorOf final : public Mirror {
 public:
  MirrorOf(const std::string& key, T value) : m_tunable{std::move(value), MakeConfig(key)} {
    tunables::Publish(PathOf(key), m_tunable);
  }

  size_t Index() const override { return prefs::Value{std::in_place_type<T>}.index(); }

  void Update(const prefs::Value& value) override {
    const T& v = std::get<T>(value);
    if (!Same(m_tunable.Get(), v)) {
      m_tunable.Set(T{v});
    }
  }

 private:
  static tunables::TunableConfig MakeConfig(const std::string& key) {
    tunables::TunableConfig config;
    // one topic flagged ROBUST, like upstream's persistent preference topics
    config.robust = true;
    // a Driver Station edit goes to the store (persisted); the next sync then finds the
    // tunable and the store equal
    config.onTune = [key](tunables::detail::TunableBase& tunable, tunables::ComplexTunable*) {
      prefs::Set(key, prefs::Value{
                          static_cast<tunables::detail::TunableValueBase<T>&>(tunable).Get()});
    };
    return config;
  }

  tunables::Tunable<T> m_tunable;
};

std::unique_ptr<Mirror> MakeMirror(const std::string& key, const prefs::Value& value) {
  return std::visit(
      [&](const auto& v) -> std::unique_ptr<Mirror> {
        using T = std::decay_t<decltype(v)>;
        return std::make_unique<MirrorOf<T>>(key, v);
      },
      value);
}

struct State {
  bool active = false;
  bool synced = false;  // revision is valid
  uint64_t revision = 0;
  std::map<std::string, std::unique_ptr<Mirror>, std::less<>> mirrors;
};

State& GetState() {
  static State state;
  return state;
}

}  // namespace

void TrackPreferences() {
  std::scoped_lock lock{tunables::TunableRegistry::GetUpdateMutex()};
  tunables::TunableRegistry::RegisterBackend(
      kTable, std::make_shared<prosdk::backend::ProbotTunableBackend>(""));
  auto& state = GetState();
  state.mirrors.clear();
  state.active = true;
  state.synced = false;
  tunables::detail::SetTunableRegistryPreUpdateCallback([] { SyncPreferences(); });
}

void SyncPreferences() {
  std::scoped_lock lock{tunables::TunableRegistry::GetUpdateMutex()};
  auto& state = GetState();
  if (!state.active) {
    return;
  }
  // the revision is read before the entries: a change in between bumps it again, so the next
  // sync sees it
  uint64_t revision = prefs::Revision();
  if (state.synced && revision == state.revision) {
    return;
  }
  auto entries = prefs::Entries();
  state.revision = revision;
  state.synced = true;

  // removed keys: destroying the tunable unregisters it and unpublishes the topic
  auto e = entries.begin();
  for (auto it = state.mirrors.begin(); it != state.mirrors.end();) {
    while (e != entries.end() && e->first < it->first) {
      ++e;
    }
    if (e == entries.end() || e->first != it->first) {
      it = state.mirrors.erase(it);
    } else {
      ++it;
    }
  }
  for (auto& [key, value] : entries) {
    auto it = state.mirrors.find(key);
    if (it != state.mirrors.end()) {
      if (it->second->Index() == value.index()) {
        it->second->Update(value);
        continue;
      }
      // the type changed (a Set of another type replaces it): new topic
      state.mirrors.erase(it);
    }
    state.mirrors.emplace(key, MakeMirror(key, value));
  }
}

void ResetPreferencesForTest() {
  std::scoped_lock lock{tunables::TunableRegistry::GetUpdateMutex()};
  auto& state = GetState();
  state.mirrors.clear();
  state.active = false;
  state.synced = false;
  tunables::detail::SetTunableRegistryPreUpdateCallback(nullptr);
}

}  // namespace prosdk::platform::dashboard
