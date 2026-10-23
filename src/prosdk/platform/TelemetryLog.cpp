// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/TelemetryLog.hpp"

#include <mutex>
#include <span>
#include <utility>

#include "prosdk/telemetry/DiscardTelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"

namespace prosdk::platform {
namespace {

using prosdk::telemetry::TelemetryBackend;

class DefaultBackend;

// the default backend whose removals this thread ignores while Apply() registers (thread_local:
// std::this_thread::get_id() asserts in a FreeRTOS task that pthread did not create)
thread_local const DefaultBackend* tKeep = nullptr;

// The default backend as registered for "" (alone or inside the pair). The registry calls
// RemoveEntry() on the backend it replaces for every path it routed there; when Apply() swaps the
// default backend for the pair or back, the default backend keeps those paths, so the removals
// that swap makes on this thread are ignored: the entries (Driver Station topics) stay published
// with their last value, also those that are only logged on change. Removals from other threads
// and from a swap that drops this backend are passed on.
class DefaultBackend final : public TelemetryBackend {
 public:
  explicit DefaultBackend(std::shared_ptr<TelemetryBackend> backend)
      : m_backend{std::move(backend)} {}

  std::shared_ptr<prosdk::telemetry::TelemetryEntry> GetEntry(std::string_view path) override {
    return m_backend->GetEntry(path);
  }

  void RemoveEntry(std::string_view path) override {
    if (tKeep != this) {
      m_backend->RemoveEntry(path);
    }
  }

  bool HasSchema(std::string_view schemaName) const override {
    return m_backend->HasSchema(schemaName);
  }

  void AddSchema(std::string_view schemaName, std::string_view type,
                 std::span<const uint8_t> schema) override {
    m_backend->AddSchema(schemaName, type, schema);
  }

  void AddSchema(std::string_view schemaName, std::string_view type,
                 std::string_view schema) override {
    m_backend->AddSchema(schemaName, type, schema);
  }

  const std::shared_ptr<TelemetryBackend>& Backend() const { return m_backend; }

 private:
  std::shared_ptr<TelemetryBackend> m_backend;
};

struct State {
  std::mutex mutex;
  std::shared_ptr<DefaultBackend> defaultBackend;
  std::shared_ptr<TelemetryBackend> logBackend;
  // set by StartTelemetryLog() (TelemetryLogStart.cpp), so a sketch without DataLogManager does
  // not link the pair or the log backend
  detail::ComposeTelemetryBackends compose = nullptr;
  prosdk::log::DataLog* log = nullptr;
  bool registered = false;
};

State& GetState() {
  static State state;
  return state;
}

// with state.mutex held (the registry does not call back into this file)
void Apply(State& state) {
  std::shared_ptr<TelemetryBackend> backend;
  if (state.defaultBackend) {
    backend = state.logBackend ? state.compose(state.defaultBackend, state.logBackend)
                               : state.defaultBackend;
  } else if (state.registered) {
    // default backend cleared: neither it nor the log copy gets values any more
    backend = std::make_shared<prosdk::telemetry::DiscardTelemetryBackend>();
  } else {
    return;
  }
  state.registered = static_cast<bool>(state.defaultBackend);
  tKeep = state.defaultBackend.get();
  prosdk::telemetry::TelemetryRegistry::RegisterBackend("", std::move(backend));
  tKeep = nullptr;
}

}  // namespace

void SetDefaultTelemetryBackend(std::shared_ptr<prosdk::telemetry::TelemetryBackend> backend) {
  auto& state = GetState();
  std::scoped_lock lock{state.mutex};
  if (!backend) {
    state.defaultBackend.reset();
  } else if (!state.defaultBackend || state.defaultBackend->Backend() != backend) {
    state.defaultBackend = std::make_shared<DefaultBackend>(std::move(backend));
  }
  Apply(state);
}

void detail::StartTelemetryLog(prosdk::log::DataLog& log,
                               std::shared_ptr<prosdk::telemetry::TelemetryBackend> logBackend,
                               ComposeTelemetryBackends compose) {
  auto& state = GetState();
  std::scoped_lock lock{state.mutex};
  if (state.log == &log) {
    return;
  }
  state.log = &log;
  state.logBackend = std::move(logBackend);
  state.compose = compose;
  Apply(state);
}

void StopTelemetryLog(prosdk::log::DataLog& log) {
  auto& state = GetState();
  std::scoped_lock lock{state.mutex};
  if (state.log != &log) {
    return;
  }
  state.log = nullptr;
  state.logBackend.reset();
  Apply(state);
}

}  // namespace prosdk::platform
