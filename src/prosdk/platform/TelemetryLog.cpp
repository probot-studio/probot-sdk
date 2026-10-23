// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/TelemetryLog.hpp"

#include <initializer_list>
#include <mutex>
#include <utility>

#include "prosdk/backend/DataLogTelemetryBackend.hpp"
#include "prosdk/telemetry/MultiTelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryBackend.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"

namespace prosdk::platform {
namespace {

struct State {
  std::mutex mutex;
  std::shared_ptr<prosdk::telemetry::TelemetryBackend> defaultBackend;
  std::shared_ptr<prosdk::telemetry::TelemetryBackend> logBackend;
  prosdk::log::DataLog* log = nullptr;
};

State& GetState() {
  static State state;
  return state;
}

// with state.mutex held (the registry does not call back into this file)
void Apply(State& state) {
  if (!state.defaultBackend) {
    return;
  }
  if (state.logBackend) {
    prosdk::telemetry::TelemetryRegistry::RegisterBackend(
        "", std::make_shared<prosdk::telemetry::MultiTelemetryBackend>(
                std::initializer_list<std::shared_ptr<prosdk::telemetry::TelemetryBackend>>{
                    state.defaultBackend, state.logBackend}));
  } else {
    prosdk::telemetry::TelemetryRegistry::RegisterBackend("", state.defaultBackend);
  }
}

}  // namespace

void SetDefaultTelemetryBackend(std::shared_ptr<prosdk::telemetry::TelemetryBackend> backend) {
  auto& state = GetState();
  std::scoped_lock lock{state.mutex};
  state.defaultBackend = std::move(backend);
  Apply(state);
}

void StartTelemetryLog(prosdk::log::DataLog& log) {
  auto& state = GetState();
  std::scoped_lock lock{state.mutex};
  if (state.log == &log) {
    return;
  }
  state.log = &log;
  state.logBackend = std::make_shared<prosdk::backend::DataLogTelemetryBackend>(
      log, kTelemetryLogPrefix);
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
