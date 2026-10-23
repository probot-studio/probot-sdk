// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// StartTelemetryLog() in its own file: only DataLogManager references it, so only a sketch that
// starts DataLogManager links the log backend and the MultiTelemetryBackend pair.

#include <initializer_list>
#include <memory>
#include <utility>

#include "prosdk/backend/DataLogTelemetryBackend.hpp"
#include "prosdk/platform/TelemetryLog.hpp"
#include "prosdk/telemetry/MultiTelemetryBackend.hpp"

namespace prosdk::platform {

void StartTelemetryLog(prosdk::log::DataLog& log) {
  using prosdk::telemetry::TelemetryBackend;
  detail::StartTelemetryLog(
      log, std::make_shared<prosdk::backend::DataLogTelemetryBackend>(log, kTelemetryLogPrefix),
      [](std::shared_ptr<TelemetryBackend> defaultBackend,
         std::shared_ptr<TelemetryBackend> logBackend) -> std::shared_ptr<TelemetryBackend> {
        return std::make_shared<prosdk::telemetry::MultiTelemetryBackend>(
            std::initializer_list<std::shared_ptr<TelemetryBackend>>{std::move(defaultBackend),
                                                                     std::move(logBackend)});
      });
}

}  // namespace prosdk::platform
