// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Console output for library diagnostics (errors, warnings, status lines).
//
// The precompiled archive never touches Serial (its type depends on board menu options,
// design/PORT.md section 7). Text goes to a sink function; the default writes to stderr. The
// probot-core bridge installs a leveled sink that forwards to the Driver Station console
// (probot-core R9 `probot::console`, with the level; an older core gets the text only). An
// optional tee receives a copy of every write (DataLogManager's console log).
//
// Every write carries a level: ReportError and HAL errors are kError, ReportWarning, MathShared
// warnings and the telemetry/tunables warning reporters kWarning, everything else kInfo.
//
// Context: callable from any task, never from an ISR. On the robot a write from an ISR is
// dropped (the sinks lock and allocate).

#pragma once

#include <stdint.h>

#include <string_view>

namespace prosdk::platform {

/** Severity of a console write (same values as probot-core's console::Level). */
enum class ConsoleLevel : uint8_t { kInfo = 0, kWarning = 1, kError = 2 };

/** Console sink: receives complete messages (normally ending in a newline), without level. */
using ConsoleSink = void (*)(std::string_view text);

/** Leveled console sink: receives the level and the message. */
using LeveledConsoleSink = void (*)(ConsoleLevel level, std::string_view text);

/**
 * Replaces the console sink with a sink that ignores the level; nullptr restores the default
 * (stderr). Also replaces a leveled sink. Thread-safe.
 */
void SetConsoleSink(ConsoleSink sink) noexcept;

/**
 * Replaces the console sink with a leveled sink; nullptr restores the default (stderr). Also
 * replaces a sink set by SetConsoleSink. Thread-safe.
 */
void SetLeveledConsoleSink(LeveledConsoleSink sink) noexcept;

/**
 * Installs a second receiver of every ConsoleWrite (DataLogManager::LogConsoleOutput uses it to
 * copy the console into the data log); nullptr removes it. Thread-safe. The tee runs on the
 * writing thread, after the sink: it must be short, must not block and must not write to the
 * console itself (nested writes are not passed to the tee again).
 */
void SetConsoleTee(ConsoleSink tee) noexcept;

/**
 * Writes text to the console sink, then to the tee if one is installed. Never throws. On the
 * robot, a call from an ISR is dropped.
 */
void ConsoleWrite(std::string_view text, ConsoleLevel level = ConsoleLevel::kInfo) noexcept;

}  // namespace prosdk::platform
