// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Console output for library diagnostics (warnings from telemetry, tunables, ...).
//
// The precompiled archive never touches Serial (its type depends on board menu options,
// design/PORT.md section 7). Text goes to a sink function; the default writes to stderr. The
// probot-core bridge installs a sink that forwards to the Driver Station console. An optional tee
// receives a copy of every write (DataLogManager's console log).

#pragma once

#include <string_view>

namespace prosdk::platform {

/** Console sink: receives complete messages (normally ending in a newline). */
using ConsoleSink = void (*)(std::string_view text);

/** Replaces the console sink; nullptr restores the default (stderr). Thread-safe. */
void SetConsoleSink(ConsoleSink sink) noexcept;

/**
 * Installs a second receiver of every ConsoleWrite (DataLogManager::LogConsoleOutput uses it to
 * copy the console into the data log); nullptr removes it. Thread-safe. The tee runs on the
 * writing thread, after the sink: it must be short, must not block and must not write to the
 * console itself (nested writes are not passed to the tee again).
 */
void SetConsoleTee(ConsoleSink tee) noexcept;

/** Writes text to the console sink, then to the tee if one is installed. Never throws. */
void ConsoleWrite(std::string_view text) noexcept;

}  // namespace prosdk::platform
