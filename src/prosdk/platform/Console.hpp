// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Console output for library diagnostics (warnings from telemetry, tunables, ...).
//
// The precompiled archive never touches Serial (its type depends on board menu options,
// design/PORT.md section 7). Text goes to a sink function; the default writes to stderr. The
// probot-core bridge installs a sink that forwards to the Driver Station console.

#pragma once

#include <string_view>

namespace prosdk::platform {

/** Console sink: receives complete messages (normally ending in a newline). */
using ConsoleSink = void (*)(std::string_view text);

/** Replaces the console sink; nullptr restores the default (stderr). Thread-safe. */
void SetConsoleSink(ConsoleSink sink) noexcept;

/** Writes text to the console sink. Never throws. */
void ConsoleWrite(std::string_view text) noexcept;

}  // namespace prosdk::platform
