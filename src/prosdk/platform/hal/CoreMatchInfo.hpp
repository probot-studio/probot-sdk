// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// probot-core match information (R18, CoreOps ABI 9) -> WPILib HAL alliance station, game data
// and match info (design/hal-esp32.md, "Driver Station"). Used by the ESP32 Driver Station
// backend's pump; compiled on the host as well so the conversion is unit-tested against a fake
// CoreOps table.

#pragma once

#include <stdint.h>

#include "prosdk/hal/DriverStationTypes.h"
#include "prosdk/platform/CoreOps.hpp"

namespace prosdk::platform {

/** The match fields of one Driver Station snapshot, as the HAL getters return them. */
struct CoreMatchData {
  HAL_AllianceStationID station;
  HAL_GameData gameData;
  HAL_MatchInfo info;
};

/**
 * Converts core's match info into the HAL types (`out` is cleared first): a station outside 0-6
 * is unknown, a match type outside 0-3 is none; the event name and game data are copied up to
 * their NUL, at most kCoreMatchInfoMaxText (64) bytes, cut at a UTF-8 character boundary.
 */
void ConvertCoreMatchInfo(const CoreMatchInfo& in, CoreMatchData* out) noexcept;

/**
 * Re-reads core's match info when `ops.matchInfoSeq()` differs from `*seq` (the counter is read
 * before the values, so a change in between is seen again on the next call). Returns true and
 * updates `*out` and `*seq` when it did; false (nothing read, nothing copied) otherwise. With a
 * core without R18 the counter stays 0, so a `*seq` and `*out` that start zeroed (unknown station,
 * no game data, match type none) are never changed.
 */
bool RefreshCoreMatchData(const CoreOps& ops, uint32_t* seq, CoreMatchData* out) noexcept;

}  // namespace prosdk::platform
