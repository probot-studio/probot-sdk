// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/platform/hal/CoreMatchInfo.hpp"

#include <cstring>

namespace prosdk::platform {

static_assert(HAL_ALLIANCE_STATION_UNKNOWN == 0 && HAL_ALLIANCE_STATION_RED_1 == 1 &&
                  HAL_ALLIANCE_STATION_BLUE_1 == 4 && HAL_ALLIANCE_STATION_BLUE_3 == 6,
              "CoreMatchInfo::station is numbered like HAL_AllianceStationID");
static_assert(HAL_MATCH_TYPE_NONE == 0 && HAL_MATCH_TYPE_PRACTICE == 1 &&
                  HAL_MATCH_TYPE_QUALIFICATION == 2 && HAL_MATCH_TYPE_ELIMINATION == 3,
              "CoreMatchInfo::type is numbered like HAL_MatchType");
static_assert(sizeof(HAL_GameData{}.gameData) == kCoreMatchInfoMaxText + 1 &&
                  sizeof(HAL_MatchInfo{}.eventName) == kCoreMatchInfoMaxText + 1,
              "HAL match text fields hold kCoreMatchInfoMaxText bytes and a NUL");

void ConvertCoreMatchInfo(const CoreMatchInfo& in, CoreMatchData* out) noexcept {
  std::memset(out, 0, sizeof(*out));
  out->station = in.station <= HAL_ALLIANCE_STATION_BLUE_3
                     ? static_cast<HAL_AllianceStationID>(in.station)
                     : HAL_ALLIANCE_STATION_UNKNOWN;
  out->info.matchType = in.type <= HAL_MATCH_TYPE_ELIMINATION ? static_cast<HAL_MatchType>(in.type)
                                                              : HAL_MATCH_TYPE_NONE;
  out->info.matchNumber = in.matchNumber;
  out->info.replayNumber = in.replayNumber;
  // CopyCoreMatchInfo already cut the bridge's text; this covers a table filled another way (a
  // field without its NUL)
  CopyCoreMatchText(in.eventName, sizeof(in.eventName), out->info.eventName);
  CopyCoreMatchText(in.gameData, sizeof(in.gameData), out->gameData.gameData);
}

bool RefreshCoreMatchData(const CoreOps& ops, uint32_t* seq, CoreMatchData* out) noexcept {
  const uint32_t now = ops.matchInfoSeq();
  if (now == *seq) {
    return false;
  }
  CoreMatchInfo info{};
  ops.matchInfo(&info);
  ConvertCoreMatchInfo(info, out);
  *seq = now;
  return true;
}

}  // namespace prosdk::platform
