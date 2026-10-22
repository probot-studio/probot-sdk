// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/AlertSim.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "prosdk/util/Alert.h"
#include "prosdk/util/string.hpp"

using namespace prosdk;
using namespace prosdk::sim;

int32_t AlertSim::GetCount() {
  // prosdk: int32_t is long on Xtensa, so the literal must not decide the type
  return std::max<int32_t>(WPI_GetNumAlerts(), 0);
}

std::vector<AlertSim::AlertInfo> AlertSim::GetAll() {
  int32_t allocLen = WPI_GetNumAlerts();
  if (allocLen <= 0) {
    return {};
  }
  WPI_AlertInfo* cInfos = new WPI_AlertInfo[allocLen];
  int32_t len = WPI_GetAlerts(cInfos, allocLen);
  if (len <= 0) {
    delete[] cInfos;
    return {};
  }
  int32_t count = len < allocLen ? len : allocLen;
  std::vector<AlertInfo> infos;
  infos.reserve(count);
  for (int32_t i = 0; i < count; ++i) {
    const auto& cInfo = cInfos[i];
    infos.emplace_back(std::string{prosdk::util::to_string_view(&cInfo.group)},
                       std::string{prosdk::util::to_string_view(&cInfo.id)},
                       std::string{prosdk::util::to_string_view(&cInfo.text)},
                       cInfo.activeStartTime,
                       static_cast<prosdk::util::Alert::Level>(cInfo.level));
  }
  WPI_FreeAlerts(cInfos, count);
  delete[] cInfos;
  return infos;
}

std::vector<AlertSim::AlertInfo> AlertSim::GetActive() {
  auto infos = GetAll();
  std::erase_if(infos, [](const AlertInfo& info) { return !info.isActive(); });
  return infos;
}

void AlertSim::ResetData() {
  WPI_ResetAlertData();
}
