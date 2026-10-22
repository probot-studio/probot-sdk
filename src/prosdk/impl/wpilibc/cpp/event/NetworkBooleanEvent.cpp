// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/event/NetworkBooleanEvent.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/TunableTable.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/tunables/detail/PathUtil.hpp"
#include "prosdk/util/mutex.hpp"

using namespace prosdk;

// prosdk: the NetworkTables boolean topic is a boolean tunable (ntcore is not
// ported); "connected" is a Driver Station being attached.
namespace {
using BooleanTunable = prosdk::tunables::Tunable<bool>;

// one tunable per name, shared by the events created from that name
std::shared_ptr<BooleanTunable> GetNamedTunable(std::string_view name) {
  static prosdk::util::mutex mutex;
  static std::map<std::string, std::weak_ptr<BooleanTunable>, std::less<>>
      tunables;
  std::string buf;
  std::string_view path = prosdk::tunables::detail::NormalizeName(name, buf);
  std::scoped_lock lock{mutex};
  auto& weak = tunables[std::string{path}];
  if (auto tunable = weak.lock()) {
    return tunable;
  }
  auto tunable = std::make_shared<BooleanTunable>(false);
  prosdk::tunables::Publish(path, *tunable);
  weak = tunable;
  return tunable;
}

std::function<bool()> MakeSignal(std::shared_ptr<BooleanTunable> tunable) {
  return [tunable = std::move(tunable)] {
    return RobotState::IsDSAttached() && tunable->Get();
  };
}
}  // namespace

NetworkBooleanEvent::NetworkBooleanEvent(EventLoop* loop,
                                         std::string_view name)
    : BooleanEvent{loop, MakeSignal(GetNamedTunable(name))} {}

NetworkBooleanEvent::NetworkBooleanEvent(
    EventLoop* loop, prosdk::tunables::Tunable<bool> tunable)
    : BooleanEvent{loop, MakeSignal(std::make_shared<BooleanTunable>(
                             std::move(tunable)))} {}

NetworkBooleanEvent::NetworkBooleanEvent(EventLoop* loop,
                                         prosdk::tunables::TunableTable table,
                                         std::string_view topicName)
    : NetworkBooleanEvent{loop, table.GetPath(), topicName} {}

NetworkBooleanEvent::NetworkBooleanEvent(EventLoop* loop,
                                         std::string_view tableName,
                                         std::string_view topicName)
    : NetworkBooleanEvent{
          loop, std::string{tableName} + "/" + std::string{topicName}} {}
