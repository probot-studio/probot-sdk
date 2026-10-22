// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/NetworkButton.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "prosdk/driverstation/RobotState.hpp"
#include "prosdk/tunables/Tunable.hpp"
#include "prosdk/tunables/Tunables.hpp"
#include "prosdk/tunables/detail/PathUtil.hpp"

using namespace prosdk::cmd;

// prosdk: re-implemented over tunables (ntcore is not ported).
namespace {

// One boolean tunable per path, shared by every NetworkButton (and Trigger
// copy) with that path.
using ButtonValue = prosdk::tunables::Tunable<bool>;

std::mutex buttonsMutex;
std::map<std::string, std::weak_ptr<ButtonValue>, std::less<>> buttons;

std::shared_ptr<ButtonValue> GetButtonValue(std::string_view path) {
  std::string buf;
  std::string key{prosdk::tunables::detail::NormalizeName(path, buf)};
  std::scoped_lock lock{buttonsMutex};
  auto it = buttons.find(key);
  if (it != buttons.end()) {
    if (auto value = it->second.lock()) {
      return value;
    }
  }
  auto value = std::shared_ptr<ButtonValue>(
      new ButtonValue{false}, [key](ButtonValue* ptr) {
        delete ptr;
        std::scoped_lock lock{buttonsMutex};
        auto it = buttons.find(key);
        if (it != buttons.end() && it->second.expired()) {
          buttons.erase(it);
        }
      });
  buttons.insert_or_assign(key, value);
  prosdk::tunables::Publish(key, *value);
  return value;
}

}  // namespace

NetworkButton::NetworkButton(std::string_view path)
    : Trigger([value = GetButtonValue(path)] {
        return prosdk::RobotState::IsDSAttached() && value->Get();
      }) {}

NetworkButton::NetworkButton(std::string_view table, std::string_view field)
    : NetworkButton(std::string{table}.append("/").append(field)) {}
