// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <concepts>
#include <memory>
#include <utility>

#include "prosdk/commands2/Command.hpp"
#include "prosdk/commands2/CommandPtr.hpp"
#include "prosdk/util/type_name.hpp"

namespace prosdk::cmd {

/**
 * CRTP implementation to allow polymorphic decorator functions in Command.
 *
 * <p>Note: ALWAYS create a subclass by extending CommandHelper<Base, Subclass>,
 * or decorators will not function!
 *
 * prosdk: built without RTTI. A command constructed through CommandHelper is
 * named after its CRTP class (prosdk::util::GetTypeName<CRTP>(), for example
 * "prosdk::cmd::WaitCommand" or "DriveDistance"); the name is set before the
 * subclass constructor body runs, so SetName() there still wins. Copies and
 * moves keep the name of their source.
 *
 * This class is provided by the Commands v2 VendorDep
 */
template <std::derived_from<Command> Base, typename CRTP>
class CommandHelper : public Base {
  using Base::Base;

 public:
  CommandHelper() = default;

  CommandPtr ToPtr() && override {
    return CommandPtr(
        std::make_unique<CRTP>(std::move(*static_cast<CRTP*>(this))));
  }

 private:
  // prosdk: names the command after CRTP at construction (also through the
  // inherited constructors); copying it does nothing.
  struct TypeName {
    explicit TypeName(Command* command) {
      command->SetName(prosdk::util::GetTypeName<CRTP>());
    }
    TypeName(const TypeName&) {}
    TypeName& operator=(const TypeName&) { return *this; }
  };
  [[no_unique_address]] TypeName m_prosdkTypeName{this};
};
}  // namespace prosdk::cmd
