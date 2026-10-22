// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "prosdk/hal/handles/IndexedClassedHandleResource.hpp"

#define HAL_TestHandle HAL_Handle

namespace {
class MyTestClass {};
}  // namespace

namespace prosdk::hal {
TEST_CASE("HandleTest ClassedHandle", "[hal][handles]") {
  prosdk::hal::IndexedClassedHandleResource<HAL_TestHandle, MyTestClass, 8,
                                         HAL_HandleEnum::VENDOR>
      testClass;
  auto resource =
      testClass.Allocate(0, std::make_shared<MyTestClass>(), "TestResource");
  CHECK(resource.has_value());
}

}  // namespace prosdk::hal
