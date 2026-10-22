// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/DifferentialDriveFeedforward.hpp"

#include <prosdk/thirdparty/eigen/Eigen/Core>

#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/math/controller/LinearPlantInversionFeedforward.hpp"
#include "prosdk/units/time.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

using namespace prosdk::math;

DifferentialDriveWheelVoltages DifferentialDriveFeedforward::Calculate(
    prosdk::units::meters_per_second_t currentLeftVelocity,
    prosdk::units::meters_per_second_t nextLeftVelocity,
    prosdk::units::meters_per_second_t currentRightVelocity,
    prosdk::units::meters_per_second_t nextRightVelocity,
    prosdk::units::second_t dt) {
  prosdk::math::LinearPlantInversionFeedforward<2, 2> feedforward{m_plant, dt};

  Eigen::Vector2d r{currentLeftVelocity, currentRightVelocity};
  Eigen::Vector2d nextR{nextLeftVelocity, nextRightVelocity};
  auto u = feedforward.Calculate(r, nextR);
  return {prosdk::units::volt_t{u(0)}, prosdk::units::volt_t{u(1)}};
}
