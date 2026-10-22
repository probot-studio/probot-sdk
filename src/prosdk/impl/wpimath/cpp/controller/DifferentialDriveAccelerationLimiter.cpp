// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/math/controller/DifferentialDriveAccelerationLimiter.hpp"

#include <prosdk/thirdparty/eigen/Eigen/QR>

#include "prosdk/math/controller/DifferentialDriveWheelVoltages.hpp"
#include "prosdk/math/linalg/EigenCore.hpp"
#include "prosdk/units/velocity.hpp"
#include "prosdk/units/voltage.hpp"

using namespace prosdk::math;

DifferentialDriveWheelVoltages DifferentialDriveAccelerationLimiter::Calculate(
    prosdk::units::meters_per_second_t leftVelocity,
    prosdk::units::meters_per_second_t rightVelocity,
    prosdk::units::volt_t leftVoltage, prosdk::units::volt_t rightVoltage) {
  Vectord<2> u{leftVoltage.value(), rightVoltage.value()};

  // Find unconstrained wheel accelerations
  Vectord<2> x{leftVelocity.value(), rightVelocity.value()};
  Vectord<2> dxdt = m_system.A() * x + m_system.B() * u;

  // Convert from wheel accelerations to linear and angular accelerations
  //
  // a = (dxdt(0) + dx/dt(1)) / 2
  //   = 0.5 dxdt(0) + 0.5 dxdt(1)
  //
  // α = (dxdt(1) - dxdt(0)) / trackwidth
  //   = -1/trackwidth dxdt(0) + 1/trackwidth dxdt(1)
  //
  // [a] = [          0.5           0.5][dxdt(0)]
  // [α]   [-1/trackwidth  1/trackwidth][dxdt(1)]
  //
  // accels = M dxdt where M = [0.5, 0.5; -1/trackwidth, 1/trackwidth]
  Matrixd<2, 2> M{{0.5, 0.5},
                  {-1.0 / m_trackwidth.value(), 1.0 / m_trackwidth.value()}};
  Vectord<2> accels = M * dxdt;

  // Constrain the linear and angular accelerations
  if (accels(0) > m_maxLinearAccel.value()) {
    accels(0) = m_maxLinearAccel.value();
  } else if (accels(0) < m_minLinearAccel.value()) {
    accels(0) = m_minLinearAccel.value();
  }
  if (accels(1) > m_maxAngularAccel.value()) {
    accels(1) = m_maxAngularAccel.value();
  } else if (accels(1) < -m_maxAngularAccel.value()) {
    accels(1) = -m_maxAngularAccel.value();
  }

  // Convert the constrained linear and angular accelerations back to wheel
  // accelerations
  dxdt = M.householderQr().solve(accels);

  // Find voltages for the given wheel accelerations
  //
  // dx/dt = Ax + Bu
  // u = B⁻¹(dx/dt - Ax)
  u = m_system.B().householderQr().solve(dxdt - m_system.A() * x);

  return {prosdk::units::volt_t{u(0)}, prosdk::units::volt_t{u(1)}};
}
