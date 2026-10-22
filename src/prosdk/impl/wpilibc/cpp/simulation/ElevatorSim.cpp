// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/ElevatorSim.hpp"

#include "prosdk/math/system/Models.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/MathExtras.hpp"

using namespace prosdk;
using namespace prosdk::sim;

ElevatorSim::ElevatorSim(const prosdk::math::LinearSystem<2, 1, 2>& plant,
                         const prosdk::math::DCMotor& gearbox,
                         prosdk::units::meter_t minHeight,
                         prosdk::units::meter_t maxHeight, bool simulateGravity,
                         prosdk::units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : LinearSystemSim(plant, measurementStdDevs),
      m_gearbox(gearbox),
      m_minHeight(minHeight),
      m_maxHeight(maxHeight),
      m_simulateGravity(simulateGravity) {
  SetState(startingHeight, 0_mps);
}

ElevatorSim::ElevatorSim(const prosdk::math::DCMotor& gearbox, double gearing,
                         prosdk::units::kilogram_t carriageMass,
                         prosdk::units::meter_t drumRadius,
                         prosdk::units::meter_t minHeight,
                         prosdk::units::meter_t maxHeight, bool simulateGravity,
                         prosdk::units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : ElevatorSim(prosdk::math::Models::ElevatorFromPhysicalConstants(
                      gearbox, carriageMass, drumRadius, gearing),
                  gearbox, minHeight, maxHeight, simulateGravity,
                  startingHeight, measurementStdDevs) {}

template <typename Distance>
  requires std::same_as<prosdk::units::meter, Distance> ||
           std::same_as<prosdk::units::radian, Distance>
ElevatorSim::ElevatorSim(decltype(1_V / Velocity_t<Distance>(1)) kV,
                         decltype(1_V / Acceleration_t<Distance>(1)) kA,
                         const prosdk::math::DCMotor& gearbox,
                         prosdk::units::meter_t minHeight,
                         prosdk::units::meter_t maxHeight, bool simulateGravity,
                         prosdk::units::meter_t startingHeight,
                         const std::array<double, 2>& measurementStdDevs)
    : ElevatorSim(prosdk::math::Models::ElevatorFromSysId(kV, kA), gearbox,
                  minHeight, maxHeight, simulateGravity, startingHeight,
                  measurementStdDevs) {}

void ElevatorSim::SetState(prosdk::units::meter_t position,
                           prosdk::units::meters_per_second_t velocity) {
  SetState(prosdk::math::Vectord<2>{std::clamp(position, m_minHeight, m_maxHeight),
                                 velocity});
}

bool ElevatorSim::WouldHitLowerLimit(prosdk::units::meter_t elevatorHeight) const {
  return elevatorHeight <= m_minHeight;
}

bool ElevatorSim::WouldHitUpperLimit(prosdk::units::meter_t elevatorHeight) const {
  return elevatorHeight >= m_maxHeight;
}

bool ElevatorSim::HasHitLowerLimit() const {
  return WouldHitLowerLimit(prosdk::units::meter_t{m_y(0)});
}

bool ElevatorSim::HasHitUpperLimit() const {
  return WouldHitUpperLimit(prosdk::units::meter_t{m_y(0)});
}

prosdk::units::meter_t ElevatorSim::GetPosition() const {
  return prosdk::units::meter_t{m_y(0)};
}

prosdk::units::meters_per_second_t ElevatorSim::GetVelocity() const {
  return prosdk::units::meters_per_second_t{m_x(1)};
}

prosdk::units::ampere_t ElevatorSim::GetCurrentDraw() const {
  // I = V / R - omega / (Kv * R)
  // Reductions are greater than 1, so a reduction of 10:1 would mean the motor
  // is spinning 10x faster than the output.

  double kA = 1.0 / m_plant.B(1, 0);
  using Kv_t = prosdk::units::unit_t<prosdk::units::compound_unit<
      prosdk::units::volt, prosdk::units::inverse<prosdk::units::meters_per_second>>>;
  Kv_t Kv = Kv_t{-kA * m_plant.A(1, 1)};
  prosdk::units::meters_per_second_t velocity{m_x(1)};
  prosdk::units::radians_per_second_t motorVelocity = velocity * Kv * m_gearbox.Kv;

  // Perform calculation and return.
  return m_gearbox.Current(motorVelocity, prosdk::units::volt_t{m_u(0)}) *
         prosdk::util::sgn(m_u(0));
}

void ElevatorSim::SetInputVoltage(prosdk::units::volt_t voltage) {
  SetInput(prosdk::math::Vectord<1>{voltage.value()});
  ClampInput(prosdk::RobotController::GetBatteryVoltage().value());
}

prosdk::math::Vectord<2> ElevatorSim::UpdateX(
    const prosdk::math::Vectord<2>& currentXhat, const prosdk::math::Vectord<1>& u,
    prosdk::units::second_t dt) {
  auto updatedXhat = prosdk::math::RKDP(
      [&](const prosdk::math::Vectord<2>& x,
          const prosdk::math::Vectord<1>& u_) -> prosdk::math::Vectord<2> {
        prosdk::math::Vectord<2> xdot = m_plant.A() * x + m_plant.B() * u;

        if (m_simulateGravity) {
          xdot += prosdk::math::Vectord<2>{0.0, -9.8};
        }
        return xdot;
      },
      currentXhat, u, dt);
  // Check for collision after updating x-hat.
  if (WouldHitLowerLimit(prosdk::units::meter_t{updatedXhat(0)})) {
    return prosdk::math::Vectord<2>{m_minHeight.value(), 0.0};
  }
  if (WouldHitUpperLimit(prosdk::units::meter_t{updatedXhat(0)})) {
    return prosdk::math::Vectord<2>{m_maxHeight.value(), 0.0};
  }
  return updatedXhat;
}
