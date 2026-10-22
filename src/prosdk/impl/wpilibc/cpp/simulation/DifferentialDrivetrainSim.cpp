// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/simulation/DifferentialDrivetrainSim.hpp"

#include <utility>

#include "prosdk/math/random/Normal.hpp"
#include "prosdk/math/system/Models.hpp"
#include "prosdk/math/system/NumericalIntegration.hpp"
#include "prosdk/math/util/StateSpaceUtil.hpp"
#include "prosdk/system/RobotController.hpp"
#include "prosdk/util/MathExtras.hpp"

using namespace prosdk;
using namespace prosdk::sim;

DifferentialDrivetrainSim::DifferentialDrivetrainSim(
    prosdk::math::LinearSystem<2, 2, 2> plant, prosdk::units::meter_t trackwidth,
    prosdk::math::DCMotor driveMotor, double gearRatio,
    prosdk::units::meter_t wheelRadius,
    const std::array<double, 7>& measurementStdDevs)
    : m_plant(std::move(plant)),
      m_rb(trackwidth / 2.0),
      m_wheelRadius(wheelRadius),
      m_motor(driveMotor),
      m_originalGearing(gearRatio),
      m_currentGearing(gearRatio),
      m_measurementStdDevs(measurementStdDevs) {
  m_x.setZero();
  m_u.setZero();
  m_y.setZero();
}

DifferentialDrivetrainSim::DifferentialDrivetrainSim(
    prosdk::math::DCMotor driveMotor, double gearing,
    prosdk::units::kilogram_square_meter_t J, prosdk::units::kilogram_t mass,
    prosdk::units::meter_t wheelRadius, prosdk::units::meter_t trackwidth,
    const std::array<double, 7>& measurementStdDevs)
    : DifferentialDrivetrainSim(
          prosdk::math::Models::DifferentialDriveFromPhysicalConstants(
              driveMotor, mass, wheelRadius, trackwidth / 2.0, J, gearing),
          trackwidth, driveMotor, gearing, wheelRadius, measurementStdDevs) {}

Eigen::Vector2d DifferentialDrivetrainSim::ClampInput(
    const Eigen::Vector2d& u) {
  return prosdk::math::DesaturateInputVector<2>(
      u, prosdk::RobotController::GetInputVoltage());
}

void DifferentialDrivetrainSim::SetInputs(prosdk::units::volt_t leftVoltage,
                                          prosdk::units::volt_t rightVoltage) {
  m_u << leftVoltage.value(), rightVoltage.value();
  m_u = ClampInput(m_u);
}

void DifferentialDrivetrainSim::SetGearing(double newGearing) {
  m_currentGearing = newGearing;
}

void DifferentialDrivetrainSim::Update(prosdk::units::second_t dt) {
  m_x = prosdk::math::RKDP([this](auto& x, auto& u) { return Dynamics(x, u); },
                        m_x, m_u, dt);
  m_y = m_x + prosdk::math::Normal<7>(m_measurementStdDevs);
}

double DifferentialDrivetrainSim::GetGearing() const {
  return m_currentGearing;
}

prosdk::math::Vectord<7> DifferentialDrivetrainSim::GetOutput() const {
  return m_y;
}

prosdk::math::Vectord<7> DifferentialDrivetrainSim::GetState() const {
  return m_x;
}

double DifferentialDrivetrainSim::GetOutput(int output) const {
  return m_y(output);
}

double DifferentialDrivetrainSim::GetState(int state) const {
  return m_x(state);
}

prosdk::math::Rotation2d DifferentialDrivetrainSim::GetHeading() const {
  return prosdk::units::radian_t{GetOutput(State::HEADING)};
}

prosdk::math::Pose2d DifferentialDrivetrainSim::GetPose() const {
  return prosdk::math::Pose2d{prosdk::units::meter_t{GetOutput(State::X)},
                           prosdk::units::meter_t{GetOutput(State::Y)},
                           GetHeading()};
}

prosdk::units::ampere_t DifferentialDrivetrainSim::GetLeftCurrentDraw() const {
  return m_motor.Current(
             prosdk::units::radians_per_second_t{m_x(State::LEFT_VELOCITY) *
                                              m_currentGearing /
                                              m_wheelRadius.value()},
             prosdk::units::volt_t{m_u(0)}) *
         prosdk::util::sgn(m_u(0));
}

prosdk::units::ampere_t DifferentialDrivetrainSim::GetRightCurrentDraw() const {
  return m_motor.Current(
             prosdk::units::radians_per_second_t{m_x(State::RIGHT_VELOCITY) *
                                              m_currentGearing /
                                              m_wheelRadius.value()},
             prosdk::units::volt_t{m_u(1)}) *
         prosdk::util::sgn(m_u(1));
}

prosdk::units::ampere_t DifferentialDrivetrainSim::GetCurrentDraw() const {
  return GetLeftCurrentDraw() + GetRightCurrentDraw();
}

void DifferentialDrivetrainSim::SetState(const prosdk::math::Vectord<7>& state) {
  m_x = state;
}

void DifferentialDrivetrainSim::SetPose(const prosdk::math::Pose2d& pose) {
  m_x(State::X) = pose.X().value();
  m_x(State::Y) = pose.Y().value();
  m_x(State::HEADING) = pose.Rotation().Radians().value();
  m_x(State::LEFT_POSITION) = 0;
  m_x(State::RIGHT_POSITION) = 0;
}

prosdk::math::Vectord<7> DifferentialDrivetrainSim::Dynamics(
    const prosdk::math::Vectord<7>& x, const Eigen::Vector2d& u) {
  // Because G² can be factored out of A, we can divide by the old ratio
  // squared and multiply by the new ratio squared to get a new drivetrain
  // model.
  prosdk::math::Matrixd<4, 2> B;
  B.block<2, 2>(0, 0) = m_plant.B() * m_currentGearing * m_currentGearing /
                        m_originalGearing / m_originalGearing;
  B.block<2, 2>(2, 0).setZero();

  // Because G can be factored out of B, we can divide by the old ratio and
  // multiply by the new ratio to get a new drivetrain model.
  prosdk::math::Matrixd<4, 4> A;
  A.block<2, 2>(0, 0) = m_plant.A() * m_currentGearing / m_originalGearing;

  A.block<2, 2>(2, 0).setIdentity();
  A.block<4, 2>(0, 2).setZero();

  double v = (x(State::LEFT_VELOCITY) + x(State::RIGHT_VELOCITY)) / 2.0;

  prosdk::math::Vectord<7> xdot;
  xdot(0) = v * std::cos(x(State::HEADING));
  xdot(1) = v * std::sin(x(State::HEADING));
  xdot(2) =
      ((x(State::RIGHT_VELOCITY) - x(State::LEFT_VELOCITY)) / (2.0 * m_rb))
          .value();
  xdot.block<4, 1>(3, 0) = A * x.block<4, 1>(3, 0) + B * u;
  return xdot;
}
