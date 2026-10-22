// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.hpp"
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/ArmFeedforward.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/simulation/BatterySim.hpp>
#include <prosdk/simulation/EncoderSim.hpp>
#include <prosdk/simulation/PWMSim.hpp>
#include <prosdk/simulation/RoboRioSim.hpp>
#include <prosdk/simulation/SingleJointedArmSim.hpp>
#include <prosdk/smartdashboard/Mechanism2d.hpp>
#include <prosdk/smartdashboard/MechanismLigament2d.hpp>
#include <prosdk/smartdashboard/MechanismRoot2d.hpp>
#include <prosdk/units/length.hpp>

class Arm {
 public:
  Arm();
  void SimulationPeriodic();
  void LoadPreferences();
  void ReachSetpoint();
  void Stop();

 private:
  // The P gain for the PID controller that drives this arm.
  double armKp = DEFAULT_ARM_KP;
  prosdk::units::degree_t armSetpoint = DEFAULT_ARM_SETPOINT;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  prosdk::math::DCMotor armGearbox = prosdk::math::DCMotor::Vex775Pro(2);

  // Standard classes for controlling our arm
  prosdk::math::PIDController controller{armKp, 0, 0};
  prosdk::Encoder encoder{ENCODER_A_CHANNEL, ENCODER_B_CHANNEL};
  prosdk::PWMSparkMax motor{MOTOR_PORT};

  // Simulation classes help us simulate what's going on, including gravity.
  // This sim represents an arm with 2 775s, a 600:1 reduction, a mass of 5kg,
  // 30in overall arm length, range of motion in [-75, 255] degrees, and noise
  // with a standard deviation of 1 encoder tick.
  prosdk::sim::SingleJointedArmSim armSim{
      armGearbox,
      ARM_REDUCTION,
      prosdk::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS),
      ARM_LENGTH,
      MIN_ANGLE,
      MAX_ANGLE,
      true,
      0_deg,
      {ARM_ENCODER_DIST_PER_PULSE}};
  prosdk::sim::EncoderSim encoderSim{encoder};

  // Create a Mechanism2d display of an Arm
  prosdk::Mechanism2d mech2d{60, 60};
  prosdk::MechanismRoot2d* armBase = mech2d.GetRoot("ArmBase", 30, 30);
  prosdk::MechanismLigament2d* armTower =
      armBase->Append<prosdk::MechanismLigament2d>(
          "Arm Tower", 30, -90_deg, 6,
          prosdk::util::Color8Bit{prosdk::util::Color::BLUE});
  prosdk::MechanismLigament2d* arm = armBase->Append<prosdk::MechanismLigament2d>(
      "Arm", 30, armSim.GetAngle(), 6,
      prosdk::util::Color8Bit{prosdk::util::Color::YELLOW});
};
