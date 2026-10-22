// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.hpp"
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/ElevatorFeedforward.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/ProfiledPIDController.hpp>
#include <prosdk/simulation/BatterySim.hpp>
#include <prosdk/simulation/ElevatorSim.hpp>
#include <prosdk/simulation/EncoderSim.hpp>
#include <prosdk/simulation/PWMMotorControllerSim.hpp>
#include <prosdk/simulation/RoboRioSim.hpp>
#include <prosdk/smartdashboard/Mechanism2d.hpp>
#include <prosdk/smartdashboard/MechanismLigament2d.hpp>
#include <prosdk/smartdashboard/MechanismRoot2d.hpp>
#include <prosdk/units/length.hpp>

class Elevator {
 public:
  Elevator();
  void SimulationPeriodic();
  void UpdateTelemetry();
  void ReachGoal(prosdk::units::meter_t goal);
  void Stop();

 private:
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  prosdk::math::DCMotor elevatorGearbox = prosdk::math::DCMotor::Vex775Pro(4);

  // Standard classes for controlling our elevator
  prosdk::math::TrapezoidProfile<prosdk::units::meters>::Constraints constraints{
      2.45_mps, 2.45_mps_sq};
  prosdk::math::ProfiledPIDController<prosdk::units::meters> controller{
      Constants::ELEVATOR_KP, Constants::ELEVATOR_KI, Constants::ELEVATOR_KD,
      constraints};

  prosdk::math::ElevatorFeedforward feedforward{
      Constants::ELEVATOR_KS, Constants::ELEVATOR_KG, Constants::ELEVATOR_KV,
      Constants::ELEVATOR_KA};
  prosdk::Encoder encoder{Constants::ENCODER_A_CHANNEL,
                       Constants::ENCODER_B_CHANNEL};
  prosdk::PWMSparkMax motor{Constants::MOTOR_PORT};
  prosdk::sim::PWMMotorControllerSim motorSim{motor};

  // Simulation classes help us simulate what's going on, including gravity.
  prosdk::sim::ElevatorSim elevatorSim{elevatorGearbox,
                                    Constants::ELEVATOR_GEARING,
                                    Constants::CARRIAGE_MASS,
                                    Constants::ELEVATOR_DRUM_RADIUS,
                                    Constants::MIN_ELEVATOR_HEIGHT,
                                    Constants::MAX_ELEVATOR_HEIGHT,
                                    true,
                                    0_m,
                                    {0.01}};
  prosdk::sim::EncoderSim encoderSim{encoder};

  // Create a Mechanism2d display of an elevator
  prosdk::Mechanism2d mech2d{20, 50};
  prosdk::MechanismRoot2d* elevatorRoot = mech2d.GetRoot("Elevator Root", 10, 0);
  prosdk::MechanismLigament2d* elevatorMech2d =
      elevatorRoot->Append<prosdk::MechanismLigament2d>(
          "Elevator", elevatorSim.GetPosition().value(), 90_deg);
};
