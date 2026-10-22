// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Arm.hpp"

#include <prosdk/preferences/Preferences.hpp>
#include <prosdk/system/RobotController.hpp>
#include <prosdk/telemetry/Telemetry.hpp>

Arm::Arm() {
  encoder.SetDistancePerPulse(ARM_ENCODER_DIST_PER_PULSE);

  // Set the Arm position setpoint and P constant to Preferences if the keys
  // don't already exist
  prosdk::Preferences::InitDouble(ARM_POSITION_KEY, armSetpoint.value());
  prosdk::Preferences::InitDouble(ARM_P_KEY, armKp);
}

void Arm::SimulationPeriodic() {
  // In this method, we update our simulation of what our arm is doing
  // First, we set our "inputs" (voltages)
  armSim.SetInput(prosdk::math::Vectord<1>{
      motor.GetThrottle() * prosdk::RobotController::GetInputVoltage()});

  // Next, we update it. The standard loop time is 20ms.
  armSim.Update(20_ms);

  // Finally, we set our simulated encoder's readings and simulated battery
  // voltage
  encoderSim.SetDistance(armSim.GetAngle().value());
  // SimBattery estimates loaded battery voltages
  prosdk::sim::RoboRioSim::SetVInVoltage(
      prosdk::sim::BatterySim::Calculate({armSim.GetCurrentDraw()}));

  // Update the Mechanism Arm angle based on the simulated arm angle
  arm->SetAngle(armSim.GetAngle());

  // Put Mechanism 2d to telemetry
  prosdk::telemetry::Log("Arm Sim", mech2d);
}

void Arm::LoadPreferences() {
  // Read Preferences for Arm setpoint and kP on entering Teleop
  armSetpoint = prosdk::units::degree_t{
      prosdk::Preferences::GetDouble(ARM_POSITION_KEY, armSetpoint.value())};
  if (armKp != prosdk::Preferences::GetDouble(ARM_P_KEY, armKp)) {
    armKp = prosdk::Preferences::GetDouble(ARM_P_KEY, armKp);
    controller.SetP(armKp);
  }
}

void Arm::ReachSetpoint() {
  // Here, we run PID control like normal, with a setpoint read from
  // preferences in degrees.
  double pidOutput = controller.Calculate(
      encoder.GetDistance(), (prosdk::units::radian_t{armSetpoint}.value()));
  motor.SetVoltage(prosdk::units::volt_t{pidOutput});
}

void Arm::Stop() {
  motor.SetThrottle(0.0);
}
