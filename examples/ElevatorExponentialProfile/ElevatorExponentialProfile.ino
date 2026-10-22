// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ElevatorExponentialProfile: elevator setpoints from a ExponentialProfile sent to a "smart" motor
// controller's onboard PID (upstream
// wpilibcExamples/src/main/cpp/examples/ElevatorExponentialProfile), ported to prosdk.
// ExampleSmartMotorController is upstream's placeholder (no hardware behind it, every method
// is empty): replace it with the driver of a motor controller that runs its own position PID.
// Joystick 1: button 2 goes to 5 m, button 3 to 0 m.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include "ExampleSmartMotorController.hpp"
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/math/trajectory/ExponentialProfile.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  static constexpr prosdk::units::second_t DT = 20_ms;

  Robot() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    motor.SetPID(1.3, 0.0, 0.7);
  }

  void TeleopPeriodic() override {
    if (joystick.GetRawButtonPressed(2)) {
      goal = {5_m, 0_mps};
    } else if (joystick.GetRawButtonPressed(3)) {
      goal = {0_m, 0_mps};
    }

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    auto next = profile.Calculate(DT, goal, setpoint);

    // Send setpoint to offboard controller PID
    motor.SetSetpoint(
        ExampleSmartMotorController::PIDMode::POSITION,
        setpoint.position.value(),
        feedforward.Calculate(setpoint.velocity, next.velocity) / 12_V);

    setpoint = next;
  }

 private:
  prosdk::Joystick joystick{1};
  ExampleSmartMotorController motor{1};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::meters> feedforward{
      // Note: These gains are fake, and will have to be tuned for your robot.
      1_V, 1_V / 1_mps, 1_V / 1_mps_sq};

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  prosdk::math::ExponentialProfile<prosdk::units::meters, prosdk::units::volts> profile{
      {10_V, 1_V / 1_mps, 1_V / 1_mps_sq}};
  prosdk::math::ExponentialProfile<prosdk::units::meters, prosdk::units::volts>::State
      goal;
  prosdk::math::ExponentialProfile<prosdk::units::meters, prosdk::units::volts>::State
      setpoint;
};

PROSDK_ROBOT_MAIN(Robot)
