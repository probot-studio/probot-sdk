// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ElevatorProfiledPID: elevator setpoints with a ProfiledPIDController and an
// ElevatorFeedforward (upstream wpilibcExamples/src/main/cpp/examples/ElevatorProfiledPID), ported
// to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 38 PWMSparkMax (or any PWM ESC), GPIO 39/40
// encoder A/B. Joystick 1: button 2 goes to 5 m, button 3 to 0 m.

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/ElevatorFeedforward.hpp>
#include <prosdk/math/controller/ProfiledPIDController.hpp>
#include <prosdk/math/trajectory/TrapezoidProfile.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/velocity.hpp>
#include <prosdk/units/voltage.hpp>

class Robot : public prosdk::TimedRobot {
 public:
  static constexpr prosdk::units::second_t DT = 20_ms;

  Robot() {
    encoder.SetDistancePerPulse(1.0 / 360.0 * 2.0 * std::numbers::pi * 1.5);
  }

  void TeleopPeriodic() override {
    if (joystick.GetRawButtonPressed(2)) {
      controller.SetGoal(5_m);
    } else if (joystick.GetRawButtonPressed(3)) {
      controller.SetGoal(0_m);
    }

    // Run controller and update motor output
    motor.SetVoltage(prosdk::units::volt_t{controller.Calculate(
                         prosdk::units::meter_t{encoder.GetDistance()})} +
                     feedforward.Calculate(controller.GetSetpoint().velocity));
  }

 private:
  static constexpr prosdk::units::meters_per_second_t MAX_VELOCITY = 1.75_mps;
  static constexpr prosdk::units::meters_per_second_squared_t MAX_ACCELERATION =
      0.75_mps_sq;
  static constexpr double kP = 1.3;
  static constexpr double kI = 0.0;
  static constexpr double kD = 0.7;
  static constexpr prosdk::units::volt_t kS = 1.1_V;
  static constexpr prosdk::units::volt_t kG = 1.2_V;
  static constexpr auto kV = 1.3_V / 1_mps;

  prosdk::Joystick joystick{1};
  prosdk::Encoder encoder{39, 40};
  prosdk::PWMSparkMax motor{38};

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  prosdk::math::TrapezoidProfile<prosdk::units::meters>::Constraints constraints{
      MAX_VELOCITY, MAX_ACCELERATION};
  prosdk::math::ProfiledPIDController<prosdk::units::meters> controller{
      kP, kI, kD, constraints, DT};
  prosdk::math::ElevatorFeedforward feedforward{kS, kG, kV};
};

PROSDK_ROBOT_MAIN(Robot)
