// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Gyro: drive straight with a gyro, joystick forward/backward (upstream
// wpilibcExamples/src/main/cpp/examples/Gyro), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO): GPIO 4 left, GPIO 5 right PWMSparkMax (or any PWM
// ESC). Joystick 0, Y axis.
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 8 SDA, GPIO 9 SCL, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <cmath>

#include <prosdk/drive/DifferentialDrive.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right.SetInverted(true);
  }

  /**
   * The motor velocity is set from the joystick while the DifferentialDrive
   * turning value is assigned from the error between the setpoint and the gyro
   * angle.
   */
  void TeleopPeriodic() override {
    double turningValue =
        (ANGLE_SETPOINT - imu.GetRotation2d().Degrees().value()) * kP;
    drive.ArcadeDrive(-joystick.GetY(), -turningValue);
  }

 private:
  static constexpr double ANGLE_SETPOINT = 0.0;
  static constexpr double kP = 0.005;  // Proportional turning constant

  static constexpr int LEFT_MOTOR_PORT = 4;
  static constexpr int RIGHT_MOTOR_PORT = 5;
  static constexpr prosdk::I2C::Port IMU_PORT = prosdk::I2C::Port::PORT_0;
  static constexpr int JOYSTICK_PORT = 0;

  prosdk::PWMSparkMax left{LEFT_MOTOR_PORT};
  prosdk::PWMSparkMax right{RIGHT_MOTOR_PORT};
  prosdk::DifferentialDrive drive{
      [&](double output) { left.SetThrottle(output); },
      [&](double output) { right.SetThrottle(output); }};

  prosdk::MPU6050 imu{IMU_PORT};
  prosdk::Joystick joystick{JOYSTICK_PORT};
};

PROSDK_ROBOT_MAIN(Robot)
