// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// MecanumDrive: field-oriented mecanum drive with a gyro (upstream
// wpilibcExamples/src/main/cpp/examples/MecanumDrive), ported to
// prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 38 front left, GPIO 37 rear left, GPIO 36 front right, GPIO 35 rear right PWMSparkMax
// Joystick 0 (X, Y, Z axes).
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drive/MecanumDrive.hpp>
#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>

/**
 * This is a sample program that uses mecanum drive with a gyro sensor to
 * maintain rotation vectors in relation to the starting orientation of the
 * robot (field-oriented controls).
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    // Invert the right side motors. You may need to change or remove this to
    // match your robot.
    frontRight.SetInverted(true);
    rearRight.SetInverted(true);
  }

  /**
   * Mecanum drive is used with the gyro angle as an input.
   */
  void TeleopPeriodic() override {
    /* Use the joystick Y axis for forward movement, X axis for lateral
     * movement, and Z axis for rotation.
     */
    robotDrive.DriveCartesian(-joystick.GetY(), -joystick.GetX(),
                              -joystick.GetZ(), imu.GetRotation2d());
  }

 private:
  static constexpr int FRONT_LEFT_MOTOR_PORT = 38;
  static constexpr int REAR_LEFT_MOTOR_PORT = 37;
  static constexpr int FRONT_RIGHT_MOTOR_PORT = 36;
  static constexpr int REAR_RIGHT_MOTOR_PORT = 35;
  static constexpr prosdk::I2C::Port IMU_PORT = prosdk::I2C::Port::PORT_0;
  static constexpr int JOYSTICK_PORT = 0;

  prosdk::PWMSparkMax frontLeft{FRONT_LEFT_MOTOR_PORT};
  prosdk::PWMSparkMax rearLeft{REAR_LEFT_MOTOR_PORT};
  prosdk::PWMSparkMax frontRight{FRONT_RIGHT_MOTOR_PORT};
  prosdk::PWMSparkMax rearRight{REAR_RIGHT_MOTOR_PORT};
  prosdk::MecanumDrive robotDrive{
      [&](double output) { frontLeft.SetThrottle(output); },
      [&](double output) { rearLeft.SetThrottle(output); },
      [&](double output) { frontRight.SetThrottle(output); },
      [&](double output) { rearRight.SetThrottle(output); }};

  prosdk::MPU6050 imu{IMU_PORT};
  prosdk::Joystick joystick{JOYSTICK_PORT};
};

PROSDK_ROBOT_MAIN(Robot)
