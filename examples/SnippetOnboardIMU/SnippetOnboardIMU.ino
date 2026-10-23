// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetOnboardIMU: IMU getter snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/OnboardIMU), ported to prosdk.
//
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/math/geometry/Rotation2d.hpp>
#include <prosdk/units/acceleration.hpp>
#include <prosdk/units/angle.hpp>

/**
 * Onboard IMU snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void TeleopPeriodic() override {
    // Gets the current acceleration in the X axis
    IMU.GetAccelX();
    // Gets the current acceleration in the Y axis
    IMU.GetAccelY();
    // Gets the current acceleration in the Z axis
    IMU.GetAccelZ();

    // Gets the current angle in the X axis
    IMU.GetAngleX();
    // Gets the current angle in the Y axis
    IMU.GetAngleY();
    // Gets the current angle in the Z axis
    IMU.GetAngleZ();

    // Gets the current angle as a Rotation2D
    IMU.GetRotation2d();
  }

 private:
  // Creates an object for the kit IMU (upstream: the Systemcore onboard IMU)
  prosdk::MPU6050 IMU{prosdk::I2C::Port::PORT_0};
};

PROSDK_ROBOT_MAIN(Robot)
