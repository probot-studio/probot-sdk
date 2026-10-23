// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAccelerometerCollision: collision detection from acceleration
// (upstream wpilibcExamples/src/main/cpp/snippets/AccelerometerCollision), ported to prosdk. X and
// Y jerk go to /Telemetry.
//
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/telemetry/Telemetry.hpp>
#include <prosdk/units/acceleration.hpp>

/**
 * Collision detection snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    // Gets the current accelerations in the X and Y directions
    auto xAccel = accelerometer.GetAccelX();
    auto yAccel = accelerometer.GetAccelY();
    // Calculates the jerk in the X and Y directions
    auto xJerk = (xAccel - prevXAccel) / GetPeriod();
    auto yJerk = (yAccel - prevYAccel) / GetPeriod();
    prevXAccel = xAccel;
    prevYAccel = yAccel;

    prosdk::telemetry::Log("X Jerk", xJerk);
    prosdk::telemetry::Log("Y Jerk", yJerk);
  }

 private:
  prosdk::units::meters_per_second_squared_t prevXAccel = 0.0_mps_sq;
  prosdk::units::meters_per_second_squared_t prevYAccel = 0.0_mps_sq;
  prosdk::MPU6050 accelerometer{prosdk::I2C::Port::PORT_0};
};

PROSDK_ROBOT_MAIN(Robot)
