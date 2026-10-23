// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetAccelerometerFilter: moving-average filter of an acceleration
// (upstream wpilibcExamples/src/main/cpp/snippets/AccelerometerFilter), ported to prosdk. Raw and
// filtered values go to /Telemetry.
//
// The Probot board has no onboard IMU: upstream's OnboardIMU is replaced by the kit IMU,
// prosdk::MPU6050 (same getters), on I2C port 0 (GPIO 1 SDA, GPIO 2 SCL: J3, the HAL default).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/math/filter/LinearFilter.hpp>
#include <prosdk/telemetry/Telemetry.hpp>
#include <prosdk/units/acceleration.hpp>

/**
 * Accelerometer filtering snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  void RobotPeriodic() override {
    prosdk::units::meters_per_second_squared_t XAccel = accelerometer.GetAccelX();
    // Get the filtered X acceleration
    prosdk::units::meters_per_second_squared_t filteredXAccel =
        xAccelFilter.Calculate(XAccel);

    prosdk::telemetry::Log("X Acceleration", XAccel);
    prosdk::telemetry::Log("Filtered X Acceleration", filteredXAccel);
  }

 private:
  prosdk::MPU6050 accelerometer{prosdk::I2C::Port::PORT_0};
  prosdk::math::LinearFilter<prosdk::units::meters_per_second_squared_t>
      xAccelFilter = prosdk::math::LinearFilter<
          prosdk::units::meters_per_second_squared_t>::MovingAverage(10);
};

PROSDK_ROBOT_MAIN(Robot)
