// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetADXLAccelerometers: ADXL345 accelerometer snippets (upstream
// wpilibcExamples/src/main/cpp/snippets/ADXLAccelerometers),
// ported to prosdk.
//
// Wiring: an ADXL345 on I2C port 0 (GPIO 8 SDA, GPIO 9 SCL, the HAL default; address 0x1D).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/accelerometer/ADXL345_I2C.hpp>
#include <prosdk/hardware/bus/I2C.hpp>

/**
 * ADXL346, 362 Accelerometer snippets for wpilib-docs.
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html
 */
class Robot : public prosdk::TimedRobot {
 public:
  Robot() {}

  void TeleopPeriodic() override {
    // Gets the current acceleration in the X axis
    accelerometer.GetX();
    // Gets the current acceleration in the Y axis
    accelerometer.GetY();
    // Gets the current acceleration in the Z axis
    accelerometer.GetZ();
  }

 private:
  // Creates an ADXL345 accelerometer object with a measurement range from -8 to
  // 8 G's
  prosdk::ADXL345_I2C accelerometer{prosdk::I2C::Port::PORT_0, 8};
};

PROSDK_ROBOT_MAIN(Robot)
