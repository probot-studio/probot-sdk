// DriversShowcase: a selection of prosdk's drivers in a TimedRobot: the ProHub board (PCA9685
// motor port and servo, encoder stream, MPU-6050, goBILDA Pinpoint) and the Spark, servo, BTS7960
// and VNH5019 outputs. The library has more (the full list is design/drivers.md; the Boardoza*
// examples show the Boardoza boards). Gamepad 0 drives every motor and servo at once; every
// sensor goes to the Driver Station (/Telemetry).
//
// Wiring (change the pins to match your robot; none may be a probot-core pin):
//   GPIO 4           Spark (or any 1000-2000 us PWM motor controller) signal
//   GPIO 5           hobby servo signal
//   GPIO 6/7/15      BTS7960 RPWM / LPWM / R_EN+L_EN (tied together)
//   GPIO 16/17/18/21 VNH5019 INA / INB / PWM / ENA+ENB (through series resistors, design/drivers.md)
//   ProHub board     I2C on GPIO 1/2 (PCA9685 0x40, MPU-6050 0x68, goBILDA Pinpoint 0x31),
//                    encoder stream on GPIO 10 (RX) / 11 (TX)
//
// The PCA9685 has one frequency for all 16 channels. Here it runs at 50 Hz so that motor port 1
// and the servo on channel 12 share it; a hub that drives only motors uses
// prosdk::prohub::kMotorPwmFrequency (872 Hz) instead.
//
// Teleop: left stick Y drives the motors, right stick X moves both servos. Hold the bottom face
// button (A on Xbox) to zero the yaw. Keep the robot still for one second after power-up (gyro
// calibration).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <numbers>

#include <prosdk/drivers/ProHub.hpp>
#include <prosdk/drivers/encoder/ProHubEncoders.hpp>
#include <prosdk/drivers/imu/MPU6050.hpp>
#include <prosdk/drivers/motor/BTS7960.hpp>
#include <prosdk/drivers/motor/ProHubMotor.hpp>
#include <prosdk/drivers/motor/Spark.hpp>
#include <prosdk/drivers/motor/VNH5019.hpp>
#include <prosdk/drivers/odometry/GoBildaPinpoint.hpp>
#include <prosdk/drivers/pwm/PCA9685.hpp>
#include <prosdk/drivers/servo/Servo.hpp>
#include <prosdk/driverstation/Gamepad.hpp>
#include <prosdk/framework/TimedRobot.hpp>

namespace prohub = prosdk::prohub;

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    m_imu.Calibrate();
    m_vnh.SetBrakeMode(true);
    m_wheel.SetDistancePerPulse(0.065 * std::numbers::pi / 1440.0);  // 65 mm wheel, 1440 ticks/rev
  }

  void RobotPeriodic() override {
    m_pinpoint.Update();
    const auto pose = m_pinpoint.GetPose();
    prosdk::telemetry::Log("Pinpoint/X (m)", pose.X().value());
    prosdk::telemetry::Log("Pinpoint/Y (m)", pose.Y().value());
    prosdk::telemetry::Log("IMU", m_imu);
    prosdk::telemetry::Log("IMU/Connected", m_imu.IsConnected());
    prosdk::telemetry::Log("Encoders/Wheel", m_wheel);
    prosdk::telemetry::Log("Encoders/Connected", m_encoders.IsConnected());
    prosdk::telemetry::Log("Encoders/CRC errors", static_cast<int64_t>(m_encoders.GetCrcErrors()));
    prosdk::telemetry::Log("PCA9685/Connected", m_hub.IsConnected());
    prosdk::telemetry::Log("Servo", m_servo);
  }

  void TeleopPeriodic() override {
    if (m_gamepad.GetFaceDownButton()) {
      m_imu.ResetYaw();
    }
    const double drive = -m_gamepad.GetLeftY();
    m_spark.SetThrottle(drive);
    m_bts.SetThrottle(drive);
    m_vnh.SetThrottle(drive);
    m_hubMotor.SetThrottle(drive);
    const double position = (m_gamepad.GetRightX() + 1.0) / 2.0;
    m_servo.Set(position);
    m_hubServo.Set(position);
  }

  void DisabledInit() override {
    m_servo.SetOffline();
    m_hubServo.SetOffline();
  }

 private:
  // first member: the ProHub I2C and encoder UART pins before any driver opens the buses
  bool m_pinsConfigured = prohub::ConfigurePins();

  prosdk::Spark m_spark{4};
  prosdk::Servo m_servo{5};
  prosdk::BTS7960 m_bts{6, 7, 15};
  prosdk::VNH5019 m_vnh{16, 17, 18, 21, 21};

  prosdk::PCA9685 m_hub{prosdk::I2C::Port::PORT_0, prohub::kPcaAddress0, 50_Hz};
  prosdk::ProHubMotor m_hubMotor{m_hub, 1};
  prosdk::Servo m_hubServo{m_hub, prohub::kFirstServoChannel};

  prosdk::MPU6050 m_imu{prosdk::I2C::Port::PORT_0};
  prosdk::GoBildaPinpoint m_pinpoint{prosdk::I2C::Port::PORT_0};
  prosdk::ProHubEncoders m_encoders;
  prosdk::ProHubEncoder m_wheel{m_encoders, 0};

  prosdk::Gamepad m_gamepad{0};
};

PROSDK_ROBOT_MAIN(Robot)
