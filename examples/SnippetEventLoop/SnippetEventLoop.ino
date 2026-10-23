// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// SnippetEventLoop: a ball shooter managed with EventLoop and BooleanEvent
// (upstream wpilibcExamples/src/main/cpp/snippets/EventLoop), ported to prosdk.
//
// Wiring (GPIO numbers; channel = GPIO):
//   GPIO 4  shooter PWMSparkMax     GPIO 6/7  shooter encoder A/B
//   GPIO 5  kicker PWMSparkMax      GPIO 12   intake PWMSparkMax
// Joystick 0: button 2 runs the intake, the trigger spins up and shoots. On the Probot DS gamepad
// raw button 2 is FACE_LEFT (X / Square) and the trigger (raw button 1) is FACE_RIGHT (B / Circle).

#define PROBOT_WIFI_AP_PASSWORD "probot-robot"
#define PROBOT_WIFI_AP_CHANNEL 6
#include <prosdk.h>

#include <prosdk/drivers/motor/PWMSparkMax.hpp>
#include <prosdk/driverstation/Joystick.hpp>
#include <prosdk/event/BooleanEvent.hpp>
#include <prosdk/event/EventLoop.hpp>
#include <prosdk/framework/TimedRobot.hpp>
#include <prosdk/hardware/rotation/Encoder.hpp>
#include <prosdk/math/controller/PIDController.hpp>
#include <prosdk/math/controller/SimpleMotorFeedforward.hpp>
#include <prosdk/units/angular_velocity.hpp>
#include <prosdk/units/length.hpp>
#include <prosdk/units/time.hpp>
#include <prosdk/units/voltage.hpp>

static const auto SHOT_VELOCITY = 200_rpm;
static const auto TOLERANCE = 8_rpm;

class Robot : public prosdk::TimedRobot {
 public:
  Robot() {
    controller.SetTolerance(TOLERANCE.value());

    prosdk::BooleanEvent isBallAtKicker{&loop, [] {
                                       return false;
                                       //    return kickerSensor.GetRange() <
                                       //           KICKER_THRESHOLD;
                                     }};
    prosdk::BooleanEvent intakeButton{
        &loop, [&joystick = joystick] { return joystick.GetRawButton(2); }};

    // if the thumb button is held
    (intakeButton
     // and there is not a ball at the kicker
     && !isBallAtKicker)
        // activate the intake
        .IfHigh([&intake = intake] { intake.SetThrottle(0.5); });

    // if the thumb button is not held
    (!intakeButton
     // or there is a ball in the kicker
     || isBallAtKicker)
        // stop the intake
        .IfHigh([&intake = intake] { intake.SetThrottle(0.0); });

    prosdk::BooleanEvent shootTrigger{
        &loop, [&joystick = joystick] { return joystick.GetTrigger(); }};

    // if the trigger is held
    shootTrigger
        // accelerate the shooter wheel
        .IfHigh([&shooter = shooter, &controller = controller, &ff = ff,
                 &encoder = shooterEncoder] {
          shooter.SetVoltage(
              prosdk::units::volt_t{controller.Calculate(encoder.GetRate(),
                                                      SHOT_VELOCITY.value())} +
              ff.Calculate(prosdk::units::radians_per_second_t{SHOT_VELOCITY}));
        });
    // if not, stop
    (!shootTrigger).IfHigh([&shooter = shooter] { shooter.SetThrottle(0.0); });

    prosdk::BooleanEvent atTargetVelocity =
        prosdk::BooleanEvent(
            &loop,
            [&controller = controller] { return controller.AtSetpoint(); })
            // debounce for more stability
            .Debounce(0.2_s);

    // if we're at the target velocity, kick the ball into the shooter wheel
    atTargetVelocity.IfHigh([&kicker = kicker] { kicker.SetThrottle(0.7); });

    // when we stop being at the target velocity, it means the ball was shot
    atTargetVelocity
        .Falling()
        // so stop the kicker
        .IfHigh([&kicker = kicker] { kicker.SetThrottle(0.0); });
  }

  void RobotPeriodic() override { loop.Poll(); }

 private:
  prosdk::PWMSparkMax shooter{4};
  prosdk::Encoder shooterEncoder{6, 7};
  prosdk::math::PIDController controller{0.3, 0, 0};
  prosdk::math::SimpleMotorFeedforward<prosdk::units::radians> ff{0.1_V,
                                                            0.065_V / 1_rpm};

  prosdk::PWMSparkMax kicker{5};

  prosdk::PWMSparkMax intake{12};

  prosdk::EventLoop loop{};
  prosdk::Joystick joystick{0};
};

PROSDK_ROBOT_MAIN(Robot)
