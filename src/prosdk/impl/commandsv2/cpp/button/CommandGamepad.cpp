// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/CommandGamepad.hpp"

#include <memory>

#include "prosdk/commands2/button/CommandGenericHID.hpp"

using namespace prosdk::cmd;

CommandGamepad::CommandGamepad(int port)
    : m_hid{&CommandGenericHID::GetCommandGenericHID(port)},
      m_gamepad{&prosdk::DriverStation::GetGamepad(port)} {}

CommandGamepad::CommandGamepad(prosdk::Gamepad* gamepad)
    : m_ownedHid{std::make_unique<CommandGenericHID>(gamepad->GetHID())},
      m_hid{m_ownedHid.get()},
      m_gamepad{gamepad} {}

CommandGenericHID& CommandGamepad::GetHID() {
  return *m_hid;
}

prosdk::Gamepad& CommandGamepad::GetGamepad() {
  return *m_gamepad;
}

Trigger CommandGamepad::Button(int button, prosdk::EventLoop* loop) const {
  return m_hid->Button(button, loop);
}

Trigger CommandGamepad::Button(enum prosdk::Gamepad::Button button,
                               prosdk::EventLoop* loop) const {
  return Button(static_cast<int>(button), loop);
}

Trigger CommandGamepad::FaceDown(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::FACE_DOWN, loop);
}

Trigger CommandGamepad::FaceRight(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::FACE_RIGHT, loop);
}

Trigger CommandGamepad::FaceLeft(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::FACE_LEFT, loop);
}

Trigger CommandGamepad::FaceUp(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::FACE_UP, loop);
}

Trigger CommandGamepad::Back(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::BACK, loop);
}

Trigger CommandGamepad::Guide(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::GUIDE, loop);
}

Trigger CommandGamepad::Start(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::START, loop);
}

Trigger CommandGamepad::LeftStick(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::LEFT_STICK, loop);
}

Trigger CommandGamepad::RightStick(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::RIGHT_STICK, loop);
}

Trigger CommandGamepad::LeftBumper(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::LEFT_BUMPER, loop);
}

Trigger CommandGamepad::RightBumper(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::RIGHT_BUMPER, loop);
}

Trigger CommandGamepad::DpadUp(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::DPAD_UP, loop);
}

Trigger CommandGamepad::DpadDown(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::DPAD_DOWN, loop);
}

Trigger CommandGamepad::DpadLeft(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::DPAD_LEFT, loop);
}

Trigger CommandGamepad::DpadRight(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::DPAD_RIGHT, loop);
}

Trigger CommandGamepad::Misc1(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_1, loop);
}

Trigger CommandGamepad::RightPaddle1(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::RIGHT_PADDLE_1, loop);
}

Trigger CommandGamepad::LeftPaddle1(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::LEFT_PADDLE_1, loop);
}

Trigger CommandGamepad::RightPaddle2(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::RIGHT_PADDLE_2, loop);
}

Trigger CommandGamepad::LeftPaddle2(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::LEFT_PADDLE_2, loop);
}

Trigger CommandGamepad::Touchpad(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::TOUCHPAD, loop);
}

Trigger CommandGamepad::Misc2(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_2, loop);
}

Trigger CommandGamepad::Misc3(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_3, loop);
}

Trigger CommandGamepad::Misc4(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_4, loop);
}

Trigger CommandGamepad::Misc5(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_5, loop);
}

Trigger CommandGamepad::Misc6(prosdk::EventLoop* loop) const {
  return Button(prosdk::Gamepad::Button::MISC_6, loop);
}

Trigger CommandGamepad::LeftTrigger(double threshold,
                                    prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, threshold] {
    return m_gamepad->GetAxis(prosdk::Gamepad::Axis::LEFT_TRIGGER) > threshold;
  });
}

Trigger CommandGamepad::RightTrigger(double threshold,
                                     prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, threshold] {
    return m_gamepad->GetAxis(prosdk::Gamepad::Axis::RIGHT_TRIGGER) > threshold;
  });
}

Trigger CommandGamepad::AxisLessThan(prosdk::Gamepad::Axis axis, double threshold,
                                     prosdk::EventLoop* loop) const {
  return m_hid->AxisLessThan(static_cast<int>(axis), threshold, loop);
}

Trigger CommandGamepad::AxisGreaterThan(prosdk::Gamepad::Axis axis,
                                        double threshold,
                                        prosdk::EventLoop* loop) const {
  return m_hid->AxisGreaterThan(static_cast<int>(axis), threshold, loop);
}

Trigger CommandGamepad::AxisMagnitudeGreaterThan(prosdk::Gamepad::Axis axis,
                                                 double threshold,
                                                 prosdk::EventLoop* loop) const {
  return m_hid->AxisMagnitudeGreaterThan(static_cast<int>(axis), threshold,
                                         loop);
}

double CommandGamepad::GetLeftX() const {
  return m_gamepad->GetLeftX();
}

double CommandGamepad::GetLeftY() const {
  return m_gamepad->GetLeftY();
}

double CommandGamepad::GetRightX() const {
  return m_gamepad->GetRightX();
}

double CommandGamepad::GetRightY() const {
  return m_gamepad->GetRightY();
}

double CommandGamepad::GetLeftTrigger() const {
  return m_gamepad->GetLeftTrigger();
}

double CommandGamepad::GetRightTrigger() const {
  return m_gamepad->GetRightTrigger();
}
