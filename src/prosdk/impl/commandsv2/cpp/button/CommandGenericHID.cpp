// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/CommandGenericHID.hpp"

#include <array>
#include <memory>
#include <mutex>

using namespace prosdk::cmd;

namespace {
std::mutex hidsMutex;
std::array<std::unique_ptr<CommandGenericHID>,
           prosdk::internal::DriverStationBackend::JOYSTICK_PORTS>
    hids;
}  // namespace

CommandGenericHID::CommandGenericHID(int port)
    : CommandGenericHID{prosdk::DriverStation::GetGenericHID(port)} {}

CommandGenericHID::CommandGenericHID(prosdk::GenericHID& hid) : m_hid{&hid} {}

CommandGenericHID& CommandGenericHID::GetCommandGenericHID(int port) {
  auto& hid = prosdk::DriverStation::GetGenericHID(port);
  std::scoped_lock lock{hidsMutex};

  if (!hids[port]) {
    hids[port] = std::make_unique<CommandGenericHID>(hid);
  }

  return *hids[port];
}

prosdk::GenericHID& CommandGenericHID::GetHID() {
  return *m_hid;
}

Trigger CommandGenericHID::Button(int button, prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, button] { return m_hid->GetRawButton(button); });
}

Trigger CommandGenericHID::POV(prosdk::POVDirection angle,
                               prosdk::EventLoop* loop) const {
  return POV(0, angle, loop);
}

Trigger CommandGenericHID::POV(int pov, prosdk::POVDirection angle,
                               prosdk::EventLoop* loop) const {
  return Trigger(loop,
                 [this, pov, angle] { return m_hid->GetPOV(pov) == angle; });
}

Trigger CommandGenericHID::POVUp(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::UP, loop);
}

Trigger CommandGenericHID::POVUpRight(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::UP_RIGHT, loop);
}

Trigger CommandGenericHID::POVRight(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::RIGHT, loop);
}

Trigger CommandGenericHID::POVDownRight(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::DOWN_RIGHT, loop);
}

Trigger CommandGenericHID::POVDown(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::DOWN, loop);
}

Trigger CommandGenericHID::POVDownLeft(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::DOWN_LEFT, loop);
}

Trigger CommandGenericHID::POVLeft(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::LEFT, loop);
}

Trigger CommandGenericHID::POVUpLeft(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::UP_LEFT, loop);
}

Trigger CommandGenericHID::POVCenter(prosdk::EventLoop* loop) const {
  return POV(prosdk::POVDirection::CENTER, loop);
}

Trigger CommandGenericHID::AxisLessThan(int axis, double threshold,
                                        prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, axis, threshold]() {
    return m_hid->GetRawAxis(axis) < threshold;
  });
}

Trigger CommandGenericHID::AxisGreaterThan(int axis, double threshold,
                                           prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, axis, threshold]() {
    return m_hid->GetRawAxis(axis) > threshold;
  });
}

Trigger CommandGenericHID::AxisMagnitudeGreaterThan(
    int axis, double threshold, prosdk::EventLoop* loop) const {
  return Trigger(loop, [this, axis, threshold]() {
    return std::abs(m_hid->GetRawAxis(axis)) > threshold;
  });
}

void CommandGenericHID::SetRumble(prosdk::GenericHID::RumbleType type,
                                  double value) {
  m_hid->SetRumble(type, value);
}

bool CommandGenericHID::IsConnected() const {
  return m_hid->IsConnected();
}
