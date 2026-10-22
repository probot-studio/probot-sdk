// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/sysid/SysIdRoutine.hpp"

#include "prosdk/sysid/SysIdRoutineLog.hpp"

using namespace prosdk::cmd::sysid;

prosdk::cmd::CommandPtr SysIdRoutine::Quasistatic(Direction direction) {
  prosdk::sysid::State state;
  if (direction == Direction::FORWARD) {
    state = prosdk::sysid::State::QUASISTATIC_FORWARD;
  } else {  // if (direction == Direction::REVERSE) {
    state = prosdk::sysid::State::QUASISTATIC_REVERSE;
  }

  double outputSign = direction == Direction::FORWARD ? 1.0 : -1.0;

  return m_mechanism.subsystem->RunOnce([this] { timer.Restart(); })
      .AndThen(
          m_mechanism.subsystem
              ->Run([this, state, outputSign] {
                m_outputVolts = outputSign * timer.Get() * m_config.rampRate;
                m_mechanism.drive(m_outputVolts);
                m_mechanism.log(this);
                m_recordState(state);
              })
              .FinallyDo([this] {
                m_mechanism.drive(0_V);
                m_recordState(prosdk::sysid::State::NONE);
                timer.Stop();
              })
              .WithName("sysid-" +
                        prosdk::sysid::SysIdRoutineLog::StateEnumToString(state) +
                        "-" + m_mechanism.name)
              .WithTimeout(m_config.timeout));
}

prosdk::cmd::CommandPtr SysIdRoutine::Dynamic(Direction direction) {
  prosdk::sysid::State state;
  if (direction == Direction::FORWARD) {
    state = prosdk::sysid::State::DYNAMIC_FORWARD;
  } else {  // if (direction == Direction::REVERSE) {
    state = prosdk::sysid::State::DYNAMIC_REVERSE;
  }

  double outputSign = direction == Direction::FORWARD ? 1.0 : -1.0;

  return m_mechanism.subsystem
      ->RunOnce([this] { m_outputVolts = m_config.stepVoltage; })
      .AndThen(m_mechanism.subsystem->Run([this, state, outputSign] {
        m_mechanism.drive(m_outputVolts * outputSign);
        m_mechanism.log(this);
        m_recordState(state);
      }))
      .FinallyDo([this] {
        m_mechanism.drive(0_V);
        m_recordState(prosdk::sysid::State::NONE);
      })
      .WithName("sysid-" +
                prosdk::sysid::SysIdRoutineLog::StateEnumToString(state) + "-" +
                m_mechanism.name)
      .WithTimeout(m_config.timeout);
}
