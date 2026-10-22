// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/button/Trigger.hpp"

#include <utility>

#include "prosdk/commands2/CommandPtr.hpp"
#include "prosdk/math/filter/Debouncer.hpp"
#include "prosdk/math/filter/EdgeCounterFilter.hpp"

using namespace prosdk;
using namespace prosdk::cmd;

Trigger::Trigger(const Trigger& other) = default;

void Trigger::AddBinding(prosdk::util::unique_function<void(bool, bool)>&& body) {
  m_loop->Bind([condition = m_condition, previous = m_condition(),
                body = std::move(body)]() mutable {
    bool current = condition();

    body(previous, current);

    previous = current;
  });
}

Trigger Trigger::OnChange(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (previous != current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::OnChange(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (previous != current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::OnTrue(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (!previous && current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::OnTrue(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (!previous && current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::IfTrue(Command* command) {
  AddBinding([command](bool, bool current) {
    if (current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::IfTrue(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool, bool current) {
    if (current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::OnFalse(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (previous && !current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::OnFalse(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (previous && !current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    }
  });
  return *this;
}

Trigger Trigger::WhileTrue(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (!previous && current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    } else if (previous && !current) {
      command->Cancel();
    }
  });
  return *this;
}

Trigger Trigger::WhileTrue(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (!previous && current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    } else if (previous && !current) {
      command.Cancel();
    }
  });
  return *this;
}

Trigger Trigger::WhileFalse(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (previous && !current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    } else if (!previous && current) {
      command->Cancel();
    }
  });
  return *this;
}

Trigger Trigger::WhileFalse(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (!previous && current) {
      prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
    } else if (previous && !current) {
      command.Cancel();
    }
  });
  return *this;
}

Trigger Trigger::ToggleOnTrue(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (!previous && current) {
      if (command->IsScheduled()) {
        command->Cancel();
      } else {
        prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
      }
    }
  });
  return *this;
}

Trigger Trigger::ToggleOnTrue(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (!previous && current) {
      if (command.IsScheduled()) {
        command.Cancel();
      } else {
        prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
      }
    }
  });
  return *this;
}

Trigger Trigger::ToggleOnFalse(Command* command) {
  AddBinding([command](bool previous, bool current) {
    if (previous && !current) {
      if (command->IsScheduled()) {
        command->Cancel();
      } else {
        prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
      }
    }
  });
  return *this;
}

Trigger Trigger::ToggleOnFalse(CommandPtr&& command) {
  AddBinding([command = std::move(command)](bool previous, bool current) {
    if (previous && !current) {
      if (command.IsScheduled()) {
        command.Cancel();
      } else {
        prosdk::cmd::CommandScheduler::GetInstance().Schedule(command);
      }
    }
  });
  return *this;
}

Trigger Trigger::Debounce(prosdk::units::second_t debounceTime,
                          prosdk::math::Debouncer::DebounceType type) {
  return Trigger(m_loop, [debouncer = prosdk::math::Debouncer(debounceTime, type),
                          condition = m_condition]() mutable {
    return debouncer.Calculate(condition());
  });
}

Trigger Trigger::MultiPress(int requiredPresses, units::second_t windowTime) {
  return Trigger(m_loop, [filter = prosdk::math::EdgeCounterFilter(requiredPresses,
                                                                windowTime),
                          condition = m_condition]() mutable {
    return filter.Calculate(condition());
  });
}

bool Trigger::Get() const {
  return m_condition();
}
