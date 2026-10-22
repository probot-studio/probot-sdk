// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/commands2/PrintCommand.hpp"

#include <regex>
#include <string>
#include <string_view>

#include "CommandTestBase.hpp"
#include "prosdk/commands2/Commands.hpp"
#include "prosdk/platform/Console.hpp"

using namespace prosdk::cmd;
class PrintCommandTest : public CommandTestBase {};

namespace {

// prosdk: PrintCommand writes to the platform console instead of stdout, so the
// test captures the console sink.
class StdoutCapture {
 public:
  StdoutCapture() {
    Captured().clear();
    prosdk::platform::SetConsoleSink(
        [](std::string_view text) { Captured().append(text); });
  }

  StdoutCapture(const StdoutCapture&) = delete;
  StdoutCapture& operator=(const StdoutCapture&) = delete;

  ~StdoutCapture() { prosdk::platform::SetConsoleSink(nullptr); }

  std::string Stop() {
    prosdk::platform::SetConsoleSink(nullptr);
    return Captured();
  }

 private:
  static std::string& Captured() {
    static std::string captured;
    return captured;
  }
};

}  // namespace

TEST_CASE_METHOD(PrintCommandTest, "PrintCommandTest PrintCommandSchedule",
                 "[commandsv2][command]") {
  CommandScheduler scheduler = GetScheduler();

  auto command = Print("Test!");

  StdoutCapture capture;

  scheduler.Schedule(command);
  scheduler.Run();

  CHECK(std::regex_search(capture.Stop(), std::regex("Test!")));

  CHECK_FALSE(scheduler.IsScheduled(command));
}
