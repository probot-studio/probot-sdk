// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// prosdk_robot_tests: test cases run in declaration order (the e-stop case latches and is last).

#include <catch2/catch_session.hpp>

int main(int argc, char** argv) {
  Catch::Session session;
  session.configData().runOrder = Catch::TestRunOrder::Declared;
  if (int rc = session.applyCommandLine(argc, argv); rc != 0) {
    return rc;
  }
  return session.run();
}
