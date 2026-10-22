// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.

#include "prosdk/drivers/bus/PulseCapture.hpp"

#include <cmath>

#include "prosdk/system/Errors.hpp"
#include "prosdk/util/StackTrace.hpp"
#include "prosdk/util/UsageReporting.hpp"

using namespace prosdk;

namespace {
// Rounded to whole microseconds; NaN and out-of-range values map to a value the HAL refuses.
uint32_t ToUs(units::microsecond_t t) {
  double us = std::round(t.value());
  return us >= 0.0 && us <= 4e9 ? static_cast<uint32_t>(us) : UINT32_MAX;
}
}  // namespace

PulseCapture::PulseCapture(int channel, units::microsecond_t idleTime,
                           units::microsecond_t glitchFilter)
    : m_channel{channel} {
  int32_t status = 0;
  auto stack = util::GetStackTrace(1);
  m_handle = platform::hal::OpenPulseRx(channel, ToUs(glitchFilter), ToUs(idleTime),
                                        stack.c_str(), &status);
  WPILIB_CheckErrorStatus(status, "Pulse capture on GPIO {} (idle {:.0f} us, glitch {:.1f} us)",
                          channel, idleTime.value(), glitchFilter.value());
  util::ReportUsage("IO", channel, "PulseCapture");
}

int PulseCapture::Read(std::span<Pulse> out) {
  int32_t status = 0;
  int32_t n = platform::hal::ReadPulses(m_handle, out, &status);
  WPILIB_ReportError(status, "Pulse capture on GPIO {}", m_channel);
  return n;
}

int64_t PulseCapture::GetDropped() const {
  int32_t status = 0;
  int64_t n = platform::hal::GetDroppedPulses(m_handle, &status);
  WPILIB_ReportError(status, "Pulse capture on GPIO {}", m_channel);
  return n;
}

int PulseCapture::GetChannel() const {
  return m_channel;
}
