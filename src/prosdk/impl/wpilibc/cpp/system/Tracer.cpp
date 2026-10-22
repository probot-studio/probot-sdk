// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "prosdk/system/Tracer.hpp"

#include <format>

#include "prosdk/system/Errors.hpp"
#include "prosdk/util/SmallString.hpp"
#include "prosdk/util/raw_ostream.hpp"

using namespace prosdk;

Tracer::Tracer() {
  ResetTimer();
}

void Tracer::ResetTimer() {
  m_startTime = prosdk::hal::monotonic_clock::now();
}

// prosdk: ClearEpochs() keeps the entries the previous loop added, marked unset, instead of freeing
// them, so a loop that adds the same epochs again (the CommandScheduler adds one per scheduled
// command) does not allocate a map node and a key string per epoch per loop. Entries left unset
// for a whole loop are erased. The first new epoch of a loop erases the entries still unset
// before it allocates, so the map never holds more than upstream's, which frees them all here.
// PrintEpochs() skips unset entries, so it prints what upstream does.
static constexpr std::chrono::nanoseconds kUnsetEpoch = std::chrono::nanoseconds::min();

void Tracer::ClearEpochs() {
  ResetTimer();
  m_unsetEpochs = 0;
  for (auto it = m_epochs.begin(); it != m_epochs.end();) {
    if (it->second == kUnsetEpoch) {
      it = m_epochs.erase(it);
    } else {
      it->second = kUnsetEpoch;
      ++m_unsetEpochs;
      ++it;
    }
  }
}

void Tracer::AddEpoch(std::string_view epochName) {
  auto currentTime = prosdk::hal::monotonic_clock::now();
  auto it = m_epochs.find(epochName);
  if (it != m_epochs.end()) {
    if (it->second == kUnsetEpoch) {
      --m_unsetEpochs;
    }
    it->second = currentTime - m_startTime;
  } else {
    if (m_unsetEpochs != 0) {
      std::erase_if(m_epochs, [](const auto& epoch) { return epoch.second == kUnsetEpoch; });
      m_unsetEpochs = 0;
    }
    m_epochs[epochName] = currentTime - m_startTime;
  }
  m_startTime = currentTime;
}

void Tracer::PrintEpochs() {
  prosdk::util::SmallString<128> buf;
  prosdk::util::raw_svector_ostream os(buf);
  PrintEpochs(os);
  if (!buf.empty()) {
    WPILIB_ReportWarning("{}", buf.c_str());
  }
}

void Tracer::PrintEpochs(prosdk::util::raw_ostream& os) {
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;

  auto now = prosdk::hal::monotonic_clock::now();
  if (now - m_lastEpochsPrintTime > MIN_PRINT_PERIOD) {
    m_lastEpochsPrintTime = now;
    for (const auto& epoch : m_epochs) {
      if (epoch.second == kUnsetEpoch) {
        continue;
      }
      os << std::format(
          "\t{}: {:.6f}s\n", epoch.first,
          duration_cast<nanoseconds>(epoch.second).count() / 1.0e9);
    }
  }
}
