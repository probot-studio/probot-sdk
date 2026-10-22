// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Load cell on any bridge ADC (design/boardoza/P6-loadcell.md): tare (zero offset) and scale
// (counts per kilogram) over a source of raw counts, such as prosdk::NAU7802::Update() or
// prosdk::HX711::Update(). A bridge output is ratiometric, so the scale is found with a known
// mass on the cell, not from the ADC's volts.

#pragma once

#include <stdint.h>

#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string_view>
#include <utility>

#include "prosdk/system/Errors.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/telemetry/TelemetryTable.hpp"
#include "prosdk/units/mass.hpp"

namespace prosdk {

/**
 * Tared and scaled load-cell reading.
 *
 * No thread: call Update() periodically; it polls the source once. Mass = (counts - offset) /
 * countsPerKg. Calibrate once: Tare() with no load, put a known mass m on the cell, then
 * SetScale(GetCounts() / m). A negative scale is allowed (a cell mounted the other way round).
 *
 * @code
 * prosdk::HX711 m_hx711{4, 5};
 * prosdk::LoadCell m_cell{[this] { return m_hx711.Update(); }};
 * @endcode
 */
class LoadCell : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Returns a new raw sample, or nothing when there is none (non-blocking).
  using Source = std::function<std::optional<int32_t>()>;

  /// Samples Tare() averages by default (1 s at 10 SPS, 0.125 s at 80 SPS).
  static constexpr int kDefaultTareSamples = 10;

  /**
   * @param source Polled by Update(). Nothing else should consume its samples.
   */
  explicit LoadCell(Source source) : m_source{std::move(source)} {}

  LoadCell(const LoadCell&) = delete;
  LoadCell& operator=(const LoadCell&) = delete;

  /**
   * Polls the source once. A new sample updates the mass, and during a tare also adds to its
   * average.
   *
   * @return True when the source gave a new sample.
   */
  bool Update() {
    std::optional<int32_t> sample = m_source();
    if (!sample) {
      return false;
    }
    std::scoped_lock lock{m_mutex};
    m_counts = *sample;
    m_hasData = true;
    if (m_tareLeft > 0) {
      m_tareSum += *sample;
      if (++m_tareCount == m_tareLeft) {
        m_offset = m_tareSum / m_tareCount;
        m_tareLeft = 0;
      }
    }
    return true;
  }

  /**
   * Starts a tare (non-blocking): the next `samples` new samples from Update() are averaged into
   * the zero offset. Until then the old offset stays in use (0 at first). Keep the cell unloaded
   * and still meanwhile.
   *
   * @throws prosdk::RuntimeError (ParameterOutOfRange) when samples < 1.
   */
  void Tare(int samples = kDefaultTareSamples) {
    if (samples < 1) {
      throw WPILIB_MakeError(err::ParameterOutOfRange, "LoadCell tare of {} samples", samples);
    }
    std::scoped_lock lock{m_mutex};
    m_tareLeft = samples;
    m_tareCount = 0;
    m_tareSum = 0.0;
  }

  /** True while a tare collects samples. */
  bool IsTaring() const {
    std::scoped_lock lock{m_mutex};
    return m_tareLeft > 0;
  }

  /**
   * Sets the scale: counts per kilogram after the tare (1 by default, so GetMass() reads counts
   * until it is set).
   *
   * @throws prosdk::RuntimeError (ParameterOutOfRange) for 0 or a value that is not finite.
   */
  void SetScale(double countsPerKg) {
    if (countsPerKg == 0.0 || !std::isfinite(countsPerKg)) {
      throw WPILIB_MakeError(err::ParameterOutOfRange, "LoadCell scale {} counts/kg",
                             countsPerKg);
    }
    std::scoped_lock lock{m_mutex};
    m_countsPerKg = countsPerKg;
  }

  /** The scale in counts per kilogram. */
  double GetScale() const {
    std::scoped_lock lock{m_mutex};
    return m_countsPerKg;
  }

  /** The zero offset in counts. */
  double GetOffset() const {
    std::scoped_lock lock{m_mutex};
    return m_offset;
  }

  /** The last sample minus the offset, in counts (0 before the first sample). */
  double GetCounts() const {
    std::scoped_lock lock{m_mutex};
    return m_hasData ? m_counts - m_offset : 0.0;
  }

  /** The mass of the last sample (0 before the first sample). */
  prosdk::units::kilogram_t GetMass() const {
    std::scoped_lock lock{m_mutex};
    return prosdk::units::kilogram_t{m_hasData ? (m_counts - m_offset) / m_countsPerKg : 0.0};
  }

  /** True once the source gave a sample. */
  bool HasData() const {
    std::scoped_lock lock{m_mutex};
    return m_hasData;
  }

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override {
    std::scoped_lock lock{m_mutex};
    double counts = m_hasData ? m_counts - m_offset : 0.0;
    table.Log("Mass", counts / m_countsPerKg);  // kg
    table.Log("Counts", counts);
    table.Log("Taring", m_tareLeft > 0);
  }

  std::string_view GetTelemetryType() const override { return "Load Cell"; }

 private:
  Source m_source;
  mutable std::mutex m_mutex;
  double m_counts = 0.0;  // last sample
  bool m_hasData = false;
  double m_offset = 0.0;
  double m_countsPerKg = 1.0;
  int m_tareLeft = 0;  // samples the running tare needs; 0: no tare running
  int m_tareCount = 0;
  double m_tareSum = 0.0;
};

}  // namespace prosdk
