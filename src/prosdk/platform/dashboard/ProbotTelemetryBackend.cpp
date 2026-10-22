// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Mirrors upstream NetworkTablesTelemetryBackend.cpp (WPILib 2027): one entry per path, the type
// fixed by the first value, "type mismatch" reported for a later value of another type, property
// JSON merged per key, KeepDuplicates() switching the topic to keep equal values.

#include "prosdk/platform/dashboard/ProbotTelemetryBackend.hpp"

#include <cstring>
#include <format>
#include <mutex>
#include <utility>
#include <vector>

#include "prosdk/telemetry/TelemetryEntry.hpp"
#include "prosdk/telemetry/TelemetryRegistry.hpp"
#include "prosdk/util/json.hpp"

using namespace prosdk::backend;
namespace dash = prosdk::platform::dashboard;
using dash::ValueType;

class ProbotTelemetryBackend::Entry final : public prosdk::telemetry::TelemetryEntry {
 public:
  Entry(std::string path, std::shared_ptr<std::atomic<bool>> anyPending)
      : m_path{std::move(path)}, m_anyPending{std::move(anyPending)} {}

  ~Entry() override { dash::UnpublishTopic(m_id); }

  void MarkRemoved() {
    std::scoped_lock lock{m_mutex};
    m_removed.store(true);
    dash::UnpublishTopic(m_id);
    m_id = dash::kInvalidTopic;
    m_pending = {};
    m_hasPending = false;
  }

  bool IsDiscard() const override { return m_removed.load() || m_failed.load(); }

  void KeepDuplicates() override {
    std::scoped_lock lock{m_mutex};
    if (m_removed.load() || m_keepDuplicates) {
      return;
    }
    m_keepDuplicates = true;
    dash::SetTopicFlags(m_id, Flags());
  }

  void SetProperty(std::string_view key, std::string_view value) override {
    if (m_removed.load()) {
      return;
    }
    try {
      auto parsedValue = prosdk::util::json::parse(value);
      if (!parsedValue) {
        prosdk::telemetry::TelemetryRegistry::ReportWarning(m_path, "invalid property JSON");
        return;
      }
      std::scoped_lock lock{m_mutex};
      if (m_removed.load()) {
        return;
      }
      auto& propMap = m_properties.get_object();
      auto it = propMap.find(key);
      if (it == propMap.end()) {
        propMap.emplace(std::string{key}, std::move(*parsedValue));
      } else {
        if (it->second == *parsedValue) {
          return;
        }
        it->second = std::move(*parsedValue);
      }
      if (m_id != dash::kInvalidTopic) {
        dash::SetTopicProperties(m_id, m_properties.to_string(), m_path);
      }
    } catch (...) {
      dash::ReportLimitOnce(dash::Limit::kProperties, m_path);
    }
  }

  void LogBoolean(bool value, int64_t) override {
    Write(ValueType::kBoolean, "boolean",
          [&](std::vector<uint8_t>& out) { return out.push_back(value ? 1 : 0), true; });
  }

  void LogInt64(int64_t value, int64_t) override {
    Write(ValueType::kInt, "int",
          [&](std::vector<uint8_t>& out) { return dash::Append(out, value), true; });
  }

  void LogFloat(float value, int64_t) override {
    Write(ValueType::kFloat, "float",
          [&](std::vector<uint8_t>& out) { return dash::Append(out, value), true; });
  }

  void LogDouble(double value, int64_t) override {
    Write(ValueType::kDouble, "double",
          [&](std::vector<uint8_t>& out) { return dash::Append(out, value), true; });
  }

  void LogString(std::string_view value, std::string_view typeString, int64_t) override {
    Write(ValueType::kString, typeString, [&](std::vector<uint8_t>& out) {
      out.assign(value.begin(), value.end());
      return true;
    });
  }

  void LogBooleanArray(std::span<const bool> value, int64_t) override {
    Write(ValueType::kBooleanArray, "boolean[]", [&](std::vector<uint8_t>& out) {
      for (bool v : value) {
        out.push_back(v ? 1 : 0);
      }
      return true;
    });
  }

  void LogBooleanArray(std::span<const int> value, int64_t) override {
    Write(ValueType::kBooleanArray, "boolean[]", [&](std::vector<uint8_t>& out) {
      for (int v : value) {
        out.push_back(v != 0 ? 1 : 0);
      }
      return true;
    });
  }

  void LogInt16Array(std::span<const int16_t> value, int64_t) override { LogInts(value); }

  void LogInt32Array(std::span<const int32_t> value, int64_t) override { LogInts(value); }

  void LogInt64Array(std::span<const int64_t> value, int64_t) override { LogInts(value); }

  void LogFloatArray(std::span<const float> value, int64_t) override {
    Write(ValueType::kFloatArray, "float[]", [&](std::vector<uint8_t>& out) {
      auto bytes = std::as_bytes(value);
      out.resize(bytes.size());
      std::memcpy(out.data(), bytes.data(), bytes.size());
      return true;
    });
  }

  void LogDoubleArray(std::span<const double> value, int64_t) override {
    Write(ValueType::kDoubleArray, "double[]", [&](std::vector<uint8_t>& out) {
      auto bytes = std::as_bytes(value);
      out.resize(bytes.size());
      std::memcpy(out.data(), bytes.data(), bytes.size());
      return true;
    });
  }

  void LogStringArray(std::span<const std::string> value, int64_t) override {
    LogStrings(value);
  }

  void LogStringArray(std::span<const std::string_view> value, int64_t) override {
    LogStrings(value);
  }

  void LogRaw(std::span<const uint8_t> value, std::string_view typeString, int64_t) override {
    Write(ValueType::kRaw, typeString, [&](std::vector<uint8_t>& out) {
      out.assign(value.begin(), value.end());
      return true;
    });
  }

  /** Publishes the topic and the waiting value once the store is ready (FlushPending). */
  void Flush() {
    std::scoped_lock lock{m_mutex};
    if (m_removed.load() || m_failed.load() || m_vt == 0) {
      return;
    }
    if (m_id == dash::kInvalidTopic && !Publish()) {
      return;
    }
    if (m_hasPending) {
      dash::SetTopicValue(m_id, m_pending, m_path);
      m_pending = {};
      m_hasPending = false;
    }
  }

 private:
  uint8_t Flags() const {
    return dash::kFlagTelemetry | (m_keepDuplicates ? dash::kFlagKeepDuplicates : 0);
  }

  // Caller holds m_mutex. True when the topic is published.
  bool Publish() {
    std::string props;
    if (!m_properties.get_object().empty()) {
      props = m_properties.to_string();
    }
    auto result = dash::PublishTopic(m_path, static_cast<ValueType>(m_vt), m_typeString, props,
                                     Flags());
    if (result.id != dash::kInvalidTopic) {
      m_id = result.id;
      return true;
    }
    if (result.notReady) {
      m_anyPending->store(true, std::memory_order_release);
    } else {
      m_failed.store(true);
      m_pending = {};
      m_hasPending = false;
    }
    return false;
  }

  template <typename Fill>
  void Write(ValueType vt, std::string_view typeString, Fill&& fill) noexcept {
    bool typeMismatch = false;
    try {
      std::scoped_lock lock{m_mutex};
      if (m_removed.load() || m_failed.load()) {
        return;
      }
      if (m_vt == 0) {
        m_vt = static_cast<uint8_t>(vt);
        m_typeString = typeString;
        Publish();
        if (m_failed.load()) {
          return;
        }
      } else if (m_vt != static_cast<uint8_t>(vt) || m_typeString != typeString) {
        typeMismatch = true;
      }
      if (!typeMismatch) {
        m_buf.clear();
        if (!fill(m_buf)) {
          dash::ReportLimitOnce(dash::Limit::kValue, m_path);
          return;
        }
        if (m_id != dash::kInvalidTopic) {
          dash::SetTopicValue(m_id, m_buf, m_path);
        } else {
          // store not ready yet: keep the last value for FlushPending()
          m_pending.swap(m_buf);
          m_hasPending = true;
          m_anyPending->store(true, std::memory_order_release);
        }
      }
    } catch (...) {
      dash::ReportLimitOnce(dash::Limit::kValue, m_path);
      return;
    }
    if (typeMismatch) {
      prosdk::telemetry::TelemetryRegistry::ReportWarning(m_path, "type mismatch");
    }
  }

  template <typename T>
  void LogInts(std::span<const T> value) {
    Write(ValueType::kIntArray, "int[]", [&](std::vector<uint8_t>& out) {
      out.reserve(value.size() * sizeof(int64_t));
      for (T v : value) {
        dash::Append<int64_t>(out, v);
      }
      return true;
    });
  }

  template <typename S>
  void LogStrings(std::span<const S> value) {
    Write(ValueType::kStringArray, "string[]", [&](std::vector<uint8_t>& out) {
      for (const auto& v : value) {
        if (!dash::AppendStringElement(out, v)) {
          return false;
        }
      }
      return true;
    });
  }

  std::string m_path;
  std::shared_ptr<std::atomic<bool>> m_anyPending;
  prosdk::util::mutex m_mutex;
  std::atomic<bool> m_removed{false};
  std::atomic<bool> m_failed{false};  // the store refused the topic (reported once)
  bool m_keepDuplicates = false;
  uint8_t m_vt = 0;  // 0 until the first value
  std::string m_typeString;
  uint16_t m_id = dash::kInvalidTopic;
  prosdk::util::json m_properties = prosdk::util::json::object();
  std::vector<uint8_t> m_buf;
  std::vector<uint8_t> m_pending;
  bool m_hasPending = false;
};

ProbotTelemetryBackend::ProbotTelemetryBackend(std::string_view prefix)
    : m_prefix{prefix}, m_anyPending{std::make_shared<std::atomic<bool>>(false)} {
  dash::AddParticipant(this);
}

ProbotTelemetryBackend::~ProbotTelemetryBackend() {
  dash::RemoveParticipant(this);
}

std::shared_ptr<prosdk::telemetry::TelemetryEntry> ProbotTelemetryBackend::GetEntry(
    std::string_view path) {
  std::scoped_lock lock{m_mutex};
  auto it = m_entries.find(path);
  if (it != m_entries.end()) {
    return it->second;
  }
  auto entry = std::make_shared<Entry>(std::format("{}{}", m_prefix, path), m_anyPending);
  m_entries.try_emplace(path, entry);
  return entry;
}

void ProbotTelemetryBackend::RemoveEntry(std::string_view path) {
  std::shared_ptr<Entry> entry;
  {
    std::scoped_lock lock{m_mutex};
    auto it = m_entries.find(path);
    if (it == m_entries.end()) {
      return;
    }
    entry = std::move(it->second);
    m_entries.erase(it);
  }
  entry->MarkRemoved();
}

bool ProbotTelemetryBackend::HasSchema(std::string_view schemaName) const {
  return dash::HasSchema(schemaName);
}

void ProbotTelemetryBackend::AddSchema(std::string_view schemaName, std::string_view type,
                                       std::span<const uint8_t> schema) {
  dash::AddSchema(schemaName, type, schema);
}

void ProbotTelemetryBackend::AddSchema(std::string_view schemaName, std::string_view type,
                                       std::string_view schema) {
  dash::AddSchema(schemaName, type,
                  {reinterpret_cast<const uint8_t*>(schema.data()), schema.size()});
}

void ProbotTelemetryBackend::FlushPending() {
  if (!m_anyPending->exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  std::vector<std::shared_ptr<Entry>> entries;
  {
    std::scoped_lock lock{m_mutex};
    entries.reserve(m_entries.size());
    for (auto&& [path, entry] : m_entries) {
      entries.push_back(entry);
    }
  }
  for (auto&& entry : entries) {
    entry->Flush();
  }
}
