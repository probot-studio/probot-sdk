// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Shared plumbing of the probot-core dashboard backends (ProbotTelemetryBackend,
// ProbotTunableBackend, the alert publisher): the value encoding probot-core's dashboard store
// uses, the calls into the store through CoreOps, the schema registry shared by all backends,
// the one-time limit reports and the routing of Driver Station writes to the backend that owns
// the topic.
//
// Archive side only: everything reaches probot-core through prosdk::platform::GetCoreOps(); no
// core header is included. Store semantics (probot-core R5, design/core-requests.md):
//   - publish() fails (kInvalid) before core's runtime_setup() ran ("not ready"), on a full topic
//     table (PROBOT_DASH_MAX_TOPICS, 128), a name/type over 255 bytes, properties over
//     PROBOT_DASH_MAX_VALUE or an exhausted pool (PROBOT_DASH_POOL_BYTES);
//   - set() copies at most PROBOT_DASH_MAX_VALUE (1024) bytes and drops values equal to the last
//     one unless the topic has KEEP_DUPLICATES; with R16 (core draft PR #42) a refused value makes
//     the Driver Station mark the topic's previous value not current until a later set() is stored;
//   - addSchema() keeps at most PROBOT_DASH_MAX_SCHEMAS (32) schemas of at most
//     PROBOT_DASH_MAX_SCHEMA_BYTES (4096) bytes each (R13; kinematics.proto's descriptor, 4332
//     bytes, needs more; an older core: PROBOT_DASH_MAX_VALUE, which refuses protobuf
//     descriptors such as geometry2d.proto's, about 1.3 KB);
//   - drainWrites() delivers each topic's last Driver Station write; a delivered topic's current
//     value is re-sent afterwards. Core allows one draining task at a time; prosdk serializes its
//     drains with the routing lock, so DrainWrites() may run on any task (core's user task every
//     cycle, a tunable's publishing task for a persistent tunable);
//   - persistence (probot-core R12, CoreOps ABI 6): a Driver Station write to a PERSISTENT topic
//     that the robot did not refuse (rejectWrite() before the next write for the topic is
//     drained, within about 2 s) is kept across reboots; values the robot sets are not. publish()
//     of a new topic queues the stored value as a write for the next drainWrites(), and
//     rejectWrite() on it erases the record (no frame to the Driver Station).
// Before the store is ready, topics, values and schemas are kept here and flushed by
// FlushPending() (run every cycle by RegisterDashboardBackends()). On a limit the store reports
// once through the console (prosdk::platform::ConsoleWrite) and the robot keeps running.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <bit>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace prosdk::platform::dashboard {

static_assert(std::endian::native == std::endian::little,
              "the dashboard value encoding is little-endian");

/** probot::dashboard::ValueType (wire value type of a topic). */
enum class ValueType : uint8_t {
  kBoolean = 1,       // u8 0/1
  kInt = 2,           // i64
  kFloat = 3,         // f32
  kDouble = 4,        // f64
  kString = 5,        // UTF-8 bytes (type string "string", "json", ...)
  kRaw = 6,           // bytes (type string "raw", "struct:X", "struct:X[]", "proto:X", ...)
  kBooleanArray = 7,  // u8 x n
  kIntArray = 8,      // i64 x n
  kFloatArray = 9,    // f32 x n
  kDoubleArray = 10,  // f64 x n
  kStringArray = 11,  // repeated {u16 len, bytes}
};

/** probot::dashboard::Flags. */
inline constexpr uint8_t kFlagTelemetry = 1;
inline constexpr uint8_t kFlagTunable = 2;
inline constexpr uint8_t kFlagMutable = 4;
inline constexpr uint8_t kFlagRobust = 8;
inline constexpr uint8_t kFlagKeepDuplicates = 16;
inline constexpr uint8_t kFlagPersistent = 32;

/** probot::dashboard::WriteStatus (reason of a refused Driver Station write). */
enum class WriteStatus : uint8_t {
  kOk = 0,
  kUnknownId = 1,
  kImmutable = 2,
  kTypeMismatch = 3,
  kDecodeError = 4,
  kQueueFull = 5,
};

/** probot::dashboard::kInvalid. */
inline constexpr uint16_t kInvalidTopic = 0xFFFF;

/** Store limit kinds; each one is reported at most once per boot. */
enum class Limit : uint8_t {
  kTopics,      // topic table full
  kName,        // path or type string over 255 bytes
  kStore,       // other publish refusal (pool, properties size, type conflict)
  kValue,       // value not stored (over PROBOT_DASH_MAX_VALUE or pool exhausted)
  kSchemas,     // schema not stored (schema table full, schema size limit, pool)
  kProperties,  // properties not updated
  kCount
};

/** Result of PublishTopic(). */
struct PublishResult {
  uint16_t id = kInvalidTopic;
  bool notReady = false;  // the store is not configured yet (retry later)
};

// ---- store access (thread-safe, never throw) ----

/** True once probot-core's dashboard store is configured (runtime_setup() ran). */
bool StoreReady() noexcept;

/**
 * Publishes a topic. On failure the cause is reported once (Limit) unless the store is just not
 * ready yet (notReady: retry later, nothing reported).
 */
PublishResult PublishTopic(const std::string& path, ValueType valueType,
                           const std::string& typeString, const std::string& properties,
                           uint8_t flags) noexcept;

/** Removes a topic (no-op for kInvalidTopic). */
void UnpublishTopic(uint16_t id) noexcept;

/** Stores a value; reports Limit::kValue once on failure. */
bool SetTopicValue(uint16_t id, std::span<const uint8_t> data, std::string_view path) noexcept;

/** Replaces a topic's properties JSON; reports Limit::kProperties once on failure. */
bool SetTopicProperties(uint16_t id, const std::string& properties,
                        std::string_view path) noexcept;

/** Replaces a topic's flags. */
bool SetTopicFlags(uint16_t id, uint8_t flags) noexcept;

/**
 * Tells the Driver Station a drained write was not applied ('K' frame); core does not store a
 * refused write to a PERSISTENT topic. For a value core restored from its persistent storage no
 * frame is sent; core erases the stored value instead.
 */
void RejectWrite(uint16_t id, WriteStatus status) noexcept;

/**
 * True when probot-core keeps PERSISTENT topic values across reboots (probot-core R12, CoreOps
 * ABI 6). Without it the PERSISTENT flag has no effect and prosdk does not set it.
 */
bool PersistenceSupported() noexcept;

/**
 * Erases every stored PERSISTENT value and the accepted writes not stored yet (core's
 * clearPersisted(); topics keep their current values, the next Driver Station write is stored
 * again). Writes flash: call it while the robot is stopped. False without
 * persistence support or when the erase failed.
 */
bool ClearPersisted() noexcept;

/**
 * Number of ClearPersisted() calls so far. The tunable backend compares it to forget the values
 * it remembered for republishing persistent tunables, so a clear is not undone by a republish.
 */
uint32_t PersistClearCount() noexcept;

/** Reports a store limit once per boot through the console. */
void ReportLimitOnce(Limit limit, std::string_view path) noexcept;

// ---- schemas (one registry for all backends; the store keeps them by name) ----

/** True if the schema was added (stored, or waiting for the store to become ready). */
bool HasSchema(std::string_view name) noexcept;

/** Adds a schema (idempotent per name). Reports Limit::kSchemas once if the store refuses it. */
void AddSchema(std::string_view name, std::string_view type,
               std::span<const uint8_t> data) noexcept;

// ---- participants: backends that own topics ----

/** A backend that receives Driver Station writes and/or has work pending until the store is ready. */
class Participant {
 public:
  /**
   * Offers a drained write. Returns std::nullopt when the topic is not this participant's;
   * otherwise kOk (taken) or the reason to refuse it. Called on the draining task with the
   * routing lock held: must not call AddParticipant/RemoveParticipant/DrainWrites/FlushPending.
   */
  virtual std::optional<WriteStatus> OfferWrite(uint16_t id, ValueType valueType,
                                                std::span<const uint8_t> data) {
    (void)id;
    (void)valueType;
    (void)data;
    return std::nullopt;
  }

  /**
   * Publishes what waited for the store (store is ready). Called with the routing lock held: the
   * same rules as OfferWrite, except that it may call DrainWrites() (the lock is recursive; a
   * persistent tunable takes its restored value that way).
   */
  virtual void FlushPending() {}

 protected:
  Participant() = default;
  ~Participant() = default;
  Participant(const Participant&) = delete;
  Participant& operator=(const Participant&) = delete;
};

/** Adds a participant (thread-safe). */
void AddParticipant(Participant* participant) noexcept;

/** Removes a participant; returns after any drain or flush that uses it has finished. */
void RemoveParticipant(Participant* participant) noexcept;

/**
 * Drains probot-core's write queue and offers every write to the participants; writes nobody
 * owns are refused with kUnknownId. Any task (not from OfferWrite); drains are serialized by the
 * routing lock. Returns the number of writes drained.
 */
size_t DrainWrites() noexcept;

/** Flushes pending schemas and participants once the store is ready (core's user task). */
void FlushPending() noexcept;

/** Host tests only: forgets cached schemas and the limit-report latches. */
void ResetForTest() noexcept;

// ---- value encoding (little-endian, the layout of ValueType) ----

/** Appends the raw bytes of a trivially copyable value. */
template <typename T>
inline void Append(std::vector<uint8_t>& out, const T& value) {
  size_t n = out.size();
  out.resize(n + sizeof(T));
  std::memcpy(out.data() + n, &value, sizeof(T));
}

/** Reads a trivially copyable value at byte offset (caller checked the size). */
template <typename T>
inline T Read(std::span<const uint8_t> data, size_t offset) {
  T value;
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

/** Appends one string[] element {u16 len, bytes}; false if longer than 65535 bytes. */
inline bool AppendStringElement(std::vector<uint8_t>& out, std::string_view s) {
  if (s.size() > 0xFFFF) {
    return false;
  }
  Append<uint16_t>(out, static_cast<uint16_t>(s.size()));
  out.insert(out.end(), s.begin(), s.end());
  return true;
}

/** Decodes a string[] payload; false if malformed. */
inline bool DecodeStringArray(std::span<const uint8_t> data, std::vector<std::string>& out) {
  out.clear();
  size_t p = 0;
  while (p < data.size()) {
    if (data.size() - p < 2) {
      return false;
    }
    size_t n = Read<uint16_t>(data, p);
    p += 2;
    if (data.size() - p < n) {
      return false;
    }
    out.emplace_back(reinterpret_cast<const char*>(data.data() + p), n);
    p += n;
  }
  return true;
}

}  // namespace prosdk::platform::dashboard
