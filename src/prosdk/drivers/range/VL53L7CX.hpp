// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ST VL53L7CX multizone time-of-flight sensor (4x4 or 8x8 zones, 90 degree diagonal field of
// view, 2 to 350 cm per zone; DS13865 rev 8, table 1) over prosdk::I2C: the Boardoza VL53L7CX-1
// to -4 boards (design/boardoza/P4-tof.md). The sensor runs ST firmware (84 KiB) that the host
// uploads after every power-up. A thread does that with ST's Ultra Lite Driver (ULD 2.0.0,
// BSD-3-Clause, vendored in prosdk/drivers/range/st), starts continuous ranging and reads the
// frames.

#pragma once

#include <stdint.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/frequency.hpp"

namespace prosdk {

/**
 * ST VL53L7CX multizone ranging sensor.
 *
 * Construction never blocks and does not use the bus. The thread starts at the first call of
 * IsReady(), GetDistances(), GetTargetStatus() or SetAddress(), so every sensor of a robot is
 * constructed (and its LPn pin driven low) before any of them talks at the default address. The
 * firmware upload then takes about 2 s at 400 kHz; IsReady() turns true with the first frame.
 *
 * Several sensors on one bus: wire each LPn pin to a GPIO, give each sensor its own address other
 * than 0x29, and construct them all before using any. Their threads take turns at the default
 * address: LPn high, firmware upload at 0x29, move to the new address (ST's multi-sensor order).
 * The new address is lost when the sensor loses power.
 *
 * Zones are numbered as in the data sheet (DS13865 section 7.1): row by row, as seen from above
 * the sensor looking into its lens. The lens flips the image horizontally and vertically: zone 0
 * sees a target at the top right-hand side of the scene (section 7.1.3, figure 19).
 *
 * If the sensor stops sending frames (for example after a power loss, which also loses its
 * address), the thread drives LPn low (if wired), waits 1 s and sets the sensor up again.
 */
class VL53L7CX : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// The sensor's 7-bit I2C address after power-up (8-bit 0x52, DS13865 section 4).
  static constexpr int kDefaultAddress = 0x29;
  /// Zones in an 8x8 frame (the most a frame has).
  static constexpr int kMaxZones = 64;

  /// Zone grid. The ULD allows 1-60 Hz with 4x4 and 1-15 Hz with 8x8 (vl53l7cx_api.h).
  enum class Resolution { k4x4 = 16, k8x8 = 64 };

  /**
   * Constructs the driver. Does not use the bus (see the class comment).
   *
   * @param port             The I2C port (prosdk::platform::SetI2CPins sets its pins). It must
   *                         run at 400 kHz or faster (the firmware goes out in 1 KiB transfers).
   * @param lpnChannel       DIO channel (GPIO) wired to the board's LPn pin, or -1 for none. The
   *                         constructor drives it low (I2C off); the thread drives it high
   *                         for its setup and ranging, and low again after a failure.
   * @param address          The 7-bit address the sensor gets (kDefaultAddress keeps it).
   * @param resolution       4x4 or 8x8 zones.
   * @param rangingFrequency Frames per second, rounded to whole hertz and limited to 1-60 Hz
   *                         (4x4) or 1-15 Hz (8x8). The data sheet's ranging figures use 15 Hz.
   */
  explicit VL53L7CX(prosdk::I2C::Port port, int lpnChannel = -1, int address = kDefaultAddress,
                    Resolution resolution = Resolution::k4x4,
                    prosdk::units::hertz_t rangingFrequency = prosdk::units::hertz_t{15});

  /** Stops the thread. The sensor keeps ranging until it loses power or is set up again. */
  ~VL53L7CX() override;

  VL53L7CX(const VL53L7CX&) = delete;
  VL53L7CX& operator=(const VL53L7CX&) = delete;

  /** True while frames arrive (false during the firmware upload and after a timeout). */
  bool IsReady();

  /**
   * Copies the distances of the latest frame (mm, 0 or more) into distances, as many as fit.
   * A distance is valid only when the zone's target status is 5 or 9 (vl53l7cx_api.h). Pass
   * status to get the target status of the same frame (a separate GetTargetStatus() call may
   * already see the next frame).
   *
   * @param distances Receives the distance of each zone (mm).
   * @param status    Optional: receives the target status of each zone (see GetTargetStatus()).
   * @return The zones in the latest frame (16 or 64), or 0 before the first frame.
   */
  int GetDistances(std::span<int16_t> distances, std::span<uint8_t> status = {});

  /**
   * Copies the target status of each zone of the latest frame, as many as fit: 5 or 9 for a
   * valid distance, 255 when the zone saw no target, other values for a measurement the sensor
   * flags as unreliable.
   *
   * @return The zones in the latest frame (16 or 64), or 0 before the first frame.
   */
  int GetTargetStatus(std::span<uint8_t> status);

  /**
   * Moves the sensor to another 7-bit address (0x08 to 0x77). The thread applies it between two
   * frames (ranging stops for a moment), or at the next setup. Not kept over a power cycle.
   */
  void SetAddress(int newAddress);

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  struct Uld;  // ULD state (vl53l7cx_api.h), kept out of this header

  void Start();
  void Run();
  bool BringUp();
  void Range();
  bool IsAlive();
  bool Fail(const std::string& problem);
  bool Wait(std::chrono::milliseconds duration);
  void Publish();

  prosdk::I2C::Port m_port;
  int m_resolution;        // zones per frame (16 or 64)
  int m_frequencyHz = 15;  // frames per second
  std::optional<prosdk::DigitalOutput> m_lpn;
  std::atomic<int> m_targetAddress;
  std::unique_ptr<Uld> m_uld;  // used by the thread only
  std::string m_problem;       // the last problem printed (thread only)

  mutable std::mutex m_mutex;  // the fields below
  bool m_ready = false;
  int m_zones = 0;
  std::array<int16_t, kMaxZones> m_distances{};
  std::array<uint8_t, kMaxZones> m_status{};

  std::once_flag m_started;
  std::mutex m_waitMutex;
  std::condition_variable m_wake;
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
