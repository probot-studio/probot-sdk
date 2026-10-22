// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// I2C switch with one control byte and no register address (3PEAK TPT29545 on the Boardoza
// TPT29545 breakout; the TI TCA9545A / NXP PCA9545A use the same protocol) over prosdk::I2C
// (design/boardoza/P5-i2c-misc.md). Lets devices with the same address share one I2C port: each
// sits on its own downstream channel and is reached after its channel is selected.
//
// Drivers of the devices behind the switch use their own prosdk::I2C on the same port and know
// nothing about the switch. The selection is global bus state, so a device on a channel may only
// be accessed while that channel is selected: select, then call the device driver's Update() (or
// read) from the same thread. Drivers that read the bus on their own background thread (MPU6050,
// PCA9685 with an update period > 0) must not sit behind the switch; construct them with an
// update period of 0 and call Update() after Select().

#pragma once

#include <stdint.h>

#include <atomic>
#include <mutex>
#include <optional>
#include <string_view>

#include "prosdk/hardware/bus/I2C.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * I2C bus switch (TPT29545 datasheet CA20241229A1, "Control Register" and table 2).
 *
 * A write is one control byte, bit n enabling channel n; any combination is allowed (the bus
 * capacitance of all enabled segments adds up, at most 400 pF). The new selection takes effect at
 * the STOP of that write. A read returns the control byte. At power-up and after a RESET pulse
 * no channel is selected. Devices with the same address on different channels must never be
 * enabled together.
 */
class I2CSwitch : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default 7-bit address (A1 = A0 = 0; 0x70 to 0x73 with A1/A0, from the board README).
  static constexpr int kDefaultAddress = 0x70;
  /// Channels of the TPT29545 / TCA9545A.
  static constexpr int kDefaultChannels = 4;
  /// Most channels a one-byte control register can address (TCA9548A-style 8-channel switches).
  static constexpr int kMaxChannels = 8;

  /**
   * Constructs the driver and deselects every channel. A missing switch prints one console
   * line; the constructor does not throw for it.
   *
   * @param port     The I2C port (prosdk::platform::SetI2CPins sets its pins).
   * @param address  The switch's 7-bit I2C address.
   * @param channels The number of downstream channels, 1 to 8 (4 for the TPT29545).
   */
  explicit I2CSwitch(prosdk::I2C::Port port, int address = kDefaultAddress,
                     int channels = kDefaultChannels);

  I2CSwitch(const I2CSwitch&) = delete;
  I2CSwitch& operator=(const I2CSwitch&) = delete;

  /**
   * Enables the channels whose bits are set in mask (bit n: channel n; 0 deselects all) and
   * disables the others. Throws when the mask has a bit at or above the channel count. Returns
   * false when the switch did not acknowledge.
   */
  bool Select(uint8_t mask);

  /** Enables only one channel (0 .. channels - 1). Returns false on no acknowledge. */
  bool SelectOnly(int channel);

  /**
   * Reads the control register, masked to the channel bits (the TPT29545 datasheet leaves the
   * upper bits undefined). Empty when the switch did not answer.
   */
  std::optional<uint8_t> GetSelected();

  /** The number of downstream channels. */
  int GetChannels() const;

  /** The switch's 7-bit I2C address. */
  int GetAddress() const;

  /** True when the last I2C transfer to the switch succeeded. */
  bool IsConnected() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  prosdk::I2C m_i2c;
  int m_address;
  int m_channels;
  uint8_t m_channelMask;

  std::mutex m_mutex;  // the switch
  std::atomic<int> m_selected{0};  // last mask written successfully
  std::atomic<bool> m_connected{false};
};

}  // namespace prosdk
