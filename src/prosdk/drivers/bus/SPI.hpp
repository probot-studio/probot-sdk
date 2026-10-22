// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
// API of the WPILib 2026 frc::SPI (WPILib BSD license), re-ported on the prosdk SPI backend.
//
// SPI device on an ESP32-S3 SPI bus (design/boardoza/F1-spi.md). WPILib 2027 removed SPI; this
// keeps the 2026 API shape, with the bus and the chip-select GPIO as constructor arguments (the
// GPIO matrix gives any pin a chip select) and multi-segment transactions added. kPort0 is SPI2,
// kPort1 is SPI3; prosdk::platform::SetSPIPins sets a bus's SCK/MOSI/MISO pins before the first
// device on it opens. Drivers own an SPI member and take (SPI::Port, csGpio), as I2C drivers take
// (I2C::Port, address).

#pragma once

#include <stdint.h>

#include <span>

#include "prosdk/platform/hal/SPI.hpp"

namespace prosdk {

/**
 * SPI bus interface class.
 *
 * This class is intended to be used by sensor (and other SPI device) drivers. It is one chip
 * select on one bus; several SPI objects with different chip selects share a bus.
 */
class SPI {
 public:
  /**
   * SPI bus.
   */
  enum Port {
    /// ESP32-S3 SPI2 (FSPI), up to 6 hardware chip selects.
    kPort0 = 0,
    /// ESP32-S3 SPI3, up to 3 hardware chip selects.
    kPort1 = 1
  };

  /**
   * SPI mode.
   */
  enum Mode {
    /// Clock idle low, data sampled on rising edge.
    kMode0 = 0,
    /// Clock idle low, data sampled on falling edge.
    kMode1 = 1,
    /// Clock idle high, data sampled on falling edge.
    kMode2 = 2,
    /// Clock idle high, data sampled on rising edge.
    kMode3 = 3
  };

  /// Clock rate of a new device.
  static constexpr int kDefaultClockRate = 1'000'000;
  /// Lowest clock rate: 80 MHz APB / (prescaler 16 x divider 64) (ESP-IDF spi_ll.h).
  static constexpr int kMinClockRate = 78'125;
  /// Highest clock rate accepted (what a device and its wiring manage is the driver's concern).
  static constexpr int kMaxClockRate = 40'000'000;

  /**
   * Opens a device: mode 0, 1 MHz, chip select active low.
   *
   * @param bus    The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio The chip-select GPIO, driven by the SPI peripheral. -1 means no chip select:
   *               only for a device that is alone on its bus (clocked LEDs).
   * @throws prosdk::RuntimeError when the bus has no pins, a pin is in use, the chip select is
   *         already open, the bus has no free chip select, or a device without chip select
   *         would share the bus.
   */
  SPI(Port bus, int csGpio);

  SPI(SPI&& other) noexcept;
  SPI& operator=(SPI&& other) noexcept;
  SPI(const SPI&) = delete;
  SPI& operator=(const SPI&) = delete;

  /** Closes the device (the last device on a bus stops the bus and frees its pins). */
  ~SPI();

  /**
   * Returns the SPI bus.
   *
   * @return The SPI bus.
   */
  Port GetPort() const;

  /**
   * Returns the chip-select GPIO.
   *
   * @return The chip-select GPIO, or -1 for none.
   */
  int GetChipSelect() const;

  /**
   * Configure the rate of the generated clock signal. The default value is 1 MHz (WPILib 2026:
   * 500 kHz). The ESP32-S3 divides 80 MHz, so the actual rate is the nearest divider result.
   *
   * @param hz The clock rate in Hertz, kMinClockRate to kMaxClockRate.
   * @throws prosdk::RuntimeError (ParameterOutOfRange) outside that range.
   */
  void SetClockRate(int hz);

  /**
   * Sets the mode for the SPI device.
   *
   * @param mode The mode to set.
   */
  void SetMode(Mode mode);

  /**
   * Configure the chip select line to be active high.
   */
  void SetChipSelectActiveHigh();

  /**
   * Configure the chip select line to be active low (the default).
   */
  void SetChipSelectActiveLow();

  /**
   * Write data to the peripheral device (one transaction; received bytes are discarded).
   *
   * @param data Bytes to send.
   * @param size Number of bytes.
   * @return size, or -1 when the transfer failed.
   */
  int Write(const uint8_t* data, int size);

  /**
   * Read data from the peripheral device: clocks size zero bytes and stores what the device
   * sends. There is no receive FIFO of a previous write, so initiate is ignored (WPILib 2026
   * read the FIFO when it was false).
   *
   * @param initiate     Ignored.
   * @param dataReceived Buffer to receive data from the device.
   * @param size         The length of the transaction, in bytes.
   * @return size, or -1 when the transfer failed.
   */
  int Read(bool initiate, uint8_t* dataReceived, int size);

  /**
   * Perform a simultaneous read/write transaction with the device.
   *
   * @param dataToSend   The data to be written out to the device.
   * @param dataReceived Buffer to receive data from the device.
   * @param size         The length of the transaction, in bytes.
   * @return size, or -1 when the transfer failed.
   */
  int Transaction(const uint8_t* dataToSend, uint8_t* dataReceived, int size);

  /**
   * One transaction made of several segments (for example command, address, data) with the
   * chip select held from the first to the last byte. A segment's tx nullptr sends zeros, rx
   * nullptr discards the received bytes.
   *
   * @param segments The segments, clocked in order.
   * @return The total number of bytes, or -1 when the transfer failed or a size is negative.
   */
  int TransactionSegments(std::span<const SpiSegment> segments);

 private:
  void Configure(const platform::hal::SpiDeviceConfig& config);

  Port m_port;
  int m_cs;
  platform::hal::SpiDeviceConfig m_config;
  int32_t m_handle = platform::hal::kInvalidSpiHandle;
};

}  // namespace prosdk
