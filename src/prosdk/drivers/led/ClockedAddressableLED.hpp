// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Clocked (two-wire: data + clock) addressable RGB LEDs over prosdk::SPI
// (design/boardoza/P10-tmag-leds.md): the Harvatek T3A33BRG of the Boardoza Phoenix boards
// (Phoenix4/6/12/24, Phoenix12/24-WENC) and the Rotary Encoder ARGB board. The frame is the
// APA102 one (0xE0 | brightness, B, G, R per LED); APA102/SK9822 strips were not tested.
// prosdk::AddressableLED (RMT, one-wire WS2812) cannot drive these LEDs.
//
// The LEDs sit on an SPI device without chip select: MOSI to DIN, SCK to CIN, alone on their bus
// (normally SPI::kPort1; prosdk::platform::SetSPIPins(1, sck, mosi, -1)). They latch every clock
// edge, so no other device may share that bus. SPI is not gated: the LEDs keep their colours
// while the robot is disabled and after an emergency stop.

#pragma once

#include <stdint.h>

#include <initializer_list>
#include <mutex>
#include <span>
#include <string_view>
#include <vector>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/hardware/led/AddressableLED.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"

namespace prosdk {

/**
 * A chain of clocked addressable LEDs (T3A33BRG; APA102-style frame).
 *
 * SetData() takes the same AddressableLED::LEDData buffer as AddressableLED, so
 * LEDPattern::ApplyTo works unchanged; it sends one frame at once (no thread). The 5-bit
 * brightness (the chip's DIMMING current control) applies to every LED on top of the colours.
 */
class ClockedAddressableLED : public prosdk::telemetry::TelemetryLoggable {
 public:
  /// Default clock (CIN) rate: a conservative choice for wires between chained boards.
  static constexpr int kDefaultClockRate = 4'000'000;
  /// Highest clock rate: CIN frequency 15 MHz maximum (T3A33BRG datasheet v2.0, page 7).
  static constexpr int kMaxClockRate = 15'000'000;
  /// Highest brightness (DIMMING 5 bits, 32 levels).
  static constexpr int kMaxBrightness = 31;

  /**
   * Opens the bus device (SPI mode 0, no chip select) and turns every LED off (the datasheet
   * does not state the power-up state).
   *
   * @param bus     The SPI bus, which must have no other device (prosdk::platform::SetSPIPins
   *                sets its SCK and MOSI pins; MISO -1).
   * @param length  Number of LEDs in the chain (4, 6, 12 or 24 per Phoenix board, 14 on the
   *                Rotary Encoder ARGB board; chained boards add up). Use the real count: a LED
   *                past the length probably takes the all-ones end frame as full white.
   * @param clockHz The clock rate, clamped to SPI::kMinClockRate to kMaxClockRate.
   * @throws prosdk::RuntimeError when the SPI device cannot be opened (see prosdk::SPI).
   */
  ClockedAddressableLED(prosdk::SPI::Port bus, int length, int clockHz = kDefaultClockRate);

  ClockedAddressableLED(const ClockedAddressableLED&) = delete;
  ClockedAddressableLED& operator=(const ClockedAddressableLED&) = delete;

  /** Number of LEDs in the chain. */
  int GetLength() const;

  /**
   * Sends the colours: element i goes to LED i (the LED nearest the DIN input is 0). LEDs past
   * the end of ledData are turned off; elements past the length are ignored.
   *
   * @param ledData The colours.
   */
  void SetData(std::span<const prosdk::AddressableLED::LEDData> ledData);

  /**
   * Sends the colours (see the span overload).
   *
   * @param ledData The colours.
   */
  void SetData(std::initializer_list<prosdk::AddressableLED::LEDData> ledData);

  /**
   * Sets the brightness of every LED (the DIMMING field, 32-level current control) and sends the
   * last colours again. Default kMaxBrightness. Whether 0 turns the LEDs off or is the lowest
   * current is not stated in the datasheet (unverified).
   *
   * @param level 0 to kMaxBrightness (clamped).
   */
  void SetBrightness(int level);

  /** The brightness level (0 to kMaxBrightness). */
  int GetBrightness() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  void SendLocked();

  prosdk::SPI m_spi;
  int m_length;

  mutable std::mutex m_mutex;
  int m_brightness = kMaxBrightness;
  std::vector<prosdk::AddressableLED::LEDData> m_pixels;
  std::vector<uint8_t> m_frame;
};

}  // namespace prosdk
