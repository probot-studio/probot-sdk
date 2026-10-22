// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Board wiring for the HAL buses that have no fixed pins on the ESP32-S3 (design/hal-esp32.md).
// WPILib names buses by port (I2C kPort0/kPort1, SerialPort kOnboard/kMXP, CAN bus 0); the GPIO
// matrix lets any pin carry them, so the sketch says which pins before it opens the bus:
//
//   prosdk::platform::SetI2CPins(1, /*sda=*/17, /*scl=*/18);   // before I2C(I2C::kPort1, ...)
//   prosdk::platform::SetCanConfig(/*tx=*/4, /*rx=*/5, 1'000'000);
//   prosdk::platform::SetSPIPins(0, /*sck=*/12, /*mosi=*/11, /*miso=*/13);  // before SPI(...)
//
// A change applies the next time the bus is opened. Defaults: I2C port 0 on GPIO 8/9 (the board's
// Wire pins, shared with Wire when Wire already runs) at 400 kHz; serial onboard (UART1) on
// RX 15 / TX 16; I2C port 1, serial MXP (UART2), CAN and both SPI buses unset (opening them fails
// with a message).

#pragma once

#include <stdint.h>

namespace prosdk::platform {

/** Pins and clock of an I2C port (-1: not configured). */
struct I2CConfig {
  int sda;
  int scl;
  uint32_t frequencyHz;
};

/** Pins of a serial port (-1: not configured). */
struct SerialConfig {
  int rx;
  int tx;
};

/** Pins of an SPI bus (-1: not configured; miso -1 on a transmit-only bus). */
struct SPIConfig {
  int sck;
  int mosi;
  int miso;
};

/** TWAI (CAN) pins and bit rate (-1: not configured). Needs an external 3.3 V transceiver. */
struct CanConfig {
  int tx;
  int rx;
  uint32_t bitrate;  // 125000, 250000, 500000 or 1000000
};

/** Number of I2C ports (WPILib I2C::Port kPort0, kPort1). */
inline constexpr int kNumI2CPorts = 2;
/** Number of serial ports (WPILib SerialPort::Port kOnboard = UART1, kMXP = UART2). */
inline constexpr int kNumSerialPorts = 2;
/** Number of SPI buses (prosdk::SPI kPort0 = SPI2, kPort1 = SPI3). */
inline constexpr int kNumSPIBuses = 2;

/** Sets the pins of I2C port 0 or 1. Returns false for an invalid port or pin. */
bool SetI2CPins(int port, int sda, int scl, uint32_t frequencyHz = 400000);
/** The configuration of an I2C port (all -1 for an invalid port). */
I2CConfig GetI2CConfig(int port);

/** Sets the pins of serial port 0 (onboard, UART1) or 1 (MXP, UART2). */
bool SetSerialPins(int port, int rx, int tx);
/** The configuration of a serial port (all -1 for an invalid port). */
SerialConfig GetSerialConfig(int port);

/**
 * Sets the pins of SPI bus 0 (SPI2) or 1 (SPI3). miso may be -1 for a bus that only transmits
 * (clocked LEDs). Returns false for an invalid bus or pin, or two equal pins. A bus already open
 * keeps its pins until its last device is closed.
 */
bool SetSPIPins(int bus, int sck, int mosi, int miso);
/** The configuration of an SPI bus (all -1 for an invalid bus or a bus without pins). */
SPIConfig GetSPIConfig(int bus);

/** Sets the CAN (TWAI) pins and bit rate. Returns false for an invalid pin or bit rate. */
bool SetCanConfig(int tx, int rx, uint32_t bitrate = 1000000);
/** The CAN configuration. */
CanConfig GetCanConfig();

}  // namespace prosdk::platform
