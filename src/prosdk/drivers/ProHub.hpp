// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// ProHub expansion board constants (design/drivers.md, "ProHub"): a PCA9685 PWM controller on
// I2C (motor ports and servo channels) and a co-processor that streams the encoder counts over a
// UART. Values from the Probot Lib 0.2.6 board file (probot/prohub/prohub_board.hpp).

#pragma once

#include <stdint.h>

#include "prosdk/units/frequency.hpp"

namespace prosdk::prohub {

/// ESP32 GPIO of the ProHub I2C SDA line.
inline constexpr int kI2CSda = 1;
/// ESP32 GPIO of the ProHub I2C SCL line.
inline constexpr int kI2CScl = 2;
/// ProHub I2C clock.
inline constexpr uint32_t kI2CFrequency = 400000;

/// I2C address of the first PCA9685 (motor ports 1-4, servo channels 12-15).
inline constexpr int kPcaAddress0 = 0x40;
/// I2C address of the second PCA9685 (motor ports 5-8; planned).
inline constexpr int kPcaAddress1 = 0x41;

/// PCA9685 frequency of the motor ports in Probot Lib 0.2.6 (prescale 6).
inline constexpr prosdk::units::hertz_t kMotorPwmFrequency{872};
/// Highest PCA9685 frequency at which a servo (2.4 ms pulse) still fits the period.
inline constexpr prosdk::units::hertz_t kMaxServoFrequency{400};

/// Number of motor ports (4 per PCA9685).
inline constexpr int kNumMotorPorts = 8;
/// First PCA9685 channel of the servo outputs (12 to 15).
inline constexpr int kFirstServoChannel = 12;
/// Number of servo outputs per PCA9685.
inline constexpr int kServosPerPca = 4;

/// ESP32 GPIO receiving the encoder stream (co-processor TX).
inline constexpr int kEncoderRx = 10;
/// ESP32 GPIO to the co-processor RX.
inline constexpr int kEncoderTx = 11;
/// Encoder stream baud rate.
inline constexpr int kEncoderBaudRate = 921600;
/// Number of encoder channels in the stream (encoder i belongs to motor port i + 1).
inline constexpr int kNumEncoders = 8;

/** The two PCA9685 channels of a motor port (H-bridge inputs). */
struct MotorChannels {
  int forward;  ///< driven for a positive throttle (LPWM in Probot Lib)
  int reverse;  ///< driven for a negative throttle (RPWM in Probot Lib)
};

/**
 * The PCA9685 channels of a motor port: port 1 is channels 0/1, port 2 is 3/4, port 3 is 6/7,
 * port 4 is 9/10; ports 5-8 repeat this on the second PCA9685.
 *
 * @param port Motor port, 1 to 8.
 */
constexpr MotorChannels GetMotorChannels(int port) {
  int base = ((port - 1) % 4) * 3;
  return {base, base + 1};
}

/** The PCA9685 address that serves a motor port. */
constexpr int GetMotorPcaAddress(int port) {
  return port <= 4 ? kPcaAddress0 : kPcaAddress1;
}

/**
 * Points I2C port 0 at the ProHub bus (SDA 1, SCL 2, 400 kHz) and the onboard serial port at the
 * encoder stream (RX 10, TX 11). Call it before a PCA9685, ProHubEncoders or other port 0 device is
 * constructed. Members are constructed before the robot constructor's body runs, so call it from a
 * member declared above them: `bool m_pinsConfigured = prosdk::prohub::ConfigurePins();` as the
 * first member (examples/DriversShowcase). Returns false if a pin was refused.
 *
 * probot-core's optional INA219/INA226 battery sensor (PROBOT_BATTERY_INA) also uses I2C
 * controller 0 (Arduino's Wire, prosdk port 0), at PCA9685 address 0x40 by default
 * (PROBOT_BATTERY_INA_ADDR). Once core has started that bus on its pins (GPIO 8/9 unless
 * PROBOT_BATTERY_INA_SDA/SCL), port 0 shares it as is and the pins set here do not apply. To use
 * both, wire the INA to the ProHub bus (PROBOT_BATTERY_INA_SDA 1, PROBOT_BATTERY_INA_SCL 2) and
 * give it an address other than 0x40 and 0x41 (PROBOT_BATTERY_INA_ADDR, set by its A0/A1 pins).
 * With the INA enabled the bus runs at 100 kHz, not 400 kHz: core sets Wire.setClock(100000)
 * whichever side starts the bus first, so every port 0 transfer takes about 4 times longer.
 */
bool ConfigurePins();

}  // namespace prosdk::prohub
