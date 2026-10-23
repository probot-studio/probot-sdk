// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

namespace prosdk::hal {
// prosdk: the DIO/PWM/analog counts follow the ESP32-S3 HAL (Esp32Internal.hpp kNumSmartIo =
// GPIO 0-48, kNumAnalogInputs = ADC1 channels up to 10), so host tests take the same pins as
// the robot. NUM_PWM_HEADERS = NUM_PWM_CHANNELS: no PWM channel takes the roboRIO MXP remap.
constexpr int32_t NUM_CAN_BUSES = 5;
constexpr int32_t ACCELEROMETERS = 1;
constexpr int32_t NUM_SMART_IO = 6;
constexpr int32_t NUM_ACCUMULATORS = 2;
constexpr int32_t NUM_ANALOG_INPUTS = 11;
constexpr int32_t NUM_ANALOG_OUTPUTS = 2;
constexpr int32_t NUM_COUNTERS = 8;
constexpr int32_t NUM_DIGITAL_HEADERS = 10;
constexpr int32_t NUM_PWM_HEADERS = 49;
constexpr int32_t NUM_DIGITAL_CHANNELS = 49;
constexpr int32_t NUM_PWM_CHANNELS = 49;
constexpr int32_t NUM_DIGITAL_PWM_OUTPUTS = 6;
constexpr int32_t NUM_ENCODERS = 8;
constexpr int32_t I2C_PORTS = 2;
constexpr int32_t NUM_INTERRUPTS = 8;
constexpr int32_t NUM_RELAY_CHANNELS = 8;
constexpr int32_t NUM_RELAY_HEADERS = NUM_RELAY_CHANNELS / 2;
constexpr int32_t NUM_CTREPCM_MODULES = 63;
constexpr int32_t NUM_CTRE_SOLENOID_CHANNELS = 8;
constexpr int32_t NUM_CTREPDP_MODULES = 63;
constexpr int32_t NUM_CTREPDP_CHANNELS = 16;
constexpr int32_t NUM_REVPDH_MODULES = 63;
constexpr int32_t NUM_REVPDH_CHANNELS = 24;
constexpr int32_t NUM_PD_SIM_MODULES = NUM_REVPDH_MODULES;
constexpr int32_t NUM_PD_SIM_CHANNELS = NUM_REVPDH_CHANNELS;
constexpr int32_t NUM_DUTY_CYCLES = 6;
constexpr int32_t NUM_ADDRESSABLE_LE_DS = 6;
constexpr int32_t NUM_REVPH_MODULES = 63;
constexpr int32_t NUM_REVPH_CHANNELS = 16;
constexpr int32_t SPI_ACCELEROMETERS = 5;
constexpr int32_t SPI_PORTS = 5;

}  // namespace prosdk::hal
