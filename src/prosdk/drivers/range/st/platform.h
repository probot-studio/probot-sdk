// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Platform layer of ST's VL53L7CX Ultra Lite Driver (ULD 2.0.0, vendored next to this file:
// vl53l7cx_api.c/.h and vl53l7cx_buffers.h from github.com/STMicroelectronics/x-cube-tof1,
// Drivers/BSP/Components/vl53l7cx/modules, BSD-3-Clause, licenses/st-vl53l7cx/LICENSE). The ULD
// includes "platform.h" and expects the type and functions below; their names and signatures are
// the ULD's. prosdk implements them on prosdk::I2C in platform.cpp (design/boardoza/P4-tof.md).
//
// C header: vl53l7cx_api.c is compiled as C.

#ifndef PROSDK_DRIVERS_RANGE_ST_PLATFORM_H_
#define PROSDK_DRIVERS_RANGE_ST_PLATFORM_H_

#include <stdint.h>
#include <string.h>

/*
 * Sensor outputs the ULD reads in every frame. prosdk::VL53L7CX uses the distance and the target
 * status; the number of targets lets the ULD mark zones without a target (status 255). The other
 * outputs are switched off with the ULD's own option macros, which cuts a frame from about 1.4 KB
 * to 128 bytes (4x4) or 320 bytes (8x8) of I2C reads.
 */
#define VL53L7CX_NB_TARGET_PER_ZONE (1U)
#define VL53L7CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L7CX_DISABLE_NB_SPADS_ENABLED
#define VL53L7CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L7CX_DISABLE_RANGE_SIGMA_MM
#define VL53L7CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L7CX_DISABLE_MOTION_INDICATOR

#ifdef __cplusplus
extern "C" {
#endif

/* One sensor on the bus. */
typedef struct {
  /* I2C address in the ULD's 8-bit form (7-bit address << 1; the default is 0x52).
   * vl53l7cx_set_i2c_address() changes it. */
  uint16_t address;
  /* prosdk::detail::VL53L7CXBus (st/VL53L7CXPlatform.hpp) that carries the transfers. */
  void* bus;
} VL53L7CX_Platform;

/* Reads one register. Returns 0 on success. */
uint8_t VL53L7CX_RdByte(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_value);

/* Writes one register. Returns 0 on success. */
uint8_t VL53L7CX_WrByte(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t value);

/* Reads size bytes from RegisterAdress on (auto-increment). Returns 0 on success. */
uint8_t VL53L7CX_RdMulti(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values,
                         uint32_t size);

/* Writes size bytes from RegisterAdress on (auto-increment). Returns 0 on success. */
uint8_t VL53L7CX_WrMulti(VL53L7CX_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values,
                         uint32_t size);

/* Reverses the byte order of each 32-bit word (the sensor sends big-endian words; size is a
 * multiple of 4). */
void VL53L7CX_SwapBuffer(uint8_t* buffer, uint16_t size);

/* Waits TimeMs milliseconds. Returns 0. */
uint8_t VL53L7CX_WaitMs(VL53L7CX_Platform* p_platform, uint32_t TimeMs);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PROSDK_DRIVERS_RANGE_ST_PLATFORM_H_
