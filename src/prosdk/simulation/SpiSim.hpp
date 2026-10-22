// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of the prosdk SPI backend (prosdk/platform/hal/SPI.hpp,
// design/boardoza/F1-spi.md). Test code registers a callback per (bus, chip select); every
// transaction on an open device with that chip select calls it once with the whole frame (all
// segments joined, as one chip-select assertion; a transaction of 0 bytes calls nothing). rx is zero-filled before the call and has the same length as tx; without a
// callback a transaction succeeds and reads zeros. Defined in platform/hal/SPISim.cpp on the host
// build only (there is no SPI simulation on the robot).

#pragma once

#include <stdint.h>

#include <functional>
#include <span>

#include "prosdk/platform/hal/SPI.hpp"

namespace prosdk::sim {

/** Called for every transaction on (bus, cs): fill rx (same length as tx). */
using SpiTransferFn = std::function<void(std::span<const uint8_t> tx, std::span<uint8_t> rx)>;

/**
 * Registers a callback for the device with chip select cs (-1: the device without chip select)
 * on bus 0 or 1. Several callbacks on one device are called in registration order with the same
 * rx. The callback runs on the transferring thread with the simulation's lock held, so
 * CancelSpiDevice from another thread waits until a call in progress returns.
 *
 * @return An id for CancelSpiDevice.
 */
int RegisterSpiDevice(int bus, int cs, SpiTransferFn fn);

/** Removes a callback; returns after any call of it in progress finished. */
void CancelSpiDevice(int id);

/**
 * The configuration of the open device with chip select cs on a bus (what prosdk::SPI last set).
 * Returns false when no such device is open.
 */
bool GetSpiDeviceConfig(int bus, int cs, prosdk::platform::hal::SpiDeviceConfig* config);

}  // namespace prosdk::sim
