// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Host simulation of the pulse I/O backend (prosdk/platform/hal/PulseIO.hpp,
// design/boardoza/F3-timed-io.md). Test code watches the trains a transmitter sends and feeds
// line waveforms into a capture. Defined in platform/hal/PulseIOSim.cpp on the host build only
// (there is no pulse I/O simulation on the robot).

#pragma once

#include <stdint.h>

#include <functional>
#include <span>

#include "prosdk/platform/hal/PulseIO.hpp"

namespace prosdk::sim {

/** Called for every train a transmitter on the GPIO sends, with its carrier (0 = none). */
using PulsesSentFn =
    std::function<void(std::span<const prosdk::platform::hal::Pulse> pulses, uint32_t carrierHz)>;

/**
 * Registers a callback for the trains sent on `gpio`. Several callbacks on one GPIO are called in
 * registration order. The callback runs on the sending thread with the simulation's lock held
 * (it may call InjectPulses, for a loopback).
 *
 * @return An id for CancelPulsesSent.
 */
int OnPulsesSent(int gpio, PulsesSentFn fn);

/** Removes a callback; returns after any call of it in progress finished. */
void CancelPulsesSent(int id);

/**
 * Feeds a line waveform into the capture open on `gpio`, processed as the RMT receiver does:
 * levels shorter than the glitch filter are ignored (the line keeps its previous level), equal
 * neighbouring levels merge, a level of at least the idle time ends the burst (a pulse of 0 us at
 * that level) and the line then idles at that level until it changes. Every other level of the
 * waveform is one captured pulse, the last one included. A full buffer drops pulses as on the
 * robot.
 *
 * @return False when no capture is open on `gpio`.
 */
bool InjectPulses(int gpio, std::span<const prosdk::platform::hal::Pulse> line);

/**
 * The carrier of the transmitter open on `gpio` (0 Hz: none). Returns false when no transmitter
 * is open there.
 */
bool GetPulseTxCarrier(int gpio, uint32_t* carrierHz, double* carrierDuty);

}  // namespace prosdk::sim
