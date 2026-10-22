// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// DataLog memory and writer-thread configuration (design/PORT.md sections 7 and 7.1).
//
// Upstream sizes DataLog for a desktop/roboRIO-class host: 16 KiB blocks, up to 1 MiB of
// outgoing buffers and 256 KiB of cached free buffers, and DataLogBackgroundWriter refuses to log
// with less than 5 MiB free. The ESP32-S3 has about 320 KiB of internal SRAM, so the target
// defaults are scaled down; the host keeps the upstream values (the upstream tests use them).
//
// The vendored DataLog.hpp includes this header, so a sketch and the precompiled archive see the
// same values. Overriding one with -D changes a class layout constant; the archive must then be
// rebuilt with the same flag (tools/build/build.sh), or header and archive disagree.

#pragma once

#include <cstddef>
#include <cstdint>

#include "prosdk/platform/config.hpp"

#if PROSDK_TARGET_HW

/** Size of one DataLog buffer block in bytes (upstream: 16 KiB). */
#ifndef PROSDK_DATALOG_BLOCK_SIZE
#define PROSDK_DATALOG_BLOCK_SIZE (2 * 1024)
#endif
/** Outgoing buffers before logging pauses (upstream: 1 MiB / BLOCK_SIZE = 64). 32 KiB here. */
#ifndef PROSDK_DATALOG_MAX_BUFFER_COUNT
#define PROSDK_DATALOG_MAX_BUFFER_COUNT 16
#endif
/** Free buffers kept for reuse (upstream: 256 KiB / BLOCK_SIZE = 16). 8 KiB here. */
#ifndef PROSDK_DATALOG_MAX_FREE_COUNT
#define PROSDK_DATALOG_MAX_FREE_COUNT 4
#endif
/**
 * Minimum free space DataLogBackgroundWriter keeps on the log volume (upstream: 5 MiB). Only
 * enforced when the file system reports its free space (see DataLogBackgroundWriter.cpp).
 */
#ifndef PROSDK_DATALOG_MIN_FREE_SPACE
#define PROSDK_DATALOG_MIN_FREE_SPACE (64 * 1024)
#endif
/** Stack of the DataLogBackgroundWriter thread in bytes (std::format, file I/O). */
#ifndef PROSDK_DATALOG_THREAD_STACK_SIZE
#define PROSDK_DATALOG_THREAD_STACK_SIZE 6144
#endif

#else  // host: upstream values

#ifndef PROSDK_DATALOG_BLOCK_SIZE
#define PROSDK_DATALOG_BLOCK_SIZE (16 * 1024)
#endif
#ifndef PROSDK_DATALOG_MAX_BUFFER_COUNT
#define PROSDK_DATALOG_MAX_BUFFER_COUNT (1024 * 1024 / PROSDK_DATALOG_BLOCK_SIZE)
#endif
#ifndef PROSDK_DATALOG_MAX_FREE_COUNT
#define PROSDK_DATALOG_MAX_FREE_COUNT (256 * 1024 / PROSDK_DATALOG_BLOCK_SIZE)
#endif
#ifndef PROSDK_DATALOG_MIN_FREE_SPACE
#define PROSDK_DATALOG_MIN_FREE_SPACE (5 * 1024 * 1024)
#endif
#ifndef PROSDK_DATALOG_THREAD_STACK_SIZE
#define PROSDK_DATALOG_THREAD_STACK_SIZE 6144
#endif

#endif  // PROSDK_TARGET_HW

static_assert(PROSDK_DATALOG_BLOCK_SIZE >= 256, "DataLog blocks must hold a record header");
static_assert(PROSDK_DATALOG_MAX_BUFFER_COUNT >= 2, "DataLog pauses at MAX_BUFFER_COUNT / 2");
