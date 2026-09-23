// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, refusals, the memory
// hooks and the CPU check. Every engine source reaches these through
// world_internal.h or directly.

#ifndef MAUL2D_SRC_CORE_H
#define MAUL2D_SRC_CORE_H

#include "maul2d/base.h"

#include <stddef.h>

// Internal invariants only: states that cannot happen unless the engine
// itself is wrong. Caller input is refused with m2Refuse, never asserted.
#if defined(NDEBUG)
#define M2_ASSERT(cond) ((void)0)
#else
#define M2_ASSERT(cond) ((cond) ? (void)0 : m2AssertFail(#cond, __FILE__, __LINE__))
#endif

// Reports a failed invariant to the host's assert handler, and aborts
// unless the handler returns nonzero.
void m2AssertFail(const char* condition, const char* file, int line);

typedef struct m2World m2World;

// Refuses a caller's input: records the reason for m2LastResult on this
// thread and, for an invalid argument against a live world, counts it in
// m2Counters.misuse. world may be NULL.
void m2Refuse(m2World* world, m2Result reason);
uint64_t m2MisuseCount(const m2World* world);

// All engine memory goes through the host's allocator hooks.
void* m2AllocZeroed(size_t bytes);
void m2Free(void* memory);

// 0 when the CPU cannot run the compiled SIMD backend (src/cpu.c).
int m2VerifyCpuBackend(void);

// Hands a message to the host's assert hook without aborting; returns
// nonzero when the host handled it.
int m2ReportToHost(const char* message, const char* where);

#endif // MAUL2D_SRC_CORE_H
