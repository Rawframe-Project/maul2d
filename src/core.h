// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, refusals, the memory
// hooks, the CPU check, the finite-input checks and the profile clock.
// Every engine source reaches these through world_internal.h or
// directly.

#ifndef MAUL2D_SRC_CORE_H
#define MAUL2D_SRC_CORE_H

#include "maul2d/base.h"
#include "maul2d/core_math.h"

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

// Records the reason m2LastResult reports on this thread.
void m2SetLastResult(m2Result reason);

// All engine memory goes through the host's allocator hooks.
void* m2AllocZeroed(size_t bytes);
void m2Free(void* memory);

// Hands a message to the host's assert hook without aborting; returns
// nonzero when the host handled it.
int m2ReportToHost(const char* message, const char* where);

// Finite checks for caller input and invariants. NaN compares false
// and infinity minus infinity is NaN, so x - x == 0 refuses NaN and
// both infinities in one test without libm.
static inline bool m2FiniteF(float x)
{
    return x - x == 0.0f; // NOLINT(misc-redundant-expression): the finite test
}
static inline bool m2FiniteD(double x)
{
    return x - x == 0.0; // NOLINT(misc-redundant-expression): the finite test
}
static inline bool m2FiniteVec2(m2Vec2 v)
{
    return m2FiniteF(v.x) && m2FiniteF(v.y);
}
static inline bool m2FinitePos2(m2Pos2 p)
{
    return m2FiniteD(p.x) && m2FiniteD(p.y);
}
// A caller rotation that is stored or used as given must already be a
// unit rotation (m2MakeRot builds one); anything else would shear.
static inline bool m2UnitRot(m2Rot q)
{
    float length2 = q.c * q.c + q.s * q.s;
    return m2FiniteF(q.c) && m2FiniteF(q.s) && length2 > 0.999f && length2 < 1.001f;
}

// Monotonic profile clock (observer only, never a hash input).
uint64_t m2TimeNowNs(void);

#endif // MAUL2D_SRC_CORE_H
