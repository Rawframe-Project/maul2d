// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for core.c: assertions, the finite-input
// checks and the profile clock. Every engine source reaches these
// through world_internal.h or directly.

#ifndef MAUL2D_SRC_CORE_H
#define MAUL2D_SRC_CORE_H

#include "maul2d/base.h"
#include "maul2d/core_math.h"

#include <stddef.h>

// M2_ASSERT states an internal invariant: a state that cannot occur
// unless the library itself is wrong. Caller input is never asserted; it
// is refused with a status or counted as misuse. The check traps in every
// build without NDEBUG (Debug, the sanitizer and fuzz builds), so a broken
// invariant stops the run where it happened. Release builds compile it
// out without evaluating the condition, so the solver's inner loops do not
// pay for it.
#if defined(NDEBUG)
#define M2_ASSERT(cond) ((void)sizeof((cond) ? 1 : 0))
#else
#define M2_ASSERT(cond) ((cond) ? (void)0 : __builtin_trap())
#endif

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
