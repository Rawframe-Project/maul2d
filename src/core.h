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
// The status checks of caller input against the published bounds of
// base.h, shared by the defs, the setters, replay and restore: a value
// past a bound is m2_errorLimit, infinities included, and NaN or a value
// outside the quantity's own domain is m2_errorInvalid. m2BothChecks
// joins two checks, an invalid value outranking one past a bound, so a
// call's status does not depend on the order it checks its fields in.
static inline m2Result m2BothChecks(m2Result a, m2Result b)
{
    if (a == m2_errorInvalid || b == m2_errorInvalid)
    {
        return m2_errorInvalid;
    }
    return a != m2_success ? a : b;
}
static inline m2Result m2CheckValid(bool valid)
{
    return valid ? m2_success : m2_errorInvalid;
}
// x within [lo, hi], both published bounds.
static inline m2Result m2CheckRange(float x, float lo, float hi)
{
    if (x >= lo && x <= hi)
    {
        return m2_success;
    }
    return x != x ? m2_errorInvalid : m2_errorLimit; // NOLINT(misc-redundant-expression): NaN
}
static inline m2Result m2CheckWithin(float x, float limit)
{
    return m2CheckRange(x, -limit, limit);
}
static inline m2Result m2CheckWithinVec2(m2Vec2 v, float limit)
{
    return m2BothChecks(m2CheckWithin(v.x, limit), m2CheckWithin(v.y, limit));
}
// x from zero to limit: below zero is invalid, as for a rate, whose sign
// has no meaning.
static inline m2Result m2CheckNonNegative(float x, float limit)
{
    return x >= 0.0f ? m2CheckRange(x, 0.0f, limit) : m2_errorInvalid;
}
// x from floor to limit, positive: zero or below is invalid, as for a
// mass, and a positive value below the floor is past a bound.
static inline m2Result m2CheckPositive(float x, float floor, float limit)
{
    return x > 0.0f ? m2CheckRange(x, floor, limit) : m2_errorInvalid;
}
// A magnitude from floor to limit, of either sign, as for a ratio.
static inline m2Result m2CheckMagnitude(float x, float floor, float limit)
{
    return m2CheckRange(x < 0.0f ? -x : x, floor, limit);
}
// A count from zero to max; a negative count is invalid.
static inline m2Result m2CheckCount(int32_t count, int32_t max)
{
    if (count < 0)
    {
        return m2_errorInvalid;
    }
    return count <= max ? m2_success : m2_errorLimit;
}

// A body-local point or length within M2_LOCAL_LIMIT.
static inline m2Result m2CheckLocalVec2(m2Vec2 v)
{
    return m2CheckWithinVec2(v, M2_LOCAL_LIMIT);
}
static inline m2Result m2CheckLocalLength(float x)
{
    return m2CheckNonNegative(x, M2_LOCAL_LIMIT);
}
static inline m2Result m2CheckSpeed(float x)
{
    return m2CheckWithin(x, M2_SPEED_LIMIT);
}
static inline m2Result m2CheckVelocity(m2Vec2 v)
{
    return m2CheckWithinVec2(v, M2_SPEED_LIMIT);
}
static inline m2Result m2CheckAcceleration(m2Vec2 a)
{
    return m2CheckWithinVec2(a, M2_ACCEL_LIMIT);
}
static inline m2Result m2CheckGravityScale(float x)
{
    return m2CheckWithin(x, M2_GRAVITY_SCALE_LIMIT);
}
static inline m2Result m2CheckRate(float x)
{
    return m2CheckNonNegative(x, M2_RATE_LIMIT);
}
static inline m2Result m2CheckCoefficient(float x)
{
    return m2CheckNonNegative(x, M2_COEFFICIENT_LIMIT);
}
// From 0 to 1 with no published bound: its own domain.
static inline m2Result m2CheckRestitution(float x)
{
    return m2CheckValid(x >= 0.0f && x <= 1.0f);
}
static inline m2Result m2CheckDensity(float x)
{
    return m2CheckNonNegative(x, M2_DENSITY_LIMIT);
}
static inline m2Result m2CheckRatio(float x)
{
    return m2CheckMagnitude(x, M2_MIN_RATIO, M2_RATIO_LIMIT);
}
static inline m2Result m2CheckMass(float x)
{
    return m2CheckPositive(x, M2_MIN_MASS, M2_MASS_LIMIT);
}
// Rotational inertia about the center of mass; 0 does not rotate.
static inline m2Result m2CheckInertia(float x)
{
    return x == 0.0f ? m2_success : m2CheckPositive(x, M2_MIN_INERTIA, M2_INERTIA_LIMIT);
}
static inline m2Result m2CheckForce(float x)
{
    return m2CheckWithin(x, M2_FORCE_LIMIT);
}
static inline m2Result m2CheckForceVec(m2Vec2 f)
{
    return m2CheckWithinVec2(f, M2_FORCE_LIMIT);
}
// A budget: a motor's or a mouse's largest force or torque.
static inline m2Result m2CheckMaxForce(float x)
{
    return m2CheckNonNegative(x, M2_FORCE_LIMIT);
}
static inline m2Result m2CheckImpulse(float x)
{
    return m2CheckWithin(x, M2_IMPULSE_LIMIT);
}
static inline m2Result m2CheckImpulseVec(m2Vec2 j)
{
    return m2CheckWithinVec2(j, M2_IMPULSE_LIMIT);
}
static inline m2Result m2CheckTimeStep(float x)
{
    return m2CheckPositive(x, M2_MIN_STEP, M2_STEP_LIMIT);
}
static inline m2Result m2CheckAngle(float x)
{
    return m2CheckWithin(x, M2_ANGLE_LIMIT);
}
// A joint length or translation, signed.
static inline m2Result m2CheckReach(float x)
{
    return m2CheckWithin(x, (float)M2_REACH_LIMIT);
}
// A length a joint takes from its bodies' poses (a distance joint's
// derived rest length, one side of a pulley's rope): two points at most
// the reach apart per axis, plus anchor levers within M2_LOCAL_LIMIT, so
// below the reach's diagonal and so below twice the reach.
static inline m2Result m2CheckSpan(float x)
{
    return m2CheckNonNegative(x, 2.0f * (float)M2_REACH_LIMIT);
}
// A share from none to all, with no published bound: a motor joint's
// position correction.
static inline m2Result m2CheckFraction(float x)
{
    return m2CheckValid(x >= 0.0f && x <= 1.0f);
}
// A ratchet's tooth pitch, signed by the free direction.
static inline m2Result m2CheckRatchetTooth(float x)
{
    return m2CheckMagnitude(x, M2_MIN_RATCHET_TOOTH, M2_ANGLE_LIMIT);
}
// A joint's break force or torque: 0 turns breaking off, infinity never
// breaks.
static inline m2Result m2CheckBreakLimit(float x)
{
    return m2CheckValid(x >= 0.0f);
}

// The same checks as a yes or no, for restore, replay and the setters
// that return nothing.
static inline bool m2LocalVec2(m2Vec2 v)
{
    return m2CheckLocalVec2(v) == m2_success;
}
static inline bool m2LocalLength(float x)
{
    return m2CheckLocalLength(x) == m2_success;
}
static inline bool m2WithinF(float x, float lo, float hi)
{
    return x >= lo && x <= hi;
}
static inline bool m2ValidSpeed(float x)
{
    return m2CheckSpeed(x) == m2_success;
}
static inline bool m2ValidVelocity(m2Vec2 v)
{
    return m2CheckVelocity(v) == m2_success;
}
static inline bool m2ValidAcceleration(m2Vec2 a)
{
    return m2CheckAcceleration(a) == m2_success;
}
static inline bool m2ValidGravityScale(float x)
{
    return m2CheckGravityScale(x) == m2_success;
}
static inline bool m2ValidRate(float x)
{
    return m2CheckRate(x) == m2_success;
}
static inline bool m2ValidCoefficient(float x)
{
    return m2CheckCoefficient(x) == m2_success;
}
static inline bool m2ValidRestitution(float x)
{
    return m2CheckRestitution(x) == m2_success;
}
static inline bool m2ValidDensity(float x)
{
    return m2CheckDensity(x) == m2_success;
}
static inline bool m2ValidRatio(float x)
{
    return m2CheckRatio(x) == m2_success;
}
static inline bool m2ValidMass(float x)
{
    return m2CheckMass(x) == m2_success;
}
static inline bool m2ValidInertia(float x)
{
    return m2CheckInertia(x) == m2_success;
}
static inline bool m2ValidForce(float x)
{
    return m2CheckForce(x) == m2_success;
}
static inline bool m2ValidForceVec(m2Vec2 f)
{
    return m2CheckForceVec(f) == m2_success;
}
static inline bool m2ValidMaxForce(float x)
{
    return m2CheckMaxForce(x) == m2_success;
}
static inline bool m2ValidImpulse(float x)
{
    return m2CheckImpulse(x) == m2_success;
}
static inline bool m2ValidImpulseVec(m2Vec2 j)
{
    return m2CheckImpulseVec(j) == m2_success;
}
static inline bool m2ValidTimeStep(float x)
{
    return m2CheckTimeStep(x) == m2_success;
}
static inline bool m2ValidAngle(float x)
{
    return m2CheckAngle(x) == m2_success;
}
static inline bool m2ValidReach(float x)
{
    return m2CheckReach(x) == m2_success;
}
static inline bool m2ValidSpan(float x)
{
    return m2CheckSpan(x) == m2_success;
}
static inline bool m2ValidFraction(float x)
{
    return m2CheckFraction(x) == m2_success;
}
static inline bool m2ValidRatchetTooth(float x)
{
    return m2CheckRatchetTooth(x) == m2_success;
}
static inline bool m2ValidBreakLimit(float x)
{
    return m2CheckBreakLimit(x) == m2_success;
}
// A world-space difference as a float, clamped to M2_REACH_LIMIT. World
// positions are 64-bit and unbounded, and their difference can pass what
// a float holds; it would reach the solver as infinity and leave NaN in
// the bodies. Clamped, a lever stays within the bound docs/user/limits.md
// budgets for. Two points this far apart no longer interact anyway: at
// 400 m/s, closing 1e9 m takes a month.
static inline float m2WorldOffset(double d)
{
    d = d < M2_REACH_LIMIT ? d : M2_REACH_LIMIT;
    d = d > -M2_REACH_LIMIT ? d : -M2_REACH_LIMIT;
    return (float)d;
}
// Saturation: a velocity component or an accumulated load held to its
// range.
static inline float m2SaturateF(float x, float limit)
{
    return x < -limit ? -limit : (x > limit ? limit : x);
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
