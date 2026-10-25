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
// A body-local point or length within M2_HUGE; NaN fails both.
static inline bool m2LocalVec2(m2Vec2 v)
{
    return v.x >= -M2_HUGE && v.x <= M2_HUGE && v.y >= -M2_HUGE && v.y <= M2_HUGE;
}
static inline bool m2LocalLength(float x)
{
    return x >= 0.0f && x <= M2_HUGE;
}
// The ranges of base.h, one check per quantity, shared by the defs, the
// setters, replay and restore. NaN fails every one of them.
static inline bool m2WithinF(float x, float lo, float hi)
{
    return x >= lo && x <= hi;
}
static inline bool m2ValidSpeed(float x)
{
    return m2WithinF(x, -M2_MAX_SPEED, M2_MAX_SPEED);
}
static inline bool m2ValidVelocity(m2Vec2 v)
{
    return m2ValidSpeed(v.x) && m2ValidSpeed(v.y);
}
static inline bool m2ValidAcceleration(m2Vec2 a)
{
    return m2WithinF(a.x, -M2_MAX_ACCELERATION, M2_MAX_ACCELERATION) &&
           m2WithinF(a.y, -M2_MAX_ACCELERATION, M2_MAX_ACCELERATION);
}
static inline bool m2ValidGravityScale(float x)
{
    return m2WithinF(x, -M2_MAX_GRAVITY_SCALE, M2_MAX_GRAVITY_SCALE);
}
static inline bool m2ValidRate(float x)
{
    return m2WithinF(x, 0.0f, M2_MAX_RATE);
}
static inline bool m2ValidCoefficient(float x)
{
    return m2WithinF(x, 0.0f, M2_MAX_COEFFICIENT);
}
static inline bool m2ValidRestitution(float x)
{
    return m2WithinF(x, 0.0f, 1.0f);
}
static inline bool m2ValidDensity(float x)
{
    return m2WithinF(x, 0.0f, M2_MAX_DENSITY);
}
static inline bool m2ValidRatio(float x)
{
    float magnitude = x < 0.0f ? -x : x;
    return m2WithinF(magnitude, M2_MIN_RATIO, M2_MAX_RATIO);
}
static inline bool m2ValidMass(float x)
{
    return m2WithinF(x, M2_MIN_MASS, M2_MAX_MASS);
}
// Rotational inertia about the center of mass; 0 does not rotate.
static inline bool m2ValidInertia(float x)
{
    return x == 0.0f || m2WithinF(x, M2_MIN_INERTIA, M2_MAX_INERTIA);
}
static inline bool m2ValidForce(float x)
{
    return m2WithinF(x, -M2_MAX_FORCE, M2_MAX_FORCE);
}
static inline bool m2ValidForceVec(m2Vec2 f)
{
    return m2ValidForce(f.x) && m2ValidForce(f.y);
}
// A budget: a motor's or a mouse's largest force or torque.
static inline bool m2ValidMaxForce(float x)
{
    return m2WithinF(x, 0.0f, M2_MAX_FORCE);
}
static inline bool m2ValidImpulse(float x)
{
    return m2WithinF(x, -M2_IMPULSE_LIMIT, M2_IMPULSE_LIMIT);
}
static inline bool m2ValidImpulseVec(m2Vec2 j)
{
    return m2ValidImpulse(j.x) && m2ValidImpulse(j.y);
}
static inline bool m2ValidTimeStep(float x)
{
    return m2WithinF(x, M2_MIN_TIME_STEP, M2_MAX_TIME_STEP);
}
static inline bool m2ValidAngle(float x)
{
    return m2WithinF(x, -M2_MAX_ANGLE, M2_MAX_ANGLE);
}
// A joint length or translation, signed.
static inline bool m2ValidReach(float x)
{
    return m2WithinF(x, -(float)M2_JOINT_REACH, (float)M2_JOINT_REACH);
}
// A length a joint takes from its bodies' poses (a distance joint's
// derived rest length, one side of a pulley's rope): two points at most
// the reach apart per axis, plus anchor levers within M2_HUGE, so below
// the reach's diagonal and so below twice the reach.
static inline bool m2ValidSpan(float x)
{
    return m2WithinF(x, 0.0f, 2.0f * (float)M2_JOINT_REACH);
}
// A share from none to all: a motor joint's position correction.
static inline bool m2ValidFraction(float x)
{
    return m2WithinF(x, 0.0f, 1.0f);
}
// A ratchet's tooth pitch, signed by the free direction.
static inline bool m2ValidRatchetTooth(float x)
{
    float magnitude = x < 0.0f ? -x : x;
    return m2WithinF(magnitude, M2_MIN_RATCHET_TOOTH, M2_MAX_ANGLE);
}
// A joint's break force or torque: 0 turns breaking off, infinity never
// breaks.
static inline bool m2ValidBreakLimit(float x)
{
    return x >= 0.0f;
}
// A world-space difference as a float, clamped to M2_JOINT_REACH. World
// positions are 64-bit and unbounded, and their difference can pass what
// a float holds; it would reach the solver as infinity and leave NaN in
// the bodies. Clamped, a lever stays within the bound docs/user/limits.md
// budgets for. Two points this far apart no longer interact anyway: at
// 400 m/s, closing 1e9 m takes a month.
static inline float m2WorldOffset(double d)
{
    d = d < M2_JOINT_REACH ? d : M2_JOINT_REACH;
    d = d > -M2_JOINT_REACH ? d : -M2_JOINT_REACH;
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
