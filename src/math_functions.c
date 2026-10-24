// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic angle functions. libm's transcendentals are not required
// to agree bit for bit across platforms, so the engine evaluates its own:
// only +, -, *, / and the IEEE-exact floorf and sqrtf are used, and the
// build turns floating-point contraction off.

#include "math_functions.h"

#include "core.h"
#include "fpenv.h"
#include "rotation.h"

#include "maul2d/base.h"
#include "maul2d/core_math.h"

#include <math.h> // floorf, sqrtf only: both are IEEE-exact operations

// The math types enter snapshots and hashes as raw bytes, so their
// layout is part of the determinism contract.
_Static_assert(sizeof(m2Vec2) == 8, "m2Vec2 must be 8 bytes");
_Static_assert(sizeof(m2Pos2) == 16, "m2Pos2 must be 16 bytes");
_Static_assert(sizeof(m2Rot) == 8, "m2Rot must be 8 bytes");
_Static_assert(sizeof(m2Transform) == 24, "m2Transform must be 24 bytes, no padding");
_Static_assert(_Alignof(m2Vec2) == 4 && _Alignof(m2Rot) == 4, "float pair alignment");
_Static_assert(_Alignof(m2Pos2) == 8 && _Alignof(m2Transform) == 8, "double alignment");

M2_NOINLINE float m2UnwindAngleWork(float radians)
{
    return m2WrapAngle(radians);
}

float m2UnwindAngle(float radians)
{
    uint64_t env = m2FloatEnvEnter();
    float result = m2UnwindAngleWork(radians);
    m2FloatEnvLeave(env);
    return result;
}

M2_NOINLINE m2Rot m2MakeRotWork(float radians)
{
    return m2RotOfAngle(radians);
}

m2Rot m2MakeRot(float radians)
{
    uint64_t env = m2FloatEnvEnter();
    m2Rot result = m2MakeRotWork(radians);
    m2FloatEnvLeave(env);
    return result;
}

// atan2 from the ratio a = min(|x|, |y|) / max(|x|, |y|) in [0, 1]. Above
// tan(pi/12) the identity atan(a) = pi/6 + atan((sqrt(3) a - 1) / (a +
// sqrt(3))) moves the argument into [-tan(pi/12), tan(pi/12)], where the
// odd Taylor series through t^11 is accurate to below 3e-9. The octant and
// quadrant then follow from the signs and the larger component.
M2_NOINLINE float m2Atan2Work(float y, float x)
{
    // (0, 0) returns 0 instead of NaN: the sim path is NaN-free by contract.
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }
    float ax = m2AbsF(x);
    float ay = m2AbsF(y);
    float a = m2MinF(ay, ax) / m2MaxF(ay, ax);
    float base = 0.0f;
    if (a > 0.2679492f)
    {
        a = (1.7320508f * a - 1.0f) / (a + 1.7320508f);
        base = 0.5235988f;
    }
    float t2 = a * a;
    float r =
        base +
        a * (1.0f +
             t2 * (-0.33333334f +
                   t2 * (0.2f + t2 * (-0.14285715f + t2 * (0.11111111f + t2 * -0.09090909f)))));
    if (ay > ax)
    {
        r = 1.5707964f - r;
    }
    if (x < 0.0f)
    {
        r = 3.1415927f - r;
    }
    if (y < 0.0f)
    {
        r = -r;
    }
    return r;
}

float m2Atan2(float y, float x)
{
    uint64_t env = m2FloatEnvEnter();
    float result = m2Atan2Work(y, x);
    m2FloatEnvLeave(env);
    return result;
}

M2_NOINLINE m2Rot m2NormalizeRotWork(m2Rot q)
{
    return m2RotNormalized(q);
}

m2Rot m2NormalizeRot(m2Rot q)
{
    uint64_t env = m2FloatEnvEnter();
    m2Rot result = m2NormalizeRotWork(q);
    m2FloatEnvLeave(env);
    return result;
}

M2_NOINLINE m2Rot m2MulRotWork(m2Rot q, m2Rot r)
{
    return m2RotCompose(q, r);
}

m2Rot m2MulRot(m2Rot q, m2Rot r)
{
    uint64_t env = m2FloatEnvEnter();
    m2Rot result = m2MulRotWork(q, r);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE bool IsNormalizedRotWorker(m2Rot q)
{
    float mag2 = q.c * q.c + q.s * q.s;
    float tolerance = 4.0f * 1.19209290e-7f; // 4 * FLT_EPSILON
    return mag2 > 1.0f - tolerance && mag2 < 1.0f + tolerance;
}

bool m2IsNormalizedRot(m2Rot q)
{
    uint64_t env = m2FloatEnvEnter();
    bool result = IsNormalizedRotWorker(q);
    m2FloatEnvLeave(env);
    return result;
}
