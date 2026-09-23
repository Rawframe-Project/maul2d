// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic angle functions. libm's transcendentals are not required
// to agree bit for bit across platforms, so the engine evaluates its own:
// only +, -, *, / and the IEEE-exact floorf and sqrtf are used, and the
// build turns floating-point contraction off.

#include "core.h"

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

float m2UnwindAngle(float radians)
{
    // Map toward [-pi, pi] using only *, +, - and floorf, then re-fold
    // once: twoPi * k carries rounding error, so a single fold pass pulls
    // boundary spill back in. The final clamp is the deterministic answer
    // for angles too large to fold precisely (beyond about 1e6 radians),
    // which caller or snapshot data can carry.
    float twoPi = 2.0f * M2_PI;
    float k = floorf((radians + M2_PI) / twoPi);
    float u = radians - twoPi * k;
    if (u > M2_PI)
    {
        u = u - twoPi;
    }
    if (u < -M2_PI)
    {
        u = u + twoPi;
    }
    return m2ClampF(u, -M2_PI, M2_PI);
}

// Cosine and sine of an angle in [-pi, pi]. The angle is reduced to r in
// [-pi/4, pi/4] around the nearest multiple of pi/2, with pi/2 split into
// a float-exact high part and a low correction so the reduction loses no
// precision. On that interval the Taylor series of sine through r^9 and of
// cosine through r^8 are accurate to well below float resolution
// (truncation under 3e-8), and the quadrant maps the pair back.
static void CosSin(float x, float* cosine, float* sine)
{
    const float halfPiHigh = 1.5707964f;    // pi/2 rounded to float
    const float halfPiLow = -4.371139e-08f; // pi/2 - halfPiHigh
    float k = floorf(x * 0.63661975f + 0.5f);
    float r = (x - k * halfPiHigh) - k * halfPiLow;
    float r2 = r * r;
    float s = r * (1.0f + r2 * (-0.16666667f +
                                r2 * (0.008333334f + r2 * (-0.0001984127f + r2 * 2.7557319e-06f))));
    float c =
        1.0f + r2 * (-0.5f + r2 * (0.041666668f + r2 * (-0.0013888889f + r2 * 2.4801588e-05f)));
    switch ((int32_t)k & 3)
    {
    case 0:
        *cosine = c;
        *sine = s;
        break;
    case 1:
        *cosine = -s;
        *sine = c;
        break;
    case 2:
        *cosine = -c;
        *sine = -s;
        break;
    default:
        *cosine = s;
        *sine = -c;
        break;
    }
}

m2Rot m2MakeRot(float radians)
{
    float c;
    float s;
    CosSin(m2UnwindAngle(radians), &c, &s);
    // The series are accurate to float resolution already; normalizing
    // makes the pair a unit rotation to the last bit.
    return m2NormalizeRot((m2Rot){c, s});
}

// atan2 from the ratio a = min(|x|, |y|) / max(|x|, |y|) in [0, 1]. Above
// tan(pi/12) the identity atan(a) = pi/6 + atan((sqrt(3) a - 1) / (a +
// sqrt(3))) moves the argument into [-tan(pi/12), tan(pi/12)], where the
// odd Taylor series through t^11 is accurate to below 3e-9. The octant and
// quadrant then follow from the signs and the larger component.
float m2Atan2(float y, float x)
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

m2Rot m2NormalizeRot(m2Rot q)
{
    float mag = sqrtf(q.c * q.c + q.s * q.s);
    if (!(mag > 0.0f))
    {
        // Degenerate input: deterministic identity, never NaN (a zero
        // inverse magnitude would manufacture the {0,0} non-rotation).
        m2Rot identity = {1.0f, 0.0f};
        return identity;
    }
    float invMag = 1.0f / mag;
    m2Rot result = {q.c * invMag, q.s * invMag};
    return result;
}

m2Rot m2MulRot(m2Rot q, m2Rot r)
{
    // Complex multiply, then renormalize: every composition site
    // renormalizes immediately (drift control is part of the contract).
    m2Rot qr;
    qr.c = q.c * r.c - q.s * r.s;
    qr.s = q.s * r.c + q.c * r.s;
    return m2NormalizeRot(qr);
}

int m2IsNormalizedRot(m2Rot q)
{
    float mag2 = q.c * q.c + q.s * q.s;
    float tolerance = 4.0f * 1.19209290e-7f; // 4 * FLT_EPSILON
    return mag2 > 1.0f - tolerance && mag2 < 1.0f + tolerance;
}
