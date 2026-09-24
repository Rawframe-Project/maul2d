// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The angle and rotation functions as inline code for the hot paths (the
// pose update runs them for every body every substep). The public
// m2UnwindAngle, m2MakeRot, m2NormalizeRot and m2MulRot are these same
// functions, so the two can never differ by a bit. Only +, -, *, / and
// the IEEE-exact floorf and sqrtf are used.

#ifndef MAUL2D_SRC_ROTATION_H
#define MAUL2D_SRC_ROTATION_H

#include "maul2d/base.h"
#include "maul2d/core_math.h"

#include <math.h>

static inline float m2WrapAngle(float radians)
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
static inline void m2CosSinOf(float x, float* cosine, float* sine)
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

static inline m2Rot m2RotNormalized(m2Rot q)
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

static inline m2Rot m2RotOfAngle(float radians)
{
    float c;
    float s;
    m2CosSinOf(m2WrapAngle(radians), &c, &s);
    // The series are accurate to float resolution already; normalizing
    // makes the pair a unit rotation to the last bit.
    return m2RotNormalized((m2Rot){c, s});
}

static inline m2Rot m2RotCompose(m2Rot q, m2Rot r)
{
    // Complex multiply, then renormalize: every composition site
    // renormalizes immediately (drift control is part of the contract).
    m2Rot qr;
    qr.c = q.c * r.c - q.s * r.s;
    qr.s = q.s * r.c + q.c * r.s;
    return m2RotNormalized(qr);
}

#endif // MAUL2D_SRC_ROTATION_H
