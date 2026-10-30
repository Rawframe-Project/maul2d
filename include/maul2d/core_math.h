// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Deterministic 2D math: vectors, rotations, transforms and the
// engine's own trigonometry. Positions are 64-bit, everything local is
// 32-bit, and every operation is plain IEEE arithmetic so the bits agree
// on every platform.

#ifndef MAUL2D_CORE_MATH_H
#define MAUL2D_CORE_MATH_H

#include "maul2d/base.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // Everything in this header is pure math: no world, no state, no side
    // effects.

#define M2_PI 3.14159265359f

    /// Local-space / velocity-space vector. World positions use m2Pos2.
    typedef struct m2Vec2
    {
        float x, y;
    } m2Vec2;

    /// World-space position. Positions are double precision so that
    /// simulation quality does not degrade far from the origin.
    typedef struct m2Pos2
    {
        M2_ALIGN8 double x, y;
    } m2Pos2;

    /// A rotation stored as a unit complex number. No angles are stored;
    /// angles exist only at the API rim.
    typedef struct m2Rot
    {
        float c, s;
    } m2Rot;

    /// A body pose: double-precision position plus rotation.
    typedef struct m2Transform
    {
        m2Pos2 p;
        m2Rot q;
    } m2Transform;

    /// Builds a rotation from an angle without calling the C library, so the
    /// bits agree everywhere. Accurate to about 4e-7.
    ///
    /// @param radians  Any angle; it is mapped into [-pi, pi] first, so an
    ///                 out-of-range or non-finite angle never yields NaN.
    /// @return A unit rotation.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Rot m2MakeRot(float radians);

    /// The engine's atan2, accurate to about 3e-7, with the same bits
    /// everywhere.
    ///
    /// @param y  The sine side.
    /// @param x  The cosine side.
    /// @return The angle in [-pi, pi]; 0 for (0, 0) instead of NaN.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API float m2Atan2(float y, float x);

    /// Renormalizes a rotation. Every composition of rotations renormalizes
    /// at once: bounded drift is part of the determinism contract.
    ///
    /// @param q  Any rotation.
    /// @return The unit rotation in q's direction, for any finite nonzero
    ///         q however long or short; the identity when q is zero or
    ///         not finite, never NaN.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Rot m2NormalizeRot(m2Rot q);

    /// Composes two rotations, q followed by r, and renormalizes.
    ///
    /// @param q  The first rotation.
    /// @param r  The rotation applied after q.
    /// @return The unit rotation of both.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API m2Rot m2MulRot(m2Rot q, m2Rot r);

    /// Tells whether a rotation is unit length within the engine's
    /// tolerance.
    ///
    /// @param q  The rotation to test.
    /// @return True when q is unit length.
    /// @par Thread safety
    /// Safe from any thread.
    M2_API bool m2IsNormalizedRot(m2Rot q);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_CORE_MATH_H
