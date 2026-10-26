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

#include <stdint.h>

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

#if defined(_MSC_VER) && defined(_M_X64)
#include <xmmintrin.h>
#elif defined(_MSC_VER) && defined(_M_ARM64)
#include <arm_neon.h>
#endif

    /// Pinned minimum: exactly (a < b ? a : b), in this operand order -
    /// including signed zero and NaN behavior. MSVC folds the plain ternary
    /// under value numbering (+0/-0 compare equal), so on MSVC x64 this is
    /// implemented with scalar MINSS, whose ISA-defined semantics are exactly
    /// the pinned form. NEON FMIN has different semantics and must never be
    /// used here; GCC/Clang compile the ternary faithfully (CI-verified).
    static inline float m2MinF(float a, float b)
    {
#if defined(_MSC_VER) && defined(_M_X64)
        return _mm_cvtss_f32(_mm_min_ss(_mm_set_ss(a), _mm_set_ss(b)));
#elif defined(_MSC_VER) && defined(_M_ARM64)
    // The plain ternary got folded away from the pinned zero-sign
    // semantics (the arm64 cell caught it on its first run), so the
    // select is explicit NEON: compare into a mask, bit-select the
    // lanes - never fminnm.
    float32x2_t va = vdup_n_f32(a);
    float32x2_t vb = vdup_n_f32(b);
    return vget_lane_f32(vbsl_f32(vclt_f32(va, vb), va, vb), 0);
#elif defined(_MSC_VER)
#error                                                                                             \
    "MSVC on this target: pinned min/max semantics unproven; implement with verified intrinsics first"
#else
    return a < b ? a : b;
#endif
    }

    /// Pinned maximum: exactly (a > b ? a : b), in this operand order.
    /// See m2MinF for the MSVC note (scalar MAXSS matches the pinned form).
    static inline float m2MaxF(float a, float b)
    {
#if defined(_MSC_VER) && defined(_M_X64)
        return _mm_cvtss_f32(_mm_max_ss(_mm_set_ss(a), _mm_set_ss(b)));
#elif defined(_MSC_VER) && defined(_M_ARM64)
    // See m2MinF: explicit NEON compare + bit-select.
    float32x2_t va = vdup_n_f32(a);
    float32x2_t vb = vdup_n_f32(b);
    return vget_lane_f32(vbsl_f32(vcgt_f32(va, vb), va, vb), 0);
#elif defined(_MSC_VER)
#error                                                                                             \
    "MSVC on this target: pinned min/max semantics unproven; implement with verified intrinsics first"
#else
    return a > b ? a : b;
#endif
    }

    /// Absolute value, pinned to IEEE |x| semantics: the sign bit is
    /// cleared, so m2AbsF(-0.0f) == +0.0f on every platform. Implemented
    /// as a bit mask because a ternary abs is compiler-foldable into
    /// divergent ±0 behavior (MSVC emits a sign-mask, GCC/Clang keep the
    /// branch - a real cross-cell hash break, caught by the gate).
    static inline float m2AbsF(float a)
    {
        union
        {
            float f;
            uint32_t u;
        } bits;
        bits.f = a;
        bits.u &= 0x7FFFFFFFu;
        return bits.f;
    }

    /// Clamp to [lo, hi] using the pinned min/max.
    static inline float m2ClampF(float a, float lo, float hi)
    {
        return m2MaxF(lo, m2MinF(a, hi));
    }

    /// Maps an angle to [-pi, pi], using only +, -, *, / and floorf, so the
    /// bits agree everywhere. The boundary lands within one rounding step
    /// of pi.
    ///
    /// @param radians  Any angle. Beyond about 1.0e6 in magnitude a float
    ///                 cannot fold precisely; such an angle returns the
    ///                 boundary, never NaN.
    /// @return The same angle in [-pi, pi].
    /// @par Thread safety
    /// Safe from any thread.
    M2_API float m2UnwindAngle(float radians);

    /// Builds a rotation from an angle without calling the C library, so the
    /// bits agree everywhere. Accurate to about 4e-7.
    ///
    /// @param radians  Any angle; it is unwound first (m2UnwindAngle), so an
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
