// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// SIMD: the eight-lane min and max against the scalar ternary law they
// promise, a > b ? a : b and a < b ? a : b, bit for bit. AVX2 uses the
// native MAXPS and MINPS, which return the second operand when either is
// NaN or both are zero; NEON and scalar use compare and select. Every
// pair of the edge values below runs in every lane position, so a
// backend that orders signed zeros, propagates the first NaN, quiets a
// signaling NaN or swaps the operands fails here before it moves a hash.

#include "simd.h"
#include "test_harness.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static float FromBits(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static uint32_t ToBits(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return bits;
}

static float LawMax(float a, float b)
{
    return a > b ? a : b;
}

static float LawMin(float a, float b)
{
    return a < b ? a : b;
}

// NaN on either side (quiet, negative, with a payload, signaling), both
// zeros, equal values, infinities, a subnormal and ordinary numbers.
static const uint32_t k_edges[] = {
    0x7FC00000u, // quiet NaN
    0xFFC00000u, // negative quiet NaN
    0x7FC12345u, // quiet NaN with a payload
    0x7FA00000u, // signaling NaN
    0x00000000u, // +0
    0x80000000u, // -0
    0x3F800000u, // 1
    0xBF800000u, // -1
    0x3F800001u, // just above 1
    0x7F800000u, // +infinity
    0xFF800000u, // -infinity
    0x00000001u, // the smallest subnormal
    0x80000001u, // its negative
    0x7F7FFFFFu, // the largest finite
};

enum
{
    EDGE_COUNT = (int)(sizeof(k_edges) / sizeof(k_edges[0])),
    PAIR_COUNT = EDGE_COUNT * EDGE_COUNT
};

static void TestMinMaxFollowTheTernary(void)
{
    float as[PAIR_COUNT];
    float bs[PAIR_COUNT];
    for (int i = 0; i < EDGE_COUNT; ++i)
    {
        for (int j = 0; j < EDGE_COUNT; ++j)
        {
            as[i * EDGE_COUNT + j] = FromBits(k_edges[i]);
            bs[i * EDGE_COUNT + j] = FromBits(k_edges[j]);
        }
    }
    // Each pair in each of the eight lanes: start the block at every shift.
    int maxWrong = 0;
    int minWrong = 0;
    for (int shift = 0; shift < 8; ++shift)
    {
        for (int base = -shift; base < PAIR_COUNT; base += 8)
        {
            float a[8];
            float b[8];
            for (int lane = 0; lane < 8; ++lane)
            {
                int k = base + lane;
                k = k < 0 ? k + PAIR_COUNT : k % PAIR_COUNT;
                a[lane] = as[k];
                b[lane] = bs[k];
            }
            float hi[8];
            float lo[8];
            m2F8Store(hi, m2F8Max(m2F8Load(a), m2F8Load(b)));
            m2F8Store(lo, m2F8Min(m2F8Load(a), m2F8Load(b)));
            for (int lane = 0; lane < 8; ++lane)
            {
                if (ToBits(hi[lane]) != ToBits(LawMax(a[lane], b[lane])))
                {
                    if (maxWrong++ == 0)
                    {
                        printf("max(%08x, %08x) gave %08x\n", ToBits(a[lane]), ToBits(b[lane]),
                               ToBits(hi[lane]));
                    }
                }
                if (ToBits(lo[lane]) != ToBits(LawMin(a[lane], b[lane])))
                {
                    if (minWrong++ == 0)
                    {
                        printf("min(%08x, %08x) gave %08x\n", ToBits(a[lane]), ToBits(b[lane]),
                               ToBits(lo[lane]));
                    }
                }
            }
        }
    }
    CHECK(maxWrong == 0, "m2F8Max is a > b ? a : b in every lane");
    CHECK(minWrong == 0, "m2F8Min is a < b ? a : b in every lane");

    // The cases the law turns on, spelled out.
    CHECK(ToBits(LawMax(0.0f, -0.0f)) == 0x80000000u && ToBits(LawMin(-0.0f, 0.0f)) == 0u,
          "with both zeros the second operand comes back");
    CHECK(ToBits(LawMax(FromBits(0x7FC00000u), 1.0f)) == ToBits(1.0f) &&
              ToBits(LawMax(1.0f, FromBits(0x7FC00000u))) == 0x7FC00000u,
          "with a NaN on either side the second operand comes back");
}

int main(void)
{
#if defined(M2_SIMD_AVX2)
    printf("simd.h backend: avx2\n");
#elif defined(M2_SIMD_NEON)
    printf("simd.h backend: neon\n");
#else
    printf("simd.h backend: scalar\n");
#endif
    TestMinMaxFollowTheTernary();
    return s_failures == 0 ? 0 : 1;
}
