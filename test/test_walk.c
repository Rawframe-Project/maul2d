// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The seeded API walk (walk.h) over the regression bank, and over as many
// further seeds as the first argument asks for: test_walk 200 is the
// nightly deep run.

#include "walk.h"

// The regression bank: seeds that ever exposed a bug live here
// forever, run through all four promises on every build so a fix
// can never quietly regress. It starts with a handful of extra
// hand-picked seeds for breadth; when a future seed catches
// something, add it here with a one-line note on what it caught.
static const uint64_t s_regressionSeeds[] = {
    0x00000000B0A710ADull, // buoyancy + jelly under churn
    0x00000000C0FFEE99ull, // dense stacks with ratchet joints
    0x000000001CEB00DAull, // shatter into a flowing pool
    0x00000000FADEDCABull, // lifetime reaping mid-rollback
    0x81B0FDFA6D7E8E98ull, // belt-wake side effect lost on journal replay
    0x73C25689D3BE14F0ull, // contradictory joints overflowing their impulses into a NaN
};

// Deterministic extra seeds for the weekly deep sweep: a mixing
// sequence off a fixed base, so more depth never means less
// reproducibility.
static uint64_t ExtraSeed(int32_t i)
{
    uint64_t x = 0x243F6A8885A308D3ull + (uint64_t)(i + 1) * 0x9E3779B97F4A7C15ull;
    x ^= x >> 33;
    x *= 0xFF51AFD7ED558CCDull;
    x ^= x >> 33;
    return x;
}

int main(int argc, char** argv)
{
    int32_t extraSeeds = argc > 1 ? atoi(argv[1]) : 0;
    if (extraSeeds < 0)
    {
        extraSeeds = 0;
    }
    uint8_t* journal = malloc(WALK_JOURNAL_CAPACITY);
    int32_t regressionCount = (int32_t)(sizeof(s_regressionSeeds) / sizeof(s_regressionSeeds[0]));
    for (int32_t i = 0; i < regressionCount; ++i)
    {
        WalkSeed(s_regressionSeeds[i], journal);
    }
    for (int32_t i = 0; i < extraSeeds; ++i)
    {
        WalkSeed(ExtraSeed(i), journal);
    }
    free(journal);
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
