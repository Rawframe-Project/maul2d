// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark scenes: two pyramids, hanging chains of 30 and 2000
// links, a joint farm and a water tank.

#ifndef MAUL2D_BENCH_SCENES_H
#define MAUL2D_BENCH_SCENES_H

#include "maul2d/maul2d.h"

#define BENCH_DT       (1.0f / 60.0f)
#define BENCH_SUBSTEPS 4

typedef m2World* BenchSceneFn(void);

typedef struct BenchScene
{
    const char* name;
    BenchSceneFn* build;
    int32_t steps;
    float speedLimit; // a body faster than this at the end means the scene blew up
} BenchScene;

// The scenes, in a fixed order.
const BenchScene* BenchScenes(int32_t* count);

// The fastest body's speed, to tell a scene that blew up.
float BenchMaxBodySpeed(m2World* world);

#endif // MAUL2D_BENCH_SCENES_H
