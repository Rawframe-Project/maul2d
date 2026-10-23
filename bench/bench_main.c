// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Benchmarks. Wall time is not deterministic, so timings are reported,
// never gated. Each scene also prints its final world hash, so two runs
// can be compared for the same physics, not just the clock.
// Pass a step count to run every scene longer than its default.

#ifndef _WIN32
#define _POSIX_C_SOURCE 199309L // clock_gettime under strict C17
#endif

#include "scenes.h"

#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
static double NowMs(void)
{
    LARGE_INTEGER freq;
    LARGE_INTEGER now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return 1000.0 * (double)now.QuadPart / (double)freq.QuadPart;
}
#else
#include <time.h>
static double NowMs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1000.0 * (double)ts.tv_sec + 1.0e-6 * (double)ts.tv_nsec;
}
#endif

int main(int argc, char** argv)
{
    int32_t override = argc > 1 ? atoi(argv[1]) : 0;
    int32_t failures = 0;
    int32_t sceneCount = 0;
    const BenchScene* scenes = BenchScenes(&sceneCount);
    for (int32_t k = 0; k < sceneCount; ++k)
    {
        const BenchScene* scene = &scenes[k];
        int32_t steps = override > 0 ? override : scene->steps;
        m2World* world = scene->build();
        double t0 = NowMs();
        for (int32_t i = 0; i < steps; ++i)
        {
            m2World_Step(world, BENCH_DT, BENCH_SUBSTEPS);
        }
        double totalMs = NowMs() - t0;
        uint64_t hash = m2World_Hash(world);
        float speed = BenchMaxBodySpeed(world);
        printf("M2_BENCH %-10s steps=%d totalMs=%9.2f perStepMs=%7.4f hash=%016llx\n", scene->name,
               steps, totalMs, totalMs / (double)steps, (unsigned long long)hash);
        if (!(speed <= scene->speedLimit))
        {
            printf("M2_BENCH %s FAILED: a body moves at %.2f m/s\n", scene->name, (double)speed);
            failures += 1;
        }
        m2DestroyWorld(world);
    }
    return failures == 0 ? 0 : 1;
}
