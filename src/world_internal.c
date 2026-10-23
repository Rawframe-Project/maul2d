// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals against a world: the reason for m2LastResult and the world's
// misuse counter.

#include "world_internal.h"

#if defined(_MSC_VER)
#include <intrin.h> // interlocked counters
#endif

void m2Refuse(m2World* world, m2Result reason)
{
    m2SetLastResult(reason);
    if (world != NULL && reason == m2_errorInvalid)
    {
        // Reader-class calls refuse too, possibly on several threads at
        // once, so the counter is bumped atomically.
#if defined(_MSC_VER)
        _InterlockedIncrement64(&world->misuseCount);
#else
        __atomic_fetch_add(&world->misuseCount, 1, __ATOMIC_RELAXED);
#endif
    }
}

uint64_t m2MisuseCount(const m2World* world)
{
#if defined(_MSC_VER)
    return (uint64_t)_InterlockedCompareExchange64((volatile long long*)&world->misuseCount, 0, 0);
#else
    return (uint64_t)__atomic_load_n(&world->misuseCount, __ATOMIC_RELAXED);
#endif
}
