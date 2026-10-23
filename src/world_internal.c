// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals against a world: the world's misuse counter, the host code
// bracket and the bracketed allocator, and the check every mutating call
// makes first.

#include "world_internal.h"

#include "allocator.h"

#if defined(_MSC_VER)
#include <intrin.h> // interlocked counters
#endif

m2Result m2Refuse(const m2World* world, m2Result reason)
{
    bool misuse = world != NULL && reason != m2_errorCapacity &&
                  (world->replaying == 0 || reason == m2_errorState);
    if (misuse)
    {
        // Reader calls refuse too, possibly on several threads at once, so
        // the counter is bumped atomically. It is the one field a reader
        // writes; the world itself is never a const object.
        m2World* counted = (m2World*)world;
#if defined(_MSC_VER)
        _InterlockedIncrement64(&counted->misuseCount);
#else
        __atomic_fetch_add(&counted->misuseCount, 1, __ATOMIC_RELAXED);
#endif
    }
    return reason;
}

uint64_t m2MisuseCount(const m2World* world)
{
#if defined(_MSC_VER)
    return (uint64_t)_InterlockedCompareExchange64((volatile long long*)&world->misuseCount, 0, 0);
#else
    return (uint64_t)__atomic_load_n(&world->misuseCount, __ATOMIC_RELAXED);
#endif
}

void m2EnterHostCode(const m2World* world)
{
    m2World* counted = (m2World*)world;
#if defined(_MSC_VER)
    _InterlockedIncrement(&counted->busy);
#else
    __atomic_fetch_add(&counted->busy, 1, __ATOMIC_RELAXED);
#endif
}

void m2LeaveHostCode(const m2World* world)
{
    m2World* counted = (m2World*)world;
#if defined(_MSC_VER)
    _InterlockedDecrement(&counted->busy);
#else
    __atomic_fetch_sub(&counted->busy, 1, __ATOMIC_RELAXED);
#endif
}

void* m2WorldAlloc(const m2World* world, size_t bytes)
{
    m2EnterHostCode(world);
    void* memory = m2AllocZeroed(&world->allocator, bytes);
    m2LeaveHostCode(world);
    return memory;
}

void m2WorldFree(const m2World* world, void* memory, size_t bytes)
{
    m2EnterHostCode(world);
    m2Free(&world->allocator, memory, bytes);
    m2LeaveHostCode(world);
}

m2Result m2CheckWriter(const m2World* world)
{
    if (world == NULL)
    {
        return m2_errorInvalid;
    }
#if defined(_MSC_VER)
    long busy = _InterlockedCompareExchange((volatile long*)&world->busy, 0, 0);
#else
    long busy = __atomic_load_n(&world->busy, __ATOMIC_RELAXED);
#endif
    if (busy != 0)
    {
        return m2Refuse(world, m2_errorState);
    }
    return m2_success;
}
