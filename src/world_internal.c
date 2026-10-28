// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Refusals against a world: the world's misuse counter, the host code
// bracket and the bracketed allocator, the check every mutating call
// makes first, and the check of a state format's prefix.

#include "world_internal.h"

#include "allocator.h"

#include <string.h>

#if defined(_MSC_VER)
#include <intrin.h> // interlocked counters
#endif

// How deep the world is in host code it runs; readers may ask from
// several threads at once.
static long HostCodeDepth(const m2World* world)
{
#if defined(_MSC_VER)
    return _InterlockedCompareExchange((volatile long*)&world->busy, 0, 0);
#else
    return __atomic_load_n(&world->busy, __ATOMIC_RELAXED);
#endif
}

m2Result m2Refuse(const m2World* world, m2Result reason)
{
    // A refusal on a replay's own path is the tape's data at fault, not the
    // caller's code; host code the replay runs (a task hook, the allocator)
    // is the caller's and still counts.
    bool tapeDriven = world != NULL && world->replaying != 0 && HostCodeDepth(world) == 0;
    // A full pool and bytes from outside are no misuse of the API.
    bool misuse =
        world != NULL && reason != m2_errorCapacity && reason != m2_errorFormat && !tapeDriven;
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
    if (HostCodeDepth(world) != 0)
    {
        return m2Refuse(world, m2_errorBusy);
    }
    return m2_success;
}

m2StatePrefix m2MakeStatePrefix(uint32_t magic, uint32_t formatVersion, int32_t headerBytes)
{
    m2StatePrefix prefix;
    prefix.magic = magic;
    prefix.abiVersion = M2_ABI_VERSION;
    prefix.formatVersion = formatVersion;
    prefix.headerBytes = (uint32_t)headerBytes;
    return prefix;
}

m2Result m2CheckStatePrefix(const void* bytes, int64_t size, uint32_t magic, uint32_t formatVersion,
                            int32_t headerBytes)
{
    if (size < (int64_t)sizeof(m2StatePrefix))
    {
        return m2_errorFormat;
    }
    m2StatePrefix prefix;
    memcpy(&prefix, bytes, sizeof(prefix));
    if (prefix.magic != magic)
    {
        return m2_errorFormat; // not this format
    }
    if (prefix.abiVersion != M2_ABI_VERSION || prefix.formatVersion != formatVersion)
    {
        return m2_errorVersion; // written by another release
    }
    if (prefix.headerBytes != (uint32_t)headerBytes || size < headerBytes)
    {
        return m2_errorFormat;
    }
    return m2_success;
}
