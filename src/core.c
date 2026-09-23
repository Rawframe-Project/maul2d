// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "world_internal.h"

#include "maul2d/base.h"

#if defined(_MSC_VER)
#include <intrin.h> // interlocked counters
#endif

#include <stdio.h>
#include <stdlib.h>

static void* DefaultAllocZeroed(size_t bytes)
{
    return calloc(1, bytes);
}

static void DefaultFree(void* memory)
{
    free(memory);
}

static m2AllocZeroedFn* s_alloc = DefaultAllocZeroed;
static m2FreeFn* s_free = DefaultFree;

void m2SetAllocator(m2AllocZeroedFn* allocZeroed, m2FreeFn* freeFn)
{
    s_alloc = allocZeroed != NULL ? allocZeroed : DefaultAllocZeroed;
    s_free = freeFn != NULL ? freeFn : DefaultFree;
}

// Internal faces (world_internal.h).
void* m2AllocZeroed(size_t bytes)
{
    return s_alloc(bytes);
}

void m2Free(void* memory)
{
    s_free(memory);
}

int32_t m2GetVersion(void)
{
    return M2_VERSION_MAJOR * 10000 + M2_VERSION_MINOR * 100 + M2_VERSION_PATCH;
}

#if defined(_MSC_VER)
#define M2_THREAD_LOCAL __declspec(thread)
#else
#define M2_THREAD_LOCAL _Thread_local
#endif

// One slot per thread: a refusal on one thread never overwrites the
// reason another thread is about to read.
static M2_THREAD_LOCAL m2Result s_lastResult = m2_success;

m2Result m2LastResult(void)
{
    return s_lastResult;
}

void m2Refuse(m2World* world, m2Result reason)
{
    s_lastResult = reason;
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

static m2AssertFn* s_assertHandler = NULL;
static void* s_assertContext = NULL;

void m2SetAssertHandler(m2AssertFn* handler, void* context)
{
    s_assertHandler = handler;
    s_assertContext = context;
}

int m2ReportToHost(const char* message, const char* where)
{
    return s_assertHandler != NULL && s_assertHandler(message, where, 0, s_assertContext) != 0;
}

uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount)
{
    // FNV-1a, 64-bit. The constants are frozen: this hash feeds the
    // determinism gates and its output is compared across platforms.
    uint64_t hash = seed;
    const uint8_t* bytes = (const uint8_t*)data;
    for (int32_t i = 0; i < byteCount; ++i)
    {
        hash = (hash ^ bytes[i]) * 1099511628211ULL;
    }
    return hash;
}

void m2AssertFail(const char* condition, const char* file, int line)
{
    if (s_assertHandler != NULL && s_assertHandler(condition, file, line, s_assertContext) != 0)
    {
        return; // handled by the host (A2)
    }
    fprintf(stderr, "maul2d assertion failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
