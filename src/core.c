// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "world_internal.h"

#include "maul2d/base.h"

#if defined(_MSC_VER)
#include <intrin.h> // interlocked counters
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The host hooks: set before the first world, constant while worlds
// live. Hooked memory may arrive uninitialized, so the zeroing happens
// here either way.
static m2AllocFn* s_hookAlloc = NULL;
static m2FreeFn* s_hookFree = NULL;
static void* s_hookContext = NULL;

void m2SetAllocator(m2AllocFn* allocFn, m2FreeFn* freeFn, void* context)
{
    // Both or neither: a mismatched pair would free with the wrong
    // authority.
    if ((allocFn == NULL) != (freeFn == NULL))
    {
        m2Refuse(NULL, m2_errorInvalid);
        return;
    }
    s_hookAlloc = allocFn;
    s_hookFree = freeFn;
    s_hookContext = context;
}

// Internal faces (world_internal.h).
void* m2AllocZeroed(size_t bytes)
{
    if (s_hookAlloc == NULL)
    {
        return calloc(1, bytes);
    }
    void* memory = s_hookAlloc(bytes, s_hookContext);
    if (memory != NULL)
    {
        memset(memory, 0, bytes);
    }
    return memory;
}

void m2Free(void* memory)
{
    if (s_hookFree == NULL)
    {
        free(memory);
    }
    else
    {
        s_hookFree(memory, s_hookContext);
    }
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

void m2AssertFail(const char* condition, const char* file, int line)
{
    if (s_assertHandler != NULL && s_assertHandler(condition, file, line, s_assertContext) != 0)
    {
        return; // the host handled it
    }
    fprintf(stderr, "maul2d assertion failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
