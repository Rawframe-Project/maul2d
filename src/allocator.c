// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The one file that reaches the C library's malloc and free: every other
// allocation goes through a world's allocator.

#include "allocator.h"

#include "fpenv.h"

#include <stdlib.h>
#include <string.h>

// malloc aligns for any type, which is all the library asks for.
static void* LibraryAlloc(size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    return malloc(size);
}

static void LibraryFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    free(memory);
}

m2Allocator m2ResolveAllocator(const m2Allocator* requested)
{
    if (requested->alloc == NULL || requested->free == NULL)
    {
        return (m2Allocator){LibraryAlloc, LibraryFree, NULL};
    }
    return *requested;
}

// The allocator is host code: it runs in the host's floating-point
// environment, through a helper whose body is only that call.
static M2_NOINLINE void* HostAlloc(const m2Allocator* allocator, size_t bytes)
{
    m2FloatHost host = m2FloatHostEnter();
    void* memory = allocator->alloc(bytes, M2_ALLOC_ALIGNMENT, allocator->context);
    m2FloatHostLeave(host);
    return memory;
}

static M2_NOINLINE void HostFree(const m2Allocator* allocator, void* memory, size_t bytes)
{
    m2FloatHost host = m2FloatHostEnter();
    allocator->free(memory, bytes, M2_ALLOC_ALIGNMENT, allocator->context);
    m2FloatHostLeave(host);
}

void* m2AllocZeroed(const m2Allocator* allocator, size_t bytes)
{
    void* memory = HostAlloc(allocator, bytes);
    if (memory != NULL)
    {
        memset(memory, 0, bytes);
    }
    return memory;
}

void m2Free(const m2Allocator* allocator, void* memory, size_t bytes)
{
    if (memory != NULL)
    {
        HostFree(allocator, memory, bytes);
    }
}
