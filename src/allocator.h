// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's allocator: the host's from the world def, or the C
// library's when the def leaves it zeroed. Every block is aligned for any
// type and comes back zeroed.

#ifndef MAUL2D_SRC_ALLOCATOR_H
#define MAUL2D_SRC_ALLOCATOR_H

#include "maul2d/base.h"

#include <stddef.h>

// The alignment of every block the library asks for.
#define M2_ALLOC_ALIGNMENT ((size_t)_Alignof(max_align_t))

// The allocator a world keeps: the def's, with the C library standing in
// for a zeroed one.
m2Allocator m2ResolveAllocator(const m2Allocator* requested);

// A zeroed block of bytes, or NULL when the allocator refuses.
void* m2AllocZeroed(const m2Allocator* allocator, size_t bytes);

// Returns a block to the allocator that gave it, with the size it was
// asked for. NULL is a no-op.
void m2Free(const m2Allocator* allocator, void* memory, size_t bytes);

#endif // MAUL2D_SRC_ALLOCATOR_H
