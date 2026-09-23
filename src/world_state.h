// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table: every array the world owns, with its element
// size, its length and whether it is snapshot state. Allocation,
// release, the snapshot walk and the memory footprint all come from the
// one table in world_state.c, so a new array is one struct field and one
// table row.

#ifndef MAUL2D_SRC_WORLD_STATE_H
#define MAUL2D_SRC_WORLD_STATE_H

#include "world_internal.h"

// Allocates every array, sized from the capacities already set in the
// world, and adds them to its memory footprint. Returns false when an
// allocation is refused; m2StateFree releases whatever was allocated.
bool m2StateAllocate(m2World* world);
void m2StateFree(m2World* world);

// Walks the snapshot state in byte order: direction 0 copies into out,
// 1 copies from in, and any other value only measures. Returns the
// number of bytes.
int32_t m2StateWalk(m2World* world, uint8_t* out, const uint8_t* in, int direction);

#endif // MAUL2D_SRC_WORLD_STATE_H
