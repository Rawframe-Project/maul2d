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

#include "world_hash.h"
#include "world_internal.h"

// Allocates every array, sized from the capacities already set in the
// world, and adds them to its memory footprint. Returns false when an
// allocation is refused; m2StateFree releases whatever was allocated.
bool m2StateAllocate(m2World* world);
void m2StateFree(m2World* world);

// The bytes of the snapshot state, from the capacities already set in the
// world alone, so a def is measured before anything is allocated. Returns
// false when the sum does not fit a size_t.
bool m2StateSnapshotBytes(const m2World* world, size_t* bytesOut);

// Walks the snapshot state in byte order: direction 0 copies into out,
// 1 copies from in, and any other value only measures. Returns the
// number of bytes, m2StateSnapshotBytes' count.
size_t m2StateWalk(m2World* world, uint8_t* out, const uint8_t* in, int direction);

// Checks an incoming snapshot block against the table before any of it
// lands: every index in range, every flag a flag, every count within its
// capacity, every number within its range. The block must be exactly the
// walk's size.
bool m2StateValidate(const m2World* world, const uint8_t* in);

// Checks one body slot against the same rules a restore applies, so a
// creation that checked its def can assert it wrote a slot a snapshot of
// the world will restore.
bool m2StateBodyValid(const m2World* world, int32_t index);

// Where an incoming snapshot block, already accepted by m2StateValidate,
// holds the bytes of one state field (a world member such as
// &world->bodies.alive); NULL when the field is not in this world's
// snapshots. The bytes may be unaligned: read them with memcpy.
const uint8_t* m2StateIncoming(const m2World* world, const uint8_t* in, const void* field);

// Folds the state the table marks as hashed into one hash: per-object
// arrays by live slot, keyed by the slot index; counted arrays up to their
// count, keyed by it; and each pool's identity (its cursors, free queue in
// order and generations), which decides where the next object lands.
uint64_t m2StateHash(const m2World* world, m2HashPart part);

#endif // MAUL2D_SRC_WORLD_STATE_H
