// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The parts of the world hash, and the id pools' share of it. The state
// table (world_state.c) folds the arrays; this folds what decides where
// the next object lands: each pool's cursors, free queue and generations.

#ifndef MAUL2D_SRC_WORLD_HASH_H
#define MAUL2D_SRC_WORLD_HASH_H

#include "world_internal.h"

// The parts of the world hash, each folded from its own seed.
typedef enum m2HashPart
{
    m2_hashWorld,     // the world's scalars
    m2_hashBodies,    // bodies, their shapes and chains, and their pools
    m2_hashContacts,  // pairs and manifolds
    m2_hashJoints,    // joints and their pool
    m2_hashParticles, // particles, springs and triads, and their pool
    m2_hashVolumes,   // fluid volumes and their pool
    m2_hashAll,       // every part, folded in table order
} m2HashPart;

// Folds each live shape's fat box in the broadphase into h (bodies part),
// keyed by the shape slot: pairs are found by querying the moved shapes'
// fat boxes, which carry their history, so two worlds whose boxes differ
// can find different pairs later. The tree's shape around them only
// speeds the query up and stays out.
uint64_t m2HashFatBoxes(const m2World* world, m2HashPart part, uint64_t h);

// Folds each pool of the part (every pool for m2_hashAll) into h: its
// high-water mark, free and retired counts, free queue in order from the
// head, and generations below the high-water mark.
uint64_t m2HashPools(const m2World* world, m2HashPart part, uint64_t h);

#endif // MAUL2D_SRC_WORLD_HASH_H
