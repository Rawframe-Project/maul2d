// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for broadphase.c.

#ifndef MAUL2D_SRC_BROADPHASE_H
#define MAUL2D_SRC_BROADPHASE_H

#include "world_internal.h"

// What the pair rules read of one shape.
typedef struct m2PairSide
{
    uint64_t category;
    uint64_t mask;
    int32_t group;
    int32_t type; // m2ShapeType
    bool sensor;
    bool dynamic; // on a dynamic body
} m2PairSide;

m2PairSide m2ShapePairSide(const m2World* world, int32_t shapeIndex);

// Whether the broadphase pairs two shapes of different bodies by their
// kinds and filters. The joints between their bodies have the last word
// (m2JointsForbidPair).
bool m2SidesMayPair(const m2PairSide* a, const m2PairSide* b);

void m2RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB);
void m2UpdatePairs(m2World* world);
void m2PrunePairsOfShape(m2World* world, int32_t shapeIndex);

#endif // MAUL2D_SRC_BROADPHASE_H
