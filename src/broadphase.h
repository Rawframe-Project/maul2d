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
    uint8_t bodyType; // m2BodyType of its body
} m2PairSide;

m2PairSide m2ShapePairSide(const m2World* world, int32_t shapeIndex);

// Whether the broadphase pairs two shapes of different bodies by their
// kinds, their bodies' types and their filters. The joints between their bodies have the last word
// (m2JointsForbidPair).
M2_NODISCARD bool m2SidesMayPair(const m2PairSide* a, const m2PairSide* b);

void m2RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB);
void m2UpdatePairs(m2World* world);
// Empties the moved list. The step calls it after the contact update,
// which reads the moved flags to tell which frozen pairs moved.
void m2ClearMoved(m2World* world);
void m2PrunePairsOfShape(m2World* world, int32_t shapeIndex);

#endif // MAUL2D_SRC_BROADPHASE_H
