// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for broadphase.c.

#ifndef MAUL2D_SRC_BROADPHASE_H
#define MAUL2D_SRC_BROADPHASE_H

#include "world_internal.h"

void m2RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB);
void m2UpdatePairs(m2World* world);
void m2PrunePairsOfShape(m2World* world, int32_t shapeIndex);

#endif // MAUL2D_SRC_BROADPHASE_H
