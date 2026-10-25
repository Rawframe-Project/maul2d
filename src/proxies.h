// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for proxies.c: a shape's broadphase box.

#ifndef MAUL2D_SRC_PROXIES_H
#define MAUL2D_SRC_PROXIES_H

#include "world_internal.h"

m2Aabb m2Fatten(m2Aabb aabb);
m2Aabb m2ShapeTightAabb(const m2World* world, int32_t shapeIndex);
int32_t m2ShapeTreeIndex(const m2World* world, int32_t shapeIndex);
// Refits a treed shape's fat box around its tight box at its body's pose
// when the tight box has left it. Returns whether the box moved.
bool m2RefitProxy(m2World* world, int32_t shapeIndex);
// Refits every treed shape of a body and marks the ones that moved.
void m2RefitBodyProxies(m2World* world, int32_t bodyIndex);
void m2PushMoved(m2World* world, int32_t shapeIndex);

#endif // MAUL2D_SRC_PROXIES_H
