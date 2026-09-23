// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal query kernels shared with other modules.

#ifndef MAUL2D_SRC_QUERY_H
#define MAUL2D_SRC_QUERY_H

#include "world_internal.h"

// One shape, the world ray conventions, the one-sided chain law: the
// particle projection pass borrows the per-shape kernel without the
// tree walk.
struct m2CastHitInternal
{
    m2Vec2 point;
    m2Vec2 normal;
    float fraction;
    bool hit;
};
struct m2CastHitInternal m2RayCastShapeIndex(const m2World* world, int32_t shapeIndex,
                                             m2Pos2 origin, m2Vec2 translation, float maxFraction);

#endif // MAUL2D_SRC_QUERY_H
