// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex distance and casts: one GJK kernel for every convex proxy.

#ifndef MAUL2D_SRC_DISTANCE_H
#define MAUL2D_SRC_DISTANCE_H

#include "world_internal.h"

// One GJK kernel for every convex proxy; callers pre-transform both proxies
// into a single body-local float frame.
typedef struct m2DistanceProxy
{
    m2Vec2 points[8];
    int32_t count;
    float radius;
} m2DistanceProxy;

typedef struct m2DistanceResult
{
    m2Vec2 pointA;
    m2Vec2 pointB;
    m2Vec2 normal; // A toward B; (0,0) when cores overlap
    float distance;
} m2DistanceResult;

typedef struct m2CastResult
{
    m2Vec2 pointA; // surface point on A's inflated boundary
    m2Vec2 normal; // (0,0) on initial overlap
    float fraction;
    bool hit;
} m2CastResult;

m2DistanceProxy m2GeometryProxy(const struct m2ShapeGeometry* g);
m2DistanceResult m2ShapeDistance(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB);
m2CastResult m2ShapeCastProxy(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB,
                              m2Vec2 translation, float maxFraction);

#endif // MAUL2D_SRC_DISTANCE_H
