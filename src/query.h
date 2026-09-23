// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal query kernels shared with other modules.

#ifndef MAUL2D_SRC_QUERY_H
#define MAUL2D_SRC_QUERY_H

#include "distance.h"
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

// The layer filter every query applies to a candidate shape.
static inline bool m2QueryShouldSee(const m2World* world, int32_t shapeIndex, m2QueryFilter filter)
{
    return (filter.categoryBits & world->shapes.shapeMask[shapeIndex]) != 0 &&
           (world->shapes.shapeCategory[shapeIndex] & filter.maskBits) != 0;
}

// A ray hit in one shape's frame (ray_cast.c).
typedef struct m2CastHit
{
    m2Vec2 point; // body-local
    m2Vec2 normal;
    float fraction; // of the local ray length parameter (0..maxFraction)
    bool hit;
} m2CastHit;
m2CastHit m2RayCastGeometry(const m2ShapeGeometry* geometry, m2Vec2 p1, m2Vec2 d,
                            float maxFraction);

// Convex casts and overlaps (query_proxy.c): the cast geometry becomes a
// distance proxy in its own frame, and per candidate both proxies meet in
// the target's body frame.
typedef struct m2ProxyQuery
{
    m2DistanceProxy castLocal; // cast geometry in its own frame
    m2Transform pose;          // world pose of that frame (f64 p)
    m2Vec2 translation;        // world sweep, zero for overlaps
    float boundRadius;         // bounding circle of the cast geometry
} m2ProxyQuery;
m2DistanceProxy m2CircleProxy(const m2Circle* circle);
m2DistanceProxy m2CapsuleProxy(const m2Capsule* capsule);
m2DistanceProxy m2PolygonProxy(const m2Polygon* polygon);
bool m2QueryMotionValid(m2Transform pose, m2Vec2 translation);
m2ProxyQuery m2MakeProxyQuery(const m2DistanceProxy* castLocal, m2Transform pose,
                              m2Vec2 translation);
void m2ProxiesInBodyFrame(const m2World* world, int32_t shapeIndex, const m2ProxyQuery* q,
                          m2DistanceProxy* target, m2DistanceProxy* cast, m2Vec2* translationLocal,
                          m2Vec2* poseOriginLocal);
bool m2ChainGhostSide(const m2World* world, int32_t shapeIndex, m2Vec2 startLocal);

#endif // MAUL2D_SRC_QUERY_H
