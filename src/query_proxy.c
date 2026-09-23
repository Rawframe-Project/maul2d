// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Distance proxies for shapes and query geometry, and the frame change
// every convex cast and overlap shares.

#include "query.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

m2DistanceProxy m2GeometryProxy(const m2ShapeGeometry* g)
{
    m2DistanceProxy p;
    p.count = 1;
    p.radius = 0.0f;
    p.points[0] = (m2Vec2){0.0f, 0.0f};
    switch (g->type)
    {
    case m2_circleShape:
        p.points[0] = g->circle.center;
        p.radius = g->circle.radius;
        break;
    case m2_capsuleShape:
        p.points[0] = g->capsule.point1;
        p.points[1] = g->capsule.point2;
        p.count = 2;
        p.radius = g->capsule.radius;
        break;
    case m2_polygonShape:
        for (int32_t i = 0; i < g->polygon.count; ++i)
        {
            p.points[i] = g->polygon.vertices[i];
        }
        p.count = g->polygon.count;
        p.radius = g->polygon.radius;
        break;
    case m2_segmentShape:
        p.points[0] = g->segment.point1;
        p.points[1] = g->segment.point2;
        p.count = 2;
        break;
    default: // chain segment
        p.points[0] = g->chainSegment.segment.point1;
        p.points[1] = g->chainSegment.segment.point2;
        p.count = 2;
        break;
    }
    return p;
}

// Caller shapes become distance proxies here, once, with the checks
// every cast and overlap shares: a real pointer, a vertex count the
// proxy can hold, finite coordinates, a finite non-negative radius. A
// refused shape comes back with count 0, which the query entry points
// turn into a refusal.
static m2DistanceProxy CheckedProxy(const m2Vec2* points, int32_t count, float radius)
{
    m2DistanceProxy p;
    memset(&p, 0, sizeof(p));
    if (points == NULL || count < 1 || count > M2_MAX_POLYGON_VERTICES || !m2FiniteF(radius) ||
        radius < 0.0f)
    {
        return p;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        if (!m2FiniteVec2(points[i]))
        {
            return p;
        }
        p.points[i] = points[i];
    }
    p.count = count;
    p.radius = radius;
    return p;
}

m2DistanceProxy m2CircleProxy(const m2Circle* circle)
{
    m2DistanceProxy none;
    memset(&none, 0, sizeof(none));
    return circle != NULL ? CheckedProxy(&circle->center, 1, circle->radius) : none;
}

m2DistanceProxy m2CapsuleProxy(const m2Capsule* capsule)
{
    m2DistanceProxy none;
    memset(&none, 0, sizeof(none));
    if (capsule == NULL)
    {
        return none;
    }
    m2Vec2 points[2] = {capsule->point1, capsule->point2};
    return CheckedProxy(points, 2, capsule->radius);
}

m2DistanceProxy m2PolygonProxy(const m2Polygon* polygon)
{
    m2DistanceProxy none;
    memset(&none, 0, sizeof(none));
    return polygon != NULL ? CheckedProxy(polygon->vertices, polygon->count, polygon->radius)
                           : none;
}

// A query pose and sweep: finite, with a unit rotation.
bool m2QueryMotionValid(m2Transform pose, m2Vec2 translation)
{
    return m2FinitePos2(pose.p) && m2UnitRot(pose.q) && m2FiniteVec2(translation);
}

m2ProxyQuery m2MakeProxyQuery(const m2DistanceProxy* castLocal, m2Transform pose,
                              m2Vec2 translation)
{
    m2ProxyQuery q;
    q.castLocal = *castLocal;
    q.pose = pose;
    q.translation = translation;
    float ext = 0.0f;
    for (int32_t i = 0; i < castLocal->count; ++i)
    {
        float d2 = castLocal->points[i].x * castLocal->points[i].x +
                   castLocal->points[i].y * castLocal->points[i].y;
        float d = sqrtf(d2);
        ext = d > ext ? d : ext;
    }
    q.boundRadius = ext + castLocal->radius;
    return q;
}

// Both proxies in the target's body frame; also reports the pose
// origin there (the chain one-sided reference point).
void m2ProxiesInBodyFrame(const m2World* world, int32_t shapeIndex, const m2ProxyQuery* q,
                          m2DistanceProxy* target, m2DistanceProxy* cast, m2Vec2* translationLocal,
                          m2Vec2* poseOriginLocal)
{
    int32_t body = world->shapes.shapeBody[shapeIndex];
    m2Transform xf = world->bodies.transforms[body];
    *target = m2GeometryProxy(&world->shapes.shapeGeometry[shapeIndex]);

    m2Vec2 rel = {(float)(q->pose.p.x - xf.p.x), (float)(q->pose.p.y - xf.p.y)};
    m2Vec2 relLocal = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    *poseOriginLocal = relLocal;

    // Combined rotation: cast local -> world (pose.q), world -> body
    // local (inverse xf.q).
    float rc = xf.q.c * q->pose.q.c + xf.q.s * q->pose.q.s;
    float rs = xf.q.c * q->pose.q.s - xf.q.s * q->pose.q.c;
    cast->count = q->castLocal.count;
    cast->radius = q->castLocal.radius;
    for (int32_t i = 0; i < q->castLocal.count; ++i)
    {
        m2Vec2 pt = q->castLocal.points[i];
        cast->points[i] =
            (m2Vec2){rc * pt.x - rs * pt.y + relLocal.x, rs * pt.x + rc * pt.y + relLocal.y};
    }
    translationLocal->x = xf.q.c * q->translation.x + xf.q.s * q->translation.y;
    translationLocal->y = -xf.q.s * q->translation.x + xf.q.c * q->translation.y;
}

// Sweeps from the ghost side pass through, same sign law as rays.
bool m2ChainGhostSide(const m2World* world, int32_t shapeIndex, m2Vec2 startLocal)
{
    const m2ShapeGeometry* g = &world->shapes.shapeGeometry[shapeIndex];
    if (g->type != m2_chainSegmentShape)
    {
        return false;
    }
    const m2Segment* seg = &g->chainSegment.segment;
    m2Vec2 e = {seg->point2.x - seg->point1.x, seg->point2.y - seg->point1.y};
    float offset = (startLocal.x - seg->point1.x) * e.y - (startLocal.y - seg->point1.y) * e.x;
    return offset < 0.0f;
}
