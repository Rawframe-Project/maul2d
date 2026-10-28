// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ray casts against one shape in its body frame. The ray is p1 + t d for
// t in [0, maxFraction]; a hit reports the first t, the surface point
// and the outward normal. A ray that starts inside a solid shape hits at
// t = 0 with a zero normal.
//
// Circles solve in the ray's own frame, about the closest approach; segments
// intersect two lines by cross products; sharp polygons clip the ray
// against every face (Cyrus and Beck); rounded polygons and capsules,
// which are rounded two-vertex polygons, take the first hit on their
// offset faces and corner circles.

#include "query.h"

#include <math.h>

static float Dot(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

static float Cross(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

static m2Vec2 Sub(m2Vec2 a, m2Vec2 b)
{
    return (m2Vec2){a.x - b.x, a.y - b.y};
}

static m2Vec2 MulAdd(m2Vec2 a, float s, m2Vec2 b)
{
    return (m2Vec2){a.x + s * b.x, a.y + s * b.y};
}

static const m2LocalHit s_miss = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};

static m2LocalHit StartsInside(m2Vec2 p1)
{
    m2LocalHit hit = {p1, {0.0f, 0.0f}, 0.0f, true};
    return hit;
}

// The circle in the ray's own terms: the ray's unit direction u, the
// distance along it to the point of closest approach, and that point's
// offset q from the center. The hit lies sqrt(r^2 - q.q) before it. The
// quadratic b^2 - a c would subtract two products of the ray's length and
// distance, which cancel from about 1e9 m and overflow from about 1e19;
// here a far or long ray loses no more than the float position of its
// origin does, and the normal comes from q, which is small near a hit.
static m2LocalHit RayCastCircle(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 center, float radius)
{
    m2Vec2 s = Sub(p1, center);
    if (Dot(s, s) < radius * radius)
    {
        return StartsInside(p1);
    }
    // The direction scaled by its largest component first, so no square
    // of the translation overflows.
    float scale = m2MaxF(m2AbsF(d.x), m2AbsF(d.y));
    if (!(scale > 0.0f))
    {
        return s_miss; // no motion
    }
    m2Vec2 e = {d.x / scale, d.y / scale};
    float length = sqrtf(Dot(e, e));
    m2Vec2 u = {e.x / length, e.y / length};
    float along = Dot(s, u);
    if (along >= 0.0f)
    {
        return s_miss; // moving away
    }
    m2Vec2 q = MulAdd(s, -along, u);
    float h2 = radius * radius - Dot(q, q);
    if (h2 < 0.0f)
    {
        return s_miss; // passing by
    }
    float h = sqrtf(h2);
    // Outside the circle the root is never negative, but rounding can take
    // it a few ulps below 0 for an origin on the surface.
    float t = m2MaxF(-along - h, 0.0f) / scale / length;
    if (t > maxFraction)
    {
        return s_miss;
    }
    m2Vec2 offset = MulAdd(q, -h, u);
    float n = sqrtf(Dot(offset, offset));
    m2LocalHit hit = {MulAdd(center, 1.0f, offset), {offset.x / n, offset.y / n}, t, true};
    if (!(n > 0.0f))
    {
        hit.normal = (m2Vec2){0.0f, 0.0f};
    }
    return hit;
}

// Two-sided: the normal faces the ray's origin.
static m2LocalHit RayCastSegment(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 v1, m2Vec2 v2)
{
    m2Vec2 e = Sub(v2, v1);
    float denom = Cross(d, e);
    if (denom == 0.0f)
    {
        return s_miss; // parallel, or a point segment
    }
    m2Vec2 r = Sub(v1, p1);
    float t = Cross(r, e) / denom;
    float u = Cross(r, d) / denom;
    if (t < 0.0f || t > maxFraction || u < 0.0f || u > 1.0f)
    {
        return s_miss;
    }
    float length = sqrtf(Dot(e, e));
    m2Vec2 n = {e.y / length, -e.x / length};
    if (Dot(n, d) > 0.0f)
    {
        n = (m2Vec2){-n.x, -n.y};
    }
    m2LocalHit hit = {MulAdd(p1, t, d), n, t, true};
    return hit;
}

// The polygon clips the ray face by face: entering faces raise the lower
// bound, leaving faces lower the upper bound, and a ray parallel to a
// face outside it misses. Positions are taken relative to the first
// vertex, since the polygon may sit far from the body origin.
static m2LocalHit RayCastSharpPolygon(m2Vec2 p1, m2Vec2 d, float maxFraction, const m2Polygon* poly)
{
    m2Vec2 base = poly->vertices[0];
    m2Vec2 start = Sub(p1, base);
    float lower = 0.0f;
    float upper = maxFraction;
    int32_t entering = -1;
    for (int32_t i = 0; i < poly->count; ++i)
    {
        m2Vec2 n = poly->normals[i];
        float gap = Dot(n, Sub(Sub(poly->vertices[i], base), start)); // > 0 inside
        float rate = Dot(n, d);
        if (rate == 0.0f)
        {
            if (gap < 0.0f)
            {
                return s_miss;
            }
            continue;
        }
        float t = gap / rate;
        if (rate < 0.0f && t > lower)
        {
            lower = t;
            entering = i;
        }
        else if (rate > 0.0f && t < upper)
        {
            upper = t;
        }
        if (upper < lower)
        {
            return s_miss;
        }
    }
    if (entering < 0)
    {
        return StartsInside(p1);
    }
    m2LocalHit hit = {MulAdd(p1, lower, d), poly->normals[entering], lower, true};
    return hit;
}

static void KeepFirst(m2LocalHit* best, m2LocalHit candidate)
{
    if (candidate.hit && (!best->hit || candidate.fraction < best->fraction))
    {
        *best = candidate;
    }
}

// True when p lies within radius of the polygon's core.
static bool InsideRounded(m2Vec2 p, const m2Polygon* poly)
{
    float inside = -3.4e38f;
    float nearest = 3.4e38f;
    for (int32_t i = 0; i < poly->count; ++i)
    {
        int32_t j = i + 1 < poly->count ? i + 1 : 0;
        m2Vec2 a = poly->vertices[i];
        m2Vec2 e = Sub(poly->vertices[j], a);
        inside =
            inside > Dot(poly->normals[i], Sub(p, a)) ? inside : Dot(poly->normals[i], Sub(p, a));
        float ee = Dot(e, e);
        float t = ee > 0.0f ? Dot(Sub(p, a), e) / ee : 0.0f;
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        m2Vec2 r = Sub(p, MulAdd(a, t, e));
        nearest = Dot(r, r) < nearest ? Dot(r, r) : nearest;
    }
    return inside <= 0.0f || nearest < poly->radius * poly->radius;
}

// A rounded polygon is its faces pushed out by the radius joined by
// circles at the corners. From outside, the first hit on an offset face
// the ray enters or on a corner circle is the first hit on the shape.
static m2LocalHit RayCastRoundedPolygon(m2Vec2 p1, m2Vec2 d, float maxFraction,
                                        const m2Polygon* poly)
{
    if (InsideRounded(p1, poly))
    {
        return StartsInside(p1);
    }
    m2LocalHit best = s_miss;
    for (int32_t i = 0; i < poly->count; ++i)
    {
        int32_t j = i + 1 < poly->count ? i + 1 : 0;
        m2Vec2 n = poly->normals[i];
        if (Dot(n, d) < 0.0f)
        {
            m2Vec2 e1 = MulAdd(poly->vertices[i], poly->radius, n);
            m2Vec2 e2 = MulAdd(poly->vertices[j], poly->radius, n);
            m2LocalHit face = RayCastSegment(p1, d, maxFraction, e1, e2);
            face.normal = n;
            KeepFirst(&best, face);
        }
        KeepFirst(&best, RayCastCircle(p1, d, maxFraction, poly->vertices[i], poly->radius));
    }
    return best;
}

static m2LocalHit RayCastPolygon(m2Vec2 p1, m2Vec2 d, float maxFraction, const m2Polygon* poly)
{
    return poly->radius == 0.0f ? RayCastSharpPolygon(p1, d, maxFraction, poly)
                                : RayCastRoundedPolygon(p1, d, maxFraction, poly);
}

m2LocalHit m2RayCastGeometry(const m2ShapeGeometry* geometry, m2Vec2 p1, m2Vec2 d,
                             float maxFraction)
{
    switch (geometry->type)
    {
    case m2_circleShape:
        return RayCastCircle(p1, d, maxFraction, geometry->circle.center, geometry->circle.radius);
    case m2_capsuleShape:
    {
        m2Polygon capsule = m2MakeSegmentProxy(geometry->capsule.point1, geometry->capsule.point2,
                                               geometry->capsule.radius);
        return RayCastRoundedPolygon(p1, d, maxFraction, &capsule);
    }
    case m2_segmentShape:
        return RayCastSegment(p1, d, maxFraction, geometry->segment.point1,
                              geometry->segment.point2);
    case m2_chainSegmentShape:
    {
        // One-sided like the collision: rays from the ghost side, on the
        // left of point1 to point2, pass through.
        const m2Segment* seg = &geometry->chainSegment.segment;
        if (Cross(Sub(seg->point2, seg->point1), Sub(p1, seg->point1)) > 0.0f)
        {
            return s_miss;
        }
        return RayCastSegment(p1, d, maxFraction, seg->point1, seg->point2);
    }
    default:
        return RayCastPolygon(p1, d, maxFraction, &geometry->polygon);
    }
}
