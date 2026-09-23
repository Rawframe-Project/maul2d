// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ray casts against one shape in its body frame: circle, segment,
// capsule and polygon kernels.
//
// Adapted from Box2D v3.1.1 geometry.c (MIT, Copyright Erin Catto; see
// THIRD_PARTY.md), reworked for Maul's frames.

#include "query.h"

#include <math.h>

static float LengthAndNormalize(m2Vec2* out, m2Vec2 v)
{
    float length = sqrtf(v.x * v.x + v.y * v.y);
    if (length < 1.19209290e-7f)
    {
        *out = (m2Vec2){0.0f, 0.0f};
        return 0.0f;
    }
    out->x = v.x / length;
    out->y = v.y / length;
    return length;
}

// Circle kernel (reference structure).
static m2CastHit RayCastCircle(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 center, float radius)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 s = {p1.x - center.x, p1.y - center.y};
    float rr = radius * radius;

    m2Vec2 unit;
    float length = LengthAndNormalize(&unit, d);
    if (length == 0.0f)
    {
        if (s.x * s.x + s.y * s.y < rr)
        {
            output.point = p1;
            output.hit = true;
        }
        return output;
    }

    float t = -(s.x * unit.x + s.y * unit.y);
    m2Vec2 c = {s.x + t * unit.x, s.y + t * unit.y};
    float cc = c.x * c.x + c.y * c.y;
    if (cc > rr)
    {
        return output;
    }

    float h = sqrtf(rr - cc);
    float fraction = t - h;
    if (fraction < 0.0f || maxFraction * length < fraction)
    {
        if (s.x * s.x + s.y * s.y < rr)
        {
            output.point = p1;
            output.hit = true;
        }
        return output;
    }

    m2Vec2 hitPoint = {s.x + fraction * unit.x, s.y + fraction * unit.y};
    float invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;
    output.fraction = fraction / length;
    output.normal = (m2Vec2){hitPoint.x * invRadius, hitPoint.y * invRadius};
    output.point =
        (m2Vec2){center.x + radius * output.normal.x, center.y + radius * output.normal.y};
    output.hit = true;
    return output;
}

// Two-sided segment kernel (reference structure).
static m2CastHit RayCastSegment(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 v1, m2Vec2 v2)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 e = {v2.x - v1.x, v2.y - v1.y};
    m2Vec2 eUnit;
    float length = LengthAndNormalize(&eUnit, e);
    if (length == 0.0f)
    {
        return output;
    }

    m2Vec2 normal = {eUnit.y, -eUnit.x}; // right perp
    float numerator = normal.x * (v1.x - p1.x) + normal.y * (v1.y - p1.y);
    float denominator = normal.x * d.x + normal.y * d.y;
    if (denominator == 0.0f)
    {
        return output;
    }

    float t = numerator / denominator;
    if (t < 0.0f || maxFraction < t)
    {
        return output;
    }

    m2Vec2 p = {p1.x + t * d.x, p1.y + t * d.y};
    float s = (p.x - v1.x) * eUnit.x + (p.y - v1.y) * eUnit.y;
    if (s < 0.0f || length < s)
    {
        return output;
    }
    if (numerator > 0.0f)
    {
        normal = (m2Vec2){-normal.x, -normal.y};
    }
    output.fraction = t;
    output.point = p;
    output.normal = normal;
    output.hit = true;
    return output;
}

// Capsule kernel (reference structure).
static m2CastHit RayCastCapsule(m2Vec2 p1, m2Vec2 d, float maxFraction, m2Vec2 v1, m2Vec2 v2,
                                float radius)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    m2Vec2 a;
    float capsuleLength = LengthAndNormalize(&a, (m2Vec2){v2.x - v1.x, v2.y - v1.y});
    if (capsuleLength == 0.0f)
    {
        return RayCastCircle(p1, d, maxFraction, v1, radius);
    }

    m2Vec2 q = {p1.x - v1.x, p1.y - v1.y};
    float qa = q.x * a.x + q.y * a.y;
    m2Vec2 qp = {q.x - qa * a.x, q.y - qa * a.y};

    if (qp.x * qp.x + qp.y * qp.y < radius * radius)
    {
        if (qa < 0.0f)
        {
            return RayCastCircle(p1, d, maxFraction, v1, radius);
        }
        if (qa > capsuleLength)
        {
            return RayCastCircle(p1, d, maxFraction, v2, radius);
        }
        output.point = p1;
        output.hit = true;
        return output;
    }

    m2Vec2 n = {a.y, -a.x};
    m2Vec2 u;
    float rayLength = LengthAndNormalize(&u, d);
    if (rayLength == 0.0f)
    {
        return output;
    }

    float den = -a.x * u.y + u.x * a.y;
    if (den > -1.19209290e-7f && den < 1.19209290e-7f)
    {
        return output; // parallel and outside
    }

    m2Vec2 b1 = {q.x - radius * n.x, q.y - radius * n.y};
    m2Vec2 b2 = {q.x + radius * n.x, q.y + radius * n.y};
    float invDen = 1.0f / den;
    float s21 = (a.x * b1.y - b1.x * a.y) * invDen;
    float s22 = (a.x * b2.y - b2.x * a.y) * invDen;

    float s2;
    m2Vec2 b;
    if (s21 < s22)
    {
        s2 = s21;
        b = b1;
    }
    else
    {
        s2 = s22;
        b = b2;
        n = (m2Vec2){-n.x, -n.y};
    }

    if (s2 < 0.0f || maxFraction * rayLength < s2)
    {
        return output;
    }

    float s1 = (-b.x * u.y + u.x * b.y) * invDen;
    if (s1 < 0.0f)
    {
        return RayCastCircle(p1, d, maxFraction, v1, radius);
    }
    if (capsuleLength < s1)
    {
        return RayCastCircle(p1, d, maxFraction, v2, radius);
    }

    float lerp = s1 / capsuleLength;
    output.fraction = s2 / rayLength;
    output.point = (m2Vec2){v1.x + lerp * (v2.x - v1.x) + radius * n.x,
                            v1.y + lerp * (v2.y - v1.y) + radius * n.y};
    output.normal = n;
    output.hit = true;
    return output;
}

// Sharp polygon kernel (reference structure, radius == 0 path).
static m2CastHit RayCastSharpPolygon(m2Vec2 p1In, m2Vec2 d, float maxFraction,
                                     const m2Polygon* shape)
{
    m2CastHit output = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    // Shift the math to the first vertex (the polygon may sit far from
    // the body origin).
    m2Vec2 base = shape->vertices[0];
    m2Vec2 p1 = {p1In.x - base.x, p1In.y - base.y};

    float lower = 0.0f;
    float upper = maxFraction;
    int32_t index = -1;

    for (int32_t i = 0; i < shape->count; ++i)
    {
        m2Vec2 vertex = {shape->vertices[i].x - base.x, shape->vertices[i].y - base.y};
        float numerator =
            shape->normals[i].x * (vertex.x - p1.x) + shape->normals[i].y * (vertex.y - p1.y);
        float denominator = shape->normals[i].x * d.x + shape->normals[i].y * d.y;

        if (denominator == 0.0f)
        {
            if (numerator < 0.0f)
            {
                return output;
            }
        }
        else
        {
            if (denominator < 0.0f && numerator < lower * denominator)
            {
                lower = numerator / denominator;
                index = i;
            }
            else if (denominator > 0.0f && numerator < upper * denominator)
            {
                upper = numerator / denominator;
            }
        }

        if (upper < lower)
        {
            return output;
        }
    }

    if (index >= 0)
    {
        output.fraction = lower;
        output.normal = shape->normals[index];
        output.point = (m2Vec2){p1In.x + lower * d.x, p1In.y + lower * d.y};
        output.hit = true;
    }
    else
    {
        output.point = p1In;
        output.hit = true;
    }
    return output;
}

static void TakeBetter(m2CastHit* best, m2CastHit candidate)
{
    if (candidate.hit && (!best->hit || candidate.fraction < best->fraction))
    {
        *best = candidate;
    }
}

// Rounded polygons cast as the union of offset edges and vertex
// circles: exact, and built from kernels that are already exact.
static m2CastHit RayCastPolygon(m2Vec2 p1, m2Vec2 d, float maxFraction, const m2Polygon* shape)
{
    if (shape->radius == 0.0f)
    {
        return RayCastSharpPolygon(p1, d, maxFraction, shape);
    }
    m2CastHit best = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
    for (int32_t i = 0; i < shape->count; ++i)
    {
        int32_t j = i + 1 < shape->count ? i + 1 : 0;
        m2Vec2 offset = {shape->normals[i].x * shape->radius, shape->normals[i].y * shape->radius};
        m2Vec2 e1 = {shape->vertices[i].x + offset.x, shape->vertices[i].y + offset.y};
        m2Vec2 e2 = {shape->vertices[j].x + offset.x, shape->vertices[j].y + offset.y};
        TakeBetter(&best, RayCastSegment(p1, d, maxFraction, e1, e2));
        TakeBetter(&best, RayCastCircle(p1, d, maxFraction, shape->vertices[i], shape->radius));
    }
    return best;
}

m2CastHit m2RayCastGeometry(const m2ShapeGeometry* geometry, m2Vec2 p1, m2Vec2 d, float maxFraction)
{
    switch (geometry->type)
    {
    case m2_circleShape:
        return RayCastCircle(p1, d, maxFraction, geometry->circle.center, geometry->circle.radius);
    case m2_capsuleShape:
        return RayCastCapsule(p1, d, maxFraction, geometry->capsule.point1,
                              geometry->capsule.point2, geometry->capsule.radius);
    case m2_segmentShape:
        return RayCastSegment(p1, d, maxFraction, geometry->segment.point1,
                              geometry->segment.point2);
    case m2_chainSegmentShape:
    {
        // One-sided, like the collision: rays from the ghost side miss.
        const m2Segment* seg = &geometry->chainSegment.segment;
        m2Vec2 e = {seg->point2.x - seg->point1.x, seg->point2.y - seg->point1.y};
        float offset = (p1.x - seg->point1.x) * e.y - (p1.y - seg->point1.y) * e.x;
        if (offset < 0.0f) // reference sign: skip rays from the ghost side
        {
            m2CastHit missHit = {{0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f, false};
            return missHit;
        }
        return RayCastSegment(p1, d, maxFraction, seg->point1, seg->point2);
    }
    default:
        return RayCastPolygon(p1, d, maxFraction, &geometry->polygon);
    }
}
