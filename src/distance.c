// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex distance and translation casts. Every convex Maul shape is a
// point set plus a radius, so one distance kernel and one cast serve
// circles, capsules, polygons, segments and chain segments. Callers put
// both proxies into one body-local float frame first, so the kernels
// carry no transforms.
//
// Distance is Gilbert, Johnson and Keerthi's algorithm over the core
// point sets: a simplex of up to three Minkowski difference points is
// reduced to the feature closest to the origin, then grown by the
// support point opposite that closest point, until the gain falls under
// a relative tolerance (van den Bergen's stopping rule) or a support
// point repeats. The cast is conservative advancement: the moving proxy
// advances by the gap divided by its approach speed until the gap
// closes. Iteration caps are fixed and all arithmetic is plain IEEE.

#include "distance.h"
#include "world_internal.h"

#include <math.h>

typedef struct Vertex
{
    m2Vec2 a;     // support point of A
    m2Vec2 b;     // support point of B
    m2Vec2 w;     // b - a, a point of the Minkowski difference
    float weight; // barycentric weight in the closest point
    int32_t ia;   // support indices, to catch a repeated vertex
    int32_t ib;
} Vertex;

typedef struct Simplex
{
    Vertex v[3];
    int32_t count;
} Simplex;

static float Dot(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

static m2Vec2 Sub(m2Vec2 a, m2Vec2 b)
{
    return (m2Vec2){a.x - b.x, a.y - b.y};
}

static int32_t Support(const m2DistanceProxy* proxy, m2Vec2 d)
{
    int32_t best = 0;
    for (int32_t i = 1; i < proxy->count; ++i)
    {
        best = Dot(proxy->points[i], d) > Dot(proxy->points[best], d) ? i : best;
    }
    return best;
}

static Vertex MakeVertex(const m2DistanceProxy* pa, const m2DistanceProxy* pb, int32_t ia,
                         int32_t ib)
{
    Vertex v;
    v.a = pa->points[ia];
    v.b = pb->points[ib];
    v.w = Sub(v.b, v.a);
    v.weight = 1.0f;
    v.ia = ia;
    v.ib = ib;
    return v;
}

// Reduces the segment p, q to its feature closest to the origin: an end
// or the segment itself with weights. Returns the squared distance.
static float ReduceSegment(Vertex p, Vertex q, Simplex* out)
{
    m2Vec2 e = Sub(q.w, p.w);
    float ee = Dot(e, e);
    float t = ee > 0.0f ? -Dot(p.w, e) / ee : 0.0f;
    if (t <= 0.0f)
    {
        p.weight = 1.0f;
        out->v[0] = p;
        out->count = 1;
        return Dot(p.w, p.w);
    }
    if (t >= 1.0f)
    {
        q.weight = 1.0f;
        out->v[0] = q;
        out->count = 1;
        return Dot(q.w, q.w);
    }
    p.weight = 1.0f - t;
    q.weight = t;
    out->v[0] = p;
    out->v[1] = q;
    out->count = 2;
    m2Vec2 c = {p.w.x + t * e.x, p.w.y + t * e.y};
    return Dot(c, c);
}

// A triangle holding the origin stays whole with its barycentric
// weights; otherwise the closest of its three edges wins, the first on
// ties.
static void ReduceTriangle(Simplex* s)
{
    m2Vec2 a = s->v[0].w;
    m2Vec2 b = s->v[1].w;
    m2Vec2 c = s->v[2].w;
    float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    float wa = b.x * c.y - b.y * c.x;
    float wb = c.x * a.y - c.y * a.x;
    float wc = a.x * b.y - a.y * b.x;
    bool inside = area > 0.0f ? (wa >= 0.0f && wb >= 0.0f && wc >= 0.0f)
                              : (wa <= 0.0f && wb <= 0.0f && wc <= 0.0f);
    if (inside && area != 0.0f)
    {
        s->v[0].weight = wa / area;
        s->v[1].weight = wb / area;
        s->v[2].weight = wc / area;
        return;
    }
    Simplex best;
    float bestDist = ReduceSegment(s->v[0], s->v[1], &best);
    Simplex edge;
    float dist = ReduceSegment(s->v[1], s->v[2], &edge);
    if (dist < bestDist)
    {
        best = edge;
        bestDist = dist;
    }
    dist = ReduceSegment(s->v[2], s->v[0], &edge);
    if (dist < bestDist)
    {
        best = edge;
    }
    *s = best;
}

static m2Vec2 Blend(const Simplex* s, bool pointsOfA)
{
    m2Vec2 p = {0.0f, 0.0f};
    for (int32_t i = 0; i < s->count; ++i)
    {
        m2Vec2 q = pointsOfA ? s->v[i].a : s->v[i].b;
        p.x += s->v[i].weight * q.x;
        p.y += s->v[i].weight * q.y;
    }
    return p;
}

static bool Repeats(const Simplex* s, int32_t ia, int32_t ib)
{
    for (int32_t i = 0; i < s->count; ++i)
    {
        if (s->v[i].ia == ia && s->v[i].ib == ib)
        {
            return true;
        }
    }
    return false;
}

// Core-shape distance (radii not applied) with witness points; the
// callers subtract radii. distance == 0 means the cores overlap and the
// normal is (0,0).
m2DistanceResult m2ShapeDistance(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB)
{
    Simplex s;
    s.v[0] = MakeVertex(proxyA, proxyB, 0, 0);
    s.count = 1;
    for (int32_t iter = 0; iter < 24; ++iter)
    {
        // A support point already in the simplex before this reduction
        // can only cycle.
        Simplex before = s;
        if (s.count == 2)
        {
            ReduceSegment(s.v[0], s.v[1], &s);
        }
        else if (s.count == 3)
        {
            ReduceTriangle(&s);
        }
        if (s.count == 3)
        {
            break; // the origin is enclosed: the cores overlap
        }
        m2Vec2 v = {0.0f, 0.0f}; // the closest point of the difference to the origin
        for (int32_t i = 0; i < s.count; ++i)
        {
            v.x += s.v[i].weight * s.v[i].w.x;
            v.y += s.v[i].weight * s.v[i].w.y;
        }
        float vv = Dot(v, v);
        if (vv == 0.0f)
        {
            break; // exactly touching cores
        }
        // The support point opposite v: A's farthest along v, B's
        // farthest against it.
        int32_t ia = Support(proxyA, v);
        int32_t ib = Support(proxyB, (m2Vec2){-v.x, -v.y});
        Vertex w = MakeVertex(proxyA, proxyB, ia, ib);
        if (Repeats(&before, ia, ib) || vv - Dot(v, w.w) <= 1.0e-6f * vv)
        {
            break; // no further gain
        }
        s.v[s.count++] = w;
    }
    m2DistanceResult result;
    result.pointA = Blend(&s, true);
    result.pointB = s.count == 3 ? result.pointA : Blend(&s, false);
    m2Vec2 gap = Sub(result.pointB, result.pointA);
    result.distance = sqrtf(Dot(gap, gap));
    result.normal = result.distance > 0.0f
                        ? (m2Vec2){gap.x / result.distance, gap.y / result.distance}
                        : (m2Vec2){0.0f, 0.0f};
    if (s.count == 3)
    {
        result.distance = 0.0f;
        result.normal = (m2Vec2){0.0f, 0.0f};
    }
    return result;
}

// Conservative advancement, translation only: proxy B slides along
// translation toward the resting proxy A and stops half a linear slop
// before the surfaces touch, so the reported normal always comes from a
// real gap. A start closer than that reports a hit at fraction 0 with a
// (0,0) normal, matching the ray convention.
m2CastResult m2ShapeCastProxy(const m2DistanceProxy* proxyA, const m2DistanceProxy* proxyB,
                              m2Vec2 translation, float maxFraction)
{
    m2CastResult out;
    out.fraction = 0.0f;
    out.normal = (m2Vec2){0.0f, 0.0f};
    out.pointA = (m2Vec2){0.0f, 0.0f};
    out.hit = false;
    const float slop = 0.005f;
    float target = proxyA->radius + proxyB->radius + 0.5f * slop;
    float tolerance = 0.25f * slop;
    m2DistanceProxy moved = *proxyB;
    float fraction = 0.0f;
    for (int32_t iter = 0; iter < 24; ++iter)
    {
        m2DistanceResult d = m2ShapeDistance(proxyA, &moved);
        if (d.distance < target + tolerance)
        {
            out.hit = true;
            if (iter > 0 && d.distance > 0.0f)
            {
                // The normal points from A toward B: it faces the incoming
                // shape, like a ray's surface normal faces the ray origin.
                out.fraction = fraction;
                out.normal = d.normal;
                out.pointA = (m2Vec2){d.pointA.x + proxyA->radius * d.normal.x,
                                      d.pointA.y + proxyA->radius * d.normal.y};
            }
            else
            {
                out.pointA = d.pointA;
            }
            return out;
        }
        float approach = -Dot(translation, d.normal);
        if (approach <= 0.0f)
        {
            return out; // separating or sliding past: no hit
        }
        fraction += (d.distance - target) / approach;
        if (fraction >= maxFraction)
        {
            return out;
        }
        for (int32_t i = 0; i < moved.count; ++i)
        {
            moved.points[i].x = proxyB->points[i].x + fraction * translation.x;
            moved.points[i].y = proxyB->points[i].y + fraction * translation.y;
        }
    }
    return out;
}
