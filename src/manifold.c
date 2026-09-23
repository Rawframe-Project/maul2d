// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact manifolds between convex shapes, all in body A's frame with B
// placed by the relative pose. Circles are points with a radius;
// capsules and segments are two-vertex polygons with a radius, so three
// kernels cover every pair: circle and circle, polygon and circle, and
// polygon and polygon.
//
// Polygon pairs use the separating axis test over both polygons' face
// normals. The deeper face becomes the reference, the most opposed face
// of the other polygon the incident edge, and the incident edge clipped
// to the reference face's extent gives up to two points (Sutherland and
// Hodgman's clipping, one edge against two side planes). Rounded shapes
// that are apart may touch corner to corner instead; the closest points
// of the two edges decide.
//
// Every contact point sits halfway between the two surfaces. Kernels are
// pure functions of their inputs.

#include "geometry.h"

#include "core.h"

#include "maul2d/base.h"
#include "maul2d/core_math.h"

// Points whose separation is within this of the best one count as ties.
#define M2_MANIFOLD_TOLERANCE (0.1f * 0.005f)

// A polygon point is named by the vertex of A and the vertex of B nearest
// to it, so the name survives a change of reference face.
#define M2_PAIR_ID(vertexA, vertexB) ((uint16_t)(((uint32_t)(vertexA) << 4) | (uint32_t)(vertexB)))

static float Dot(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

static m2Vec2 Sub(m2Vec2 a, m2Vec2 b)
{
    return (m2Vec2){a.x - b.x, a.y - b.y};
}

static m2Vec2 MulAdd(m2Vec2 a, float s, m2Vec2 b)
{
    return (m2Vec2){a.x + s * b.x, a.y + s * b.y};
}

// B's frame to A's frame.
static m2Vec2 ToA(m2RelativePose pose, m2Vec2 v)
{
    return (m2Vec2){pose.q.c * v.x - pose.q.s * v.y + pose.p.x,
                    pose.q.s * v.x + pose.q.c * v.y + pose.p.y};
}

// A's frame to B's frame.
static m2Vec2 ToB(m2RelativePose pose, m2Vec2 v)
{
    m2Vec2 d = Sub(v, pose.p);
    return (m2Vec2){pose.q.c * d.x + pose.q.s * d.y, -pose.q.s * d.x + pose.q.c * d.y};
}

static m2Manifold EmptyManifold(void)
{
    m2Manifold manifold;
    memset(&manifold, 0, sizeof(manifold));
    return manifold;
}

// One contact point: the surfaces lie radiusA past p along the normal on
// A's side and separation further on B's side; the point is their
// midpoint.
static void SetPoint(m2Manifold* manifold, int32_t k, m2Vec2 p, float radiusA, float separation,
                     uint16_t id, m2RelativePose pose)
{
    m2ManifoldPoint* mp = &manifold->points[k];
    mp->anchorA = MulAdd(p, radiusA + 0.5f * separation, manifold->normal);
    mp->anchorB = ToB(pose, mp->anchorA);
    mp->separation = separation;
    mp->id = id;
}

m2Manifold m2CollideCircles(const m2Circle* a, const m2Circle* b, m2RelativePose pose)
{
    m2Manifold manifold = EmptyManifold();
    m2Vec2 d = Sub(ToA(pose, b->center), a->center);
    float radius = a->radius + b->radius;
    float reach = radius + M2_SPECULATIVE_DISTANCE;
    float dd = Dot(d, d);
    if (dd > reach * reach)
    {
        return manifold;
    }
    float distance = sqrtf(dd);
    // Coincident centers have no direction; up is the fixed stand-in.
    manifold.normal =
        distance > 1.19209290e-7f ? (m2Vec2){d.x / distance, d.y / distance} : (m2Vec2){0.0f, 1.0f};
    manifold.pointCount = 1;
    SetPoint(&manifold, 0, a->center, a->radius, distance - radius, 0, pose);
    return manifold;
}

m2Manifold m2CollidePolygonAndCircle(const m2Polygon* a, const m2Circle* b, m2RelativePose pose)
{
    m2Manifold manifold = EmptyManifold();
    m2Vec2 c = ToA(pose, b->center);
    float radius = a->radius + b->radius;
    float separations[M2_MAX_POLYGON_VERTICES] = {0.0f};
    int32_t deepest = 0;
    for (int32_t i = 0; i < a->count; ++i)
    {
        separations[i] = Dot(a->normals[i], Sub(c, a->vertices[i]));
        deepest = separations[i] > separations[deepest] ? i : deepest;
    }
    if (separations[deepest] > radius + M2_SPECULATIVE_DISTANCE)
    {
        return manifold;
    }
    m2Vec2 closest;
    float distance;
    uint16_t id;
    if (separations[deepest] <= 0.0f)
    {
        // The center is inside the core: out through the nearest face.
        manifold.normal = a->normals[deepest];
        distance = separations[deepest];
        closest = MulAdd(c, -distance, manifold.normal);
        id = (uint16_t)deepest;
    }
    else
    {
        // Outside: the nearest boundary point over every edge that faces
        // the center, first edge first on ties.
        float best = 3.4e38f;
        closest = a->vertices[0];
        id = 0;
        for (int32_t i = 0; i < a->count; ++i)
        {
            if (separations[i] <= 0.0f)
            {
                continue;
            }
            int32_t j = i + 1 < a->count ? i + 1 : 0;
            m2Vec2 e = Sub(a->vertices[j], a->vertices[i]);
            float t = m2ClampF(Dot(Sub(c, a->vertices[i]), e) / Dot(e, e), 0.0f, 1.0f);
            m2Vec2 q = MulAdd(a->vertices[i], t, e);
            m2Vec2 r = Sub(c, q);
            if (Dot(r, r) < best)
            {
                best = Dot(r, r);
                closest = q;
                id = t == 0.0f   ? (uint16_t)(M2_FEATURE_VERTEX | (uint32_t)i)
                     : t == 1.0f ? (uint16_t)(M2_FEATURE_VERTEX | (uint32_t)j)
                                 : (uint16_t)i;
            }
        }
        distance = sqrtf(best);
        if (distance > radius + M2_SPECULATIVE_DISTANCE)
        {
            return manifold;
        }
        m2Vec2 r = Sub(c, closest);
        manifold.normal = (m2Vec2){r.x / distance, r.y / distance};
    }
    manifold.pointCount = 1;
    SetPoint(&manifold, 0, closest, a->radius, distance - radius, id, pose);
    return manifold;
}

// --- Polygon vs polygon --------------------------------------------------------

typedef struct Face
{
    int32_t index;
    float separation;
} Face;

// The face of ref that separates other best: the largest over ref's faces
// of the smallest signed distance of other's vertices past the face.
// Ties keep the lower face.
static Face DeepestFace(const m2Polygon* ref, const m2Polygon* other)
{
    Face best = {0, -3.4e38f};
    for (int32_t i = 0; i < ref->count; ++i)
    {
        float s = 3.4e38f;
        for (int32_t j = 0; j < other->count; ++j)
        {
            s = m2MinF(s, Dot(ref->normals[i], Sub(other->vertices[j], ref->vertices[i])));
        }
        if (s > best.separation)
        {
            best = (Face){i, s};
        }
    }
    return best;
}

// The edge of poly whose normal opposes n most; ties keep the lower edge.
static int32_t MostOpposedEdge(const m2Polygon* poly, m2Vec2 n)
{
    int32_t best = 0;
    for (int32_t i = 1; i < poly->count; ++i)
    {
        best = Dot(poly->normals[i], n) < Dot(poly->normals[best], n) ? i : best;
    }
    return best;
}

// Closest points of segments p1-q1 and p2-q2 as fractions along each,
// clamped to the segments. Degenerate segments act as points.
static void ClosestFractions(m2Vec2 p1, m2Vec2 q1, m2Vec2 p2, m2Vec2 q2, float* s, float* t)
{
    m2Vec2 d1 = Sub(q1, p1);
    m2Vec2 d2 = Sub(q2, p2);
    m2Vec2 r = Sub(p1, p2);
    float a = Dot(d1, d1);
    float e = Dot(d2, d2);
    float f = Dot(d2, r);
    const float tiny = 1.0e-12f;
    *s = 0.0f;
    *t = 0.0f;
    if (a <= tiny)
    {
        *t = e > tiny ? m2ClampF(f / e, 0.0f, 1.0f) : 0.0f;
        return;
    }
    float c = Dot(d1, r);
    if (e <= tiny)
    {
        *s = m2ClampF(-c / a, 0.0f, 1.0f);
        return;
    }
    float b = Dot(d1, d2);
    float denom = a * e - b * b;
    *s = denom > 0.0f ? m2ClampF((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
    *t = (b * *s + f) / e;
    if (*t < 0.0f)
    {
        *t = 0.0f;
        *s = m2ClampF(-c / a, 0.0f, 1.0f);
    }
    else if (*t > 1.0f)
    {
        *t = 1.0f;
        *s = m2ClampF((b - c) / a, 0.0f, 1.0f);
    }
}

// Both polygons in one frame: A's frame shifted to the mean of A's
// vertices, so the coordinates the kernel works with stay small.
static m2Vec2 ShiftedPair(const m2Polygon* a, const m2Polygon* b, m2RelativePose pose,
                          m2Polygon* la, m2Polygon* lb)
{
    m2Vec2 o = {0.0f, 0.0f};
    for (int32_t i = 0; i < a->count; ++i)
    {
        o = MulAdd(o, 1.0f, a->vertices[i]);
    }
    o = (m2Vec2){o.x / (float)a->count, o.y / (float)a->count};
    *la = *a;
    *lb = *b;
    for (int32_t i = 0; i < a->count; ++i)
    {
        la->vertices[i] = Sub(a->vertices[i], o);
    }
    for (int32_t i = 0; i < b->count; ++i)
    {
        lb->vertices[i] = Sub(ToA(pose, b->vertices[i]), o);
        lb->normals[i] = (m2Vec2){pose.q.c * b->normals[i].x - pose.q.s * b->normals[i].y,
                                  pose.q.s * b->normals[i].x + pose.q.c * b->normals[i].y};
    }
    return o;
}

// Clips the incident edge to the extent of the reference face and keeps
// the points within the speculative distance, in the shifted frame, with
// the normal pointing from A to B.
static m2Manifold ClipToFace(const m2Polygon* ref, const m2Polygon* inc, int32_t face, int32_t edge,
                             bool flip)
{
    m2Manifold manifold = EmptyManifold();
    int32_t face2 = face + 1 < ref->count ? face + 1 : 0;
    int32_t edge2 = edge + 1 < inc->count ? edge + 1 : 0;
    m2Vec2 n = ref->normals[face];
    m2Vec2 t = {-n.y, n.x}; // along the face, counterclockwise
    m2Vec2 v1 = ref->vertices[face];
    float length = Dot(Sub(ref->vertices[face2], v1), t);
    // The incident edge runs against the face: its first vertex lies at
    // the face's far end.
    m2Vec2 w[2] = {inc->vertices[edge], inc->vertices[edge2]};
    int32_t incVertex[2] = {edge, edge2};
    int32_t refVertex[2] = {face2, face};
    float s[2] = {Dot(Sub(w[0], v1), t), Dot(Sub(w[1], v1), t)};
    if (m2MaxF(s[0], s[1]) < 0.0f || m2MinF(s[0], s[1]) > length)
    {
        return manifold;
    }
    float span = s[1] - s[0];
    float radius = ref->radius + inc->radius;
    manifold.normal = flip ? (m2Vec2){-n.x, -n.y} : n;
    for (int32_t k = 0; k < 2; ++k)
    {
        float bound = s[k] < 0.0f ? 0.0f : (s[k] > length ? length : s[k]);
        m2Vec2 q = w[k];
        if (bound != s[k] && (span > 1.19209290e-7f || span < -1.19209290e-7f))
        {
            q = MulAdd(w[0], (bound - s[0]) / span, Sub(w[1], w[0]));
        }
        float depth = Dot(Sub(q, v1), n);
        if (depth - radius > M2_SPECULATIVE_DISTANCE)
        {
            continue;
        }
        // The point on the reference surface, then halfway to the other.
        m2Vec2 onRef = MulAdd(q, ref->radius - depth, n);
        m2ManifoldPoint* mp = &manifold.points[manifold.pointCount++];
        mp->anchorA = MulAdd(onRef, 0.5f * (depth - radius), n);
        mp->separation = depth - radius;
        mp->id =
            flip ? M2_PAIR_ID(incVertex[k], refVertex[k]) : M2_PAIR_ID(refVertex[k], incVertex[k]);
    }
    return manifold;
}

// Rounded polygons apart from each other can touch corner to corner,
// closer than any clipped point: one point on the line between the two
// corners replaces the clip then.
static bool CornerContact(const m2Polygon* la, const m2Polygon* lb, int32_t edgeA, int32_t edgeB,
                          m2Manifold* manifold)
{
    int32_t a2 = edgeA + 1 < la->count ? edgeA + 1 : 0;
    int32_t b2 = edgeB + 1 < lb->count ? edgeB + 1 : 0;
    float s;
    float t;
    ClosestFractions(la->vertices[edgeA], la->vertices[a2], lb->vertices[edgeB], lb->vertices[b2],
                     &s, &t);
    bool cornerA = s == 0.0f || s == 1.0f;
    bool cornerB = t == 0.0f || t == 1.0f;
    int32_t va = s == 1.0f ? a2 : edgeA;
    int32_t vb = t == 1.0f ? b2 : edgeB;
    m2Vec2 d = Sub(lb->vertices[vb], la->vertices[va]);
    float distance = sqrtf(Dot(d, d));
    float radius = la->radius + lb->radius;
    float clipped = 3.4e38f;
    for (int32_t k = 0; k < manifold->pointCount; ++k)
    {
        clipped = m2MinF(clipped, manifold->points[k].separation);
    }
    if (!cornerA || !cornerB || distance <= 1.19209290e-7f ||
        distance - radius + M2_MANIFOLD_TOLERANCE >= clipped)
    {
        return false;
    }
    *manifold = EmptyManifold();
    manifold->normal = (m2Vec2){d.x / distance, d.y / distance};
    manifold->pointCount = 1;
    m2ManifoldPoint* mp = &manifold->points[0];
    mp->separation = distance - radius;
    mp->anchorA = MulAdd(la->vertices[va], la->radius + 0.5f * mp->separation, manifold->normal);
    mp->id = M2_PAIR_ID(va, vb);
    return true;
}

m2Manifold m2CollidePolygons(const m2Polygon* a, const m2Polygon* b, m2RelativePose pose)
{
    m2Polygon la;
    m2Polygon lb;
    m2Vec2 origin = ShiftedPair(a, b, pose, &la, &lb);
    Face fa = DeepestFace(&la, &lb);
    Face fb = DeepestFace(&lb, &la);
    float radius = la.radius + lb.radius;
    if (fa.separation > radius + M2_SPECULATIVE_DISTANCE ||
        fb.separation > radius + M2_SPECULATIVE_DISTANCE)
    {
        return EmptyManifold();
    }
    // B's face becomes the reference only when clearly deeper, so a pair
    // at rest does not flip between the two and lose its warm start.
    bool flip = fb.separation > fa.separation + M2_MANIFOLD_TOLERANCE;
    const m2Polygon* ref = flip ? &lb : &la;
    const m2Polygon* inc = flip ? &la : &lb;
    int32_t face = flip ? fb.index : fa.index;
    int32_t edge = MostOpposedEdge(inc, ref->normals[face]);
    m2Manifold manifold = ClipToFace(ref, inc, face, edge, flip);
    if (m2MaxF(fa.separation, fb.separation) > M2_MANIFOLD_TOLERANCE)
    {
        CornerContact(&la, &lb, flip ? edge : face, flip ? face : edge, &manifold);
    }
    // Two points always run along the tangent (the normal turned
    // counterclockwise), whichever polygon holds the reference face: the
    // solver visits them in this order, and an order that changed with
    // the reference side would bias a stack sideways.
    m2Vec2 tangent = {-manifold.normal.y, manifold.normal.x};
    if (manifold.pointCount == 2 &&
        Dot(Sub(manifold.points[1].anchorA, manifold.points[0].anchorA), tangent) < 0.0f)
    {
        m2ManifoldPoint swap = manifold.points[0];
        manifold.points[0] = manifold.points[1];
        manifold.points[1] = swap;
    }
    for (int32_t k = 0; k < manifold.pointCount && k < 2; ++k)
    {
        m2ManifoldPoint* mp = &manifold.points[k];
        mp->anchorA = MulAdd(mp->anchorA, 1.0f, origin);
        mp->anchorB = ToB(pose, mp->anchorA);
    }
    return manifold;
}

// --- Point-to-shape distance (bullet CCD kernel) ------------------------------

static float PointSegmentDistance(m2Vec2 p, m2Vec2 a, m2Vec2 b)
{
    m2Vec2 e = Sub(b, a);
    float ee = Dot(e, e);
    float t = ee > 0.0f ? m2ClampF(Dot(Sub(p, a), e) / ee, 0.0f, 1.0f) : 0.0f;
    m2Vec2 r = Sub(p, MulAdd(a, t, e));
    return sqrtf(Dot(r, r));
}

float m2PointShapeDistance(const m2ShapeGeometry* geometry, m2Vec2 point)
{
    switch (geometry->type)
    {
    case m2_circleShape:
    {
        m2Vec2 d = Sub(point, geometry->circle.center);
        return sqrtf(Dot(d, d)) - geometry->circle.radius;
    }
    case m2_capsuleShape:
        return PointSegmentDistance(point, geometry->capsule.point1, geometry->capsule.point2) -
               geometry->capsule.radius;
    case m2_segmentShape:
        return PointSegmentDistance(point, geometry->segment.point1, geometry->segment.point2);
    case m2_chainSegmentShape:
        return PointSegmentDistance(point, geometry->chainSegment.segment.point1,
                                    geometry->chainSegment.segment.point2);
    default:
    {
        // Inside the core the nearest face decides; outside, the
        // nearest edge.
        const m2Polygon* poly = &geometry->polygon;
        float inside = -3.4e38f;
        float outside = 3.4e38f;
        for (int32_t i = 0; i < poly->count; ++i)
        {
            int32_t j = i + 1 < poly->count ? i + 1 : 0;
            inside = m2MaxF(inside, Dot(poly->normals[i], Sub(point, poly->vertices[i])));
            outside =
                m2MinF(outside, PointSegmentDistance(point, poly->vertices[i], poly->vertices[j]));
        }
        return (inside <= 0.0f ? inside : outside) - poly->radius;
    }
    }
}
