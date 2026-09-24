// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape validation, convex hulls, world-space AABBs and mass properties.
// Validation uses relative and geometric thresholds: these guarantees are
// the non-zero-divisor preconditions the simulation relies on.

#include "geometry.h"
#include "world_internal.h"

#include "maul2d/base.h"
#include "maul2d/core_math.h"

// Slop-scaled geometric floors.
#define M2_LINEAR_SLOP     0.005f
#define M2_MIN_EDGE_LENGTH (2.0f * M2_LINEAR_SLOP)
// Thinness bound: area must exceed this fraction of perimeter^2 (a
// scale-free sliver rejector; a square scores 1/16 = 0.0625).
#define M2_MIN_THINNESS 0.001f

static bool IsFiniteF(float x)
{
    return m2FiniteF(x);
}

static bool IsFiniteVec(m2Vec2 v)
{
    return IsFiniteF(v.x) && IsFiniteF(v.y);
}

static float EdgeLength(m2Vec2 a, m2Vec2 b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    // sqrtf is IEEE-exact (allowed op set).
    return sqrtf(dx * dx + dy * dy);
}

bool m2ValidateCircle(const m2Circle* circle)
{
    return circle != NULL && IsFiniteVec(circle->center) && IsFiniteF(circle->radius) &&
           circle->radius >= M2_LINEAR_SLOP;
}

bool m2ValidateCapsule(const m2Capsule* capsule)
{
    if (capsule == NULL || !IsFiniteVec(capsule->point1) || !IsFiniteVec(capsule->point2) ||
        !IsFiniteF(capsule->radius) || capsule->radius < M2_LINEAR_SLOP)
    {
        return false;
    }
    // Relative floor: 1-ulp-apart points must not pass. The
    // axis normalization divides by this length.
    float length = EdgeLength(capsule->point1, capsule->point2);
    float scale = m2MaxF(m2AbsF(capsule->point1.x) + m2AbsF(capsule->point1.y),
                         m2AbsF(capsule->point2.x) + m2AbsF(capsule->point2.y));
    return length >= m2MaxF(M2_MIN_EDGE_LENGTH, 1.0e-5f * scale);
}

bool m2ValidateSegment(const m2Segment* segment)
{
    return segment != NULL && IsFiniteVec(segment->point1) && IsFiniteVec(segment->point2) &&
           EdgeLength(segment->point1, segment->point2) >= M2_MIN_EDGE_LENGTH;
}

bool m2ValidatePolygon(const m2Polygon* polygon)
{
    if (polygon == NULL || polygon->count < 3 || polygon->count > M2_MAX_POLYGON_VERTICES ||
        !IsFiniteF(polygon->radius) || polygon->radius < 0.0f)
    {
        return false;
    }

    float area = 0.0f;
    float perimeter = 0.0f;
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        m2Vec2 a = polygon->vertices[i];
        m2Vec2 b = polygon->vertices[(i + 1) % polygon->count];
        if (!IsFiniteVec(a))
        {
            return false;
        }
        float edge = EdgeLength(a, b);
        if (edge < M2_MIN_EDGE_LENGTH)
        {
            return false; // near-coincident vertices: cancellation in normals
        }
        perimeter += edge;
        area += 0.5f * (a.x * b.y - a.y * b.x);
        // Convexity + CCW winding: every cross product must be positive.
        m2Vec2 c = polygon->vertices[(i + 2) % polygon->count];
        float cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (cross <= 0.0f)
        {
            return false;
        }
    }
    // Scale-free sliver rejection (an absolute area epsilon would be
    // scale-dependent).
    return area > M2_MIN_THINNESS * perimeter * perimeter;
}

// Twice the signed area of the triangle o, a, b: positive when b lies
// to the left of o->a.
static float Turn(m2Vec2 o, m2Vec2 a, m2Vec2 b)
{
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// The distance of p from the line through a and b.
static float LineDistance(m2Vec2 a, m2Vec2 b, m2Vec2 p)
{
    float length = EdgeLength(a, b);
    return length > 0.0f ? (Turn(a, b, p) < 0.0f ? -Turn(a, b, p) : Turn(a, b, p)) / length
                         : EdgeLength(a, p);
}

// Keeps the first of any points closer than four linear slops. Returns
// how many remain.
static int32_t WeldPoints(const m2Vec2* points, int32_t count, m2Vec2* out)
{
    const float tolSqr = 16.0f * M2_LINEAR_SLOP * M2_LINEAR_SLOP;
    int32_t n = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        bool unique = true;
        for (int32_t j = 0; j < n && unique; ++j)
        {
            float dx = points[i].x - out[j].x;
            float dy = points[i].y - out[j].y;
            unique = dx * dx + dy * dy >= tolSqr;
        }
        if (unique)
        {
            out[n++] = points[i];
        }
    }
    return n;
}

// Andrew's monotone chain: points sorted by x then y, a lower chain left
// to right and an upper chain right to left, each popping points that
// do not turn left. The result is counterclockwise from the leftmost
// point.
static int32_t MonotoneChain(m2Vec2* ps, int32_t n, m2Vec2* hull)
{
    for (int32_t i = 1; i < n; ++i)
    {
        m2Vec2 key = ps[i];
        int32_t j = i - 1;
        while (j >= 0 && (ps[j].x > key.x || (ps[j].x == key.x && ps[j].y > key.y)))
        {
            ps[j + 1] = ps[j];
            j -= 1;
        }
        ps[j + 1] = key;
    }
    int32_t k = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        while (k >= 2 && Turn(hull[k - 2], hull[k - 1], ps[i]) <= 0.0f)
        {
            k -= 1;
        }
        hull[k++] = ps[i];
    }
    for (int32_t i = n - 2, lower = k + 1; i >= 0; --i)
    {
        while (k >= lower && Turn(hull[k - 2], hull[k - 1], ps[i]) <= 0.0f)
        {
            k -= 1;
        }
        hull[k++] = ps[i];
    }
    return k - 1; // the last point repeats the first
}

// Drops vertices within two linear slops of the line through their
// neighbors, one at a time, lowest index first, until none is left.
static int32_t MergeCollinear(m2Vec2* hull, int32_t n)
{
    bool merged = true;
    while (merged && n > 2)
    {
        merged = false;
        for (int32_t i = 0; i < n && !merged; ++i)
        {
            m2Vec2 prev = hull[(i + n - 1) % n];
            m2Vec2 next = hull[(i + 1) % n];
            if (LineDistance(prev, next, hull[i]) <= 2.0f * M2_LINEAR_SLOP)
            {
                for (int32_t j = i; j < n - 1; ++j)
                {
                    hull[j] = hull[j + 1];
                }
                n -= 1;
                merged = true;
            }
        }
    }
    return n;
}

m2Polygon m2ComputeHull(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon invalid;
    memset(&invalid, 0, sizeof(invalid));
    if (points == NULL || count < 3)
    {
        return invalid; // check your data: count == 0 is the loud sign
    }
    count = count < M2_MAX_POLYGON_VERTICES ? count : M2_MAX_POLYGON_VERTICES;
    m2Vec2 ps[M2_MAX_POLYGON_VERTICES];
    int32_t n = WeldPoints(points, count, ps);
    if (n < 3)
    {
        return invalid; // welded away: a scale problem, be loud
    }
    m2Vec2 hull[2 * M2_MAX_POLYGON_VERTICES];
    n = MergeCollinear(hull, MonotoneChain(ps, n, hull));
    if (n < 3)
    {
        return invalid; // all collinear
    }
    // The constructor adds the normals and its own loud validation.
    return m2MakePolygon(hull, n, radius);
}

m2Polygon m2MakePolygon(const m2Vec2* points, int32_t count, float radius)
{
    m2Polygon polygon;
    memset(&polygon, 0, sizeof(polygon)); // deterministic bytes in unions
    if (points == NULL || count < 3 || count > M2_MAX_POLYGON_VERTICES)
    {
        return polygon; // count == 0 marks invalid
    }
    for (int32_t i = 0; i < count; ++i)
    {
        polygon.vertices[i] = points[i];
    }
    polygon.count = count;
    polygon.radius = radius;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 a = polygon.vertices[i];
        m2Vec2 b = polygon.vertices[(i + 1) % count];
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float length = sqrtf(dx * dx + dy * dy);
        if (length < M2_MIN_EDGE_LENGTH)
        {
            memset(&polygon, 0, sizeof(polygon));
            return polygon;
        }
        float inv = 1.0f / length;
        polygon.normals[i] = (m2Vec2){dy * inv, -dx * inv};
    }
    if (!m2ValidatePolygon(&polygon))
    {
        memset(&polygon, 0, sizeof(polygon));
    }
    return polygon;
}

m2Polygon m2MakeBox(float halfWidth, float halfHeight)
{
    m2Vec2 points[4] = {{-halfWidth, -halfHeight},
                        {halfWidth, -halfHeight},
                        {halfWidth, halfHeight},
                        {-halfWidth, halfHeight}};
    return m2MakePolygon(points, 4, 0.0f);
}

// --- World-space AABBs (rotation-aware; f64 crossing at body position) -------

static m2Pos2 WorldPoint(m2Transform xf, m2Vec2 local)
{
    // The single f64 crossing for this stage: rotate in f32, then add to
    // the f64 body position.
    float x = xf.q.c * local.x - xf.q.s * local.y;
    float y = xf.q.s * local.x + xf.q.c * local.y;
    return (m2Pos2){xf.p.x + (double)x, xf.p.y + (double)y};
}

m2Aabb m2ComputeShapeAabb(const m2ShapeGeometry* geometry, m2Transform xf)
{
    m2Aabb aabb;
    switch (geometry->type)
    {
    case m2_circleShape:
    {
        m2Pos2 c = WorldPoint(xf, geometry->circle.center);
        double r = (double)geometry->circle.radius;
        aabb.lowerBound = (m2Pos2){c.x - r, c.y - r};
        aabb.upperBound = (m2Pos2){c.x + r, c.y + r};
        return aabb;
    }
    case m2_capsuleShape:
    {
        m2Pos2 p1 = WorldPoint(xf, geometry->capsule.point1);
        m2Pos2 p2 = WorldPoint(xf, geometry->capsule.point2);
        double r = (double)geometry->capsule.radius;
        aabb.lowerBound =
            (m2Pos2){(p1.x < p2.x ? p1.x : p2.x) - r, (p1.y < p2.y ? p1.y : p2.y) - r};
        aabb.upperBound =
            (m2Pos2){(p1.x > p2.x ? p1.x : p2.x) + r, (p1.y > p2.y ? p1.y : p2.y) + r};
        return aabb;
    }
    case m2_polygonShape:
    {
        m2Pos2 first = WorldPoint(xf, geometry->polygon.vertices[0]);
        aabb.lowerBound = first;
        aabb.upperBound = first;
        for (int32_t i = 1; i < geometry->polygon.count; ++i)
        {
            m2Pos2 p = WorldPoint(xf, geometry->polygon.vertices[i]);
            aabb.lowerBound.x = p.x < aabb.lowerBound.x ? p.x : aabb.lowerBound.x;
            aabb.lowerBound.y = p.y < aabb.lowerBound.y ? p.y : aabb.lowerBound.y;
            aabb.upperBound.x = p.x > aabb.upperBound.x ? p.x : aabb.upperBound.x;
            aabb.upperBound.y = p.y > aabb.upperBound.y ? p.y : aabb.upperBound.y;
        }
        double r = (double)geometry->polygon.radius;
        aabb.lowerBound.x -= r;
        aabb.lowerBound.y -= r;
        aabb.upperBound.x += r;
        aabb.upperBound.y += r;
        return aabb;
    }
    default:
    {
        M2_ASSERT(geometry->type == m2_segmentShape || geometry->type == m2_chainSegmentShape);
        const m2Segment* seg = geometry->type == m2_segmentShape ? &geometry->segment
                                                                 : &geometry->chainSegment.segment;
        m2Pos2 p1 = WorldPoint(xf, seg->point1);
        m2Pos2 p2 = WorldPoint(xf, seg->point2);
        aabb.lowerBound = (m2Pos2){p1.x < p2.x ? p1.x : p2.x, p1.y < p2.y ? p1.y : p2.y};
        aabb.upperBound = (m2Pos2){p1.x > p2.x ? p1.x : p2.x, p1.y > p2.y ? p1.y : p2.y};
        return aabb;
    }
    }
}

// --- Mass properties ----------------------------------------------------------

// Area, first moment and second moment of a region about a reference
// point, accumulated part by part.
typedef struct Moments
{
    float area;
    m2Vec2 first; // area times the centroid
    float polar;  // second moment about the reference point
} Moments;

// Adds a part given by its area, its centroid and its own second moment
// about that centroid.
static void AddPart(Moments* m, float area, m2Vec2 centroid, float ownPolar)
{
    m->area += area;
    m->first.x += area * centroid.x;
    m->first.y += area * centroid.y;
    m->polar += ownPolar + area * (centroid.x * centroid.x + centroid.y * centroid.y);
}

// A convex polygon swollen by radius: by Steiner's decomposition, the
// core polygon, a rectangle radius wide on every edge, and a circular
// sector at every vertex spanning the turn between its two edge
// normals (the sectors together form one disc). Positions are taken
// relative to the vertex mean, which lies inside the polygon, so the
// sums carry no large offsets. Two vertices make a capsule.
static m2MassData RoundedPolygonMass(const m2Vec2* vertices, const m2Vec2* normals, int32_t count,
                                     float radius, float density)
{
    m2Vec2 o = {0.0f, 0.0f};
    for (int32_t i = 0; i < count; ++i)
    {
        o.x += vertices[i].x / (float)count;
        o.y += vertices[i].y / (float)count;
    }
    Moments m = {0.0f, {0.0f, 0.0f}, 0.0f};
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t j = i + 1 < count ? i + 1 : 0;
        m2Vec2 a = {vertices[i].x - o.x, vertices[i].y - o.y};
        m2Vec2 b = {vertices[j].x - o.x, vertices[j].y - o.y};
        // The core triangle o, a, b. Its second moment about o is
        // area / 6 * (a.a + a.b + b.b).
        float tri = 0.5f * (a.x * b.y - a.y * b.x);
        float triPolar = tri *
                         (a.x * a.x + a.y * a.y + a.x * b.x + a.y * b.y + b.x * b.x + b.y * b.y) *
                         (1.0f / 6.0f);
        m.area += tri;
        m.first.x += tri * (a.x + b.x) * (1.0f / 3.0f);
        m.first.y += tri * (a.y + b.y) * (1.0f / 3.0f);
        m.polar += triPolar;
        if (radius <= 0.0f)
        {
            continue;
        }
        // The edge rectangle.
        m2Vec2 n = normals[i];
        float length = EdgeLength(a, b);
        float rect = length * radius;
        m2Vec2 rectCenter = {0.5f * (a.x + b.x) + 0.5f * radius * n.x,
                             0.5f * (a.y + b.y) + 0.5f * radius * n.y};
        AddPart(&m, rect, rectCenter, rect * (length * length + radius * radius) * (1.0f / 12.0f));
        // The sector at vertex b, turning from this edge's normal to the
        // next one. With half angle h: |n1 - n2| = 2 sin h, |n1 + n2| =
        // 2 cos h, and the sector's centroid lies 4 r sin h / (3 * 2h)
        // out along the bisector.
        m2Vec2 n2 = normals[j];
        m2Vec2 sum = {n.x + n2.x, n.y + n2.y};
        m2Vec2 diff = {n2.x - n.x, n2.y - n.y};
        float sinHalf = 0.5f * sqrtf(diff.x * diff.x + diff.y * diff.y);
        float cosHalf = 0.5f * sqrtf(sum.x * sum.x + sum.y * sum.y);
        float half = m2Atan2(sinHalf, cosHalf);
        // At a two-vertex end the normals are opposite and the bisector
        // points along the edge, away from the other vertex.
        m2Vec2 bisector = cosHalf > 1.0e-6f
                              ? (m2Vec2){0.5f * sum.x / cosHalf, 0.5f * sum.y / cosHalf}
                              : (m2Vec2){-n.y, n.x};
        float sector = half * radius * radius;
        float reach = half > 0.0f ? 4.0f * radius * sinHalf / (6.0f * half) : 0.0f;
        m2Vec2 sectorCenter = {b.x + reach * bisector.x, b.y + reach * bisector.y};
        // Its second moment about the apex is r^2 / 2 per unit area; move
        // it to the sector's own centroid.
        float ownPolar = sector * (0.5f * radius * radius - reach * reach);
        AddPart(&m, sector, sectorCenter, ownPolar);
    }
    m2MassData data;
    data.mass = density * m.area;
    M2_ASSERT(m.area > 0.0f); // validation guarantees this
    m2Vec2 c = {m.first.x / m.area, m.first.y / m.area};
    data.center = (m2Vec2){o.x + c.x, o.y + c.y};
    // About the centroid, so the caller's shift to the body center of
    // mass stays free of cancellation.
    data.rotationalInertia = density * (m.polar - m.area * (c.x * c.x + c.y * c.y));
    return data;
}

m2MassData m2ComputeShapeMass(const m2ShapeGeometry* geometry, float density)
{
    m2MassData data = {0};
    switch (geometry->type)
    {
    case m2_circleShape:
    {
        float r = geometry->circle.radius;
        data.mass = density * M2_PI * r * r;
        data.center = geometry->circle.center;
        // About the centroid; the caller shifts to the body center of mass.
        data.rotationalInertia = data.mass * 0.5f * r * r;
        return data;
    }
    case m2_capsuleShape:
    {
        m2Polygon core = m2MakeSegmentProxy(geometry->capsule.point1, geometry->capsule.point2,
                                            geometry->capsule.radius);
        return RoundedPolygonMass(core.vertices, core.normals, 2, core.radius, density);
    }
    case m2_polygonShape:
    {
        const m2Polygon* poly = &geometry->polygon;
        return RoundedPolygonMass(poly->vertices, poly->normals, poly->count, poly->radius,
                                  density);
    }
    default:
        return data; // segments and chains are massless
    }
}

m2Polygon m2MakeSegmentProxy(m2Vec2 p1, m2Vec2 p2, float radius)
{
    m2Polygon proxy;
    memset(&proxy, 0, sizeof(proxy));
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float length = sqrtf(dx * dx + dy * dy);
    M2_ASSERT(length >= M2_MIN_EDGE_LENGTH); // shape validation guarantees
    float inv = 1.0f / length;
    m2Vec2 axis = {dx * inv, dy * inv};
    proxy.vertices[0] = p1;
    proxy.vertices[1] = p2;
    proxy.normals[0] = (m2Vec2){axis.y, -axis.x};
    proxy.normals[1] = (m2Vec2){-axis.y, axis.x};
    proxy.count = 2;
    proxy.radius = radius;
    return proxy;
}

// --- Convex decomposition -------------------------------------------------------
//
// Ear clipping into triangles, then Hertel-Mehlhorn style merging:
// pieces fuse across a shared edge whenever the union stays strictly
// convex and inside the 8-vertex polygon limit. Every scan runs in
// ascending index order and every pass picks the lowest-index
// candidate, so the decomposition is canonical: same outline, same
// pieces, on every platform.

#define M2_MAX_OUTLINE 64

typedef struct m2DecompPiece
{
    int32_t idx[M2_MAX_POLYGON_VERTICES];
    int32_t n;
} m2DecompPiece;

static float DecompCross(m2Vec2 a, m2Vec2 b, m2Vec2 c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Inclusive point-in-triangle for a CCW triangle: boundary counts as
// inside, which makes the ear test conservative.
static bool DecompPointInTriangle(m2Vec2 a, m2Vec2 b, m2Vec2 c, m2Vec2 p)
{
    return DecompCross(a, b, p) >= 0.0f && DecompCross(b, c, p) >= 0.0f &&
           DecompCross(c, a, p) >= 0.0f;
}

// Proper or improper intersection of segments ab and cd, endpoints
// included; used to reject self-intersecting outlines loudly.
static bool DecompSegmentsCross(m2Vec2 a, m2Vec2 b, m2Vec2 c, m2Vec2 d)
{
    float d1 = DecompCross(c, d, a);
    float d2 = DecompCross(c, d, b);
    float d3 = DecompCross(a, b, c);
    float d4 = DecompCross(a, b, d);
    if (((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) &&
        ((d3 > 0.0f && d4 < 0.0f) || (d3 < 0.0f && d4 > 0.0f)))
    {
        return true;
    }
    return false;
}

int32_t m2DecomposeOutline(const m2Vec2* points, int32_t count, m2Polygon* pieces, int32_t capacity)
{
    if (points == NULL || count < 3 || count > M2_MAX_OUTLINE || capacity < 0)
    {
        m2Refuse(NULL, m2_errorInvalid);
        return 0;
    }
    float area2 = 0.0f;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 p = points[i];
        if (!m2FiniteF(p.x) || !m2FiniteF(p.y))
        {
            m2Refuse(NULL, m2_errorInvalid);
            return 0;
        }
        m2Vec2 q = points[(i + 1) % count];
        area2 += p.x * q.y - q.x * p.y;
    }
    if (!(area2 > 0.0f))
    {
        m2Refuse(NULL, m2_errorInvalid); // clockwise or degenerate outline
        return 0;
    }
    for (int32_t i = 0; i < count; ++i)
    {
        for (int32_t j = i + 1; j < count; ++j)
        {
            // Skip adjacent segments (they share an endpoint).
            if (j == i || (j + 1) % count == i || (i + 1) % count == j)
            {
                continue;
            }
            if (DecompSegmentsCross(points[i], points[(i + 1) % count], points[j],
                                    points[(j + 1) % count]))
            {
                m2Refuse(NULL, m2_errorInvalid); // self-intersecting outline
                return 0;
            }
        }
    }

    // Ear clipping. The ring holds original indices; each pass clips
    // the valid ear with the lowest original index.
    int32_t ring[M2_MAX_OUTLINE];
    int32_t ringCount = count;
    for (int32_t i = 0; i < count; ++i)
    {
        ring[i] = i;
    }
    int32_t triangles[3 * (M2_MAX_OUTLINE - 2)];
    int32_t triangleCount = 0;
    while (ringCount > 3)
    {
        int32_t bestPos = -1;
        int32_t bestIndex = M2_MAX_OUTLINE;
        bool degenerate = false;
        for (int32_t k = 0; k < ringCount; ++k)
        {
            int32_t ip = ring[(k + ringCount - 1) % ringCount];
            int32_t ic = ring[k];
            int32_t in = ring[(k + 1) % ringCount];
            float cross = DecompCross(points[ip], points[ic], points[in]);
            if (cross == 0.0f)
            {
                // A straight vertex clips for free, no triangle.
                bestPos = k;
                degenerate = true;
                break;
            }
            if (cross < 0.0f)
            {
                continue; // reflex
            }
            bool blocked = false;
            for (int32_t m = 0; m < ringCount && !blocked; ++m)
            {
                int32_t io = ring[m];
                if (io == ip || io == ic || io == in)
                {
                    continue;
                }
                blocked = DecompPointInTriangle(points[ip], points[ic], points[in], points[io]);
            }
            if (!blocked && ic < bestIndex)
            {
                bestIndex = ic;
                bestPos = k;
            }
        }
        if (bestPos < 0)
        {
            m2Refuse(NULL, m2_errorInvalid); // no ear: numerically hostile outline
            return 0;
        }
        if (!degenerate)
        {
            triangles[3 * triangleCount + 0] = ring[(bestPos + ringCount - 1) % ringCount];
            triangles[3 * triangleCount + 1] = ring[bestPos];
            triangles[3 * triangleCount + 2] = ring[(bestPos + 1) % ringCount];
            triangleCount += 1;
        }
        for (int32_t k = bestPos; k < ringCount - 1; ++k)
        {
            ring[k] = ring[k + 1];
        }
        ringCount -= 1;
    }
    float lastCross = DecompCross(points[ring[0]], points[ring[1]], points[ring[2]]);
    if (lastCross > 0.0f)
    {
        triangles[3 * triangleCount + 0] = ring[0];
        triangles[3 * triangleCount + 1] = ring[1];
        triangles[3 * triangleCount + 2] = ring[2];
        triangleCount += 1;
    }

    // Merge pass: fuse two pieces across a shared edge whenever the
    // union stays strictly convex and at most 8 vertices. Ascending
    // pair order, restart after every fuse: canonical.
    m2DecompPiece work[M2_MAX_OUTLINE - 2];
    int32_t pieceCount = triangleCount;
    for (int32_t t = 0; t < triangleCount; ++t)
    {
        work[t].idx[0] = triangles[3 * t + 0];
        work[t].idx[1] = triangles[3 * t + 1];
        work[t].idx[2] = triangles[3 * t + 2];
        work[t].n = 3;
    }
    bool fused = true;
    while (fused)
    {
        fused = false;
        for (int32_t i = 0; i < pieceCount && !fused; ++i)
        {
            for (int32_t j = i + 1; j < pieceCount && !fused; ++j)
            {
                if (work[i].n + work[j].n - 2 > M2_MAX_POLYGON_VERTICES)
                {
                    continue;
                }
                for (int32_t e = 0; e < work[i].n && !fused; ++e)
                {
                    int32_t u = work[i].idx[e];
                    int32_t v = work[i].idx[(e + 1) % work[i].n];
                    int32_t f = -1;
                    for (int32_t g = 0; g < work[j].n; ++g)
                    {
                        if (work[j].idx[g] == v && work[j].idx[(g + 1) % work[j].n] == u)
                        {
                            f = g;
                        }
                    }
                    if (f < 0)
                    {
                        continue;
                    }
                    // Merged ring: piece i from v around to u, then
                    // piece j's far chain from u back toward v.
                    m2DecompPiece merged;
                    merged.n = 0;
                    for (int32_t k = 0; k < work[i].n; ++k)
                    {
                        merged.idx[merged.n++] = work[i].idx[(e + 1 + k) % work[i].n];
                    }
                    for (int32_t k = 2; k < work[j].n; ++k)
                    {
                        merged.idx[merged.n++] = work[j].idx[(f + k) % work[j].n];
                    }
                    bool convex = true;
                    for (int32_t k = 0; k < merged.n && convex; ++k)
                    {
                        m2Vec2 a = points[merged.idx[k]];
                        m2Vec2 b = points[merged.idx[(k + 1) % merged.n]];
                        m2Vec2 c = points[merged.idx[(k + 2) % merged.n]];
                        convex = DecompCross(a, b, c) > 0.0f;
                    }
                    if (!convex)
                    {
                        continue;
                    }
                    work[i] = merged;
                    for (int32_t k = j; k < pieceCount - 1; ++k)
                    {
                        work[k] = work[k + 1];
                    }
                    pieceCount -= 1;
                    fused = true;
                }
            }
        }
    }

    // Validate each piece through the ordinary polygon road; slivers
    // that validation rejects are skipped (documented).
    int32_t total = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2Vec2 verts[M2_MAX_POLYGON_VERTICES];
        for (int32_t k = 0; k < work[i].n; ++k)
        {
            verts[k] = points[work[i].idx[k]];
        }
        m2Polygon piece = m2MakePolygon(verts, work[i].n, 0.0f);
        if (piece.count == 0)
        {
            continue;
        }
        if (pieces != NULL && total < capacity)
        {
            pieces[total] = piece;
        }
        total += 1;
    }
    return total;
}

// Area of a shape's core, used by the buoyancy fallback for capsules
// and thin shapes (circles and polygons get exact submersion).
float m2ShapeArea(const m2ShapeGeometry* g)
{
    switch (g->type)
    {
    case m2_circleShape:
        return M2_PI * g->circle.radius * g->circle.radius;
    case m2_capsuleShape:
    {
        m2Vec2 p1 = g->capsule.point1;
        m2Vec2 p2 = g->capsule.point2;
        float r = g->capsule.radius;
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float length = sqrtf(dx * dx + dy * dy);
        return 2.0f * r * length + M2_PI * r * r;
    }
    case m2_polygonShape:
    {
        float area2 = 0.0f;
        for (int32_t i = 0; i < g->polygon.count; ++i)
        {
            m2Vec2 a = g->polygon.vertices[i];
            m2Vec2 b = g->polygon.vertices[(i + 1) % g->polygon.count];
            area2 += a.x * b.y - b.x * a.y;
        }
        return 0.5f * (area2 < 0.0f ? -area2 : area2);
    }
    case m2_segmentShape:
    case m2_chainSegmentShape:
    {
        const m2Segment* seg = g->type == m2_segmentShape ? &g->segment : &g->chainSegment.segment;
        float dx = seg->point2.x - seg->point1.x;
        float dy = seg->point2.y - seg->point1.y;
        return 0.05f * sqrtf(dx * dx + dy * dy); // a thin sliver, nominal
    }
    default:
        return 0.0f;
    }
}
