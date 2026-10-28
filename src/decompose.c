// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Convex decomposition of a caller's outline into polygons.

#include "core.h"
#include "fpenv.h"
#include "geometry.h"

#include "maul2d/base.h"
#include "maul2d/core_math.h"

// --- Convex decomposition -------------------------------------------------------
//
// Ear clipping into triangles, then Hertel-Mehlhorn style merging:
// pieces fuse across a shared edge whenever the union stays strictly
// convex and inside the 8-vertex polygon limit. Every scan runs in
// ascending index order and every pass picks the lowest-index
// candidate, so the decomposition is canonical: same outline, same
// pieces, on every platform.

typedef struct m2DecompPiece
{
    int32_t idx[M2_MAX_POLYGON_VERTICES];
    int32_t n;
} m2DecompPiece;

// The predicates work in double: the points are within M2_LOCAL_LIMIT, so every
// difference and product is far from overflow, and a near-zero turn keeps
// its sign where float would round it away.
static double DecompCross(m2Vec2 a, m2Vec2 b, m2Vec2 c)
{
    double abx = (double)b.x - (double)a.x;
    double aby = (double)b.y - (double)a.y;
    double acx = (double)c.x - (double)a.x;
    double acy = (double)c.y - (double)a.y;
    return abx * acy - aby * acx;
}

// Inclusive point-in-triangle for a CCW triangle: boundary counts as
// inside, which makes the ear test conservative.
static bool DecompPointInTriangle(m2Vec2 a, m2Vec2 b, m2Vec2 c, m2Vec2 p)
{
    return DecompCross(a, b, p) >= 0.0 && DecompCross(b, c, p) >= 0.0 &&
           DecompCross(c, a, p) >= 0.0;
}

// Whether p, collinear with segment ab, lies on it, endpoints included.
static bool DecompOnSegment(m2Vec2 a, m2Vec2 b, m2Vec2 p)
{
    return (a.x <= b.x ? a.x <= p.x && p.x <= b.x : b.x <= p.x && p.x <= a.x) &&
           (a.y <= b.y ? a.y <= p.y && p.y <= b.y : b.y <= p.y && p.y <= a.y);
}

// Whether segments ab and cd meet: they cross, or one's endpoint lies on
// the other (a vertex on an edge, a repeated vertex, a collinear
// overlap). An outline whose edges meet anywhere but at their shared
// corners is not simple, and its pieces could cover what it cuts out.
static bool DecompSegmentsMeet(m2Vec2 a, m2Vec2 b, m2Vec2 c, m2Vec2 d)
{
    double d1 = DecompCross(c, d, a);
    double d2 = DecompCross(c, d, b);
    double d3 = DecompCross(a, b, c);
    double d4 = DecompCross(a, b, d);
    if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
        ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)))
    {
        return true;
    }
    return (d1 == 0.0 && DecompOnSegment(c, d, a)) || (d2 == 0.0 && DecompOnSegment(c, d, b)) ||
           (d3 == 0.0 && DecompOnSegment(a, b, c)) || (d4 == 0.0 && DecompOnSegment(a, b, d));
}

// Whether the corner at b folds the outline straight back over the edge
// it arrived on: a spike of zero width. The edge test alone would catch a
// fold through a different cross product when the fold overshoots, and
// rounding can tell the two apart, so the corner is tested exactly here.
static bool DecompFoldsBack(m2Vec2 a, m2Vec2 b, m2Vec2 c)
{
    double dot = ((double)b.x - (double)a.x) * ((double)c.x - (double)b.x) +
                 ((double)b.y - (double)a.y) * ((double)c.y - (double)b.y);
    return DecompCross(a, b, c) == 0.0 && dot < 0.0;
}

// Whether p lies within M2_LINEAR_SLOP of segment ab. Closer than that,
// a vertex touches the edge for the solver, and float rounding in the
// pieces can make them cross it.
static bool DecompNear(m2Vec2 a, m2Vec2 b, m2Vec2 p)
{
    double ex = (double)b.x - (double)a.x;
    double ey = (double)b.y - (double)a.y;
    double px = (double)p.x - (double)a.x;
    double py = (double)p.y - (double)a.y;
    double along = px * ex + py * ey;
    double length2 = ex * ex + ey * ey;
    if (along > 0.0 && length2 > 0.0)
    {
        double t = along < length2 ? along / length2 : 1.0;
        px -= t * ex;
        py -= t * ey;
    }
    double slop = (double)M2_LINEAR_SLOP;
    return px * px + py * py < slop * slop;
}

// Whether an outline is simple: no corner folds back, no two edges meet
// except adjacent ones at their shared corner, and no corner comes within
// M2_LINEAR_SLOP of an edge it does not end.
static bool DecompSimple(const m2Vec2* points, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        if (DecompFoldsBack(points[(i + count - 1) % count], points[i], points[(i + 1) % count]))
        {
            return false;
        }
        for (int32_t j = 0; j < count; ++j)
        {
            if (j != i && (j + 1) % count != i &&
                DecompNear(points[j], points[(j + 1) % count], points[i]))
            {
                return false;
            }
        }
        for (int32_t j = i + 1; j < count; ++j)
        {
            // Skip adjacent segments (they share an endpoint).
            if ((j + 1) % count == i || (i + 1) % count == j)
            {
                continue;
            }
            if (DecompSegmentsMeet(points[i], points[(i + 1) % count], points[j],
                                   points[(j + 1) % count]))
            {
                return false;
            }
        }
    }
    return true;
}

// The decomposition behind m2DecomposeOutline: the true piece total, or
// -1 for an outline it refuses.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): one ring: clip ears, merge, validate
static int32_t Decompose(const m2Vec2* points, int32_t count, m2Polygon* pieces, int32_t capacity)
{
    if (points == NULL || count < 3 || count > M2_MAX_OUTLINE_POINTS || capacity < 0)
    {
        return -1;
    }
    // Summed in double, where the products of floats are exact: in float, a
    // small clockwise outline far from the origin can sum to a positive area.
    double area2 = 0.0;
    for (int32_t i = 0; i < count; ++i)
    {
        m2Vec2 p = points[i];
        if (!m2LocalVec2(p))
        {
            return -1;
        }
        m2Vec2 q = points[(i + 1) % count];
        area2 += (double)p.x * (double)q.y - (double)q.x * (double)p.y;
    }
    if (!(area2 > 0.0))
    {
        return -1; // clockwise or degenerate outline
    }
    if (!DecompSimple(points, count))
    {
        return -1;
    }

    // Ear clipping. The ring holds original indices; each pass clips
    // the valid ear with the lowest original index.
    int32_t ring[M2_MAX_OUTLINE_POINTS];
    int32_t ringCount = count;
    for (int32_t i = 0; i < count; ++i)
    {
        ring[i] = i;
    }
    int32_t triangles[3 * (M2_MAX_OUTLINE_POINTS - 2)];
    int32_t triangleCount = 0;
    while (ringCount > 3)
    {
        int32_t bestPos = -1;
        int32_t bestIndex = M2_MAX_OUTLINE_POINTS;
        bool degenerate = false;
        for (int32_t k = 0; k < ringCount; ++k)
        {
            int32_t ip = ring[(k + ringCount - 1) % ringCount];
            int32_t ic = ring[k];
            int32_t in = ring[(k + 1) % ringCount];
            double cross = DecompCross(points[ip], points[ic], points[in]);
            if (cross == 0.0)
            {
                // A straight vertex clips for free, no triangle.
                bestPos = k;
                degenerate = true;
                break;
            }
            if (cross < 0.0)
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
            return -1; // no ear: numerically hostile outline
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
    double lastCross = DecompCross(points[ring[0]], points[ring[1]], points[ring[2]]);
    if (lastCross > 0.0)
    {
        triangles[3 * triangleCount + 0] = ring[0];
        triangles[3 * triangleCount + 1] = ring[1];
        triangles[3 * triangleCount + 2] = ring[2];
        triangleCount += 1;
    }

    // Merge pass: fuse two pieces across a shared edge whenever the
    // union stays strictly convex and at most 8 vertices. Ascending
    // pair order, restart after every fuse: canonical.
    m2DecompPiece work[M2_MAX_OUTLINE_POINTS - 2];
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
                        convex = DecompCross(a, b, c) > 0.0;
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
    // that validation rejects are skipped (documented). An outline that
    // leaves no piece is refused, not accepted as empty.
    int32_t total = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2Vec2 verts[M2_MAX_POLYGON_VERTICES];
        for (int32_t k = 0; k < work[i].n; ++k)
        {
            verts[k] = points[work[i].idx[k]];
        }
        m2Polygon piece = m2BuildPolygon(verts, work[i].n, 0.0f);
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
    return total > 0 ? total : -1;
}

// The outline's bounds: past M2_MAX_OUTLINE_POINTS points or a point
// past M2_LOCAL_LIMIT is m2_errorLimit, a NaN m2_errorInvalid.
static m2Result OutlineBoundsCheck(const m2Vec2* points, int32_t count)
{
    if (points == NULL || count < 3)
    {
        return m2_errorInvalid;
    }
    m2Result result = m2CheckCount(count, M2_MAX_OUTLINE_POINTS);
    for (int32_t i = 0; count <= M2_MAX_OUTLINE_POINTS && i < count; ++i)
    {
        result = m2BothChecks(result, m2CheckLocalVec2(points[i]));
    }
    return result;
}

static M2_NOINLINE m2Result DecomposeOutlineWorker(const m2Vec2* points, int32_t count,
                                                   m2Polygon* pieces, int32_t capacity,
                                                   int32_t* countOut)
{
    if (countOut == NULL)
    {
        return m2_errorInvalid;
    }
    m2Result bounds = OutlineBoundsCheck(points, count);
    if (bounds != m2_success)
    {
        return bounds;
    }
    int32_t total = Decompose(points, count, pieces, capacity);
    if (total < 0)
    {
        return m2_errorInvalid;
    }
    *countOut = total;
    return m2_success;
}

m2Result m2DecomposeOutline(const m2Vec2* points, int32_t count, m2Polygon* pieces,
                            int32_t capacity, int32_t* countOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = DecomposeOutlineWorker(points, count, pieces, capacity, countOut);
    m2FloatEnvLeave(env);
    return result;
}

// Area of a shape's core, used by the buoyancy fallback for capsules
// and thin shapes (circles and polygons get exact submersion).
