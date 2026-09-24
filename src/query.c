// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World queries: closest ray cast and AABB
// overlap over the broadphase trees. Read-only by construction - no
// world state is touched, so a query storm between steps cannot move
// the simulation hash. Results are canonical: the ray tie-breaks equal
// fractions on the lower shape index, overlap lists sort ascending.
// The ray drops into each body's local frame through one f64->f32
// crossing, exactly like the narrowphase, so casts stay exact far from
// the origin.

#include "query.h"
#include "distance.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

m2QueryFilter m2DefaultQueryFilter(void)
{
    m2QueryFilter filter = {1u, 0xFFFFFFFFu};
    return filter;
}

// Ray vs one shape in the body frame: one f64 subtraction per body is
// the only precision crossing, same law as the narrowphase.
static m2CastHit RayCastShape(const m2World* world, int32_t shapeIndex, m2Pos2 origin, m2Vec2 d,
                              float maxFraction)
{
    int32_t body = world->shapes.shapeBody[shapeIndex];
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 rel = {(float)(origin.x - xf.p.x), (float)(origin.y - xf.p.y)};
    // Inverse-rotate into the body frame.
    m2Vec2 pLocal = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2Vec2 dLocal = {xf.q.c * d.x + xf.q.s * d.y, -xf.q.s * d.x + xf.q.c * d.y};
    m2CastHit hit =
        m2RayCastGeometry(&world->shapes.shapeGeometry[shapeIndex], pLocal, dLocal, maxFraction);
    if (hit.hit)
    {
        // Rotate the normal back out; the point is rebuilt in f64 by
        // the caller from the fraction.
        m2Vec2 n = hit.normal;
        hit.normal = (m2Vec2){xf.q.c * n.x - xf.q.s * n.y, xf.q.s * n.x + xf.q.c * n.y};
    }
    return hit;
}

typedef struct m2RayState
{
    m2Pos2 origin;
    m2Vec2 translation;
    m2QueryFilter filter;
    float fraction; // current best (starts at 1)
    int32_t shapeIndex;
    m2Vec2 normal;
    bool hit;
    bool initialOverlap;
} m2RayState;

// A tree node is skipped when the remaining ray segment misses its box:
// either box axis or the segment's normal separates them (the separating
// axis test for a segment and a box). Everything runs in f64, so worlds
// far from the origin cull exactly.
static bool RayMissesNode(const m2RayState* ray, m2Aabb box)
{
    double ax = ray->origin.x;
    double ay = ray->origin.y;
    double dx = (double)ray->fraction * (double)ray->translation.x;
    double dy = (double)ray->fraction * (double)ray->translation.y;
    double cx = 0.5 * (box.lowerBound.x + box.upperBound.x);
    double cy = 0.5 * (box.lowerBound.y + box.upperBound.y);
    double hx = 0.5 * (box.upperBound.x - box.lowerBound.x);
    double hy = 0.5 * (box.upperBound.y - box.lowerBound.y);
    // The segment's midpoint relative to the box center, and its half
    // extent.
    double mx = ax + 0.5 * dx - cx;
    double my = ay + 0.5 * dy - cy;
    double ex = 0.5 * fabs(dx);
    double ey = 0.5 * fabs(dy);
    if (fabs(mx) > hx + ex || fabs(my) > hy + ey)
    {
        return true;
    }
    return fabs(mx * dy - my * dx) > hx * fabs(dy) + hy * fabs(dx);
}

static void RayCastTree(const m2World* world, int32_t treeIndex, m2RayState* ray)
{
    const m2DynamicTree* tree = &world->broadphase.trees[treeIndex];
    const m2TreeNode* nodes = world->broadphase.treeNodes[treeIndex];
    int32_t stack[256];
    int32_t top = 0;
    if (tree->root != M2_NULL_NODE)
    {
        stack[top++] = tree->root;
    }
    while (top > 0)
    {
        int32_t index = stack[--top];
        if (RayMissesNode(ray, nodes[index].aabb))
        {
            continue;
        }
        if (nodes[index].height > 0)
        {
            M2_ASSERT(top + 2 <= 256);
            // Push child2 first so child1 is visited first: canonical.
            stack[top++] = nodes[index].child2;
            stack[top++] = nodes[index].child1;
            continue;
        }
        int32_t shapeIndex = nodes[index].userData;
        if (world->shapes.shapeAlive[shapeIndex] == 0 ||
            !m2QueryShouldSee(world, shapeIndex, ray->filter))
        {
            continue;
        }
        m2CastHit hit =
            RayCastShape(world, shapeIndex, ray->origin, ray->translation, ray->fraction);
        if (!hit.hit)
        {
            continue;
        }
        // Canonical winner: strictly closer, or same fraction and a
        // lower shape index.
        if (!ray->hit || hit.fraction < ray->fraction ||
            (hit.fraction == ray->fraction && shapeIndex < ray->shapeIndex))
        {
            ray->hit = true;
            ray->fraction = hit.fraction;
            ray->shapeIndex = shapeIndex;
            ray->normal = hit.normal;
            ray->initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
        }
    }
}

m2RayCastResult m2World_CastRayClosest(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation,
                                       m2QueryFilter filter)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = origin;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || !m2FinitePos2(origin) || !m2FiniteVec2(translation))
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }

    m2RayState ray;
    ray.origin = origin;
    ray.translation = translation;
    ray.filter = filter;
    ray.fraction = 1.0f;
    ray.shapeIndex = -1;
    ray.normal = (m2Vec2){0.0f, 0.0f};
    ray.hit = false;
    ray.initialOverlap = false;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        RayCastTree(world, t, &ray);
    }
    if (!ray.hit)
    {
        return result;
    }

    result.shapeId.index1 = ray.shapeIndex + 1;
    result.shapeId.world0 = worldId.index1;
    result.shapeId.generation = world->shapes.shapeGenerations[ray.shapeIndex];
    result.fraction = ray.initialOverlap ? 0.0f : ray.fraction;
    result.point = (m2Pos2){origin.x + (double)result.fraction * (double)translation.x,
                            origin.y + (double)result.fraction * (double)translation.y};
    result.normal = ray.normal;
    result.hit = true;
    return result;
}

// Keeps the `capacity` lowest shape slots offered, ascending, in the
// caller's result array, and counts every offer. A bounded max-heap on
// the slot index while collecting, sorted in place at the end: overlap
// queries need no buffer beyond the caller's and share nothing.
typedef struct ShapeSelection
{
    m2ShapeId* ids; // NULL when the caller only counts
    int32_t capacity;
    int32_t size;
    int32_t total;
} ShapeSelection;

static void SiftDown(m2ShapeId* heap, int32_t size, int32_t i)
{
    for (;;)
    {
        int32_t largest = i;
        int32_t left = 2 * i + 1;
        int32_t right = left + 1;
        if (left < size && heap[left].index1 > heap[largest].index1)
        {
            largest = left;
        }
        if (right < size && heap[right].index1 > heap[largest].index1)
        {
            largest = right;
        }
        if (largest == i)
        {
            return;
        }
        m2ShapeId swap = heap[i];
        heap[i] = heap[largest];
        heap[largest] = swap;
        i = largest;
    }
}

static void OfferShape(ShapeSelection* sel, const m2World* world, m2WorldId worldId,
                       int32_t shapeIndex)
{
    sel->total += 1;
    if (sel->ids == NULL || sel->capacity <= 0)
    {
        return;
    }
    m2ShapeId id = {shapeIndex + 1, worldId.index1, world->shapes.shapeGenerations[shapeIndex]};
    if (sel->size < sel->capacity)
    {
        // Sift up.
        int32_t i = sel->size++;
        sel->ids[i] = id;
        while (i > 0 && sel->ids[(i - 1) / 2].index1 < sel->ids[i].index1)
        {
            m2ShapeId swap = sel->ids[i];
            sel->ids[i] = sel->ids[(i - 1) / 2];
            sel->ids[(i - 1) / 2] = swap;
            i = (i - 1) / 2;
        }
    }
    else if (id.index1 < sel->ids[0].index1)
    {
        sel->ids[0] = id;
        SiftDown(sel->ids, sel->size, 0);
    }
}

// Sorts the kept ids ascending and returns the total offered.
static int32_t FinishSelection(ShapeSelection* sel)
{
    for (int32_t end = sel->size - 1; end > 0; --end)
    {
        m2ShapeId swap = sel->ids[0];
        sel->ids[0] = sel->ids[end];
        sel->ids[end] = swap;
        SiftDown(sel->ids, end, 0);
    }
    return sel->total;
}

int32_t m2World_OverlapAabb(m2WorldId worldId, m2Pos2 lower, m2Pos2 upper, m2ShapeId* results,
                            int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || !m2FinitePos2(lower) || !m2FinitePos2(upper) || upper.x < lower.x ||
        upper.y < lower.y)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }

    m2Aabb aabb = {lower, upper};
    ShapeSelection sel = {results, capacity, 0, 0};
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                         aabb);
        int32_t shapeIndex;
        while (m2TreeNextQuery(&cursor, &shapeIndex))
        {
            if (world->shapes.shapeAlive[shapeIndex] == 0 ||
                !m2QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            // Tight filter: the fat tree AABB over-reports.
            int32_t body = world->shapes.shapeBody[shapeIndex];
            m2Aabb tight = m2ComputeShapeAabb(&world->shapes.shapeGeometry[shapeIndex],
                                              world->bodies.transforms[body]);
            if (!m2Aabb_Overlaps(tight, aabb))
            {
                continue;
            }
            OfferShape(&sel, world, worldId, shapeIndex);
        }
    }
    return FinishSelection(&sel);
}

// ---------------------------------------------------------------
// Convex sweeps and overlaps. One generic
// walk serves circle, capsule and polygon: the cast geometry becomes
// a m2DistanceProxy in its own local frame, and per candidate shape
// both proxies meet in the TARGET's body-local frame (the same single
// f64 crossing as rays).

static m2RayCastResult CastProxyClosest(m2WorldId worldId, const m2DistanceProxy* castLocal,
                                        m2Transform pose, m2Vec2 translation, m2QueryFilter filter)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = pose.p;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || castLocal->count == 0 || !m2QueryMotionValid(pose, translation))
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    m2ProxyQuery q = m2MakeProxyQuery(castLocal, pose, translation);

    // Swept bounding circle in f64: exact culling far from the origin.
    double lox = q.pose.p.x - (double)q.boundRadius;
    double loy = q.pose.p.y - (double)q.boundRadius;
    double hix = q.pose.p.x + (double)q.boundRadius;
    double hiy = q.pose.p.y + (double)q.boundRadius;
    double tx = (double)translation.x;
    double ty = (double)translation.y;
    m2Aabb aabb;
    aabb.lowerBound.x = tx < 0.0 ? lox + tx : lox;
    aabb.lowerBound.y = ty < 0.0 ? loy + ty : loy;
    aabb.upperBound.x = tx > 0.0 ? hix + tx : hix;
    aabb.upperBound.y = ty > 0.0 ? hiy + ty : hiy;

    float bestFraction = 1.0f;
    int32_t bestShape = -1;
    m2Vec2 bestNormalLocal = {0.0f, 0.0f};
    m2Vec2 bestPointLocal = {0.0f, 0.0f};
    bool bestOverlap = false;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                         aabb);
        int32_t shapeIndex;
        while (m2TreeNextQuery(&cursor, &shapeIndex))
        {
            if (world->shapes.shapeAlive[shapeIndex] == 0 ||
                !m2QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            m2ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                                 &startLocal);
            if (m2ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2CastResult hit = m2ShapeCastProxy(&target, &cast, translationLocal, bestFraction);
            if (!hit.hit)
            {
                continue;
            }
            if (bestShape < 0 || hit.fraction < bestFraction ||
                (hit.fraction == bestFraction && shapeIndex < bestShape))
            {
                bestShape = shapeIndex;
                bestFraction = hit.fraction;
                bestNormalLocal = hit.normal;
                bestPointLocal = hit.pointA;
                bestOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            }
        }
    }
    if (bestShape < 0)
    {
        return result;
    }
    int32_t body = world->shapes.shapeBody[bestShape];
    m2Transform xf = world->bodies.transforms[body];
    result.shapeId.index1 = bestShape + 1;
    result.shapeId.world0 = worldId.index1;
    result.shapeId.generation = world->shapes.shapeGenerations[bestShape];
    result.fraction = bestOverlap ? 0.0f : bestFraction;
    result.hit = true;
    if (bestOverlap)
    {
        result.point = pose.p; // the ray convention: origin on overlap
        return result;
    }
    result.normal = (m2Vec2){xf.q.c * bestNormalLocal.x - xf.q.s * bestNormalLocal.y,
                             xf.q.s * bestNormalLocal.x + xf.q.c * bestNormalLocal.y};
    result.point =
        (m2Pos2){xf.p.x + (double)(xf.q.c * bestPointLocal.x - xf.q.s * bestPointLocal.y),
                 xf.p.y + (double)(xf.q.s * bestPointLocal.x + xf.q.c * bestPointLocal.y)};
    return result;
}

static int32_t OverlapProxy(m2WorldId worldId, const m2DistanceProxy* castLocal, m2Transform pose,
                            m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || castLocal->count == 0 || !m2QueryMotionValid(pose, (m2Vec2){0.0f, 0.0f}))
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    m2ProxyQuery q = m2MakeProxyQuery(castLocal, pose, (m2Vec2){0.0f, 0.0f});
    m2Aabb aabb;
    aabb.lowerBound.x = q.pose.p.x - (double)q.boundRadius;
    aabb.lowerBound.y = q.pose.p.y - (double)q.boundRadius;
    aabb.upperBound.x = q.pose.p.x + (double)q.boundRadius;
    aabb.upperBound.y = q.pose.p.y + (double)q.boundRadius;

    ShapeSelection sel = {ids, capacity, 0, 0};
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                         aabb);
        int32_t shapeIndex;
        while (m2TreeNextQuery(&cursor, &shapeIndex))
        {
            if (world->shapes.shapeAlive[shapeIndex] == 0 ||
                !m2QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            m2ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                                 &startLocal);
            if (m2ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2DistanceResult d = m2ShapeDistance(&target, &cast);
            // Touching within the engine's linear slop skin counts as
            // overlap: the same tolerance the solver's speculative
            // margin uses, and it absorbs GJK witness noise on deeply
            // contained pairs.
            if (d.distance - target.radius - cast.radius > 0.005f)
            {
                continue;
            }
            OfferShape(&sel, world, worldId, shapeIndex);
        }
    }
    return FinishSelection(&sel);
}

m2RayCastResult m2World_CastCircleClosest(m2WorldId worldId, const m2Circle* circle,
                                          m2Transform origin, m2Vec2 translation,
                                          m2QueryFilter filter)
{
    m2DistanceProxy p = m2CircleProxy(circle);
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

m2RayCastResult m2World_CastCapsuleClosest(m2WorldId worldId, const m2Capsule* capsule,
                                           m2Transform origin, m2Vec2 translation,
                                           m2QueryFilter filter)
{
    m2DistanceProxy p = m2CapsuleProxy(capsule);
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

m2RayCastResult m2World_CastPolygonClosest(m2WorldId worldId, const m2Polygon* polygon,
                                           m2Transform origin, m2Vec2 translation,
                                           m2QueryFilter filter)
{
    m2DistanceProxy p = m2PolygonProxy(polygon);
    return CastProxyClosest(worldId, &p, origin, translation, filter);
}

int32_t m2World_OverlapCircle(m2WorldId worldId, const m2Circle* circle, m2Transform origin,
                              m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p = m2CircleProxy(circle);
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

int32_t m2World_OverlapCapsule(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin,
                               m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p = m2CapsuleProxy(capsule);
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

int32_t m2World_OverlapPolygon(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin,
                               m2ShapeId* ids, int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy p = m2PolygonProxy(polygon);
    return OverlapProxy(worldId, &p, origin, ids, capacity, filter);
}

// One shape, the world ray conventions, the one-sided chain law: the
// The particle projection pass borrows the per-shape kernel (chain
// one-sided law included) without the tree walk.
struct m2CastHitInternal m2RayCastShapeIndex(const m2World* world, int32_t shapeIndex,
                                             m2Pos2 origin, m2Vec2 translation, float maxFraction)
{
    m2CastHit hit = RayCastShape(world, shapeIndex, origin, translation, maxFraction);
    struct m2CastHitInternal out;
    out.point = hit.point;
    out.normal = hit.normal;
    out.fraction = hit.fraction;
    out.hit = hit.hit;
    return out;
}

// same RayCastShape the world walk uses, minus the walk.
m2RayCastResult m2Shape_CastRay(m2ShapeId shapeId, m2Pos2 origin, m2Vec2 translation)
{
    m2RayCastResult result;
    result.shapeId = m2_nullShapeId;
    result.point = origin;
    result.normal = (m2Vec2){0.0f, 0.0f};
    result.fraction = 0.0f;
    result.hit = false;

    m2World* world = m2WorldFromIndex0(shapeId.world0);
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity || world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation || !m2FinitePos2(origin) ||
        !m2FiniteVec2(translation))
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    m2CastHit hit = RayCastShape(world, index, origin, translation, 1.0f);
    if (!hit.hit)
    {
        return result;
    }
    result.shapeId = shapeId;
    result.hit = true;
    bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
    result.fraction = initialOverlap ? 0.0f : hit.fraction;
    result.normal = hit.normal;
    result.point = (m2Pos2){origin.x + (double)result.fraction * (double)translation.x,
                            origin.y + (double)result.fraction * (double)translation.y};
    return result;
}

// -------------------------------------------------------- all-hits
// Bounded keep-the-closest insertion: candidates stream in, the hits
// array holds the best `capacity` in ascending (fraction, shape slot)
// order, and the true total keeps counting past it. The slot tie-break
// reads the ids already stored, so no side buffer caps the capacity.
static int32_t InsertHitSorted(m2RayHit* hits, int32_t kept, int32_t capacity, m2RayHit candidate)
{
    int32_t at = kept;
    while (at > 0)
    {
        bool after = hits[at - 1].fraction < candidate.fraction ||
                     (hits[at - 1].fraction == candidate.fraction &&
                      hits[at - 1].shapeId.index1 < candidate.shapeId.index1);
        if (after)
        {
            break;
        }
        at -= 1;
    }
    if (at >= capacity)
    {
        return kept; // worse than everything kept
    }
    int32_t last = kept < capacity ? kept : capacity - 1;
    for (int32_t i = last; i > at; --i)
    {
        hits[i] = hits[i - 1];
    }
    hits[at] = candidate;
    return kept < capacity ? kept + 1 : kept;
}

int32_t m2World_CastRayAll(m2WorldId worldId, m2Pos2 origin, m2Vec2 translation, m2RayHit* hits,
                           int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || !m2FinitePos2(origin) || !m2FiniteVec2(translation))
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t cap = hits != NULL && capacity > 0 ? capacity : 0;
    int32_t kept = 0;
    int32_t total = 0;

    m2RayState ray;
    ray.origin = origin;
    ray.translation = translation;
    ray.filter = filter;
    ray.fraction = 1.0f; // no pruning: every hit counts
    ray.shapeIndex = -1;
    ray.normal = (m2Vec2){0.0f, 0.0f};
    ray.hit = false;
    ray.initialOverlap = false;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        const m2DynamicTree* tree = &world->broadphase.trees[t];
        const m2TreeNode* nodes = world->broadphase.treeNodes[t];
        int32_t stack[256];
        int32_t top = 0;
        if (tree->root != M2_NULL_NODE)
        {
            stack[top++] = tree->root;
        }
        while (top > 0)
        {
            int32_t index = stack[--top];
            if (RayMissesNode(&ray, nodes[index].aabb))
            {
                continue;
            }
            if (nodes[index].height > 0)
            {
                M2_ASSERT(top + 2 <= 256);
                stack[top++] = nodes[index].child2;
                stack[top++] = nodes[index].child1;
                continue;
            }
            int32_t shapeIndex = nodes[index].userData;
            if (world->shapes.shapeAlive[shapeIndex] == 0 ||
                !m2QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2CastHit hit = RayCastShape(world, shapeIndex, origin, translation, 1.0f);
            if (!hit.hit)
            {
                continue;
            }
            bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            m2RayHit out;
            out.shapeId.index1 = shapeIndex + 1;
            out.shapeId.world0 = worldId.index1;
            out.shapeId.generation = world->shapes.shapeGenerations[shapeIndex];
            out.fraction = initialOverlap ? 0.0f : hit.fraction;
            out.normal = hit.normal;
            out.point = (m2Pos2){origin.x + (double)out.fraction * (double)translation.x,
                                 origin.y + (double)out.fraction * (double)translation.y};
            total += 1;
            if (cap > 0)
            {
                kept = InsertHitSorted(hits, kept, cap, out);
            }
        }
    }
    return total;
}

static int32_t CastProxyAll(m2WorldId worldId, const m2DistanceProxy* castLocal, m2Transform pose,
                            m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                            m2QueryFilter filter)
{
    m2World* world = m2WorldFromId(worldId);
    if (world == NULL || castLocal->count == 0 || !m2QueryMotionValid(pose, translation))
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    m2ProxyQuery q = m2MakeProxyQuery(castLocal, pose, translation);

    double lox = q.pose.p.x - (double)q.boundRadius;
    double loy = q.pose.p.y - (double)q.boundRadius;
    double hix = q.pose.p.x + (double)q.boundRadius;
    double hiy = q.pose.p.y + (double)q.boundRadius;
    double tx = (double)translation.x;
    double ty = (double)translation.y;
    m2Aabb aabb;
    aabb.lowerBound.x = tx < 0.0 ? lox + tx : lox;
    aabb.lowerBound.y = ty < 0.0 ? loy + ty : loy;
    aabb.upperBound.x = tx > 0.0 ? hix + tx : hix;
    aabb.upperBound.y = ty > 0.0 ? hiy + ty : hiy;

    int32_t cap = hits != NULL && capacity > 0 ? capacity : 0;
    int32_t kept = 0;
    int32_t total = 0;

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                         aabb);
        int32_t shapeIndex;
        while (m2TreeNextQuery(&cursor, &shapeIndex))
        {
            if (world->shapes.shapeAlive[shapeIndex] == 0 ||
                !m2QueryShouldSee(world, shapeIndex, filter))
            {
                continue;
            }
            m2DistanceProxy target;
            m2DistanceProxy cast;
            m2Vec2 translationLocal;
            m2Vec2 startLocal;
            m2ProxiesInBodyFrame(world, shapeIndex, &q, &target, &cast, &translationLocal,
                                 &startLocal);
            if (m2ChainGhostSide(world, shapeIndex, startLocal))
            {
                continue;
            }
            m2CastResult hit = m2ShapeCastProxy(&target, &cast, translationLocal, 1.0f);
            if (!hit.hit)
            {
                continue;
            }
            int32_t body = world->shapes.shapeBody[shapeIndex];
            m2Transform xf = world->bodies.transforms[body];
            bool initialOverlap = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            m2RayHit out;
            out.shapeId.index1 = shapeIndex + 1;
            out.shapeId.world0 = worldId.index1;
            out.shapeId.generation = world->shapes.shapeGenerations[shapeIndex];
            out.fraction = initialOverlap ? 0.0f : hit.fraction;
            if (initialOverlap)
            {
                out.normal = (m2Vec2){0.0f, 0.0f};
                out.point = pose.p;
            }
            else
            {
                out.normal = (m2Vec2){xf.q.c * hit.normal.x - xf.q.s * hit.normal.y,
                                      xf.q.s * hit.normal.x + xf.q.c * hit.normal.y};
                out.point =
                    (m2Pos2){xf.p.x + (double)(xf.q.c * hit.pointA.x - xf.q.s * hit.pointA.y),
                             xf.p.y + (double)(xf.q.s * hit.pointA.x + xf.q.c * hit.pointA.y)};
            }
            total += 1;
            if (cap > 0)
            {
                kept = InsertHitSorted(hits, kept, cap, out);
            }
        }
    }
    return total;
}

int32_t m2World_CastCircleAll(m2WorldId worldId, const m2Circle* circle, m2Transform origin,
                              m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                              m2QueryFilter filter)
{
    m2DistanceProxy p = m2CircleProxy(circle);
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}

int32_t m2World_CastCapsuleAll(m2WorldId worldId, const m2Capsule* capsule, m2Transform origin,
                               m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                               m2QueryFilter filter)
{
    m2DistanceProxy p = m2CapsuleProxy(capsule);
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}

int32_t m2World_CastPolygonAll(m2WorldId worldId, const m2Polygon* polygon, m2Transform origin,
                               m2Vec2 translation, m2RayHit* hits, int32_t capacity,
                               m2QueryFilter filter)
{
    m2DistanceProxy p = m2PolygonProxy(polygon);
    return CastProxyAll(worldId, &p, origin, translation, hits, capacity, filter);
}
