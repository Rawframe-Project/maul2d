// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover kit: collision planes for a posed capsule, the closest move
// they allow, and velocity clipping.
//
// The move is the translation d closest to the wish w that keeps every
// plane clear, n_i . d + s_i >= 0: a small strictly convex problem, so
// its optimum is unique. At the optimum d = w + sum lambda_j n_j over the
// planes it rests on, each with lambda_j >= 0 (a plane pushes, never
// pulls), and in 2D it rests on at most two. The candidate rest sets are
// tried smallest first, in plane order; the first that clears every
// plane with pushes that are not negative is the optimum. Planes that
// contradict each other leave no such point, and the candidate that
// overlaps least is taken instead.

#include "core.h"
#include "fpenv.h"
#include "query.h"

#include "maul2d/base.h"

#include <math.h>

// Keeps the `capacity` planes of the lowest shape slots offered, as a
// max-heap on the slot while collecting, so which planes a short array
// holds depends only on which shapes touch, never on the order the trees
// yield them. SortPlanes then lays them out ascending.
static void SiftPlaneDown(m2MoverPlane* heap, int32_t size, int32_t i)
{
    for (;;)
    {
        int32_t largest = i;
        int32_t left = 2 * i + 1;
        int32_t right = left + 1;
        if (left < size && heap[left].shapeId.index1 > heap[largest].shapeId.index1)
        {
            largest = left;
        }
        if (right < size && heap[right].shapeId.index1 > heap[largest].shapeId.index1)
        {
            largest = right;
        }
        if (largest == i)
        {
            return;
        }
        m2MoverPlane swap = heap[i];
        heap[i] = heap[largest];
        heap[largest] = swap;
        i = largest;
    }
}

static void OfferPlane(m2MoverPlane* heap, int32_t* size, int32_t capacity, m2MoverPlane plane)
{
    if (*size < capacity)
    {
        int32_t i = (*size)++;
        heap[i] = plane;
        while (i > 0 && heap[(i - 1) / 2].shapeId.index1 < heap[i].shapeId.index1)
        {
            m2MoverPlane swap = heap[i];
            heap[i] = heap[(i - 1) / 2];
            heap[(i - 1) / 2] = swap;
            i = (i - 1) / 2;
        }
    }
    else if (capacity > 0 && plane.shapeId.index1 < heap[0].shapeId.index1)
    {
        heap[0] = plane;
        SiftPlaneDown(heap, *size, 0);
    }
}

static void SortPlanes(m2MoverPlane* heap, int32_t size)
{
    for (int32_t end = size - 1; end > 0; --end)
    {
        m2MoverPlane swap = heap[0];
        heap[0] = heap[end];
        heap[end] = swap;
        SiftPlaneDown(heap, end, 0);
    }
}

// Planes for a posed capsule: one distance query per nearby shape. A
// collar of four linear slops lets a controller see a wall before it
// reaches it.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): one tree walk, a plane per shape
static M2_NOINLINE int32_t WorldCollideMoverWorker(const m2World* world, const m2Capsule* mover,
                                                   m2Transform origin, m2MoverPlane* results,
                                                   int32_t capacity, m2QueryFilter filter)
{
    m2DistanceProxy moverLocal = m2CapsuleProxy(mover);
    if (world == NULL || !m2ListOutputValid(results, capacity) || moverLocal.count == 0 ||
        !m2QueryMotionValid(origin, (m2Vec2){0.0f, 0.0f}))
    {
        return m2RefuseList(world, results, capacity, sizeof(*results));
    }
    m2ProxyQuery q = m2MakeProxyQuery(&moverLocal, origin, (m2Vec2){0.0f, 0.0f});

    float collar = 0.02f; // 4x linear slop, the speculative margin
    m2Aabb aabb;
    aabb.lowerBound.x = q.pose.p.x - (double)(q.boundRadius + collar);
    aabb.lowerBound.y = q.pose.p.y - (double)(q.boundRadius + collar);
    aabb.upperBound.x = q.pose.p.x + (double)(q.boundRadius + collar);
    aabb.upperBound.y = q.pose.p.y + (double)(q.boundRadius + collar);

    int32_t total = 0;
    int32_t kept = 0;
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
            float separation = d.distance - target.radius - cast.radius;
            if (separation > collar)
            {
                continue;
            }
            // Deep overlap loses the normal; fall back to pushing the
            // mover toward its own pose origin side, or skip if even
            // that is degenerate (dead-centered).
            m2Vec2 normalLocal = d.normal;
            if (d.distance <= 0.0f && normalLocal.x == 0.0f && normalLocal.y == 0.0f)
            {
                float len = sqrtf(startLocal.x * startLocal.x + startLocal.y * startLocal.y);
                if (!(len > 0.0f))
                {
                    continue;
                }
                normalLocal = (m2Vec2){startLocal.x / len, startLocal.y / len};
            }
            int32_t body = world->shapes.shapeBody[shapeIndex];
            m2Transform xf = world->bodies.transforms[body];
            m2Vec2 surf = {d.pointA.x + target.radius * normalLocal.x,
                           d.pointA.y + target.radius * normalLocal.y};
            if (results != NULL)
            {
                m2MoverPlane plane;
                plane.shapeId.index1 = (uint32_t)(shapeIndex + 1);
                plane.shapeId.generation = world->shapes.shapeGenerations[shapeIndex];
                plane.normal = (m2Vec2){xf.q.c * normalLocal.x - xf.q.s * normalLocal.y,
                                        xf.q.s * normalLocal.x + xf.q.c * normalLocal.y};
                plane.separation = separation;
                plane.point = (m2Pos2){xf.p.x + (double)(xf.q.c * surf.x - xf.q.s * surf.y),
                                       xf.p.y + (double)(xf.q.s * surf.x + xf.q.c * surf.y)};
                OfferPlane(results, &kept, capacity, plane);
            }
            total += 1;
        }
    }
    SortPlanes(results, kept); // ascending shape order (canonical)
    return total;
}

int32_t m2World_CollideMover(const m2World* world, const m2Capsule* mover, m2Transform origin,
                             m2QueryFilter filter, m2MoverPlane* planesOut, int32_t capacity)
{
    uint64_t env = m2FloatEnvEnter();
    int32_t result = WorldCollideMoverWorker(world, mover, origin, planesOut, capacity, filter);
    m2FloatEnvLeave(env);
    return result;
}

// How far a move may stand inside a plane and still count as clear.
#define M2_MOVER_TOLERANCE 1.0e-5f

typedef struct Candidate
{
    m2Vec2 d;
    float clearance; // the smallest n . d + s over all planes
    uint32_t pressed;
} Candidate;

static float Dot(m2Vec2 a, m2Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

// The point closest to the wish on the planes of set (k of them),
// pushed by each: solves the Gram system G lambda = -(s + N w). False
// when the set is degenerate or a push would pull.
M2_NODISCARD static bool RestOn(m2Vec2 wish, const m2MoverPlane* planes, const int32_t* set,
                                int32_t k, m2Vec2* d, uint32_t* pressed)
{
    float lambda[2] = {0.0f, 0.0f};
    if (k == 1)
    {
        m2Vec2 n = planes[set[0]].normal;
        lambda[0] = -(planes[set[0]].separation + Dot(n, wish)) / Dot(n, n);
    }
    else if (k == 2)
    {
        m2Vec2 n0 = planes[set[0]].normal;
        m2Vec2 n1 = planes[set[1]].normal;
        float g00 = Dot(n0, n0);
        float g01 = Dot(n0, n1);
        float g11 = Dot(n1, n1);
        float det = g00 * g11 - g01 * g01;
        if (!(det > 1.0e-10f))
        {
            return false;
        }
        float r0 = -(planes[set[0]].separation + Dot(n0, wish));
        float r1 = -(planes[set[1]].separation + Dot(n1, wish));
        lambda[0] = (g11 * r0 - g01 * r1) / det;
        lambda[1] = (g00 * r1 - g01 * r0) / det;
    }
    *d = wish;
    *pressed = 0;
    for (int32_t j = 0; j < k && j < 2; ++j)
    {
        if (!(lambda[j] >= 0.0f))
        {
            return false;
        }
        m2Vec2 n = planes[set[j]].normal;
        d->x += lambda[j] * n.x;
        d->y += lambda[j] * n.y;
        *pressed |= lambda[j] > 0.0f ? 1u << set[j] : 0u;
    }
    return true;
}

// Keeps the better of two candidates: a clear one over an overlapping
// one, the earlier clear one, or the one that overlaps less.
static void Consider(m2Vec2 wish, const m2MoverPlane* planes, int32_t count, const int32_t* set,
                     int32_t k, Candidate* best)
{
    Candidate c;
    if (!RestOn(wish, planes, set, k, &c.d, &c.pressed))
    {
        return;
    }
    c.clearance = 3.4e38f;
    for (int32_t i = 0; i < count; ++i)
    {
        c.clearance = m2MinF(c.clearance, Dot(planes[i].normal, c.d) + planes[i].separation);
    }
    bool bestClear = best->clearance >= -M2_MOVER_TOLERANCE;
    if (!bestClear && c.clearance > best->clearance)
    {
        *best = c;
    }
}

static M2_NOINLINE m2MoverMove SolveMoverWorker(m2Vec2 wish, const m2MoverPlane* planes,
                                                int32_t count)
{
    m2MoverMove move = {wish, 0};
    if (planes == NULL || count <= 0 || !m2FiniteVec2(wish))
    {
        return move;
    }
    count = count < M2_MAX_MOVER_PLANES ? count : M2_MAX_MOVER_PLANES;
    Candidate best = {wish, -3.4e38f, 0};
    int32_t set[2] = {0, 0};
    Consider(wish, planes, count, set, 0, &best);
    for (int32_t a = 0; a < count; ++a)
    {
        set[0] = a;
        Consider(wish, planes, count, set, 1, &best);
    }
    for (int32_t a = 0; a < count; ++a)
    {
        for (int32_t b = a + 1; b < count; ++b)
        {
            set[0] = a;
            set[1] = b;
            Consider(wish, planes, count, set, 2, &best);
        }
    }
    move.translation = best.d;
    move.pressed = best.pressed;
    return move;
}

m2MoverMove m2SolveMover(m2Vec2 wish, const m2MoverPlane* planes, int32_t count)
{
    uint64_t env = m2FloatEnvEnter();
    m2MoverMove result = SolveMoverWorker(wish, planes, count);
    m2FloatEnvLeave(env);
    return result;
}

// The velocity closest to the given one that no pressed plane opposes:
// the same problem with the pressed planes at zero separation.
static M2_NOINLINE m2Vec2 ClipMoverVelocityWorker(m2Vec2 velocity, const m2MoverPlane* planes,
                                                  int32_t count, uint32_t pressed)
{
    if (planes == NULL || count <= 0 || pressed == 0)
    {
        return velocity;
    }
    m2MoverPlane touching[M2_MAX_MOVER_PLANES];
    int32_t n = 0;
    for (int32_t i = 0; i < count && i < M2_MAX_MOVER_PLANES; ++i)
    {
        if ((pressed & (1u << i)) != 0)
        {
            touching[n] = planes[i];
            touching[n].separation = 0.0f;
            n += 1;
        }
    }
    return m2SolveMover(velocity, touching, n).translation;
}

m2Vec2 m2ClipMoverVelocity(m2Vec2 velocity, const m2MoverPlane* planes, int32_t count,
                           uint32_t pressed)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = ClipMoverVelocityWorker(velocity, planes, count, pressed);
    m2FloatEnvLeave(env);
    return result;
}
