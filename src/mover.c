// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mover kit: collision planes for a posed capsule, the plane solver
// and velocity clipping.

#include "query.h"
#include "world.h"

#include "maul2d/base.h"

#include <math.h>

// Collision planes for a posed capsule mover: one GJK query per
// nearby shape, plane normal from the shape toward the mover,
// separation measured along it (negative = penetration), the
// speculative collar included so a controller sees walls before it
// clips them. Reference architecture (mover.c), Maul frames.
int32_t m2World_CollideMover(m2WorldId worldId, const m2Capsule* mover, m2Transform origin,
                             m2PlaneResult* results, int32_t capacity, m2QueryFilter filter)
{
    m2World* world = m2WorldFromId(worldId);
    m2DistanceProxy moverLocal = m2CapsuleProxy(mover);
    if (world == NULL || moverLocal.count == 0 || !m2QueryMotionValid(origin, (m2Vec2){0.0f, 0.0f}))
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    m2ProxyQuery q = m2MakeProxyQuery(&moverLocal, origin, (m2Vec2){0.0f, 0.0f});

    float collar = 0.02f; // 4x linear slop, the speculative margin
    m2AABB aabb;
    aabb.lowerBound.x = q.pose.p.x - (double)(q.boundRadius + collar);
    aabb.lowerBound.y = q.pose.p.y - (double)(q.boundRadius + collar);
    aabb.upperBound.x = q.pose.p.x + (double)(q.boundRadius + collar);
    aabb.upperBound.y = q.pose.p.y + (double)(q.boundRadius + collar);

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
            if (results != NULL && total < capacity)
            {
                m2PlaneResult* out = results + total;
                out->shapeId.index1 = shapeIndex + 1;
                out->shapeId.world0 = worldId.index1;
                out->shapeId.generation = world->shapes.shapeGenerations[shapeIndex];
                out->normal = (m2Vec2){xf.q.c * normalLocal.x - xf.q.s * normalLocal.y,
                                       xf.q.s * normalLocal.x + xf.q.c * normalLocal.y};
                out->separation = separation;
                out->point = (m2Pos2){xf.p.x + (double)(xf.q.c * surf.x - xf.q.s * surf.y),
                                      xf.p.y + (double)(xf.q.s * surf.x + xf.q.c * surf.y)};
            }
            total += 1;
        }
    }
    // Ascending shape order for the filled portion (canonical).
    int32_t filled = results != NULL ? (total < capacity ? total : capacity) : 0;
    for (int32_t i = 1; i < filled; ++i)
    {
        m2PlaneResult key = results[i];
        int32_t j = i - 1;
        while (j >= 0 && results[j].shapeId.index1 > key.shapeId.index1)
        {
            results[j + 1] = results[j];
            j -= 1;
        }
        results[j + 1] = key;
    }
    return total;
}

// The plane solver: iterate the planes,
// push the delta out along each normal with a clamped accumulator,
// stop when the total push falls under the slop tolerance.
m2PlaneSolverResult m2SolvePlanes(m2Vec2 targetDelta, m2CollisionPlane* planes, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        planes[i].push = 0.0f;
    }
    m2Vec2 delta = targetDelta;
    float tolerance = 0.005f; // linear slop

    int32_t iteration = 0;
    for (; iteration < 20; ++iteration)
    {
        float totalPush = 0.0f;
        for (int32_t i = 0; i < count; ++i)
        {
            m2CollisionPlane* plane = planes + i;
            // Separation of the moved mover from this plane, slopped
            // to prevent jitter.
            float separation =
                plane->separation + delta.x * plane->normal.x + delta.y * plane->normal.y + 0.005f;
            float push = -separation;
            float accumulated = plane->push;
            float next = accumulated + push;
            next = next < 0.0f ? 0.0f : (next > plane->pushLimit ? plane->pushLimit : next);
            plane->push = next;
            push = next - accumulated;
            delta.x += push * plane->normal.x;
            delta.y += push * plane->normal.y;
            totalPush += push < 0.0f ? -push : push;
        }
        if (totalPush < tolerance)
        {
            break;
        }
    }
    m2PlaneSolverResult result = {delta, iteration};
    return result;
}

m2Vec2 m2ClipVector(m2Vec2 vector, const m2CollisionPlane* planes, int32_t count)
{
    m2Vec2 v = vector;
    for (int32_t i = 0; i < count; ++i)
    {
        const m2CollisionPlane* plane = planes + i;
        if (plane->push == 0.0f || plane->clipVelocity == false)
        {
            continue;
        }
        float vn = v.x * plane->normal.x + v.y * plane->normal.y;
        if (vn < 0.0f)
        {
            v.x -= vn * plane->normal.x;
            v.y -= vn * plane->normal.y;
        }
    }
    return v;
}
