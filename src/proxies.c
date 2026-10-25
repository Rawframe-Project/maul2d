// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A shape's proxy in the broadphase trees: its fat box, refitted when the
// shape leaves it, and the moved list the next pair update reads. The
// writers, the step and the sleep pass all refit through here.

#include "proxies.h"

#include "dynamic_tree.h"
#include "geometry.h"
#include "world_internal.h"

// Fat margin in meters: how far a shape may move before its proxy refits.
#define M2_AABB_MARGIN 0.1

m2Aabb m2Fatten(m2Aabb aabb)
{
    aabb.lowerBound.x -= M2_AABB_MARGIN;
    aabb.lowerBound.y -= M2_AABB_MARGIN;
    aabb.upperBound.x += M2_AABB_MARGIN;
    aabb.upperBound.y += M2_AABB_MARGIN;
    return aabb;
}

m2Aabb m2ShapeTightAabb(const m2World* world, int32_t shapeIndex)
{
    int32_t body = world->shapes.shapeBody[shapeIndex];
    return m2ComputeShapeAabb(&world->shapes.shapeGeometry[shapeIndex],
                              world->bodies.transforms[body]);
}

int32_t m2ShapeTreeIndex(const m2World* world, int32_t shapeIndex)
{
    return world->bodies.types[world->shapes.shapeBody[shapeIndex]];
}

bool m2RefitProxy(m2World* world, int32_t shapeIndex)
{
    int32_t proxy = world->broadphase.proxyIds[shapeIndex];
    m2Aabb tight = m2ShapeTightAabb(world, shapeIndex);
    int32_t tree = m2ShapeTreeIndex(world, shapeIndex);
    if (m2Aabb_Contains(world->broadphase.treeNodes[tree][proxy].aabb, tight))
    {
        return false;
    }
    m2TreeMove(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree], proxy,
               m2Fatten(tight));
    return true;
}

void m2RefitBodyProxies(m2World* world, int32_t bodyIndex)
{
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        if (world->broadphase.proxyIds[s] != M2_NULL_NODE && m2RefitProxy(world, s))
        {
            m2PushMoved(world, s);
        }
    }
}

void m2PushMoved(m2World* world, int32_t shapeIndex)
{
    if (world->broadphase.inMoved[shapeIndex] != 0)
    {
        return;
    }
    world->broadphase.inMoved[shapeIndex] = 1;
    world->broadphase.moved[world->broadphase.movedCount] = shapeIndex;
    world->broadphase.movedCount += 1;
}
