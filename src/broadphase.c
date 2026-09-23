// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase: fat AABBs in the dynamic trees, the moved set, and the
// candidate pair list rebuilt from moved shapes each step.

#include "broadphase.h"

#include "contact.h"
#include "joint.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <stdlib.h>
#include <string.h>

// Fat margin in meters (topic-02 §3; harness-tuned later, F-T2-1).
#define M2_AABB_MARGIN 0.1

// --- Broadphase helpers ------------------------------------------------------

m2AABB m2Fatten(m2AABB aabb)
{
    aabb.lowerBound.x -= M2_AABB_MARGIN;
    aabb.lowerBound.y -= M2_AABB_MARGIN;
    aabb.upperBound.x += M2_AABB_MARGIN;
    aabb.upperBound.y += M2_AABB_MARGIN;
    return aabb;
}

m2AABB m2ShapeTightAABB(const m2World* world, int32_t shapeIndex)
{
    int32_t body = world->shapes.shapeBody[shapeIndex];
    return m2ComputeShapeAABB(&world->shapes.shapeGeometry[shapeIndex],
                              world->bodies.transforms[body]);
}

int32_t m2ShapeTreeIndex(const m2World* world, int32_t shapeIndex)
{
    return world->bodies.types[world->shapes.shapeBody[shapeIndex]];
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

// The filter takes effect through the normal rebuild road: wake both
// ends and push their shapes, and the pair diff emits the M19 ends.
void m2RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB)
{
    if (world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[bodyA] = 0;
        world->bodies.sleepTimes[bodyA] = 0.0f;
    }
    if (world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[bodyB] = 0;
        world->bodies.sleepTimes[bodyB] = 0.0f;
    }
    for (int32_t s = world->bodies.bodyShapeHead[bodyA]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2PushMoved(world, s);
    }
    for (int32_t s = world->bodies.bodyShapeHead[bodyB]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2PushMoved(world, s);
    }
}

static uint64_t PairKey(int32_t a, int32_t b)
{
    uint64_t lo = (uint64_t)(a < b ? a : b);
    uint64_t hi = (uint64_t)(a < b ? b : a);
    return (lo << 32) | hi;
}

static int CompareU64(const void* a, const void* b)
{
    uint64_t ua = *(const uint64_t*)a;
    uint64_t ub = *(const uint64_t*)b;
    return ua < ub ? -1 : (ua > ub ? 1 : 0);
}

static int CompareI32(const void* a, const void* b)
{
    int32_t ia = *(const int32_t*)a;
    int32_t ib = *(const int32_t*)b;
    return ia < ib ? -1 : (ia > ib ? 1 : 0);
}

static bool ShapeIsDynamic(const m2World* world, int32_t shapeIndex)
{
    return world->bodies.types[world->shapes.shapeBody[shapeIndex]] == (uint8_t)m2_dynamicBody;
}

// Re-derive pairs touched by the moved set, then batch-merge with the
// untouched remainder.
// Which trees a moved shape queries: dynamic and kinematic movers sweep
// every tree, static movers (teleports) only the dynamic one.
static bool MoverSeesTree(const m2World* world, int32_t shapeIndex, int32_t tree)
{
    return world->bodies.types[world->shapes.shapeBody[shapeIndex]] != (uint8_t)m2_staticBody ||
           tree == (int32_t)m2_dynamicBody;
}

void m2UpdatePairs(m2World* world)
{
    world->contacts.pairOverflow = 0;
    if (world->broadphase.movedCount == 0)
    {
        return;
    }

    qsort(world->broadphase.moved, (size_t)world->broadphase.movedCount, sizeof(int32_t),
          CompareI32);

    int32_t collected = 0;
    for (int32_t m = 0; m < world->broadphase.movedCount; ++m)
    {
        int32_t shapeIndex = world->broadphase.moved[m];
        if (world->shapes.shapeAlive[shapeIndex] == 0 ||
            world->broadphase.proxyIds[shapeIndex] == M2_NULL_NODE)
        {
            continue;
        }
        int32_t treeIndex = m2ShapeTreeIndex(world, shapeIndex);
        m2AABB fat =
            world->broadphase.treeNodes[treeIndex][world->broadphase.proxyIds[shapeIndex]].aabb;

        bool moverDynamic = ShapeIsDynamic(world, shapeIndex);
        // Kinematic movers sweep every tree: a kinematic character
        // walking through a static trigger zone is the most ordinary
        // sensor story there is. Static movers only exist via
        // teleport and keep the narrow scan.
        bool moverKinematic =
            world->bodies.types[world->shapes.shapeBody[shapeIndex]] == (uint8_t)m2_kinematicBody;
        int32_t firstTree = moverDynamic || moverKinematic ? 0 : m2_dynamicBody;
        int32_t lastTree = moverDynamic || moverKinematic ? M2_TREE_COUNT - 1 : m2_dynamicBody;
        for (int32_t t = firstTree; t <= lastTree; ++t)
        {
            m2TreeCursor cursor;
            m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                             fat);
            int32_t other;
            while (m2TreeNextQuery(&cursor, &other))
            {
                if (other == shapeIndex || world->shapes.shapeAlive[other] == 0)
                {
                    continue;
                }
                if (world->broadphase.inMoved[other] != 0 && other < shapeIndex &&
                    MoverSeesTree(world, other, m2ShapeTreeIndex(world, shapeIndex)))
                {
                    continue; // both moved and both see each other: the lower slot recorded it
                }
                if (world->shapes.shapeBody[other] == world->shapes.shapeBody[shapeIndex])
                {
                    continue; // same-body shapes never pair
                }
                if (!moverDynamic && !ShapeIsDynamic(world, other) &&
                    world->shapes.shapeSensor[shapeIndex] == 0 &&
                    world->shapes.shapeSensor[other] == 0)
                {
                    continue; // two non-dynamics only pair through a sensor
                }
                if (world->shapes.shapeSensor[shapeIndex] != 0 &&
                    world->shapes.shapeSensor[other] != 0)
                {
                    continue; // two sensors never detect each other
                }
                int32_t groupA = world->shapes.shapeGroup[shapeIndex];
                if (groupA != 0 && groupA == world->shapes.shapeGroup[other])
                {
                    if (groupA < 0)
                    {
                        continue; // same negative group: never collide
                    }
                }
                else if ((world->shapes.shapeCategory[shapeIndex] &
                          world->shapes.shapeMask[other]) == 0 ||
                         (world->shapes.shapeCategory[other] &
                          world->shapes.shapeMask[shapeIndex]) == 0)
                {
                    continue; // filtered out (category/mask, both ways)
                }
                if (m2JointsForbidPair(world, world->shapes.shapeBody[shapeIndex],
                                       world->shapes.shapeBody[other]))
                {
                    continue; // connected without collideConnected
                }
                if (collected < world->contacts.pairCapacity)
                {
                    world->contacts.pairScratch[collected] = PairKey(shapeIndex, other);
                }
                collected += 1;
            }
        }
    }
    // A full pair table drops the excess candidates, counted loudly in
    // m2Counters.pairOverflow rather than asserted away.
    if (collected > world->contacts.pairCapacity)
    {
        world->contacts.pairOverflow += collected - world->contacts.pairCapacity;
        collected = world->contacts.pairCapacity;
    }

    qsort(world->contacts.pairScratch, (size_t)collected, sizeof(uint64_t), CompareU64);

    // Old set stash for the end-event diff and the touching carry.
    memcpy(world->contacts.oldPairScratch, world->contacts.pairKeys,
           (size_t)world->contacts.pairCount * sizeof(uint64_t));
    memcpy(world->solver.touchingScratch, world->contacts.pairTouching,
           (size_t)world->contacts.pairCount * sizeof(uint8_t));
    int32_t oldCount = world->contacts.pairCount;

    int32_t kept = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (world->broadphase.inMoved[a] == 0 && world->broadphase.inMoved[b] == 0 &&
            world->shapes.shapeAlive[a] != 0 && world->shapes.shapeAlive[b] != 0)
        {
            world->contacts.pairKeys[kept] = world->contacts.pairKeys[i];
            kept += 1;
        }
    }

    // Merge the surviving pairs and the new candidates (both sorted)
    // into a separate buffer, keeping the smallest keys when the table
    // is full and counting the rest.
    int32_t i = 0;
    int32_t j = 0;
    int32_t outCount = 0;
    uint64_t previous = 0;
    bool hasPrevious = false;
    while (i < kept || j < collected)
    {
        uint64_t next;
        if (i < kept &&
            (j >= collected || world->contacts.pairKeys[i] <= world->contacts.pairScratch[j]))
        {
            next = world->contacts.pairKeys[i];
            i += 1;
        }
        else
        {
            next = world->contacts.pairScratch[j];
            j += 1;
        }
        if (hasPrevious && next == previous)
        {
            continue;
        }
        previous = next;
        hasPrevious = true;
        if (outCount < world->contacts.pairCapacity)
        {
            world->contacts.pairMergeScratch[outCount] = next;
            outCount += 1;
        }
        else
        {
            world->contacts.pairOverflow += 1;
        }
    }
    memcpy(world->contacts.pairKeys, world->contacts.pairMergeScratch,
           (size_t)outCount * sizeof(uint64_t));
    world->contacts.pairCount = outCount;

#ifndef NDEBUG
    // The invariant that hid a reversed read-back for twenty slices:
    // the pair list must be strictly ascending after every rebuild.
    for (int32_t v = 1; v < world->contacts.pairCount; ++v)
    {
        M2_ASSERT(world->contacts.pairKeys[v - 1] < world->contacts.pairKeys[v]);
    }
#endif

    // Diff old vs new (both sorted): vanished-and-touching pairs emit
    // their end events here (M19: pair loss is a contact-killing path);
    // surviving pairs carry their touching flag to the new slot.
    {
        int32_t oi = 0;
        int32_t ni = 0;
        while (oi < oldCount || ni < world->contacts.pairCount)
        {
            uint64_t ok = oi < oldCount ? world->contacts.oldPairScratch[oi] : UINT64_MAX;
            uint64_t nk =
                ni < world->contacts.pairCount ? world->contacts.pairKeys[ni] : UINT64_MAX;
            if (ok == nk)
            {
                world->contacts.pairTouching[ni] = world->solver.touchingScratch[oi];
                oi += 1;
                ni += 1;
            }
            else if (ok < nk)
            {
                if (world->solver.touchingScratch[oi] != 0)
                {
                    int32_t a = (int32_t)(ok >> 32);
                    int32_t b = (int32_t)(ok & 0xFFFFFFFFu);
                    if (world->shapes.shapeAlive[a] != 0 && world->shapes.shapeAlive[b] != 0)
                    {
                        m2EmitEnd(world, a, b); // destroy path emits its own
                    }
                }
                oi += 1;
            }
            else
            {
                world->contacts.pairTouching[ni] = 0;
                ni += 1;
            }
        }
    }

    for (int32_t m = 0; m < world->broadphase.movedCount; ++m)
    {
        world->broadphase.inMoved[world->broadphase.moved[m]] = 0;
    }
    world->broadphase.movedCount = 0;
}

void m2PrunePairsOfShape(m2World* world, int32_t shapeIndex)
{
    int32_t kept = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            world->contacts.pairKeys[kept] = world->contacts.pairKeys[i];
            world->contacts.pairTouching[kept] = world->contacts.pairTouching[i];
            world->contacts.manifolds[kept] = world->contacts.manifolds[i];
            kept += 1;
        }
    }
    world->contacts.pairCount = kept;
}
