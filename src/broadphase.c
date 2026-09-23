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
    int32_t body = world->shapeBody[shapeIndex];
    return m2ComputeShapeAABB(&world->shapeGeometry[shapeIndex], world->transforms[body]);
}

int32_t m2ShapeTreeIndex(const m2World* world, int32_t shapeIndex)
{
    return world->types[world->shapeBody[shapeIndex]];
}

void m2PushMoved(m2World* world, int32_t shapeIndex)
{
    if (world->inMoved[shapeIndex] != 0)
    {
        return;
    }
    world->inMoved[shapeIndex] = 1;
    world->moved[world->movedCount] = shapeIndex;
    world->movedCount += 1;
}

// The filter takes effect through the normal rebuild road: wake both
// ends and push their shapes, and the pair diff emits the M19 ends.
void m2RefilterJointedBodies(m2World* world, int32_t bodyA, int32_t bodyB)
{
    if (world->types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyA] = 0;
        world->sleepTimes[bodyA] = 0.0f;
    }
    if (world->types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyB] = 0;
        world->sleepTimes[bodyB] = 0.0f;
    }
    for (int32_t s = world->bodyShapeHead[bodyA]; s != -1; s = world->shapeNext[s])
    {
        m2PushMoved(world, s);
    }
    for (int32_t s = world->bodyShapeHead[bodyB]; s != -1; s = world->shapeNext[s])
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
    return world->types[world->shapeBody[shapeIndex]] == (uint8_t)m2_dynamicBody;
}

// Re-derive pairs touched by the moved set, then batch-merge with the
// untouched remainder.
// Which trees a moved shape queries: dynamic and kinematic movers sweep
// every tree, static movers (teleports) only the dynamic one.
static bool MoverSeesTree(const m2World* world, int32_t shapeIndex, int32_t tree)
{
    return world->types[world->shapeBody[shapeIndex]] != (uint8_t)m2_staticBody ||
           tree == (int32_t)m2_dynamicBody;
}

void m2UpdatePairs(m2World* world)
{
    world->pairOverflow = 0;
    if (world->movedCount == 0)
    {
        return;
    }

    qsort(world->moved, (size_t)world->movedCount, sizeof(int32_t), CompareI32);

    int32_t collected = 0;
    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        int32_t shapeIndex = world->moved[m];
        if (world->shapeAlive[shapeIndex] == 0 || world->proxyIds[shapeIndex] == M2_NULL_NODE)
        {
            continue;
        }
        int32_t treeIndex = m2ShapeTreeIndex(world, shapeIndex);
        m2AABB fat = world->treeNodes[treeIndex][world->proxyIds[shapeIndex]].aabb;

        bool moverDynamic = ShapeIsDynamic(world, shapeIndex);
        // Kinematic movers sweep every tree: a kinematic character
        // walking through a static trigger zone is the most ordinary
        // sensor story there is. Static movers only exist via
        // teleport and keep the narrow scan.
        bool moverKinematic =
            world->types[world->shapeBody[shapeIndex]] == (uint8_t)m2_kinematicBody;
        int32_t firstTree = moverDynamic || moverKinematic ? 0 : m2_dynamicBody;
        int32_t lastTree = moverDynamic || moverKinematic ? M2_TREE_COUNT - 1 : m2_dynamicBody;
        for (int32_t t = firstTree; t <= lastTree; ++t)
        {
            m2TreeCursor cursor;
            m2TreeBeginQuery(&cursor, &world->trees[t], world->treeNodes[t], fat);
            int32_t other;
            while (m2TreeNextQuery(&cursor, &other))
            {
                if (other == shapeIndex || world->shapeAlive[other] == 0)
                {
                    continue;
                }
                if (world->inMoved[other] != 0 && other < shapeIndex &&
                    MoverSeesTree(world, other, m2ShapeTreeIndex(world, shapeIndex)))
                {
                    continue; // both moved and both see each other: the lower slot recorded it
                }
                if (world->shapeBody[other] == world->shapeBody[shapeIndex])
                {
                    continue; // same-body shapes never pair
                }
                if (!moverDynamic && !ShapeIsDynamic(world, other) &&
                    world->shapeSensor[shapeIndex] == 0 && world->shapeSensor[other] == 0)
                {
                    continue; // two non-dynamics only pair through a sensor
                }
                if (world->shapeSensor[shapeIndex] != 0 && world->shapeSensor[other] != 0)
                {
                    continue; // two sensors never detect each other
                }
                int32_t groupA = world->shapeGroup[shapeIndex];
                if (groupA != 0 && groupA == world->shapeGroup[other])
                {
                    if (groupA < 0)
                    {
                        continue; // same negative group: never collide
                    }
                }
                else if ((world->shapeCategory[shapeIndex] & world->shapeMask[other]) == 0 ||
                         (world->shapeCategory[other] & world->shapeMask[shapeIndex]) == 0)
                {
                    continue; // filtered out (category/mask, both ways)
                }
                if (m2JointsForbidPair(world, world->shapeBody[shapeIndex],
                                       world->shapeBody[other]))
                {
                    continue; // connected without collideConnected
                }
                if (collected < world->pairCapacity)
                {
                    world->pairScratch[collected] = PairKey(shapeIndex, other);
                }
                collected += 1;
            }
        }
    }
    // A full pair table drops the excess candidates, counted loudly in
    // m2Counters.pairOverflow rather than asserted away.
    if (collected > world->pairCapacity)
    {
        world->pairOverflow += collected - world->pairCapacity;
        collected = world->pairCapacity;
    }

    qsort(world->pairScratch, (size_t)collected, sizeof(uint64_t), CompareU64);

    // Old set stash for the end-event diff and the touching carry.
    memcpy(world->oldPairScratch, world->pairKeys, (size_t)world->pairCount * sizeof(uint64_t));
    memcpy(world->touchingScratch, world->pairTouching, (size_t)world->pairCount * sizeof(uint8_t));
    int32_t oldCount = world->pairCount;

    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (world->inMoved[a] == 0 && world->inMoved[b] == 0 && world->shapeAlive[a] != 0 &&
            world->shapeAlive[b] != 0)
        {
            world->pairKeys[kept] = world->pairKeys[i];
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
        if (i < kept && (j >= collected || world->pairKeys[i] <= world->pairScratch[j]))
        {
            next = world->pairKeys[i];
            i += 1;
        }
        else
        {
            next = world->pairScratch[j];
            j += 1;
        }
        if (hasPrevious && next == previous)
        {
            continue;
        }
        previous = next;
        hasPrevious = true;
        if (outCount < world->pairCapacity)
        {
            world->pairMergeScratch[outCount] = next;
            outCount += 1;
        }
        else
        {
            world->pairOverflow += 1;
        }
    }
    memcpy(world->pairKeys, world->pairMergeScratch, (size_t)outCount * sizeof(uint64_t));
    world->pairCount = outCount;

#ifndef NDEBUG
    // The invariant that hid a reversed read-back for twenty slices:
    // the pair list must be strictly ascending after every rebuild.
    for (int32_t v = 1; v < world->pairCount; ++v)
    {
        M2_ASSERT(world->pairKeys[v - 1] < world->pairKeys[v]);
    }
#endif

    // Diff old vs new (both sorted): vanished-and-touching pairs emit
    // their end events here (M19: pair loss is a contact-killing path);
    // surviving pairs carry their touching flag to the new slot.
    {
        int32_t oi = 0;
        int32_t ni = 0;
        while (oi < oldCount || ni < world->pairCount)
        {
            uint64_t ok = oi < oldCount ? world->oldPairScratch[oi] : UINT64_MAX;
            uint64_t nk = ni < world->pairCount ? world->pairKeys[ni] : UINT64_MAX;
            if (ok == nk)
            {
                world->pairTouching[ni] = world->touchingScratch[oi];
                oi += 1;
                ni += 1;
            }
            else if (ok < nk)
            {
                if (world->touchingScratch[oi] != 0)
                {
                    int32_t a = (int32_t)(ok >> 32);
                    int32_t b = (int32_t)(ok & 0xFFFFFFFFu);
                    if (world->shapeAlive[a] != 0 && world->shapeAlive[b] != 0)
                    {
                        m2EmitEnd(world, a, b); // destroy path emits its own
                    }
                }
                oi += 1;
            }
            else
            {
                world->pairTouching[ni] = 0;
                ni += 1;
            }
        }
    }

    for (int32_t m = 0; m < world->movedCount; ++m)
    {
        world->inMoved[world->moved[m]] = 0;
    }
    world->movedCount = 0;
}

void m2PrunePairsOfShape(m2World* world, int32_t shapeIndex)
{
    int32_t kept = 0;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            world->pairKeys[kept] = world->pairKeys[i];
            world->pairTouching[kept] = world->pairTouching[i];
            world->manifolds[kept] = world->manifolds[i];
            kept += 1;
        }
    }
    world->pairCount = kept;
}
