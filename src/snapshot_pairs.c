// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pair part of an incoming snapshot's structural check
// (snapshot_check.c): the moved list, the pairs and the end events queued
// for the next step. Like the rest of the check, it reads the bytes in
// place and bounds every index it reads.

#include "snapshot_pairs.h"

#include "broadphase.h"
#include "geometry.h"
#include "snapshot_read.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// The moved list holds distinct shapes, and the flags mark exactly them.
static bool CheckMoved(m2World* world, const uint8_t* in)
{
    const uint8_t* moved = M2_IN(broadphase.moved);
    const uint8_t* inMoved = M2_IN(broadphase.inMoved);
    int32_t count = InlineI32(world, in, &world->broadphase.movedCount);
    if (count < 0 || count > world->shapes.shapeCapacity)
    {
        return false;
    }
    uint8_t* marks = world->restoreMarks;
    memset(marks, 0, (size_t)world->shapes.shapeCapacity);
    for (int32_t k = 0; k < count; ++k)
    {
        int32_t s = ReadI32(moved, k);
        if (!InPool(s, world->shapes.shapeCapacity) || marks[s] != 0 || inMoved[s] == 0)
        {
            return false;
        }
        marks[s] = 1;
    }
    int32_t flagged = 0;
    for (int32_t s = 0; s < world->shapes.shapeCapacity; ++s)
    {
        flagged += inMoved[s] != 0 ? 1 : 0;
    }
    return flagged == count;
}

// Pairs are in strictly increasing key order, and each joins two live
// shapes of different bodies, both in the broadphase.
static bool CheckPairs(const m2World* world, const uint8_t* in)
{
    const uint8_t* keys = M2_IN(contacts.pairKeys);
    const uint8_t* touching = M2_IN(contacts.pairTouching);
    const uint8_t* shapeAlive = M2_IN(shapes.shapeAlive);
    const uint8_t* shapeBody = M2_IN(shapes.shapeBody);
    const uint8_t* proxyIds = M2_IN(broadphase.proxyIds);
    int32_t count = InlineI32(world, in, &world->contacts.pairCount);
    if (count < 0 || count > world->contacts.pairCapacity)
    {
        return false;
    }
    uint64_t previous = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t key = ReadU64(keys, i);
        int32_t a = (int32_t)(key >> 32);
        int32_t b = (int32_t)(key & 0xFFFFFFFFu);
        if (a < 0 || b >= world->shapes.shapeCapacity || (i > 0 && key <= previous) || a >= b ||
            touching[i] > 1 || shapeAlive[a] == 0 || shapeAlive[b] == 0 ||
            ReadI32(proxyIds, a) == M2_NULL_NODE || ReadI32(proxyIds, b) == M2_NULL_NODE ||
            ReadI32(shapeBody, a) == ReadI32(shapeBody, b))
        {
            return false;
        }
        previous = key;
    }
    return true;
}

// One shape as the pair rules read it, from the incoming bytes.
typedef struct IncomingFilters
{
    const uint8_t* categories; // uint64 per shape
    const uint8_t* masks;      // uint64 per shape
    const uint8_t* groups;     // int32 per shape
    const uint8_t* sensors;
    const uint8_t* geometries;
    const uint8_t* shapeBody;
    const uint8_t* bodyTypes;
} IncomingFilters;

// The shape is live, so its body is live and in range: CheckPairs and
// CheckShapeLists ran first.
static m2PairSide IncomingSide(const IncomingFilters* f, int32_t shape)
{
    int32_t type;
    memcpy(&type,
           f->geometries + (size_t)shape * sizeof(m2ShapeGeometry) +
               offsetof(m2ShapeGeometry, type),
           sizeof(type));
    int32_t body = ReadI32(f->shapeBody, shape);
    m2PairSide side = {
        ReadU64(f->categories, shape), ReadU64(f->masks, shape),
        ReadI32(f->groups, shape),     type,
        f->sensors[shape] != 0,        f->bodyTypes[body] == (uint8_t)m2_dynamicBody};
    return side;
}

static uint64_t BodyPairKey(int32_t a, int32_t b)
{
    return a < b ? ((uint64_t)(uint32_t)a << 32) | (uint32_t)b
                 : ((uint64_t)(uint32_t)b << 32) | (uint32_t)a;
}

static int CompareKeys(const void* a, const void* b)
{
    uint64_t x;
    uint64_t y;
    memcpy(&x, a, sizeof(x));
    memcpy(&y, b, sizeof(y));
    return (x > y) - (x < y);
}

// Whether `key`, two bodies, is among the `count` sorted keys.
static bool HasKey(const uint64_t* keys, int32_t count, uint64_t key)
{
    int32_t lo = 0;
    int32_t hi = count - 1;
    while (lo <= hi)
    {
        int32_t mid = lo + (hi - lo) / 2;
        if (keys[mid] == key)
        {
            return true;
        }
        if (keys[mid] < key)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
    return false;
}

// The pairs the next step keeps, those with no moved shape, are pairs the
// broadphase would make: their kinds and filters let them pair, and no
// joint without connected collision joins their bodies. A host call that
// changes any of these between steps marks the shapes moved, and the step
// drops their pairs before it reads one. The kept pairs' bodies go into
// pairScratch, sorted, for the joints to look up.
static bool CheckKeptPairs(m2World* world, const uint8_t* in)
{
    const IncomingFilters filters = {M2_IN(shapes.shapeCategory), M2_IN(shapes.shapeMask),
                                     M2_IN(shapes.shapeGroup),    M2_IN(shapes.shapeSensor),
                                     M2_IN(shapes.shapeGeometry), M2_IN(shapes.shapeBody),
                                     M2_IN(bodies.types)};
    const uint8_t* keys = M2_IN(contacts.pairKeys);
    const uint8_t* inMoved = M2_IN(broadphase.inMoved);
    int32_t count = InlineI32(world, in, &world->contacts.pairCount);
    uint64_t* bodyPairs = world->contacts.pairScratch;
    int32_t kept = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        uint64_t key = ReadU64(keys, i);
        int32_t a = (int32_t)(key >> 32);
        int32_t b = (int32_t)(key & 0xFFFFFFFFu);
        if (inMoved[a] != 0 || inMoved[b] != 0)
        {
            continue;
        }
        m2PairSide sideA = IncomingSide(&filters, a);
        m2PairSide sideB = IncomingSide(&filters, b);
        if (!m2SidesMayPair(&sideA, &sideB))
        {
            return false;
        }
        bodyPairs[kept] = BodyPairKey(ReadI32(filters.shapeBody, a), ReadI32(filters.shapeBody, b));
        kept += 1;
    }
    qsort(bodyPairs, (size_t)kept, sizeof(uint64_t), CompareKeys);
    const uint8_t* jointAlive = M2_IN(joints.jointAlive);
    const uint8_t* collide = M2_IN(joints.jointCollide);
    const uint8_t* bodyA = M2_IN(joints.jointBodyA);
    const uint8_t* bodyB = M2_IN(joints.jointBodyB);
    for (int32_t j = 0; j < world->joints.jointCapacity; ++j)
    {
        if (jointAlive[j] != 0 && collide[j] == 0 &&
            HasKey(bodyPairs, kept, BodyPairKey(ReadI32(bodyA, j), ReadI32(bodyB, j))))
        {
            return false;
        }
    }
    return true;
}

// Whether `count` queued end events, laid out as m2ContactEndEvent and
// m2SensorEndEvent both are, each carry the step count and name two shape
// slots of this world.
static bool QueuedEndsValid(const m2World* world, const uint8_t* events, int32_t count,
                            uint64_t step)
{
    if (count < 0 || count > world->contacts.pairCapacity)
    {
        return false;
    }
    uint32_t capacity = (uint32_t)world->shapes.shapeCapacity;
    for (int32_t i = 0; i < count; ++i)
    {
        m2ContactEndEvent e;
        memcpy(&e, events + (size_t)i * sizeof(e), sizeof(e));
        if (e.shapeIdA.index1 < 1 || e.shapeIdA.index1 > capacity || e.shapeIdB.index1 < 1 ||
            e.shapeIdB.index1 > capacity || e.step != step)
        {
            return false;
        }
    }
    return true;
}

// The end events a destroy or a disable queued since the last step: each
// was queued at the snapshot's step count and names two shape slots, so
// the next step hands the host no id outside its pools.
static bool CheckQueuedEnds(const m2World* world, const uint8_t* in)
{
    _Static_assert(sizeof(m2ContactEndEvent) == sizeof(m2SensorEndEvent),
                   "both queues share one layout");
    uint64_t step = ReadU64(M2_IN(stepCount), 0);
    return QueuedEndsValid(world, M2_IN(events.pendingEndEvents),
                           InlineI32(world, in, &world->events.pendingEndCount), step) &&
           QueuedEndsValid(world, M2_IN(events.pendingSensorEnd),
                           InlineI32(world, in, &world->events.pendingSensorEndCount), step);
}

#undef M2_IN

bool m2SnapshotCheckPairs(m2World* world, const uint8_t* in)
{
    return CheckMoved(world, in) && CheckPairs(world, in) && CheckKeptPairs(world, in) &&
           CheckQueuedEnds(world, in);
}
