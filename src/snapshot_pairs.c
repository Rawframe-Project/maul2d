// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pair part of an incoming snapshot's structural check
// (snapshot_check.c): the moved list, the pairs, the manifolds of frozen
// pairs and the end events queued for the next step. Like the rest of the check, it reads the bytes
// in place and bounds every index it reads.

#include "snapshot_pairs.h"

#include "broadphase.h"
#include "contact.h"
#include "geometry.h"
#include "snapshot_read.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// The moved list holds distinct shapes, and the flags mark exactly them.
M2_NODISCARD static bool CheckMoved(m2World* world, const uint8_t* in)
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
M2_NODISCARD static bool CheckPairs(const m2World* world, const uint8_t* in)
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
    m2PairSide side = {ReadU64(f->categories, shape), ReadU64(f->masks, shape),
                       ReadI32(f->groups, shape),     type,
                       f->sensors[shape] != 0,        f->bodyTypes[body]};
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
M2_NODISCARD static bool HasKey(const uint64_t* keys, int32_t count, uint64_t key)
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
// broadphase would make: their kinds, their bodies' types and their
// filters let them pair, and no joint without connected collision joins
// their bodies. A host call that changes any of these between steps marks
// the shapes moved, and the step drops their pairs before it reads one.
// The kept pairs' bodies go into pairScratch, sorted, for the joints to
// look up.
M2_NODISCARD static bool CheckKeptPairs(m2World* world, const uint8_t* in)
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

// The incoming arrays that say whether a shape's end of a pair is frozen,
// and what its manifold is computed from.
typedef struct IncomingEnds
{
    const uint8_t* inMoved;
    const uint8_t* shapeBody;
    const uint8_t* geometries;
    const uint8_t* types;
    const uint8_t* asleep;
    const uint8_t* sleepStreak;
    const uint8_t* transforms;
} IncomingEnds;

// A frozen end, as the step's contact update reads one: not moved since
// the last step, and static or asleep for two step ends. The shape is
// live, so its body is live and in range: CheckPairs and CheckShapeLists
// ran first.
M2_NODISCARD static bool IncomingFrozen(const IncomingEnds* e, int32_t shape)
{
    int32_t body = ReadI32(e->shapeBody, shape);
    return e->inMoved[shape] == 0 && (e->types[body] == (uint8_t)m2_staticBody ||
                                      (e->types[body] == (uint8_t)m2_dynamicBody &&
                                       e->asleep[body] != 0 && e->sleepStreak[body] >= 2));
}

// The manifold the step computes for a pair of shapes at their bodies'
// incoming poses.
static m2Manifold IncomingManifold(const IncomingEnds* e, int32_t a, int32_t b)
{
    m2ShapeGeometry ga;
    m2ShapeGeometry gb;
    memcpy(&ga, e->geometries + (size_t)a * sizeof(ga), sizeof(ga));
    memcpy(&gb, e->geometries + (size_t)b * sizeof(gb), sizeof(gb));
    m2Transform xfA;
    m2Transform xfB;
    memcpy(&xfA, e->transforms + (size_t)ReadI32(e->shapeBody, a) * sizeof(xfA), sizeof(xfA));
    memcpy(&xfB, e->transforms + (size_t)ReadI32(e->shapeBody, b) * sizeof(xfB), sizeof(xfB));
    return m2ComputeManifold(&ga, &gb, m2MakeRelativePose(xfA, xfB));
}

// Whether two floats have the same bits: -0 differs from 0, and a NaN
// matches only its own pattern.
M2_NODISCARD static bool SameBits(float x, float y)
{
    uint32_t a;
    uint32_t b;
    memcpy(&a, &x, sizeof(a));
    memcpy(&b, &y, sizeof(b));
    return a == b;
}

M2_NODISCARD static bool SameVecBits(m2Vec2 x, m2Vec2 y)
{
    return SameBits(x.x, y.x) && SameBits(x.y, y.y);
}

// Whether two manifolds hold the same contact, bit for bit: the normal and
// each point's anchors, separation and feature id. The impulses and the
// persisted bit are left out.
M2_NODISCARD static bool SameContact(const m2Manifold* x, const m2Manifold* y)
{
    if (x->pointCount != y->pointCount || !SameVecBits(x->normal, y->normal))
    {
        return false;
    }
    for (int32_t k = 0; k < x->pointCount; ++k)
    {
        const m2ManifoldPoint* p = &x->points[k];
        const m2ManifoldPoint* q = &y->points[k];
        if (!SameVecBits(p->anchorA, q->anchorA) || !SameVecBits(p->anchorB, q->anchorB) ||
            !SameBits(p->separation, q->separation) || p->id != q->id ||
            (p->flags & ~1u) != (q->flags & ~1u))
        {
            return false;
        }
    }
    return true;
}

// Each pair's touching flag is its manifold's: the step sets every flag
// from the point count, and nothing between steps changes one without
// the other. A frozen pair holds the manifold its shapes give at their
// bodies' poses, since the step copies it forward instead of computing
// it; only its impulses, which the solver accumulates, and the persisted
// bit are the snapshot's own. Every snapshot a world writes keeps the
// rule: a host call that moves a shape wakes its sleeping body or marks
// the shape moved, and a body falling asleep computes its manifolds one
// more step before they freeze.
M2_NODISCARD static bool CheckFrozenManifolds(const m2World* world, const uint8_t* in)
{
    const IncomingEnds ends = {M2_IN(broadphase.inMoved),   M2_IN(shapes.shapeBody),
                               M2_IN(shapes.shapeGeometry), M2_IN(bodies.types),
                               M2_IN(bodies.asleep),        M2_IN(bodies.sleepStreak),
                               M2_IN(bodies.transforms)};
    const uint8_t* keys = M2_IN(contacts.pairKeys);
    const uint8_t* touching = M2_IN(contacts.pairTouching);
    const uint8_t* manifolds = M2_IN(contacts.manifolds);
    int32_t count = InlineI32(world, in, &world->contacts.pairCount);
    for (int32_t i = 0; i < count; ++i)
    {
        m2Manifold stored;
        memcpy(&stored, manifolds + (size_t)i * sizeof(stored), sizeof(stored));
        if ((touching[i] != 0) != (stored.pointCount > 0))
        {
            return false;
        }
        uint64_t key = ReadU64(keys, i);
        int32_t a = (int32_t)(key >> 32);
        int32_t b = (int32_t)(key & 0xFFFFFFFFu);
        if (!IncomingFrozen(&ends, a) || !IncomingFrozen(&ends, b))
        {
            continue;
        }
        m2Manifold fresh = IncomingManifold(&ends, a, b);
        if (!SameContact(&stored, &fresh))
        {
            return false;
        }
    }
    return true;
}

// Whether `count` queued end events, laid out as m2ContactEndEvent and
// m2SensorEndEvent both are, each carry the step count and name two shape
// slots of this world.
M2_NODISCARD static bool QueuedEndsValid(const m2World* world, const uint8_t* events, int32_t count,
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
M2_NODISCARD static bool CheckQueuedEnds(const m2World* world, const uint8_t* in)
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
           CheckFrozenManifolds(world, in) && CheckQueuedEnds(world, in);
}
