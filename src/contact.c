// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contacts: manifolds for every candidate pair, warm-start carry, the
// contact and sensor event streams and contact readback.

#include "contact.h"

#include "shape.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

void m2EmitEnd(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->events.endEventCount >= world->contacts.pairCapacity)
    {
        return;
    }
    m2ContactEndEvent* e = &world->events.endEvents[world->events.endEventCount++];
    e->shapeIdA = m2MakeShapeId(world, shapeA);
    e->shapeIdB = m2MakeShapeId(world, shapeB);
    e->step = world->stepCount;
}

// Shared begin-event geometry: the manifold's world normal, the world
// hit points, and the first-point closing speed, all in body A's frame.
// Used by the solid-contact and the sensor begin streams alike, so a
// sensor overlap reports where it was entered and how fast.
static void FillBeginGeometry(m2World* world, m2ContactBeginEvent* e, int32_t pairIndex,
                              int32_t shapeA, int32_t shapeB)
{
    const m2Manifold* manifold = &world->contacts.manifolds[pairIndex];
    int32_t bodyA = world->shapes.shapeBody[shapeA];
    int32_t bodyB = world->shapes.shapeBody[shapeB];
    m2Transform xfA = world->bodies.transforms[bodyA];
    m2Rot qA = xfA.q;
    e->normal = (m2Vec2){qA.c * manifold->normal.x - qA.s * manifold->normal.y,
                         qA.s * manifold->normal.x + qA.c * manifold->normal.y};
    e->pointCount = manifold->pointCount;
    for (int32_t k = 0; k < manifold->pointCount && k < 2; ++k)
    {
        m2Vec2 anchor = manifold->points[k].anchorA;
        e->points[k].x = xfA.p.x + (double)(qA.c * anchor.x - qA.s * anchor.y);
        e->points[k].y = xfA.p.y + (double)(qA.s * anchor.x + qA.c * anchor.y);
    }

    if (manifold->pointCount > 0)
    {
        // Closing speed at the first point: how hard the hit landed.
        m2Vec2 lcA = world->bodies.localCenters[bodyA];
        m2Vec2 lcB = world->bodies.localCenters[bodyB];
        m2Vec2 anchor = manifold->points[0].anchorA;
        m2Vec2 rA = {qA.c * (anchor.x - lcA.x) - qA.s * (anchor.y - lcA.y),
                     qA.s * (anchor.x - lcA.x) + qA.c * (anchor.y - lcA.y)};
        m2Rot qB = world->bodies.transforms[bodyB].q;
        // The same world point measured from B's center of mass.
        m2Vec2 rB = {(float)(e->points[0].x - world->bodies.transforms[bodyB].p.x) -
                         (qB.c * lcB.x - qB.s * lcB.y),
                     (float)(e->points[0].y - world->bodies.transforms[bodyB].p.y) -
                         (qB.s * lcB.x + qB.c * lcB.y)};
        m2Vec2 vA = world->bodies.linearVelocities[bodyA];
        m2Vec2 vB = world->bodies.linearVelocities[bodyB];
        float wA = world->bodies.angularVelocities[bodyA];
        float wB = world->bodies.angularVelocities[bodyB];
        m2Vec2 velA = {vA.x - wA * rA.y, vA.y + wA * rA.x};
        m2Vec2 velB = {vB.x - wB * rB.y, vB.y + wB * rB.x};
        float vn = (velB.x - velA.x) * e->normal.x + (velB.y - velA.y) * e->normal.y;
        e->approachSpeed = vn < 0.0f ? -vn : 0.0f;
    }
}

void m2EmitBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex)
{
    if (world->events.beginEventCount >= world->contacts.pairCapacity)
    {
        return;
    }
    m2ContactBeginEvent* e = &world->events.beginEvents[world->events.beginEventCount++];
    memset(e, 0, sizeof(*e)); // no stack garbage in observer payloads
    e->shapeIdA = m2MakeShapeId(world, shapeA);
    e->shapeIdB = m2MakeShapeId(world, shapeB);
    e->step = world->stepCount;
    FillBeginGeometry(world, e, pairIndex, shapeA, shapeB);
}

void m2EmitSensorEnd(m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->events.sensorEndCount < world->contacts.pairCapacity)
    {
        m2ContactEndEvent* e = &world->events.sensorEndEvents[world->events.sensorEndCount++];
        e->shapeIdA = m2MakeShapeId(world, shapeA);
        e->shapeIdB = m2MakeShapeId(world, shapeB);
        e->step = world->stepCount;
    }
}

void m2EmitSensorBegin(m2World* world, int32_t shapeA, int32_t shapeB, int32_t pairIndex)
{
    if (world->events.sensorBeginCount < world->contacts.pairCapacity)
    {
        m2ContactBeginEvent* e = &world->events.sensorBeginEvents[world->events.sensorBeginCount++];
        memset(e, 0, sizeof(*e));
        e->shapeIdA = m2MakeShapeId(world, shapeA);
        e->shapeIdB = m2MakeShapeId(world, shapeB);
        e->step = world->stepCount;
        // The overlap manifold carries the hit point and normal.
        FillBeginGeometry(world, e, pairIndex, shapeA, shapeB);
    }
}

// --- Contacts -------------------------------------------------------

// Relative pose of shape B's body in shape A's body frame: the single
// f64 -> f32 crossing for the contact stage.
static m2RelativePose MakeRelativePose(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    m2Transform xfA = world->bodies.transforms[bodyA];
    m2Transform xfB = world->bodies.transforms[bodyB];
    float dx = (float)(xfB.p.x - xfA.p.x);
    float dy = (float)(xfB.p.y - xfA.p.y);
    m2RelativePose pose;
    pose.p = (m2Vec2){xfA.q.c * dx + xfA.q.s * dy, -xfA.q.s * dx + xfA.q.c * dy};
    m2Rot rel = {xfA.q.c * xfB.q.c + xfA.q.s * xfB.q.s, xfA.q.c * xfB.q.s - xfA.q.s * xfB.q.c};
    pose.q = m2NormalizeRot(rel); // every composition renormalizes
    return pose;
}

// Convert a manifold computed with swapped shape roles back into the
// canonical frame (A = lower shape index).
static m2Manifold FlipManifold(m2Manifold in, m2RelativePose poseOfBInA)
{
    m2Manifold out = in;
    for (int32_t k = 0; k < in.pointCount; ++k)
    {
        out.points[k].anchorA = in.points[k].anchorB;
        out.points[k].anchorB = in.points[k].anchorA;
    }
    m2Vec2 n = {poseOfBInA.q.c * in.normal.x - poseOfBInA.q.s * in.normal.y,
                poseOfBInA.q.s * in.normal.x + poseOfBInA.q.c * in.normal.y};
    out.normal = (m2Vec2){-n.x, -n.y};
    return out;
}

static m2RelativePose InvertPose(m2RelativePose pose)
{
    m2RelativePose inv;
    inv.q = (m2Rot){pose.q.c, -pose.q.s};
    m2Vec2 r = {inv.q.c * pose.p.x - inv.q.s * pose.p.y, inv.q.s * pose.p.x + inv.q.c * pose.p.y};
    inv.p = (m2Vec2){-r.x, -r.y};
    return inv;
}

// Chain laws, applied in the chain's own frame after the ordinary SAT
// pipeline has spoken. DELIBERATE DEVIATION from the reference (argued
// per CONTRIBUTING): Box2D integrates ghost handling into a GJK-based
// collider; Maul reuses its bit-proven SAT+clip pipeline and enforces
// one-sidedness and ghost Voronoi rejection as an explicit post-pass.
// Buys: no new collider subsystem, laws visible and testable in one
// place. Cost: seam behavior lives or dies by the crossing tests.
static void ApplyChainLaws(m2Manifold* manifold, const m2ChainSegment* chain)
{
    if (manifold->pointCount == 0)
    {
        return;
    }
    m2Vec2 p1 = chain->segment.point1;
    m2Vec2 p2 = chain->segment.point2;
    m2Vec2 e = {p2.x - p1.x, p2.y - p1.y};
    m2Vec2 rightPerp = {e.y, -e.x};

    // One-sided: solid on the right of point1 -> point2 only.
    if (manifold->normal.x * rightPerp.x + manifold->normal.y * rightPerp.y <= 0.0f)
    {
        manifold->pointCount = 0;
        return;
    }

    float ee = e.x * e.x + e.y * e.y;
    int32_t kept = 0;
    for (int32_t k = 0; k < manifold->pointCount; ++k)
    {
        m2Vec2 pos = manifold->points[k].anchorA;
        float v = e.x * (pos.x - p1.x) + e.y * (pos.y - p1.y);
        bool drop = false;
        if (v < 0.0f)
        {
            // Behind point1: the previous edge's Voronoi region owns
            // anything out here; this segment lets go.
            m2Vec2 prevEdge = {p1.x - chain->ghost1.x, p1.y - chain->ghost1.y};
            float uPrev = prevEdge.x * (pos.x - p1.x) + prevEdge.y * (pos.y - p1.y);
            drop = uPrev <= 0.0f;
        }
        else if (v > ee)
        {
            m2Vec2 nextEdge = {chain->ghost2.x - p2.x, chain->ghost2.y - p2.y};
            float vNext = nextEdge.x * (pos.x - p2.x) + nextEdge.y * (pos.y - p2.y);
            drop = vNext > 0.0f;
        }
        if (!drop)
        {
            manifold->points[kept] = manifold->points[k];
            kept += 1;
        }
    }
    manifold->pointCount = kept;
}

static m2Manifold ComputeManifoldRaw(const m2World* world, int32_t shapeA, int32_t shapeB,
                                     m2RelativePose pose);

static m2Manifold ComputeManifold(const m2World* world, int32_t shapeA, int32_t shapeB,
                                  m2RelativePose pose)
{
    const m2ShapeGeometry* ga = &world->shapes.shapeGeometry[shapeA];
    const m2ShapeGeometry* gb = &world->shapes.shapeGeometry[shapeB];
    if (gb->type == m2_chainSegmentShape && ga->type != m2_chainSegmentShape)
    {
        // Canonical: the chain plays shape A so its laws apply in its
        // own frame; the manifold flips back on the way out.
        m2Manifold m = ComputeManifold(world, shapeB, shapeA, InvertPose(pose));
        return FlipManifold(m, pose);
    }
    m2Manifold m = ComputeManifoldRaw(world, shapeA, shapeB, pose);
    if (ga->type == m2_chainSegmentShape)
    {
        ApplyChainLaws(&m, &ga->chainSegment);
    }
    return m;
}

static m2Manifold ComputeManifoldRaw(const m2World* world, int32_t shapeA, int32_t shapeB,
                                     m2RelativePose pose)
{
    const m2ShapeGeometry* ga = &world->shapes.shapeGeometry[shapeA];
    const m2ShapeGeometry* gb = &world->shapes.shapeGeometry[shapeB];

    if (ga->type == m2_circleShape && gb->type == m2_circleShape)
    {
        return m2CollideCircles(&ga->circle, &gb->circle, pose);
    }
    if (ga->type == m2_polygonShape && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(&ga->polygon, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && gb->type == m2_polygonShape)
    {
        m2Manifold m = m2CollidePolygonAndCircle(&gb->polygon, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }

    // Capsules and segments become 2-vertex rounded polygons; every
    // remaining combination goes through the SAT+clip kernel.
    m2Polygon proxyA;
    m2Polygon proxyB;
    const m2Polygon* pa = NULL;
    const m2Polygon* pb = NULL;
    if (ga->type == m2_polygonShape)
    {
        pa = &ga->polygon;
    }
    else if (ga->type == m2_capsuleShape)
    {
        proxyA = m2MakeSegmentProxy(ga->capsule.point1, ga->capsule.point2, ga->capsule.radius);
        pa = &proxyA;
    }
    else if (ga->type == m2_segmentShape)
    {
        proxyA = m2MakeSegmentProxy(ga->segment.point1, ga->segment.point2, 0.0f);
        pa = &proxyA;
    }
    else if (ga->type == m2_chainSegmentShape)
    {
        proxyA = m2MakeSegmentProxy(ga->chainSegment.segment.point1,
                                    ga->chainSegment.segment.point2, 0.0f);
        pa = &proxyA;
    }
    if (gb->type == m2_polygonShape)
    {
        pb = &gb->polygon;
    }
    else if (gb->type == m2_capsuleShape)
    {
        proxyB = m2MakeSegmentProxy(gb->capsule.point1, gb->capsule.point2, gb->capsule.radius);
        pb = &proxyB;
    }
    else if (gb->type == m2_segmentShape)
    {
        proxyB = m2MakeSegmentProxy(gb->segment.point1, gb->segment.point2, 0.0f);
        pb = &proxyB;
    }
    else if (gb->type == m2_chainSegmentShape)
    {
        proxyB = m2MakeSegmentProxy(gb->chainSegment.segment.point1,
                                    gb->chainSegment.segment.point2, 0.0f);
        pb = &proxyB;
    }

    if (pa != NULL && gb->type == m2_circleShape)
    {
        return m2CollidePolygonAndCircle(pa, &gb->circle, pose);
    }
    if (ga->type == m2_circleShape && pb != NULL)
    {
        m2Manifold m = m2CollidePolygonAndCircle(pb, &ga->circle, InvertPose(pose));
        return FlipManifold(m, pose);
    }
    M2_ASSERT(pa != NULL && pb != NULL);
    return m2CollidePolygons(pa, pb, pose);
}

// Stash old (key, manifold) rows, then rebuild aligned to the new pair
// array, carrying warm-start impulses across by pair key and point id.
void m2StashContacts(m2World* world)
{
    memcpy(world->contacts.oldPairScratch, world->contacts.pairKeys,
           (size_t)world->contacts.pairCount * sizeof(uint64_t));
    memcpy(world->contacts.manifoldScratch, world->contacts.manifolds,
           (size_t)world->contacts.pairCount * sizeof(m2Manifold));
}

typedef struct m2UpdateContactsCtx
{
    m2World* world;
} m2UpdateContactsCtx;

// Each pair writes only its own manifold slot, so the range splits
// freely across workers without touching the arithmetic.
static void UpdateContactsRange(int32_t begin, int32_t end, void* userCtx)
{
    m2World* world = ((m2UpdateContactsCtx*)userCtx)->world;
    for (int32_t i = begin; i < end; ++i)
    {
        int32_t shapeA = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);

        // Frozen pair: both ends static or sleeping, so transforms and
        // geometry are untouched and the stored manifold is exactly
        // what ComputeManifold would return - skip the arithmetic,
        // keep the bits. A kinematic end never freezes (its velocity
        // can change without stepping).
        int32_t frozenBodyA = world->shapes.shapeBody[shapeA];
        int32_t frozenBodyB = world->shapes.shapeBody[shapeB];
        bool frozenA =
            world->bodies.types[frozenBodyA] == (uint8_t)m2_staticBody ||
            (world->bodies.types[frozenBodyA] == (uint8_t)m2_dynamicBody &&
             world->bodies.asleep[frozenBodyA] != 0 && world->bodies.sleepStreak[frozenBodyA] >= 2);
        bool frozenB =
            world->bodies.types[frozenBodyB] == (uint8_t)m2_staticBody ||
            (world->bodies.types[frozenBodyB] == (uint8_t)m2_dynamicBody &&
             world->bodies.asleep[frozenBodyB] != 0 && world->bodies.sleepStreak[frozenBodyB] >= 2);
        if (frozenA && frozenB)
        {
            int32_t flo = 0;
            int32_t fhi = world->contacts.oldPairCount - 1;
            while (flo <= fhi)
            {
                int32_t mid = (flo + fhi) / 2;
                if (world->contacts.oldPairScratch[mid] == world->contacts.pairKeys[i])
                {
                    // NOLINTNEXTLINE(clang-analyzer-core.NullDereference): allocated at creation
                    world->contacts.manifolds[i] = world->contacts.manifoldScratch[mid];
                    // Recompute would match every id against itself and
                    // set the persisted bit; the copy owes the same.
                    for (int32_t k = 0; k < world->contacts.manifolds[i].pointCount; ++k)
                    {
                        world->contacts.manifolds[i].points[k].flags |= 1;
                    }
                    break;
                }
                if (world->contacts.oldPairScratch[mid] < world->contacts.pairKeys[i])
                {
                    flo = mid + 1;
                }
                else
                {
                    fhi = mid - 1;
                }
            }
            if (flo <= fhi)
            {
                continue; // found and copied
            }
            // Brand-new pair between frozen bodies (restore edges):
            // fall through and compute once.
        }

        m2RelativePose pose = MakeRelativePose(world, world->shapes.shapeBody[shapeA],
                                               world->shapes.shapeBody[shapeB]);
        m2Manifold fresh = ComputeManifold(world, shapeA, shapeB, pose);

        // Locate the previous manifold for this pair (both lists sorted;
        // binary search keeps this O(P log P) worst case).
        const m2Manifold* previous = NULL;
        int32_t lo = 0;
        int32_t hi = world->contacts.oldPairCount - 1;
        while (lo <= hi)
        {
            int32_t mid = (lo + hi) / 2;
            if (world->contacts.oldPairScratch[mid] == world->contacts.pairKeys[i])
            {
                previous = &world->contacts.manifoldScratch[mid];
                break;
            }
            if (world->contacts.oldPairScratch[mid] < world->contacts.pairKeys[i])
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid - 1;
            }
        }

        if (previous != NULL)
        {
            for (int32_t k = 0; k < fresh.pointCount; ++k)
            {
                for (int32_t o = 0; o < previous->pointCount; ++o)
                {
                    if (previous->points[o].id == fresh.points[k].id)
                    {
                        fresh.points[k].normalImpulse = previous->points[o].normalImpulse;
                        fresh.points[k].tangentImpulse = previous->points[o].tangentImpulse;
                        fresh.points[k].flags |= 1; // persisted
                        break;
                    }
                }
            }
        }
        world->contacts.manifolds[i] = fresh;
    }
}

void m2UpdateContacts(m2World* world)
{
    m2UpdateContactsCtx ctx = {world};
    m2RunParallel(world, UpdateContactsRange, &ctx, world->contacts.pairCount, 16);
}

m2ContactEvents m2World_GetContactEvents(m2WorldId worldId)
{
    m2ContactEvents events = {NULL, NULL, 0, 0};
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.beginEvents = world->events.beginEvents;
    events.beginCount = world->events.beginEventCount;
    events.endEvents = world->events.endEvents;
    events.endCount = world->events.endEventCount;
    return events;
}

m2SensorEvents m2World_GetSensorEvents(m2WorldId worldId)
{
    m2SensorEvents events = {NULL, NULL, 0, 0};
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.beginEvents = world->events.sensorBeginEvents;
    events.beginCount = world->events.sensorBeginCount;
    events.endEvents = world->events.sensorEndEvents;
    events.endCount = world->events.sensorEndCount;
    return events;
}

m2JointEvents m2World_GetJointEvents(m2WorldId worldId)
{
    m2JointEvents events = {NULL, 0};
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return events;
    }
    events.breakEvents = world->events.jointBreakEvents;
    events.breakCount = world->events.jointBreakEventCount;
    return events;
}

int32_t m2Shape_GetSensorOverlaps(m2ShapeId sensorShapeId, m2ShapeId* overlaps, int32_t capacity)
{
    m2World* world = m2WorldFromIndex(sensorShapeId.world0);
    if (world == NULL)
    {
        return 0;
    }
    int32_t sensor = sensorShapeId.index1 - 1;
    if (sensor < 0 || sensor >= world->shapes.shapeCapacity ||
        world->shapes.shapeAlive[sensor] == 0 ||
        world->shapes.shapeGenerations[sensor] != sensorShapeId.generation ||
        world->shapes.shapeSensor[sensor] == 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (a != sensor && b != sensor)
        {
            continue;
        }
        if (total < capacity)
        {
            overlaps[total] = m2MakeShapeId(world, a == sensor ? b : a);
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetContactData(m2WorldId worldId, m2ContactData* data, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        {
            int32_t sa = (int32_t)(world->contacts.pairKeys[i] >> 32);
            int32_t sb = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
            if (world->shapes.shapeSensor[sa] != 0 || world->shapes.shapeSensor[sb] != 0)
            {
                continue; // sensors carry no physical contact
            }
        }
        if (total < capacity)
        {
            int32_t shapeA = (int32_t)(world->contacts.pairKeys[i] >> 32);
            int32_t shapeB = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
            m2Manifold* manifold = &world->contacts.manifolds[i];
            m2ContactData* out = data + total;
            out->shapeIdA = m2MakeShapeId(world, shapeA);
            out->shapeIdB = m2MakeShapeId(world, shapeB);
            m2Rot qA = world->bodies.transforms[world->shapes.shapeBody[shapeA]].q;
            out->normal = (m2Vec2){qA.c * manifold->normal.x - qA.s * manifold->normal.y,
                                   qA.s * manifold->normal.x + qA.c * manifold->normal.y};
            out->pointCount = manifold->pointCount;
            for (int32_t k = 0; k < 2; ++k)
            {
                bool live = k < manifold->pointCount;
                out->separations[k] = live ? manifold->points[k].separation : 0.0f;
                out->normalImpulses[k] = live ? manifold->points[k].normalImpulse : 0.0f;
                out->tangentImpulses[k] = live ? manifold->points[k].tangentImpulse : 0.0f;
            }
        }
        total += 1;
    }
    return total;
}
