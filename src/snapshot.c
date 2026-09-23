// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshots and hashing: the block walk that sizes, writes and restores
// the world state, and the world hashes built on the same walk.

#include "snapshot.h"

#include "world_state.h"

#include "joint.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_SNAPSHOT_MAGIC 0x4D32534Eu // 'M2SN'

#define M2_SNAPSHOT_VERSION 1u

// --- Snapshot -------------------------------------------------------------------

typedef struct m2SnapshotHeader
{
    uint32_t magic;
    uint32_t version;
    int32_t bodyCapacity;
    int32_t maxBodyIndex;
    uint64_t stepCount;
    m2Vec2 gravity;
    m2Vec2 windVelocity;
    float windLinearDrag;
    int32_t windReserved; // keeps the header 8-aligned and padding-free
    int32_t freeHead;
    int32_t freeTail;
    int32_t freeCount;
    int32_t retiredCount;
    int32_t movedCount;
    int32_t pairCount;
    int32_t shapeCapacity;
    int32_t maxShapeIndex;
    int32_t shapeFreeHead;
    int32_t shapeFreeTail;
    int32_t shapeFreeCount;
    int32_t shapeRetiredCount;
} m2SnapshotHeader;

_Static_assert(sizeof(m2SnapshotHeader) == 96, "snapshot header must be padding-free");

static bool InRange(int32_t value, int32_t lo, int32_t hi)
{
    return value >= lo && value <= hi;
}

// The header's counters index the world's arrays directly after a
// restore, so a buffer whose counters point outside the capacities is
// refused before anything is overwritten.
static bool HeaderCountsInRange(const m2SnapshotHeader* h)
{
    int32_t bodies = h->bodyCapacity;
    int32_t shapes = h->shapeCapacity;
    return InRange(h->maxBodyIndex, 0, bodies) && InRange(h->freeHead, 0, bodies - 1) &&
           InRange(h->freeTail, 0, bodies - 1) && InRange(h->freeCount, 0, bodies) &&
           InRange(h->retiredCount, 0, bodies) && InRange(h->movedCount, 0, shapes) &&
           InRange(h->pairCount, 0, 8 * shapes) && InRange(h->maxShapeIndex, 0, shapes) &&
           InRange(h->shapeFreeHead, 0, shapes - 1) && InRange(h->shapeFreeTail, 0, shapes - 1) &&
           InRange(h->shapeFreeCount, 0, shapes) && InRange(h->shapeRetiredCount, 0, shapes);
}

// Single source of truth: the size IS the walk (measure mode). The
// duplicated byte formula died here after its third drift (assert-caught
// every time; root cause now removed).
static int32_t BlockBytes(const m2World* world)
{
    return m2StateWalk((m2World*)world, NULL, NULL, 2);
}

int32_t m2World_SnapshotSize(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    return (int32_t)sizeof(m2SnapshotHeader) + BlockBytes(world);
}

// The block walk is shared by Snapshot and Restore so the layouts can
// never drift apart (direction 0 = write, 1 = read).

int32_t m2World_Snapshot(m2WorldId worldId, void* buffer, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    int32_t size = m2World_SnapshotSize(worldId);
    if (world == NULL || buffer == NULL || capacity < size)
    {
        return 0;
    }

    m2SnapshotHeader header;
    memset(&header, 0, sizeof(header));
    header.magic = M2_SNAPSHOT_MAGIC;
    header.version = M2_SNAPSHOT_VERSION;
    header.bodyCapacity = world->bodies.bodyCapacity;
    header.maxBodyIndex = world->bodies.maxBodyIndex;
    header.stepCount = world->stepCount;
    header.gravity = world->gravity;
    header.windVelocity = world->windVelocity;
    header.windLinearDrag = world->windLinearDrag;
    header.windReserved = 0;
    header.freeHead = world->bodies.freeHead;
    header.freeTail = world->bodies.freeTail;
    header.freeCount = world->bodies.freeCount;
    header.retiredCount = world->bodies.retiredCount;
    header.movedCount = world->broadphase.movedCount;
    header.pairCount = world->contacts.pairCount;
    header.shapeCapacity = world->shapes.shapeCapacity;
    header.maxShapeIndex = world->shapes.maxShapeIndex;
    header.shapeFreeHead = world->shapes.shapeFreeHead;
    header.shapeFreeTail = world->shapes.shapeFreeTail;
    header.shapeFreeCount = world->shapes.shapeFreeCount;
    header.shapeRetiredCount = world->shapes.shapeRetiredCount;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t cursor = (int32_t)sizeof(header) + m2StateWalk(world, out + sizeof(header), NULL, 0);
    M2_ASSERT(cursor == size);
    return cursor;
}

bool m2World_Restore(m2WorldId worldId, const void* buffer, int32_t size)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || buffer == NULL || size < (int32_t)sizeof(m2SnapshotHeader))
    {
        return false;
    }
    m2SnapshotHeader header;
    memcpy(&header, buffer, sizeof(header));
    if (header.magic != M2_SNAPSHOT_MAGIC || header.version != M2_SNAPSHOT_VERSION ||
        header.bodyCapacity != world->bodies.bodyCapacity ||
        header.shapeCapacity != world->shapes.shapeCapacity ||
        size != (int32_t)sizeof(header) + BlockBytes(world) || !HeaderCountsInRange(&header))
    {
        return false;
    }

    world->bodies.maxBodyIndex = header.maxBodyIndex;
    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->windVelocity = header.windVelocity;
    world->windLinearDrag = header.windLinearDrag;
    world->bodies.freeHead = header.freeHead;
    world->bodies.freeTail = header.freeTail;
    world->bodies.freeCount = header.freeCount;
    world->bodies.retiredCount = header.retiredCount;
    world->broadphase.movedCount = header.movedCount;
    world->contacts.pairCount = header.pairCount;
    world->shapes.maxShapeIndex = header.maxShapeIndex;
    world->shapes.shapeFreeHead = header.shapeFreeHead;
    world->shapes.shapeFreeTail = header.shapeFreeTail;
    world->shapes.shapeFreeCount = header.shapeFreeCount;
    world->shapes.shapeRetiredCount = header.shapeRetiredCount;

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header) + m2StateWalk(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == size);
    (void)cursor;
    m2RebuildJointEdges(world);

    // Restores are first-class journal citizens: the tape carries the
    // snapshot itself, so rollback-heavy sessions replay bit-exactly.
    // The price is tape size, and that is the caller's tradeoff.
    m2JournalRecordRestore(world, buffer, size);

    // Events are an observer stream from an abandoned timeline: cleared
    // on restore, re-emitted by re-simulation.
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    world->events.pendingEndCount = 0;
    world->events.sensorBeginCount = 0;
    world->events.sensorEndCount = 0;
    world->events.pendingSensorEndCount = 0;
    world->events.jointBreakEventCount = 0;
    return true;
}

uint64_t m2World_Hash(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, &world->stepCount, (int32_t)sizeof(world->stepCount));
    h = m2Hash64(h, &world->gravity, (int32_t)sizeof(world->gravity));
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->bodies.transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->bodies.linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->bodies.angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->bodies.types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->bodies.asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->bodies.sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.sleepStreak[i], 1);
        h = m2Hash64(h, &world->bodies.bullets[i], (int32_t)sizeof(uint8_t));
    }
    h = m2Hash64(h, world->contacts.pairKeys,
                 world->contacts.pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->contacts.manifolds,
                 world->contacts.pairCount * (int32_t)sizeof(m2Manifold));
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        if (world->joints.jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->joints.jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->joints.jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    if (world->volumes.fvCapacity > 0)
    {
        for (int32_t i = 0; i < world->volumes.maxFvIndex; ++i)
        {
            h = m2Hash64(h, &world->volumes.fvAlive[i], 1);
            if (world->volumes.fvAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->volumes.fvSurface[i], (int32_t)sizeof(double));
        }
    }
    if (world->particles.particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particles.particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particles.particleAlive[i], 1);
            if (world->particles.particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particles.particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particles.particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particles.particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particles.particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particles.particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particles.particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringA,
                     world->particles.particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringB,
                     world->particles.particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringRest,
                     world->particles.particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particles.particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadA,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadB,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadC,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
    }
    return h;
}

// Subsystem hashes: independent seeds on purpose (the total is not
// a function of the parts), each loop mirroring the gated hash's
// coverage for its slice of the world.
m2WorldHashParts m2World_HashParts(m2WorldId worldId)
{
    m2WorldHashParts parts;
    memset(&parts, 0, sizeof(parts));
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return parts;
    }

    uint64_t h = M2_HASH_INIT;
    h = m2Hash64(h, &world->stepCount, (int32_t)sizeof(world->stepCount));
    h = m2Hash64(h, &world->gravity, (int32_t)sizeof(world->gravity));
    parts.world = h;

    h = M2_HASH_INIT;
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->bodies.transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->bodies.linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->bodies.angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->bodies.types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->bodies.asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->bodies.sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->bodies.sleepStreak[i], 1);
        h = m2Hash64(h, &world->bodies.bullets[i], (int32_t)sizeof(uint8_t));
    }
    parts.bodies = h;

    h = M2_HASH_INIT;
    h = m2Hash64(h, world->contacts.pairKeys,
                 world->contacts.pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->contacts.manifolds,
                 world->contacts.pairCount * (int32_t)sizeof(m2Manifold));
    parts.contacts = h;

    h = M2_HASH_INIT;
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        if (world->joints.jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->joints.jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->joints.jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->joints.jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    parts.joints = h;

    h = M2_HASH_INIT;
    if (world->particles.particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particles.particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particles.particleAlive[i], 1);
            if (world->particles.particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particles.particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particles.particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particles.particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particles.particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particles.particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particles.particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringA,
                     world->particles.particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringB,
                     world->particles.particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleSpringRest,
                     world->particles.particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particles.particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadA,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadB,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particles.particleTriadC,
                     world->particles.particleTriadCount * (int32_t)sizeof(int32_t));
    }
    parts.particles = h;

    return parts;
}
