// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Snapshots and hashing: the block walk that sizes, writes and restores
// the world state, and the world hashes built on the same walk.

#include "snapshot.h"

#include "joint.h"
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

static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction);

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
    return WalkBlocks((m2World*)world, NULL, NULL, 2);
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
static int32_t WalkBlocks(m2World* world, uint8_t* out, const uint8_t* in, int direction)
{
    int32_t cursor = 0;
    size_t cap = (size_t)world->bodyCapacity;
    size_t shapeCap = (size_t)world->shapeCapacity;
#define M2_BLOCK(ptr, bytes)                                                                       \
    do                                                                                             \
    {                                                                                              \
        if (direction == 0)                                                                        \
        {                                                                                          \
            memcpy(out + cursor, ptr, (size_t)(bytes));                                            \
        }                                                                                          \
        else if (direction == 1)                                                                   \
        {                                                                                          \
            memcpy(ptr, in + cursor, (size_t)(bytes));                                             \
        }                                                                                          \
        cursor += (int32_t)(bytes);                                                                \
    } while (0)
    M2_BLOCK(world->transforms, cap * sizeof(m2Transform));
    M2_BLOCK(world->linearVelocities, cap * sizeof(m2Vec2));
    M2_BLOCK(world->angularVelocities, cap * sizeof(float));
    M2_BLOCK(world->gravityScales, cap * sizeof(float));
    M2_BLOCK(world->invMass, cap * sizeof(float));
    M2_BLOCK(world->invInertia, cap * sizeof(float));
    M2_BLOCK(world->localCenters, cap * sizeof(m2Vec2));
    M2_BLOCK(world->asleep, cap * sizeof(uint8_t));
    M2_BLOCK(world->sleepTimes, cap * sizeof(float));
    M2_BLOCK(world->sleepStreak, cap * sizeof(uint8_t));
    M2_BLOCK(world->bullets, cap * sizeof(uint8_t));
    M2_BLOCK(world->userData, cap * sizeof(uint64_t));
    M2_BLOCK(world->types, cap * sizeof(uint8_t));
    M2_BLOCK(world->alive, cap * sizeof(uint8_t));
    M2_BLOCK(world->bodyShapeHead, cap * sizeof(int32_t));
    M2_BLOCK(world->generations, cap * sizeof(uint16_t));
    M2_BLOCK(world->freeQueue, cap * sizeof(int32_t));
    M2_BLOCK(world->shapeGeometry, shapeCap * sizeof(m2ShapeGeometry));
    M2_BLOCK(world->shapeDensity, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeFriction, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeRestitution, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeTangentSpeed, shapeCap * sizeof(float));
    M2_BLOCK(world->shapeUserData, shapeCap * sizeof(uint64_t));
    M2_BLOCK(world->shapeBody, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeNext, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->shapeAlive, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->shapeGenerations, shapeCap * sizeof(uint16_t));
    M2_BLOCK(world->shapeCategory, (size_t)world->shapeCapacity * sizeof(uint32_t));
    M2_BLOCK(world->shapeMask, (size_t)world->shapeCapacity * sizeof(uint32_t));
    M2_BLOCK(world->shapeGroup, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->shapeSensor, (size_t)world->shapeCapacity * sizeof(uint8_t));
    M2_BLOCK(world->shapeFreeQueue, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->proxyIds, shapeCap * sizeof(int32_t));
    M2_BLOCK(world->inMoved, shapeCap * sizeof(uint8_t));
    M2_BLOCK(world->moved, shapeCap * sizeof(int32_t));
    M2_BLOCK(&world->maxJointIndex, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeHead, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeTail, sizeof(int32_t));
    M2_BLOCK(&world->jointFreeCount, sizeof(int32_t));
    M2_BLOCK(&world->jointRetiredCount, sizeof(int32_t));
    M2_BLOCK(world->jointType, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointAlive, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointBodyA, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointBodyB, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(world->jointLocalAnchorA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLocalAnchorB, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointLength, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointHertz, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointDamping, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointHertz2, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointDamping2, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointImpulse, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointFlags, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointMotorSpeed, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMaxMotor, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLower, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpper, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLocalAxisA, (size_t)world->jointCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->jointRefAngle, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointMotorImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointLowerImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointUpperImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointSpringImpulse, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointBreakForce, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointCollide, (size_t)world->jointCapacity * sizeof(uint8_t));
    M2_BLOCK(world->jointTargets, (size_t)world->jointCapacity * sizeof(m2Pos2));
    M2_BLOCK(world->jointTargetsB, (size_t)world->jointCapacity * sizeof(m2Pos2));
    M2_BLOCK(world->jointUserData, (size_t)world->jointCapacity * sizeof(uint64_t));
    M2_BLOCK(world->jointBreakTorque, (size_t)world->jointCapacity * sizeof(float));
    M2_BLOCK(world->jointGenerations, (size_t)world->jointCapacity * sizeof(uint16_t));
    M2_BLOCK(world->jointFreeQueue, (size_t)world->jointCapacity * sizeof(int32_t));
    M2_BLOCK(&world->maxChainIndex, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeHead, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeTail, sizeof(int32_t));
    M2_BLOCK(&world->chainFreeCount, sizeof(int32_t));
    M2_BLOCK(&world->chainRetiredCount, sizeof(int32_t));
    M2_BLOCK(&world->lastInvH, sizeof(float));
    M2_BLOCK(&world->sleepEnabled, sizeof(uint8_t));
    M2_BLOCK(world->linearDampings, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->angularDampings, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->fixedRotations, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->motionLocks, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->sleepEnables, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->forces, (size_t)world->bodyCapacity * sizeof(m2Vec2));
    M2_BLOCK(world->torques, (size_t)world->bodyCapacity * sizeof(float));
    M2_BLOCK(world->disabled, (size_t)world->bodyCapacity * sizeof(uint8_t));
    M2_BLOCK(world->dominances, (size_t)world->bodyCapacity * sizeof(int8_t));
    M2_BLOCK(world->shapeChain, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->chainAlive, (size_t)world->shapeCapacity * sizeof(uint8_t));
    M2_BLOCK(world->chainBody, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->chainGenerations, (size_t)world->shapeCapacity * sizeof(uint16_t));
    M2_BLOCK(world->chainFreeQueue, (size_t)world->shapeCapacity * sizeof(int32_t));
    M2_BLOCK(world->trees, M2_TREE_COUNT * sizeof(m2DynamicTree));
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        M2_BLOCK(world->treeNodes[t], (size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
    }
    M2_BLOCK(world->pairKeys, (size_t)world->pairCapacity * sizeof(uint64_t));
    M2_BLOCK(world->pairTouching, (size_t)world->pairCapacity * sizeof(uint8_t));
    M2_BLOCK(world->manifolds, (size_t)world->pairCapacity * sizeof(m2Manifold));
    if (world->particleCapacity > 0)
    {
        size_t particleCap = (size_t)world->particleCapacity;
        M2_BLOCK(world->particlePositions, particleCap * sizeof(m2Pos2));
        M2_BLOCK(world->particleVelocities, particleCap * sizeof(m2Vec2));
        M2_BLOCK(world->particleAlive, particleCap * sizeof(uint8_t));
        M2_BLOCK(world->particleGenerations, particleCap * sizeof(uint16_t));
        M2_BLOCK(world->particleFlags, particleCap * sizeof(uint32_t));
        M2_BLOCK(world->particleLifetime, particleCap * sizeof(float));
        M2_BLOCK(world->particleUserData, particleCap * sizeof(uint64_t));
        M2_BLOCK(world->particleFreeQueue, particleCap * sizeof(int32_t));
        M2_BLOCK(&world->particleFreeHead, sizeof(int32_t));
        M2_BLOCK(&world->particleFreeCount, sizeof(int32_t));
        M2_BLOCK(&world->particleCount, sizeof(int32_t));
        M2_BLOCK(&world->maxParticleIndex, sizeof(int32_t));
        M2_BLOCK(world->particleSpringA, (size_t)world->particleSpringCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleSpringB, (size_t)world->particleSpringCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleSpringRest, (size_t)world->particleSpringCapacity * sizeof(float));
        M2_BLOCK(&world->particleSpringCount, sizeof(int32_t));
        M2_BLOCK(world->particleTriadA, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadB, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadC, (size_t)world->particleTriadCapacity * sizeof(int32_t));
        M2_BLOCK(world->particleTriadPA, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(world->particleTriadPB, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(world->particleTriadPC, (size_t)world->particleTriadCapacity * sizeof(m2Vec2));
        M2_BLOCK(&world->particleTriadCount, sizeof(int32_t));
    }
    if (world->fvCapacity > 0)
    {
        size_t fvCap = (size_t)world->fvCapacity;
        M2_BLOCK(world->fvLower, fvCap * sizeof(m2Pos2));
        M2_BLOCK(world->fvUpper, fvCap * sizeof(m2Pos2));
        M2_BLOCK(world->fvSurface, fvCap * sizeof(double));
        M2_BLOCK(world->fvDensity, fvCap * sizeof(float));
        M2_BLOCK(world->fvLinearDrag, fvCap * sizeof(float));
        M2_BLOCK(world->fvAngularDrag, fvCap * sizeof(float));
        M2_BLOCK(world->fvFlow, fvCap * sizeof(m2Vec2));
        M2_BLOCK(world->fvUserData, fvCap * sizeof(uint64_t));
        M2_BLOCK(world->fvAlive, fvCap * sizeof(uint8_t));
        M2_BLOCK(world->fvGenerations, fvCap * sizeof(uint16_t));
        M2_BLOCK(world->fvFreeQueue, fvCap * sizeof(int32_t));
        M2_BLOCK(&world->fvFreeHead, sizeof(int32_t));
        M2_BLOCK(&world->fvFreeCount, sizeof(int32_t));
        M2_BLOCK(&world->maxFvIndex, sizeof(int32_t));
    }
#undef M2_BLOCK
    return cursor;
}

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
    header.bodyCapacity = world->bodyCapacity;
    header.maxBodyIndex = world->maxBodyIndex;
    header.stepCount = world->stepCount;
    header.gravity = world->gravity;
    header.windVelocity = world->windVelocity;
    header.windLinearDrag = world->windLinearDrag;
    header.windReserved = 0;
    header.freeHead = world->freeHead;
    header.freeTail = world->freeTail;
    header.freeCount = world->freeCount;
    header.retiredCount = world->retiredCount;
    header.movedCount = world->movedCount;
    header.pairCount = world->pairCount;
    header.shapeCapacity = world->shapeCapacity;
    header.maxShapeIndex = world->maxShapeIndex;
    header.shapeFreeHead = world->shapeFreeHead;
    header.shapeFreeTail = world->shapeFreeTail;
    header.shapeFreeCount = world->shapeFreeCount;
    header.shapeRetiredCount = world->shapeRetiredCount;

    uint8_t* out = buffer;
    memcpy(out, &header, sizeof(header));
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, out + sizeof(header), NULL, 0);
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
        header.bodyCapacity != world->bodyCapacity ||
        header.shapeCapacity != world->shapeCapacity ||
        size != (int32_t)sizeof(header) + BlockBytes(world) || !HeaderCountsInRange(&header))
    {
        return false;
    }

    world->maxBodyIndex = header.maxBodyIndex;
    world->stepCount = header.stepCount;
    world->gravity = header.gravity;
    world->windVelocity = header.windVelocity;
    world->windLinearDrag = header.windLinearDrag;
    world->freeHead = header.freeHead;
    world->freeTail = header.freeTail;
    world->freeCount = header.freeCount;
    world->retiredCount = header.retiredCount;
    world->movedCount = header.movedCount;
    world->pairCount = header.pairCount;
    world->maxShapeIndex = header.maxShapeIndex;
    world->shapeFreeHead = header.shapeFreeHead;
    world->shapeFreeTail = header.shapeFreeTail;
    world->shapeFreeCount = header.shapeFreeCount;
    world->shapeRetiredCount = header.shapeRetiredCount;

    const uint8_t* in = buffer;
    int32_t cursor = (int32_t)sizeof(header) + WalkBlocks(world, NULL, in + sizeof(header), 1);
    M2_ASSERT(cursor == size);
    (void)cursor;
    m2RebuildJointEdges(world);

    // Restores are first-class journal citizens: the tape carries the
    // snapshot itself, so rollback-heavy sessions replay bit-exactly.
    // The price is tape size, and that is the caller's tradeoff.
    m2JournalRecordRestore(world, buffer, size);

    // Events are an observer stream from an abandoned timeline: cleared
    // on restore, re-emitted by re-simulation.
    world->beginEventCount = 0;
    world->endEventCount = 0;
    world->pendingEndCount = 0;
    world->sensorBeginCount = 0;
    world->sensorEndCount = 0;
    world->pendingSensorEndCount = 0;
    world->jointBreakEventCount = 0;
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
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->sleepStreak[i], 1);
        h = m2Hash64(h, &world->bullets[i], (int32_t)sizeof(uint8_t));
    }
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->manifolds, world->pairCount * (int32_t)sizeof(m2Manifold));
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    if (world->fvCapacity > 0)
    {
        for (int32_t i = 0; i < world->maxFvIndex; ++i)
        {
            h = m2Hash64(h, &world->fvAlive[i], 1);
            if (world->fvAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->fvSurface[i], (int32_t)sizeof(double));
        }
    }
    if (world->particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particleAlive[i], 1);
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringA,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringB,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringRest,
                     world->particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadA,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadB,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadC,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
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
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->transforms[i], (int32_t)sizeof(m2Transform));
        h = m2Hash64(h, &world->linearVelocities[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->angularVelocities[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invMass[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->invInertia[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->localCenters[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->types[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->asleep[i], (int32_t)sizeof(uint8_t));
        h = m2Hash64(h, &world->sleepTimes[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->sleepStreak[i], 1);
        h = m2Hash64(h, &world->bullets[i], (int32_t)sizeof(uint8_t));
    }
    parts.bodies = h;

    h = M2_HASH_INIT;
    h = m2Hash64(h, world->pairKeys, world->pairCount * (int32_t)sizeof(uint64_t));
    h = m2Hash64(h, world->manifolds, world->pairCount * (int32_t)sizeof(m2Manifold));
    parts.contacts = h;

    h = M2_HASH_INIT;
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        h = m2Hash64(h, &world->jointImpulse[i], (int32_t)sizeof(m2Vec2));
        h = m2Hash64(h, &world->jointMotorImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointLowerImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointUpperImpulse[i], (int32_t)sizeof(float));
        h = m2Hash64(h, &world->jointSpringImpulse[i], (int32_t)sizeof(float));
    }
    parts.joints = h;

    h = M2_HASH_INIT;
    if (world->particleCapacity > 0)
    {
        h = m2Hash64(h, &world->particleCount, (int32_t)sizeof(int32_t));
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            h = m2Hash64(h, &world->particleAlive[i], 1);
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &world->particlePositions[i], (int32_t)sizeof(m2Pos2));
            h = m2Hash64(h, &world->particleVelocities[i], (int32_t)sizeof(m2Vec2));
            h = m2Hash64(h, &world->particleFlags[i], (int32_t)sizeof(uint32_t));
            h = m2Hash64(h, &world->particleLifetime[i], (int32_t)sizeof(float));
            h = m2Hash64(h, &world->particleUserData[i], (int32_t)sizeof(uint64_t));
        }
        h = m2Hash64(h, &world->particleSpringCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringA,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringB,
                     world->particleSpringCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleSpringRest,
                     world->particleSpringCount * (int32_t)sizeof(float));
        h = m2Hash64(h, &world->particleTriadCount, (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadA,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadB,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
        h = m2Hash64(h, world->particleTriadC,
                     world->particleTriadCount * (int32_t)sizeof(int32_t));
    }
    parts.particles = h;

    return parts;
}
