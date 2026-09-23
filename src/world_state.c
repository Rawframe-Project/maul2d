// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table and the three walks built on it: allocate,
// free and snapshot. The table order is the snapshot byte order.

#include "world_state.h"

#include "contact_solver.h"
#include "contact_solver_wide.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <stddef.h>
#include <string.h>

// How many elements an array holds, as a function of the capacities.
typedef enum m2Extent
{
    m2_extent_one,
    m2_extent_body,
    m2_extent_shape,
    m2_extent_joint,
    m2_extent_jointEdge,
    m2_extent_pair,
    m2_extent_treeNode,
    m2_extent_particle,
    m2_extent_particleStage,
    m2_extent_particlePair,
    m2_extent_particleSpring,
    m2_extent_particleTriad,
    m2_extent_particleBody,
    m2_extent_fluidVolume,
    m2_extent_constraintBytes,
    m2_extent_contactBlockBytes,
} m2Extent;

#define M2_STATE_SNAPSHOT  0x01u // walked by snapshots and restores
#define M2_STATE_PARTICLES 0x02u // exists only when the world has particles
#define M2_STATE_FLUIDS    0x04u // exists only when the world has fluid volumes
#define M2_STATE_POINTER   0x08u // the field points at the array; else it is inline

typedef struct m2StateArray
{
    uint32_t offset;      // of the field in m2World
    uint32_t elementSize; // bytes per element (inline: bytes of the field)
    uint8_t extent;       // m2Extent
    uint8_t extra;        // elements allocated beyond the extent, never walked
    uint8_t flags;
} m2StateArray;

#define M2_STATE_ARRAY(field, type, ext, extraElements, stateFlags)                                \
    {(uint32_t)offsetof(m2World, field), (uint32_t)sizeof(type), m2_extent_##ext, extraElements,   \
     (uint8_t)((stateFlags) | M2_STATE_POINTER)}
#define M2_STATE_BYTES(field, bytes, ext, stateFlags)                                              \
    {(uint32_t)offsetof(m2World, field), (uint32_t)(bytes), m2_extent_##ext, 0,                    \
     (uint8_t)((stateFlags) | M2_STATE_POINTER)}
#define M2_STATE_INLINE(field, stateFlags)                                                         \
    {(uint32_t)offsetof(m2World, field), (uint32_t)sizeof(((m2World*)0)->field), m2_extent_one, 0, \
     (uint8_t)(stateFlags)}

static const m2StateArray s_state[] = {
    // Snapshot state, in snapshot byte order.
    M2_STATE_ARRAY(transforms, m2Transform, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(linearVelocities, m2Vec2, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(angularVelocities, float, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(gravityScales, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(invMass, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(invInertia, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(localCenters, m2Vec2, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(asleep, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(sleepTimes, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(sleepStreak, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bullets, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(userData, uint64_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(types, uint8_t, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(alive, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodyShapeHead, int32_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(generations, uint16_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(freeQueue, int32_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeGeometry, m2ShapeGeometry, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeDensity, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeFriction, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeRestitution, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeTangentSpeed, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeUserData, uint64_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeBody, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeNext, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeCategory, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeMask, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeGroup, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeSensor, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(proxyIds, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(inMoved, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(moved, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(maxJointIndex, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(jointFreeHead, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(jointFreeTail, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(jointFreeCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(jointRetiredCount, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointType, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointAlive, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointBodyA, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointBodyB, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLocalAnchorA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLocalAnchorB, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLength, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointHertz, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointDamping, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointHertz2, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointDamping2, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointImpulse, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointFlags, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointMotorSpeed, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointMaxMotor, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLower, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointUpper, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLocalAxisA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointRefAngle, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointMotorImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointLowerImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointUpperImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointSpringImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointBreakForce, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointCollide, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointTargets, m2Pos2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointTargetsB, m2Pos2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointUserData, uint64_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointBreakTorque, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointGenerations, uint16_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(jointFreeQueue, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(maxChainIndex, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chainFreeHead, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chainFreeTail, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chainFreeCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chainRetiredCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(lastInvH, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(sleepEnabled, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(linearDampings, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(angularDampings, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(fixedRotations, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(motionLocks, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(sleepEnables, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(forces, m2Vec2, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(torques, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(disabled, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(dominances, int8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapeChain, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chainAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chainBody, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chainGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chainFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(trees, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(treeNodes[0], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(treeNodes[1], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(treeNodes[2], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(pairKeys, uint64_t, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(pairTouching, uint8_t, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(manifolds, m2Manifold, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(particlePositions, m2Pos2, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleVelocities, m2Vec2, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleAlive, uint8_t, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleGenerations, uint16_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleFlags, uint32_t, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleLifetime, float, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleUserData, uint64_t, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleFreeQueue, int32_t, particle, 0, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particleFreeHead, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particleFreeCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particleCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(maxParticleIndex, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleSpringA, int32_t, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleSpringB, int32_t, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleSpringRest, float, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particleSpringCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadA, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadB, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadC, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadPA, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadPB, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleTriadPC, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particleTriadCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(fvLower, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvUpper, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvSurface, double, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvDensity, float, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvLinearDrag, float, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvAngularDrag, float, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvFlow, m2Vec2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvUserData, uint64_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvAlive, uint8_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvGenerations, uint16_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(fvFreeQueue, int32_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(fvFreeHead, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(fvFreeCount, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(maxFvIndex, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),

    // Scratch and derived arrays: allocated with the world, never walked.
    M2_STATE_ARRAY(ccdPrevPositions, m2Pos2, body, 0, 0),
    M2_STATE_ARRAY(islandParent, int32_t, body, 0, 0),
    M2_STATE_ARRAY(islandDisturbed, uint8_t, body, 0, 0),
    M2_STATE_ARRAY(bodyJointHead, int32_t, body, 0, 0),
    M2_STATE_ARRAY(jointEdgeNext, int32_t, jointEdge, 0, 0),
    M2_STATE_ARRAY(particlePairA, int32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particlePairB, int32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particlePairWeight, float, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particlePairFlags, uint32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particlePairNormal, m2Vec2, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleWeights, float, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleAccumulation, float, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleAccumulation2, m2Vec2, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyParticle, int32_t, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyBody, int32_t, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyWeight, float, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyNormal, m2Vec2, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyMass, float, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyStageBody, int32_t, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyStageWeight, float, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyStageNormal, m2Vec2, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyStageMass, float, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particlePairWorkCount, int32_t, particle, 1, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particleBodyStageDrops, int32_t, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(touchingScratch, uint8_t, pair, 0, 0),
    M2_STATE_ARRAY(colorMasks, uint32_t, body, 0, 0),
    M2_STATE_ARRAY(constraintColors, uint8_t, pair, 0, 0),
    M2_STATE_ARRAY(colorOrder, int32_t, pair, 0, 0),
    M2_STATE_ARRAY(beginEvents, m2ContactBeginEvent, pair, 0, 0),
    M2_STATE_ARRAY(endEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(pendingEndEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(sensorBeginEvents, m2ContactBeginEvent, pair, 0, 0),
    M2_STATE_ARRAY(sensorEndEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(pendingSensorEnd, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(jointBreakEvents, m2JointBreakEvent, joint, 0, 0),
    M2_STATE_ARRAY(pairScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(oldPairScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(pairMergeScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(manifoldScratch, m2Manifold, pair, 0, 0),
    M2_STATE_ARRAY(deltaPositions, m2Vec2, body, 1, 0),
    M2_STATE_ARRAY(deltaRotations, m2Rot, body, 1, 0),
    M2_STATE_BYTES(particleProxies, 16, particle, M2_STATE_PARTICLES),
    M2_STATE_BYTES(particleProxiesTmp, 16, particle, M2_STATE_PARTICLES),
    M2_STATE_BYTES(constraintScratch, 1, constraintBytes, 0),
    M2_STATE_BYTES(contactBlocks, 1, contactBlockBytes, 0),
};

static size_t ExtentCount(const m2World* world, uint8_t extent)
{
    switch ((m2Extent)extent)
    {
    case m2_extent_one:
        return 1;
    case m2_extent_body:
        return (size_t)world->bodyCapacity;
    case m2_extent_shape:
        return (size_t)world->shapeCapacity;
    case m2_extent_joint:
        return (size_t)world->jointCapacity;
    case m2_extent_jointEdge:
        return 2 * (size_t)world->jointCapacity;
    case m2_extent_pair:
        return (size_t)world->pairCapacity;
    case m2_extent_treeNode:
        return (size_t)world->treeNodeCapacity;
    case m2_extent_particle:
        return (size_t)world->particleCapacity;
    case m2_extent_particleStage:
        return 4 * (size_t)world->particleCapacity;
    case m2_extent_particlePair:
        return (size_t)world->particlePairCapacity;
    case m2_extent_particleSpring:
        return (size_t)world->particleSpringCapacity;
    case m2_extent_particleTriad:
        return (size_t)world->particleTriadCapacity;
    case m2_extent_particleBody:
        return (size_t)world->particleBodyCapacity;
    case m2_extent_fluidVolume:
        return (size_t)world->fvCapacity;
    case m2_extent_constraintBytes:
        return (size_t)world->pairCapacity * (size_t)m2ContactConstraintSize();
    case m2_extent_contactBlockBytes:
        return (size_t)m2ContactBlockScratchBytes(world->pairCapacity);
    }
    M2_ASSERT(false);
    return 0;
}

static bool Present(const m2World* world, uint8_t flags)
{
    if ((flags & M2_STATE_PARTICLES) != 0 && world->particleCapacity == 0)
    {
        return false;
    }
    if ((flags & M2_STATE_FLUIDS) != 0 && world->fvCapacity == 0)
    {
        return false;
    }
    return true;
}

static void** FieldPointer(m2World* world, const m2StateArray* array)
{
    return (void**)((uint8_t*)world + array->offset);
}

bool m2StateAllocate(m2World* world)
{
    bool ok = true;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if ((array->flags & M2_STATE_POINTER) == 0 || !Present(world, array->flags))
        {
            continue;
        }
        size_t bytes = (ExtentCount(world, array->extent) + array->extra) * array->elementSize;
        if (bytes == 0)
        {
            continue;
        }
        void* memory = m2AllocZeroed(bytes);
        *FieldPointer(world, array) = memory;
        world->memoryBytes += (int64_t)bytes;
        ok = ok && memory != NULL;
    }
    return ok;
}

void m2StateFree(m2World* world)
{
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if ((array->flags & M2_STATE_POINTER) != 0)
        {
            m2Free(*FieldPointer(world, array));
            *FieldPointer(world, array) = NULL;
        }
    }
}

int32_t m2StateWalk(m2World* world, uint8_t* out, const uint8_t* in, int direction)
{
    size_t cursor = 0;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if ((array->flags & M2_STATE_SNAPSHOT) == 0 || !Present(world, array->flags))
        {
            continue;
        }
        bool pointer = (array->flags & M2_STATE_POINTER) != 0;
        size_t bytes =
            pointer ? ExtentCount(world, array->extent) * array->elementSize : array->elementSize;
        void* data = pointer ? *FieldPointer(world, array) : (void*)FieldPointer(world, array);
        if (direction == 0)
        {
            memcpy(out + cursor, data, bytes);
        }
        else if (direction == 1)
        {
            memcpy(data, in + cursor, bytes);
        }
        cursor += bytes;
    }
    return (int32_t)cursor;
}
