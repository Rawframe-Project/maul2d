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
    M2_STATE_ARRAY(bodies.transforms, m2Transform, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.linearVelocities, m2Vec2, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.angularVelocities, float, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.gravityScales, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.invMass, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.invInertia, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.localCenters, m2Vec2, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.asleep, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.sleepTimes, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.sleepStreak, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.bullets, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.userData, uint64_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.types, uint8_t, body, 1, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.alive, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.bodyShapeHead, int32_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.generations, uint16_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.freeQueue, int32_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeGeometry, m2ShapeGeometry, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeDensity, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeFriction, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeRestitution, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeTangentSpeed, float, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeUserData, uint64_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeBody, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeNext, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeCategory, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeMask, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeGroup, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeSensor, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.proxyIds, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.inMoved, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.moved, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(joints.maxJointIndex, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(joints.jointFreeHead, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(joints.jointFreeTail, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(joints.jointFreeCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(joints.jointRetiredCount, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointType, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointAlive, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointBodyA, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointBodyB, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLocalAnchorA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLocalAnchorB, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLength, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointHertz, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointDamping, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointHertz2, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointDamping2, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointImpulse, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointFlags, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointMotorSpeed, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointMaxMotor, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLower, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointUpper, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLocalAxisA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointRefAngle, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointMotorImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointLowerImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointUpperImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointSpringImpulse, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointBreakForce, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointCollide, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointTargets, m2Pos2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointTargetsB, m2Pos2, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointUserData, uint64_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointBreakTorque, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointGenerations, uint16_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointFreeQueue, int32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chains.maxChainIndex, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chains.chainFreeHead, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chains.chainFreeTail, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chains.chainFreeCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(chains.chainRetiredCount, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(lastInvH, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(sleepEnabled, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.linearDampings, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.angularDampings, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.fixedRotations, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.motionLocks, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.sleepEnables, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.forces, m2Vec2, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.torques, float, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.disabled, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(bodies.dominances, int8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeChain, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chains.chainAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chains.chainBody, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chains.chainGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(chains.chainFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_INLINE(broadphase.trees, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.treeNodes[0], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.treeNodes[1], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(broadphase.treeNodes[2], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(contacts.pairKeys, uint64_t, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(contacts.pairTouching, uint8_t, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(contacts.manifolds, m2Manifold, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(particles.particlePositions, m2Pos2, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleVelocities, m2Vec2, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleAlive, uint8_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleGenerations, uint16_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleFlags, uint32_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleLifetime, float, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleUserData, uint64_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleFreeQueue, int32_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.particleFreeHead, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.particleFreeCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.particleCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.maxParticleIndex, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleSpringA, int32_t, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleSpringB, int32_t, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleSpringRest, float, particleSpring, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.particleSpringCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadA, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadB, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadC, int32_t, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadPA, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadPB, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleTriadPC, m2Vec2, particleTriad, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_INLINE(particles.particleTriadCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(volumes.fvLower, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvUpper, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvSurface, double, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvDensity, float, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvLinearDrag, float, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvAngularDrag, float, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvFlow, m2Vec2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvUserData, uint64_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvAlive, uint8_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvGenerations, uint16_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_ARRAY(volumes.fvFreeQueue, int32_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(volumes.fvFreeHead, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(volumes.fvFreeCount, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_INLINE(volumes.maxFvIndex, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),

    // Scratch and derived arrays: allocated with the world, never walked.
    M2_STATE_ARRAY(solver.ccdPrevPositions, m2Pos2, body, 0, 0),
    M2_STATE_ARRAY(solver.islandParent, int32_t, body, 0, 0),
    M2_STATE_ARRAY(solver.islandDisturbed, uint8_t, body, 0, 0),
    M2_STATE_ARRAY(joints.bodyJointHead, int32_t, body, 0, 0),
    M2_STATE_ARRAY(joints.jointEdgeNext, int32_t, jointEdge, 0, 0),
    M2_STATE_ARRAY(particles.particlePairA, int32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particlePairB, int32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particlePairWeight, float, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particlePairFlags, uint32_t, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particlePairNormal, m2Vec2, particlePair, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleWeights, float, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleAccumulation, float, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleAccumulation2, m2Vec2, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyParticle, int32_t, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyBody, int32_t, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyWeight, float, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyNormal, m2Vec2, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyMass, float, particleBody, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyStageBody, int32_t, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyStageWeight, float, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyStageNormal, m2Vec2, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyStageMass, float, particleStage, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particlePairWorkCount, int32_t, particle, 1, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleBodyStageDrops, int32_t, particle, 0, M2_STATE_PARTICLES),
    M2_STATE_ARRAY(solver.touchingScratch, uint8_t, pair, 0, 0),
    M2_STATE_ARRAY(solver.colorMasks, uint32_t, body, 0, 0),
    M2_STATE_ARRAY(solver.constraintColors, uint8_t, pair, 0, 0),
    M2_STATE_ARRAY(solver.colorOrder, int32_t, pair, 0, 0),
    M2_STATE_ARRAY(events.beginEvents, m2ContactBeginEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.endEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.pendingEndEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.sensorBeginEvents, m2ContactBeginEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.sensorEndEvents, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.pendingSensorEnd, m2ContactEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.jointBreakEvents, m2JointBreakEvent, joint, 0, 0),
    M2_STATE_ARRAY(contacts.pairScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(contacts.oldPairScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(contacts.pairMergeScratch, uint64_t, pair, 0, 0),
    M2_STATE_ARRAY(contacts.manifoldScratch, m2Manifold, pair, 0, 0),
    M2_STATE_ARRAY(solver.deltaPositions, m2Vec2, body, 1, 0),
    M2_STATE_ARRAY(solver.deltaRotations, m2Rot, body, 1, 0),
    M2_STATE_BYTES(particles.particleProxies, 16, particle, M2_STATE_PARTICLES),
    M2_STATE_BYTES(particles.particleProxiesTmp, 16, particle, M2_STATE_PARTICLES),
    M2_STATE_BYTES(solver.constraintScratch, 1, constraintBytes, 0),
    M2_STATE_BYTES(solver.contactBlocks, 1, contactBlockBytes, 0),
};

static size_t ExtentCount(const m2World* world, uint8_t extent)
{
    switch ((m2Extent)extent)
    {
    case m2_extent_one:
        return 1;
    case m2_extent_body:
        return (size_t)world->bodies.bodyCapacity;
    case m2_extent_shape:
        return (size_t)world->shapes.shapeCapacity;
    case m2_extent_joint:
        return (size_t)world->joints.jointCapacity;
    case m2_extent_jointEdge:
        return 2 * (size_t)world->joints.jointCapacity;
    case m2_extent_pair:
        return (size_t)world->contacts.pairCapacity;
    case m2_extent_treeNode:
        return (size_t)world->broadphase.treeNodeCapacity;
    case m2_extent_particle:
        return (size_t)world->particles.particleCapacity;
    case m2_extent_particleStage:
        return 4 * (size_t)world->particles.particleCapacity;
    case m2_extent_particlePair:
        return (size_t)world->particles.particlePairCapacity;
    case m2_extent_particleSpring:
        return (size_t)world->particles.particleSpringCapacity;
    case m2_extent_particleTriad:
        return (size_t)world->particles.particleTriadCapacity;
    case m2_extent_particleBody:
        return (size_t)world->particles.particleBodyCapacity;
    case m2_extent_fluidVolume:
        return (size_t)world->volumes.fvCapacity;
    case m2_extent_constraintBytes:
        return (size_t)world->contacts.pairCapacity * (size_t)m2ContactConstraintSize();
    case m2_extent_contactBlockBytes:
        return (size_t)m2ContactBlockScratchBytes(world->contacts.pairCapacity);
    }
    M2_ASSERT(false);
    return 0;
}

static bool Present(const m2World* world, uint8_t flags)
{
    if ((flags & M2_STATE_PARTICLES) != 0 && world->particles.particleCapacity == 0)
    {
        return false;
    }
    if ((flags & M2_STATE_FLUIDS) != 0 && world->volumes.fvCapacity == 0)
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
