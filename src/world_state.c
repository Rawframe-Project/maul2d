// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's state table and the three walks built on it: allocate,
// free and snapshot. The table order is the snapshot byte order.

#include "world_state.h"

#include "contact_solver.h"
#include "state_checks.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <stdckdint.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// How many elements an array holds, as a function of the capacities.
typedef enum m2Extent
{
    m2_extent_one,
    m2_extent_body,
    m2_extent_shape,
    m2_extent_chain, // chain slots: as many as shape slots
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
    m2_extent_markBytes,
} m2Extent;

#define M2_STATE_SNAPSHOT  0x01u // walked by snapshots and restores
#define M2_STATE_PARTICLES 0x02u // exists only when the world has particles
#define M2_STATE_FLUIDS    0x04u // exists only when the world has fluid volumes
#define M2_STATE_POINTER   0x08u // the field points at the array; else it is inline
#define M2_STATE_HASH      0x10u // folded into m2World_Hash (m2StateHash)

typedef struct m2StateArray
{
    uint32_t offset;      // of the field in m2World
    uint32_t elementSize; // bytes per element (inline: bytes of the field)
    uint8_t extent;       // m2Extent
    uint8_t extra;        // elements allocated beyond the extent, never walked
    uint8_t flags;
    uint8_t check; // m2Check, validated on restore
    uint8_t range; // m2Extent the checked values index into
} m2StateArray;

#define M2_STATE_ARRAY(field, type, ext, extraElements, stateFlags)                                \
    {(uint32_t)offsetof(m2World, field),                                                           \
     (uint32_t)sizeof(type),                                                                       \
     m2_extent_##ext,                                                                              \
     extraElements,                                                                                \
     (uint8_t)((stateFlags) | M2_STATE_POINTER),                                                   \
     m2_check_none,                                                                                \
     m2_extent_one}
#define M2_STATE_CHECKED(field, type, ext, extraElements, stateFlags, checkKind, rangeExtent)      \
    {(uint32_t)offsetof(m2World, field),                                                           \
     (uint32_t)sizeof(type),                                                                       \
     m2_extent_##ext,                                                                              \
     extraElements,                                                                                \
     (uint8_t)((stateFlags) | M2_STATE_POINTER),                                                   \
     m2_check_##checkKind,                                                                         \
     m2_extent_##rangeExtent}
#define M2_STATE_BYTES(field, bytes, ext, stateFlags)                                              \
    {(uint32_t)offsetof(m2World, field),         (uint32_t)(bytes), m2_extent_##ext, 0,            \
     (uint8_t)((stateFlags) | M2_STATE_POINTER), m2_check_none,     m2_extent_one}
#define M2_STATE_INLINE(field, stateFlags)                                                         \
    {(uint32_t)offsetof(m2World, field),                                                           \
     (uint32_t)sizeof(((m2World*)0)->field),                                                       \
     m2_extent_one,                                                                                \
     0,                                                                                            \
     (uint8_t)(stateFlags),                                                                        \
     m2_check_none,                                                                                \
     m2_extent_one}
#define M2_STATE_INLINE_CHECKED(field, stateFlags, checkKind, rangeExtent)                         \
    {(uint32_t)offsetof(m2World, field),                                                           \
     (uint32_t)sizeof(((m2World*)0)->field),                                                       \
     m2_extent_one,                                                                                \
     0,                                                                                            \
     (uint8_t)(stateFlags),                                                                        \
     m2_check_##checkKind,                                                                         \
     m2_extent_##rangeExtent}

static const m2StateArray s_state[] = {
    // Snapshot state, in snapshot byte order: the world scalars and the
    // pool cursors first, each checked before any byte lands.
    M2_STATE_INLINE(stepCount, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_INLINE_CHECKED(gravity, M2_STATE_SNAPSHOT | M2_STATE_HASH, acceleration, one),
    M2_STATE_INLINE_CHECKED(windVelocity, M2_STATE_SNAPSHOT | M2_STATE_HASH, speed, one),
    M2_STATE_INLINE_CHECKED(windLinearDrag, M2_STATE_SNAPSHOT | M2_STATE_HASH, rate, one),
    M2_STATE_INLINE_CHECKED(bodies.maxBodyIndex, M2_STATE_SNAPSHOT, count, body),
    M2_STATE_INLINE_CHECKED(bodies.freeHead, M2_STATE_SNAPSHOT, cursor, body),
    M2_STATE_INLINE_CHECKED(bodies.freeTail, M2_STATE_SNAPSHOT, cursor, body),
    M2_STATE_INLINE_CHECKED(bodies.freeCount, M2_STATE_SNAPSHOT, count, body),
    M2_STATE_INLINE_CHECKED(bodies.retiredCount, M2_STATE_SNAPSHOT, count, body),
    M2_STATE_INLINE_CHECKED(shapes.maxShapeIndex, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(shapes.shapeFreeHead, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(shapes.shapeFreeTail, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(shapes.shapeFreeCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(shapes.shapeRetiredCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(broadphase.movedCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(contacts.pairCount, M2_STATE_SNAPSHOT, count, pair),
    M2_STATE_CHECKED(bodies.transforms, m2Transform, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     transform, one),
    M2_STATE_CHECKED(bodies.linearVelocities, m2Vec2, body, 1, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     speed, one),
    M2_STATE_CHECKED(bodies.angularVelocities, float, body, 1, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     speed, one),
    M2_STATE_CHECKED(bodies.gravityScales, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     gravityScale, one),
    M2_STATE_CHECKED(bodies.invMass, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, finite32,
                     one),
    M2_STATE_CHECKED(bodies.invInertia, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, finite32,
                     one),
    M2_STATE_CHECKED(bodies.localCenters, m2Vec2, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, local,
                     one),
    M2_STATE_CHECKED(bodies.asleep, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool, one),
    M2_STATE_CHECKED(bodies.sleepTimes, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, finite32,
                     one),
    M2_STATE_ARRAY(bodies.sleepStreak, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(bodies.bullets, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool,
                     one),
    M2_STATE_ARRAY(bodies.userData, uint64_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.types, uint8_t, body, 1, M2_STATE_SNAPSHOT | M2_STATE_HASH, bodyType,
                     one),
    M2_STATE_CHECKED(bodies.alive, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(bodies.bodyShapeHead, int32_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, shape),
    M2_STATE_ARRAY(bodies.generations, uint32_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.freeQueue, int32_t, body, 0, M2_STATE_SNAPSHOT, index, body),
    M2_STATE_CHECKED(shapes.shapeGeometry, m2ShapeGeometry, shape, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH, geometry, one),
    M2_STATE_CHECKED(shapes.shapeDensity, float, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     density, one),
    M2_STATE_CHECKED(shapes.shapeFriction, float, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     coefficient, one),
    M2_STATE_CHECKED(shapes.shapeRestitution, float, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     restitution, one),
    M2_STATE_CHECKED(shapes.shapeTangentSpeed, float, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     speed, one),
    M2_STATE_ARRAY(shapes.shapeUserData, uint64_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(shapes.shapeBody, int32_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, body),
    M2_STATE_CHECKED(shapes.shapeNext, int32_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, shape),
    M2_STATE_CHECKED(shapes.shapeAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(shapes.shapeGenerations, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeCategory, uint64_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_ARRAY(shapes.shapeMask, uint64_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_ARRAY(shapes.shapeGroup, int32_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(shapes.shapeSensor, uint8_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool,
                     one),
    M2_STATE_CHECKED(shapes.shapeFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_CHECKED(broadphase.proxyIds, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone,
                     treeNode),
    M2_STATE_CHECKED(broadphase.inMoved, uint8_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool,
                     one),
    M2_STATE_CHECKED(broadphase.moved, int32_t, shape, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_INLINE_CHECKED(joints.maxJointIndex, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeHead, M2_STATE_SNAPSHOT, cursor, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeTail, M2_STATE_SNAPSHOT, cursor, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeCount, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_INLINE_CHECKED(joints.jointRetiredCount, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_CHECKED(joints.jointType, uint8_t, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     jointType, one),
    M2_STATE_CHECKED(joints.jointAlive, uint8_t, joint, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(joints.jointBodyA, int32_t, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, body),
    M2_STATE_CHECKED(joints.jointBodyB, int32_t, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, body),
    M2_STATE_CHECKED(joints.jointLocalAnchorA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     local, one),
    M2_STATE_CHECKED(joints.jointLocalAnchorB, m2Vec2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     local, one),
    M2_STATE_CHECKED(joints.jointLength, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite32, one),
    M2_STATE_CHECKED(joints.jointHertz, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, rate,
                     one),
    M2_STATE_CHECKED(joints.jointDamping, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     coefficient, one),
    M2_STATE_CHECKED(joints.jointHertz2, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, rate,
                     one),
    M2_STATE_CHECKED(joints.jointDamping2, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     coefficient, one),
    M2_STATE_CHECKED(joints.jointImpulse, m2Vec2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     impulse, one),
    M2_STATE_ARRAY(joints.jointFlags, uint8_t, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(joints.jointMotorSpeed, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     speed, one),
    M2_STATE_CHECKED(joints.jointMaxMotor, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     maxForce, one),
    M2_STATE_CHECKED(joints.jointLower, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite32, one),
    M2_STATE_CHECKED(joints.jointUpper, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite32, one),
    M2_STATE_CHECKED(joints.jointLocalAxisA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     local, one),
    M2_STATE_CHECKED(joints.jointRefAngle, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite32, one),
    M2_STATE_CHECKED(joints.jointMotorImpulse, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     impulse, one),
    M2_STATE_CHECKED(joints.jointLowerImpulse, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     impulse, one),
    M2_STATE_CHECKED(joints.jointUpperImpulse, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     impulse, one),
    M2_STATE_CHECKED(joints.jointSpringImpulse, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     impulse, one),
    M2_STATE_CHECKED(joints.jointBreakForce, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     breakLimit, one),
    M2_STATE_CHECKED(joints.jointCollide, uint8_t, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     bool, one),
    M2_STATE_CHECKED(joints.jointTargets, m2Pos2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite64, one),
    M2_STATE_CHECKED(joints.jointTargetsB, m2Pos2, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     finite64, one),
    M2_STATE_ARRAY(joints.jointUserData, uint64_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(joints.jointBreakTorque, float, joint, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     breakLimit, one),
    M2_STATE_ARRAY(joints.jointGenerations, uint32_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(joints.jointFreeQueue, int32_t, joint, 0, M2_STATE_SNAPSHOT, index, joint),
    M2_STATE_INLINE_CHECKED(chains.maxChainIndex, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeHead, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeTail, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(chains.chainRetiredCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(lastInvH, M2_STATE_SNAPSHOT | M2_STATE_HASH, inverseStep, one),
    M2_STATE_INLINE_CHECKED(sleepEnabled, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool, one),
    M2_STATE_CHECKED(bodies.linearDampings, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, rate,
                     one),
    M2_STATE_CHECKED(bodies.angularDampings, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     rate, one),
    M2_STATE_CHECKED(bodies.fixedRotations, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     bool, one),
    M2_STATE_ARRAY(bodies.motionLocks, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(bodies.sleepEnables, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool,
                     one),
    M2_STATE_CHECKED(bodies.forces, m2Vec2, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, force, one),
    M2_STATE_CHECKED(bodies.torques, float, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, force, one),
    M2_STATE_CHECKED(bodies.disabled, uint8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH, bool,
                     one),
    M2_STATE_ARRAY(bodies.dominances, int8_t, body, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(bodies.massOverrides, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(shapes.shapeChain, int32_t, shape, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, shape),
    M2_STATE_CHECKED(chains.chainAlive, uint8_t, chain, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(chains.chainBody, int32_t, chain, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     indexOrNone, body),
    M2_STATE_CHECKED(chains.chainLoop, uint8_t, chain, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(chains.chainGenerations, uint32_t, chain, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(chains.chainFreeQueue, int32_t, chain, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_INLINE_CHECKED(broadphase.trees, M2_STATE_SNAPSHOT, tree, treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[0], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[1], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[2], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(contacts.pairKeys, uint64_t, pair, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     pairKey, shape),
    M2_STATE_ARRAY(contacts.pairTouching, uint8_t, pair, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH),
    M2_STATE_CHECKED(contacts.manifolds, m2Manifold, pair, 0, M2_STATE_SNAPSHOT | M2_STATE_HASH,
                     manifold, one),
    M2_STATE_CHECKED(particles.particlePositions, m2Pos2, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite64, one),
    M2_STATE_CHECKED(particles.particleVelocities, m2Vec2, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, speed, one),
    M2_STATE_CHECKED(particles.particleAlive, uint8_t, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, bool, one),
    M2_STATE_ARRAY(particles.particleGenerations, uint32_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleFlags, uint32_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES),
    M2_STATE_CHECKED(particles.particleLifetime, float, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_ARRAY(particles.particleUserData, uint64_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_CHECKED(particles.particleFreeQueue, int32_t, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_INLINE_CHECKED(particles.particleFreeHead, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            cursor, particle),
    M2_STATE_INLINE_CHECKED(particles.particleFreeCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particle),
    M2_STATE_INLINE_CHECKED(particles.particleRetiredCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particle),
    M2_STATE_INLINE_CHECKED(particles.particleCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, count,
                            particle),
    M2_STATE_INLINE_CHECKED(particles.maxParticleIndex, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particle),
    M2_STATE_CHECKED(particles.particleSpringA, int32_t, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleSpringB, int32_t, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleSpringRest, float, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_INLINE_CHECKED(particles.particleSpringCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particleSpring),
    M2_STATE_CHECKED(particles.particleTriadA, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadB, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadC, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadPA, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_CHECKED(particles.particleTriadPB, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_CHECKED(particles.particleTriadPC, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_INLINE_CHECKED(particles.particleTriadCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particleTriad),
    M2_STATE_CHECKED(volumes.fvLower, m2Pos2, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, finite64, one),
    M2_STATE_CHECKED(volumes.fvUpper, m2Pos2, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, finite64, one),
    M2_STATE_CHECKED(volumes.fvSurface, double, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, finite64, one),
    M2_STATE_CHECKED(volumes.fvDensity, float, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, density, one),
    M2_STATE_CHECKED(volumes.fvLinearDrag, float, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, rate, one),
    M2_STATE_CHECKED(volumes.fvAngularDrag, float, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, rate, one),
    M2_STATE_CHECKED(volumes.fvFlow, m2Vec2, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_HASH | M2_STATE_FLUIDS, speed, one),
    M2_STATE_ARRAY(volumes.fvUserData, uint64_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_CHECKED(volumes.fvAlive, uint8_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     bool, one),
    M2_STATE_ARRAY(volumes.fvGenerations, uint32_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_CHECKED(volumes.fvFreeQueue, int32_t, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, index, fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.fvFreeHead, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, cursor,
                            fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.fvFreeCount, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, count,
                            fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.fvRetiredCount, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, count,
                            fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.maxFvIndex, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, count,
                            fluidVolume),

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
    M2_STATE_ARRAY(events.sensorBeginEvents, m2SensorBeginEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.sensorEndEvents, m2SensorEndEvent, pair, 0, 0),
    M2_STATE_ARRAY(events.pendingSensorEnd, m2SensorEndEvent, pair, 0, 0),
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
    M2_STATE_BYTES(restoreMarks, 1, markBytes, 0),
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
    case m2_extent_chain:
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
    {
        // Saturates instead of wrapping on 32-bit targets, so the checked
        // sum in ArrayBytes refuses it.
        size_t bytes = 0;
        bool over = ckd_mul(&bytes, (size_t)world->contacts.pairCapacity,
                            (size_t)m2ContactConstraintSize());
        return over ? SIZE_MAX : bytes;
    }
    case m2_extent_contactBlockBytes:
        return m2ContactBlockScratchBytes(world->contacts.pairCapacity);
    case m2_extent_markBytes:
    {
        int32_t most = world->bodies.bodyCapacity;
        const int32_t others[] = {world->shapes.shapeCapacity, world->joints.jointCapacity,
                                  world->particles.particleCapacity, world->volumes.fvCapacity};
        for (size_t i = 0; i < sizeof(others) / sizeof(others[0]); ++i)
        {
            most = others[i] > most ? others[i] : most;
        }
        return (size_t)most;
    }
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

// The bytes an allocated array takes, or false when the size does not
// fit a size_t.
static bool ArrayBytes(const m2World* world, const m2StateArray* array, size_t* bytes)
{
    size_t count = 0;
    return !ckd_add(&count, ExtentCount(world, array->extent), (size_t)array->extra) &&
           !ckd_mul(bytes, count, array->elementSize);
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
        size_t bytes = 0;
        if (!ArrayBytes(world, array, &bytes))
        {
            return false;
        }
        if (bytes == 0)
        {
            continue;
        }
        void* memory = m2WorldAlloc(world, bytes);
        *FieldPointer(world, array) = memory;
        world->memoryBytes += (int64_t)bytes;
        ok = ok && memory != NULL;
        if (!ok)
        {
            return false;
        }
    }
    return ok;
}

void m2StateFree(m2World* world)
{
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        void** field = FieldPointer(world, array);
        if ((array->flags & M2_STATE_POINTER) != 0 && *field != NULL)
        {
            size_t bytes = 0;
            bool sized = ArrayBytes(world, array, &bytes);
            M2_ASSERT(sized); // it was allocated with this size
            (void)sized;
            m2WorldFree(world, *field, bytes);
            *field = NULL;
        }
    }
}

// Whether an array is snapshot state in this world.
static bool InSnapshot(const m2World* world, const m2StateArray* array)
{
    return (array->flags & M2_STATE_SNAPSHOT) != 0 && Present(world, array->flags);
}

// The bytes one array takes in a snapshot: its element, or one element per
// slot of its extent; false when that does not fit a size_t. The measure
// and the walk size every array here, so a world measures to exactly the
// bytes its walk copies.
static bool SnapshotArrayBytes(const m2World* world, const m2StateArray* array, size_t* bytes)
{
    *bytes = array->elementSize;
    return (array->flags & M2_STATE_POINTER) == 0 ||
           !ckd_mul(bytes, ExtentCount(world, array->extent), array->elementSize);
}

bool m2StateSnapshotBytes(const m2World* world, size_t* bytesOut)
{
    size_t total = 0;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        size_t bytes = 0;
        if (InSnapshot(world, array) &&
            (!SnapshotArrayBytes(world, array, &bytes) || ckd_add(&total, total, bytes)))
        {
            return false;
        }
    }
    *bytesOut = total;
    return true;
}

size_t m2StateWalk(m2World* world, uint8_t* out, const uint8_t* in, int direction)
{
    size_t cursor = 0;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if (!InSnapshot(world, array))
        {
            continue;
        }
        size_t bytes = 0;
        bool sized = SnapshotArrayBytes(world, array, &bytes);
        M2_ASSERT(sized); // m2CreateWorld measured every array
        (void)sized;
        bool pointer = (array->flags & M2_STATE_POINTER) != 0;
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
    return cursor;
}

static bool CheckBlock(const m2World* world, const m2StateArray* array, const uint8_t* data,
                       size_t count)
{
    m2StateBounds bounds = {(int64_t)ExtentCount(world, array->range),
                            (int64_t)world->shapes.shapeCapacity};
    return m2StateCheckElements(data, count, array->elementSize, (m2Check)array->check, bounds);
}

bool m2StateValidate(const m2World* world, const uint8_t* in)
{
    size_t cursor = 0;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if (!InSnapshot(world, array))
        {
            continue;
        }
        size_t bytes = 0;
        bool sized = SnapshotArrayBytes(world, array, &bytes);
        M2_ASSERT(sized); // m2CreateWorld measured every array
        (void)sized;
        bool pointer = (array->flags & M2_STATE_POINTER) != 0;
        size_t count = pointer ? ExtentCount(world, array->extent) : 1;
        M2_ASSERT(count * array->elementSize == bytes);
        if (!CheckBlock(world, array, in + cursor, count))
        {
            return false;
        }
        cursor += bytes;
    }
    return true;
}

bool m2StateBodyValid(const m2World* world, int32_t index)
{
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if (!InSnapshot(world, array) || (array->flags & M2_STATE_POINTER) == 0 ||
            array->extent != m2_extent_body)
        {
            continue;
        }
        const uint8_t* data = *(const uint8_t* const*)((const uint8_t*)world + array->offset);
        if (!CheckBlock(world, array, data + (size_t)index * array->elementSize, 1))
        {
            return false;
        }
    }
    return true;
}

const uint8_t* m2StateIncoming(const m2World* world, const uint8_t* in, const void* field)
{
    size_t offset = (size_t)((const uint8_t*)field - (const uint8_t*)world);
    size_t cursor = 0;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if (!InSnapshot(world, array))
        {
            continue;
        }
        if (array->offset == offset)
        {
            return in + cursor;
        }
        size_t bytes = 0;
        bool sized = SnapshotArrayBytes(world, array, &bytes);
        M2_ASSERT(sized); // m2CreateWorld measured every array
        (void)sized;
        cursor += bytes;
    }
    return NULL;
}

// Which slots of an extent are hashed: the live ones below bound when
// alive is set, else the first bound elements, keyed by the count.
typedef struct m2HashSlots
{
    const uint8_t* alive;
    int32_t bound;
    m2HashPart part;
} m2HashSlots;

static m2HashSlots HashSlots(const m2World* world, uint8_t extent)
{
    switch ((m2Extent)extent)
    {
    case m2_extent_body:
        return (m2HashSlots){world->bodies.alive, world->bodies.maxBodyIndex, m2_hashBodies};
    case m2_extent_shape:
        return (m2HashSlots){world->shapes.shapeAlive, world->shapes.maxShapeIndex, m2_hashBodies};
    case m2_extent_chain:
        return (m2HashSlots){world->chains.chainAlive, world->chains.maxChainIndex, m2_hashBodies};
    case m2_extent_joint:
        return (m2HashSlots){world->joints.jointAlive, world->joints.maxJointIndex, m2_hashJoints};
    case m2_extent_particle:
        return (m2HashSlots){world->particles.particleAlive, world->particles.maxParticleIndex,
                             m2_hashParticles};
    case m2_extent_fluidVolume:
        return (m2HashSlots){world->volumes.fvAlive, world->volumes.maxFvIndex, m2_hashVolumes};
    case m2_extent_pair:
        return (m2HashSlots){NULL, world->contacts.pairCount, m2_hashContacts};
    case m2_extent_particleSpring:
        return (m2HashSlots){NULL, world->particles.particleSpringCount, m2_hashParticles};
    case m2_extent_particleTriad:
        return (m2HashSlots){NULL, world->particles.particleTriadCount, m2_hashParticles};
    default:
        return (m2HashSlots){NULL, 1, m2_hashWorld}; // inline scalars
    }
}

static uint64_t HashArray(const m2World* world, const m2StateArray* array, m2HashPart part,
                          uint64_t h)
{
    m2HashSlots slots = HashSlots(world, array->extent);
    if (part != m2_hashAll && part != slots.part)
    {
        return h;
    }
    const uint8_t* data = (const uint8_t*)world + array->offset;
    if ((array->flags & M2_STATE_POINTER) != 0)
    {
        data = *(const uint8_t* const*)data;
    }
    int32_t size = (int32_t)array->elementSize;
    if (slots.alive == NULL)
    {
        h = m2Hash64(h, &slots.bound, (int32_t)sizeof(slots.bound));
    }
    for (int32_t i = 0; i < slots.bound; ++i)
    {
        if (slots.alive != NULL)
        {
            if (slots.alive[i] == 0)
            {
                continue;
            }
            h = m2Hash64(h, &i, (int32_t)sizeof(i));
        }
        h = m2Hash64(h, data + (size_t)i * (size_t)size, size);
    }
    return h;
}

uint64_t m2StateHash(const m2World* world, m2HashPart part)
{
    uint64_t h = M2_HASH_INIT;
    for (size_t i = 0; i < sizeof(s_state) / sizeof(s_state[0]); ++i)
    {
        const m2StateArray* array = &s_state[i];
        if ((array->flags & M2_STATE_HASH) != 0 && Present(world, array->flags))
        {
            h = HashArray(world, array, part, h);
        }
    }
    return m2HashPools(world, part, m2HashFatBoxes(world, part, h));
}
