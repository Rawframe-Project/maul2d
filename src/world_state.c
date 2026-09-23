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

// What a restore checks in an incoming block before any of it lands, so
// hostile snapshot bytes can never index out of bounds.
typedef enum m2Check
{
    m2_check_none,
    m2_check_index,       // int32 elements in [0, range)
    m2_check_indexOrNone, // int32 elements in [-1, range)
    m2_check_count,       // an inline int32 in [0, range]
    m2_check_cursor,      // an inline int32 ring cursor in [0, max(range - 1, 0)]
    m2_check_bool,        // uint8 elements 0 or 1
    m2_check_bodyType,    // uint8 elements below the body type count
    m2_check_jointType,   // uint8 elements below the joint type count
    m2_check_geometry,    // shape geometry: a known type, a polygon count that fits
    m2_check_manifold,    // manifold point counts that fit
    m2_check_pairKey,     // both shape indices of every pair key in range
    m2_check_tree,        // the inline tree headers
    m2_check_treeNode,    // tree node links and leaf payloads
    m2_check_finite32,    // every float of every element finite
    m2_check_finite64,    // every double of every element finite
    m2_check_transform,   // a double position and a float rotation, finite
} m2Check;

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
    // Snapshot state, in snapshot byte order.
    M2_STATE_CHECKED(bodies.transforms, m2Transform, body, 0, M2_STATE_SNAPSHOT, transform, one),
    M2_STATE_CHECKED(bodies.linearVelocities, m2Vec2, body, 1, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.angularVelocities, float, body, 1, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.gravityScales, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.invMass, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.invInertia, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.localCenters, m2Vec2, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.asleep, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(bodies.sleepTimes, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_ARRAY(bodies.sleepStreak, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.bullets, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(bodies.userData, uint64_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.types, uint8_t, body, 1, M2_STATE_SNAPSHOT, bodyType, one),
    M2_STATE_CHECKED(bodies.alive, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(bodies.bodyShapeHead, int32_t, body, 0, M2_STATE_SNAPSHOT, indexOrNone, shape),
    M2_STATE_ARRAY(bodies.generations, uint16_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.freeQueue, int32_t, body, 0, M2_STATE_SNAPSHOT, index, body),
    M2_STATE_CHECKED(shapes.shapeGeometry, m2ShapeGeometry, shape, 0, M2_STATE_SNAPSHOT, geometry,
                     one),
    M2_STATE_CHECKED(shapes.shapeDensity, float, shape, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(shapes.shapeFriction, float, shape, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(shapes.shapeRestitution, float, shape, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(shapes.shapeTangentSpeed, float, shape, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_ARRAY(shapes.shapeUserData, uint64_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(shapes.shapeBody, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone, body),
    M2_STATE_CHECKED(shapes.shapeNext, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone, shape),
    M2_STATE_CHECKED(shapes.shapeAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(shapes.shapeGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeCategory, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeMask, uint32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(shapes.shapeGroup, int32_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(shapes.shapeSensor, uint8_t, shape, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(shapes.shapeFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_CHECKED(broadphase.proxyIds, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone,
                     treeNode),
    M2_STATE_CHECKED(broadphase.inMoved, uint8_t, shape, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(broadphase.moved, int32_t, shape, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_INLINE_CHECKED(joints.maxJointIndex, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeHead, M2_STATE_SNAPSHOT, cursor, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeTail, M2_STATE_SNAPSHOT, cursor, joint),
    M2_STATE_INLINE_CHECKED(joints.jointFreeCount, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_INLINE_CHECKED(joints.jointRetiredCount, M2_STATE_SNAPSHOT, count, joint),
    M2_STATE_CHECKED(joints.jointType, uint8_t, joint, 0, M2_STATE_SNAPSHOT, jointType, one),
    M2_STATE_CHECKED(joints.jointAlive, uint8_t, joint, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(joints.jointBodyA, int32_t, joint, 0, M2_STATE_SNAPSHOT, indexOrNone, body),
    M2_STATE_CHECKED(joints.jointBodyB, int32_t, joint, 0, M2_STATE_SNAPSHOT, indexOrNone, body),
    M2_STATE_CHECKED(joints.jointLocalAnchorA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointLocalAnchorB, m2Vec2, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointLength, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointHertz, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointDamping, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointHertz2, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointDamping2, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointImpulse, m2Vec2, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_ARRAY(joints.jointFlags, uint8_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(joints.jointMotorSpeed, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointMaxMotor, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointLower, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointUpper, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointLocalAxisA, m2Vec2, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointRefAngle, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointMotorImpulse, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointLowerImpulse, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointUpperImpulse, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(joints.jointSpringImpulse, float, joint, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_ARRAY(joints.jointBreakForce, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(joints.jointCollide, uint8_t, joint, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(joints.jointTargets, m2Pos2, joint, 0, M2_STATE_SNAPSHOT, finite64, one),
    M2_STATE_CHECKED(joints.jointTargetsB, m2Pos2, joint, 0, M2_STATE_SNAPSHOT, finite64, one),
    M2_STATE_ARRAY(joints.jointUserData, uint64_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointBreakTorque, float, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_ARRAY(joints.jointGenerations, uint16_t, joint, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(joints.jointFreeQueue, int32_t, joint, 0, M2_STATE_SNAPSHOT, index, joint),
    M2_STATE_INLINE_CHECKED(chains.maxChainIndex, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeHead, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeTail, M2_STATE_SNAPSHOT, cursor, shape),
    M2_STATE_INLINE_CHECKED(chains.chainFreeCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(chains.chainRetiredCount, M2_STATE_SNAPSHOT, count, shape),
    M2_STATE_INLINE_CHECKED(lastInvH, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_INLINE_CHECKED(sleepEnabled, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(bodies.linearDampings, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.angularDampings, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.fixedRotations, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(bodies.motionLocks, uint8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(bodies.sleepEnables, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(bodies.forces, m2Vec2, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.torques, float, body, 0, M2_STATE_SNAPSHOT, finite32, one),
    M2_STATE_CHECKED(bodies.disabled, uint8_t, body, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_ARRAY(bodies.dominances, int8_t, body, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(shapes.shapeChain, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone, shape),
    M2_STATE_CHECKED(chains.chainAlive, uint8_t, shape, 0, M2_STATE_SNAPSHOT, bool, one),
    M2_STATE_CHECKED(chains.chainBody, int32_t, shape, 0, M2_STATE_SNAPSHOT, indexOrNone, body),
    M2_STATE_ARRAY(chains.chainGenerations, uint16_t, shape, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(chains.chainFreeQueue, int32_t, shape, 0, M2_STATE_SNAPSHOT, index, shape),
    M2_STATE_INLINE_CHECKED(broadphase.trees, M2_STATE_SNAPSHOT, tree, treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[0], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[1], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(broadphase.treeNodes[2], m2TreeNode, treeNode, 0, M2_STATE_SNAPSHOT, treeNode,
                     treeNode),
    M2_STATE_CHECKED(contacts.pairKeys, uint64_t, pair, 0, M2_STATE_SNAPSHOT, pairKey, shape),
    M2_STATE_ARRAY(contacts.pairTouching, uint8_t, pair, 0, M2_STATE_SNAPSHOT),
    M2_STATE_CHECKED(contacts.manifolds, m2Manifold, pair, 0, M2_STATE_SNAPSHOT, manifold, one),
    M2_STATE_CHECKED(particles.particlePositions, m2Pos2, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite64, one),
    M2_STATE_CHECKED(particles.particleVelocities, m2Vec2, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_CHECKED(particles.particleAlive, uint8_t, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, bool, one),
    M2_STATE_ARRAY(particles.particleGenerations, uint16_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_ARRAY(particles.particleFlags, uint32_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_CHECKED(particles.particleLifetime, float, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_ARRAY(particles.particleUserData, uint64_t, particle, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_PARTICLES),
    M2_STATE_CHECKED(particles.particleFreeQueue, int32_t, particle, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_INLINE_CHECKED(particles.particleFreeHead, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            cursor, particle),
    M2_STATE_INLINE_CHECKED(particles.particleFreeCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particle),
    M2_STATE_INLINE_CHECKED(particles.particleCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, count,
                            particle),
    M2_STATE_INLINE_CHECKED(particles.maxParticleIndex, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particle),
    M2_STATE_CHECKED(particles.particleSpringA, int32_t, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleSpringB, int32_t, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleSpringRest, float, particleSpring, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_INLINE_CHECKED(particles.particleSpringCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particleSpring),
    M2_STATE_CHECKED(particles.particleTriadA, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadB, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadC, int32_t, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, index, particle),
    M2_STATE_CHECKED(particles.particleTriadPA, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_CHECKED(particles.particleTriadPB, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_CHECKED(particles.particleTriadPC, m2Vec2, particleTriad, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_PARTICLES, finite32, one),
    M2_STATE_INLINE_CHECKED(particles.particleTriadCount, M2_STATE_SNAPSHOT | M2_STATE_PARTICLES,
                            count, particleTriad),
    M2_STATE_CHECKED(volumes.fvLower, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     finite64, one),
    M2_STATE_CHECKED(volumes.fvUpper, m2Pos2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     finite64, one),
    M2_STATE_CHECKED(volumes.fvSurface, double, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     finite64, one),
    M2_STATE_CHECKED(volumes.fvDensity, float, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     finite32, one),
    M2_STATE_CHECKED(volumes.fvLinearDrag, float, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, finite32, one),
    M2_STATE_CHECKED(volumes.fvAngularDrag, float, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, finite32, one),
    M2_STATE_CHECKED(volumes.fvFlow, m2Vec2, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     finite32, one),
    M2_STATE_ARRAY(volumes.fvUserData, uint64_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_CHECKED(volumes.fvAlive, uint8_t, fluidVolume, 0, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS,
                     bool, one),
    M2_STATE_ARRAY(volumes.fvGenerations, uint16_t, fluidVolume, 0,
                   M2_STATE_SNAPSHOT | M2_STATE_FLUIDS),
    M2_STATE_CHECKED(volumes.fvFreeQueue, int32_t, fluidVolume, 0,
                     M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, index, fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.fvFreeHead, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, cursor,
                            fluidVolume),
    M2_STATE_INLINE_CHECKED(volumes.fvFreeCount, M2_STATE_SNAPSHOT | M2_STATE_FLUIDS, count,
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

static int32_t ReadInt32(const uint8_t* bytes)
{
    int32_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

static bool InRange(int64_t value, int64_t lo, int64_t hi)
{
    return value >= lo && value <= hi;
}

static bool CheckIndices(const uint8_t* data, size_t count, int64_t lo, int64_t range)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (!InRange(ReadInt32(data + 4 * i), lo, range - 1))
        {
            return false;
        }
    }
    return true;
}

static bool CheckBytes(const uint8_t* data, size_t count, uint8_t max)
{
    for (size_t i = 0; i < count; ++i)
    {
        if (data[i] > max)
        {
            return false;
        }
    }
    return true;
}

static bool CheckGeometry(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2ShapeGeometry g;
        memcpy(&g, data + i * sizeof(g), sizeof(g));
        if (!InRange(g.type, m2_circleShape, m2_chainSegmentShape) ||
            (g.type == m2_polygonShape && !InRange(g.polygon.count, 3, M2_MAX_POLYGON_VERTICES)))
        {
            return false;
        }
    }
    return true;
}

static bool CheckManifolds(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2Manifold m;
        memcpy(&m, data + i * sizeof(m), sizeof(m));
        if (!InRange(m.pointCount, 0, 2))
        {
            return false;
        }
    }
    return true;
}

static bool CheckPairKeys(const uint8_t* data, size_t count, int64_t shapes)
{
    for (size_t i = 0; i < count; ++i)
    {
        uint64_t key;
        memcpy(&key, data + i * sizeof(key), sizeof(key));
        if ((int64_t)(key >> 32) >= shapes || (int64_t)(key & 0xFFFFFFFFu) >= shapes)
        {
            return false;
        }
    }
    return true;
}

static bool CheckTrees(const uint8_t* data, size_t bytes, int64_t nodes)
{
    for (size_t at = 0; at + sizeof(m2DynamicTree) <= bytes; at += sizeof(m2DynamicTree))
    {
        m2DynamicTree tree;
        memcpy(&tree, data + at, sizeof(tree));
        if (!InRange(tree.root, -1, nodes - 1) || !InRange(tree.freeList, -1, nodes - 1) ||
            !InRange(tree.nodeCount, 0, nodes) || tree.nodeCapacity != nodes)
        {
            return false;
        }
    }
    return true;
}

static bool CheckTreeNodes(const uint8_t* data, size_t count, int64_t nodes, int64_t shapes)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2TreeNode node;
        memcpy(&node, data + i * sizeof(node), sizeof(node));
        if (!InRange(node.child1, -1, nodes - 1) || !InRange(node.child2, -1, nodes - 1) ||
            !InRange(node.parent, -1, nodes - 1) || !InRange(node.userData, -1, shapes - 1))
        {
            return false;
        }
    }
    return true;
}

static bool CheckFinite(const uint8_t* data, size_t bytes, size_t width)
{
    for (size_t at = 0; at + width <= bytes; at += width)
    {
        bool finite;
        if (width == sizeof(float))
        {
            float f;
            memcpy(&f, data + at, sizeof(f));
            finite = m2FiniteF(f);
        }
        else
        {
            double d;
            memcpy(&d, data + at, sizeof(d));
            finite = m2FiniteD(d);
        }
        if (!finite)
        {
            return false;
        }
    }
    return true;
}

static bool CheckTransforms(const uint8_t* data, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        m2Transform xf;
        memcpy(&xf, data + i * sizeof(xf), sizeof(xf));
        if (!m2FinitePos2(xf.p) || !m2FiniteF(xf.q.c) || !m2FiniteF(xf.q.s))
        {
            return false;
        }
    }
    return true;
}

static bool CheckBlock(const m2World* world, const m2StateArray* array, const uint8_t* data,
                       size_t count)
{
    int64_t range = (int64_t)ExtentCount(world, array->range);
    switch ((m2Check)array->check)
    {
    case m2_check_none:
        return true;
    case m2_check_index:
        return CheckIndices(data, count, 0, range);
    case m2_check_indexOrNone:
        return CheckIndices(data, count, -1, range);
    case m2_check_count:
        return InRange(ReadInt32(data), 0, range);
    case m2_check_cursor:
        return InRange(ReadInt32(data), 0, range > 0 ? range - 1 : 0);
    case m2_check_bool:
        return CheckBytes(data, count, 1);
    case m2_check_bodyType:
        return CheckBytes(data, count, (uint8_t)m2_dynamicBody);
    case m2_check_jointType:
        return CheckBytes(data, count, (uint8_t)m2_ratchetJoint);
    case m2_check_geometry:
        return CheckGeometry(data, count);
    case m2_check_manifold:
        return CheckManifolds(data, count);
    case m2_check_pairKey:
        return CheckPairKeys(data, count, range);
    case m2_check_tree:
        return CheckTrees(data, array->elementSize, range);
    case m2_check_treeNode:
        return CheckTreeNodes(data, count, range, (int64_t)world->shapes.shapeCapacity);
    case m2_check_finite32:
        return CheckFinite(data, count * array->elementSize, sizeof(float));
    case m2_check_finite64:
        return CheckFinite(data, count * array->elementSize, sizeof(double));
    case m2_check_transform:
        return CheckTransforms(data, count);
    }
    return false;
}

bool m2StateValidate(const m2World* world, const uint8_t* in)
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
        size_t count = pointer ? ExtentCount(world, array->extent) : 1;
        if (!CheckBlock(world, array, in + cursor, count))
        {
            return false;
        }
        cursor += pointer ? count * array->elementSize : array->elementSize;
    }
    return true;
}
