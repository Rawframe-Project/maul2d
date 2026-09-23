// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: the registry of live worlds, creation and destruction,
// the step, world settings, enumeration and the invariant check.

#include "world.h"

#include "broadphase.h"
#include "contact.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_MAX_WORLDS 16

static m2World* s_worlds[M2_MAX_WORLDS];

static uint16_t s_worldGenerations[M2_MAX_WORLDS];

m2World* m2GetWorld(m2WorldId id)
{
    if (id.index1 < 1 || id.index1 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    m2World* world = s_worlds[id.index1 - 1];
    if (world == NULL || s_worldGenerations[id.index1 - 1] != id.generation)
    {
        return NULL;
    }
    return world;
}

m2World* m2WorldFromId(m2WorldId worldId)
{
    return m2GetWorld(worldId);
}

m2World* m2WorldFromIndex(uint16_t world0)
{
    if (world0 < 1 || world0 > M2_MAX_WORLDS)
    {
        return NULL;
    }
    return s_worlds[world0 - 1];
}

// --- Defs & world lifecycle ----------------------------------------------------

void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange)
{
    if (itemCount <= 0)
    {
        return;
    }
    if (world->enqueueTask != NULL)
    {
        void* task = world->enqueueTask(fn, itemCount, minRange, ctx, world->userTaskContext);
        world->finishTask(task, world->userTaskContext);
        return;
    }
    fn(0, itemCount, ctx);
}

m2WorldDef m2DefaultWorldDef(void)
{
    m2WorldDef def;
    memset(&def, 0, sizeof(def));
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.jointCapacity = 256;
    def.particleCapacity = 0;    // fluids are opt-in
    def.fluidVolumeCapacity = 0; // buoyancy volumes are opt-in
    def.particleRadius = 0.05f;
    def.particleDensity = 1.0f;
    def.particleGravityScale = 1.0f;
    def.particlePressureStrength = 0.05f;
    def.particleDampingStrength = 1.0f;
    def.particleViscousStrength = 0.25f; // used by viscous-flagged particles only
    def.particlePowderStrength = 0.5f;
    def.particleSpringStrength = 0.25f;  // overlapping nets sum; reference value
    def.particleElasticStrength = 0.25f; // stiff blobs combine spring|elastic flags
    def.particleTensilePressureStrength = 0.2f;
    def.particleTensileNormalStrength = 0.2f;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a clear abort beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    if (m2VerifyCpuBackend() == 0)
    {
        m2Refuse(NULL, m2_errorConfig);
        return m2_nullWorldId; // B1: typed refusal, never an abort
    }
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1 ||
        def->shapeCapacity < 1 || def->jointCapacity < 1)
    {
        m2Refuse(NULL, m2_errorInvalid);
        return m2_nullWorldId;
    }
    if (def->fluidVolumeCapacity < 0)
    {
        m2Refuse(NULL, m2_errorInvalid);
        return m2_nullWorldId;
    }
    if (def->particleCapacity < 0 ||
        (def->particleCapacity > 0 &&
         (!(def->particleRadius >= 0.02f) || !(def->particleDensity > 0.0f) ||
          !m2FiniteF(def->particleGravityScale) || !(def->particlePressureStrength >= 0.0f) ||
          !(def->particleDampingStrength >= 0.0f) || !(def->particleViscousStrength >= 0.0f) ||
          !(def->particleTensilePressureStrength >= 0.0f) ||
          !(def->particleTensileNormalStrength >= 0.0f) || !(def->particlePowderStrength >= 0.0f) ||
          !(def->particleSpringStrength >= 0.0f) || !(def->particleElasticStrength >= 0.0f))))
    {
        // Fluids config is validated loudly: the radius floor is 4x
        // linear slop so the skin laws keep meaning.
        m2Refuse(NULL, m2_errorInvalid);
        return m2_nullWorldId;
    }

    int32_t slot = -1;
    for (int32_t i = 0; i < M2_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        m2Refuse(NULL, m2_errorCapacity);
        return m2_nullWorldId;
    }

    m2World* world = m2AllocZeroed(sizeof(m2World));
    if (world == NULL)
    {
        m2Refuse(NULL, m2_errorCapacity);
        return m2_nullWorldId;
    }

    int32_t cap = def->bodyCapacity;
    int32_t shapeCap = def->shapeCapacity;
    int32_t jointCap = def->jointCapacity;
    world->gravity = def->gravity;
    world->windVelocity = (m2Vec2){0.0f, 0.0f};
    world->windLinearDrag = 0.0f; // wind is opt-in via m2World_SetWind
    world->bodyCapacity = cap;
    world->shapeCapacity = shapeCap;
    world->jointCapacity = jointCap;
    world->treeNodeCapacity = 2 * shapeCap;
    world->pairCapacity = 8 * shapeCap;
    world->particleCapacity = def->particleCapacity;
    world->fvCapacity = def->fluidVolumeCapacity;
    world->particleRadius = def->particleRadius;
    world->particleDensity = def->particleDensity;
    world->particleGravityScale = def->particleGravityScale;
    world->particlePressureStrength = def->particlePressureStrength;
    world->particleDampingStrength = def->particleDampingStrength;
    world->particleViscousStrength = def->particleViscousStrength;
    world->particleTensilePressure = def->particleTensilePressureStrength;
    world->particlePowderStrength = def->particlePowderStrength;
    world->particleSpringStrength = def->particleSpringStrength;
    world->particleElasticStrength = def->particleElasticStrength;
    world->particleTensileNormal = def->particleTensileNormalStrength;

    bool ok = true;
// The macro also meters the world's persistent footprint
//: every use site is inside create where
// `world` is in scope by construction.
#define M2_ALLOC(field, count, type)                                                               \
    do                                                                                             \
    {                                                                                              \
        world->field = m2AllocZeroed((size_t)(count) * sizeof(type));                              \
        world->memoryBytes += (int64_t)((size_t)(count) * sizeof(type));                           \
        ok = ok && world->field != NULL;                                                           \
    } while (0)
    M2_ALLOC(transforms, cap, m2Transform);
    M2_ALLOC(linearVelocities, cap + 1, m2Vec2); // +1: wide-lane dummy slot
    M2_ALLOC(angularVelocities, cap + 1, float); // +1: wide-lane dummy slot
    M2_ALLOC(gravityScales, cap, float);
    M2_ALLOC(userData, cap, uint64_t);
    M2_ALLOC(types, cap + 1, uint8_t); // +1: wide-lane dummy slot
    M2_ALLOC(alive, cap, uint8_t);
    M2_ALLOC(bodyShapeHead, cap, int32_t);
    M2_ALLOC(invMass, cap, float);
    M2_ALLOC(invInertia, cap, float);
    M2_ALLOC(localCenters, cap, m2Vec2);
    M2_ALLOC(asleep, cap, uint8_t);
    M2_ALLOC(sleepTimes, cap, float);
    M2_ALLOC(sleepStreak, cap, uint8_t);
    M2_ALLOC(bullets, cap, uint8_t);
    M2_ALLOC(ccdPrevPositions, cap, m2Pos2);
    M2_ALLOC(islandParent, cap, int32_t);
    M2_ALLOC(islandDisturbed, cap, uint8_t);
    M2_ALLOC(generations, cap, uint16_t);
    M2_ALLOC(freeQueue, cap, int32_t);
    M2_ALLOC(shapeGeometry, shapeCap, m2ShapeGeometry);
    M2_ALLOC(shapeDensity, shapeCap, float);
    M2_ALLOC(shapeFriction, shapeCap, float);
    M2_ALLOC(shapeRestitution, shapeCap, float);
    M2_ALLOC(shapeTangentSpeed, shapeCap, float);
    M2_ALLOC(shapeUserData, shapeCap, uint64_t);
    M2_ALLOC(shapeBody, shapeCap, int32_t);
    M2_ALLOC(shapeNext, shapeCap, int32_t);
    M2_ALLOC(shapeAlive, shapeCap, uint8_t);
    M2_ALLOC(shapeGenerations, shapeCap, uint16_t);
    M2_ALLOC(shapeCategory, shapeCap, uint32_t);
    M2_ALLOC(shapeMask, shapeCap, uint32_t);
    M2_ALLOC(shapeGroup, shapeCap, int32_t);
    M2_ALLOC(shapeSensor, shapeCap, uint8_t);
    M2_ALLOC(shapeFreeQueue, shapeCap, int32_t);
    M2_ALLOC(proxyIds, shapeCap, int32_t);
    M2_ALLOC(inMoved, shapeCap, uint8_t);
    M2_ALLOC(moved, shapeCap, int32_t);
    M2_ALLOC(jointType, jointCap, uint8_t);
    M2_ALLOC(jointAlive, jointCap, uint8_t);
    M2_ALLOC(jointBodyA, jointCap, int32_t);
    M2_ALLOC(jointBodyB, jointCap, int32_t);
    M2_ALLOC(bodyJointHead, cap, int32_t);
    M2_ALLOC(jointEdgeNext, 2 * jointCap, int32_t);
    M2_ALLOC(jointLocalAnchorA, jointCap, m2Vec2);
    M2_ALLOC(jointLocalAnchorB, jointCap, m2Vec2);
    M2_ALLOC(jointLength, jointCap, float);
    M2_ALLOC(jointHertz, jointCap, float);
    M2_ALLOC(jointDamping, jointCap, float);
    M2_ALLOC(jointHertz2, jointCap, float);
    M2_ALLOC(jointDamping2, jointCap, float);
    M2_ALLOC(jointImpulse, jointCap, m2Vec2);
    M2_ALLOC(jointFlags, jointCap, uint8_t);
    M2_ALLOC(jointMotorSpeed, jointCap, float);
    M2_ALLOC(jointMaxMotor, jointCap, float);
    M2_ALLOC(jointLower, jointCap, float);
    M2_ALLOC(jointUpper, jointCap, float);
    M2_ALLOC(jointLocalAxisA, jointCap, m2Vec2);
    M2_ALLOC(jointRefAngle, jointCap, float);
    M2_ALLOC(jointMotorImpulse, jointCap, float);
    M2_ALLOC(jointLowerImpulse, jointCap, float);
    M2_ALLOC(jointUpperImpulse, jointCap, float);
    M2_ALLOC(jointSpringImpulse, jointCap, float);
    M2_ALLOC(jointBreakForce, jointCap, float);
    M2_ALLOC(jointCollide, jointCap, uint8_t);
    M2_ALLOC(jointTargets, jointCap, m2Pos2);
    M2_ALLOC(jointTargetsB, jointCap, m2Pos2);
    if (def->particleCapacity > 0)
    {
        int32_t particleCap = def->particleCapacity;
        M2_ALLOC(particlePositions, particleCap, m2Pos2);
        M2_ALLOC(particleVelocities, particleCap, m2Vec2);
        M2_ALLOC(particleAlive, particleCap, uint8_t);
        M2_ALLOC(particleGenerations, particleCap, uint16_t);
        M2_ALLOC(particleFlags, particleCap, uint32_t);
        M2_ALLOC(particleLifetime, particleCap, float);
        M2_ALLOC(particleUserData, particleCap, uint64_t);
        M2_ALLOC(particleFreeQueue, particleCap, int32_t);
        world->particlePairCapacity = 12 * particleCap;
        world->particleProxies = m2AllocZeroed((size_t)particleCap * 16);
        ok = ok && world->particleProxies != NULL;
        world->particleProxiesTmp = m2AllocZeroed((size_t)particleCap * 16);
        ok = ok && world->particleProxiesTmp != NULL;
        M2_ALLOC(particlePairA, world->particlePairCapacity, int32_t);
        M2_ALLOC(particlePairB, world->particlePairCapacity, int32_t);
        M2_ALLOC(particlePairWeight, world->particlePairCapacity, float);
        M2_ALLOC(particlePairFlags, world->particlePairCapacity, uint32_t);
        M2_ALLOC(particlePairNormal, world->particlePairCapacity, m2Vec2);
        M2_ALLOC(particleWeights, particleCap, float);
        M2_ALLOC(particleAccumulation, particleCap, float);
        M2_ALLOC(particleAccumulation2, particleCap, m2Vec2);
        world->particleSpringCapacity = 4 * particleCap;
        M2_ALLOC(particleSpringA, world->particleSpringCapacity, int32_t);
        M2_ALLOC(particleSpringB, world->particleSpringCapacity, int32_t);
        M2_ALLOC(particleSpringRest, world->particleSpringCapacity, float);
        world->particleTriadCapacity = 2 * particleCap;
        M2_ALLOC(particleTriadA, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadB, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadC, world->particleTriadCapacity, int32_t);
        M2_ALLOC(particleTriadPA, world->particleTriadCapacity, m2Vec2);
        M2_ALLOC(particleTriadPB, world->particleTriadCapacity, m2Vec2);
        M2_ALLOC(particleTriadPC, world->particleTriadCapacity, m2Vec2);
        world->particleBodyCapacity = 4 * particleCap;
        M2_ALLOC(particleBodyParticle, world->particleBodyCapacity, int32_t);
        M2_ALLOC(particleBodyBody, world->particleBodyCapacity, int32_t);
        M2_ALLOC(particleBodyWeight, world->particleBodyCapacity, float);
        M2_ALLOC(particleBodyNormal, world->particleBodyCapacity, m2Vec2);
        M2_ALLOC(particleBodyMass, world->particleBodyCapacity, float);
        M2_ALLOC(particleBodyStageBody, 4 * particleCap, int32_t);
        M2_ALLOC(particleBodyStageWeight, 4 * particleCap, float);
        M2_ALLOC(particleBodyStageNormal, 4 * particleCap, m2Vec2);
        M2_ALLOC(particleBodyStageMass, 4 * particleCap, float);
        M2_ALLOC(particlePairWorkCount, particleCap + 1, int32_t);
        M2_ALLOC(particleBodyStageDrops, particleCap, int32_t);
    }
    M2_ALLOC(jointUserData, jointCap, uint64_t);
    M2_ALLOC(jointBreakTorque, jointCap, float);
    M2_ALLOC(jointGenerations, jointCap, uint16_t);
    M2_ALLOC(jointFreeQueue, jointCap, int32_t);
    M2_ALLOC(linearDampings, cap, float);
    M2_ALLOC(angularDampings, cap, float);
    M2_ALLOC(fixedRotations, cap, uint8_t);
    M2_ALLOC(motionLocks, cap, uint8_t);
    M2_ALLOC(sleepEnables, cap, uint8_t);
    M2_ALLOC(forces, cap, m2Vec2);
    M2_ALLOC(torques, cap, float);
    M2_ALLOC(disabled, cap, uint8_t);
    M2_ALLOC(dominances, cap, int8_t);
    if (def->fluidVolumeCapacity > 0)
    {
        int32_t fvCap = def->fluidVolumeCapacity;
        M2_ALLOC(fvLower, fvCap, m2Pos2);
        M2_ALLOC(fvUpper, fvCap, m2Pos2);
        M2_ALLOC(fvSurface, fvCap, double);
        M2_ALLOC(fvDensity, fvCap, float);
        M2_ALLOC(fvLinearDrag, fvCap, float);
        M2_ALLOC(fvAngularDrag, fvCap, float);
        M2_ALLOC(fvFlow, fvCap, m2Vec2);
        M2_ALLOC(fvUserData, fvCap, uint64_t);
        M2_ALLOC(fvAlive, fvCap, uint8_t);
        M2_ALLOC(fvGenerations, fvCap, uint16_t);
        M2_ALLOC(fvFreeQueue, fvCap, int32_t);
    }
    M2_ALLOC(shapeChain, shapeCap, int32_t);
    M2_ALLOC(chainAlive, shapeCap, uint8_t);
    M2_ALLOC(chainBody, shapeCap, int32_t);
    M2_ALLOC(chainGenerations, shapeCap, uint16_t);
    M2_ALLOC(chainFreeQueue, shapeCap, int32_t);
    M2_ALLOC(pairKeys, world->pairCapacity, uint64_t);
    M2_ALLOC(pairTouching, world->pairCapacity, uint8_t);
    M2_ALLOC(touchingScratch, world->pairCapacity, uint8_t);
    M2_ALLOC(colorMasks, cap, uint32_t);
    // Indexed by CONSTRAINT, not body: one slot per potential pair.
    // (Sized by body capacity until the pyramid30 perf scene found the
    // overflow - constraints outnumber bodies in dense stacks.)
    M2_ALLOC(constraintColors, world->pairCapacity, uint8_t);
    M2_ALLOC(colorOrder, world->pairCapacity, int32_t);

    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;
    M2_ALLOC(beginEvents, world->pairCapacity, m2ContactBeginEvent);
    M2_ALLOC(endEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pendingEndEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(sensorBeginEvents, world->pairCapacity, m2ContactBeginEvent);
    M2_ALLOC(sensorEndEvents, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(pendingSensorEnd, world->pairCapacity, m2ContactEndEvent);
    M2_ALLOC(jointBreakEvents, world->jointCapacity, m2JointBreakEvent);
    M2_ALLOC(pairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifolds, world->pairCapacity, m2Manifold);
    M2_ALLOC(oldPairScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(pairMergeScratch, world->pairCapacity, uint64_t);
    M2_ALLOC(manifoldScratch, world->pairCapacity, m2Manifold);
    M2_ALLOC(deltaPositions, cap + 1, m2Vec2); // +1: wide-lane dummy slot
    M2_ALLOC(deltaRotations, cap + 1, m2Rot);  // +1: wide-lane dummy slot
    world->constraintScratch =
        m2AllocZeroed((size_t)world->pairCapacity * (size_t)m2ContactConstraintSize());
    world->contactBlocks = m2AllocZeroed((size_t)m2ContactBlockScratchBytes(world->pairCapacity));
    ok = ok && world->contactBlocks != NULL;
    ok = ok && world->constraintScratch != NULL;
#undef M2_ALLOC
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        world->treeNodes[t] = m2AllocZeroed((size_t)world->treeNodeCapacity * sizeof(m2TreeNode));
        ok = ok && world->treeNodes[t] != NULL;
    }
    if (!ok)
    {
        m2Refuse(NULL, m2_errorCapacity); // out of memory
        m2WorldId failed = {(uint16_t)(slot + 1), s_worldGenerations[slot]};
        s_worlds[slot] = world;
        m2DestroyWorld(failed);
        return m2_nullWorldId;
    }

    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeInit(&world->trees[t], world->treeNodes[t], world->treeNodeCapacity);
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->freeQueue[i] = i;
        world->bodyShapeHead[i] = -1;
        world->bodyJointHead[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapeFreeQueue[i] = i;
        world->shapeNext[i] = -1;
        world->proxyIds[i] = M2_NULL_NODE;
        world->shapeChain[i] = -1;
        world->chainFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < jointCap; ++i)
    {
        world->jointFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < 2 * jointCap; ++i)
    {
        world->jointEdgeNext[i] = -1;
    }
    world->jointFreeCount = jointCap;
    for (int32_t i = 0; i < world->particleCapacity; ++i)
    {
        world->particleFreeQueue[i] = i;
    }
    world->particleFreeCount = world->particleCapacity;
    for (int32_t i = 0; i < world->fvCapacity; ++i)
    {
        world->fvFreeQueue[i] = i;
    }
    world->fvFreeCount = world->fvCapacity;
    world->freeHead = 0;
    world->freeTail = 0;
    world->freeCount = cap;
    world->shapeFreeHead = 0;
    world->shapeFreeTail = 0;
    world->shapeFreeCount = shapeCap;
    world->chainFreeCount = shapeCap;

    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
    world->worldIndex0 = (uint16_t)(slot + 1);
    world->sleepEnabled = 1;
    s_worlds[slot] = world;

    m2WorldId id = {(uint16_t)(slot + 1), world->worldGeneration};
    return id;
}

void m2DestroyWorld(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL && worldId.index1 >= 1 && worldId.index1 <= M2_MAX_WORLDS)
    {
        world = s_worlds[worldId.index1 - 1]; // failed-allocation path
    }
    if (world == NULL)
    {
        return;
    }
    m2Free(world->transforms);
    m2Free(world->linearVelocities);
    m2Free(world->angularVelocities);
    m2Free(world->gravityScales);
    m2Free(world->userData);
    m2Free(world->types);
    m2Free(world->alive);
    m2Free(world->bodyShapeHead);
    m2Free(world->invMass);
    m2Free(world->invInertia);
    m2Free(world->localCenters);
    m2Free(world->asleep);
    m2Free(world->sleepTimes);
    m2Free(world->sleepStreak);
    m2Free(world->bullets);
    m2Free(world->ccdPrevPositions);
    m2Free(world->islandParent);
    m2Free(world->islandDisturbed);
    m2Free(world->generations);
    m2Free(world->freeQueue);
    m2Free(world->shapeGeometry);
    m2Free(world->shapeDensity);
    m2Free(world->shapeFriction);
    m2Free(world->shapeRestitution);
    m2Free(world->shapeTangentSpeed);
    m2Free(world->shapeUserData);
    m2Free(world->shapeBody);
    m2Free(world->shapeNext);
    m2Free(world->shapeAlive);
    m2Free(world->shapeGenerations);
    m2Free(world->shapeCategory);
    m2Free(world->shapeMask);
    m2Free(world->shapeGroup);
    m2Free(world->shapeSensor);
    m2Free(world->shapeFreeQueue);
    m2Free(world->proxyIds);
    m2Free(world->inMoved);
    m2Free(world->moved);
    m2Free(world->jointType);
    m2Free(world->jointAlive);
    m2Free(world->jointBodyA);
    m2Free(world->jointBodyB);
    m2Free(world->bodyJointHead);
    m2Free(world->jointEdgeNext);
    m2Free(world->jointLocalAnchorA);
    m2Free(world->jointLocalAnchorB);
    m2Free(world->jointLength);
    m2Free(world->jointHertz);
    m2Free(world->jointDamping);
    m2Free(world->jointHertz2);
    m2Free(world->jointDamping2);
    m2Free(world->jointImpulse);
    m2Free(world->jointFlags);
    m2Free(world->jointMotorSpeed);
    m2Free(world->jointMaxMotor);
    m2Free(world->jointLower);
    m2Free(world->jointUpper);
    m2Free(world->jointLocalAxisA);
    m2Free(world->jointRefAngle);
    m2Free(world->jointMotorImpulse);
    m2Free(world->jointLowerImpulse);
    m2Free(world->jointUpperImpulse);
    m2Free(world->jointSpringImpulse);
    m2Free(world->jointBreakForce);
    m2Free(world->jointCollide);
    m2Free(world->jointTargets);
    m2Free(world->jointTargetsB);
    m2Free(world->particlePositions);
    m2Free(world->particleVelocities);
    m2Free(world->particleAlive);
    m2Free(world->particleGenerations);
    m2Free(world->particleFlags);
    m2Free(world->particleLifetime);
    m2Free(world->particleUserData);
    m2Free(world->particleFreeQueue);
    m2Free(world->fvLower);
    m2Free(world->fvUpper);
    m2Free(world->fvSurface);
    m2Free(world->fvDensity);
    m2Free(world->fvLinearDrag);
    m2Free(world->fvAngularDrag);
    m2Free(world->fvFlow);
    m2Free(world->fvUserData);
    m2Free(world->fvAlive);
    m2Free(world->fvGenerations);
    m2Free(world->fvFreeQueue);
    m2Free(world->particleProxies);
    m2Free(world->particleProxiesTmp);
    m2Free(world->particlePairA);
    m2Free(world->particlePairB);
    m2Free(world->particlePairWeight);
    m2Free(world->particlePairFlags);
    m2Free(world->particlePairNormal);
    m2Free(world->particleWeights);
    m2Free(world->particleAccumulation);
    m2Free(world->particleAccumulation2);
    m2Free(world->particleSpringA);
    m2Free(world->particleSpringB);
    m2Free(world->particleSpringRest);
    m2Free(world->particleTriadA);
    m2Free(world->particleTriadB);
    m2Free(world->particleTriadC);
    m2Free(world->particleTriadPA);
    m2Free(world->particleTriadPB);
    m2Free(world->particleTriadPC);
    m2Free(world->particleBodyParticle);
    m2Free(world->particleBodyBody);
    m2Free(world->particleBodyWeight);
    m2Free(world->particleBodyNormal);
    m2Free(world->particleBodyMass);
    m2Free(world->particleBodyStageBody);
    m2Free(world->particleBodyStageWeight);
    m2Free(world->particleBodyStageNormal);
    m2Free(world->particleBodyStageMass);
    m2Free(world->particlePairWorkCount);
    m2Free(world->particleBodyStageDrops);
    m2Free(world->jointUserData);
    m2Free(world->jointBreakTorque);
    m2Free(world->jointGenerations);
    m2Free(world->jointFreeQueue);
    m2Free(world->linearDampings);
    m2Free(world->angularDampings);
    m2Free(world->fixedRotations);
    m2Free(world->motionLocks);
    m2Free(world->sleepEnables);
    m2Free(world->forces);
    m2Free(world->torques);
    m2Free(world->disabled);
    m2Free(world->dominances);
    m2Free(world->shapeChain);
    m2Free(world->chainAlive);
    m2Free(world->chainBody);
    m2Free(world->chainGenerations);
    m2Free(world->chainFreeQueue);
    m2Free(world->pairKeys);
    m2Free(world->pairTouching);
    m2Free(world->touchingScratch);
    m2Free(world->colorMasks);
    m2Free(world->constraintColors);
    m2Free(world->colorOrder);
    m2Free(world->beginEvents);
    m2Free(world->endEvents);
    m2Free(world->pendingEndEvents);
    m2Free(world->sensorBeginEvents);
    m2Free(world->sensorEndEvents);
    m2Free(world->pendingSensorEnd);
    m2Free(world->jointBreakEvents);
    m2Free(world->pairScratch);
    m2Free(world->manifolds);
    m2Free(world->oldPairScratch);
    m2Free(world->pairMergeScratch);
    m2Free(world->manifoldScratch);
    m2Free(world->deltaPositions);
    m2Free(world->deltaRotations);
    m2Free(world->constraintScratch);
    m2Free(world->contactBlocks);
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2Free(world->treeNodes[t]);
    }
    m2Free(world);
    s_worlds[worldId.index1 - 1] = NULL;
}

bool m2World_IsValid(m2WorldId worldId)
{
    return m2GetWorld(worldId) != NULL;
}

void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || !(dt > 0.0f) || substepCount < 1)
    {
        M2_ASSERT(world != NULL && dt > 0.0f && substepCount >= 1);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            float dt;
            int32_t substepCount;
        } marker;
        memset(&marker, 0, sizeof(marker));
        marker.dt = dt;
        marker.substepCount = substepCount;
        m2JournalRecord(world, m2_opStep, &marker, (int32_t)sizeof(marker));
    }

    // Fresh event window: clear the public buffers, then flush ends
    // queued by between-step destroys (they belong to this window).
    world->beginEventCount = 0;
    world->endEventCount = 0;
    for (int32_t i = 0; i < world->pendingEndCount && i < world->pairCapacity; ++i)
    {
        world->endEvents[world->endEventCount++] = world->pendingEndEvents[i];
    }
    world->pendingEndCount = 0;
    world->sensorBeginCount = 0;
    world->sensorEndCount = 0;
    for (int32_t i = 0; i < world->pendingSensorEndCount && i < world->pairCapacity; ++i)
    {
        world->sensorEndEvents[world->sensorEndCount++] = world->pendingSensorEnd[i];
    }
    world->pendingSensorEndCount = 0;
    world->jointBreakEventCount = 0;

    // Wall-clock diagnostics only; never fed back into simulation.
    uint64_t tStart = m2TimeNowNs();

    // Hibernation: when every dynamic body sleeps, no kinematic is
    // moving and nothing was teleported, the full pipeline provably
    // changes no state at all (frozen pairs copy themselves, islands
    // rebuild to the same roots, the solver has no constraints). Skip
    // it wholesale - bit-identical by construction, and a sleeping
    // city costs what a sleeping city should.
    if (world->movedCount == 0 && world->particleCount == 0)
    {
        bool anyoneStirring = false;
        for (int32_t i = 0; i < world->maxBodyIndex && !anyoneStirring; ++i)
        {
            if (world->alive[i] == 0)
            {
                continue;
            }
            if (world->types[i] == (uint8_t)m2_dynamicBody)
            {
                // A body that JUST fell asleep still owes one manifold
                // refresh (its stash can be one solve stale - the same
                // freshness rule the frozen-pair skip lives by).
                anyoneStirring =
                    world->disabled[i] == 0 && (world->asleep[i] == 0 || world->sleepStreak[i] < 2);
            }
            else if (world->types[i] == (uint8_t)m2_kinematicBody)
            {
                anyoneStirring = world->linearVelocities[i].x != 0.0f ||
                                 world->linearVelocities[i].y != 0.0f ||
                                 world->angularVelocities[i] != 0.0f;
            }
        }
        if (!anyoneStirring)
        {
            world->profile.stepMs = (float)((double)(m2TimeNowNs() - tStart) * 1.0e-6);
            world->profile.pairsMs = 0.0f;
            world->profile.contactsMs = 0.0f;
            world->profile.solveMs = 0.0f;
            world->profile.sleepMs = 0.0f;
            world->stepCount += 1;
            return;
        }
    }

    // Collide first (reference order): broadphase + narrowphase produce
    // fresh manifolds from current positions, then the solver moves the
    // world. Warm-start impulses arrive via the manifold carry.
    // Broadphase update: single-threaded, fixed body order, shape-list
    // order within a body (both snapshot-deterministic).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->disabled[i] != 0 ||
            world->types[i] == (uint8_t)m2_staticBody ||
            (world->types[i] == (uint8_t)m2_dynamicBody && world->asleep[i] != 0))
        {
            continue;
        }
        for (int32_t s = world->bodyShapeHead[i]; s != -1; s = world->shapeNext[s])
        {
            m2AABB tight = m2ShapeTightAABB(world, s);
            int32_t tree = m2ShapeTreeIndex(world, s);
            if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[s]].aabb, tight))
            {
                m2TreeMove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[s],
                           m2Fatten(tight));
                m2PushMoved(world, s);
            }
        }
    }
    world->oldPairCount = world->pairCount;
    m2StashContacts(world);
    m2UpdatePairs(world);
    uint64_t tPairs = m2TimeNowNs();
    m2UpdateContacts(world);
    uint64_t tContacts = m2TimeNowNs();

    // Touch transitions in canonical contact order (serial compaction:
    // the topic-08 event law, scalar edition).
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        uint8_t touchingNow = world->manifolds[i].pointCount > 0 ? 1 : 0;
        if (touchingNow != world->pairTouching[i])
        {
            int32_t a = (int32_t)(world->pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            bool sensor = world->shapeSensor[a] != 0 || world->shapeSensor[b] != 0;
            if (touchingNow != 0)
            {
                if (sensor)
                {
                    m2EmitSensorBegin(world, a, b, i);
                }
                else
                {
                    m2EmitBegin(world, a, b, i);
                }
            }
            else if (sensor)
            {
                m2EmitSensorEnd(world, a, b);
            }
            else
            {
                m2EmitEnd(world, a, b);
            }
            world->pairTouching[i] = touchingNow;
        }
    }

    if (world->particleCount > 0)
    {
        // The whole fluid pass runs once per step before the rigid
        // solve, the reference schedule; pairs freeze at step start.
        // It runs BEFORE the island update so a body the water wakes
        // pulls its whole island awake, the island-coupled sleep law.
        m2SolveParticles(world, dt);
        // Lifetimes count down and expire in ascending slot order at
        // step end; derived from state, so no journal op and it
        // replays and rolls back by itself.
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            if (world->particleAlive[i] == 0 || world->particleLifetime[i] <= 0.0f)
            {
                continue;
            }
            world->particleLifetime[i] -= dt;
            if (world->particleLifetime[i] <= 0.0f)
            {
                m2ParticleId dying = {i + 1, worldId.index1, world->particleGenerations[i]};
                uint8_t journalWas = world->journalActive;
                world->journalActive = 0; // derived death is never recorded
                m2World_DestroyParticle(dying);
                world->journalActive = journalWas;
            }
        }
    }
    if (world->maxFvIndex > 0)
    {
        // Buoyancy feeds the force accumulators before the solve, so
        // it integrates alongside gravity and dies with the step.
        m2ApplyFluidVolumes(world, dt);
    }
    if (world->windLinearDrag > 0.0f)
    {
        // Global wind, after buoyancy and before the solve (canonical
        // fluid-then-wind order): an area-weighted linear drag toward
        // the wind velocity into the same force accumulators.
        m2ApplyWind(world, dt);
    }
    m2UpdateIslandsAndWake(world);
    uint64_t tIslands = m2TimeNowNs();
    m2SolveStep(world, dt, substepCount);
    uint64_t tSolve = m2TimeNowNs();
    m2UpdateSleep(world, dt);
#ifdef MAUL2D_VALIDATE
    // The validate build walks the invariants after every step.
    M2_ASSERT(m2World_Validate(worldId));
#endif
    // Freshness streak: two consecutive step-ends asleep guarantee the
    // stashed manifolds were computed from these exact transforms.
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        world->sleepStreak[i] =
            world->asleep[i] != 0
                ? (uint8_t)(world->sleepStreak[i] < 2 ? world->sleepStreak[i] + 1 : 2)
                : 0;
    }
    uint64_t tEnd = m2TimeNowNs();

    world->profile.stepMs = (float)((double)(tEnd - tStart) * 1.0e-6);
    world->profile.pairsMs = (float)((double)(tPairs - tStart) * 1.0e-6);
    world->profile.contactsMs = (float)((double)(tContacts - tPairs) * 1.0e-6);
    world->profile.solveMs = (float)((double)(tSolve - tIslands) * 1.0e-6);
    world->profile.sleepMs =
        (float)((double)(tIslands - tContacts) * 1.0e-6 + (double)(tEnd - tSolve) * 1.0e-6);

    // Forces live for exactly one step (reference lifetime).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        world->forces[i] = (m2Vec2){0.0f, 0.0f};
        world->torques[i] = 0.0f;
    }
    world->stepCount += 1;
}

uint64_t m2World_GetStepCount(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL ? world->stepCount : 0;
}

int64_t m2World_MemoryBytes(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    return world->memoryBytes + (int64_t)sizeof(m2World);
}

void m2World_EnableSleeping(m2WorldId worldId, bool flag)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return;
    }
    uint8_t next = flag ? 1 : 0;
    if (world->sleepEnabled == next)
    {
        return; // no-op stays unjournaled, like SetGravity
    }
    if (world->journalActive != 0)
    {
        struct
        {
            uint8_t flag;
        } record;
        record.flag = next;
        m2JournalRecord(world, m2_opEnableSleeping, &record, (int32_t)sizeof(record));
    }
    world->sleepEnabled = next;
    if (next == 0)
    {
        // The rule that let them sleep is gone; wake everyone (the
        // SetGravity law).
        for (int32_t i = 0; i < world->maxBodyIndex; ++i)
        {
            if (world->alive[i] != 0 && world->types[i] == (uint8_t)m2_dynamicBody)
            {
                world->asleep[i] = 0;
                world->sleepTimes[i] = 0.0f;
            }
        }
    }
}

bool m2World_IsSleepingEnabled(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL && world->sleepEnabled != 0;
}

m2Profile m2World_GetProfile(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        m2Profile zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        return zero;
    }
    return world->profile;
}

double m2World_GetKineticEnergy(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0.0;
    }
    double energy = 0.0;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0 || world->types[i] != (uint8_t)m2_dynamicBody ||
            world->invMass[i] == 0.0f)
        {
            continue;
        }
        double vx = (double)world->linearVelocities[i].x;
        double vy = (double)world->linearVelocities[i].y;
        energy += 0.5 * (1.0 / (double)world->invMass[i]) * (vx * vx + vy * vy);
        if (world->invInertia[i] > 0.0f)
        {
            double w = (double)world->angularVelocities[i];
            energy += 0.5 * (1.0 / (double)world->invInertia[i]) * w * w;
        }
    }
    return energy;
}

void m2World_SetGravity(m2WorldId worldId, m2Vec2 gravity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->gravity.x == gravity.x && world->gravity.y == gravity.y)
    {
        return; // no-op, not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2Vec2 gravity;
        } record;
        memset(&record, 0, sizeof(record));
        record.gravity = gravity;
        m2JournalRecord(world, m2_opSetGravity, &record, (int32_t)sizeof(record));
    }
    world->gravity = gravity;
    // Honesty over precedent: a sleeping stack must feel the new
    // world. Wake every dynamic sleeper (deterministic, one pass).
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] != 0 && world->types[i] == (uint8_t)m2_dynamicBody &&
            world->asleep[i] != 0)
        {
            world->asleep[i] = 0;
            world->sleepTimes[i] = 0.0f;
        }
    }
}

m2Vec2 m2World_GetGravity(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL ? world->gravity : (m2Vec2){0.0f, 0.0f};
}

void m2World_SetWind(m2WorldId worldId, m2Vec2 velocity, float linearDrag)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || !(linearDrag >= 0.0f) || !m2FiniteVec2(velocity))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->windVelocity.x == velocity.x && world->windVelocity.y == velocity.y &&
        world->windLinearDrag == linearDrag)
    {
        return; // no-op, not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2Vec2 velocity;
            float linearDrag;
        } record;
        memset(&record, 0, sizeof(record));
        record.velocity = velocity;
        record.linearDrag = linearDrag;
        m2JournalRecord(world, m2_opSetWind, &record, (int32_t)sizeof(record));
    }
    world->windVelocity = velocity;
    world->windLinearDrag = linearDrag;
    // Deliberate deviation from SetGravity, which wakes every sleeper:
    // wind is expected to change often (gusts), so waking all sleepers
    // on each change would defeat sleeping. Asleep bodies are frozen and
    // deterministically skip the wind pass, exactly as they skip gravity
    // integration; a settled pile stays settled until roused otherwise.
}

void m2World_GetWind(m2WorldId worldId, m2Vec2* velocity, float* linearDrag)
{
    m2World* world = m2GetWorld(worldId);
    if (velocity != NULL)
    {
        *velocity = world != NULL ? world->windVelocity : (m2Vec2){0.0f, 0.0f};
    }
    if (linearDrag != NULL)
    {
        *linearDrag = world != NULL ? world->windLinearDrag : 0.0f;
    }
}

m2World* m2WorldFromIndex0(uint16_t index0)
{
    return m2WorldFromIndex(index0);
}

m2Counters m2World_GetCounters(m2WorldId worldId)
{
    m2Counters counters;
    memset(&counters, 0, sizeof(counters));
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return counters;
    }
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        counters.bodies += 1;
        counters.awakeBodies += world->asleep[i] == 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        counters.shapes += world->shapeAlive[i] != 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        counters.joints += world->jointAlive[i] != 0 ? 1 : 0;
    }
    counters.pairs = world->pairCount;
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        counters.touchingPairs += world->pairTouching[i] != 0 ? 1 : 0;
    }
    counters.constraints = world->lastConstraintCount;
    counters.graphColors = world->lastGraphColors;
    counters.overflowConstraints = world->lastOverflow;
    counters.stepCount = world->stepCount;
    counters.pairOverflow = world->pairOverflow;
    counters.particlePairOverflow = world->particlePairOverflow;
    counters.particleBodyOverflow = world->particleBodyOverflow;
    counters.particlePoolFull = world->particlePoolFullCount;
    counters.misuse = m2MisuseCount(world);
    return counters;
}

int32_t m2World_GetBodies(m2WorldId worldId, m2BodyId* ids, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2BodyId id = {i + 1, world->worldIndex0, world->generations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetJoints(m2WorldId worldId, m2JointId* ids, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {i + 1, world->worldIndex0, world->jointGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetChains(m2WorldId worldId, m2ChainId* ids, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxChainIndex; ++i)
    {
        if (world->chainAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2ChainId id = {i + 1, world->worldIndex0, world->chainGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

// The invariant walk: everything a healthy world must be able to
// say about itself, checked loudly. Pure reader.
bool m2World_Validate(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
#define M2_CHECK_INVARIANT(cond)                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return false;                                                                          \
        }                                                                                          \
    } while (0)

    for (int32_t i = 0; i < world->maxBodyIndex; ++i)
    {
        if (world->alive[i] == 0)
        {
            continue;
        }
        m2Transform xf = world->transforms[i];
        M2_CHECK_INVARIANT(m2FinitePos2(xf.p));
        M2_CHECK_INVARIANT(m2FiniteF(xf.q.c) && m2FiniteF(xf.q.s));
        m2Vec2 v = world->linearVelocities[i];
        M2_CHECK_INVARIANT(m2FiniteVec2(v));
        M2_CHECK_INVARIANT(m2FiniteF(world->angularVelocities[i]));
        M2_CHECK_INVARIANT(world->types[i] <= 2);
    }
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0)
        {
            continue;
        }
        int32_t body = world->shapeBody[i];
        M2_CHECK_INVARIANT(body >= 0 && body < world->bodyCapacity && world->alive[body] != 0);
    }
    for (int32_t i = 0; i < world->maxJointIndex; ++i)
    {
        if (world->jointAlive[i] == 0)
        {
            continue;
        }
        M2_CHECK_INVARIANT(world->jointType[i] <= 10);
        int32_t a = world->jointBodyA[i];
        int32_t b = world->jointBodyB[i];
        M2_CHECK_INVARIANT(a >= 0 && a < world->bodyCapacity && world->alive[a] != 0);
        M2_CHECK_INVARIANT(b >= 0 && b < world->bodyCapacity && world->alive[b] != 0);
    }
    for (int32_t i = 1; i < world->pairCount; ++i)
    {
        // The canonical ordering law, checked where it lives.
        M2_CHECK_INVARIANT(world->pairKeys[i - 1] < world->pairKeys[i]);
    }
    if (world->particleCapacity > 0)
    {
        int32_t alive = 0;
        for (int32_t i = 0; i < world->maxParticleIndex; ++i)
        {
            if (world->particleAlive[i] == 0)
            {
                continue;
            }
            alive += 1;
            m2Pos2 p = world->particlePositions[i];
            M2_CHECK_INVARIANT(m2FinitePos2(p));
            m2Vec2 v = world->particleVelocities[i];
            M2_CHECK_INVARIANT(m2FiniteVec2(v));
        }
        M2_CHECK_INVARIANT(alive == world->particleCount);
        for (int32_t k = 0; k < world->particleSpringCount; ++k)
        {
            M2_CHECK_INVARIANT(world->particleAlive[world->particleSpringA[k]] != 0 &&
                               world->particleAlive[world->particleSpringB[k]] != 0);
        }
        for (int32_t k = 0; k < world->particleTriadCount; ++k)
        {
            M2_CHECK_INVARIANT(world->particleAlive[world->particleTriadA[k]] != 0 &&
                               world->particleAlive[world->particleTriadB[k]] != 0 &&
                               world->particleAlive[world->particleTriadC[k]] != 0);
        }
    }
#undef M2_CHECK_INVARIANT
    return true;
}
