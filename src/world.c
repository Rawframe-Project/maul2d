// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: the registry of live worlds, creation and destruction,
// the step, world settings, enumeration and the invariant check.

#include "world.h"

#include "world_state.h"

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

    int32_t particleCap = def->particleCapacity;
    world->particlePairCapacity = 12 * particleCap;
    world->particleSpringCapacity = 4 * particleCap;
    world->particleTriadCapacity = 2 * particleCap;
    world->particleBodyCapacity = 4 * particleCap;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;

    bool ok = m2StateAllocate(world);
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
    m2StateFree(world);
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
