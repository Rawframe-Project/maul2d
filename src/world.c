// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: the registry of live worlds, creation and destruction,
// the step, world settings, enumeration and the invariant check.

#include "world.h"

#include "buoyancy.h"
#include "island.h"
#include "particle.h"
#include "solver.h"
#include "world_state.h"

#include "broadphase.h"
#include "contact.h"
#include "journal.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

#define M2_MAX_WORLDS 16
_Static_assert(M2_MAX_WORLDS <= (1 << M2_WORLD_SLOT_BITS), "a world slot fits the id bits");

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

m2World* m2WorldFromTag(uint16_t tag)
{
    // Slots past the table read as empty rather than wrapping onto a
    // live world.
    uint32_t slot = tag & ((1u << M2_WORLD_SLOT_BITS) - 1u);
    m2World* world = slot < M2_MAX_WORLDS ? s_worlds[slot] : NULL;
    if (world == NULL || world->idWorld != tag)
    {
        m2Refuse(NULL, m2_errorInvalid); // an id of a world that is gone
        return NULL;
    }
    return world;
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
    def.enableSleeping = true;
    def.bodyCapacity = 1024;
    def.shapeCapacity = 2048;
    def.jointCapacity = 256;
    def.particleCapacity = 0;    // fluids are opt-in
    def.fluidVolumeCapacity = 0; // buoyancy volumes are opt-in
    def.particleRadius = 0.05f;
    def.particleDensity = 1.0f;
    def.particleGravityScale = 1.0f;
    def.particlePressureStrength = 0.2f;
    def.particleDampingStrength = 1.0f;
    def.particleViscousStrength = 0.25f; // viscous-flagged particles only
    def.particlePowderStrength = 0.5f;
    def.particleSpringStrength = 0.25f;  // overlapping nets sum
    def.particleElasticStrength = 0.25f; // stiff blobs combine spring|elastic flags
    def.particleCohesionStrength = 0.2f;
    def.particleNearPressureStrength = 0.2f;
    def.internalValue = M2_WORLD_COOKIE;
    return def;
}

// Checks a world def. The fluids radius floor is 4x linear slop so the
// skin laws keep meaning.
static bool ValidWorldDef(const m2WorldDef* def)
{
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE || def->bodyCapacity < 1 ||
        def->shapeCapacity < 1 || def->jointCapacity < 1 || def->fluidVolumeCapacity < 0 ||
        def->particleCapacity < 0)
    {
        return false;
    }
    if (def->particleCapacity == 0)
    {
        return true;
    }
    return def->particleRadius >= 0.02f && def->particleDensity > 0.0f &&
           m2FiniteF(def->particleGravityScale) && def->particlePressureStrength >= 0.0f &&
           def->particleDampingStrength >= 0.0f && def->particleViscousStrength >= 0.0f &&
           def->particleCohesionStrength >= 0.0f && def->particleNearPressureStrength >= 0.0f &&
           def->particlePowderStrength >= 0.0f && def->particleSpringStrength >= 0.0f &&
           def->particleElasticStrength >= 0.0f;
}

// Copies the def's settings and capacities into a zeroed world.
static void ApplyWorldDef(m2World* world, const m2WorldDef* def)
{
    int32_t shapeCap = def->shapeCapacity;
    world->gravity = def->gravity;
    world->windVelocity = (m2Vec2){0.0f, 0.0f};
    world->windLinearDrag = 0.0f; // wind is opt-in via m2World_SetWind
    world->bodies.bodyCapacity = def->bodyCapacity;
    world->shapes.shapeCapacity = shapeCap;
    world->joints.jointCapacity = def->jointCapacity;
    world->broadphase.treeNodeCapacity = 2 * shapeCap;
    world->contacts.pairCapacity = 8 * shapeCap;
    world->volumes.fvCapacity = def->fluidVolumeCapacity;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;

    m2Particles* p = &world->particles;
    int32_t particleCap = def->particleCapacity;
    p->particleCapacity = particleCap;
    p->particleRadius = def->particleRadius;
    p->particleDensity = def->particleDensity;
    p->particleGravityScale = def->particleGravityScale;
    p->particlePressureStrength = def->particlePressureStrength;
    p->particleDampingStrength = def->particleDampingStrength;
    p->particleViscousStrength = def->particleViscousStrength;
    p->particleCohesion = def->particleCohesionStrength;
    p->particlePowderStrength = def->particlePowderStrength;
    p->particleSpringStrength = def->particleSpringStrength;
    p->particleElasticStrength = def->particleElasticStrength;
    p->particleNearPressure = def->particleNearPressureStrength;
    p->particlePairCapacity = 12 * particleCap;
    p->particleSpringCapacity = 4 * particleCap;
    p->particleTriadCapacity = 2 * particleCap;
    p->particleBodyCapacity = 4 * particleCap;
}

// Fills every free list and clears every link of freshly allocated state.
static void SeedFreeLists(m2World* world)
{
    int32_t cap = world->bodies.bodyCapacity;
    int32_t shapeCap = world->shapes.shapeCapacity;
    int32_t jointCap = world->joints.jointCapacity;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeInit(&world->broadphase.trees[t], world->broadphase.treeNodes[t],
                   world->broadphase.treeNodeCapacity);
    }
    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodies.freeQueue[i] = i;
        world->bodies.bodyShapeHead[i] = -1;
        world->joints.bodyJointHead[i] = -1;
    }
    for (int32_t i = 0; i < shapeCap; ++i)
    {
        world->shapes.shapeFreeQueue[i] = i;
        world->shapes.shapeNext[i] = -1;
        world->broadphase.proxyIds[i] = M2_NULL_NODE;
        world->shapes.shapeChain[i] = -1;
        world->chains.chainFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < jointCap; ++i)
    {
        world->joints.jointFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < 2 * jointCap; ++i)
    {
        world->joints.jointEdgeNext[i] = -1;
    }
    for (int32_t i = 0; i < world->particles.particleCapacity; ++i)
    {
        world->particles.particleFreeQueue[i] = i;
    }
    for (int32_t i = 0; i < world->volumes.fvCapacity; ++i)
    {
        world->volumes.fvFreeQueue[i] = i;
    }
    world->joints.jointFreeCount = jointCap;
    world->particles.particleFreeCount = world->particles.particleCapacity;
    world->volumes.fvFreeCount = world->volumes.fvCapacity;
    world->bodies.freeCount = cap;
    world->shapes.shapeFreeCount = shapeCap;
    world->chains.chainFreeCount = shapeCap;
}

m2WorldId m2CreateWorld(const m2WorldDef* def)
{
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a typed refusal beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    if (m2VerifyCpuBackend() == 0)
    {
        m2Refuse(NULL, m2_errorConfig);
        return m2_nullWorldId;
    }
    if (!ValidWorldDef(def))
    {
        m2Refuse(NULL, m2_errorInvalid);
        return m2_nullWorldId;
    }
    int32_t slot = -1;
    for (int32_t i = 0; i < M2_MAX_WORLDS && slot < 0; ++i)
    {
        slot = s_worlds[i] == NULL ? i : -1;
    }
    m2World* world = slot < 0 ? NULL : m2AllocZeroed(sizeof(m2World));
    if (world == NULL)
    {
        m2Refuse(NULL, m2_errorCapacity);
        return m2_nullWorldId;
    }
    ApplyWorldDef(world, def);
    if (!m2StateAllocate(world))
    {
        m2Refuse(NULL, m2_errorCapacity); // out of memory
        m2WorldId failed = {(uint16_t)(slot + 1), s_worldGenerations[slot]};
        s_worlds[slot] = world;
        m2DestroyWorld(failed);
        return m2_nullWorldId;
    }
    SeedFreeLists(world);

    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
    world->slot = (uint16_t)slot;
    world->idWorld =
        (uint16_t)(((uint32_t)world->worldGeneration << M2_WORLD_SLOT_BITS) | (uint32_t)slot);
    world->sleepEnabled = def->enableSleeping ? 1 : 0;
    s_worlds[slot] = world;
    return (m2WorldId){(uint16_t)(slot + 1), world->worldGeneration};
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

// Opens a fresh event window: clears the public buffers, then flushes the
// ends queued by destroys between steps (they belong to this window).
static void OpenEventWindow(m2World* world)
{
    world->events.beginEventCount = 0;
    world->events.endEventCount = 0;
    for (int32_t i = 0; i < world->events.pendingEndCount && i < world->contacts.pairCapacity; ++i)
    {
        world->events.endEvents[world->events.endEventCount++] = world->events.pendingEndEvents[i];
    }
    world->events.pendingEndCount = 0;
    world->events.sensorBeginCount = 0;
    world->events.sensorEndCount = 0;
    for (int32_t i = 0; i < world->events.pendingSensorEndCount && i < world->contacts.pairCapacity;
         ++i)
    {
        world->events.sensorEndEvents[world->events.sensorEndCount++] =
            world->events.pendingSensorEnd[i];
    }
    world->events.pendingSensorEndCount = 0;
    world->events.jointBreakEventCount = 0;
}

// Hibernation: when every dynamic body sleeps, no kinematic moves and
// nothing was teleported, the pipeline provably changes no state (frozen
// pairs copy themselves, islands rebuild to the same roots, the solver
// has no constraints), so the step can skip it bit-identically.
static bool WorldIsHibernating(const m2World* world)
{
    if (world->broadphase.movedCount != 0 || world->particles.particleCount != 0)
    {
        return false;
    }
    bool anyoneStirring = false;
    for (int32_t i = 0; i < world->bodies.maxBodyIndex && !anyoneStirring; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        if (world->bodies.types[i] == (uint8_t)m2_dynamicBody)
        {
            // A body that JUST fell asleep still owes one manifold
            // refresh (its stash can be one solve stale - the same
            // freshness rule the frozen-pair skip lives by).
            anyoneStirring = world->bodies.disabled[i] == 0 &&
                             (world->bodies.asleep[i] == 0 || world->bodies.sleepStreak[i] < 2);
        }
        else if (world->bodies.types[i] == (uint8_t)m2_kinematicBody)
        {
            anyoneStirring = world->bodies.linearVelocities[i].x != 0.0f ||
                             world->bodies.linearVelocities[i].y != 0.0f ||
                             world->bodies.angularVelocities[i] != 0.0f;
        }
    }
    return !anyoneStirring;
}

// Refits the proxies of every moving body whose tight box left its fat
// box: fixed body order, shape-list order within a body.
static void MoveProxies(m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.disabled[i] != 0 ||
            world->bodies.types[i] == (uint8_t)m2_staticBody ||
            (world->bodies.types[i] == (uint8_t)m2_dynamicBody && world->bodies.asleep[i] != 0))
        {
            continue;
        }
        for (int32_t s = world->bodies.bodyShapeHead[i]; s != -1; s = world->shapes.shapeNext[s])
        {
            m2Aabb tight = m2ShapeTightAabb(world, s);
            int32_t tree = m2ShapeTreeIndex(world, s);
            if (!m2Aabb_Contains(
                    world->broadphase.treeNodes[tree][world->broadphase.proxyIds[s]].aabb, tight))
            {
                m2TreeMove(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                           world->broadphase.proxyIds[s], m2Fatten(tight));
                m2PushMoved(world, s);
            }
        }
    }
}

// Begin and end events for every pair whose touching flag changed, in
// canonical pair order.
static void EmitTouchEvents(m2World* world)
{
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        uint8_t touchingNow = world->contacts.manifolds[i].pointCount > 0 ? 1 : 0;
        if (touchingNow != world->contacts.pairTouching[i])
        {
            int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
            bool sensor = world->shapes.shapeSensor[a] != 0 || world->shapes.shapeSensor[b] != 0;
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
            world->contacts.pairTouching[i] = touchingNow;
        }
    }
}

// Particle lifetimes count down and expire in ascending slot order at
// step end; derived from state, so no journal op, and it replays and
// rolls back by itself.
static void AgeParticles(m2World* world, float dt)
{
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0 || world->particles.particleLifetime[i] <= 0.0f)
        {
            continue;
        }
        world->particles.particleLifetime[i] -= dt;
        if (world->particles.particleLifetime[i] <= 0.0f)
        {
            m2ParticleId dying = {i + 1, world->idWorld, world->particles.particleGenerations[i]};
            uint8_t journalWas = world->recorder.journalActive;
            world->recorder.journalActive = 0; // derived death is never recorded
            m2DestroyParticle(dying);
            world->recorder.journalActive = journalWas;
        }
    }
}

// Two consecutive step ends asleep guarantee the stashed manifolds were
// computed from these exact transforms.
static void AgeSleepStreaks(m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        world->bodies.sleepStreak[i] =
            world->bodies.asleep[i] != 0
                ? (uint8_t)(world->bodies.sleepStreak[i] < 2 ? world->bodies.sleepStreak[i] + 1 : 2)
                : 0;
    }
}

// Forces live for exactly one step.
static void ClearForces(m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        world->bodies.forces[i] = (m2Vec2){0.0f, 0.0f};
        world->bodies.torques[i] = 0.0f;
    }
}

// Wall-clock stage times from the step's clock marks: start, pairs,
// contacts, islands, solve, end. Diagnostics only.
static void RecordProfile(m2World* world, const uint64_t marks[6])
{
    world->profile.stepMs = (float)((double)(marks[5] - marks[0]) * 1.0e-6);
    world->profile.pairsMs = (float)((double)(marks[1] - marks[0]) * 1.0e-6);
    world->profile.contactsMs = (float)((double)(marks[2] - marks[1]) * 1.0e-6);
    world->profile.solveMs = (float)((double)(marks[4] - marks[3]) * 1.0e-6);
    world->profile.sleepMs =
        (float)((double)(marks[3] - marks[2]) * 1.0e-6 + (double)(marks[5] - marks[4]) * 1.0e-6);
}

void m2World_Step(m2WorldId worldId, float dt, int32_t substepCount)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || !m2FiniteF(dt) || !(dt > 0.0f) || substepCount < 1)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpStep marker;
        memset(&marker, 0, sizeof(marker));
        marker.dt = dt;
        marker.substepCount = substepCount;
        m2JournalRecord(world, m2_opStep, &marker, (int32_t)sizeof(marker));
    }
    OpenEventWindow(world);

    // Wall-clock diagnostics only; never fed back into simulation.
    uint64_t tStart = m2TimeNowNs();
    if (WorldIsHibernating(world))
    {
        world->profile.stepMs = (float)((double)(m2TimeNowNs() - tStart) * 1.0e-6);
        world->profile.pairsMs = 0.0f;
        world->profile.contactsMs = 0.0f;
        world->profile.solveMs = 0.0f;
        world->profile.sleepMs = 0.0f;
        world->stepCount += 1;
        return;
    }

    // Collide first: broadphase and narrowphase build fresh manifolds from
    // the current positions (warm-start impulses carry over through them),
    // then the solver moves the world.
    MoveProxies(world);
    world->contacts.oldPairCount = world->contacts.pairCount;
    m2StashContacts(world);
    m2UpdatePairs(world);
    uint64_t tPairs = m2TimeNowNs();
    m2UpdateContacts(world);
    uint64_t tContacts = m2TimeNowNs();
    EmitTouchEvents(world);

    if (world->particles.particleCount > 0)
    {
        // The fluid pass runs once per step before the rigid solve, with
        // pairs frozen at step start, and before the island update, so a
        // body the water wakes pulls its whole island awake.
        m2SolveParticles(world, dt);
        AgeParticles(world, dt);
    }
    if (world->volumes.maxFvIndex > 0)
    {
        // Buoyancy feeds the force accumulators before the solve, so it
        // integrates alongside gravity and dies with the step.
        m2ApplyFluidVolumes(world, dt);
    }
    if (world->windLinearDrag > 0.0f)
    {
        // Wind after buoyancy and before the solve: an area-weighted
        // linear drag toward the wind velocity, into the same accumulators.
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
    AgeSleepStreaks(world);
    uint64_t tEnd = m2TimeNowNs();

    uint64_t marks[6] = {tStart, tPairs, tContacts, tIslands, tSolve, tEnd};
    RecordProfile(world, marks);
    ClearForces(world);
    world->stepCount += 1;
}

uint64_t m2World_GetStepCount(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL ? world->stepCount : 0;
}

m2MemoryUsage m2World_GetMemoryUsage(m2WorldId worldId)
{
    m2MemoryUsage usage = {0};
    m2World* world = m2GetWorld(worldId);
    if (world != NULL)
    {
        usage.persistentBytes = world->memoryBytes + (int64_t)sizeof(m2World);
    }
    return usage;
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
    if (world->recorder.journalActive != 0)
    {
        m2OpFlag record;
        record.flag = next;
        m2JournalRecord(world, m2_opEnableSleeping, &record, (int32_t)sizeof(record));
    }
    world->sleepEnabled = next;
    if (next == 0)
    {
        // The rule that let them sleep is gone; wake everyone (the
        // SetGravity law).
        for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
        {
            if (world->bodies.alive[i] != 0 && world->bodies.types[i] == (uint8_t)m2_dynamicBody)
            {
                world->bodies.asleep[i] = 0;
                world->bodies.sleepTimes[i] = 0.0f;
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
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.invMass[i] == 0.0f)
        {
            continue;
        }
        double vx = (double)world->bodies.linearVelocities[i].x;
        double vy = (double)world->bodies.linearVelocities[i].y;
        energy += 0.5 * (1.0 / (double)world->bodies.invMass[i]) * (vx * vx + vy * vy);
        if (world->bodies.invInertia[i] > 0.0f)
        {
            double w = (double)world->bodies.angularVelocities[i];
            energy += 0.5 * (1.0 / (double)world->bodies.invInertia[i]) * w * w;
        }
    }
    return energy;
}

void m2World_SetGravity(m2WorldId worldId, m2Vec2 gravity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || !m2FiniteVec2(gravity))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->gravity.x == gravity.x && world->gravity.y == gravity.y)
    {
        return; // no-op, not journaled
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpVec record;
        memset(&record, 0, sizeof(record));
        record.value = gravity;
        m2JournalRecord(world, m2_opSetGravity, &record, (int32_t)sizeof(record));
    }
    world->gravity = gravity;
    // Honesty over precedent: a sleeping stack must feel the new
    // world. Wake every dynamic sleeper (deterministic, one pass).
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] != 0 && world->bodies.types[i] == (uint8_t)m2_dynamicBody &&
            world->bodies.asleep[i] != 0)
        {
            world->bodies.asleep[i] = 0;
            world->bodies.sleepTimes[i] = 0.0f;
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
    if (world == NULL || !m2FiniteF(linearDrag) || linearDrag < 0.0f || !m2FiniteVec2(velocity))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->windVelocity.x == velocity.x && world->windVelocity.y == velocity.y &&
        world->windLinearDrag == linearDrag)
    {
        return; // no-op, not journaled
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpSetWind record;
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

m2Counters m2World_GetCounters(m2WorldId worldId)
{
    m2Counters counters;
    memset(&counters, 0, sizeof(counters));
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return counters;
    }
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        counters.bodies += 1;
        counters.awakeBodies += world->bodies.asleep[i] == 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        counters.shapes += world->shapes.shapeAlive[i] != 0 ? 1 : 0;
    }
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        counters.joints += world->joints.jointAlive[i] != 0 ? 1 : 0;
    }
    counters.pairs = world->contacts.pairCount;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        counters.touchingPairs += world->contacts.pairTouching[i] != 0 ? 1 : 0;
    }
    counters.constraints = world->solver.lastConstraintCount;
    counters.graphColors = world->solver.lastGraphColors;
    counters.overflowConstraints = world->solver.lastOverflow;
    counters.stepCount = world->stepCount;
    counters.pairOverflow = world->contacts.pairOverflow;
    counters.particlePairOverflow = world->particles.particlePairOverflow;
    counters.particleBodyOverflow = world->particles.particleBodyOverflow;
    counters.particlePoolFull = world->particles.particlePoolFullCount;
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
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2BodyId id = {i + 1, world->idWorld, world->bodies.generations[i]};
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
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        if (world->joints.jointAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {i + 1, world->idWorld, world->joints.jointGenerations[i]};
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
    for (int32_t i = 0; i < world->chains.maxChainIndex; ++i)
    {
        if (world->chains.chainAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            m2ChainId id = {i + 1, world->idWorld, world->chains.chainGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

// Asserts in debug builds and fails the walk in release builds.
#define M2_CHECK_INVARIANT(cond)                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            M2_ASSERT(false);                                                                      \
            return false;                                                                          \
        }                                                                                          \
    } while (0)

static bool ValidBodies(const m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0)
        {
            continue;
        }
        m2Transform xf = world->bodies.transforms[i];
        M2_CHECK_INVARIANT(m2FinitePos2(xf.p));
        M2_CHECK_INVARIANT(m2FiniteF(xf.q.c) && m2FiniteF(xf.q.s));
        m2Vec2 v = world->bodies.linearVelocities[i];
        M2_CHECK_INVARIANT(m2FiniteVec2(v));
        M2_CHECK_INVARIANT(m2FiniteF(world->bodies.angularVelocities[i]));
        M2_CHECK_INVARIANT(world->bodies.types[i] <= 2);
    }
    return true;
}

// Shapes and joints point at live bodies; contact pairs stay in key order.
static bool ValidLinks(const m2World* world)
{
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        if (world->shapes.shapeAlive[i] == 0)
        {
            continue;
        }
        int32_t body = world->shapes.shapeBody[i];
        M2_CHECK_INVARIANT(body >= 0 && body < world->bodies.bodyCapacity &&
                           world->bodies.alive[body] != 0);
    }
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        if (world->joints.jointAlive[i] == 0)
        {
            continue;
        }
        M2_CHECK_INVARIANT(world->joints.jointType[i] <= 10);
        int32_t a = world->joints.jointBodyA[i];
        int32_t b = world->joints.jointBodyB[i];
        M2_CHECK_INVARIANT(a >= 0 && a < world->bodies.bodyCapacity && world->bodies.alive[a] != 0);
        M2_CHECK_INVARIANT(b >= 0 && b < world->bodies.bodyCapacity && world->bodies.alive[b] != 0);
    }
    for (int32_t i = 1; i < world->contacts.pairCount; ++i)
    {
        // The canonical ordering law, checked where it lives.
        M2_CHECK_INVARIANT(world->contacts.pairKeys[i - 1] < world->contacts.pairKeys[i]);
    }
    return true;
}

static bool ValidParticles(const m2World* world)
{
    if (world->particles.particleCapacity == 0)
    {
        return true;
    }
    int32_t alive = 0;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        alive += 1;
        m2Pos2 p = world->particles.particlePositions[i];
        M2_CHECK_INVARIANT(m2FinitePos2(p));
        m2Vec2 v = world->particles.particleVelocities[i];
        M2_CHECK_INVARIANT(m2FiniteVec2(v));
    }
    M2_CHECK_INVARIANT(alive == world->particles.particleCount);
    for (int32_t k = 0; k < world->particles.particleSpringCount; ++k)
    {
        M2_CHECK_INVARIANT(
            world->particles.particleAlive[world->particles.particleSpringA[k]] != 0 &&
            world->particles.particleAlive[world->particles.particleSpringB[k]] != 0);
    }
    for (int32_t k = 0; k < world->particles.particleTriadCount; ++k)
    {
        M2_CHECK_INVARIANT(
            world->particles.particleAlive[world->particles.particleTriadA[k]] != 0 &&
            world->particles.particleAlive[world->particles.particleTriadB[k]] != 0 &&
            world->particles.particleAlive[world->particles.particleTriadC[k]] != 0);
    }
    return true;
}

#undef M2_CHECK_INVARIANT

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
    return ValidBodies(world) && ValidLinks(world) && ValidParticles(world);
}
