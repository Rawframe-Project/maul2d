// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world: creation and destruction, the step, world settings,
// enumeration; world_validate.c holds the invariant check.

#include "world.h"

#include "allocator.h"
#include "broadphase.h"
#include "buoyancy.h"
#include "contact.h"
#include "cpu.h"
#include "fpenv.h"
#include "island.h"
#include "journal.h"
#include "parallel.h"
#include "particle.h"
#include "proxies.h"
#include "snapshot.h"
#include "solver.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <stdckdint.h>
#include <string.h>

// --- Defs & world lifecycle ----------------------------------------------------

m2WorldDef m2DefaultWorldDefValues(void)
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

// Checks a world def past its cookie and version. The fluids radius
// floor is 4x linear slop so the skin laws keep meaning.
static bool ValidWorldDef(const m2WorldDef* def)
{
    if (def->bodyCapacity < 1 || def->shapeCapacity < 1 || def->jointCapacity < 1 ||
        def->fluidVolumeCapacity < 0 || def->particleCapacity < 0 ||
        (def->allocator.alloc == NULL) != (def->allocator.free == NULL) ||
        !m2ValidAcceleration(def->gravity))
    {
        return false;
    }
    if (def->particleCapacity == 0)
    {
        return true;
    }
    // A particle's mass is its density over a square of 1.5 radii, the
    // spacing of a fill; it must be a mass a body could have.
    float stride = 1.5f * def->particleRadius;
    return m2WithinF(def->particleRadius, 0.02f, M2_HUGE) && m2ValidDensity(def->particleDensity) &&
           m2ValidMass(def->particleDensity * stride * stride) &&
           m2ValidGravityScale(def->particleGravityScale) &&
           m2ValidCoefficient(def->particlePressureStrength) &&
           m2ValidCoefficient(def->particleDampingStrength) &&
           m2ValidCoefficient(def->particleViscousStrength) &&
           m2ValidCoefficient(def->particleCohesionStrength) &&
           m2ValidCoefficient(def->particleNearPressureStrength) &&
           m2ValidCoefficient(def->particlePowderStrength) &&
           m2ValidCoefficient(def->particleSpringStrength) &&
           m2ValidCoefficient(def->particleElasticStrength);
}

// Copies the def's settings and capacities into a zeroed world. The
// derived capacities are checked products, so a def too large to address
// is refused instead of wrapping.
static bool ApplyWorldDef(m2World* world, const m2WorldDef* def)
{
    int32_t shapeCap = def->shapeCapacity;
    int32_t particleCap = def->particleCapacity;
    m2Particles* p = &world->particles;
    if (ckd_mul(&world->broadphase.treeNodeCapacity, 2, shapeCap) ||
        ckd_mul(&world->contacts.pairCapacity, 8, shapeCap) ||
        ckd_mul(&p->particlePairCapacity, 12, particleCap) ||
        ckd_mul(&p->particleSpringCapacity, 4, particleCap) ||
        ckd_mul(&p->particleTriadCapacity, 2, particleCap) ||
        ckd_mul(&p->particleBodyCapacity, 4, particleCap))
    {
        return false;
    }
    world->gravity = def->gravity;
    world->windVelocity = (m2Vec2){0.0f, 0.0f};
    world->windLinearDrag = 0.0f; // wind is opt-in via m2World_SetWind
    world->bodies.bodyCapacity = def->bodyCapacity;
    world->shapes.shapeCapacity = shapeCap;
    world->joints.jointCapacity = def->jointCapacity;
    world->volumes.fvCapacity = def->fluidVolumeCapacity;
    world->enqueueTask = def->enqueueTask;
    world->finishTask = def->finishTask;
    world->userTaskContext = def->userTaskContext;

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
    return true;
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
    // Every body slot, live or not, holds a unit rotation, which is what
    // a restore checks; a destroyed body leaves its last one behind.
    m2Transform identity = {{0.0, 0.0}, {1.0f, 0.0f}};
    for (int32_t i = 0; i < cap; ++i)
    {
        world->bodies.transforms[i] = identity;
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

static M2_NOINLINE m2Result CreateWorldWorker(const m2WorldDef* def, m2World** worldOut)
{
    if (worldOut == NULL)
    {
        return m2_errorInvalid;
    }
    *worldOut = NULL;
    // Before any solver kernel runs, make sure this CPU can execute the
    // backend the binary was built for: a typed refusal beats a bare
    // illegal-instruction trap on pre-Haswell hardware.
    if (m2CheckCpu(NULL) == 0)
    {
        return m2_errorUnsupported;
    }
    if (def == NULL || def->internalValue != M2_WORLD_COOKIE)
    {
        return m2_errorInvalid;
    }
    if (def->version != M2_ABI_VERSION)
    {
        return m2_errorVersion;
    }
    if (!ValidWorldDef(def))
    {
        return m2_errorInvalid;
    }
    // The world's snapshot, and a journal holding two of them, must fit
    // their int32_t byte counts: measured on the def's capacities alone,
    // before anything is allocated.
    m2World shape;
    memset(&shape, 0, sizeof(shape));
    if (!ApplyWorldDef(&shape, def) || !m2SnapshotFits(&shape))
    {
        return m2_errorCapacity;
    }
    m2Allocator allocator = m2ResolveAllocator(&def->allocator);
    m2World* world = m2AllocZeroed(&allocator, sizeof(m2World));
    if (world == NULL)
    {
        return m2_errorCapacity;
    }
    world->allocator = allocator;
    if (!ApplyWorldDef(world, def) || !m2StateAllocate(world))
    {
        m2StateFree(world);
        m2Free(&allocator, world, sizeof(m2World));
        return m2_errorCapacity;
    }
    SeedFreeLists(world);

    world->sleepEnabled = def->enableSleeping ? 1 : 0;
    *worldOut = world;
    return m2_success;
}

m2Result m2CreateWorld(const m2WorldDef* def, m2World** worldOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateWorldWorker(def, worldOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void DestroyWorldWorker(m2World* world)
{
    if (world == NULL)
    {
        return;
    }
    // The frees are host code, and nothing may change a world being
    // destroyed: its writers refuse from here on.
    m2EnterHostCode(world);
    m2Allocator allocator = world->allocator;
    m2StateFree(world);
    m2Free(&allocator, world, sizeof(m2World));
}

void m2DestroyWorld(m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    DestroyWorldWorker(world);
    m2FloatEnvLeave(env);
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
        m2RefitBodyProxies(world, i);
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
            m2DestroyParticleInternal(world, i); // derived death is never recorded
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

static void StepWorld(m2World* world, float dt, int32_t substepCount);

static M2_NOINLINE void WorldStepWorker(m2World* world, float dt, int32_t substepCount)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (!m2ValidTimeStep(dt) || substepCount < 1 || substepCount > M2_MAX_SUBSTEPS)
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
    // The task hooks run host code inside the step: until it returns, the
    // world's writers refuse.
    m2EnterHostCode(world);
    StepWorld(world, dt, substepCount);
    m2LeaveHostCode(world);
}

void m2World_Step(m2World* world, float dt, int32_t substepCount)
{
    uint64_t env = m2FloatEnvEnter();
    WorldStepWorker(world, dt, substepCount);
    m2FloatEnvLeave(env);
}

// The step itself, after the arguments are checked and the step recorded.
static void StepWorld(m2World* world, float dt, int32_t substepCount)
{
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
    M2_ASSERT(m2World_Validate(world));
#endif
    AgeSleepStreaks(world);
    uint64_t tEnd = m2TimeNowNs();

    uint64_t marks[6] = {tStart, tPairs, tContacts, tIslands, tSolve, tEnd};
    RecordProfile(world, marks);
    ClearForces(world);
    world->stepCount += 1;
}

uint64_t m2World_GetStepCount(const m2World* world)
{
    return world != NULL ? world->stepCount : 0;
}

m2MemoryUsage m2World_GetMemoryUsage(const m2World* world)
{
    m2MemoryUsage usage = {0};
    if (world != NULL)
    {
        usage.persistentBytes = world->memoryBytes + (int64_t)sizeof(m2World);
    }
    return usage;
}

static M2_NOINLINE void WorldEnableSleepingWorker(m2World* world, bool flag)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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

void m2World_EnableSleeping(m2World* world, bool flag)
{
    uint64_t env = m2FloatEnvEnter();
    WorldEnableSleepingWorker(world, flag);
    m2FloatEnvLeave(env);
}

bool m2World_IsSleepingEnabled(const m2World* world)
{
    return world != NULL && world->sleepEnabled != 0;
}

static M2_NOINLINE m2Profile WorldGetProfileWorker(const m2World* world)
{
    if (world == NULL)
    {
        m2Profile zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        return zero;
    }
    return world->profile;
}

m2Profile m2World_GetProfile(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    m2Profile result = WorldGetProfileWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE double WorldGetKineticEnergyWorker(const m2World* world)
{
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

double m2World_GetKineticEnergy(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    double result = WorldGetKineticEnergyWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void WorldSetGravityWorker(m2World* world, m2Vec2 gravity)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (world == NULL || !m2ValidAcceleration(gravity))
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

void m2World_SetGravity(m2World* world, m2Vec2 gravity)
{
    uint64_t env = m2FloatEnvEnter();
    WorldSetGravityWorker(world, gravity);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE m2Vec2 WorldGetGravityWorker(const m2World* world)
{
    return world != NULL ? world->gravity : (m2Vec2){0.0f, 0.0f};
}

m2Vec2 m2World_GetGravity(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = WorldGetGravityWorker(world);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void WorldSetWindWorker(m2World* world, m2Vec2 velocity, float linearDrag)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (world == NULL || !m2ValidRate(linearDrag) || !m2ValidVelocity(velocity))
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

void m2World_SetWind(m2World* world, m2Vec2 velocity, float linearDrag)
{
    uint64_t env = m2FloatEnvEnter();
    WorldSetWindWorker(world, velocity, linearDrag);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void WorldGetWindWorker(const m2World* world, m2Vec2* velocityOut,
                                           float* linearDragOut)
{
    if (velocityOut != NULL)
    {
        *velocityOut = world != NULL ? world->windVelocity : (m2Vec2){0.0f, 0.0f};
    }
    if (linearDragOut != NULL)
    {
        *linearDragOut = world != NULL ? world->windLinearDrag : 0.0f;
    }
}

void m2World_GetWind(const m2World* world, m2Vec2* velocityOut, float* linearDragOut)
{
    uint64_t env = m2FloatEnvEnter();
    WorldGetWindWorker(world, velocityOut, linearDragOut);
    m2FloatEnvLeave(env);
}

m2Counters m2World_GetCounters(const m2World* world)
{
    m2Counters counters;
    memset(&counters, 0, sizeof(counters));
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

static M2_NOINLINE int32_t WorldGetBodiesWorker(const m2World* world, m2BodyId* ids,
                                                int32_t capacity)
{
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
            m2BodyId id = {(uint32_t)(i + 1), world->bodies.generations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetBodies(const m2World* world, m2BodyId* ids, int32_t capacity)
{
    uint64_t env = m2FloatEnvEnter();
    int32_t result = WorldGetBodiesWorker(world, ids, capacity);
    m2FloatEnvLeave(env);
    return result;
}

int32_t m2World_GetJoints(const m2World* world, m2JointId* ids, int32_t capacity)
{
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
            m2JointId id = {(uint32_t)(i + 1), world->joints.jointGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

int32_t m2World_GetChains(const m2World* world, m2ChainId* ids, int32_t capacity)
{
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
            m2ChainId id = {(uint32_t)(i + 1), world->chains.chainGenerations[i]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}
