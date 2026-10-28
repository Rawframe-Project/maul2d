// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle storage: emitting and destroying particles and their
// per-particle state. The solver lives in particle_solver.c.

#include "particle.h"

#include "fpenv.h"
#include "geometry.h"
#include "journal.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

// --- Fluids: storage surface (the solver arrives in later slices) ------------------

static int32_t ParticleSlot(const m2World* world, m2ParticleId id)
{
    int32_t index = (int32_t)id.index1 - 1;
    if (world == NULL || index < 0 || index >= world->particles.particleCapacity ||
        world->particles.particleAlive[index] == 0 ||
        world->particles.particleGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ParticleDef m2DefaultParticleDef(void)
{
    m2ParticleDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_PARTICLE_COOKIE;
    return def;
}

// A particle def a create or a fill accepts. A world created with a
// particle pool of 0 refuses every particle as a full pool does.
static m2Result ParticleDefCheck(const m2ParticleDef* def)
{
    if (def == NULL || def->internalValue != M2_PARTICLE_COOKIE)
    {
        return m2_errorInvalid;
    }
    return m2BothChecks(m2CheckValid(m2FinitePos2(def->position)), m2CheckVelocity(def->velocity));
}

static M2_NOINLINE m2Result CreateParticleWorker(m2World* world, const m2ParticleDef* def,
                                                 m2ParticleId* particleOut)
{
    if (particleOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *particleOut = m2_nullParticleId;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    m2Result checked = ParticleDefCheck(def);
    if (checked != m2_success)
    {
        return m2Refuse(world, checked);
    }
    if (world->particles.particleFreeCount == 0)
    {
        // A full pool is a runtime fact, not misuse: pace emitters
        // off m2World_GetParticleCount; the counter keeps the score.
        world->particles.particlePoolFullCount += 1;
        return m2Refuse(world, m2_errorCapacity);
    }
    int32_t index = world->particles.particleFreeQueue[world->particles.particleFreeHead];
    world->particles.particleFreeHead =
        (world->particles.particleFreeHead + 1) % world->particles.particleCapacity;
    world->particles.particleFreeCount -= 1;
    if (index + 1 > world->particles.maxParticleIndex)
    {
        world->particles.maxParticleIndex = index + 1;
    }
    world->particles.particlePositions[index] = def->position;
    world->particles.particleVelocities[index] = def->velocity;
    world->particles.particleFlags[index] = def->flags;
    world->particles.particleLifetime[index] = 0.0f;
    world->particles.particleUserData[index] = 0;
    world->particles.particleAlive[index] = 1;
    world->particles.particleCount += 1;
    m2ParticleId id = {(uint32_t)(index + 1), world->particles.particleGenerations[index]};
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateParticle record;
        memset(&record, 0, sizeof(record));
        record.position = def->position;
        record.velocity = def->velocity;
        record.flags = def->flags;
        record.expected = id;
        m2JournalRecord(world, m2_opCreateParticle, &record, (int32_t)sizeof(record));
    }
    *particleOut = id;
    return m2_success;
}

m2Result m2CreateParticle(m2World* world, const m2ParticleDef* def, m2ParticleId* particleOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateParticleWorker(world, def, particleOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Result DestroyParticleWorker(m2World* world, m2ParticleId particleId)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        return m2_errorStale; // particles expire on their own: no misuse
    }
    if (world->recorder.journalActive != 0)
    {
        m2JournalRecord(world, m2_opDestroyParticle, &particleId, (int32_t)sizeof(particleId));
    }
    m2DestroyParticleInternal(world, index);
    return m2_success;
}

m2Result m2DestroyParticle(m2World* world, m2ParticleId particleId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = DestroyParticleWorker(world, particleId);
    m2FloatEnvLeave(env);
    return result;
}

void m2DestroyParticleInternal(m2World* world, int32_t index)
{
    world->particles.particleAlive[index] = 0;
    // Jelly bookkeeping: springs and triads die with their particle,
    // compacted in order so the lists stay canonical.
    if (world->particles.particleSpringCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particles.particleSpringCount; ++k)
        {
            if (world->particles.particleSpringA[k] == index ||
                world->particles.particleSpringB[k] == index)
            {
                continue;
            }
            world->particles.particleSpringA[keep] = world->particles.particleSpringA[k];
            world->particles.particleSpringB[keep] = world->particles.particleSpringB[k];
            world->particles.particleSpringRest[keep] = world->particles.particleSpringRest[k];
            keep += 1;
        }
        world->particles.particleSpringCount = keep;
    }
    if (world->particles.particleTriadCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particles.particleTriadCount; ++k)
        {
            if (world->particles.particleTriadA[k] == index ||
                world->particles.particleTriadB[k] == index ||
                world->particles.particleTriadC[k] == index)
            {
                continue;
            }
            world->particles.particleTriadA[keep] = world->particles.particleTriadA[k];
            world->particles.particleTriadB[keep] = world->particles.particleTriadB[k];
            world->particles.particleTriadC[keep] = world->particles.particleTriadC[k];
            world->particles.particleTriadPA[keep] = world->particles.particleTriadPA[k];
            world->particles.particleTriadPB[keep] = world->particles.particleTriadPB[k];
            world->particles.particleTriadPC[keep] = world->particles.particleTriadPC[k];
            keep += 1;
        }
        world->particles.particleTriadCount = keep;
    }
    world->particles.particleCount -= 1;
    if (world->particles.particleGenerations[index] == M2_LAST_GENERATION)
    {
        world->particles.particleRetiredCount += 1;
        return;
    }
    world->particles.particleGenerations[index] += 1;
    world->particles.particleFreeQueue[(world->particles.particleFreeHead +
                                        world->particles.particleFreeCount) %
                                       world->particles.particleCapacity] = index;
    world->particles.particleFreeCount += 1;
}

bool m2Particle_IsValid(const m2World* world, m2ParticleId particleId)
{
    return ParticleSlot(world, particleId) >= 0;
}

static M2_NOINLINE m2Pos2 ParticleGetPositionWorker(const m2World* world, m2ParticleId particleId)
{
    m2Pos2 zero = {0.0, 0.0};
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particlePositions[index] : zero;
}

m2Pos2 m2Particle_GetPosition(const m2World* world, m2ParticleId particleId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Pos2 result = ParticleGetPositionWorker(world, particleId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 ParticleGetVelocityWorker(const m2World* world, m2ParticleId particleId)
{
    m2Vec2 zero = {0.0f, 0.0f};
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleVelocities[index] : zero;
}

m2Vec2 m2Particle_GetVelocity(const m2World* world, m2ParticleId particleId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = ParticleGetVelocityWorker(world, particleId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void ParticleSetVelocityWorker(m2World* world, m2ParticleId particleId,
                                                  m2Vec2 velocity)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        return; // expired: changes nothing, counts nothing
    }
    if (!m2ValidVelocity(velocity))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpParticleVec record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetParticleVelocity, &record, (int32_t)sizeof(record));
    }
    world->particles.particleVelocities[index] = velocity;
}

void m2Particle_SetVelocity(m2World* world, m2ParticleId particleId, m2Vec2 velocity)
{
    uint64_t env = m2FloatEnvEnter();
    ParticleSetVelocityWorker(world, particleId, velocity);
    m2FloatEnvLeave(env);
}

uint32_t m2Particle_GetFlags(const m2World* world, m2ParticleId particleId)
{
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleFlags[index] : 0;
}

static M2_NOINLINE void ParticleSetLifetimeWorker(m2World* world, m2ParticleId particleId,
                                                  float seconds)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        return; // expired: changes nothing, counts nothing
    }
    if (!m2FiniteF(seconds) || seconds < 0.0f)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpParticleFloat record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.value = seconds;
        m2JournalRecord(world, m2_opSetParticleLifetime, &record, (int32_t)sizeof(record));
    }
    world->particles.particleLifetime[index] = seconds;
}

void m2Particle_SetLifetime(m2World* world, m2ParticleId particleId, float seconds)
{
    uint64_t env = m2FloatEnvEnter();
    ParticleSetLifetimeWorker(world, particleId, seconds);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE float ParticleGetLifetimeWorker(const m2World* world, m2ParticleId particleId)
{
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleLifetime[index] : 0.0f;
}

float m2Particle_GetLifetime(const m2World* world, m2ParticleId particleId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = ParticleGetLifetimeWorker(world, particleId);
    m2FloatEnvLeave(env);
    return result;
}

void m2Particle_SetUserData(m2World* world, m2ParticleId particleId, uint64_t userData)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        return; // expired: changes nothing, counts nothing
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpParticleUserData record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.userData = userData;
        m2JournalRecord(world, m2_opSetParticleUserData, &record, (int32_t)sizeof(record));
    }
    world->particles.particleUserData[index] = userData;
}

uint64_t m2Particle_GetUserData(const m2World* world, m2ParticleId particleId)
{
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleUserData[index] : 0;
}

int32_t m2World_GetParticleCount(const m2World* world)
{
    return world != NULL ? world->particles.particleCount : 0;
}

int32_t m2World_GetParticles(const m2World* world, m2ParticleId* ids, int32_t capacity)
{
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m2ParticleId){(uint32_t)(i + 1), world->particles.particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}

// --- Fills: a convex polygon of particles in one call ------------------------------

// The fill lattice is at most this many columns wide.

typedef struct FillBatch
{
    int32_t* emitted; // borrowed scratch, one slot per particle of capacity
    int32_t count;
    float stride;
    int32_t prevRow[M2_MAX_FILL_COLUMNS];
    int32_t currRow[M2_MAX_FILL_COLUMNS];
} FillBatch;

static bool InsidePolygon(const m2Polygon* polygon, float x, float y)
{
    for (int32_t i = 0; i < polygon->count; ++i)
    {
        m2Vec2 v = polygon->vertices[i];
        m2Vec2 n = polygon->normals[i];
        if (n.x * (x - v.x) + n.y * (y - v.y) > 0.0f)
        {
            return false;
        }
    }
    return true;
}

// Two triangles per complete lattice cell ending at column col, the rest
// shape centered on each triad's spawn centroid.
static void AddCellTriads(m2World* world, const FillBatch* batch, int32_t col, int32_t slot)
{
    int32_t left = col > 0 ? batch->currRow[col - 1] : -1;
    int32_t below = batch->prevRow[col];
    int32_t belowLeft = col > 0 ? batch->prevRow[col - 1] : -1;
    m2Particles* p = &world->particles;
    if (left < 0 || below < 0 || belowLeft < 0 ||
        p->particleTriadCount + 2 > p->particleTriadCapacity)
    {
        return;
    }
    float third = batch->stride / 3.0f;
    float twoThirds = 2.0f * batch->stride / 3.0f;
    int32_t t = p->particleTriadCount;
    p->particleTriadA[t] = belowLeft;
    p->particleTriadB[t] = below;
    p->particleTriadC[t] = left;
    p->particleTriadPA[t] = (m2Vec2){-third, -third};
    p->particleTriadPB[t] = (m2Vec2){twoThirds, -third};
    p->particleTriadPC[t] = (m2Vec2){-third, twoThirds};
    p->particleTriadA[t + 1] = below;
    p->particleTriadB[t + 1] = slot;
    p->particleTriadC[t + 1] = left;
    p->particleTriadPA[t + 1] = (m2Vec2){third, -twoThirds};
    p->particleTriadPB[t + 1] = (m2Vec2){third, third};
    p->particleTriadPC[t + 1] = (m2Vec2){-twoThirds, third};
    p->particleTriadCount = t + 2;
}

// Every batch pair inside one diameter becomes a spring that remembers
// its spawn length, in ascending order; a full spring pool truncates.
static void AddFillSprings(m2World* world, const FillBatch* batch)
{
    m2Particles* p = &world->particles;
    float diameter = 2.0f * p->particleRadius;
    for (int32_t i = 0; i < batch->count; ++i)
    {
        for (int32_t j = i + 1; j < batch->count; ++j)
        {
            int32_t a = batch->emitted[i];
            int32_t b = batch->emitted[j];
            float dx = (float)(p->particlePositions[b].x - p->particlePositions[a].x);
            float dy = (float)(p->particlePositions[b].y - p->particlePositions[a].y);
            float distSq = dx * dx + dy * dy;
            if (distSq >= diameter * diameter || distSq <= 0.0f ||
                p->particleSpringCount >= p->particleSpringCapacity)
            {
                continue;
            }
            int32_t k = p->particleSpringCount;
            p->particleSpringA[k] = a;
            p->particleSpringB[k] = b;
            p->particleSpringRest[k] = sqrtf(distSq);
            p->particleSpringCount = k + 1;
        }
    }
}

// Emits one lattice row; false once the pool is full.
static bool FillRow(m2World* world, FillBatch* batch, const m2Polygon* polygon, float minX,
                    float maxX, float y, const m2OpFillParticles* op)
{
    for (int32_t i = 0; i < M2_MAX_FILL_COLUMNS; ++i)
    {
        batch->currRow[i] = -1;
    }
    // The column coordinate accumulates stride by stride; the sum is part
    // of the fill result, so it is not recomputed from the column index.
    float x = minX + 0.5f * batch->stride;
    for (int32_t col = 0; col < M2_MAX_FILL_COLUMNS && x < maxX; ++col)
    {
        float atX = x;
        x += batch->stride;
        if (!InsidePolygon(polygon, atX, y))
        {
            continue;
        }
        m2ParticleDef one = m2DefaultParticleDef();
        one.position = (m2Pos2){op->position.x + (double)atX, op->position.y + (double)y};
        one.velocity = op->velocity;
        one.flags = op->flags;
        m2ParticleId id = m2_nullParticleId;
        if (CreateParticleWorker(world, &one, &id) != m2_success)
        {
            return false; // pool full: a quiet runtime fact
        }
        int32_t slot = (int32_t)id.index1 - 1;
        batch->emitted[batch->count++] = slot;
        batch->currRow[col] = slot;
        if ((op->flags & m2_elasticParticle) != 0)
        {
            AddCellTriads(world, batch, col, slot);
        }
    }
    memcpy(batch->prevRow, batch->currRow, sizeof(batch->prevRow));
    return true;
}

// The lower and upper corners of a polygon's vertices.
static void PolygonBounds(const m2Polygon* polygon, m2Vec2* lower, m2Vec2* upper)
{
    *lower = polygon->vertices[0];
    *upper = polygon->vertices[0];
    for (int32_t i = 1; i < polygon->count; ++i)
    {
        lower->x = m2MinF(lower->x, polygon->vertices[i].x);
        lower->y = m2MinF(lower->y, polygon->vertices[i].y);
        upper->x = m2MaxF(upper->x, polygon->vertices[i].x);
        upper->y = m2MaxF(upper->y, polygon->vertices[i].y);
    }
}

// Fill a convex polygon with particles at 0.75 diameters, row-major from
// the bottom-left of its bounds: a pool in one call.
static M2_NOINLINE m2Result CreateParticlesInPolygonWorker(m2World* world, const m2ParticleDef* def,
                                                           const m2Polygon* polygon,
                                                           int32_t* countOut)
{
    if (countOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *countOut = 0;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    // The region is the polygon a shape would take: within M2_LOCAL_LIMIT,
    // convex, not a sliver, with its normals recomputed from its vertices.
    m2Result checked = m2BothChecks(ParticleDefCheck(def), m2CheckPolygon(polygon));
    if (checked != m2_success)
    {
        return m2Refuse(world, checked);
    }
    if (world->particles.particleCapacity == 0)
    {
        // A pool of 0 is full; stop before the unchecked radius sizes a lattice.
        world->particles.particlePoolFullCount += 1;
        return m2Refuse(world, m2_errorCapacity);
    }
    m2Polygon region = m2CanonicalPolygon(polygon);
    if (region.count == 0)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    m2Vec2 lower;
    m2Vec2 upper;
    PolygonBounds(&region, &lower, &upper);
    FillBatch batch;
    batch.stride = 0.75f * 2.0f * world->particles.particleRadius;
    if ((upper.x - lower.x) / batch.stride >= (float)M2_MAX_FILL_COLUMNS)
    {
        return m2Refuse(world, m2_errorLimit); // wider than the fill lattice allows
    }
    m2OpFillParticles op;
    memset(&op, 0, sizeof(op));
    // The canonical region, so no unused slot of the caller's polygon
    // reaches the tape.
    op.polygon = region;
    op.position = def->position;
    op.velocity = def->velocity;
    op.flags = def->flags;

    // The whole fill is one journal op with the inner creates suppressed:
    // replay rebuilds the springs and triads from the op.
    uint8_t journalWas = world->recorder.journalActive;
    world->recorder.journalActive = 0;
    batch.emitted = (int32_t*)world->particles.particleProxies;
    batch.count = 0;
    for (int32_t i = 0; i < M2_MAX_FILL_COLUMNS; ++i)
    {
        batch.prevRow[i] = -1;
    }
    // The row coordinate accumulates stride by stride, since the sum is
    // part of the result. The fill ends where a row no longer moves it, or
    // where the pool runs out.
    bool full = false;
    bool open = true;
    float y = lower.y + 0.5f * batch.stride;
    while (y < upper.y && open)
    {
        full = !FillRow(world, &batch, &region, lower.x, upper.x, y, &op);
        float next = y + batch.stride;
        open = !full && next > y;
        y = next;
    }
    if ((def->flags & m2_springParticle) != 0)
    {
        AddFillSprings(world, &batch);
    }
    world->recorder.journalActive = journalWas;
    op.expected = batch.count;
    m2JournalRecord(world, m2_opFillParticles, &op, (int32_t)sizeof(op));
    *countOut = batch.count;
    // A fill the pool cut short keeps what it created and says so; a full
    // pool is a runtime fact, not misuse.
    return full ? m2_errorCapacity : m2_success;
}

m2Result m2CreateParticlesInPolygon(m2World* world, const m2ParticleDef* def,
                                    const m2Polygon* polygon, int32_t* countOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateParticlesInPolygonWorker(world, def, polygon, countOut);
    m2FloatEnvLeave(env);
    return result;
}

// Region query over the pool: a plain ascending scan. Particles
// carry no tree (their grid is step-transient); a linear
// walk over a fixed-capacity pool is deterministic and cheap.
static M2_NOINLINE int32_t WorldOverlapParticlesAabbWorker(const m2World* world, m2WorldAabb box,
                                                           m2ParticleId* ids, int32_t capacity)
{
    m2Pos2 lower = box.lowerBound;
    m2Pos2 upper = box.upperBound;
    if (world == NULL || !m2FinitePos2(lower) || !m2FinitePos2(upper))
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 p = world->particles.particlePositions[i];
        if (p.x < lower.x || p.x > upper.x || p.y < lower.y || p.y > upper.y)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m2ParticleId){(uint32_t)(i + 1), world->particles.particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}

int32_t m2World_OverlapParticlesAabb(const m2World* world, m2WorldAabb box, m2ParticleId* ids,
                                     int32_t capacity)
{
    uint64_t env = m2FloatEnvEnter();
    int32_t result = WorldOverlapParticlesAabbWorker(world, box, ids, capacity);
    m2FloatEnvLeave(env);
    return result;
}
