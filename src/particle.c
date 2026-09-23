// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle storage: emitting and destroying particles and their
// per-particle state. The solver lives in particle_solver.c.

#include "particle.h"

#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

// --- Fluids: storage surface (the solver arrives in later slices) ------------------

static int32_t ParticleSlot(const m2World* world, m2ParticleId id)
{
    int32_t index = id.index1 - 1;
    if (world == NULL || index < 0 || index >= world->particles.particleCapacity ||
        world->particles.particleAlive[index] == 0 ||
        world->particles.particleGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity,
                                  uint32_t flags)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || world->particles.particleCapacity == 0)
    {
        m2Refuse(world, m2_errorInvalid); // no particle system in this world: misuse
        return m2_nullParticleId;
    }
    if (!m2FinitePos2(position) || !m2FiniteVec2(velocity))
    {
        m2Refuse(world, m2_errorInvalid); // NaN screen, the def-validation law
        return m2_nullParticleId;
    }
    if (world->particles.particleFreeCount == 0)
    {
        // A full pool is a runtime fact, not misuse: pace emitters
        // off m2World_GetParticleCount; the counter keeps the score.
        world->particles.particlePoolFullCount += 1;
        m2Refuse(world, m2_errorCapacity);
        return m2_nullParticleId;
    }
    int32_t index = world->particles.particleFreeQueue[world->particles.particleFreeHead];
    world->particles.particleFreeHead =
        (world->particles.particleFreeHead + 1) % world->particles.particleCapacity;
    world->particles.particleFreeCount -= 1;
    if (index + 1 > world->particles.maxParticleIndex)
    {
        world->particles.maxParticleIndex = index + 1;
    }
    world->particles.particlePositions[index] = position;
    world->particles.particleVelocities[index] = velocity;
    world->particles.particleFlags[index] = flags;
    world->particles.particleLifetime[index] = 0.0f;
    world->particles.particleUserData[index] = 0;
    world->particles.particleAlive[index] = 1;
    world->particles.particleCount += 1;
    m2ParticleId id = {index + 1, worldId.index1, world->particles.particleGenerations[index]};
    if (world->recorder.journalActive != 0)
    {
        m2OpEmitParticle record;
        memset(&record, 0, sizeof(record));
        record.position = position;
        record.velocity = velocity;
        record.flags = flags;
        record.expected = id;
        m2JournalRecord(world, m2_opEmitParticle, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m2World_DestroyParticle(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2JournalRecord(world, m2_opDestroyParticle, &particleId, (int32_t)sizeof(particleId));
    }
    world->particles.particleAlive[index] = 0;
    world->particles.particleGenerations[index] += 1; // retire under a fresh generation
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
    world->particles.particleFreeQueue[(world->particles.particleFreeHead +
                                        world->particles.particleFreeCount) %
                                       world->particles.particleCapacity] = index;
    world->particles.particleFreeCount += 1;
    world->particles.particleCount -= 1;
}

bool m2Particle_IsValid(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    return ParticleSlot(world, particleId) >= 0;
}

m2Pos2 m2Particle_GetPosition(m2ParticleId particleId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particlePositions[index] : zero;
}

m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId)
{
    m2Vec2 zero = {0.0f, 0.0f};
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleVelocities[index] : zero;
}

void m2Particle_SetVelocity(m2ParticleId particleId, m2Vec2 velocity)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0 || !m2FiniteVec2(velocity))
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

uint32_t m2Particle_GetFlags(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleFlags[index] : 0;
}

void m2Particle_SetLifetime(m2ParticleId particleId, float seconds)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0 || !m2FiniteF(seconds) || seconds < 0.0f)
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

float m2Particle_GetLifetime(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleLifetime[index] : 0.0f;
}

void m2Particle_SetUserData(m2ParticleId particleId, uint64_t userData)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
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

uint64_t m2Particle_GetUserData(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particles.particleUserData[index] : 0;
}

int32_t m2World_GetParticleCount(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL ? world->particles.particleCount : 0;
}

int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
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
            ids[total] =
                (m2ParticleId){i + 1, worldId.index1, world->particles.particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}
