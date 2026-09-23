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
    if (world == NULL || index < 0 || index >= world->particleCapacity ||
        world->particleAlive[index] == 0 || world->particleGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ParticleId m2World_EmitParticle(m2WorldId worldId, m2Pos2 position, m2Vec2 velocity,
                                  uint32_t flags)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || world->particleCapacity == 0)
    {
        m2Refuse(world, m2_errorInvalid); // no particle system in this world: misuse
        return m2_nullParticleId;
    }
    if (!m2FinitePos2(position) || !m2FiniteVec2(velocity))
    {
        m2Refuse(world, m2_errorInvalid); // NaN screen, the def-validation law
        return m2_nullParticleId;
    }
    if (world->particleFreeCount == 0)
    {
        // A full pool is a runtime fact, not misuse: pace emitters
        // off m2World_GetParticleCount; the counter keeps the score.
        world->particlePoolFullCount += 1;
        m2Refuse(world, m2_errorCapacity);
        return m2_nullParticleId;
    }
    int32_t index = world->particleFreeQueue[world->particleFreeHead];
    world->particleFreeHead = (world->particleFreeHead + 1) % world->particleCapacity;
    world->particleFreeCount -= 1;
    if (index + 1 > world->maxParticleIndex)
    {
        world->maxParticleIndex = index + 1;
    }
    world->particlePositions[index] = position;
    world->particleVelocities[index] = velocity;
    world->particleFlags[index] = flags;
    world->particleLifetime[index] = 0.0f;
    world->particleUserData[index] = 0;
    world->particleAlive[index] = 1;
    world->particleCount += 1;
    m2ParticleId id = {index + 1, worldId.index1, world->particleGenerations[index]};
    if (world->journalActive != 0)
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
    if (world->journalActive != 0)
    {
        m2JournalRecord(world, m2_opDestroyParticle, &particleId, (int32_t)sizeof(particleId));
    }
    world->particleAlive[index] = 0;
    world->particleGenerations[index] += 1; // retire under a fresh generation
    // Jelly bookkeeping: springs and triads die with their particle,
    // compacted in order so the lists stay canonical.
    if (world->particleSpringCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particleSpringCount; ++k)
        {
            if (world->particleSpringA[k] == index || world->particleSpringB[k] == index)
            {
                continue;
            }
            world->particleSpringA[keep] = world->particleSpringA[k];
            world->particleSpringB[keep] = world->particleSpringB[k];
            world->particleSpringRest[keep] = world->particleSpringRest[k];
            keep += 1;
        }
        world->particleSpringCount = keep;
    }
    if (world->particleTriadCount > 0)
    {
        int32_t keep = 0;
        for (int32_t k = 0; k < world->particleTriadCount; ++k)
        {
            if (world->particleTriadA[k] == index || world->particleTriadB[k] == index ||
                world->particleTriadC[k] == index)
            {
                continue;
            }
            world->particleTriadA[keep] = world->particleTriadA[k];
            world->particleTriadB[keep] = world->particleTriadB[k];
            world->particleTriadC[keep] = world->particleTriadC[k];
            world->particleTriadPA[keep] = world->particleTriadPA[k];
            world->particleTriadPB[keep] = world->particleTriadPB[k];
            world->particleTriadPC[keep] = world->particleTriadPC[k];
            keep += 1;
        }
        world->particleTriadCount = keep;
    }
    world->particleFreeQueue[(world->particleFreeHead + world->particleFreeCount) %
                             world->particleCapacity] = index;
    world->particleFreeCount += 1;
    world->particleCount -= 1;
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
    return index >= 0 ? world->particlePositions[index] : zero;
}

m2Vec2 m2Particle_GetVelocity(m2ParticleId particleId)
{
    m2Vec2 zero = {0.0f, 0.0f};
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleVelocities[index] : zero;
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
    if (world->journalActive != 0)
    {
        m2OpParticleVec record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetParticleVelocity, &record, (int32_t)sizeof(record));
    }
    world->particleVelocities[index] = velocity;
}

uint32_t m2Particle_GetFlags(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleFlags[index] : 0;
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
    if (world->journalActive != 0)
    {
        m2OpParticleFloat record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.value = seconds;
        m2JournalRecord(world, m2_opSetParticleLifetime, &record, (int32_t)sizeof(record));
    }
    world->particleLifetime[index] = seconds;
}

float m2Particle_GetLifetime(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleLifetime[index] : 0.0f;
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
    if (world->journalActive != 0)
    {
        m2OpParticleUserData record;
        memset(&record, 0, sizeof(record));
        record.id = particleId;
        record.userData = userData;
        m2JournalRecord(world, m2_opSetParticleUserData, &record, (int32_t)sizeof(record));
    }
    world->particleUserData[index] = userData;
}

uint64_t m2Particle_GetUserData(m2ParticleId particleId)
{
    m2World* world = m2WorldFromIndex(particleId.world0);
    int32_t index = ParticleSlot(world, particleId);
    return index >= 0 ? world->particleUserData[index] : 0;
}

int32_t m2World_GetParticleCount(m2WorldId worldId)
{
    m2World* world = m2GetWorld(worldId);
    return world != NULL ? world->particleCount : 0;
}

int32_t m2World_GetParticles(m2WorldId worldId, m2ParticleId* ids, int32_t capacity)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxParticleIndex; ++i)
    {
        if (world->particleAlive[i] == 0)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = (m2ParticleId){i + 1, worldId.index1, world->particleGenerations[i]};
        }
        total += 1;
    }
    return total;
}
