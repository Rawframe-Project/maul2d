// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The id pools' part of the world hash.

#include "world_hash.h"

// An id pool: where the next object lands and which id it gets.
typedef struct m2HashPool
{
    const int32_t* queue;
    const uint16_t* generations;
    int32_t capacity;
    int32_t head;
    int32_t count;
    int32_t maxIndex;
    int32_t retired;
    m2HashPart part;
} m2HashPool;

static uint64_t HashPool(const m2HashPool* pool, uint64_t h)
{
    h = m2Hash64(h, &pool->maxIndex, (int32_t)sizeof(int32_t));
    h = m2Hash64(h, &pool->count, (int32_t)sizeof(int32_t));
    h = m2Hash64(h, &pool->retired, (int32_t)sizeof(int32_t));
    for (int32_t k = 0; k < pool->count; ++k)
    {
        h = m2Hash64(h, &pool->queue[(pool->head + k) % pool->capacity], (int32_t)sizeof(int32_t));
    }
    // Generations below the high-water mark, live and free: a freed slot's
    // generation is the one its next object takes.
    return m2Hash64(h, pool->generations, pool->maxIndex * (int32_t)sizeof(uint16_t));
}

uint64_t m2HashPools(const m2World* world, m2HashPart part, uint64_t h)
{
    const m2Bodies* b = &world->bodies;
    const m2Shapes* s = &world->shapes;
    const m2Chains* c = &world->chains;
    const m2Joints* j = &world->joints;
    const m2Particles* p = &world->particles;
    const m2FluidVolumes* v = &world->volumes;
    const m2HashPool pools[] = {
        {b->freeQueue, b->generations, b->bodyCapacity, b->freeHead, b->freeCount, b->maxBodyIndex,
         b->retiredCount, m2_hashBodies},
        {s->shapeFreeQueue, s->shapeGenerations, s->shapeCapacity, s->shapeFreeHead,
         s->shapeFreeCount, s->maxShapeIndex, s->shapeRetiredCount, m2_hashBodies},
        {c->chainFreeQueue, c->chainGenerations, s->shapeCapacity, c->chainFreeHead,
         c->chainFreeCount, c->maxChainIndex, c->chainRetiredCount, m2_hashBodies},
        {j->jointFreeQueue, j->jointGenerations, j->jointCapacity, j->jointFreeHead,
         j->jointFreeCount, j->maxJointIndex, j->jointRetiredCount, m2_hashJoints},
        {p->particleFreeQueue, p->particleGenerations, p->particleCapacity, p->particleFreeHead,
         p->particleFreeCount, p->maxParticleIndex, 0, m2_hashParticles},
        {v->fvFreeQueue, v->fvGenerations, v->fvCapacity, v->fvFreeHead, v->fvFreeCount,
         v->maxFvIndex, 0, m2_hashVolumes},
    };
    for (size_t i = 0; i < sizeof(pools) / sizeof(pools[0]); ++i)
    {
        if (pools[i].capacity > 0 && (part == m2_hashAll || part == pools[i].part))
        {
            h = HashPool(&pools[i], h);
        }
    }
    return h;
}
