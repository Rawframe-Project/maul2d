// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The process-wide table of live worlds behind m2WorldId, and the world
// tag every object id carries.

#include "world_registry.h"

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
        m2SetLastResult(m2_errorInvalid); // an id of a world that is gone
        return NULL;
    }
    return world;
}

int32_t m2FreeWorldSlot(void)
{
    for (int32_t i = 0; i < M2_MAX_WORLDS; ++i)
    {
        if (s_worlds[i] == NULL)
        {
            return i;
        }
    }
    return -1;
}

m2WorldId m2PublishWorld(m2World* world, int32_t slot)
{
    s_worldGenerations[slot] += 1;
    world->worldGeneration = s_worldGenerations[slot];
    world->slot = (uint16_t)slot;
    world->idWorld =
        (uint16_t)(((uint32_t)world->worldGeneration << M2_WORLD_SLOT_BITS) | (uint32_t)slot);
    s_worlds[slot] = world;
    return (m2WorldId){(uint16_t)(slot + 1), world->worldGeneration};
}

void m2RetireWorld(const m2World* world)
{
    s_worlds[world->slot] = NULL;
}
