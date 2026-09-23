// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for body.c.

#ifndef MAUL2D_SRC_BODY_H
#define MAUL2D_SRC_BODY_H

#include "world_internal.h"

m2World* m2GetBodyWorld(m2BodyId id);
int32_t m2BodySlot(const m2World* world, m2BodyId id);
void m2RecomputeMass(m2World* world, int32_t bodyIndex);

// Wakes a dynamic body; static and kinematic bodies never sleep.
static inline void m2WakeIfDynamic(m2World* world, int32_t body)
{
    if (world->bodies.types[body] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[body] = 0;
        world->bodies.sleepTimes[body] = 0.0f;
    }
}

// A def is valid only when its internalValue matches its cookie.
#define M2_BODY_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)

// The journaled parameter channel (codes in journal.h).
bool m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value);

#endif // MAUL2D_SRC_BODY_H
