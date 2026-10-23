// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Islands and sleep: union-find over touching contacts and joints.

#ifndef MAUL2D_SRC_ISLAND_H
#define MAUL2D_SRC_ISLAND_H

#include "world_internal.h"

// Wakes whole islands touched by an awake body, then puts islands to
// sleep once every body in them has rested long enough.
void m2UpdateIslandsAndWake(m2World* world);
void m2UpdateSleep(m2World* world, float dt);

// Wakes a dynamic body; static and kinematic bodies never sleep.
static inline void m2WakeIfDynamic(m2World* world, int32_t body)
{
    if (world->bodies.types[body] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[body] = 0;
        world->bodies.sleepTimes[body] = 0.0f;
    }
}

#endif // MAUL2D_SRC_ISLAND_H
