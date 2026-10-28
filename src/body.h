// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for body.c.

#ifndef MAUL2D_SRC_BODY_H
#define MAUL2D_SRC_BODY_H

#include "world_internal.h"

// A def is valid only when its internalValue matches its cookie.
#define M2_BODY_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2BodyDef) << 8) ^ 2)

// The journaled parameter channel (codes in journal.h).
bool m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value);

// Holds a body's velocity within M2_SPEED_LIMIT after an impulse, a blast
// or a shatter, as the solver does after each of its pushes.
static inline void m2SaturateBodyVelocity(m2World* world, int32_t index)
{
    m2Vec2* v = &world->bodies.linearVelocities[index];
    v->x = m2SaturateF(v->x, M2_SPEED_LIMIT);
    v->y = m2SaturateF(v->y, M2_SPEED_LIMIT);
    world->bodies.angularVelocities[index] =
        m2SaturateF(world->bodies.angularVelocities[index], M2_SPEED_LIMIT);
}

#endif // MAUL2D_SRC_BODY_H
