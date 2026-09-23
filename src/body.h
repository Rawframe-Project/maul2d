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

#endif // MAUL2D_SRC_BODY_H
