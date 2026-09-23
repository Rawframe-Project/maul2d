// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for world.c.

#ifndef MAUL2D_SRC_WORLD_H
#define MAUL2D_SRC_WORLD_H

#include "world_internal.h"

m2World* m2GetWorld(m2WorldId id);
m2World* m2WorldFromIndex(uint16_t world0);

#endif // MAUL2D_SRC_WORLD_H
