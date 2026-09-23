// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for world.c.

#ifndef MAUL2D_SRC_WORLD_H
#define MAUL2D_SRC_WORLD_H

#include "world_internal.h"

m2World* m2GetWorld(m2WorldId id);
m2World* m2WorldFromIndex(uint16_t world0);

// A def is valid only when its internalValue matches its cookie.
#define M2_WORLD_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2WorldFromId(m2WorldId worldId);
m2World* m2WorldFromIndex0(uint16_t index0);

// The task dispatch: hooks when the host installed them,
// serial in the caller otherwise. Bit-blind to the split by the
// worker-count law.
void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange);

#endif // MAUL2D_SRC_WORLD_H
