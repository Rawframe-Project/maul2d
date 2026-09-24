// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for world.c.

#ifndef MAUL2D_SRC_WORLD_H
#define MAUL2D_SRC_WORLD_H

#include "world_internal.h"

m2World* m2GetWorld(m2WorldId id);

// The world an object id names (its world field: slot and generation),
// or NULL when that world is gone.
m2World* m2WorldFromTag(uint16_t tag);

// A def is valid only when its internalValue matches its cookie.
#define M2_WORLD_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2WorldDef) << 8) ^ 1)

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2WorldFromId(m2WorldId worldId);

// The task dispatch: hooks when the host installed them,
// serial in the caller otherwise. Bit-blind to the split by the
// worker-count law.
void m2RunParallel(m2World* world, m2TaskFn* fn, void* ctx, int32_t itemCount, int32_t minRange);

#endif // MAUL2D_SRC_WORLD_H
