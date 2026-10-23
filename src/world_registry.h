// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The table of live worlds: m2WorldId lookups, the world tag in object
// ids, and the slots m2CreateWorld and m2DestroyWorld claim and free.

#ifndef MAUL2D_SRC_WORLD_REGISTRY_H
#define MAUL2D_SRC_WORLD_REGISTRY_H

#include "world_internal.h"

m2World* m2GetWorld(m2WorldId id);

// The world an object id names (its world field: slot and generation),
// or NULL when that world is gone.
m2World* m2WorldFromTag(uint16_t tag);

// White-box accessor for tests and internal modules. Returns NULL for a
// stale or null id. Not part of the public ABI.
m2World* m2WorldFromId(m2WorldId worldId);

// A free slot of the table, or -1 when every slot holds a world.
int32_t m2FreeWorldSlot(void);

// Puts a fully built world into a free slot, gives it the next
// generation of that slot and returns its id.
m2WorldId m2PublishWorld(m2World* world, int32_t slot);

// Frees the slot of a world that is being destroyed.
void m2RetireWorld(const m2World* world);

#endif // MAUL2D_SRC_WORLD_REGISTRY_H
