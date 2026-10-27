// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for snapshot.c.

#ifndef MAUL2D_SRC_SNAPSHOT_H
#define MAUL2D_SRC_SNAPSHOT_H

#include "world_internal.h"

#include <stdbool.h>

// Whether a world of these capacities keeps its snapshot within
// M2_MAX_SNAPSHOT_BYTES. It reads the capacities alone, so m2CreateWorld
// asks before it allocates.
bool m2SnapshotFits(const m2World* world);

#endif // MAUL2D_SRC_SNAPSHOT_H
