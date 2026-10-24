// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The structural check of an incoming snapshot block: how its arrays fit
// together, after m2StateValidate has checked each one alone. Snapshot
// restore and journal replay (through restore) both call it.

#ifndef MAUL2D_SRC_SNAPSHOT_CHECK_H
#define MAUL2D_SRC_SNAPSHOT_CHECK_H

#include "world_internal.h"

#include <stdbool.h>
#include <stdint.h>

// True when the block, already accepted by m2StateValidate, describes a
// world no later call or step can drive out of its pools: free queues,
// owner lists, the broadphase trees, the moved list and the pairs all
// agree. Reads the block in place and writes only world->restoreMarks.
bool m2SnapshotCheck(m2World* world, const uint8_t* in);

#endif // MAUL2D_SRC_SNAPSHOT_CHECK_H
