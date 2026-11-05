// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pair part of an incoming snapshot's structural check.

#ifndef MAUL2D_SRC_SNAPSHOT_PAIRS_H
#define MAUL2D_SRC_SNAPSHOT_PAIRS_H

#include "world_internal.h"

#include <stdbool.h>
#include <stdint.h>

// True when the block's moved list matches its flags, its pairs are in
// order, join live shapes in the broadphase and, where the next step keeps
// them, are pairs the broadphase would make, its touching flags and frozen
// pairs' manifolds are those its shapes give, and its queued end events
// carry the step count and name shape slots. The trees and shape lists
// have passed m2SnapshotCheck first. Writes only world->restoreMarks and
// world->contacts.pairScratch.
M2_NODISCARD bool m2SnapshotCheckPairs(m2World* world, const uint8_t* in);

#endif // MAUL2D_SRC_SNAPSHOT_PAIRS_H
