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

#endif // MAUL2D_SRC_ISLAND_H
