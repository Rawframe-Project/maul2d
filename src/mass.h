// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mass properties of a body, summed from its shapes.

#ifndef MAUL2D_SRC_MASS_H
#define MAUL2D_SRC_MASS_H

#include "world_internal.h"

// Sets a body's inverse mass, inverse inertia and center of mass from its
// shapes, in shape-list order; static and kinematic bodies get zero.
void m2RecomputeMass(m2World* world, int32_t bodyIndex);

#endif // MAUL2D_SRC_MASS_H
