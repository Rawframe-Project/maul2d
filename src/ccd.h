// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bullet continuous collision.

#ifndef MAUL2D_SRC_CCD_H
#define MAUL2D_SRC_CCD_H

#include "world_internal.h"

// Sweeps bullets against non-bullets after each substep.
void m2SolveContinuous(m2World* world);

#endif // MAUL2D_SRC_CCD_H
