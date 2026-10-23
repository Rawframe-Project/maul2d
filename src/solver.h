// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The step entry point: the soft-step solve.

#ifndef MAUL2D_SRC_SOLVER_H
#define MAUL2D_SRC_SOLVER_H

#include "world_internal.h"

// The soft-step solve for one step.
void m2SolveStep(m2World* world, float dt, int32_t substepCount);

#endif // MAUL2D_SRC_SOLVER_H
