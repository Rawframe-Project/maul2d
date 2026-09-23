// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for contact_solver_wide.c.

#ifndef MAUL2D_SRC_CONTACT_SOLVER_WIDE_H
#define MAUL2D_SRC_CONTACT_SOLVER_WIDE_H

#include "solver.h"

int32_t m2PackContactBlocks(m2World* world, m2ContactConstraint* constraints,
                            const int32_t* colorStart, int32_t* blockStart);
void m2RunContactStageWide(m2World* world, m2ContactConstraint* constraints,
                           const int32_t* colorStart, const int32_t* blockStart,
                           m2ContactStage stage, float invH, float minBiasVel, bool useBias);

#endif // MAUL2D_SRC_CONTACT_SOLVER_WIDE_H
