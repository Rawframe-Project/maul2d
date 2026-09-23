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

// Lane width is a layout constant, never a semantics knob: every lane
// runs the same scalar IEEE sequence, so lane packing cannot move a bit.
#define M2_LANES 8

// Bytes of the wide SoA scratch for a pair capacity.
int32_t m2ContactBlockScratchBytes(int32_t pairCapacity);

#endif // MAUL2D_SRC_CONTACT_SOLVER_WIDE_H
