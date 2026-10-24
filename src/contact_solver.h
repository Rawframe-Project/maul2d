// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact constraints: building them from the manifolds, grouping them
// into lane blocks by graph color, and running the solver stages.

#ifndef MAUL2D_SRC_CONTACT_SOLVER_H
#define MAUL2D_SRC_CONTACT_SOLVER_H

#include "contact_constraint.h"
#include "solver_base.h"
#include "world_internal.h"

#include <stddef.h>

// A step's contact work: the constraints, their colors and blocks.
typedef struct m2ContactPlan
{
    m2ContactConstraint* constraints;
    int32_t count;
    int32_t colorStart[M2_GRAPH_COLORS + 2]; // the last range is the overflow
    int32_t blockStart[M2_GRAPH_COLORS + 1];
    float invH;
    bool reversed; // points in reverse order this substep
} m2ContactPlan;

int32_t m2PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h);

// Colors the prepared constraints and packs the colored ones into blocks.
void m2PlanContacts(m2World* world, m2ContactPlan* plan);

// Colors run block-parallel in color order; the overflow runs serially.
void m2RunContactStage(m2World* world, const m2ContactPlan* plan, m2ContactStage stage);

// sizeof(m2ContactConstraint): the scratch block is sized by its owner.
int32_t m2ContactConstraintSize(void);

// Bytes of the block scratch for a pair capacity.
size_t m2ContactBlockScratchBytes(int32_t pairCapacity);

#endif // MAUL2D_SRC_CONTACT_SOLVER_H
