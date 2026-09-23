// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for contact_solver.c.

#ifndef MAUL2D_SRC_CONTACT_SOLVER_H
#define MAUL2D_SRC_CONTACT_SOLVER_H

#include "solver.h"

int32_t m2PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h);
void m2WarmStartOne(m2World* world, m2ContactConstraint* c);
void m2SolveContactOne(m2World* world, m2ContactConstraint* c, float invH, float minBiasVel,
                       bool useBias);
void m2RestitutionOne(m2World* world, m2ContactConstraint* c);
void m2ContactStageRange(int32_t begin, int32_t end, void* userCtx);

#endif // MAUL2D_SRC_CONTACT_SOLVER_H
