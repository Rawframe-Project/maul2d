// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for joint_solver.c.

#ifndef MAUL2D_SRC_JOINT_SOLVER_H
#define MAUL2D_SRC_JOINT_SOLVER_H

#include "solver.h"

int32_t m2PrepareJoints(m2World* world, m2JointConstraint* joints, float h);
void m2WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count);
void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool useBias,
                   float invH);
void m2StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count);

#endif // MAUL2D_SRC_JOINT_SOLVER_H
