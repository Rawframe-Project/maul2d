// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations for joint.c.

#ifndef MAUL2D_SRC_JOINT_H
#define MAUL2D_SRC_JOINT_H

#include "world_internal.h"

void m2UnlinkJoint(m2World* world, int32_t joint);
void m2RebuildJointEdges(m2World* world);
bool m2JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB);
float m2RelativeJointAngle(m2Rot qA, m2Rot qB);

#endif // MAUL2D_SRC_JOINT_H
