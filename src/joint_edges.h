// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint edges of each body: the lists that tell which joints touch a
// body and which jointed pairs may not collide.

#ifndef MAUL2D_SRC_JOINT_EDGES_H
#define MAUL2D_SRC_JOINT_EDGES_H

#include "world_internal.h"

// Adds a joint to both bodies' edge lists, and removes it.
void m2LinkJoint(m2World* world, int32_t joint);
void m2UnlinkJoint(m2World* world, int32_t joint);

// Rebuilds every edge list from the joint table, after a restore.
void m2RebuildJointEdges(m2World* world);

// Whether a joint between the two bodies forbids their collision.
bool m2JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB);

#endif // MAUL2D_SRC_JOINT_EDGES_H
