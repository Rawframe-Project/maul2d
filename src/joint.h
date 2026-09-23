// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations shared by joint.c (slots, links, the parameter
// channel) and the per-kind joint_<kind>.c files.

#ifndef MAUL2D_SRC_JOINT_H
#define MAUL2D_SRC_JOINT_H

#include "world_internal.h"

// Joint flags. The first three mirror public switches and live in
// world->jointFlags; the rest are set by prepare for this step only and
// mean different things to different kinds.
#define M2_JOINT_MOTOR  1u // motor enabled
#define M2_JOINT_LIMIT  2u // limit (or wheel travel stops) enabled
#define M2_JOINT_SPRING 4u // wheel suspension spring enabled

#define M2_JOINT_HARD_RANGE   8u  // distance: a finite length range is active
#define M2_JOINT_ROPE         16u // distance: spring enabled (stored with the joint)
#define M2_JOINT_FREE_LENGTH  32u // distance: zero-stiffness spring, skip the rest row
#define M2_JOINT_SPRING_DRIVE 32u // motor: soft spring drive replaces the hard bias

void m2UnlinkJoint(m2World* world, int32_t joint);
void m2RebuildJointEdges(m2World* world);
bool m2JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB);
float m2RelativeJointAngle(m2Rot qA, m2Rot qB);

// Def checks shared by the kinds: a finite value, and a finite value that
// is not negative (stiffness, damping, budgets).
static inline bool m2JointGain(float x)
{
    return m2FiniteF(x) && x >= 0.0f;
}

// Slot lookups: the live slot of a joint id, or -1. The typed lookup
// also demands a joint type and refuses (world may be NULL) on a miss.
int32_t m2JointSlotChecked(const m2World* world, m2JointId jointId);
int32_t m2TypedJointSlot(m2World* world, m2JointId jointId, uint8_t type);

// Creation: a free slot (or -1), then the fields every joint starts
// with, linked into both bodies' lists, both bodies awake.
int32_t m2AllocateJoint(m2World* world);
m2JointId m2FinishJoint(m2World* world, m2WorldId worldId, int32_t index, uint8_t type,
                        int32_t bodyA, int32_t bodyB, m2Vec2 anchorA, m2Vec2 anchorB, float length,
                        float hertz, float damping);

// The rope length on one side of a pulley (0 = A, 1 = B) right now.
float m2PulleyLiveLength(m2World* world, int32_t index, int32_t side);

#endif // MAUL2D_SRC_JOINT_H
