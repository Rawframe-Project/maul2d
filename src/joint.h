// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Internal declarations shared by joint.c (slots, links, the parameter
// channel) and the per-kind joint_<kind>.c files.

#ifndef MAUL2D_SRC_JOINT_H
#define MAUL2D_SRC_JOINT_H

#include "world_internal.h"

// Joint flags. The first three mirror public switches and live in
// world->joints.jointFlags; the rest are set by prepare for this step only and
// mean different things to different kinds.
#define M2_JOINT_MOTOR  1u // motor enabled
#define M2_JOINT_LIMIT  2u // limit (or wheel travel stops) enabled
#define M2_JOINT_SPRING 4u // wheel suspension or revolute angular spring enabled

#define M2_JOINT_HARD_RANGE  8u  // distance: a finite length range is active
#define M2_JOINT_ROPE        16u // distance: spring enabled (stored with the joint)
#define M2_JOINT_FREE_LENGTH 32u // distance: zero-stiffness spring, skip the rest row

// Slot lookups: the live slot of a joint id, or -1. The typed lookup
// also demands a joint type and refuses (world may be NULL) on a miss.
int32_t m2JointSlotChecked(const m2World* world, m2JointId jointId);
int32_t m2TypedJointSlot(const m2World* world, m2JointId jointId, uint8_t type);

// The joint kinds with a motor and limits (bit n = m2JointType n). The
// motor joint's torque budget rides the max motor channel too. Other
// kinds may keep their own state where these kinds keep limits (the
// ratchet's tooth), so the channels refuse them.
#define M2_MOTOR_LIMIT_KINDS                                                                       \
    ((1u << m2_revoluteJoint) | (1u << m2_prismaticJoint) | (1u << m2_wheelJoint))
#define M2_MAX_MOTOR_KINDS (M2_MOTOR_LIMIT_KINDS | (1u << m2_motorJoint))

// The joint kinds with a spring toggle: a distance joint's spring, rope or
// strut, a wheel's suspension and a revolute joint's angular spring.
#define M2_SPRING_KINDS                                                                            \
    ((1u << m2_distanceJoint) | (1u << m2_wheelJoint) | (1u << m2_revoluteJoint))

// A joint keeps two stiffness pairs: the first (jointHertz, jointDamping)
// and the second (jointHertz2, jointDamping2). Each public tuning names
// the kinds it is valid for and the pair it reads and writes there:
// - constraint: the softness of the rows that hold a revolute, prismatic,
//   gear, pulley or ratchet joint together, in the first pair;
// - spring: a distance, wheel or mouse joint's spring in the first pair,
//   a revolute's angular spring or a motor joint's drive in the second;
// - weld linear and weld angular: a weld's two springs, first and second.
typedef enum m2JointTuning
{
    m2_tuneConstraint,
    m2_tuneSpring,
    m2_tuneWeldLinear,
    m2_tuneWeldAngular,
} m2JointTuning;

// The live slot of a joint a tuning is valid for, and in secondOut
// whether the tuning uses its second pair; -1 and one misuse counted for
// a stale id or another kind.
int32_t m2JointTuningSlot(const m2World* world, m2JointId jointId, m2JointTuning tuning,
                          bool* secondOut);

// A distance joint's maximum length when it has none.
#define M2_OPEN_LENGTH 3.4e38f

// Creation: a free slot (or -1), then the fields every joint starts
// with, linked into both bodies' lists, both bodies awake.
int32_t m2AllocateJoint(m2World* world);
m2JointId m2FinishJoint(m2World* world, int32_t index, uint8_t type, int32_t bodyA, int32_t bodyB,
                        m2Vec2 anchorA, m2Vec2 anchorB, float length, float hertz, float damping);

// The rope length on one side of a pulley (0 = A, 1 = B) right now.
float m2PulleyLiveLength(const m2World* world, int32_t index, int32_t side);

// A def is valid only when its internalValue matches its cookie.
#define M2_DJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2DistanceJointDef) << 8) ^ 4)
#define M2_RJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2RevoluteJointDef) << 8) ^ 5)
#define M2_PJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2PrismaticJointDef) << 8) ^ 6)
#define M2_WJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2WeldJointDef) << 8) ^ 7)
#define M2_WHJOINT_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2WheelJointDef) << 8) ^ 8)
#define M2_FJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2FilterJointDef) << 8) ^ 10)
#define M2_MOJOINT_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2MotorJointDef) << 8) ^ 11)
#define M2_MSJOINT_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2MouseJointDef) << 8) ^ 12)
#define M2_GJOINT_COOKIE  (M2_COOKIE ^ ((int32_t)sizeof(m2GearJointDef) << 8) ^ 14)
#define M2_PLJOINT_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2PulleyJointDef) << 8) ^ 15)
#define M2_RTJOINT_COOKIE (M2_COOKIE ^ ((int32_t)sizeof(m2RatchetJointDef) << 8) ^ 16)

// The journaled parameter channel (codes in journal.h) and the destroy
// shared with the solver's break pass.
bool m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value);
void m2DestroyJointInternal(m2World* world, int32_t index);

#endif // MAUL2D_SRC_JOINT_H
