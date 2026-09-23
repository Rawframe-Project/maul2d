// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's shape: one kind table entry per joint type, the
// frame every prepare receives, the velocities every solve receives,
// and the helpers the kinds share. Each kind lives in its own
// joint_<kind>.c.

#ifndef MAUL2D_SRC_JOINT_SOLVER_H
#define MAUL2D_SRC_JOINT_SOLVER_H

#include "joint.h"
#include "solver.h"
#include "world_internal.h"

// What a kind's prepare starts from: the common setup is already in the
// constraint (bodies, flags, softness, warm impulses, world arms rA and
// rB); the frame adds the pose, the anchor separation and the masses.
typedef struct m2JointFrame
{
    int32_t joint;
    float h;
    m2Rot qA;
    m2Rot qB;
    m2Vec2 lcA; // local centers of mass
    m2Vec2 lcB;
    float dx; // anchor separation, B minus A
    float dy;
    float mA;
    float iA;
    float mB;
    float iB;
} m2JointFrame;

// What a kind's solve starts from: both bodies' velocities as the
// substep found them and the anchor drift since prepare.
typedef struct m2JointSolveContext
{
    m2Vec2 vA;
    float wA;
    m2Vec2 vB;
    float wB;
    m2Vec2 ds;  // anchor separation drift since prepare
    m2Vec2 drA; // arms rotated by the substep's delta rotations
    m2Vec2 drB;
    bool useBias;
    float invH;
} m2JointSolveContext;

// A kind's functions; a kind with no rows (the filter joint) leaves them
// NULL and is skipped.
typedef struct m2JointKind
{
    void (*prepare)(m2World* world, m2JointConstraint* c, const m2JointFrame* f);
    void (*warmStart)(m2World* world, const m2JointConstraint* c);
    void (*solve)(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx);
    // Last step's load as a force and a torque magnitude.
    void (*reaction)(const m2World* world, int32_t joint, float invH, float* force, float* torque);
} m2JointKind;

extern const m2JointKind m2_distanceJointKind;
extern const m2JointKind m2_revoluteJointKind;
extern const m2JointKind m2_prismaticJointKind;
extern const m2JointKind m2_weldJointKind;
extern const m2JointKind m2_wheelJointKind;
extern const m2JointKind m2_filterJointKind;
extern const m2JointKind m2_motorJointKind;
extern const m2JointKind m2_mouseJointKind;
extern const m2JointKind m2_gearJointKind;
extern const m2JointKind m2_pulleyJointKind;
extern const m2JointKind m2_ratchetJointKind;

// The step's joint stages, in the order the solver runs them.
int32_t m2PrepareJoints(m2World* world, m2JointConstraint* joints, float h);
void m2WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count);
void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool useBias,
                   float invH);
void m2StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count);

// Shared by the kinds.
void m2ApplyJointImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P);
void m2ApplyJointArmImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P, float LA,
                            float LB);
float m2SolveJointAxial(m2JointConstraint* c, float cdot, float bias, float massScale,
                        float impulseScale, float* accumulated, bool oneSided);
void m2PreparePointBlock(m2JointConstraint* c, const m2JointFrame* f);
void m2WarmStartPointJoint(m2World* world, const m2JointConstraint* c);
void m2SolvePointBlock(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx,
                       float wA, float wB, bool biased);

#endif // MAUL2D_SRC_JOINT_SOLVER_H
