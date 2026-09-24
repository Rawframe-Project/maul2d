// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver: the per-step constraint every joint gets, the pose
// its rows are built from, and the kind table (one joint_<kind>.c per
// type).

#ifndef MAUL2D_SRC_JOINT_SOLVER_H
#define MAUL2D_SRC_JOINT_SOLVER_H

#include "joint.h"
#include "joint_row.h"
#include "solver.h"
#include "world_internal.h"

// A joint as the solver sees it for one step. Accumulated impulses live
// in the five stored slots; what each slot means is the kind's, and its
// reaction function reads them back.
typedef struct m2JointConstraint
{
    int32_t jointIndex;
    int32_t bodyA;
    int32_t bodyB;
    uint8_t type;  // m2JointType, the index into the kind table
    uint8_t flags; // M2_JOINT_* (joint.h)
    m2Vec2 armA;   // anchors from the centers of mass, world-rotated at prepare
    m2Vec2 armB;
    m2Vec2 gap;   // anchor B minus anchor A at prepare
    float angle;  // relative angle past the joint's zero at prepare
    m2Vec2 axis;  // prismatic, wheel: the slide axis at prepare
    m2Vec2 ropeA; // pulley: each anchor minus its ground point at prepare
    m2Vec2 ropeB;
    float ratio;  // gear, pulley; ratchet: the signed tooth pitch
    float length; // distance: rest length; pulley: rope total
    float lower;  // limits (distance: lengths)
    float upper;
    float motorSpeed;
    float maxMotorImpulse; // h times the motor's force or torque
    float maxPullImpulse;  // motor, mouse: h times the linear force budget
    float correction;      // motor: the fraction of error removed per step
    m2Softness soft;       // the constraint rows
    m2Softness spring;     // the kind's spring (see each kind)
    bool linearSpring;     // a real spring: pulls in the relax pass too
    bool angularSpring;
    // The point pair's mass inverse, taken once per step at the prepare
    // arms: within a step the arms turn by one step's rotation at most.
    m2PointMass pointMass;
    m2Vec2 impulse;
    float motorImpulse;
    float lowerImpulse;
    float upperImpulse;
    float springImpulse;
} m2JointConstraint;

// What a kind's prepare starts from; the common part of the constraint
// is already filled.
typedef struct m2JointFrame
{
    int32_t joint;
    float h;
    m2Rot qA;
    m2Rot qB;
} m2JointFrame;

// The joint where the substep has taken it: the arms turned by each
// body's rotation since the step began.
typedef struct m2JointPose
{
    m2Vec2 armA;
    m2Vec2 armB;
    m2Vec2 moveA; // how far each center of mass moved since the step began
    m2Vec2 moveB;
    m2Rot turnA; // each body's rotation since the step began
    m2Rot turnB;
} m2JointPose;

// One solve pass: biased while solving, not while relaxing.
typedef struct m2JointPass
{
    bool biased;
    float invH;
} m2JointPass;

// A kind's functions; a kind with no rows (the filter joint) leaves them
// NULL and is skipped.
typedef struct m2JointKind
{
    void (*prepare)(m2World* world, m2JointConstraint* c, const m2JointFrame* f);
    void (*warmStart)(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b);
    void (*solve)(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                  const m2JointPass* pass);
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
void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool biased,
                   float invH);
void m2StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count);

// Anchor B minus anchor A now.
m2Vec2 m2PoseGap(const m2JointConstraint* c, const m2JointPose* pose);

// A row along or across a slide axis fixed in A, given the gap now.
m2JointRow m2AxisRow(const m2JointPose* pose, m2Vec2 gap, m2Vec2 axis);

// Both limits of a row whose value is held in [lower, upper]. The upper
// limit runs the row backward, so both impulses are never negative.
void m2SolveLimits(m2JointConstraint* c, const m2JointRow* row, float value, m2Softness soft,
                   m2JointBodies* b, const m2JointPass* pass);
void m2WarmStartLimits(const m2JointConstraint* c, const m2JointRow* row, m2JointBodies* b);

// The joint angle now: the prepare angle plus the relative turn since.
float m2PoseAngle(const m2JointConstraint* c, const m2JointPose* pose);

// The stiff softness a joint row gets when its user gives none.
m2Softness m2StiffJointSoftness(float h);

// Reaction magnitudes from the stored impulses: the one mapping shared
// by the break pass and the public getters, so the number a game reads
// is bit for bit the number the break test compares.
void m2JointReactionMagnitudes(const m2World* world, int32_t j, float invH, float* force,
                               float* torque);

#endif // MAUL2D_SRC_JOINT_SOLVER_H
