// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint: a slider with a suspension spring and free
// rotation, driven by a rotational motor.

#include "joint_solver.h"

#include "body.h"
#include "broadphase.h"
#include "joint.h"
#include "journal.h"
#include "solver.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

m2WheelJointDef m2DefaultWheelJointDef(void)
{
    m2WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){0.0f, 1.0f};
    def.enableSpring = true;
    def.hertz = 2.0f;
    def.dampingRatio = 0.7f;
    def.internalValue = M2_WHJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2WheelJointDef* def)
{
    return m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2FiniteVec2(def->localAxisA) && m2JointGain(def->hertz) &&
           m2JointGain(def->dampingRatio) && m2FiniteF(def->motorSpeed) &&
           m2JointGain(def->maxMotorTorque) && m2FiniteF(def->lowerTranslation) &&
           m2FiniteF(def->upperTranslation) && def->lowerTranslation <= def->upperTranslation;
}

m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WHJOINT_COOKIE || !DefValid(def))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0 || bodyA == bodyB)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullJointId;
    }
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2JointId jointId =
        m2FinishJoint(world, worldId, index, (uint8_t)m2_wheelJoint, bodyA, bodyB,
                      def->localAnchorA, def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->joints.jointFlags[index] = (def->enableMotor ? M2_JOINT_MOTOR : 0u) |
                                      (def->enableLimit ? M2_JOINT_LIMIT : 0u) |
                                      (def->enableSpring ? M2_JOINT_SPRING : 0u);
    world->joints.jointMotorSpeed[index] = def->motorSpeed;
    world->joints.jointMaxMotor[index] = def->maxMotorTorque;
    world->joints.jointLower[index] = def->lowerTranslation;
    world->joints.jointUpper[index] = def->upperTranslation;
    world->joints.jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateWheelJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWheelJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------
//
// A slider with free rotation. The spin motor (motorImpulse) is an angle
// row; along the axis run the suspension spring (impulse.y) and the
// travel stops; the perpendicular row (impulse.x) keeps the wheel on the
// axis. The user's stiffness shapes the suspension; the other rows stay
// on the stiff softness.

static void PrepareWheel(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    c->spring = c->soft;
    c->soft = m2StiffJointSoftness(f->h);
    c->axis = m2RotateVec2(f->qA, world->joints.jointLocalAxisA[f->joint]);
}

static void WheelRows(const m2JointConstraint* c, const m2JointPose* pose, m2JointRow* along,
                      m2JointRow* across, float* travel, float* offAxis)
{
    m2Vec2 axis = m2RotateVec2(pose->turnA, c->axis);
    m2Vec2 perp = {-axis.y, axis.x};
    m2Vec2 gap = m2PoseGap(c, pose);
    *along = m2AxisRow(pose, gap, axis);
    *across = m2AxisRow(pose, gap, perp);
    *travel = gap.x * axis.x + gap.y * axis.y;
    *offAxis = gap.x * perp.x + gap.y * perp.y;
}

static void WarmStartWheel(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    m2JointRow along;
    m2JointRow across;
    float travel;
    float offAxis;
    WheelRows(c, pose, &along, &across, &travel, &offAxis);
    m2JointRow turn = m2TurnRow();
    m2PushRow(&turn, b, c->motorImpulse);
    m2PushRow(&along, b, c->impulse.y);
    m2WarmStartLimits(c, &along, b);
    m2PushRow(&across, b, c->impulse.x);
}

static void SolveWheel(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                       const m2JointPass* pass)
{
    m2JointRow along;
    m2JointRow across;
    float travel;
    float offAxis;
    WheelRows(c, pose, &along, &across, &travel, &offAxis);
    if ((c->flags & M2_JOINT_MOTOR) != 0)
    {
        m2JointRow turn = m2TurnRow();
        m2SolveRow(&turn, b, m2RigidDrive(-c->motorSpeed), &c->motorImpulse, -c->maxMotorImpulse,
                   c->maxMotorImpulse);
    }
    if ((c->flags & M2_JOINT_SPRING) != 0)
    {
        m2SolveRow(&along, b, m2SpringDrive(c->spring, travel), &c->impulse.y, -M2_ROW_FREE,
                   M2_ROW_FREE);
    }
    if ((c->flags & M2_JOINT_LIMIT) != 0)
    {
        m2SolveLimits(c, &along, travel, c->soft, b, pass);
    }
    m2SolveRow(&across, b, m2HeldDrive(c->soft, offAxis, pass->biased), &c->impulse.x, -M2_ROW_FREE,
               M2_ROW_FREE);
}

static void WheelReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The perpendicular row, the spring and the stops as force; the motor
    // as torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    float axial =
        impulse.y + world->joints.jointLowerImpulse[j] - world->joints.jointUpperImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + axial * axial) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_wheelJointKind = {PrepareWheel, WarmStartWheel, SolveWheel, WheelReaction};
