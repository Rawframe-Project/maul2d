// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The prismatic joint: a slider along an axis fixed in body A, with
// an optional motor and translation limits.

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

m2PrismaticJointDef m2DefaultPrismaticJointDef(void)
{
    m2PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){1.0f, 0.0f};
    def.internalValue = M2_PJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2PrismaticJointDef* def)
{
    return m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2FiniteVec2(def->localAxisA) && m2JointGain(def->hertz) &&
           m2JointGain(def->dampingRatio) && m2FiniteF(def->motorSpeed) &&
           m2JointGain(def->maxMotorForce) && m2FiniteF(def->lowerTranslation) &&
           m2FiniteF(def->upperTranslation) && def->lowerTranslation <= def->upperTranslation;
}

m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PJOINT_COOKIE || !DefValid(def))
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
        m2FinishJoint(world, index, (uint8_t)m2_prismaticJoint, bodyA, bodyB, def->localAnchorA,
                      def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->joints.jointFlags[index] =
        (def->enableMotor ? M2_JOINT_MOTOR : 0u) | (def->enableLimit ? M2_JOINT_LIMIT : 0u);
    world->joints.jointMotorSpeed[index] = def->motorSpeed;
    world->joints.jointMaxMotor[index] = def->maxMotorForce;
    world->joints.jointLower[index] = def->lowerTranslation;
    world->joints.jointUpper[index] = def->upperTranslation;
    world->joints.jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    world->joints.jointRefAngle[index] =
        m2RelativeJointAngle(world->bodies.transforms[bodyA].q, world->bodies.transforms[bodyB].q);
    if (world->recorder.journalActive != 0)
    {
        m2OpCreatePrismaticJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePrismaticJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------
//
// The slide axis is fixed in A and turns with it. Along the axis: the
// motor (motorImpulse) and the limits; across it, the perpendicular row
// and the angle row as one coupled pair (impulse).

static void PreparePrismatic(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    c->axis = m2RotateVec2(f->qA, world->joints.jointLocalAxisA[f->joint]);
}

static void SlideRows(const m2JointConstraint* c, const m2JointPose* pose, m2JointRow* along,
                      m2JointRow cross[2], float* travel, float* offAxis)
{
    m2Vec2 axis = m2RotateVec2(pose->turnA, c->axis);
    m2Vec2 perp = {-axis.y, axis.x};
    m2Vec2 gap = m2PoseGap(c, pose);
    *along = m2AxisRow(pose, gap, axis);
    cross[0] = m2AxisRow(pose, gap, perp);
    cross[1] = m2TurnRow();
    *travel = gap.x * axis.x + gap.y * axis.y;
    *offAxis = gap.x * perp.x + gap.y * perp.y;
}

static void WarmStartPrismatic(const m2JointConstraint* c, const m2JointPose* pose,
                               m2JointBodies* b)
{
    m2JointRow along;
    m2JointRow cross[2];
    float travel;
    float offAxis;
    SlideRows(c, pose, &along, cross, &travel, &offAxis);
    m2PushRow(&along, b, c->motorImpulse);
    m2WarmStartLimits(c, &along, b);
    m2PushRow(&cross[0], b, c->impulse.x);
    m2PushRow(&cross[1], b, c->impulse.y);
}

static void SolvePrismatic(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                           const m2JointPass* pass)
{
    m2JointRow along;
    m2JointRow cross[2];
    float travel;
    float offAxis;
    SlideRows(c, pose, &along, cross, &travel, &offAxis);
    if ((c->flags & M2_JOINT_MOTOR) != 0)
    {
        m2SolveRow(&along, b, m2RigidDrive(-c->motorSpeed), &c->motorImpulse, -c->maxMotorImpulse,
                   c->maxMotorImpulse);
    }
    if ((c->flags & M2_JOINT_LIMIT) != 0)
    {
        m2SolveLimits(c, &along, travel, c->soft, b, pass);
    }
    float angle = pass->biased ? m2PoseAngle(c, pose) : 0.0f;
    m2RowDrive x = m2HeldDrive(c->soft, offAxis, pass->biased);
    m2RowDrive y = m2HeldDrive(c->soft, angle, pass->biased);
    m2SolveRowPair(cross, b, (m2Vec2){x.bias, y.bias}, x, &c->impulse);
}

static void PrismaticReaction(const m2World* world, int32_t j, float invH, float* force,
                              float* torque)
{
    // The perpendicular row and the axial motor and limits as force, the
    // angle row as torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    float axial = world->joints.jointSpringImpulse[j] + world->joints.jointMotorImpulse[j] +
                  world->joints.jointLowerImpulse[j] - world->joints.jointUpperImpulse[j];
    float linear = sqrtf(impulse.x * impulse.x + axial * axial);
    *force = linear * invH;
    *torque = m2AbsF(impulse.y) * invH;
}

const m2JointKind m2_prismaticJointKind = {PreparePrismatic, WarmStartPrismatic, SolvePrismatic,
                                           PrismaticReaction};
