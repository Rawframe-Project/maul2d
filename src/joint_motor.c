// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint: drives body B toward an offset pose relative to
// body A within force and torque budgets, hard or as a spring.

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

m2MotorJointDef m2DefaultMotorJointDef(void)
{
    m2MotorJointDef def;
    memset(&def, 0, sizeof(def));
    def.maxForce = 1.0f;
    def.maxTorque = 1.0f;
    def.correctionFactor = 0.3f;
    def.internalValue = M2_MOJOINT_COOKIE;
    return def;
}

// Registry mapping for the utility joints (documented deviations from
// the slot names): motor keeps linearOffset in jointLocalAxisA,
// angularOffset in jointRefAngle, maxForce in jointLength and
// correctionFactor in jointDamping; mouse keeps maxForce in
// jointLength and its world target in jointTargets.
// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2MotorJointDef* def)
{
    return m2FiniteVec2(def->linearOffset) && m2FiniteF(def->angularOffset) &&
           m2JointGain(def->maxForce) && m2JointGain(def->maxTorque) &&
           m2JointGain(def->correctionFactor) && def->correctionFactor <= 1.0f &&
           m2JointGain(def->hertz) && m2JointGain(def->dampingRatio);
}

m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MOJOINT_COOKIE || !DefValid(def))
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
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullJointId;
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = m2FinishJoint(world, index, (uint8_t)m2_motorJoint, bodyA, bodyB, zero,
                                      zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointLocalAxisA[index] = def->linearOffset;
    world->joints.jointRefAngle[index] = def->angularOffset;
    world->joints.jointMaxMotor[index] = def->maxTorque;
    world->joints.jointLength[index] = def->maxForce;
    world->joints.jointDamping[index] = def->correctionFactor;
    // Spring drive rides the otherwise-idle secondary spring slots
    // (jointHertz2/jointDamping2 are only read for the angular spring of
    // the revolute and weld, which type 6 is not).
    world->joints.jointHertz2[index] = def->hertz;
    world->joints.jointDamping2[index] = def->dampingRatio;
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateMotorJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMotorJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2FiniteVec2(linearOffset) || !m2FiniteF(angularOffset))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpMotorOffsets record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.linear = linearOffset;
        record.angular = angularOffset;
        m2JournalRecord(world, m2_opMotorOffsets, &record, (int32_t)sizeof(record));
    }
    world->joints.jointLocalAxisA[index] = linearOffset;
    world->joints.jointRefAngle[index] = angularOffset;
    // Retargeting wakes both ends: the platform starts moving.
    int32_t bodyA = world->joints.jointBodyA[index];
    int32_t bodyB = world->joints.jointBodyB[index];
    m2WakeIfDynamic(world, bodyA);
    m2WakeIfDynamic(world, bodyB);
}

m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAxisA[index] : zero;
}

float m2MotorJoint_GetAngularOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointRefAngle[index] : 0.0f;
}

float m2MotorJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2MotorJoint_GetCorrectionFactor(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointDamping[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------
//
// The motor joint drives B toward a pose relative to A: the angle row
// (motorImpulse) within the torque budget, then the point rows
// (impulse) within the force budget. Without a spring both remove the
// correction factor's share of their error each step; with one they
// pull as that spring.

static void PrepareMotor(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    // The linear offset lives in A's frame and turns with A.
    c->axis = m2RotateVec2(f->qA, world->joints.jointLocalAxisA[j]);
    c->correction = world->joints.jointDamping[j];
    c->maxPullImpulse = f->h * world->joints.jointLength[j];
    c->linearSpring = world->joints.jointHertz2[j] > 0.0f;
    if (c->linearSpring)
    {
        c->spring = m2MakeSoft(world->joints.jointHertz2[j], world->joints.jointDamping2[j], f->h);
    }
}

static void WarmStartMotor(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    m2JointRow turn = m2TurnRow();
    m2PushRow(&turn, b, c->motorImpulse);
    m2PushPointPair(pose->armA, pose->armB, b, c->impulse);
}

static m2RowDrive MotorDrive(const m2JointConstraint* c, float C, float invH)
{
    return c->linearSpring ? m2SpringDrive(c->spring, C) : m2RigidDrive(invH * c->correction * C);
}

static void SolveMotor(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                       const m2JointPass* pass)
{
    m2JointRow turn = m2TurnRow();
    m2SolveRow(&turn, b, MotorDrive(c, m2PoseAngle(c, pose), pass->invH), &c->motorImpulse,
               -c->maxMotorImpulse, c->maxMotorImpulse);
    m2Vec2 offset = m2RotateVec2(pose->turnA, c->axis);
    m2Vec2 gap = m2PoseGap(c, pose);
    m2RowDrive x = MotorDrive(c, gap.x - offset.x, pass->invH);
    m2RowDrive y = MotorDrive(c, gap.y - offset.y, pass->invH);
    m2SolvePointPair(pose->armA, pose->armB, c->pointMass, b, (m2Vec2){x.bias, y.bias}, x,
                     &c->impulse, c->maxPullImpulse);
}

static void MotorReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The point rows, and the angle row's pure torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_motorJointKind = {PrepareMotor, WarmStartMotor, SolveMotor, MotorReaction};
