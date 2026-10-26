// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint: drives body B toward an offset pose relative to
// body A within force and torque budgets, hard or as a spring.

#include "body.h"
#include "broadphase.h"
#include "fpenv.h"
#include "island.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "object_ids.h"
#include "solver_base.h"
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
    return m2LocalVec2(def->linearOffset) && m2ValidAngle(def->angularOffset) &&
           m2ValidMaxForce(def->maxForce) && m2ValidMaxForce(def->maxTorque) &&
           m2ValidFraction(def->correctionFactor) && m2ValidRate(def->hertz) &&
           m2ValidCoefficient(def->dampingRatio);
}

static M2_NOINLINE m2Result CreateMotorJointWorker(m2World* world, const m2MotorJointDef* def,
                                                   m2JointId* jointOut)
{
    if (jointOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *jointOut = m2_nullJointId;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (def == NULL || def->internalValue != M2_MOJOINT_COOKIE || !DefValid(def))
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    if (bodyA == bodyB)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorCapacity);
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
    *jointOut = jointId;
    return m2_success;
}

m2Result m2CreateMotorJoint(m2World* world, const m2MotorJointDef* def, m2JointId* jointOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateMotorJointWorker(world, def, jointOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void MotorJointSetOffsetsWorker(m2World* world, m2JointId jointId,
                                                   m2Vec2 linearOffset, float angularOffset)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2LocalVec2(linearOffset) || !m2ValidAngle(angularOffset))
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

void m2MotorJoint_SetOffsets(m2World* world, m2JointId jointId, m2Vec2 linearOffset,
                             float angularOffset)
{
    uint64_t env = m2FloatEnvEnter();
    MotorJointSetOffsetsWorker(world, jointId, linearOffset, angularOffset);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE m2Vec2 MotorJointGetLinearOffsetWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAxisA[index] : zero;
}

m2Vec2 m2MotorJoint_GetLinearOffset(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = MotorJointGetLinearOffsetWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float MotorJointGetAngularOffsetWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointRefAngle[index] : 0.0f;
}

float m2MotorJoint_GetAngularOffset(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = MotorJointGetAngularOffsetWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float MotorJointGetMaxForceWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2MotorJoint_GetMaxForce(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = MotorJointGetMaxForceWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float MotorJointGetCorrectionFactorWorker(const m2World* world,
                                                             m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointDamping[index] : 0.0f;
}

float m2MotorJoint_GetCorrectionFactor(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = MotorJointGetCorrectionFactorWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
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
    m2SolvePointPair(pose->armA, pose->armB, b, (m2Vec2){x.bias, y.bias}, x, &c->impulse,
                     c->maxPullImpulse);
}

static void MotorReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The point rows, and the angle row's pure torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_motorJointKind = {PrepareMotor, WarmStartMotor, SolveMotor, MotorReaction};
