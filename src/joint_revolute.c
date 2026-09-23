// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The revolute joint: a pinned point with optional angular spring,
// motor and angle limits.

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

m2RevoluteJointDef m2DefaultRevoluteJointDef(void)
{
    m2RevoluteJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_RJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2RevoluteJointDef* def)
{
    return m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2JointGain(def->hertz) && m2JointGain(def->dampingRatio) &&
           m2JointGain(def->springHertz) && m2JointGain(def->springDampingRatio) &&
           m2FiniteF(def->motorSpeed) && m2JointGain(def->maxMotorTorque) &&
           m2FiniteF(def->lowerAngle) && m2FiniteF(def->upperAngle) &&
           def->lowerAngle <= def->upperAngle;
}

m2JointId m2CreateRevoluteJoint(m2WorldId worldId, const m2RevoluteJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId =
        m2FinishJoint(world, worldId, index, (uint8_t)m2_revoluteJoint, bodyA, bodyB,
                      def->localAnchorA, def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->joints.jointFlags[index] =
        (def->enableMotor ? M2_JOINT_MOTOR : 0u) | (def->enableLimit ? M2_JOINT_LIMIT : 0u);
    world->joints.jointMotorSpeed[index] = def->motorSpeed;
    world->joints.jointMaxMotor[index] = def->maxMotorTorque;
    world->joints.jointLower[index] = def->lowerAngle;
    world->joints.jointUpper[index] = def->upperAngle;
    world->joints.jointRefAngle[index] =
        m2RelativeJointAngle(world->bodies.transforms[bodyA].q, world->bodies.transforms[bodyB].q);
    world->joints.jointHertz2[index] = def->springHertz;
    world->joints.jointDamping2[index] = def->springDampingRatio;
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateRevoluteJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRevoluteJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------
//
// The angle row carries the spring (springImpulse), the motor
// (motorImpulse) and both limits; two point rows pin the anchors
// (impulse).

static void PrepareRevolute(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    c->angularSpring = world->joints.jointHertz2[j] > 0.0f;
    if (c->angularSpring)
    {
        c->spring = m2MakeSoft(world->joints.jointHertz2[j], world->joints.jointDamping2[j], f->h);
    }
}

static void WarmStartRevolute(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    m2JointRow turn = m2TurnRow();
    m2PushRow(&turn, b, c->springImpulse + c->motorImpulse);
    m2WarmStartLimits(c, &turn, b);
    m2PushPointPair(pose->armA, pose->armB, b, c->impulse);
}

static void SolveRevolute(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                          const m2JointPass* pass)
{
    m2JointRow turn = m2TurnRow();
    bool limited = (c->flags & M2_JOINT_LIMIT) != 0;
    float angle = c->angularSpring || limited ? m2PoseAngle(c, pose) : 0.0f;
    if (c->angularSpring)
    {
        m2SolveRow(&turn, b, m2SpringDrive(c->spring, angle), &c->springImpulse, -M2_ROW_FREE,
                   M2_ROW_FREE);
    }
    if ((c->flags & M2_JOINT_MOTOR) != 0)
    {
        m2SolveRow(&turn, b, m2RigidDrive(-c->motorSpeed), &c->motorImpulse, -c->maxMotorImpulse,
                   c->maxMotorImpulse);
    }
    if (limited)
    {
        m2SolveLimits(c, &turn, angle, c->soft, b, pass);
    }
    m2Vec2 gap = pass->biased ? m2PoseGap(c, pose) : (m2Vec2){0.0f, 0.0f};
    m2RowDrive x = m2HeldDrive(c->soft, gap.x, pass->biased);
    m2RowDrive y = m2HeldDrive(c->soft, gap.y, pass->biased);
    m2SolvePointPair(pose->armA, pose->armB, b, (m2Vec2){x.bias, y.bias}, x, &c->impulse,
                     M2_ROW_FREE);
}

static void RevoluteReaction(const m2World* world, int32_t j, float invH, float* force,
                             float* torque)
{
    // The point rows are the linear load; spring, motor and limits the torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    float axial = world->joints.jointSpringImpulse[j] + world->joints.jointMotorImpulse[j] +
                  world->joints.jointLowerImpulse[j] - world->joints.jointUpperImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(axial) * invH;
}

const m2JointKind m2_revoluteJointKind = {PrepareRevolute, WarmStartRevolute, SolveRevolute,
                                          RevoluteReaction};
