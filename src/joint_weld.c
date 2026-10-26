// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The weld joint: a pinned point plus an angle lock, either of them
// optionally soft.

#include "body.h"
#include "broadphase.h"
#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "object_ids.h"
#include "solver_base.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

m2WeldJointDef m2DefaultWeldJointDef(void)
{
    m2WeldJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_WJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2WeldJointDef* def)
{
    return m2LocalVec2(def->localAnchorA) && m2LocalVec2(def->localAnchorB) &&
           m2ValidRate(def->linearHertz) && m2ValidCoefficient(def->linearDampingRatio) &&
           m2ValidRate(def->angularHertz) && m2ValidCoefficient(def->angularDampingRatio);
}

static M2_NOINLINE m2Result CreateWeldJointWorker(m2World* world, const m2WeldJointDef* def,
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
    if (def == NULL || def->internalValue != M2_WJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId =
        m2FinishJoint(world, index, (uint8_t)m2_weldJoint, bodyA, bodyB, def->localAnchorA,
                      def->localAnchorB, 0.0f, def->linearHertz, def->linearDampingRatio);
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->joints.jointHertz2[index] = def->angularHertz;
    world->joints.jointDamping2[index] = def->angularDampingRatio;
    world->joints.jointRefAngle[index] =
        m2RelativeJointAngle(world->bodies.transforms[bodyA].q, world->bodies.transforms[bodyB].q);
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateWeldJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWeldJoint, &record, (int32_t)sizeof(record));
    }
    *jointOut = jointId;
    return m2_success;
}

m2Result m2CreateWeldJoint(m2World* world, const m2WeldJointDef* def, m2JointId* jointOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateWeldJointWorker(world, def, jointOut);
    m2FloatEnvLeave(env);
    return result;
}

// --- Solver ---------------------------------------------------------------
//
// The angle row (motorImpulse) on the angular softness and two point rows
// (impulse) on the linear one. A side with a user stiffness is a real
// spring and pulls in the relax pass too. A rigid weld solves all three
// rows as one block (m2SolveWeldBlock).

static void PrepareWeld(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    c->linearSpring = world->joints.jointHertz[j] > 0.0f;
    c->angularSpring = world->joints.jointHertz2[j] > 0.0f;
    c->spring = c->angularSpring
                    ? m2MakeSoft(world->joints.jointHertz2[j], world->joints.jointDamping2[j], f->h)
                    : m2StiffJointSoftness(f->h);
}

static void WarmStartWeld(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    m2PushWeldBlock(pose->armA, pose->armB, b, c->impulse, c->motorImpulse);
}

static void SolveWeld(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                      const m2JointPass* pass)
{
    if (!c->linearSpring && !c->angularSpring)
    {
        // Rigid: both parts are held on the joint's stiff softness, so the
        // three rows share one drive and solve as one block.
        float angle = pass->biased ? m2PoseAngle(c, pose) : 0.0f;
        m2Vec2 gap = pass->biased ? m2PoseGap(c, pose) : (m2Vec2){0.0f, 0.0f};
        m2RowDrive x = m2HeldDrive(c->soft, gap.x, pass->biased);
        m2RowDrive y = m2HeldDrive(c->soft, gap.y, pass->biased);
        m2RowDrive turn = m2HeldDrive(c->spring, angle, pass->biased);
        if (m2SolveWeldBlock(pose->armA, pose->armB, b, (m2Vec2){x.bias, y.bias}, turn.bias, x,
                             &c->impulse, &c->motorImpulse))
        {
            return;
        }
    }
    m2JointRow turn = m2TurnRow();
    bool angular = pass->biased || c->angularSpring;
    float angle = angular ? m2PoseAngle(c, pose) : 0.0f;
    m2SolveRow(&turn, b, m2HeldDrive(c->spring, angle, angular), &c->motorImpulse, -M2_ROW_FREE,
               M2_ROW_FREE);
    bool linear = pass->biased || c->linearSpring;
    m2Vec2 gap = linear ? m2PoseGap(c, pose) : (m2Vec2){0.0f, 0.0f};
    m2RowDrive x = m2HeldDrive(c->soft, gap.x, linear);
    m2RowDrive y = m2HeldDrive(c->soft, gap.y, linear);
    m2SolvePointPair(pose->armA, pose->armB, b, (m2Vec2){x.bias, y.bias}, x, &c->impulse,
                     M2_ROW_FREE);
}

static void WeldReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The point rows are the linear load; the angle row the torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_weldJointKind = {PrepareWeld, WarmStartWeld, SolveWeld, WeldReaction};
