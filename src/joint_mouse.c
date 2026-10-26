// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mouse joint: pulls one point of body B toward a target with a
// soft spring and a force budget.

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

m2MouseJointDef m2DefaultMouseJointDef(void)
{
    m2MouseJointDef def;
    memset(&def, 0, sizeof(def));
    def.hertz = 4.0f;
    def.dampingRatio = 1.0f;
    def.maxForce = 35.0f;
    def.internalValue = M2_MSJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2MouseJointDef* def)
{
    return m2FinitePos2(def->target) && m2ValidRate(def->hertz) &&
           m2ValidCoefficient(def->dampingRatio) && m2ValidMaxForce(def->maxForce);
}

static M2_NOINLINE m2Result CreateMouseJointWorker(m2World* world, const m2MouseJointDef* def,
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
    if (def == NULL || def->internalValue != M2_MSJOINT_COOKIE || !DefValid(def))
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
    // The grab point is where the target sits at creation, in B's
    // local frame (the single f64 crossing). It becomes B's anchor, a
    // body-local length, so it stays within M2_HUGE like any other.
    m2Transform xfB = world->bodies.transforms[bodyB];
    m2Vec2 rel = {(float)(def->target.x - xfB.p.x), (float)(def->target.y - xfB.p.y)};
    m2Vec2 grab = {xfB.q.c * rel.x + xfB.q.s * rel.y, -xfB.q.s * rel.x + xfB.q.c * rel.y};
    if (!m2LocalVec2(grab))
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = m2FinishJoint(world, index, (uint8_t)m2_mouseJoint, bodyA, bodyB, zero,
                                      grab, 0.0f, def->hertz, def->dampingRatio);
    world->joints.jointLength[index] = def->maxForce;
    world->joints.jointTargets[index] = def->target;
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateMouseJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMouseJoint, &record, (int32_t)sizeof(record));
    }
    *jointOut = jointId;
    return m2_success;
}

m2Result m2CreateMouseJoint(m2World* world, const m2MouseJointDef* def, m2JointId* jointOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateMouseJointWorker(world, def, jointOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void MouseJointSetTargetWorker(m2World* world, m2JointId jointId, m2Pos2 target)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2FinitePos2(target))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpMouseTarget record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.target = target;
        m2JournalRecord(world, m2_opMouseTarget, &record, (int32_t)sizeof(record));
    }
    world->joints.jointTargets[index] = target;
    int32_t bodyB = world->joints.jointBodyB[index];
    m2WakeIfDynamic(world, bodyB);
}

void m2MouseJoint_SetTarget(m2World* world, m2JointId jointId, m2Pos2 target)
{
    uint64_t env = m2FloatEnvEnter();
    MouseJointSetTargetWorker(world, jointId, target);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE m2Pos2 MouseJointGetTargetWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    m2Pos2 zero = {0.0, 0.0};
    return index >= 0 ? world->joints.jointTargets[index] : zero;
}

m2Pos2 m2MouseJoint_GetTarget(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Pos2 result = MouseJointGetTargetWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float MouseJointGetMaxForceWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2MouseJoint_GetMaxForce(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = MouseJointGetMaxForceWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

// --- Solver ---------------------------------------------------------------
//
// Only body B has rows: a soft spin damper (motorImpulse), then a spring
// of B's own mass pulling B's anchor to the target (impulse) within the
// force budget (m2SolveSpringPair). The gap is B's anchor minus the
// target.

static void PrepareMouse(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Vec2 center = m2RotateVec2(f->qB, world->bodies.localCenters[c->bodyB]);
    m2Pos2 origin = world->bodies.transforms[c->bodyB].p;
    c->gap =
        (m2Vec2){m2WorldOffset(origin.x - world->joints.jointTargets[j].x) + center.x + c->armB.x,
                 m2WorldOffset(origin.y - world->joints.jointTargets[j].y) + center.y + c->armB.y};
    c->spring = m2MakeSoft(0.5f, 0.1f, f->h);
    c->maxPullImpulse = f->h * world->joints.jointLength[j];
    // The pull's spring, w = 2 pi hertz: 1 / (h w (2 zeta + h w)), as
    // m2SolveSpringPair takes it, with none when that is not finite.
    float omega = 2.0f * M2_PI * world->joints.jointHertz[j];
    float a = f->h * omega * (2.0f * world->joints.jointDamping[j] + f->h * omega);
    float compliance = 1.0f / a;
    c->compliance = a > 0.0f && compliance <= M2_ROW_FREE ? compliance : 0.0f;
}

// Body B alone: A drops out of the rows with its motion and its mass.
static m2JointBodies BodyBOnly(const m2JointBodies* b)
{
    m2JointBodies only = *b;
    only.vA = (m2Vec2){0.0f, 0.0f};
    only.wA = 0.0f;
    only.mA = 0.0f;
    only.iA = 0.0f;
    return only;
}

static const m2JointRow s_spin = {{0.0f, 0.0f}, 0.0f, {0.0f, 0.0f}, 1.0f};

static void WarmStartMouse(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    m2JointBodies only = BodyBOnly(b);
    m2PushRow(&s_spin, &only, c->motorImpulse);
    m2PushPointPair((m2Vec2){0.0f, 0.0f}, pose->armB, &only, c->impulse);
    b->vB = only.vB;
    b->wB = only.wB;
}

static void SolveMouse(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                       const m2JointPass* pass)
{
    (void)pass; // always pulling: the target is a spring by definition
    m2JointBodies only = BodyBOnly(b);
    m2SolveRow(&s_spin, &only, m2SpringDrive(c->spring, 0.0f), &c->motorImpulse, -M2_ROW_FREE,
               M2_ROW_FREE);
    m2Vec2 gap = {c->gap.x + pose->moveB.x + (pose->armB.x - c->armB.x),
                  c->gap.y + pose->moveB.y + (pose->armB.y - c->armB.y)};
    m2SolveSpringPair((m2Vec2){0.0f, 0.0f}, pose->armB, &only, gap, c->soft.biasRate, c->compliance,
                      &c->impulse, c->maxPullImpulse);
    b->vB = only.vB;
    b->wB = only.wB;
}

static void MouseReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The pull, and the spin damper's torque.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_mouseJointKind = {PrepareMouse, WarmStartMouse, SolveMouse, MouseReaction};
