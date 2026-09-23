// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ratchet joint: rotation runs free one way and holds at the
// engaged tooth the other way.

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

m2RatchetJointDef m2DefaultRatchetJointDef(void)
{
    m2RatchetJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratchet = 0.5f;
    def.internalValue = M2_RTJOINT_COOKIE;
    return def;
}

// Ratchet registry mapping: tooth angle rides jointLength, phase
// rides jointRefAngle, the accumulated relative angle rides
// jointUpper (multi-turn exact via the gear trick: previous body
// rotations live in the anchor slots as (c, s) pairs), and the
// engaged tooth rides jointLower. All snapshot state.
// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2RatchetJointDef* def)
{
    return m2FiniteF(def->ratchet) && def->ratchet != 0.0f && m2FiniteF(def->phase);
}

m2JointId m2CreateRatchetJoint(m2WorldId worldId, const m2RatchetJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_RTJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_ratchetJoint, bodyA, bodyB,
                                      zero, zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointLength[index] = def->ratchet;
    world->joints.jointRefAngle[index] = def->phase;
    m2Rot qA = world->bodies.transforms[bodyA].q;
    m2Rot qB = world->bodies.transforms[bodyB].q;
    world->joints.jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->joints.jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->joints.jointUpper[index] = 0.0f; // accumulated relative angle
    // Engage the tooth at or behind the angle at creation.
    world->joints.jointLower[index] =
        floorf((0.0f - def->phase) / def->ratchet) * def->ratchet + def->phase;
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateRatchetJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateRatchetJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

float m2RatchetJoint_GetRatchet(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_ratchetJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2RatchetJoint_GetPhase(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_ratchetJoint);
    return index >= 0 ? world->joints.jointRefAngle[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------
//
// The relative angle is tracked across turns through the last step's
// rotations. Turning the free way (the sign of the pitch) moves the
// engaged tooth along to the one at or behind the angle; turning back
// meets a one-sided angle row at that tooth (impulse.x).

static void PrepareRatchet(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float pitch = world->joints.jointLength[j];
    float phase = world->joints.jointRefAngle[j];
    m2Rot lastA = {world->joints.jointLocalAnchorA[j].x, world->joints.jointLocalAnchorA[j].y};
    m2Rot lastB = {world->joints.jointLocalAnchorB[j].x, world->joints.jointLocalAnchorB[j].y};
    float angle = world->joints.jointUpper[j];
    angle += m2RelativeJointAngle(lastB, f->qB) - m2RelativeJointAngle(lastA, f->qA);
    world->joints.jointUpper[j] = angle;
    world->joints.jointLocalAnchorA[j] = (m2Vec2){f->qA.c, f->qA.s};
    world->joints.jointLocalAnchorB[j] = (m2Vec2){f->qB.c, f->qB.s};
    float tooth = world->joints.jointLower[j];
    if (!((tooth - angle) * pitch > 0.0f))
    {
        tooth = floorf((angle - phase) / pitch) * pitch + phase;
        world->joints.jointLower[j] = tooth;
    }
    c->angle = angle - tooth;
    c->ratio = pitch;
}

// The angle row turned so the held direction is positive.
static m2JointRow HoldRow(const m2JointConstraint* c)
{
    return m2ScaleRow(m2TurnRow(), c->ratio > 0.0f ? 1.0f : -1.0f);
}

static void WarmStartRatchet(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    (void)pose;
    m2JointRow row = HoldRow(c);
    m2PushRow(&row, b, c->impulse.x);
}

static void SolveRatchet(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                         const m2JointPass* pass)
{
    float sign = c->ratio > 0.0f ? 1.0f : -1.0f;
    float past = c->angle + m2RelativeJointAngle(pose->turnA, pose->turnB);
    m2JointRow row = HoldRow(c);
    m2RowDrive drive = m2LimitDrive(c->soft, sign * past, pass->invH, pass->biased);
    m2SolveRow(&row, b, drive, &c->impulse.x, 0.0f, M2_ROW_FREE);
}

static void RatchetReaction(const m2World* world, int32_t j, float invH, float* force,
                            float* torque)
{
    // A pure holding torque.
    *force = 0.0f;
    *torque = m2AbsF(world->joints.jointImpulse[j].x) * invH;
}

const m2JointKind m2_ratchetJointKind = {PrepareRatchet, WarmStartRatchet, SolveRatchet,
                                         RatchetReaction};
