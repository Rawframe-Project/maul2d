// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The gear joint: couples the spins of two bodies by a ratio.

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

m2GearJointDef m2DefaultGearJointDef(void)
{
    m2GearJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_GJOINT_COOKIE;
    return def;
}

// Gear registry mapping: ratio rides jointLength; the two previous
// body rotations ride the anchor slots as (c, s) pairs so the phase
// accumulator in prepare survives any number of full turns; the
// accumulated phase itself rides jointRefAngle. All snapshot state.
// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2GearJointDef* def)
{
    return m2FiniteF(def->ratio) && def->ratio != 0.0f;
}

m2JointId m2CreateGearJoint(m2WorldId worldId, const m2GearJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_GJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_gearJoint, bodyA, bodyB,
                                      zero, zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointLength[index] = def->ratio;
    m2Rot qA = world->bodies.transforms[bodyA].q;
    m2Rot qB = world->bodies.transforms[bodyB].q;
    world->joints.jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->joints.jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->joints.jointRefAngle[index] = 0.0f; // in phase by definition at birth
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateGearJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateGearJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2GearJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamGearRatio,
                            ratio);
}

float m2GearJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_gearJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------
//
// One row holds ratio * angle A + angle B at its phase (impulse.x). The
// phase accumulates each step by how far each body actually turned since
// the last one, so many full turns stay exact.

static void PrepareGear(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float ratio = world->joints.jointLength[j];
    m2Rot lastA = {world->joints.jointLocalAnchorA[j].x, world->joints.jointLocalAnchorA[j].y};
    m2Rot lastB = {world->joints.jointLocalAnchorB[j].x, world->joints.jointLocalAnchorB[j].y};
    float phase = world->joints.jointRefAngle[j];
    phase += ratio * m2RelativeJointAngle(lastA, f->qA) + m2RelativeJointAngle(lastB, f->qB);
    world->joints.jointRefAngle[j] = phase;
    world->joints.jointLocalAnchorA[j] = (m2Vec2){f->qA.c, f->qA.s};
    world->joints.jointLocalAnchorB[j] = (m2Vec2){f->qB.c, f->qB.s};
    c->angle = phase;
    c->ratio = ratio;
}

static m2JointRow GearRow(const m2JointConstraint* c)
{
    m2JointRow row = {{0.0f, 0.0f}, c->ratio, {0.0f, 0.0f}, 1.0f};
    return row;
}

static void WarmStartGear(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    (void)pose;
    m2JointRow row = GearRow(c);
    m2PushRow(&row, b, c->impulse.x);
}

static void SolveGear(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                      const m2JointPass* pass)
{
    float C = 0.0f;
    if (pass->biased)
    {
        float turnA = m2Atan2(pose->turnA.s, pose->turnA.c);
        float turnB = m2Atan2(pose->turnB.s, pose->turnB.c);
        C = c->angle + c->ratio * turnA + turnB;
    }
    m2JointRow row = GearRow(c);
    m2SolveRow(&row, b, m2HeldDrive(c->soft, C, pass->biased), &c->impulse.x, -M2_ROW_FREE,
               M2_ROW_FREE);
}

static void GearReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // A pure torque coupling.
    *force = 0.0f;
    *torque = m2AbsF(world->joints.jointImpulse[j].x) * invH;
}

const m2JointKind m2_gearJointKind = {PrepareGear, WarmStartGear, SolveGear, GearReaction};
