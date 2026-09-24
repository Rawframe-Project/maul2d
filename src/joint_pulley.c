// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pulley joint: two ropes over fixed ground anchors sharing one
// total length, one side geared by a ratio.

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

m2PulleyJointDef m2DefaultPulleyJointDef(void)
{
    m2PulleyJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_PLJOINT_COOKIE;
    return def;
}

// Live rope length for one pulley side: attach point (f64 body origin
// plus rotated local anchor) against the f64 ground anchor, a single
// f64 crossing like every other narrowphase entry.
float m2PulleyLiveLength(m2World* world, int32_t index, int32_t side)
{
    int32_t body = side == 0 ? world->joints.jointBodyA[index] : world->joints.jointBodyB[index];
    m2Vec2 la =
        side == 0 ? world->joints.jointLocalAnchorA[index] : world->joints.jointLocalAnchorB[index];
    m2Pos2 g = side == 0 ? world->joints.jointTargets[index] : world->joints.jointTargetsB[index];
    m2Rot q = world->bodies.transforms[body].q;
    m2Vec2 arm = {q.c * la.x - q.s * la.y, q.s * la.x + q.c * la.y};
    float dx = (float)(world->bodies.transforms[body].p.x - g.x) + arm.x;
    float dy = (float)(world->bodies.transforms[body].p.y - g.y) + arm.y;
    return sqrtf(dx * dx + dy * dy);
}

// Pulley registry mapping: ratio rides jointLength, the rope total
// (constant) rides jointRefAngle, ground anchors ride jointTargets
// (A side, shared with mouse) and jointTargetsB. The total is taken
// from the geometry at creation, the way a revolute joint takes its
// reference angle: defs carry no length. All snapshot state.
// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2PulleyJointDef* def)
{
    return m2FinitePos2(def->groundAnchorA) && m2FinitePos2(def->groundAnchorB) &&
           m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2FiniteF(def->ratio) && def->ratio > 0.0f;
}

m2JointId m2CreatePulleyJoint(m2WorldId worldId, const m2PulleyJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PLJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId = m2FinishJoint(world, index, (uint8_t)m2_pulleyJoint, bodyA, bodyB,
                                      def->localAnchorA, def->localAnchorB, def->ratio, 0.0f, 0.0f);
    world->joints.jointTargets[index] = def->groundAnchorA;
    world->joints.jointTargetsB[index] = def->groundAnchorB;
    float lengthA = m2PulleyLiveLength(world, index, 0);
    float lengthB = m2PulleyLiveLength(world, index, 1);
    world->joints.jointRefAngle[index] = lengthA + def->ratio * lengthB;
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreatePulleyJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePulleyJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2PulleyJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2SetJointParamInternal(m2WorldFromTag(jointId.world), jointId, m2_jointParamPulleyRatio,
                            ratio);
}

float m2PulleyJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2PulleyJoint_GetLengthA(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? m2PulleyLiveLength(world, index, 0) : 0.0f;
}

float m2PulleyJoint_GetLengthB(m2JointId jointId)
{
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? m2PulleyLiveLength(world, index, 1) : 0.0f;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorA(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->joints.jointTargets[index] : zero;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorB(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromTag(jointId.world);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->joints.jointTargetsB[index] : zero;
}

// --- Solver ---------------------------------------------------------------
//
// One row holds the rope: length A + ratio * length B stays at the total
// (impulse.x). Each rope runs from its ground point to its anchor; a side
// shorter than 5 cm goes limp.

static void PreparePulley(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Pos2 originA = world->bodies.transforms[c->bodyA].p;
    m2Pos2 originB = world->bodies.transforms[c->bodyB].p;
    m2Vec2 anchorA = m2RotateVec2(f->qA, world->joints.jointLocalAnchorA[j]);
    m2Vec2 anchorB = m2RotateVec2(f->qB, world->joints.jointLocalAnchorB[j]);
    c->ropeA = (m2Vec2){(float)(originA.x - world->joints.jointTargets[j].x) + anchorA.x,
                        (float)(originA.y - world->joints.jointTargets[j].y) + anchorA.y};
    c->ropeB = (m2Vec2){(float)(originB.x - world->joints.jointTargetsB[j].x) + anchorB.x,
                        (float)(originB.y - world->joints.jointTargetsB[j].y) + anchorB.y};
    c->ratio = world->joints.jointLength[j];
    c->length = world->joints.jointRefAngle[j];
}

// A rope now, and its unit direction (zero when limp).
static m2Vec2 Rope(m2Vec2 rope0, m2Vec2 move, m2Vec2 arm, m2Vec2 arm0, float* length)
{
    m2Vec2 rope = {rope0.x + move.x + (arm.x - arm0.x), rope0.y + move.y + (arm.y - arm0.y)};
    *length = sqrtf(rope.x * rope.x + rope.y * rope.y);
    return *length > 0.05f ? (m2Vec2){rope.x / *length, rope.y / *length} : (m2Vec2){0.0f, 0.0f};
}

// The rope row and its error.
static m2JointRow PulleyRow(const m2JointConstraint* c, const m2JointPose* pose, float* C)
{
    float lengthA;
    float lengthB;
    m2Vec2 uA = Rope(c->ropeA, pose->moveA, pose->armA, c->armA, &lengthA);
    m2Vec2 uB = Rope(c->ropeB, pose->moveB, pose->armB, c->armB, &lengthB);
    *C = c->length - lengthA - c->ratio * lengthB;
    m2JointRow row = {{-uA.x, -uA.y},
                      -m2Cross2(pose->armA, uA),
                      {-c->ratio * uB.x, -c->ratio * uB.y},
                      -c->ratio * m2Cross2(pose->armB, uB)};
    return row;
}

static void WarmStartPulley(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    float C;
    m2JointRow row = PulleyRow(c, pose, &C);
    m2PushRow(&row, b, c->impulse.x);
}

static void SolvePulley(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                        const m2JointPass* pass)
{
    float C;
    m2JointRow row = PulleyRow(c, pose, &C);
    m2SolveRow(&row, b, m2HeldDrive(c->soft, C, pass->biased), &c->impulse.x, -M2_ROW_FREE,
               M2_ROW_FREE);
}

static void PulleyReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The A-side rope tension (B feels ratio times this).
    *force = m2AbsF(world->joints.jointImpulse[j].x) * invH;
    *torque = 0.0f;
}

const m2JointKind m2_pulleyJointKind = {PreparePulley, WarmStartPulley, SolvePulley,
                                        PulleyReaction};
