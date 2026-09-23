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
    int32_t body = side == 0 ? world->jointBodyA[index] : world->jointBodyB[index];
    m2Vec2 la = side == 0 ? world->jointLocalAnchorA[index] : world->jointLocalAnchorB[index];
    m2Pos2 g = side == 0 ? world->jointTargets[index] : world->jointTargetsB[index];
    m2Rot q = world->transforms[body].q;
    m2Vec2 arm = {q.c * la.x - q.s * la.y, q.s * la.x + q.c * la.y};
    float dx = (float)(world->transforms[body].p.x - g.x) + arm.x;
    float dy = (float)(world->transforms[body].p.y - g.y) + arm.y;
    return sqrtf(dx * dx + dy * dy);
}

// Pulley registry mapping: ratio rides jointLength, the rope total
// (constant) rides jointRefAngle, ground anchors ride jointTargets
// (A side, shared with mouse) and jointTargetsB. The total is
// CAPTURED from spawn geometry, the reference-angle convention: defs
// carry no length knobs. All snapshot state.
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
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_pulleyJoint, bodyA, bodyB,
                                      def->localAnchorA, def->localAnchorB, def->ratio, 0.0f, 0.0f);
    world->jointTargets[index] = def->groundAnchorA;
    world->jointTargetsB[index] = def->groundAnchorB;
    float lengthA = m2PulleyLiveLength(world, index, 0);
    float lengthB = m2PulleyLiveLength(world, index, 1);
    world->jointRefAngle[index] = lengthA + def->ratio * lengthB;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
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
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamPulleyRatio,
                            ratio);
}

float m2PulleyJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2PulleyJoint_GetLengthA(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? m2PulleyLiveLength(world, index, 0) : 0.0f;
}

float m2PulleyJoint_GetLengthB(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? m2PulleyLiveLength(world, index, 1) : 0.0f;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorA(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointTargets[index] : zero;
}

m2Pos2 m2PulleyJoint_GetGroundAnchorB(m2JointId jointId)
{
    m2Pos2 zero = {0.0, 0.0};
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_pulleyJoint);
    return index >= 0 ? world->jointTargetsB[index] : zero;
}

// --- Solver ---------------------------------------------------------------

static void PreparePulley(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Rot qA = f->qA;
    m2Rot qB = f->qB;
    float mA = f->mA;
    float iA = f->iA;
    float mB = f->mB;
    float iB = f->iB;
    int32_t bodyA = c->bodyA;
    int32_t bodyB = c->bodyB;
    // Pulley (b2 v2.4 reconciliation): equality constraint
    // C = total - lengthA - ratio * lengthB with unit rope
    // directions frozen at prepare. uA rides the axis slot,
    // uB the perp slot, ratio the motorSpeed slot. A side
    // shorter than 10 slop goes limp (zero direction), the
    // reference's slack guard.
    float ratio = world->jointLength[j];
    m2Vec2 armA = m2RotateVec2(qA, world->jointLocalAnchorA[j]);
    m2Vec2 armB = m2RotateVec2(qB, world->jointLocalAnchorB[j]);
    m2Vec2 uA = {(float)(world->transforms[bodyA].p.x - world->jointTargets[j].x) + armA.x,
                 (float)(world->transforms[bodyA].p.y - world->jointTargets[j].y) + armA.y};
    m2Vec2 uB = {(float)(world->transforms[bodyB].p.x - world->jointTargetsB[j].x) + armB.x,
                 (float)(world->transforms[bodyB].p.y - world->jointTargetsB[j].y) + armB.y};
    float lengthA = sqrtf(uA.x * uA.x + uA.y * uA.y);
    float lengthB = sqrtf(uB.x * uB.x + uB.y * uB.y);
    c->axis = lengthA > 0.05f ? (m2Vec2){uA.x / lengthA, uA.y / lengthA} : (m2Vec2){0.0f, 0.0f};
    c->perp = lengthB > 0.05f ? (m2Vec2){uB.x / lengthB, uB.y / lengthB} : (m2Vec2){0.0f, 0.0f};
    c->baseC = world->jointRefAngle[j] - lengthA - ratio * lengthB;
    c->motorSpeed = ratio; // carried into the solve
    float ruA = m2Cross2(c->rA, c->axis);
    float ruB = m2Cross2(c->rB, c->perp);
    float k = mA + iA * ruA * ruA + ratio * ratio * (mB + iB * ruB * ruB);
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
}

static void WarmStartPulley(m2World* world, const m2JointConstraint* c)
{
    // Pulley: PA = -L*uA on A, PB = -ratio*L*uB on B.
    float L = c->impulse.x;
    m2Vec2 PA = {-L * c->axis.x, -L * c->axis.y};
    m2Vec2 PB = {-c->motorSpeed * L * c->perp.x, -c->motorSpeed * L * c->perp.y};
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    world->linearVelocities[c->bodyA].x += mA * PA.x;
    world->linearVelocities[c->bodyA].y += mA * PA.y;
    world->angularVelocities[c->bodyA] += iA * m2Cross2(c->rA, PA);
    world->linearVelocities[c->bodyB].x += mB * PB.x;
    world->linearVelocities[c->bodyB].y += mB * PB.y;
    world->angularVelocities[c->bodyB] += iB * m2Cross2(c->rB, PB);
}

static void SolvePulley(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vA = ctx->vA;
    float wA = ctx->wA;
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 drA = ctx->drA;
    m2Vec2 drB = ctx->drB;
    bool useBias = ctx->useBias;
    // Pulley: Cdot = -dot(uA, vpA) - ratio * dot(uB, vpB),
    // stiff-biased with C tracked through per-body position
    // deltas against the frozen rope directions.
    float ratio = c->motorSpeed;
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    float bias = 0.0f;
    float massScale = 1.0f;
    float impulseScale = 0.0f;
    if (useBias)
    {
        m2Vec2 dsA = {world->deltaPositions[c->bodyA].x + (drA.x - c->rA.x),
                      world->deltaPositions[c->bodyA].y + (drA.y - c->rA.y)};
        m2Vec2 dsB = {world->deltaPositions[c->bodyB].x + (drB.x - c->rB.x),
                      world->deltaPositions[c->bodyB].y + (drB.y - c->rB.y)};
        float C = c->baseC - (dsA.x * c->axis.x + dsA.y * c->axis.y) -
                  ratio * (dsB.x * c->perp.x + dsB.y * c->perp.y);
        bias = c->softness.massScale * c->softness.biasRate * C;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m2Vec2 vpA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
    m2Vec2 vpB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
    float cdot =
        -(vpA.x * c->axis.x + vpA.y * c->axis.y) - ratio * (vpB.x * c->perp.x + vpB.y * c->perp.y);
    float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
    c->impulse.x += impulse;
    m2Vec2 PA = {-impulse * c->axis.x, -impulse * c->axis.y};
    m2Vec2 PB = {-ratio * impulse * c->perp.x, -ratio * impulse * c->perp.y};
    world->linearVelocities[c->bodyA].x = vA.x + mA * PA.x;
    world->linearVelocities[c->bodyA].y = vA.y + mA * PA.y;
    world->angularVelocities[c->bodyA] = wA + iA * m2Cross2(c->rA, PA);
    world->linearVelocities[c->bodyB].x = vB.x + mB * PB.x;
    world->linearVelocities[c->bodyB].y = vB.y + mB * PB.y;
    world->angularVelocities[c->bodyB] = wB + iB * m2Cross2(c->rB, PB);
}

static void PulleyReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The A-side rope tension (B feels ratio times this).
    *force = m2AbsF(world->jointImpulse[j].x) * invH;
    *torque = 0.0f;
}

const m2JointKind m2_pulleyJointKind = {PreparePulley, WarmStartPulley, SolvePulley,
                                        PulleyReaction};
