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
    world->jointLength[index] = def->ratchet;
    world->jointRefAngle[index] = def->phase;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointUpper[index] = 0.0f; // accumulated relative angle
    // Engage the tooth at or behind the spawn angle (reference click).
    world->jointLower[index] =
        floorf((0.0f - def->phase) / def->ratchet) * def->ratchet + def->phase;
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
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
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

float m2RatchetJoint_GetPhase(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_ratchetJoint);
    return index >= 0 ? world->jointRefAngle[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------

static void PrepareRatchet(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Rot qA = f->qA;
    m2Rot qB = f->qB;
    float iA = f->iA;
    float iB = f->iB;
    // Ratchet (Chipmunk cpRatchetJoint reconciliation): track
    // the relative angle multi-turn exact via previous-rotation
    // slots, click the engaged tooth forward when the angle
    // passes it, and hold a one-sided row against back-spin.
    float ratchet = world->jointLength[j];
    float phase = world->jointRefAngle[j];
    m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
    m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
    float angle = world->jointUpper[j];
    angle += m2RelativeJointAngle(prevB, qB) - m2RelativeJointAngle(prevA, qA);
    world->jointUpper[j] = angle;
    world->jointLocalAnchorA[j] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[j] = (m2Vec2){qB.c, qB.s};
    float engaged = world->jointLower[j];
    float diff = engaged - angle;
    if (!(diff * ratchet > 0.0f))
    {
        // Free direction: click to the tooth at or behind us.
        engaged = floorf((angle - phase) / ratchet) * ratchet + phase;
        world->jointLower[j] = engaged;
    }
    c->baseAngle = angle - engaged; // C0, sign-adjusted in solve
    c->motorSpeed = ratchet;        // carries the free direction
    float k = iA + iB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
}

static void WarmStartRatchet(m2World* world, const m2JointConstraint* c)
{
    // Ratchet: one-sided angular impulse in the hold direction.
    float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
    float L = s * c->impulse.x;
    world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * L;
    world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
}

static void SolveRatchet(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    float wA = ctx->wA;
    float wB = ctx->wB;
    bool useBias = ctx->useBias;
    float invH = ctx->invH;
    // Ratchet: the revolute limit row, sign-folded so the free
    // direction never feels it: C' = s*(angle - engaged) >= 0,
    // speculative when open, stiff-soft when violated.
    float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
    float iA = world->invInertia[c->bodyA];
    float iB = world->invInertia[c->bodyB];
    float angleNow = c->baseAngle + m2RelativeJointAngle(world->deltaRotations[c->bodyA],
                                                         world->deltaRotations[c->bodyB]);
    float C = s * angleNow;
    float bias = 0.0f;
    float massScale = 1.0f;
    float impulseScale = 0.0f;
    if (C > 0.0f)
    {
        bias = C * invH; // speculative: stop exactly at the tooth
    }
    else if (useBias)
    {
        bias = c->softness.biasRate * C;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    float cdot = s * (wB - wA);
    float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * c->impulse.x;
    float next = c->impulse.x + impulse;
    next = next > 0.0f ? next : 0.0f;
    impulse = next - c->impulse.x;
    c->impulse.x = next;
    world->angularVelocities[c->bodyA] = wA - iA * (s * impulse);
    world->angularVelocities[c->bodyB] = wB + iB * (s * impulse);
}

static void RatchetReaction(const m2World* world, int32_t j, float invH, float* force,
                            float* torque)
{
    // A pure holding torque.
    *force = 0.0f;
    *torque = m2AbsF(world->jointImpulse[j].x) * invH;
}

const m2JointKind m2_ratchetJointKind = {PrepareRatchet, WarmStartRatchet, SolveRatchet,
                                         RatchetReaction};
