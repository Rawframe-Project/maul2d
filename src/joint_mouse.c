// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mouse joint: pulls one point of body B toward a target with a
// soft spring and a force budget.

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
    return m2FinitePos2(def->target) && m2JointGain(def->hertz) && m2JointGain(def->dampingRatio) &&
           m2JointGain(def->maxForce);
}

m2JointId m2CreateMouseJoint(m2WorldId worldId, const m2MouseJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MSJOINT_COOKIE || !DefValid(def))
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
    // The grab point is where the target sits at creation, in B's
    // local frame (the single f64 crossing).
    m2Transform xfB = world->bodies.transforms[bodyB];
    m2Vec2 rel = {(float)(def->target.x - xfB.p.x), (float)(def->target.y - xfB.p.y)};
    m2Vec2 grab = {xfB.q.c * rel.x + xfB.q.s * rel.y, -xfB.q.s * rel.x + xfB.q.c * rel.y};
    m2Vec2 zero = {0.0f, 0.0f};
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_mouseJoint, bodyA, bodyB,
                                      zero, grab, 0.0f, def->hertz, def->dampingRatio);
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
    return jointId;
}

void m2MouseJoint_SetTarget(m2JointId jointId, m2Pos2 target)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
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
    if (world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[bodyB] = 0;
        world->bodies.sleepTimes[bodyB] = 0.0f;
    }
}

m2Pos2 m2MouseJoint_GetTarget(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    m2Pos2 zero = {0.0, 0.0};
    return index >= 0 ? world->joints.jointTargets[index] : zero;
}

float m2MouseJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_mouseJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------

static void PrepareMouse(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float h = f->h;
    m2Rot qB = f->qB;
    m2Vec2 lcB = f->lcB;
    float mB = f->mB;
    float iB = f->iB;
    int32_t bodyB = c->bodyB;
    // Mouse: only body B has rows. Grab arm rB comes from the
    // generic anchors; the target separation base is the f64
    // crossing between B's center and the stored target.
    c->k11 = mB + iB * c->rB.y * c->rB.y;
    c->k12 = -iB * c->rB.x * c->rB.y;
    c->k22 = mB + iB * c->rB.x * c->rB.x;
    m2Vec2 comB2 = m2RotateVec2(qB, lcB);
    float cbx = (float)(world->bodies.transforms[bodyB].p.x - world->joints.jointTargets[j].x);
    float cby = (float)(world->bodies.transforms[bodyB].p.y - world->joints.jointTargets[j].y);
    c->baseCVec = (m2Vec2){cbx + comB2.x, cby + comB2.y};
    c->softness2 = m2MakeSoft(0.5f, 0.1f, h);    // reference spin damper
    c->lower = h * world->joints.jointLength[j]; // force budget
}

static void WarmStartMouse(m2World* world, const m2JointConstraint* c)
{
    // Mouse: body B only.
    int32_t bodyB = c->bodyB;
    float mB = world->bodies.invMass[bodyB];
    float iB = world->bodies.invInertia[bodyB];
    world->bodies.linearVelocities[bodyB].x += mB * c->impulse.x;
    world->bodies.linearVelocities[bodyB].y += mB * c->impulse.y;
    world->bodies.angularVelocities[bodyB] += iB * (m2Cross2(c->rB, c->impulse) + c->motorImpulse);
}

static void SolveMouse(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 drB = ctx->drB;
    // Mouse joint (reference solve, always biased): a soft spin
    // damper, then the soft pull toward the target, clamped to
    // the force budget in lower.
    float mB = world->bodies.invMass[c->bodyB];
    float iB = world->bodies.invInertia[c->bodyB];
    {
        float impulse = iB > 0.0f ? -wB / iB : 0.0f;
        impulse = c->softness2.massScale * impulse - c->softness2.impulseScale * c->motorImpulse;
        c->motorImpulse += impulse;
        wB += iB * impulse;
    }
    {
        m2Vec2 sep = {c->baseCVec.x + world->solver.deltaPositions[c->bodyB].x + drB.x,
                      c->baseCVec.y + world->solver.deltaPositions[c->bodyB].y + drB.y};
        m2Vec2 bias = {c->softness.biasRate * sep.x, c->softness.biasRate * sep.y};
        m2Vec2 cdot = {vB.x - wB * drB.y + bias.x, vB.y + wB * drB.x + bias.y};
        float det = c->k11 * c->k22 - c->k12 * c->k12;
        float invDet = det != 0.0f ? 1.0f / det : 0.0f;
        m2Vec2 raw = {invDet * (c->k22 * cdot.x - c->k12 * cdot.y),
                      invDet * (c->k11 * cdot.y - c->k12 * cdot.x)};
        m2Vec2 impulse = {-c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x,
                          -c->softness.massScale * raw.y - c->softness.impulseScale * c->impulse.y};
        m2Vec2 old = c->impulse;
        c->impulse.x += impulse.x;
        c->impulse.y += impulse.y;
        float budget = c->lower;
        float mag2 = c->impulse.x * c->impulse.x + c->impulse.y * c->impulse.y;
        if (mag2 > budget * budget)
        {
            float mag = sqrtf(mag2);
            float scale = mag > 0.0f ? budget / mag : 0.0f;
            c->impulse.x *= scale;
            c->impulse.y *= scale;
        }
        impulse.x = c->impulse.x - old.x;
        impulse.y = c->impulse.y - old.y;
        vB.x += mB * impulse.x;
        vB.y += mB * impulse.y;
        wB += iB * m2Cross2(drB, impulse);
    }
    world->bodies.linearVelocities[c->bodyB] = vB;
    world->bodies.angularVelocities[c->bodyB] = wB;
}

static void MouseReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The linear pull, and the spin damper's torque in the motor slot.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_mouseJointKind = {PrepareMouse, WarmStartMouse, SolveMouse, MouseReaction};
