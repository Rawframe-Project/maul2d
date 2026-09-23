// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The weld joint: a pinned point plus an angle lock, either of them
// optionally soft.

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
    return m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2JointGain(def->linearHertz) && m2JointGain(def->linearDampingRatio) &&
           m2JointGain(def->angularHertz) && m2JointGain(def->angularDampingRatio);
}

m2JointId m2CreateWeldJoint(m2WorldId worldId, const m2WeldJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WJOINT_COOKIE || !DefValid(def))
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
        m2FinishJoint(world, worldId, index, (uint8_t)m2_weldJoint, bodyA, bodyB, def->localAnchorA,
                      def->localAnchorB, 0.0f, def->linearHertz, def->linearDampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointHertz2[index] = def->angularHertz;
    world->jointDamping2[index] = def->angularDampingRatio;
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        m2OpCreateWeldJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWeldJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------

static void PrepareWeld(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2PreparePointBlock(c, f);
    float k = f->iA + f->iB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    c->baseAngle = m2UnwindAngle(m2RelativeJointAngle(f->qA, f->qB) - world->jointRefAngle[j]);
    c->linearSpring = world->jointHertz[j] > 0.0f;
    c->angularSpring = world->jointHertz2[j] > 0.0f;
    c->softness2 = c->angularSpring
                       ? m2MakeSoft(world->jointHertz2[j], world->jointDamping2[j], f->h)
                       : m2MakeSoft(60.0f, 2.0f, f->h);
}

static void SolveWeld(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    float wA = ctx->wA;
    float wB = ctx->wB;
    bool useBias = ctx->useBias;
    if (c->axialMass > 0.0f)
    {
        // Angle lock (reference weld_joint.c): a full constraint row on
        // relative rotation, accumulator in the motor slot.
        float bias = 0.0f;
        float massScale = 1.0f;
        float impulseScale = 0.0f;
        // Nonzero angular hertz = a real spring: biased even during
        // relax (reference semantics).
        if (useBias || c->angularSpring)
        {
            float C =
                m2UnwindAngle(c->baseAngle + m2RelativeJointAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
            bias = c->softness2.biasRate * C;
            massScale = c->softness2.massScale;
            impulseScale = c->softness2.impulseScale;
        }
        float cdot = wB - wA;
        float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * c->motorImpulse;
        c->motorImpulse += impulse;
        wA -= world->invInertia[c->bodyA] * impulse;
        wB += world->invInertia[c->bodyB] * impulse;
    }
    m2SolvePointBlock(world, c, ctx, wA, wB, useBias || c->linearSpring);
}

static void WeldReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The point block is the linear load; the angle lock in the motor slot the torque.
    m2Vec2 impulse = world->jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_weldJointKind = {PrepareWeld, m2WarmStartPointJoint, SolveWeld, WeldReaction};
