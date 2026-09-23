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
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] =
        (def->enableMotor ? M2_JOINT_MOTOR : 0u) | (def->enableLimit ? M2_JOINT_LIMIT : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerAngle;
    world->jointUpper[index] = def->upperAngle;
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    world->jointHertz2[index] = def->springHertz;
    world->jointDamping2[index] = def->springDampingRatio;
    if (world->journalActive != 0)
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

static void PrepareRevolute(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2PreparePointBlock(c, f);
    float k = f->iA + f->iB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    c->baseAngle = m2UnwindAngle(m2RelativeJointAngle(f->qA, f->qB) - world->jointRefAngle[j]);
    c->linearSpring = false;
    c->angularSpring = world->jointHertz2[j] > 0.0f;
    c->softness2 = c->angularSpring
                       ? m2MakeSoft(world->jointHertz2[j], world->jointDamping2[j], f->h)
                       : m2MakeSoft(60.0f, 2.0f, f->h);
}

static void SolveRevolute(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    float wA = ctx->wA;
    float wB = ctx->wB;
    bool useBias = ctx->useBias;
    float invH = ctx->invH;
    if (c->angularSpring && c->axialMass > 0.0f)
    {
        // Angular spring (reference revolute_joint.c): a real spring
        // toward the creation angle, biased in every pass by definition.
        float C =
            m2UnwindAngle(c->baseAngle + m2RelativeJointAngle(world->deltaRotations[c->bodyA],
                                                              world->deltaRotations[c->bodyB]));
        float bias = c->softness2.biasRate * C;
        float massScale = c->softness2.massScale;
        float impulseScale = c->softness2.impulseScale;
        float cdot = wB - wA;
        float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * c->springImpulse;
        c->springImpulse += impulse;
        wA -= world->invInertia[c->bodyA] * impulse;
        wB += world->invInertia[c->bodyB] * impulse;
    }
    if ((c->flags & M2_JOINT_MOTOR) != 0 && c->axialMass > 0.0f)
    {
        // Motor: drive relative spin toward motorSpeed within the
        // per-step torque budget (reference formulation).
        float cdot = wB - wA - c->motorSpeed;
        float delta = m2SolveJointAxial(c, cdot, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
        wA -= world->invInertia[c->bodyA] * delta;
        wB += world->invInertia[c->bodyB] * delta;
    }
    if ((c->flags & M2_JOINT_LIMIT) != 0 && c->axialMass > 0.0f)
    {
        float jointAngle =
            m2UnwindAngle(c->baseAngle + m2RelativeJointAngle(world->deltaRotations[c->bodyA],
                                                              world->deltaRotations[c->bodyB]));
        { // lower: open limits speculate, violated limits go soft
            float C = jointAngle - c->lower;
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (C > 0.0f)
            {
                bias = C * invH;
            }
            else if (useBias)
            {
                bias = c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            float delta = m2SolveJointAxial(c, wB - wA, bias, massScale, impulseScale,
                                            &c->lowerImpulse, true);
            wA -= world->invInertia[c->bodyA] * delta;
            wB += world->invInertia[c->bodyB] * delta;
        }
        { // upper: signs flipped so C stays positive when satisfied
            float C = c->upper - jointAngle;
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (C > 0.0f)
            {
                bias = C * invH;
            }
            else if (useBias)
            {
                bias = c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            float delta = m2SolveJointAxial(c, wA - wB, bias, massScale, impulseScale,
                                            &c->upperImpulse, true);
            wA += world->invInertia[c->bodyA] * delta;
            wB -= world->invInertia[c->bodyB] * delta;
        }
    }
    m2SolvePointBlock(world, c, ctx, wA, wB, useBias);
}

static void RevoluteReaction(const m2World* world, int32_t j, float invH, float* force,
                             float* torque)
{
    // The point block is the linear load; spring, motor and limits the torque.
    m2Vec2 impulse = world->jointImpulse[j];
    float axial = world->jointSpringImpulse[j] + world->jointMotorImpulse[j] +
                  world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(axial) * invH;
}

const m2JointKind m2_revoluteJointKind = {PrepareRevolute, m2WarmStartPointJoint, SolveRevolute,
                                          RevoluteReaction};
