// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motor joint: drives body B toward an offset pose relative to
// body A within force and torque budgets, hard or as a spring.

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

m2MotorJointDef m2DefaultMotorJointDef(void)
{
    m2MotorJointDef def;
    memset(&def, 0, sizeof(def));
    def.maxForce = 1.0f;
    def.maxTorque = 1.0f;
    def.correctionFactor = 0.3f;
    def.internalValue = M2_MOJOINT_COOKIE;
    return def;
}

// Registry mapping for the utility joints (documented deviations from
// the slot names): motor keeps linearOffset in jointLocalAxisA,
// angularOffset in jointRefAngle, maxForce in jointLength and
// correctionFactor in jointDamping; mouse keeps maxForce in
// jointLength and its world target in jointTargets.
// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2MotorJointDef* def)
{
    return m2FiniteVec2(def->linearOffset) && m2FiniteF(def->angularOffset) &&
           m2JointGain(def->maxForce) && m2JointGain(def->maxTorque) &&
           m2JointGain(def->correctionFactor) && def->correctionFactor <= 1.0f &&
           m2JointGain(def->hertz) && m2JointGain(def->dampingRatio);
}

m2JointId m2CreateMotorJoint(m2WorldId worldId, const m2MotorJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_MOJOINT_COOKIE || !DefValid(def))
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
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_motorJoint, bodyA, bodyB,
                                      zero, zero, 0.0f, 0.0f, 0.0f);
    world->joints.jointLocalAxisA[index] = def->linearOffset;
    world->joints.jointRefAngle[index] = def->angularOffset;
    world->joints.jointMaxMotor[index] = def->maxTorque;
    world->joints.jointLength[index] = def->maxForce;
    world->joints.jointDamping[index] = def->correctionFactor;
    // Spring drive rides the otherwise-idle secondary spring slots
    // (jointHertz2/jointDamping2 are only read for the angular spring of
    // the revolute and weld, which type 6 is not).
    world->joints.jointHertz2[index] = def->hertz;
    world->joints.jointDamping2[index] = def->dampingRatio;
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateMotorJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateMotorJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2MotorJoint_SetOffsets(m2JointId jointId, m2Vec2 linearOffset, float angularOffset)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    if (index < 0)
    {
        return; // TypedJointSlot refused
    }
    if (!m2FiniteVec2(linearOffset) || !m2FiniteF(angularOffset))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpMotorOffsets record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.linear = linearOffset;
        record.angular = angularOffset;
        m2JournalRecord(world, m2_opMotorOffsets, &record, (int32_t)sizeof(record));
    }
    world->joints.jointLocalAxisA[index] = linearOffset;
    world->joints.jointRefAngle[index] = angularOffset;
    // Retargeting wakes both ends: the platform starts moving.
    int32_t bodyA = world->joints.jointBodyA[index];
    int32_t bodyB = world->joints.jointBodyB[index];
    if (world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[bodyA] = 0;
        world->bodies.sleepTimes[bodyA] = 0.0f;
    }
    if (world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[bodyB] = 0;
        world->bodies.sleepTimes[bodyB] = 0.0f;
    }
}

m2Vec2 m2MotorJoint_GetLinearOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAxisA[index] : zero;
}

float m2MotorJoint_GetAngularOffset(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointRefAngle[index] : 0.0f;
}

float m2MotorJoint_GetMaxForce(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointLength[index] : 0.0f;
}

float m2MotorJoint_GetCorrectionFactor(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_motorJoint);
    return index >= 0 ? world->joints.jointDamping[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------

static void PrepareMotor(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float h = f->h;
    m2PreparePointBlock(c, f);
    // Separations are measured against the offsets. linearOffset lives
    // in A's frame (the documented contract; the reference's code
    // disagrees with its own comment and we side with the comment).
    m2Vec2 offset = m2RotateVec2(f->qA, world->joints.jointLocalAxisA[j]);
    c->baseCVec = (m2Vec2){f->dx - offset.x, f->dy - offset.y};
    c->baseAngle =
        m2UnwindAngle(m2RelativeJointAngle(f->qA, f->qB) - world->joints.jointRefAngle[j]);
    float k = f->iA + f->iB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
    // correctionFactor rides the motorSpeed slot into the solve; the
    // linear budget rides lower.
    c->motorSpeed = world->joints.jointDamping[j];
    c->lower = h * world->joints.jointLength[j];
    if (world->joints.jointHertz2[j] > 0.0f)
    {
        // Spring drive: a soft softness replaces the hard
        // correctionFactor bias in both rows.
        c->flags |= M2_JOINT_SPRING_DRIVE;
        c->softness = m2MakeSoft(world->joints.jointHertz2[j], world->joints.jointDamping2[j], h);
    }
    c->linearSpring = false;
    c->angularSpring = false;
    c->softness2 = m2MakeSoft(60.0f, 2.0f, h);
}

static void SolveMotor(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vA = ctx->vA;
    float wA = ctx->wA;
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 ds = ctx->ds;
    float invH = ctx->invH;
    // Motor joint (reference solve, always biased): angular row
    // first, then the clamped linear block. correctionFactor
    // rides the motorSpeed slot, the force budget rides lower.
    float mA = world->bodies.invMass[c->bodyA];
    float iA = world->bodies.invInertia[c->bodyA];
    float mB = world->bodies.invMass[c->bodyB];
    float iB = world->bodies.invInertia[c->bodyB];
    bool motorSpring = (c->flags & M2_JOINT_SPRING_DRIVE) != 0;
    {
        float angC = m2UnwindAngle(c->baseAngle +
                                   m2RelativeJointAngle(world->solver.deltaRotations[c->bodyA],
                                                        world->solver.deltaRotations[c->bodyB]));
        float impulse;
        if (motorSpring)
        {
            // Soft drive: the spring biasRate replaces the hard
            // correctionFactor, with its mass and impulse scales.
            float angBias = c->softness.biasRate * angC;
            impulse = -c->softness.massScale * c->axialMass * ((wB - wA) + angBias) -
                      c->softness.impulseScale * c->motorImpulse;
        }
        else
        {
            float angBias = invH * c->motorSpeed * angC;
            impulse = -c->axialMass * ((wB - wA) + angBias);
        }
        float old = c->motorImpulse;
        c->motorImpulse = m2ClampF(old + impulse, -c->maxMotorImpulse, c->maxMotorImpulse);
        impulse = c->motorImpulse - old;
        wA -= iA * impulse;
        wB += iB * impulse;
    }
    {
        m2Vec2 sep = {c->baseCVec.x + ds.x, c->baseCVec.y + ds.y};
        float biasMul = motorSpring ? c->softness.biasRate : invH * c->motorSpeed;
        m2Vec2 bias = {biasMul * sep.x, biasMul * sep.y};
        m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
        m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
        m2Vec2 cdot = {vrB.x - vrA.x + bias.x, vrB.y - vrA.y + bias.y};
        // Solve K impulse = -cdot with the 2x2 from prepare.
        float det = c->k11 * c->k22 - c->k12 * c->k12;
        float invDet = det != 0.0f ? 1.0f / det : 0.0f;
        m2Vec2 raw = {-invDet * (c->k22 * cdot.x - c->k12 * cdot.y),
                      -invDet * (c->k11 * cdot.y - c->k12 * cdot.x)};
        m2Vec2 impulse;
        if (motorSpring)
        {
            impulse.x = c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x;
            impulse.y = c->softness.massScale * raw.y - c->softness.impulseScale * c->impulse.y;
        }
        else
        {
            impulse = raw;
        }
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
        vA.x -= mA * impulse.x;
        vA.y -= mA * impulse.y;
        wA -= iA * m2Cross2(c->rA, impulse);
        vB.x += mB * impulse.x;
        vB.y += mB * impulse.y;
        wB += iB * m2Cross2(c->rB, impulse);
    }
    if (world->bodies.types[c->bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyA] = vA;
        world->bodies.angularVelocities[c->bodyA] = wA;
    }
    if (world->bodies.types[c->bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyB] = vB;
        world->bodies.angularVelocities[c->bodyB] = wB;
    }
}

static void MotorReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // The linear block, and the pure torque in the motor slot.
    m2Vec2 impulse = world->joints.jointImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
    *torque = m2AbsF(world->joints.jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_motorJointKind = {PrepareMotor, m2WarmStartPointJoint, SolveMotor,
                                       MotorReaction};
