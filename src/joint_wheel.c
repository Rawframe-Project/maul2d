// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wheel joint: a slider with a suspension spring and free
// rotation, driven by a rotational motor.

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

m2WheelJointDef m2DefaultWheelJointDef(void)
{
    m2WheelJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){0.0f, 1.0f};
    def.enableSpring = true;
    def.hertz = 2.0f;
    def.dampingRatio = 0.7f;
    def.internalValue = M2_WHJOINT_COOKIE;
    return def;
}

m2JointId m2CreateWheelJoint(m2WorldId worldId, const m2WheelJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_WHJOINT_COOKIE)
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
    float axisLength =
        sqrtf(def->localAxisA.x * def->localAxisA.x + def->localAxisA.y * def->localAxisA.y);
    if (!(axisLength > 1.19209290e-7f))
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
        m2FinishJoint(world, worldId, index, (uint8_t)m2_wheelJoint, bodyA, bodyB,
                      def->localAnchorA, def->localAnchorB, 0.0f, def->hertz, def->dampingRatio);
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    world->jointFlags[index] = (def->enableMotor ? M2_JOINT_MOTOR : 0u) |
                               (def->enableLimit ? M2_JOINT_LIMIT : 0u) |
                               (def->enableSpring ? M2_JOINT_SPRING : 0u);
    world->jointMotorSpeed[index] = def->motorSpeed;
    world->jointMaxMotor[index] = def->maxMotorTorque;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    if (world->journalActive != 0)
    {
        m2OpCreateWheelJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateWheelJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------

static void PrepareWheel(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float h = f->h;
    m2Rot qA = f->qA;
    float dx = f->dx;
    float dy = f->dy;
    float mA = f->mA;
    float iA = f->iA;
    float mB = f->mB;
    float iB = f->iB;
    // Wheel: slider frame like the prismatic, but rotation is
    // free. The user's hertz shapes the suspension spring; the
    // constraint rows stay on the stiff default.
    c->springSoftness = c->softness;
    c->softness = m2MakeSoft(60.0f, 2.0f, h);
    c->axis = m2RotateVec2(qA, world->jointLocalAxisA[j]);
    c->perp = (m2Vec2){-c->axis.y, c->axis.x};
    c->baseC = dx * c->axis.x + dy * c->axis.y; // translation0
    c->baseCVec = (m2Vec2){dx * c->perp.x + dy * c->perp.y, 0.0f};
    m2Vec2 dPlusRA = {dx + c->rA.x, dy + c->rA.y};
    c->a1 = m2Cross2(dPlusRA, c->axis);
    c->a2 = m2Cross2(c->rB, c->axis);
    c->s1 = m2Cross2(dPlusRA, c->perp);
    c->s2 = m2Cross2(c->rB, c->perp);
    float ka = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
    c->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;
    float kp = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
    c->k11 = kp > 0.0f ? 1.0f / kp : 0.0f; // perp effective mass
    float km = iA + iB;
    c->k22 = km > 0.0f ? 1.0f / km : 0.0f; // motor effective mass
}

static void WarmStartWheel(m2World* world, const m2JointConstraint* c)
{
    // Wheel: spring rides impulse.y, the motor is pure angular.
    float axial = c->impulse.y + c->lowerImpulse - c->upperImpulse;
    m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                axial * c->axis.y + c->impulse.x * c->perp.y};
    float LA = axial * c->a1 + c->impulse.x * c->s1 + c->motorImpulse;
    float LB = axial * c->a2 + c->impulse.x * c->s2 + c->motorImpulse;
    m2ApplyJointArmImpulse(world, c, P, LA, LB);
}

static void SolveWheel(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vA = ctx->vA;
    float wA = ctx->wA;
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 ds = ctx->ds;
    bool useBias = ctx->useBias;
    float invH = ctx->invH;
    // Wheel (reference wheel_joint.c): rotational motor, real
    // suspension spring (biased even in relax), translation
    // limits, single perpendicular row. Rotation stays free.
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

    if ((c->flags & M2_JOINT_MOTOR) != 0 && c->k22 > 0.0f)
    {
        float cdot = wB - wA - c->motorSpeed;
        float unclamped = c->motorImpulse - c->k22 * cdot;
        float clamped = m2ClampF(unclamped, -c->maxMotorImpulse, c->maxMotorImpulse);
        float delta = clamped - c->motorImpulse;
        c->motorImpulse = clamped;
        wA -= iA * delta;
        wB += iB * delta;
    }
    if ((c->flags & M2_JOINT_SPRING) != 0 && c->axialMass > 0.0f)
    {
        float C = translation;
        float bias = c->springSoftness.biasRate * C;
        float cdot =
            c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
        float impulse = -c->springSoftness.massScale * c->axialMass * (cdot + bias) -
                        c->springSoftness.impulseScale * c->impulse.y;
        c->impulse.y += impulse;
        vA.x -= mA * impulse * c->axis.x;
        vA.y -= mA * impulse * c->axis.y;
        wA -= iA * impulse * c->a1;
        vB.x += mB * impulse * c->axis.x;
        vB.y += mB * impulse * c->axis.y;
        wB += iB * impulse * c->a2;
    }
    if ((c->flags & M2_JOINT_LIMIT) != 0 && c->axialMass > 0.0f)
    {
        { // lower travel stop
            float C = translation - c->lower;
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
            float cdot =
                c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
            float delta =
                m2SolveJointAxial(c, cdot, bias, massScale, impulseScale, &c->lowerImpulse, true);
            vA.x -= mA * delta * c->axis.x;
            vA.y -= mA * delta * c->axis.y;
            wA -= iA * delta * c->a1;
            vB.x += mB * delta * c->axis.x;
            vB.y += mB * delta * c->axis.y;
            wB += iB * delta * c->a2;
        }
        { // upper travel stop, signs flipped
            float C = c->upper - translation;
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
            float cdot =
                c->axis.x * (vA.x - vB.x) + c->axis.y * (vA.y - vB.y) + c->a1 * wA - c->a2 * wB;
            float delta =
                m2SolveJointAxial(c, cdot, bias, massScale, impulseScale, &c->upperImpulse, true);
            vA.x += mA * delta * c->axis.x;
            vA.y += mA * delta * c->axis.y;
            wA += iA * delta * c->a1;
            vB.x -= mB * delta * c->axis.x;
            vB.y -= mB * delta * c->axis.y;
            wB -= iB * delta * c->a2;
        }
    }
    if (c->k11 > 0.0f)
    { // point-to-line row
        float cdot =
            c->perp.x * (vB.x - vA.x) + c->perp.y * (vB.y - vA.y) + c->s2 * wB - c->s1 * wA;
        float bias = 0.0f;
        float massScale = 1.0f;
        float impulseScale = 0.0f;
        if (useBias)
        {
            float C = c->baseCVec.x + c->perp.x * ds.x + c->perp.y * ds.y;
            bias = c->softness.biasRate * C;
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        float impulse = -massScale * c->k11 * (cdot + bias) - impulseScale * c->impulse.x;
        c->impulse.x += impulse;
        vA.x -= mA * impulse * c->perp.x;
        vA.y -= mA * impulse * c->perp.y;
        wA -= iA * impulse * c->s1;
        vB.x += mB * impulse * c->perp.x;
        vB.y += mB * impulse * c->perp.y;
        wB += iB * impulse * c->s2;
    }
    world->linearVelocities[c->bodyA] = vA;
    world->angularVelocities[c->bodyA] = wA;
    world->linearVelocities[c->bodyB] = vB;
    world->angularVelocities[c->bodyB] = wB;
}

static void WheelReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // (perpendicular, spring) linear rows; the motor as torque.
    m2Vec2 impulse = world->jointImpulse[j];
    float axial = impulse.y + world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    *force = sqrtf(impulse.x * impulse.x + axial * axial) * invH;
    *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
}

const m2JointKind m2_wheelJointKind = {PrepareWheel, WarmStartWheel, SolveWheel, WheelReaction};
