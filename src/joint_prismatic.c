// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The prismatic joint: a slider along an axis fixed in body A, with
// an optional motor and translation limits.

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

m2PrismaticJointDef m2DefaultPrismaticJointDef(void)
{
    m2PrismaticJointDef def;
    memset(&def, 0, sizeof(def));
    def.localAxisA = (m2Vec2){1.0f, 0.0f};
    def.internalValue = M2_PJOINT_COOKIE;
    return def;
}

m2JointId m2CreatePrismaticJoint(m2WorldId worldId, const m2PrismaticJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_PJOINT_COOKIE)
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
        m2FinishJoint(world, worldId, index, (uint8_t)m2_prismaticJoint, bodyA, bodyB,
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
    world->jointMaxMotor[index] = def->maxMotorForce;
    world->jointLower[index] = def->lowerTranslation;
    world->jointUpper[index] = def->upperTranslation;
    world->jointLocalAxisA[index] =
        (m2Vec2){def->localAxisA.x / axisLength, def->localAxisA.y / axisLength};
    world->jointRefAngle[index] =
        m2RelativeJointAngle(world->transforms[bodyA].q, world->transforms[bodyB].q);
    if (world->journalActive != 0)
    {
        m2OpCreatePrismaticJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreatePrismaticJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

// --- Solver ---------------------------------------------------------------

static void PreparePrismatic(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Rot qA = f->qA;
    m2Rot qB = f->qB;
    float dx = f->dx;
    float dy = f->dy;
    float mA = f->mA;
    float iA = f->iA;
    float mB = f->mB;
    float iB = f->iB;
    // Prismatic frame at prepare: Jacobians frozen for the
    // substep like the revolute point block (recorded
    // adaptation of the reference's per-iteration re-rotation).
    c->axis = m2RotateVec2(qA, world->jointLocalAxisA[j]);
    c->perp = (m2Vec2){-c->axis.y, c->axis.x};
    c->baseC = dx * c->axis.x + dy * c->axis.y; // translation0
    c->baseCVec = (m2Vec2){dx * c->perp.x + dy * c->perp.y, 0.0f};
    c->baseAngle = m2UnwindAngle(m2RelativeJointAngle(qA, qB) - world->jointRefAngle[j]);
    m2Vec2 dPlusRA = {dx + c->rA.x, dy + c->rA.y};
    c->a1 = m2Cross2(dPlusRA, c->axis);
    c->a2 = m2Cross2(c->rB, c->axis);
    c->s1 = m2Cross2(dPlusRA, c->perp);
    c->s2 = m2Cross2(c->rB, c->perp);
    float ka = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
    c->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;
    c->k11 = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
    c->k12 = iA * c->s1 + iB * c->s2;
    float k22 = iA + iB;
    c->k22 = k22 > 0.0f ? k22 : 1.0f; // fixed-rotation guard (reference)
}

static void WarmStartPrismatic(m2World* world, const m2JointConstraint* c)
{
    float axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
    m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                axial * c->axis.y + c->impulse.x * c->perp.y};
    float LA = axial * c->a1 + c->impulse.x * c->s1 + c->impulse.y;
    float LB = axial * c->a2 + c->impulse.x * c->s2 + c->impulse.y;
    m2ApplyJointArmImpulse(world, c, P, LA, LB);
}

static void SolvePrismatic(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vA = ctx->vA;
    float wA = ctx->wA;
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 ds = ctx->ds;
    bool useBias = ctx->useBias;
    float invH = ctx->invH;
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

    // Fresh effective mass per substep (reference b2 #981): the axial
    // and perpendicular torque arms a1/s1 track the current
    // separation, so recompute them and the effective masses here
    // instead of leaving them frozen at prepare. A stressed slider
    // whose geometry rotates within the step no longer solves against
    // a stale mass and diverges. cross is linear, so the fresh arm is
    // a1_prepare + cross(ds, axis); a2/s2 and the angle row do not
    // depend on the separation. The prepare-time values are swapped
    // back in at the end so the next substep recomputes from them.
    float a1Frozen = c->a1;
    float s1Frozen = c->s1;
    float axialMassFrozen = c->axialMass;
    float k11Frozen = c->k11;
    float k12Frozen = c->k12;
    c->a1 = a1Frozen + m2Cross2(ds, c->axis);
    c->s1 = s1Frozen + m2Cross2(ds, c->perp);
    float kaFresh = mA + mB + iA * c->a1 * c->a1 + iB * c->a2 * c->a2;
    c->axialMass = kaFresh > 0.0f ? 1.0f / kaFresh : 0.0f;
    c->k11 = mA + mB + iA * c->s1 * c->s1 + iB * c->s2 * c->s2;
    c->k12 = iA * c->s1 + iB * c->s2;

    if ((c->flags & M2_JOINT_MOTOR) != 0 && c->axialMass > 0.0f)
    {
        float cdot =
            c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
        float delta =
            m2SolveJointAxial(c, cdot - c->motorSpeed, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
        vA.x -= mA * delta * c->axis.x;
        vA.y -= mA * delta * c->axis.y;
        wA -= iA * delta * c->a1;
        vB.x += mB * delta * c->axis.x;
        vB.y += mB * delta * c->axis.y;
        wB += iB * delta * c->a2;
    }
    if ((c->flags & M2_JOINT_LIMIT) != 0 && c->axialMass > 0.0f)
    {
        { // lower translation limit
            float C = translation - c->lower;
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (C > 0.0f)
            {
                // Clamp the speculative distance to a safe span (b2
                // #981): a slider far inside its range cannot inject
                // a huge corrective velocity toward the limit.
                bias = m2MinF(C, 1.0f) * invH;
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
        { // upper limit: signs flipped, impulse stays positive
            float C = c->upper - translation;
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (C > 0.0f)
            {
                // Speculative distance clamped to a safe span (b2 #981).
                bias = m2MinF(C, 1.0f) * invH;
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
    { // perpendicular + angle lock, block form (reference)
        float cdotPerp =
            c->perp.x * (vB.x - vA.x) + c->perp.y * (vB.y - vA.y) + c->s2 * wB - c->s1 * wA;
        float cdotAngle = wB - wA;
        m2Vec2 bias = {0.0f, 0.0f};
        float massScale = 1.0f;
        float impulseScale = 0.0f;
        if (useBias)
        {
            float perpC = c->baseCVec.x + c->perp.x * ds.x + c->perp.y * ds.y;
            float angleC =
                m2UnwindAngle(c->baseAngle + m2RelativeJointAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
            bias.x = c->softness.biasRate * perpC;
            bias.y = c->softness.biasRate * angleC;
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        float det = c->k11 * c->k22 - c->k12 * c->k12;
        if (det > 0.0f)
        {
            float invDet = 1.0f / det;
            m2Vec2 b = {cdotPerp + bias.x, cdotAngle + bias.y};
            m2Vec2 impulse = {
                -massScale * invDet * (c->k22 * b.x - c->k12 * b.y) - impulseScale * c->impulse.x,
                -massScale * invDet * (c->k11 * b.y - c->k12 * b.x) - impulseScale * c->impulse.y};
            c->impulse.x += impulse.x;
            c->impulse.y += impulse.y;
            m2Vec2 P = {impulse.x * c->perp.x, impulse.x * c->perp.y};
            float LA = impulse.x * c->s1 + impulse.y;
            float LB = impulse.x * c->s2 + impulse.y;
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * LA;
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * LB;
        }
    }
    // Swap the prepare-time arms and masses back so the next substep
    // rebuilds the fresh values from them, not from this substep's.
    c->a1 = a1Frozen;
    c->s1 = s1Frozen;
    c->axialMass = axialMassFrozen;
    c->k11 = k11Frozen;
    c->k12 = k12Frozen;
    world->linearVelocities[c->bodyA] = vA;
    world->angularVelocities[c->bodyA] = wA;
    world->linearVelocities[c->bodyB] = vB;
    world->angularVelocities[c->bodyB] = wB;
}

static void PrismaticReaction(const m2World* world, int32_t j, float invH, float* force,
                              float* torque)
{
    // (perpendicular, angle) block plus the axial motor and limits.
    m2Vec2 impulse = world->jointImpulse[j];
    float axial = world->jointSpringImpulse[j] + world->jointMotorImpulse[j] +
                  world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    float linear = sqrtf(impulse.x * impulse.x + axial * axial);
    *force = linear * invH;
    *torque = m2AbsF(impulse.y) * invH;
}

const m2JointKind m2_prismaticJointKind = {PreparePrismatic, WarmStartPrismatic, SolvePrismatic,
                                           PrismaticReaction};
