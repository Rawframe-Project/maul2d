// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The distance joint: a rest-length row, optionally soft, with an
// optional hard length range.

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

m2DistanceJointDef m2DefaultDistanceJointDef(void)
{
    m2DistanceJointDef def;
    memset(&def, 0, sizeof(def));
    def.internalValue = M2_DJOINT_COOKIE;
    return def;
}

// The def contract: finite values, non-negative gains and budgets,
// ordered ranges.
static bool DefValid(const m2DistanceJointDef* def)
{
    return m2FiniteVec2(def->localAnchorA) && m2FiniteVec2(def->localAnchorB) &&
           m2FiniteF(def->length) && m2FiniteF(def->minLength) && m2FiniteF(def->maxLength) &&
           !(def->maxLength > 0.0f && def->minLength > def->maxLength) && m2JointGain(def->hertz) &&
           m2JointGain(def->dampingRatio);
}

m2JointId m2CreateDistanceJoint(m2WorldId worldId, const m2DistanceJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_DJOINT_COOKIE || !DefValid(def))
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
    float length = def->length;
    if (!(length > 0.0f))
    {
        // Derive from spawn poses: the single f64 crossing.
        m2Transform xfA = world->transforms[bodyA];
        m2Transform xfB = world->transforms[bodyB];
        m2Vec2 wA = {xfA.q.c * def->localAnchorA.x - xfA.q.s * def->localAnchorA.y,
                     xfA.q.s * def->localAnchorA.x + xfA.q.c * def->localAnchorA.y};
        m2Vec2 wB = {xfB.q.c * def->localAnchorB.x - xfB.q.s * def->localAnchorB.y,
                     xfB.q.s * def->localAnchorB.x + xfB.q.c * def->localAnchorB.y};
        float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
        float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
        length = sqrtf(dx * dx + dy * dy);
    }
    m2JointId jointId =
        m2FinishJoint(world, worldId, index, (uint8_t)m2_distanceJoint, bodyA, bodyB,
                      def->localAnchorA, def->localAnchorB, length, def->hertz, def->dampingRatio);
    // The hard range: off by default (0 .. huge); a def maxLength <= 0
    // means unbounded, mirroring "length <= 0 derives".
    world->jointLower[index] = def->minLength > 0.0f ? def->minLength : 0.0f;
    world->jointUpper[index] = def->maxLength > 0.0f ? def->maxLength : 3.4e38f;
    if (def->enableSpring)
    {
        world->jointFlags[index] |= M2_JOINT_ROPE; // rope/rod: gate the rest-length row
    }
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateDistanceJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateDistanceJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2DistanceJoint_SetLength(m2JointId jointId, float length)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamLength, length);
}

void m2DistanceJoint_SetLengthRange(m2JointId jointId, float minLength, float maxLength)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    if (!m2FiniteF(minLength) || !m2FiniteF(maxLength))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Reference clamps: slop floor, ordered pair.
    float lo = minLength > 0.005f ? minLength : 0.005f;
    float hi = maxLength > 0.005f ? maxLength : 0.005f;
    float lower = lo < hi ? lo : hi;
    float upper = lo < hi ? hi : lo;
    if (m2SetJointParamInternal(world, jointId, m2_jointParamMinLength, lower))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMaxLength, upper);
    }
}

// --- Solver ---------------------------------------------------------------

static void PrepareDistance(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    float h = f->h;
    float dx = f->dx;
    float dy = f->dy;
    float mA = f->mA;
    float iA = f->iA;
    float mB = f->mB;
    float iB = f->iB;
    float length = sqrtf(dx * dx + dy * dy);
    c->axis = length > 1.19209290e-7f ? (m2Vec2){dx / length, dy / length}
                                      : (m2Vec2){0.0f, 1.0f}; // canonical fallback
    c->baseC = length - world->jointLength[j];
    // The rod's spawn-time length rides the (unused) motor
    // speed slot so the limit rows can track absolute length;
    // M2_JOINT_HARD_RANGE marks an active range.
    c->motorSpeed = length;
    if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
    {
        c->flags |= M2_JOINT_HARD_RANGE;
        // Hard stops stay hard even on a soft rope: the limit
        // rows run on the stiff default, reference-style.
        c->springSoftness = m2MakeSoft(60.0f, 2.0f, h);
    }
    if ((c->flags & M2_JOINT_ROPE) != 0 && world->jointHertz[j] == 0.0f)
    {
        // enableSpring with zero stiffness is a free rope or rod:
        // skip the rest-length row so only the limits act, and
        // drop any rest impulse so warm start carries nothing.
        c->flags |= M2_JOINT_FREE_LENGTH;
        c->impulse.x = 0.0f;
    }
    float crA = m2Cross2(c->rA, c->axis);
    float crB = m2Cross2(c->rB, c->axis);
    float k = mA + mB + iA * crA * crA + iB * crB * crB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
}

static void WarmStartDistance(m2World* world, const m2JointConstraint* c)
{
    float axial = (c->flags & M2_JOINT_HARD_RANGE) != 0
                      ? c->impulse.x + c->lowerImpulse - c->upperImpulse
                      : c->impulse.x;
    m2ApplyJointImpulse(world, c, (m2Vec2){axial * c->axis.x, axial * c->axis.y});
}

static void SolveDistance(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    m2Vec2 vA = ctx->vA;
    float wA = ctx->wA;
    m2Vec2 vB = ctx->vB;
    float wB = ctx->wB;
    m2Vec2 ds = ctx->ds;
    bool useBias = ctx->useBias;
    float invH = ctx->invH;
    if ((c->flags & M2_JOINT_FREE_LENGTH) == 0)
    {
        // Rest-length row (skipped for a free rope or rod).
        float bias = 0.0f;
        float massScale = 1.0f;
        float impulseScale = 0.0f;
        if (useBias)
        {
            float C = c->baseC + ds.x * c->axis.x + ds.y * c->axis.y;
            bias = c->softness.massScale * c->softness.biasRate * C;
            massScale = c->softness.massScale;
            impulseScale = c->softness.impulseScale;
        }
        m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
        m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
        float cdot = (vrB.x - vrA.x) * c->axis.x + (vrB.y - vrA.y) * c->axis.y;
        float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
        c->impulse.x += impulse;
        m2ApplyJointImpulse(world, c, (m2Vec2){impulse * c->axis.x, impulse * c->axis.y});
    }

    if ((c->flags & M2_JOINT_HARD_RANGE) != 0)
    {
        // Hard length range (reference distance_joint.c): one
        // one-sided row per bound on the shared axis, reading
        // fresh world velocities after each apply.
        float lengthNow = c->motorSpeed + ds.x * c->axis.x + ds.y * c->axis.y;
        {
            m2Vec2 vA2 = world->linearVelocities[c->bodyA];
            float wA2 = world->angularVelocities[c->bodyA];
            m2Vec2 vB2 = world->linearVelocities[c->bodyB];
            float wB2 = world->angularVelocities[c->bodyB];
            m2Vec2 vrA2 = {vA2.x - wA2 * c->rA.y, vA2.y + wA2 * c->rA.x};
            m2Vec2 vrB2 = {vB2.x - wB2 * c->rB.y, vB2.y + wB2 * c->rB.x};
            float cdotL = (vrB2.x - vrA2.x) * c->axis.x + (vrB2.y - vrA2.y) * c->axis.y;
            float C = lengthNow - c->lower;
            float biasL = 0.0f;
            float massL = 1.0f;
            float scaleL = 0.0f;
            if (C > 0.0f)
            {
                biasL = C * invH; // speculative
            }
            else if (useBias)
            {
                biasL = c->springSoftness.biasRate * C;
                massL = c->springSoftness.massScale;
                scaleL = c->springSoftness.impulseScale;
            }
            float imp = -massL * c->axialMass * (cdotL + biasL) - scaleL * c->lowerImpulse;
            float next = c->lowerImpulse + imp;
            next = next > 0.0f ? next : 0.0f;
            imp = next - c->lowerImpulse;
            c->lowerImpulse = next;
            m2ApplyJointImpulse(world, c, (m2Vec2){imp * c->axis.x, imp * c->axis.y});
        }
        {
            m2Vec2 vA2 = world->linearVelocities[c->bodyA];
            float wA2 = world->angularVelocities[c->bodyA];
            m2Vec2 vB2 = world->linearVelocities[c->bodyB];
            float wB2 = world->angularVelocities[c->bodyB];
            m2Vec2 vrA2 = {vA2.x - wA2 * c->rA.y, vA2.y + wA2 * c->rA.x};
            m2Vec2 vrB2 = {vB2.x - wB2 * c->rB.y, vB2.y + wB2 * c->rB.x};
            // Upper bound: the constraint runs the other way.
            float cdotU = (vrA2.x - vrB2.x) * c->axis.x + (vrA2.y - vrB2.y) * c->axis.y;
            float C = c->upper - lengthNow;
            float biasU = 0.0f;
            float massU = 1.0f;
            float scaleU = 0.0f;
            if (C > 0.0f)
            {
                biasU = C * invH;
            }
            else if (useBias)
            {
                biasU = c->springSoftness.biasRate * C;
                massU = c->springSoftness.massScale;
                scaleU = c->springSoftness.impulseScale;
            }
            float imp = -massU * c->axialMass * (cdotU + biasU) - scaleU * c->upperImpulse;
            float next = c->upperImpulse + imp;
            next = next > 0.0f ? next : 0.0f;
            imp = next - c->upperImpulse;
            c->upperImpulse = next;
            m2ApplyJointImpulse(world, c, (m2Vec2){-imp * c->axis.x, -imp * c->axis.y});
        }
    }
}

static void DistanceReaction(const m2World* world, int32_t j, float invH, float* force,
                             float* torque)
{
    // Axial row, plus the range rows when bounded.
    float axial = world->jointImpulse[j].x;
    if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
    {
        axial += world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    }
    *force = m2AbsF(axial) * invH;
    *torque = 0.0f;
}

const m2JointKind m2_distanceJointKind = {PrepareDistance, WarmStartDistance, SolveDistance,
                                          DistanceReaction};
