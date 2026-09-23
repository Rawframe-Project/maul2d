// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The scalar contact solver: preparation, warm starting, solving,
// restitution and the serial and colored stage runners.

#include "contact_solver.h"

#include "solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

// The whole solver scratch lives on world arrays sized at creation; the
// constraint list is rebuilt every step (step-transient, snapshot-benign).

int32_t m2ContactConstraintSize(void)
{
    // Joint constraints ride in the tail of the same scratch block.
    return (int32_t)sizeof(m2ContactConstraint) + (int32_t)sizeof(m2JointConstraint);
}

int32_t m2PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h)
{
    // Stiffer for static contacts, exactly like the reference: a soft
    // ground row is an energy reservoir under a tall stack.
    m2Softness soft = m2MakeSoft(M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
    m2Softness staticSoft = m2MakeSoft(2.0f * M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h);
    int32_t count = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        m2Manifold* manifold = &world->contacts.manifolds[i];
        if (manifold->pointCount == 0)
        {
            continue;
        }
        int32_t shapeA = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (world->shapes.shapeSensor[shapeA] != 0 || world->shapes.shapeSensor[shapeB] != 0)
        {
            continue; // sensors observe, never push
        }
        int32_t bodyA = world->shapes.shapeBody[shapeA];
        int32_t bodyB = world->shapes.shapeBody[shapeB];
        float mA = world->bodies.invMass[bodyA];
        float iA = world->bodies.invInertia[bodyA];
        float mB = world->bodies.invMass[bodyB];
        float iB = world->bodies.invInertia[bodyB];
        // Dominance (contacts only): the higher side is unmovable in
        // this pair; statics outrank every dynamic by construction.
        int32_t domA = world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody
                           ? (int32_t)world->bodies.dominances[bodyA]
                           : 128;
        int32_t domB = world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody
                           ? (int32_t)world->bodies.dominances[bodyB]
                           : 128;
        if (domA > domB)
        {
            mA = 0.0f;
            iA = 0.0f;
        }
        else if (domB > domA)
        {
            mB = 0.0f;
            iB = 0.0f;
        }
        if (mA + mB == 0.0f && iA + iB == 0.0f)
        {
            continue; // both non-dynamic
        }
        bool asleepA = world->bodies.types[bodyA] != (uint8_t)m2_dynamicBody ||
                       world->bodies.asleep[bodyA] != 0;
        bool asleepB = world->bodies.types[bodyB] != (uint8_t)m2_dynamicBody ||
                       world->bodies.asleep[bodyB] != 0;
        if (asleepA && asleepB)
        {
            continue; // frozen contact inside a sleeping island
        }

        m2ContactConstraint* c = constraints + count;
        count += 1;
        c->pairIndex = i;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->invMassA = mA;
        c->invIA = iA;
        c->invMassB = mB;
        c->invIB = iB;
        // Geometric-mean friction, max restitution (reference mixing).
        c->friction =
            sqrtf(world->shapes.shapeFriction[shapeA] * world->shapes.shapeFriction[shapeB]);
        c->tangentSpeed =
            world->shapes.shapeTangentSpeed[shapeA] + world->shapes.shapeTangentSpeed[shapeB];
        float restA = world->shapes.shapeRestitution[shapeA];
        float restB = world->shapes.shapeRestitution[shapeB];
        c->restitution = m2MaxF(restA, restB);
        c->softness = world->bodies.types[bodyA] != (uint8_t)m2_dynamicBody ||
                              world->bodies.types[bodyB] != (uint8_t)m2_dynamicBody
                          ? staticSoft
                          : soft;
        c->pointCount = manifold->pointCount;

        m2Rot qA = world->bodies.transforms[bodyA].q;
        m2Rot qB = world->bodies.transforms[bodyB].q;
        c->normal = m2RotateVec2(qA, manifold->normal);
        m2Vec2 tangent = {-c->normal.y, c->normal.x};

        m2Vec2 vA = world->bodies.linearVelocities[bodyA];
        float wA = world->bodies.angularVelocities[bodyA];
        m2Vec2 vB = world->bodies.linearVelocities[bodyB];
        float wB = world->bodies.angularVelocities[bodyB];

        for (int32_t k = 0; k < manifold->pointCount; ++k)
        {
            m2ManifoldPoint* mp = &manifold->points[k];
            m2ConstraintPoint* cp = &c->points[k];
            // Anchors relative to each body's center of mass: the arm
            // the impulse actually torques about (bit-neutral when the
            // COM sits on the origin).
            m2Vec2 lcA = world->bodies.localCenters[bodyA];
            m2Vec2 lcB = world->bodies.localCenters[bodyB];
            cp->rA = m2RotateVec2(qA, (m2Vec2){mp->anchorA.x - lcA.x, mp->anchorA.y - lcA.y});
            cp->rB = m2RotateVec2(qB, (m2Vec2){mp->anchorB.x - lcB.x, mp->anchorB.y - lcB.y});
            // Reference factoring: fold the prepare-time anchor gap in,
            // then track the ABSOLUTE rotated gap during solve - the
            // incremental (rs - r0) form cancels catastrophically at
            // small rotations and feeds the bias noise.
            cp->baseSeparation = mp->separation - ((cp->rB.x - cp->rA.x) * c->normal.x +
                                                   (cp->rB.y - cp->rA.y) * c->normal.y);
            cp->normalImpulse = mp->normalImpulse;
            cp->tangentImpulse = mp->tangentImpulse;
            cp->id = mp->id;
            cp->persisted = (uint16_t)(mp->flags & 1);

            float kNormal = mA + mB +
                            iA * m2Cross2(cp->rA, c->normal) * m2Cross2(cp->rA, c->normal) +
                            iB * m2Cross2(cp->rB, c->normal) * m2Cross2(cp->rB, c->normal);
            cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f; // row-skip law
            float kTangent = mA + mB + iA * m2Cross2(cp->rA, tangent) * m2Cross2(cp->rA, tangent) +
                             iB * m2Cross2(cp->rB, tangent) * m2Cross2(cp->rB, tangent);
            cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            cp->relativeVelocity = (vrB.x - vrA.x) * c->normal.x + (vrB.y - vrA.y) * c->normal.y;
        }
    }
    return count;
}

void m2WarmStartOne(m2World* world, m2ContactConstraint* c)
{
    float mA = c->invMassA;
    float iA = c->invIA;
    float mB = c->invMassB;
    float iB = c->invIB;
    m2Vec2 vA = world->bodies.linearVelocities[c->bodyA];
    float wA = world->bodies.angularVelocities[c->bodyA];
    m2Vec2 vB = world->bodies.linearVelocities[c->bodyB];
    float wB = world->bodies.angularVelocities[c->bodyB];
    m2Vec2 tangent = {-c->normal.y, c->normal.x};
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        m2ConstraintPoint* cp = &c->points[k];
        m2Vec2 P = {cp->normalImpulse * c->normal.x + cp->tangentImpulse * tangent.x,
                    cp->normalImpulse * c->normal.y + cp->tangentImpulse * tangent.y};
        vA.x -= mA * P.x;
        vA.y -= mA * P.y;
        wA -= iA * m2Cross2(cp->rA, P);
        vB.x += mB * P.x;
        vB.y += mB * P.y;
        wB += iB * m2Cross2(cp->rB, P);
    }
    m2StoreBodyVelocities(world, c, vA, wA, vB, wB);
}

void m2SolveContactOne(m2World* world, m2ContactConstraint* c, float invH, float minBiasVel,
                       bool useBias)
{
    {
        float mA = c->invMassA;
        float iA = c->invIA;
        float mB = c->invMassB;
        float iB = c->invIB;
        m2Vec2 vA = world->bodies.linearVelocities[c->bodyA];
        float wA = world->bodies.angularVelocities[c->bodyA];
        m2Vec2 vB = world->bodies.linearVelocities[c->bodyB];
        float wB = world->bodies.angularVelocities[c->bodyB];
        m2Vec2 normal = c->normal;
        m2Vec2 tangent = {-normal.y, normal.x};

        // Separation drift from accumulated deltas (never fresh
        // world-space math inside the step).
        m2Vec2 dp = {
            world->solver.deltaPositions[c->bodyB].x - world->solver.deltaPositions[c->bodyA].x,
            world->solver.deltaPositions[c->bodyB].y - world->solver.deltaPositions[c->bodyA].y};

        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m2ConstraintPoint* cp = &c->points[k];
            // Reference discipline: FIXED prepare-time anchors for the
            // Jacobian and the applied torque; anchors re-rotated by
            // the substep deltas only measure the current separation.
            m2Vec2 rsA = m2RotateVec2(world->solver.deltaRotations[c->bodyA], cp->rA);
            m2Vec2 rsB = m2RotateVec2(world->solver.deltaRotations[c->bodyB], cp->rB);
            m2Vec2 ds = {dp.x + rsB.x - rsA.x, dp.y + rsB.y - rsA.y};
            float s = cp->baseSeparation + ds.x * normal.x + ds.y * normal.y;

            // Reference bias selection: the push clamp lands BEFORE the
            // mass scale, and the whole (vn + bias) is scaled together.
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (s > 0.0f)
            {
                bias = s * invH; // speculative: prevent crossing
            }
            else if (useBias)
            {
                bias = m2MaxF(c->softness.biasRate * s, minBiasVel);
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }

            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            float vn = (vrB.x - vrA.x) * normal.x + (vrB.y - vrA.y) * normal.y;

            float impulse =
                -cp->normalMass * massScale * (vn + bias) - impulseScale * cp->normalImpulse;
            float newImpulse = m2MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;

            m2Vec2 P = {impulse * normal.x, impulse * normal.y};
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * m2Cross2(cp->rA, P);
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * m2Cross2(cp->rB, P);
        }

        {
            // Friction solves in BOTH passes, like the reference. This
            // was unstable for twenty slices - because the scrambled
            // pair order was silently dropping warm-start carries and
            // amplifying bias contamination in the accumulators. With
            // the sort fixed, this schedule settles the pyramid benchmark
            // fastest of the orders tried.
            for (int32_t k = 0; k < c->pointCount; ++k)
            {
                m2ConstraintPoint* cp = &c->points[k];
                m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
                m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
                // Our tangent is the LEFT perp of the normal (the
                // reference uses the right perp), so the belt term
                // ADDS: positive tangentSpeed drives riders toward
                // +x on an upward-facing floor, the reference's
                // observable convention.
                float vt =
                    (vrB.x - vrA.x) * tangent.x + (vrB.y - vrA.y) * tangent.y + c->tangentSpeed;
                float impulse = -cp->tangentMass * vt;
                float maxFriction = c->friction * cp->normalImpulse;
                float newImpulse =
                    m2ClampF(cp->tangentImpulse + impulse, -maxFriction, maxFriction);
                impulse = newImpulse - cp->tangentImpulse;
                cp->tangentImpulse = newImpulse;

                m2Vec2 P = {impulse * tangent.x, impulse * tangent.y};
                vA.x -= mA * P.x;
                vA.y -= mA * P.y;
                wA -= iA * m2Cross2(cp->rA, P);
                vB.x += mB * P.x;
                vB.y += mB * P.y;
                wB += iB * m2Cross2(cp->rB, P);
            }
        }

        m2StoreBodyVelocities(world, c, vA, wA, vB, wB);
    }
}

void m2RestitutionOne(m2World* world, m2ContactConstraint* c)
{
    {
        if (c->restitution == 0.0f)
        {
            return;
        }
        float mA = c->invMassA;
        float iA = c->invIA;
        float mB = c->invMassB;
        float iB = c->invIB;
        m2Vec2 vA = world->bodies.linearVelocities[c->bodyA];
        float wA = world->bodies.angularVelocities[c->bodyA];
        m2Vec2 vB = world->bodies.linearVelocities[c->bodyB];
        float wB = world->bodies.angularVelocities[c->bodyB];
        for (int32_t k = 0; k < c->pointCount; ++k)
        {
            m2ConstraintPoint* cp = &c->points[k];
            // Only points that arrived fast and actually carried load.
            if (cp->relativeVelocity > -M2_RESTITUTION_THRESHOLD || cp->normalImpulse == 0.0f)
            {
                continue;
            }
            m2Vec2 vrA = {vA.x - wA * cp->rA.y, vA.y + wA * cp->rA.x};
            m2Vec2 vrB = {vB.x - wB * cp->rB.y, vB.y + wB * cp->rB.x};
            float vn = (vrB.x - vrA.x) * c->normal.x + (vrB.y - vrA.y) * c->normal.y;
            float impulse = -cp->normalMass * (vn + c->restitution * cp->relativeVelocity);
            float newImpulse = m2MaxF(cp->normalImpulse + impulse, 0.0f);
            impulse = newImpulse - cp->normalImpulse;
            cp->normalImpulse = newImpulse;
            m2Vec2 P = {impulse * c->normal.x, impulse * c->normal.y};
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * m2Cross2(cp->rA, P);
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * m2Cross2(cp->rB, P);
        }
        m2StoreBodyVelocities(world, c, vA, wA, vB, wB);
    }
}

void m2ContactStageRange(int32_t begin, int32_t end, void* userCtx)
{
    m2ContactStageCtx* ctx = (m2ContactStageCtx*)userCtx;
    for (int32_t k = begin; k < end; ++k)
    {
        m2ContactConstraint* c = ctx->constraints + ctx->order[k];
        switch (ctx->stage)
        {
        case m2_stageWarmStart:
            m2WarmStartOne(ctx->world, c);
            break;
        case m2_stageSolve:
            m2SolveContactOne(ctx->world, c, ctx->invH, ctx->minBiasVel, ctx->useBias);
            break;
        case m2_stageRestitution:
            m2RestitutionOne(ctx->world, c);
            break;
        default:
        {
            m2Manifold* manifold = &ctx->world->contacts.manifolds[c->pairIndex];
            for (int32_t j = 0; j < c->pointCount; ++j)
            {
                manifold->points[j].normalImpulse = c->points[j].normalImpulse;
                manifold->points[j].tangentImpulse = c->points[j].tangentImpulse;
            }
            break;
        }
        }
    }
}
