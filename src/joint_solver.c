// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's stages: the setup every joint shares, dispatch to
// the kind table (one joint_<kind>.c per type), impulse storage, and the
// helpers the kinds share.

#include "joint_solver.h"

#include "joint.h"
#include "solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

static const m2JointKind* const s_kinds[] = {
    [m2_distanceJoint] = &m2_distanceJointKind,   [m2_revoluteJoint] = &m2_revoluteJointKind,
    [m2_prismaticJoint] = &m2_prismaticJointKind, [m2_weldJoint] = &m2_weldJointKind,
    [m2_wheelJoint] = &m2_wheelJointKind,         [m2_filterJoint] = &m2_filterJointKind,
    [m2_motorJoint] = &m2_motorJointKind,         [m2_mouseJoint] = &m2_mouseJointKind,
    [m2_gearJoint] = &m2_gearJointKind,           [m2_pulleyJoint] = &m2_pulleyJointKind,
    [m2_ratchetJoint] = &m2_ratchetJointKind,
};

int32_t m2PrepareJoints(m2World* world, m2JointConstraint* joints, float h)
{
    // Stiff default softness for hertz==0 (F-T5-4 surface pending).
    int32_t count = 0;
    for (int32_t j = 0; j < world->joints.maxJointIndex; ++j)
    {
        if (world->joints.jointAlive[j] == 0)
        {
            continue;
        }
        const m2JointKind* kind = s_kinds[world->joints.jointType[j]];
        if (kind->prepare == NULL)
        {
            continue; // filter joints have no rows at all
        }
        if (world->joints.jointType[j] == (uint8_t)m2_mouseJoint &&
            (world->bodies.types[world->joints.jointBodyB[j]] != (uint8_t)m2_dynamicBody ||
             world->bodies.asleep[world->joints.jointBodyB[j]] != 0))
        {
            continue; // a mouse joint only ever moves body B
        }
        if (world->bodies.disabled[world->joints.jointBodyA[j]] != 0 ||
            world->bodies.disabled[world->joints.jointBodyB[j]] != 0)
        {
            continue; // a dormant end pauses the whole joint
        }
        int32_t bodyA = world->joints.jointBodyA[j];
        int32_t bodyB = world->joints.jointBodyB[j];
        if ((world->bodies.types[bodyA] != (uint8_t)m2_dynamicBody ||
             world->bodies.asleep[bodyA] != 0) &&
            (world->bodies.types[bodyB] != (uint8_t)m2_dynamicBody ||
             world->bodies.asleep[bodyB] != 0))
        {
            continue; // both ends inert this step
        }
        m2JointConstraint* c = joints + count;
        count += 1;
        c->jointIndex = j;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->type = world->joints.jointType[j];
        c->flags = world->joints.jointFlags[j];
        float hertz = world->joints.jointHertz[j] > 0.0f ? world->joints.jointHertz[j] : 60.0f;
        float damping = world->joints.jointHertz[j] > 0.0f ? world->joints.jointDamping[j] : 2.0f;
        c->softness = m2MakeSoft(hertz, damping, h);
        c->impulse = world->joints.jointImpulse[j];
        c->motorSpeed = world->joints.jointMotorSpeed[j];
        c->maxMotorImpulse = h * world->joints.jointMaxMotor[j];
        c->lower = world->joints.jointLower[j];
        c->upper = world->joints.jointUpper[j];
        c->motorImpulse = world->joints.jointMotorImpulse[j];
        c->lowerImpulse = world->joints.jointLowerImpulse[j];
        c->upperImpulse = world->joints.jointUpperImpulse[j];
        c->springImpulse = world->joints.jointSpringImpulse[j];

        m2Rot qA = world->bodies.transforms[bodyA].q;
        m2Rot qB = world->bodies.transforms[bodyB].q;
        m2Vec2 lcA = world->bodies.localCenters[bodyA];
        m2Vec2 lcB = world->bodies.localCenters[bodyB];
        c->rA = m2RotateVec2(qA, (m2Vec2){world->joints.jointLocalAnchorA[j].x - lcA.x,
                                          world->joints.jointLocalAnchorA[j].y - lcA.y});
        c->rB = m2RotateVec2(qB, (m2Vec2){world->joints.jointLocalAnchorB[j].x - lcB.x,
                                          world->joints.jointLocalAnchorB[j].y - lcB.y});
        m2Vec2 comA = m2RotateVec2(qA, lcA);
        m2Vec2 comB = m2RotateVec2(qB, lcB);
        float dx =
            (float)(world->bodies.transforms[bodyB].p.x - world->bodies.transforms[bodyA].p.x) +
            (comB.x - comA.x) + c->rB.x - c->rA.x;
        float dy =
            (float)(world->bodies.transforms[bodyB].p.y - world->bodies.transforms[bodyA].p.y) +
            (comB.y - comA.y) + c->rB.y - c->rA.y;
        float mA = world->bodies.invMass[bodyA];
        float iA = world->bodies.invInertia[bodyA];
        float mB = world->bodies.invMass[bodyB];
        float iB = world->bodies.invInertia[bodyB];

        m2JointFrame frame = {j, h, qA, qB, lcA, lcB, dx, dy, mA, iA, mB, iB};
        kind->prepare(world, c, &frame);
    }
    return count;
}

void m2WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        s_kinds[c->type]->warmStart(world, c);
    }
}

void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool useBias,
                   float invH)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        m2JointSolveContext ctx;
        ctx.vA = world->bodies.linearVelocities[c->bodyA];
        ctx.wA = world->bodies.angularVelocities[c->bodyA];
        ctx.vB = world->bodies.linearVelocities[c->bodyB];
        ctx.wB = world->bodies.angularVelocities[c->bodyB];
        m2Vec2 dp = {
            world->solver.deltaPositions[c->bodyB].x - world->solver.deltaPositions[c->bodyA].x,
            world->solver.deltaPositions[c->bodyB].y - world->solver.deltaPositions[c->bodyA].y};
        ctx.drB = m2RotateVec2(world->solver.deltaRotations[c->bodyB], c->rB);
        ctx.drA = m2RotateVec2(world->solver.deltaRotations[c->bodyA], c->rA);
        ctx.ds = (m2Vec2){dp.x + (ctx.drB.x - c->rB.x) - (ctx.drA.x - c->rA.x),
                          dp.y + (ctx.drB.y - c->rB.y) - (ctx.drA.y - c->rA.y)};
        ctx.useBias = useBias;
        ctx.invH = invH;
        s_kinds[c->type]->solve(world, c, &ctx);
    }
}

void m2StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t j = joints[i].jointIndex;
        world->joints.jointImpulse[j] = joints[i].impulse;
        world->joints.jointMotorImpulse[j] = joints[i].motorImpulse;
        world->joints.jointLowerImpulse[j] = joints[i].lowerImpulse;
        world->joints.jointUpperImpulse[j] = joints[i].upperImpulse;
        world->joints.jointSpringImpulse[j] = joints[i].springImpulse;
    }
}

void m2JointReactionMagnitudes(const m2World* world, int32_t j, float invH, float* force,
                               float* torque)
{
    *force = 0.0f;
    *torque = 0.0f;
    const m2JointKind* kind = s_kinds[world->joints.jointType[j]];
    if (kind->reaction != NULL)
    {
        kind->reaction(world, j, invH, force, torque);
    }
}

// --- Shared by the kinds -----------------------------------------------------

void m2ApplyJointImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P)
{
    float mA = world->bodies.invMass[c->bodyA];
    float iA = world->bodies.invInertia[c->bodyA];
    float mB = world->bodies.invMass[c->bodyB];
    float iB = world->bodies.invInertia[c->bodyB];
    world->bodies.linearVelocities[c->bodyA].x -= mA * P.x;
    world->bodies.linearVelocities[c->bodyA].y -= mA * P.y;
    world->bodies.angularVelocities[c->bodyA] -= iA * m2Cross2(c->rA, P);
    world->bodies.linearVelocities[c->bodyB].x += mB * P.x;
    world->bodies.linearVelocities[c->bodyB].y += mB * P.y;
    world->bodies.angularVelocities[c->bodyB] += iB * m2Cross2(c->rB, P);
}

// Slider impulses act along Jacobians frozen at prepare, not through rA
// and rB: LA and LB are the angular parts.
void m2ApplyJointArmImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P, float LA,
                            float LB)
{
    float mA = world->bodies.invMass[c->bodyA];
    float iA = world->bodies.invInertia[c->bodyA];
    float mB = world->bodies.invMass[c->bodyB];
    float iB = world->bodies.invInertia[c->bodyB];
    world->bodies.linearVelocities[c->bodyA].x -= mA * P.x;
    world->bodies.linearVelocities[c->bodyA].y -= mA * P.y;
    world->bodies.angularVelocities[c->bodyA] -= iA * LA;
    world->bodies.linearVelocities[c->bodyB].x += mB * P.x;
    world->bodies.linearVelocities[c->bodyB].y += mB * P.y;
    world->bodies.angularVelocities[c->bodyB] += iB * LB;
}

// One axial sub-constraint (motor or limit) on the shared axial
// Jacobian; oneSided clamps the accumulator at zero (limits), otherwise
// it clamps symmetrically at the motor budget. Returns the delta.
float m2SolveJointAxial(m2JointConstraint* c, float cdot, float bias, float massScale,
                        float impulseScale, float* accumulated, bool oneSided)
{
    float old = *accumulated;
    float impulse = -massScale * c->axialMass * (cdot + bias) - impulseScale * old;
    float next = old + impulse;
    if (oneSided)
    {
        next = m2MaxF(next, 0.0f);
    }
    else
    {
        next = m2ClampF(next, -c->maxMotorImpulse, c->maxMotorImpulse);
    }
    *accumulated = next;
    return next - old;
}

// The pinned-point 2x2 block shared by the revolute, weld and motor
// joints: separation at prepare and its effective mass.
void m2PreparePointBlock(m2JointConstraint* c, const m2JointFrame* f)
{
    float mA = f->mA;
    float iA = f->iA;
    float mB = f->mB;
    float iB = f->iB;
    c->baseCVec = (m2Vec2){f->dx, f->dy};
    c->k11 = mA + mB + iA * c->rA.y * c->rA.y + iB * c->rB.y * c->rB.y;
    c->k12 = -iA * c->rA.x * c->rA.y - iB * c->rB.x * c->rB.y;
    c->k22 = mA + mB + iA * c->rA.x * c->rA.x + iB * c->rB.x * c->rB.x;
}

// Warm start for the point joints: the point block through the arms,
// and every angular accumulator (weld's angle lock and the motor
// joint's torque ride the motor slot; unused ones are zero).
void m2WarmStartPointJoint(m2World* world, const m2JointConstraint* c)
{
    m2ApplyJointImpulse(world, c, c->impulse);
    float axial = c->springImpulse + c->motorImpulse + c->lowerImpulse - c->upperImpulse;
    world->bodies.angularVelocities[c->bodyA] -= world->bodies.invInertia[c->bodyA] * axial;
    world->bodies.angularVelocities[c->bodyB] += world->bodies.invInertia[c->bodyB] * axial;
}

// The revolute and weld point block, after their angular rows: stores
// the angular velocities those rows produced, then solves the 2x2 with
// a guarded determinant (a singular block skips the row).
void m2SolvePointBlock(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx,
                       float wA, float wB, bool biased)
{
    world->bodies.angularVelocities[c->bodyA] = wA;
    world->bodies.angularVelocities[c->bodyB] = wB;
    m2Vec2 vA = ctx->vA;
    m2Vec2 vB = ctx->vB;
    m2Vec2 bias = {0.0f, 0.0f};
    float massScale = 1.0f;
    float impulseScale = 0.0f;
    if (biased)
    {
        m2Vec2 C = {c->baseCVec.x + ctx->ds.x, c->baseCVec.y + ctx->ds.y};
        bias.x = c->softness.massScale * c->softness.biasRate * C.x;
        bias.y = c->softness.massScale * c->softness.biasRate * C.y;
        massScale = c->softness.massScale;
        impulseScale = c->softness.impulseScale;
    }
    m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
    m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
    m2Vec2 cdot = {vrB.x - vrA.x, vrB.y - vrA.y};
    m2Vec2 b = {massScale * cdot.x + bias.x, massScale * cdot.y + bias.y};
    float det = c->k11 * c->k22 - c->k12 * c->k12;
    if (det <= 0.0f)
    {
        return;
    }
    float invDet = 1.0f / det;
    m2Vec2 impulse = {-invDet * (c->k22 * b.x - c->k12 * b.y) - impulseScale * c->impulse.x,
                      -invDet * (c->k11 * b.y - c->k12 * b.x) - impulseScale * c->impulse.y};
    c->impulse.x += impulse.x;
    c->impulse.y += impulse.y;
    m2ApplyJointImpulse(world, c, impulse);
}
