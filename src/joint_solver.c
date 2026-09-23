// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver: preparation, warm starting, solving and impulse
// storage for every joint type.

#include "joint_solver.h"

#include "solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

// Relative angle of B vs A (own trig per ADR-0010).
static float RelativeRotAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2(sin, cos);
}

int32_t m2PrepareJoints(m2World* world, m2JointConstraint* joints, float h)
{
    // Stiff default softness for hertz==0 (F-T5-4 surface pending).
    int32_t count = 0;
    for (int32_t j = 0; j < world->maxJointIndex; ++j)
    {
        if (world->jointAlive[j] == 0)
        {
            continue;
        }
        if (world->jointType[j] == 5)
        {
            continue; // filter joints have no rows at all
        }
        if (world->jointType[j] == 7 &&
            (world->types[world->jointBodyB[j]] != (uint8_t)m2_dynamicBody ||
             world->asleep[world->jointBodyB[j]] != 0))
        {
            continue; // a mouse joint only ever moves body B
        }
        if (world->disabled[world->jointBodyA[j]] != 0 ||
            world->disabled[world->jointBodyB[j]] != 0)
        {
            continue; // a dormant end pauses the whole joint
        }
        int32_t bodyA = world->jointBodyA[j];
        int32_t bodyB = world->jointBodyB[j];
        if ((world->types[bodyA] != (uint8_t)m2_dynamicBody || world->asleep[bodyA] != 0) &&
            (world->types[bodyB] != (uint8_t)m2_dynamicBody || world->asleep[bodyB] != 0))
        {
            continue; // both ends inert this step
        }
        m2JointConstraint* c = joints + count;
        count += 1;
        c->jointIndex = j;
        c->bodyA = bodyA;
        c->bodyB = bodyB;
        c->type = world->jointType[j];
        c->flags = world->jointFlags[j];
        float hertz = world->jointHertz[j] > 0.0f ? world->jointHertz[j] : 60.0f;
        float damping = world->jointHertz[j] > 0.0f ? world->jointDamping[j] : 2.0f;
        c->softness = m2MakeSoft(hertz, damping, h);
        c->impulse = world->jointImpulse[j];
        c->motorSpeed = world->jointMotorSpeed[j];
        c->maxMotorImpulse = h * world->jointMaxMotor[j];
        c->lower = world->jointLower[j];
        c->upper = world->jointUpper[j];
        c->motorImpulse = world->jointMotorImpulse[j];
        c->lowerImpulse = world->jointLowerImpulse[j];
        c->upperImpulse = world->jointUpperImpulse[j];
        c->springImpulse = world->jointSpringImpulse[j];

        m2Rot qA = world->transforms[bodyA].q;
        m2Rot qB = world->transforms[bodyB].q;
        m2Vec2 lcA = world->localCenters[bodyA];
        m2Vec2 lcB = world->localCenters[bodyB];
        c->rA = m2RotateVec2(qA, (m2Vec2){world->jointLocalAnchorA[j].x - lcA.x,
                                          world->jointLocalAnchorA[j].y - lcA.y});
        c->rB = m2RotateVec2(qB, (m2Vec2){world->jointLocalAnchorB[j].x - lcB.x,
                                          world->jointLocalAnchorB[j].y - lcB.y});
        m2Vec2 comA = m2RotateVec2(qA, lcA);
        m2Vec2 comB = m2RotateVec2(qB, lcB);
        float dx = (float)(world->transforms[bodyB].p.x - world->transforms[bodyA].p.x) +
                   (comB.x - comA.x) + c->rB.x - c->rA.x;
        float dy = (float)(world->transforms[bodyB].p.y - world->transforms[bodyA].p.y) +
                   (comB.y - comA.y) + c->rB.y - c->rA.y;
        float mA = world->invMass[bodyA];
        float iA = world->invInertia[bodyA];
        float mB = world->invMass[bodyB];
        float iB = world->invInertia[bodyB];

        if (c->type == 0)
        {
            float length = sqrtf(dx * dx + dy * dy);
            c->axis = length > 1.19209290e-7f ? (m2Vec2){dx / length, dy / length}
                                              : (m2Vec2){0.0f, 1.0f}; // canonical fallback
            c->baseC = length - world->jointLength[j];
            // The rod's spawn-time length rides the (unused) motor
            // speed slot so the limit rows can track absolute length;
            // flag bit3 marks an active hard range.
            c->motorSpeed = length;
            if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
            {
                c->flags |= 8;
                // Hard stops stay hard even on a soft rope: the limit
                // rows run on the stiff default, reference-style.
                c->springSoftness = m2MakeSoft(60.0f, 2.0f, h);
            }
            if ((c->flags & 16u) != 0 && world->jointHertz[j] == 0.0f)
            {
                // enableSpring with zero stiffness is a free rope or rod:
                // skip the rest-length row so only the limits act, and
                // drop any rest impulse so warm start carries nothing.
                c->flags |= 32u;
                c->impulse.x = 0.0f;
            }
            float crA = m2Cross2(c->rA, c->axis);
            float crB = m2Cross2(c->rB, c->axis);
            float k = mA + mB + iA * crA * crA + iB * crB * crB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 1 || c->type == 3 || c->type == 6)
        {
            c->baseCVec = (m2Vec2){dx, dy};
            c->k11 = mA + mB + iA * c->rA.y * c->rA.y + iB * c->rB.y * c->rB.y;
            c->k12 = -iA * c->rA.x * c->rA.y - iB * c->rB.x * c->rB.y;
            c->k22 = mA + mB + iA * c->rA.x * c->rA.x + iB * c->rB.x * c->rB.x;
            if (c->type == 6)
            {
                // Motor: separations are measured against the offsets.
                // linearOffset lives in A's frame (the documented
                // contract; the reference's code disagrees with its own
                // comment and we side with the comment).
                m2Vec2 offset = m2RotateVec2(qA, world->jointLocalAxisA[j]);
                c->baseCVec = (m2Vec2){dx - offset.x, dy - offset.y};
                c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
                float ki = iA + iB;
                c->axialMass = ki > 0.0f ? 1.0f / ki : 0.0f;
                // correctionFactor rides the motorSpeed slot into the
                // solve; the linear budget rides lower.
                c->motorSpeed = world->jointDamping[j];
                c->lower = h * world->jointLength[j];
                if (world->jointHertz2[j] > 0.0f)
                {
                    // Spring drive: a soft softness replaces the hard
                    // correctionFactor bias in both rows (flag bit 32).
                    c->flags |= 32u;
                    c->softness = m2MakeSoft(world->jointHertz2[j], world->jointDamping2[j], h);
                }
            }
            float k = iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
            c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
            c->linearSpring = c->type == 3 && world->jointHertz[j] > 0.0f;
            c->angularSpring = (c->type == 1 || c->type == 3) && world->jointHertz2[j] > 0.0f;
            c->softness2 = c->angularSpring
                               ? m2MakeSoft(world->jointHertz2[j], world->jointDamping2[j], h)
                               : m2MakeSoft(60.0f, 2.0f, h);
        }
        else if (c->type == 8)
        {
            // Gear: accumulate the phase by how far each body actually
            // rotated since last prepare (per-step deltas stay far from
            // the wrap, so many full turns remain exact), then couple
            // the spins. ratio rides jointLength -> loaded fields.
            float ratio = world->jointLength[j];
            m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
            m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
            float phase = world->jointRefAngle[j];
            phase += ratio * RelativeRotAngle(prevA, qA) + RelativeRotAngle(prevB, qB);
            world->jointRefAngle[j] = phase;
            world->jointLocalAnchorA[j] = (m2Vec2){qA.c, qA.s};
            world->jointLocalAnchorB[j] = (m2Vec2){qB.c, qB.s};
            c->baseAngle = phase;
            c->motorSpeed = ratio; // carried into the solve
            float k = ratio * ratio * iA + iB;
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 10)
        {
            // Ratchet (Chipmunk cpRatchetJoint reconciliation): track
            // the relative angle multi-turn exact via previous-rotation
            // slots, click the engaged tooth forward when the angle
            // passes it, and hold a one-sided row against back-spin.
            float ratchet = world->jointLength[j];
            float phase = world->jointRefAngle[j];
            m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
            m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
            float angle = world->jointUpper[j];
            angle += RelativeRotAngle(prevB, qB) - RelativeRotAngle(prevA, qA);
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
        else if (c->type == 9)
        {
            // Pulley (b2 v2.4 reconciliation): equality constraint
            // C = total - lengthA - ratio * lengthB with unit rope
            // directions frozen at prepare. uA rides the axis slot,
            // uB the perp slot, ratio the motorSpeed slot. A side
            // shorter than 10 slop goes limp (zero direction), the
            // reference's slack guard.
            float ratio = world->jointLength[j];
            m2Vec2 armA = m2RotateVec2(qA, world->jointLocalAnchorA[j]);
            m2Vec2 armB = m2RotateVec2(qB, world->jointLocalAnchorB[j]);
            m2Vec2 uA = {(float)(world->transforms[bodyA].p.x - world->jointTargets[j].x) + armA.x,
                         (float)(world->transforms[bodyA].p.y - world->jointTargets[j].y) + armA.y};
            m2Vec2 uB = {(float)(world->transforms[bodyB].p.x - world->jointTargetsB[j].x) + armB.x,
                         (float)(world->transforms[bodyB].p.y - world->jointTargetsB[j].y) +
                             armB.y};
            float lengthA = sqrtf(uA.x * uA.x + uA.y * uA.y);
            float lengthB = sqrtf(uB.x * uB.x + uB.y * uB.y);
            c->axis =
                lengthA > 0.05f ? (m2Vec2){uA.x / lengthA, uA.y / lengthA} : (m2Vec2){0.0f, 0.0f};
            c->perp =
                lengthB > 0.05f ? (m2Vec2){uB.x / lengthB, uB.y / lengthB} : (m2Vec2){0.0f, 0.0f};
            c->baseC = world->jointRefAngle[j] - lengthA - ratio * lengthB;
            c->motorSpeed = ratio; // carried into the solve
            float ruA = m2Cross2(c->rA, c->axis);
            float ruB = m2Cross2(c->rB, c->perp);
            float k = mA + iA * ruA * ruA + ratio * ratio * (mB + iB * ruB * ruB);
            c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
        }
        else if (c->type == 7)
        {
            // Mouse: only body B has rows. Grab arm rB comes from the
            // generic anchors; the target separation base is the f64
            // crossing between B's center and the stored target.
            c->k11 = mB + iB * c->rB.y * c->rB.y;
            c->k12 = -iB * c->rB.x * c->rB.y;
            c->k22 = mB + iB * c->rB.x * c->rB.x;
            m2Vec2 comB2 = m2RotateVec2(qB, lcB);
            float cbx = (float)(world->transforms[bodyB].p.x - world->jointTargets[j].x);
            float cby = (float)(world->transforms[bodyB].p.y - world->jointTargets[j].y);
            c->baseCVec = (m2Vec2){cbx + comB2.x, cby + comB2.y};
            c->softness2 = m2MakeSoft(0.5f, 0.1f, h); // reference spin damper
            c->lower = h * world->jointLength[j];     // force budget
        }
        else if (c->type == 2)
        {
            // Prismatic frame at prepare: Jacobians frozen for the
            // substep like the revolute point block (recorded
            // adaptation of the reference's per-iteration re-rotation).
            c->axis = m2RotateVec2(qA, world->jointLocalAxisA[j]);
            c->perp = (m2Vec2){-c->axis.y, c->axis.x};
            c->baseC = dx * c->axis.x + dy * c->axis.y; // translation0
            c->baseCVec = (m2Vec2){dx * c->perp.x + dy * c->perp.y, 0.0f};
            c->baseAngle = m2UnwindAngle(RelativeRotAngle(qA, qB) - world->jointRefAngle[j]);
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
        else
        {
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
    }
    return count;
}

static void ApplyJointImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P)
{
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    world->linearVelocities[c->bodyA].x -= mA * P.x;
    world->linearVelocities[c->bodyA].y -= mA * P.y;
    world->angularVelocities[c->bodyA] -= iA * m2Cross2(c->rA, P);
    world->linearVelocities[c->bodyB].x += mB * P.x;
    world->linearVelocities[c->bodyB].y += mB * P.y;
    world->angularVelocities[c->bodyB] += iB * m2Cross2(c->rB, P);
}

// Prismatic impulses act along frozen Jacobians, not through rA/rB.
static void ApplyPrismaticImpulse(m2World* world, const m2JointConstraint* c, m2Vec2 P, float LA,
                                  float LB)
{
    float mA = world->invMass[c->bodyA];
    float iA = world->invInertia[c->bodyA];
    float mB = world->invMass[c->bodyB];
    float iB = world->invInertia[c->bodyB];
    world->linearVelocities[c->bodyA].x -= mA * P.x;
    world->linearVelocities[c->bodyA].y -= mA * P.y;
    world->angularVelocities[c->bodyA] -= iA * LA;
    world->linearVelocities[c->bodyB].x += mB * P.x;
    world->linearVelocities[c->bodyB].y += mB * P.y;
    world->angularVelocities[c->bodyB] += iB * LB;
}

void m2WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        if (c->type == 0)
        {
            float axial = (c->flags & 8) != 0 ? c->impulse.x + c->lowerImpulse - c->upperImpulse
                                              : c->impulse.x;
            ApplyJointImpulse(world, c, (m2Vec2){axial * c->axis.x, axial * c->axis.y});
        }
        else if (c->type == 1 || c->type == 3 || c->type == 6)
        {
            // Weld's angle-lock and the motor joint's torque both ride
            // the motor slot; limits are zero where unused. The
            // revolute angular spring joins the axial sum.
            ApplyJointImpulse(world, c, c->impulse);
            float axial = c->springImpulse + c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * axial;
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * axial;
        }
        else if (c->type == 8)
        {
            // Gear: one angular impulse, ratio-weighted on side A.
            float L = c->impulse.x;
            world->angularVelocities[c->bodyA] += world->invInertia[c->bodyA] * (c->motorSpeed * L);
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
        }
        else if (c->type == 10)
        {
            // Ratchet: one-sided angular impulse in the hold direction.
            float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
            float L = s * c->impulse.x;
            world->angularVelocities[c->bodyA] -= world->invInertia[c->bodyA] * L;
            world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
        }
        else if (c->type == 9)
        {
            // Pulley: PA = -L*uA on A, PB = -ratio*L*uB on B.
            float L = c->impulse.x;
            m2Vec2 PA = {-L * c->axis.x, -L * c->axis.y};
            m2Vec2 PB = {-c->motorSpeed * L * c->perp.x, -c->motorSpeed * L * c->perp.y};
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            world->linearVelocities[c->bodyA].x += mA * PA.x;
            world->linearVelocities[c->bodyA].y += mA * PA.y;
            world->angularVelocities[c->bodyA] += iA * m2Cross2(c->rA, PA);
            world->linearVelocities[c->bodyB].x += mB * PB.x;
            world->linearVelocities[c->bodyB].y += mB * PB.y;
            world->angularVelocities[c->bodyB] += iB * m2Cross2(c->rB, PB);
        }
        else if (c->type == 7)
        {
            // Mouse: body B only.
            int32_t bodyB = c->bodyB;
            float mB = world->invMass[bodyB];
            float iB = world->invInertia[bodyB];
            world->linearVelocities[bodyB].x += mB * c->impulse.x;
            world->linearVelocities[bodyB].y += mB * c->impulse.y;
            world->angularVelocities[bodyB] += iB * (m2Cross2(c->rB, c->impulse) + c->motorImpulse);
        }
        else if (c->type == 2)
        {
            float axial = c->motorImpulse + c->lowerImpulse - c->upperImpulse;
            m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                        axial * c->axis.y + c->impulse.x * c->perp.y};
            float LA = axial * c->a1 + c->impulse.x * c->s1 + c->impulse.y;
            float LB = axial * c->a2 + c->impulse.x * c->s2 + c->impulse.y;
            ApplyPrismaticImpulse(world, c, P, LA, LB);
        }
        else
        {
            // Wheel: spring rides impulse.y, the motor is pure angular.
            float axial = c->impulse.y + c->lowerImpulse - c->upperImpulse;
            m2Vec2 P = {axial * c->axis.x + c->impulse.x * c->perp.x,
                        axial * c->axis.y + c->impulse.x * c->perp.y};
            float LA = axial * c->a1 + c->impulse.x * c->s1 + c->motorImpulse;
            float LB = axial * c->a2 + c->impulse.x * c->s2 + c->motorImpulse;
            ApplyPrismaticImpulse(world, c, P, LA, LB);
        }
    }
}

// One axial sub-constraint (motor or limit) on the shared axial
// Jacobian; oneSided clamps the accumulator at zero (limits), otherwise
// it clamps symmetrically at the motor budget. Returns the delta.
static float SolveAxial(m2JointConstraint* c, float cdot, float bias, float massScale,
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

void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool useBias,
                   float invH)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        m2Vec2 vA = world->linearVelocities[c->bodyA];
        float wA = world->angularVelocities[c->bodyA];
        m2Vec2 vB = world->linearVelocities[c->bodyB];
        float wB = world->angularVelocities[c->bodyB];
        m2Vec2 dp = {world->deltaPositions[c->bodyB].x - world->deltaPositions[c->bodyA].x,
                     world->deltaPositions[c->bodyB].y - world->deltaPositions[c->bodyA].y};
        m2Vec2 drB = m2RotateVec2(world->deltaRotations[c->bodyB], c->rB);
        m2Vec2 drA = m2RotateVec2(world->deltaRotations[c->bodyA], c->rA);
        m2Vec2 ds = {dp.x + (drB.x - c->rB.x) - (drA.x - c->rA.x),
                     dp.y + (drB.y - c->rB.y) - (drA.y - c->rA.y)};

        if (c->type == 0)
        {
            if ((c->flags & 32u) == 0)
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
                float impulse =
                    -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
                c->impulse.x += impulse;
                ApplyJointImpulse(world, c, (m2Vec2){impulse * c->axis.x, impulse * c->axis.y});
            }

            if ((c->flags & 8) != 0)
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
                    ApplyJointImpulse(world, c, (m2Vec2){imp * c->axis.x, imp * c->axis.y});
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
                    ApplyJointImpulse(world, c, (m2Vec2){-imp * c->axis.x, -imp * c->axis.y});
                }
            }
        }
        else if (c->type == 1 || c->type == 3)
        {
            if (c->type == 3 && c->axialMass > 0.0f)
            {
                // Weld angle lock (reference weld_joint.c): a full
                // constraint row on relative rotation, accumulator in
                // the motor slot.
                float bias = 0.0f;
                float massScale = 1.0f;
                float impulseScale = 0.0f;
                // Nonzero angular hertz = a real spring: biased even
                // during relax (reference semantics).
                if (useBias || c->angularSpring)
                {
                    float C = m2UnwindAngle(c->baseAngle +
                                            RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                             world->deltaRotations[c->bodyB]));
                    bias = c->softness2.biasRate * C;
                    massScale = c->softness2.massScale;
                    impulseScale = c->softness2.impulseScale;
                }
                float cdot = wB - wA;
                float impulse =
                    -massScale * c->axialMass * (cdot + bias) - impulseScale * c->motorImpulse;
                c->motorImpulse += impulse;
                wA -= world->invInertia[c->bodyA] * impulse;
                wB += world->invInertia[c->bodyB] * impulse;
            }
            if (c->type == 1 && c->angularSpring && c->axialMass > 0.0f)
            {
                // Revolute angular spring (reference revolute_joint.c):
                // a real spring toward the creation angle, biased in
                // every pass by definition.
                float C =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
                float bias = c->softness2.biasRate * C;
                float massScale = c->softness2.massScale;
                float impulseScale = c->softness2.impulseScale;
                float cdot = wB - wA;
                float impulse =
                    -massScale * c->axialMass * (cdot + bias) - impulseScale * c->springImpulse;
                c->springImpulse += impulse;
                wA -= world->invInertia[c->bodyA] * impulse;
                wB += world->invInertia[c->bodyB] * impulse;
            }
            if (c->type == 1 && (c->flags & 1u) != 0 && c->axialMass > 0.0f)
            {
                // Motor: drive relative spin toward motorSpeed within
                // the per-step torque budget (reference formulation).
                float cdot = wB - wA - c->motorSpeed;
                float delta = SolveAxial(c, cdot, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
                wA -= world->invInertia[c->bodyA] * delta;
                wB += world->invInertia[c->bodyB] * delta;
            }
            if (c->type == 1 && (c->flags & 2u) != 0 && c->axialMass > 0.0f)
            {
                float jointAngle =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
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
                    float delta = SolveAxial(c, wB - wA, bias, massScale, impulseScale,
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
                    float delta = SolveAxial(c, wA - wB, bias, massScale, impulseScale,
                                             &c->upperImpulse, true);
                    wA += world->invInertia[c->bodyA] * delta;
                    wB -= world->invInertia[c->bodyB] * delta;
                }
            }
            world->angularVelocities[c->bodyA] = wA;
            world->angularVelocities[c->bodyB] = wB;

            m2Vec2 bias = {0.0f, 0.0f};
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias || (c->type == 3 && c->linearSpring))
            {
                m2Vec2 C = {c->baseCVec.x + ds.x, c->baseCVec.y + ds.y};
                bias.x = c->softness.massScale * c->softness.biasRate * C.x;
                bias.y = c->softness.massScale * c->softness.biasRate * C.y;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m2Vec2 vrA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
            m2Vec2 vrB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
            m2Vec2 cdot = {vrB.x - vrA.x, vrB.y - vrA.y};
            m2Vec2 b = {massScale * cdot.x + bias.x, massScale * cdot.y + bias.y};
            // Solve K * impulse = -b (2x2, guarded determinant: row-skip law).
            float det = c->k11 * c->k22 - c->k12 * c->k12;
            if (det <= 0.0f)
            {
                continue;
            }
            float invDet = 1.0f / det;
            m2Vec2 impulse = {-invDet * (c->k22 * b.x - c->k12 * b.y) - impulseScale * c->impulse.x,
                              -invDet * (c->k11 * b.y - c->k12 * b.x) -
                                  impulseScale * c->impulse.y};
            c->impulse.x += impulse.x;
            c->impulse.y += impulse.y;
            ApplyJointImpulse(world, c, impulse);
        }
        else if (c->type == 6)
        {
            // Motor joint (reference solve, always biased): angular row
            // first, then the clamped linear block. correctionFactor
            // rides the motorSpeed slot, the force budget rides lower.
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            bool motorSpring = (c->flags & 32u) != 0;
            {
                float angC =
                    m2UnwindAngle(c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
                                                                  world->deltaRotations[c->bodyB]));
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
                    impulse.x =
                        c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x;
                    impulse.y =
                        c->softness.massScale * raw.y - c->softness.impulseScale * c->impulse.y;
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
            if (world->types[c->bodyA] == (uint8_t)m2_dynamicBody)
            {
                world->linearVelocities[c->bodyA] = vA;
                world->angularVelocities[c->bodyA] = wA;
            }
            if (world->types[c->bodyB] == (uint8_t)m2_dynamicBody)
            {
                world->linearVelocities[c->bodyB] = vB;
                world->angularVelocities[c->bodyB] = wB;
            }
        }
        else if (c->type == 8)
        {
            // Gear: C = ratio*angleA + angleB - phase0, tracked through
            // the substep delta rotations; stiff-biased like the weld
            // angle row.
            float ratio = c->motorSpeed;
            float iA = world->invInertia[c->bodyA];
            float iB = world->invInertia[c->bodyB];
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                float dA =
                    m2Atan2(world->deltaRotations[c->bodyA].s, world->deltaRotations[c->bodyA].c);
                float dB =
                    m2Atan2(world->deltaRotations[c->bodyB].s, world->deltaRotations[c->bodyB].c);
                float C = c->baseAngle + ratio * dA + dB;
                bias = c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            float cdot = ratio * wA + wB;
            float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
            c->impulse.x += impulse;
            world->angularVelocities[c->bodyA] = wA + iA * (ratio * impulse);
            world->angularVelocities[c->bodyB] = wB + iB * impulse;
        }
        else if (c->type == 10)
        {
            // Ratchet: the revolute limit row, sign-folded so the free
            // direction never feels it: C' = s*(angle - engaged) >= 0,
            // speculative when open, stiff-soft when violated.
            float s = c->motorSpeed > 0.0f ? 1.0f : -1.0f;
            float iA = world->invInertia[c->bodyA];
            float iB = world->invInertia[c->bodyB];
            float angleNow = c->baseAngle + RelativeRotAngle(world->deltaRotations[c->bodyA],
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
        else if (c->type == 9)
        {
            // Pulley: Cdot = -dot(uA, vpA) - ratio * dot(uB, vpB),
            // stiff-biased with C tracked through per-body position
            // deltas against the frozen rope directions.
            float ratio = c->motorSpeed;
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float bias = 0.0f;
            float massScale = 1.0f;
            float impulseScale = 0.0f;
            if (useBias)
            {
                m2Vec2 dsA = {world->deltaPositions[c->bodyA].x + (drA.x - c->rA.x),
                              world->deltaPositions[c->bodyA].y + (drA.y - c->rA.y)};
                m2Vec2 dsB = {world->deltaPositions[c->bodyB].x + (drB.x - c->rB.x),
                              world->deltaPositions[c->bodyB].y + (drB.y - c->rB.y)};
                float C = c->baseC - (dsA.x * c->axis.x + dsA.y * c->axis.y) -
                          ratio * (dsB.x * c->perp.x + dsB.y * c->perp.y);
                bias = c->softness.massScale * c->softness.biasRate * C;
                massScale = c->softness.massScale;
                impulseScale = c->softness.impulseScale;
            }
            m2Vec2 vpA = {vA.x - wA * c->rA.y, vA.y + wA * c->rA.x};
            m2Vec2 vpB = {vB.x - wB * c->rB.y, vB.y + wB * c->rB.x};
            float cdot = -(vpA.x * c->axis.x + vpA.y * c->axis.y) -
                         ratio * (vpB.x * c->perp.x + vpB.y * c->perp.y);
            float impulse = -c->axialMass * (massScale * cdot + bias) - impulseScale * c->impulse.x;
            c->impulse.x += impulse;
            m2Vec2 PA = {-impulse * c->axis.x, -impulse * c->axis.y};
            m2Vec2 PB = {-ratio * impulse * c->perp.x, -ratio * impulse * c->perp.y};
            world->linearVelocities[c->bodyA].x = vA.x + mA * PA.x;
            world->linearVelocities[c->bodyA].y = vA.y + mA * PA.y;
            world->angularVelocities[c->bodyA] = wA + iA * m2Cross2(c->rA, PA);
            world->linearVelocities[c->bodyB].x = vB.x + mB * PB.x;
            world->linearVelocities[c->bodyB].y = vB.y + mB * PB.y;
            world->angularVelocities[c->bodyB] = wB + iB * m2Cross2(c->rB, PB);
        }
        else if (c->type == 7)
        {
            // Mouse joint (reference solve, always biased): a soft spin
            // damper, then the soft pull toward the target, clamped to
            // the force budget in lower.
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            {
                float impulse = iB > 0.0f ? -wB / iB : 0.0f;
                impulse =
                    c->softness2.massScale * impulse - c->softness2.impulseScale * c->motorImpulse;
                c->motorImpulse += impulse;
                wB += iB * impulse;
            }
            {
                m2Vec2 sep = {c->baseCVec.x + world->deltaPositions[c->bodyB].x + drB.x,
                              c->baseCVec.y + world->deltaPositions[c->bodyB].y + drB.y};
                m2Vec2 bias = {c->softness.biasRate * sep.x, c->softness.biasRate * sep.y};
                m2Vec2 cdot = {vB.x - wB * drB.y + bias.x, vB.y + wB * drB.x + bias.y};
                float det = c->k11 * c->k22 - c->k12 * c->k12;
                float invDet = det != 0.0f ? 1.0f / det : 0.0f;
                m2Vec2 raw = {invDet * (c->k22 * cdot.x - c->k12 * cdot.y),
                              invDet * (c->k11 * cdot.y - c->k12 * cdot.x)};
                m2Vec2 impulse = {
                    -c->softness.massScale * raw.x - c->softness.impulseScale * c->impulse.x,
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
            world->linearVelocities[c->bodyB] = vB;
            world->angularVelocities[c->bodyB] = wB;
        }
        else if (c->type == 2)
        {
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

            if ((c->flags & 1u) != 0 && c->axialMass > 0.0f)
            {
                float cdot =
                    c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) + c->a2 * wB - c->a1 * wA;
                float delta =
                    SolveAxial(c, cdot - c->motorSpeed, 0.0f, 1.0f, 0.0f, &c->motorImpulse, false);
                vA.x -= mA * delta * c->axis.x;
                vA.y -= mA * delta * c->axis.y;
                wA -= iA * delta * c->a1;
                vB.x += mB * delta * c->axis.x;
                vB.y += mB * delta * c->axis.y;
                wB += iB * delta * c->a2;
            }
            if ((c->flags & 2u) != 0 && c->axialMass > 0.0f)
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
                    float cdot = c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) +
                                 c->a2 * wB - c->a1 * wA;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->lowerImpulse, true);
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
                    float cdot = c->axis.x * (vA.x - vB.x) + c->axis.y * (vA.y - vB.y) +
                                 c->a1 * wA - c->a2 * wB;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->upperImpulse, true);
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
                    float angleC = m2UnwindAngle(c->baseAngle +
                                                 RelativeRotAngle(world->deltaRotations[c->bodyA],
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
                    m2Vec2 impulse = {-massScale * invDet * (c->k22 * b.x - c->k12 * b.y) -
                                          impulseScale * c->impulse.x,
                                      -massScale * invDet * (c->k11 * b.y - c->k12 * b.x) -
                                          impulseScale * c->impulse.y};
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
        else
        {
            // Wheel (reference wheel_joint.c): rotational motor, real
            // suspension spring (biased even in relax), translation
            // limits, single perpendicular row. Rotation stays free.
            float mA = world->invMass[c->bodyA];
            float iA = world->invInertia[c->bodyA];
            float mB = world->invMass[c->bodyB];
            float iB = world->invInertia[c->bodyB];
            float translation = c->baseC + c->axis.x * ds.x + c->axis.y * ds.y;

            if ((c->flags & 1u) != 0 && c->k22 > 0.0f)
            {
                float cdot = wB - wA - c->motorSpeed;
                float unclamped = c->motorImpulse - c->k22 * cdot;
                float clamped = m2ClampF(unclamped, -c->maxMotorImpulse, c->maxMotorImpulse);
                float delta = clamped - c->motorImpulse;
                c->motorImpulse = clamped;
                wA -= iA * delta;
                wB += iB * delta;
            }
            if ((c->flags & 4u) != 0 && c->axialMass > 0.0f)
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
            if ((c->flags & 2u) != 0 && c->axialMass > 0.0f)
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
                    float cdot = c->axis.x * (vB.x - vA.x) + c->axis.y * (vB.y - vA.y) +
                                 c->a2 * wB - c->a1 * wA;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->lowerImpulse, true);
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
                    float cdot = c->axis.x * (vA.x - vB.x) + c->axis.y * (vA.y - vB.y) +
                                 c->a1 * wA - c->a2 * wB;
                    float delta =
                        SolveAxial(c, cdot, bias, massScale, impulseScale, &c->upperImpulse, true);
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
    }
}

void m2StoreJointImpulses(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t j = joints[i].jointIndex;
        world->jointImpulse[j] = joints[i].impulse;
        world->jointMotorImpulse[j] = joints[i].motorImpulse;
        world->jointLowerImpulse[j] = joints[i].lowerImpulse;
        world->jointUpperImpulse[j] = joints[i].upperImpulse;
        world->jointSpringImpulse[j] = joints[i].springImpulse;
    }
}

void m2JointReactionMagnitudes(const m2World* world, int32_t j, float invH, float* force,
                               float* torque)
{
    m2Vec2 impulse = world->jointImpulse[j];
    float axialTrio = world->jointSpringImpulse[j] + world->jointMotorImpulse[j] +
                      world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
    *force = 0.0f;
    *torque = 0.0f;
    switch (world->jointType[j])
    {
    case 0: // distance: axial row, plus range rows when bounded
    {
        float axial = impulse.x;
        if (world->jointLower[j] > 0.0f || world->jointUpper[j] < 3.0e38f)
        {
            axial += world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
        }
        *force = m2AbsF(axial) * invH;
        break;
    }
    case 1: // revolute: point block linear, motor/limits as torque
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(axialTrio) * invH;
        break;
    case 2: // prismatic: (perp, angle) + axial trio
    {
        float linear = sqrtf(impulse.x * impulse.x + axialTrio * axialTrio);
        *force = linear * invH;
        *torque = m2AbsF(impulse.y) * invH;
        break;
    }
    case 3: // weld: point block linear, angle row in the motor slot
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    case 5: // filter: no rows, no loads, by definition
        break;
    case 6: // motor: linear block + pure torque in the motor slot
    case 7: // mouse: same accumulator shape, body B only
        *force = sqrtf(impulse.x * impulse.x + impulse.y * impulse.y) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    case 8: // gear: pure torque coupling
        *torque = m2AbsF(impulse.x) * invH;
        break;
    case 9: // pulley: A-side rope tension (B side feels ratio times this)
        *force = m2AbsF(impulse.x) * invH;
        break;
    case 10: // ratchet: pure holding torque
        *torque = m2AbsF(impulse.x) * invH;
        break;
    default: // wheel: (perp, spring) linear, motor as torque
    {
        float axial = impulse.y + world->jointLowerImpulse[j] - world->jointUpperImpulse[j];
        *force = sqrtf(impulse.x * impulse.x + axial * axial) * invH;
        *torque = m2AbsF(world->jointMotorImpulse[j]) * invH;
        break;
    }
    }
}
