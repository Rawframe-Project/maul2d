// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The gear joint: couples the spins of two bodies by a ratio.

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

m2GearJointDef m2DefaultGearJointDef(void)
{
    m2GearJointDef def;
    memset(&def, 0, sizeof(def));
    def.ratio = 1.0f;
    def.internalValue = M2_GJOINT_COOKIE;
    return def;
}

// Gear registry mapping: ratio rides jointLength; the two previous
// body rotations ride the anchor slots as (c, s) pairs so the phase
// accumulator in prepare survives any number of full turns; the
// accumulated phase itself rides jointRefAngle. All snapshot state.
m2JointId m2CreateGearJoint(m2WorldId worldId, const m2GearJointDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_GJOINT_COOKIE ||
        !(def->ratio != 0.0f))
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
    m2JointId jointId = m2FinishJoint(world, worldId, index, (uint8_t)m2_gearJoint, bodyA, bodyB,
                                      zero, zero, 0.0f, 0.0f, 0.0f);
    world->jointLength[index] = def->ratio;
    m2Rot qA = world->transforms[bodyA].q;
    m2Rot qB = world->transforms[bodyB].q;
    world->jointLocalAnchorA[index] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[index] = (m2Vec2){qB.c, qB.s};
    world->jointRefAngle[index] = 0.0f; // in phase by definition at birth
    world->jointUserData[index] = def->userData;
    world->jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->journalActive != 0)
    {
        m2OpCreateGearJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateGearJoint, &record, (int32_t)sizeof(record));
    }
    return jointId;
}

void m2GearJoint_SetRatio(m2JointId jointId, float ratio)
{
    m2SetJointParamInternal(m2WorldFromIndex(jointId.world0), jointId, m2_jointParamGearRatio,
                            ratio);
}

float m2GearJoint_GetRatio(m2JointId jointId)
{
    m2World* world = m2WorldFromIndex(jointId.world0);
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_gearJoint);
    return index >= 0 ? world->jointLength[index] : 0.0f;
}

// --- Solver ---------------------------------------------------------------

static void PrepareGear(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    m2Rot qA = f->qA;
    m2Rot qB = f->qB;
    float iA = f->iA;
    float iB = f->iB;
    // Gear: accumulate the phase by how far each body actually
    // rotated since last prepare (per-step deltas stay far from
    // the wrap, so many full turns remain exact), then couple
    // the spins. ratio rides jointLength -> loaded fields.
    float ratio = world->jointLength[j];
    m2Rot prevA = {world->jointLocalAnchorA[j].x, world->jointLocalAnchorA[j].y};
    m2Rot prevB = {world->jointLocalAnchorB[j].x, world->jointLocalAnchorB[j].y};
    float phase = world->jointRefAngle[j];
    phase += ratio * m2RelativeJointAngle(prevA, qA) + m2RelativeJointAngle(prevB, qB);
    world->jointRefAngle[j] = phase;
    world->jointLocalAnchorA[j] = (m2Vec2){qA.c, qA.s};
    world->jointLocalAnchorB[j] = (m2Vec2){qB.c, qB.s};
    c->baseAngle = phase;
    c->motorSpeed = ratio; // carried into the solve
    float k = ratio * ratio * iA + iB;
    c->axialMass = k > 0.0f ? 1.0f / k : 0.0f;
}

static void WarmStartGear(m2World* world, const m2JointConstraint* c)
{
    // Gear: one angular impulse, ratio-weighted on side A.
    float L = c->impulse.x;
    world->angularVelocities[c->bodyA] += world->invInertia[c->bodyA] * (c->motorSpeed * L);
    world->angularVelocities[c->bodyB] += world->invInertia[c->bodyB] * L;
}

static void SolveGear(m2World* world, m2JointConstraint* c, const m2JointSolveContext* ctx)
{
    float wA = ctx->wA;
    float wB = ctx->wB;
    bool useBias = ctx->useBias;
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
        float dA = m2Atan2(world->deltaRotations[c->bodyA].s, world->deltaRotations[c->bodyA].c);
        float dB = m2Atan2(world->deltaRotations[c->bodyB].s, world->deltaRotations[c->bodyB].c);
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

static void GearReaction(const m2World* world, int32_t j, float invH, float* force, float* torque)
{
    // A pure torque coupling.
    *force = 0.0f;
    *torque = m2AbsF(world->jointImpulse[j].x) * invH;
}

const m2JointKind m2_gearJointKind = {PrepareGear, WarmStartGear, SolveGear, GearReaction};
