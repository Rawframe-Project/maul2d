// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The distance joint: a rest-length row, optionally soft, with an
// optional hard length range.

#include "body.h"
#include "broadphase.h"
#include "fpenv.h"
#include "joint.h"
#include "joint_solver.h"
#include "journal.h"
#include "object_ids.h"
#include "solver_base.h"
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
    return m2LocalVec2(def->localAnchorA) && m2LocalVec2(def->localAnchorB) &&
           m2ValidReach(def->length) && m2ValidReach(def->minLength) &&
           m2ValidReach(def->maxLength) &&
           !(def->maxLength > 0.0f && def->minLength > def->maxLength) && m2ValidRate(def->hertz) &&
           m2ValidCoefficient(def->dampingRatio);
}

static M2_NOINLINE m2Result CreateDistanceJointWorker(m2World* world, const m2DistanceJointDef* def,
                                                      m2JointId* jointOut)
{
    if (jointOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *jointOut = m2_nullJointId;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (def == NULL || def->internalValue != M2_DJOINT_COOKIE || !DefValid(def))
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t bodyA = m2BodySlot(world, def->bodyIdA);
    int32_t bodyB = m2BodySlot(world, def->bodyIdB);
    if (bodyA < 0 || bodyB < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    if (bodyA == bodyB)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    int32_t index = m2AllocateJoint(world);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    float length = def->length;
    if (!(length > 0.0f))
    {
        // Derive from spawn poses: the single f64 crossing.
        m2Transform xfA = world->bodies.transforms[bodyA];
        m2Transform xfB = world->bodies.transforms[bodyB];
        m2Vec2 wA = {xfA.q.c * def->localAnchorA.x - xfA.q.s * def->localAnchorA.y,
                     xfA.q.s * def->localAnchorA.x + xfA.q.c * def->localAnchorA.y};
        m2Vec2 wB = {xfB.q.c * def->localAnchorB.x - xfB.q.s * def->localAnchorB.y,
                     xfB.q.s * def->localAnchorB.x + xfB.q.c * def->localAnchorB.y};
        float dx = m2WorldOffset(xfB.p.x - xfA.p.x) + wB.x - wA.x;
        float dy = m2WorldOffset(xfB.p.y - xfA.p.y) + wB.y - wA.y;
        length = sqrtf(dx * dx + dy * dy);
    }
    m2JointId jointId =
        m2FinishJoint(world, index, (uint8_t)m2_distanceJoint, bodyA, bodyB, def->localAnchorA,
                      def->localAnchorB, length, def->hertz, def->dampingRatio);
    // The hard range: off by default (0 .. huge); a def maxLength <= 0
    // means unbounded, mirroring "length <= 0 derives".
    world->joints.jointLower[index] = def->minLength > 0.0f ? def->minLength : 0.0f;
    world->joints.jointUpper[index] = def->maxLength > 0.0f ? def->maxLength : M2_OPEN_LENGTH;
    if (def->enableSpring)
    {
        world->joints.jointFlags[index] |= M2_JOINT_ROPE; // rope/rod: gate the rest-length row
    }
    world->joints.jointUserData[index] = def->userData;
    world->joints.jointCollide[index] = def->collideConnected ? 1 : 0;
    if (def->collideConnected == false)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB);
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateDistanceJoint record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = jointId;
        m2JournalRecord(world, m2_opCreateDistanceJoint, &record, (int32_t)sizeof(record));
    }
    *jointOut = jointId;
    return m2_success;
}

m2Result m2CreateDistanceJoint(m2World* world, const m2DistanceJointDef* def, m2JointId* jointOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateDistanceJointWorker(world, def, jointOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void DistanceJointSetLengthWorker(m2World* world, m2JointId jointId,
                                                     float length)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamLength, length);
}

void m2DistanceJoint_SetLength(m2World* world, m2JointId jointId, float length)
{
    uint64_t env = m2FloatEnvEnter();
    DistanceJointSetLengthWorker(world, jointId, length);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void DistanceJointSetLengthRangeWorker(m2World* world, m2JointId jointId,
                                                          float minLength, float maxLength)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (!m2ValidReach(minLength) || !m2ValidReach(maxLength))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Lengths are floored at the linear slop and put in order.
    float lo = minLength > 0.005f ? minLength : 0.005f;
    float hi = maxLength > 0.005f ? maxLength : 0.005f;
    float lower = lo < hi ? lo : hi;
    float upper = lo < hi ? hi : lo;
    if (m2SetJointParamInternal(world, jointId, m2_jointParamMinLength, lower))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamMaxLength, upper);
    }
}

void m2DistanceJoint_SetLengthRange(m2World* world, m2JointId jointId, float minLength,
                                    float maxLength)
{
    uint64_t env = m2FloatEnvEnter();
    DistanceJointSetLengthRangeWorker(world, jointId, minLength, maxLength);
    m2FloatEnvLeave(env);
}

bool m2DistanceJoint_IsSpringEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = m2TypedJointSlot(world, jointId, (uint8_t)m2_distanceJoint);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_ROPE) != 0;
}

// --- Solver ---------------------------------------------------------------
//
// One row along the line between the anchors: the rest length (impulse.x)
// unless the joint is a free rope, and the length range when one is set.
// The range stays hard on a soft rope: its rows use the stiff softness.

static void PrepareDistance(m2World* world, m2JointConstraint* c, const m2JointFrame* f)
{
    int32_t j = f->joint;
    c->length = world->joints.jointLength[j];
    if (world->joints.jointLower[j] > 0.0f || world->joints.jointUpper[j] < M2_OPEN_LENGTH)
    {
        c->flags |= M2_JOINT_HARD_RANGE;
        c->spring = m2StiffJointSoftness(f->h);
    }
    if ((c->flags & M2_JOINT_ROPE) != 0 && world->joints.jointHertz[j] == 0.0f)
    {
        // A spring with no stiffness: only the range acts, and no rest
        // impulse is carried.
        c->flags |= M2_JOINT_FREE_LENGTH;
        c->impulse.x = 0.0f;
    }
}

// The row along the anchor line and the current length. Anchors that
// meet leave the direction open; +y stands in.
static m2JointRow RopeRow(const m2JointConstraint* c, const m2JointPose* pose, float* length)
{
    m2Vec2 gap = m2PoseGap(c, pose);
    *length = sqrtf(gap.x * gap.x + gap.y * gap.y);
    m2Vec2 u = *length > 1.19209290e-7f ? (m2Vec2){gap.x / *length, gap.y / *length}
                                        : (m2Vec2){0.0f, 1.0f};
    return m2LineRow(pose->armA, pose->armB, u);
}

static void WarmStartDistance(const m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b)
{
    float length;
    m2JointRow row = RopeRow(c, pose, &length);
    m2PushRow(&row, b, c->impulse.x);
    if ((c->flags & M2_JOINT_HARD_RANGE) != 0)
    {
        m2WarmStartLimits(c, &row, b);
    }
}

static void SolveDistance(m2JointConstraint* c, const m2JointPose* pose, m2JointBodies* b,
                          const m2JointPass* pass)
{
    float length;
    m2JointRow row = RopeRow(c, pose, &length);
    if ((c->flags & M2_JOINT_FREE_LENGTH) == 0)
    {
        m2RowDrive drive = m2HeldDrive(c->soft, length - c->length, pass->biased);
        m2SolveRow(&row, b, drive, &c->impulse.x, -M2_ROW_FREE, M2_ROW_FREE);
    }
    if ((c->flags & M2_JOINT_HARD_RANGE) != 0)
    {
        m2SolveLimits(c, &row, length, c->spring, b, pass);
    }
}

static void DistanceReaction(const m2World* world, int32_t j, float invH, float* force,
                             float* torque)
{
    // The rest row, plus the range rows when bounded.
    float axial = world->joints.jointImpulse[j].x;
    if (world->joints.jointLower[j] > 0.0f || world->joints.jointUpper[j] < M2_OPEN_LENGTH)
    {
        axial += world->joints.jointLowerImpulse[j] - world->joints.jointUpperImpulse[j];
    }
    *force = m2AbsF(axial) * invH;
    *torque = 0.0f;
}

const m2JointKind m2_distanceJointKind = {PrepareDistance, WarmStartDistance, SolveDistance,
                                          DistanceReaction};
