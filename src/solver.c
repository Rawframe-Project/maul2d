// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The step: integration, the solver stages in order, and the joint
// break pass.

#include "solver.h"

#include "ccd.h"
#include "contact_solver.h"
#include "joint_solver.h"
#include "rotation.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

// A body the integrator moves this substep.
static bool Moving(const m2World* world, int32_t i)
{
    return world->bodies.alive[i] != 0 && world->bodies.types[i] != (uint8_t)m2_staticBody &&
           world->bodies.asleep[i] == 0 && world->bodies.disabled[i] == 0;
}

// A body's locked linear axes hold still: their velocity is zeroed where
// the solve begins and again where the position consumes it. A locked
// rotation needs nothing here; its inverse inertia is zero.
static void HoldLockedAxes(m2World* world, int32_t i)
{
    uint8_t locks = world->bodies.motionLocks[i];
    if (locks & M2_LOCK_LINEAR_X)
    {
        world->bodies.linearVelocities[i].x = 0.0f;
    }
    if (locks & M2_LOCK_LINEAR_Y)
    {
        world->bodies.linearVelocities[i].y = 0.0f;
    }
}

// Forces, gravity and damping for one substep. Damping is implicit,
// v' = v / (1 + h d), so it can never reverse a velocity however large
// h d grows; the forces add on top: v' = h (F / m + s g) + v / (1 + h d).
static void IntegrateVelocities(m2World* world, float h)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (!Moving(world, i) || world->bodies.types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        float linDamp = 1.0f / (1.0f + h * world->bodies.linearDampings[i]);
        float angDamp = 1.0f / (1.0f + h * world->bodies.angularDampings[i]);
        float lvdx = h * world->bodies.invMass[i] * world->bodies.forces[i].x +
                     h * world->bodies.gravityScales[i] * world->gravity.x;
        float lvdy = h * world->bodies.invMass[i] * world->bodies.forces[i].y +
                     h * world->bodies.gravityScales[i] * world->gravity.y;
        world->bodies.linearVelocities[i].x = lvdx + linDamp * world->bodies.linearVelocities[i].x;
        world->bodies.linearVelocities[i].y = lvdy + linDamp * world->bodies.linearVelocities[i].y;
        world->bodies.angularVelocities[i] =
            h * world->bodies.invInertia[i] * world->bodies.torques[i] +
            angDamp * world->bodies.angularVelocities[i];
        HoldLockedAxes(world, i);
    }
}

// Bullet substep origins, captured before positions move.
static void CaptureBulletOrigins(m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.bullets[i] != 0 && world->bodies.disabled[i] == 0)
        {
            world->solver.ccdPrevPositions[i] = world->bodies.transforms[i].p;
        }
    }
}

// Safety caps on a dynamic body's velocity: M2_MAX_LINEAR_SPEED, and a
// quarter turn per substep. They bound what the position update sees,
// after every solver stage has had its say, so an over-constrained
// setup that pumps velocity each substep is stopped at the source
// before it can reach infinity. Ordinary scenes never reach either cap.
static void CapVelocity(m2World* world, int32_t i, float invH)
{
    float vx = world->bodies.linearVelocities[i].x;
    float vy = world->bodies.linearVelocities[i].y;
    float v2 = vx * vx + vy * vy;
    if (v2 > M2_MAX_LINEAR_SPEED * M2_MAX_LINEAR_SPEED)
    {
        float ratio = M2_MAX_LINEAR_SPEED / sqrtf(v2);
        world->bodies.linearVelocities[i].x = vx * ratio;
        world->bodies.linearVelocities[i].y = vy * ratio;
    }
    float maxW = 0.25f * M2_PI * invH;
    float w = world->bodies.angularVelocities[i];
    if (w * w > maxW * maxW)
    {
        world->bodies.angularVelocities[i] = w * (maxW / m2AbsF(w));
    }
}

// Moves the center of mass by the velocity and turns the body about it;
// the origin follows the turn. The f64 position advances and the f32
// deltas track the motion since the step began.
static void AdvancePose(m2World* world, int32_t i, float h)
{
    m2Vec2 lc = world->bodies.localCenters[i];
    m2Vec2 v = world->bodies.linearVelocities[i];
    m2Vec2 centerBefore = m2RotateVec2(world->bodies.transforms[i].q, lc);
    world->bodies.transforms[i].p.x += (double)v.x * (double)h;
    world->bodies.transforms[i].p.y += (double)v.y * (double)h;
    world->solver.deltaPositions[i].x += v.x * h;
    world->solver.deltaPositions[i].y += v.y * h;
    m2Rot turn = m2RotOfAngle(world->bodies.angularVelocities[i] * h);
    world->bodies.transforms[i].q = m2RotCompose(world->bodies.transforms[i].q, turn);
    world->solver.deltaRotations[i] = m2RotCompose(world->solver.deltaRotations[i], turn);
    m2Vec2 centerAfter = m2RotateVec2(world->bodies.transforms[i].q, lc);
    world->bodies.transforms[i].p.x += (double)(centerBefore.x - centerAfter.x);
    world->bodies.transforms[i].p.y += (double)(centerBefore.y - centerAfter.y);
}

static void IntegratePositions(m2World* world, float h, float invH)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (!Moving(world, i))
        {
            continue;
        }
        if (world->bodies.types[i] == (uint8_t)m2_dynamicBody)
        {
            CapVelocity(world, i, invH);
        }
        HoldLockedAxes(world, i);
        AdvancePose(world, i, h);
    }
}

// Breaks overloaded joints. Reaction magnitudes come straight from the
// stored impulses, so breaking is a pure function of state: twins snap on
// the same step and replays never disagree. Canonical joint order; the
// destroyed id is reported with the generation it had.
static void BreakJoints(m2World* world, float invH)
{
    for (int32_t j = 0; j < world->joints.maxJointIndex; ++j)
    {
        if (world->joints.jointAlive[j] == 0)
        {
            continue;
        }
        float breakForce = world->joints.jointBreakForce[j];
        float breakTorque = world->joints.jointBreakTorque[j];
        if (breakForce == 0.0f && breakTorque == 0.0f)
        {
            continue;
        }

        float force = 0.0f;
        float torque = 0.0f;
        m2JointReactionMagnitudes(world, j, invH, &force, &torque);

        bool snapped = (breakForce > 0.0f && force > breakForce) ||
                       (breakTorque > 0.0f && torque > breakTorque);
        if (!snapped)
        {
            continue;
        }

        if (world->events.jointBreakEventCount < world->joints.jointCapacity)
        {
            m2JointBreakEvent* e =
                &world->events.jointBreakEvents[world->events.jointBreakEventCount++];
            memset(e, 0, sizeof(*e));
            e->jointId.index1 = j + 1;
            e->jointId.world = world->idWorld;
            e->jointId.generation = world->joints.jointGenerations[j];
            e->step = world->stepCount;
            e->force = force;
            e->torque = torque;
        }
        m2DestroyJointInternal(world, j);
    }
}

// The step's constraints: contacts planned into colored blocks and the
// joints behind them in the same scratch block.
typedef struct StepWork
{
    m2ContactPlan plan;
    m2JointConstraint* joints;
    int32_t jointCount;
} StepWork;

static void PrepareStep(m2World* world, StepWork* work, float h, float invH)
{
    // The deltas measure motion since the step began.
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        world->solver.deltaPositions[i] = (m2Vec2){0.0f, 0.0f};
        world->solver.deltaRotations[i] = (m2Rot){1.0f, 0.0f};
    }
    work->plan.constraints = (m2ContactConstraint*)world->solver.constraintScratch;
    work->plan.count = m2PrepareContacts(world, work->plan.constraints, h);
    work->plan.invH = invH;
    work->plan.reversed = false;
    work->joints =
        (m2JointConstraint*)((uint8_t*)world->solver.constraintScratch +
                             (size_t)world->contacts.pairCapacity * sizeof(m2ContactConstraint));
    work->jointCount = m2PrepareJoints(world, work->joints, h);
    m2PlanContacts(world, &work->plan);

    const int32_t* colorStart = work->plan.colorStart;
    world->solver.lastConstraintCount = work->plan.count;
    world->solver.lastOverflow = colorStart[M2_GRAPH_COLORS + 1] - colorStart[M2_GRAPH_COLORS];
    world->solver.lastGraphColors = 0;
    for (int32_t c = 0; c < M2_GRAPH_COLORS; ++c)
    {
        world->solver.lastGraphColors += colorStart[c + 1] > colorStart[c] ? 1 : 0;
    }
}

// One substep: forces in; warm start, then the biased solve pushing
// overlap and joint error out (joints first, so contacts see the
// joints' result); positions move; continuous collision clamps the
// fast bodies; then the relax pass, unbiased, removes the velocity the
// push added.
static void Substep(m2World* world, StepWork* work, float h, float invH, int32_t index)
{
    IntegrateVelocities(world, h);
    work->plan.reversed = (index & 1) != 0;
    m2WarmStartJoints(world, work->joints, work->jointCount);
    m2RunContactStage(world, &work->plan, m2_stageWarmStart);
    m2SolveJoints(world, work->joints, work->jointCount, true, invH);
    m2RunContactStage(world, &work->plan, m2_stageSolve);
    CaptureBulletOrigins(world);
    IntegratePositions(world, h, invH);
    m2SolveContinuous(world); // the last stage that moves transforms
    m2SolveJoints(world, work->joints, work->jointCount, false, invH);
    m2RunContactStage(world, &work->plan, m2_stageRelax);
}

void m2SolveStep(m2World* world, float dt, int32_t substepCount)
{
    float h = dt / (float)substepCount;
    float invH = h > 0.0f ? 1.0f / h : 0.0f;
    world->lastInvH = invH;
    StepWork work;
    PrepareStep(world, &work, h, invH);
    for (int32_t sub = 0; sub < substepCount; ++sub)
    {
        Substep(world, &work, h, invH, sub);
    }
    // Once per step: the bounce, then the impulses kept for the next
    // step's warm start, then the joints that could not hold.
    m2RunContactStage(world, &work.plan, m2_stageRestitution);
    m2RunContactStage(world, &work.plan, m2_stageStore);
    m2StoreJointImpulses(world, work.joints, work.jointCount);
    BreakJoints(world, invH);
}
