// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The step: integration, the solver stages in order, and the joint
// break pass.

#include "solver.h"

#include "ccd.h"
#include "contact_solver.h"
#include "joint_solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

// Integrates velocities in fixed body order: v = lvd + damp * v with Pade
// damping 1/(1+h*d) and lvd = h*invM*force + h*gScale*g; torque and angular
// damping mirror it.
static void IntegrateVelocities(m2World* world, float h)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[i] != 0 || world->bodies.disabled[i] != 0)
        {
            continue;
        }
        // Reference form: v = lvd + damp * v, with the Pade damping
        // 1/(1+h*d) and lvd = h*invM*force + h*gScale*g. Torque and
        // angular damping mirror it.
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
        // Motion locks: a locked axis holds still,
        // so its velocity is zeroed here, before the constraint solve,
        // and again at integrate-positions below (angular is locked via
        // the mass, invInertia = 0). Off the locked axes are untouched.
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

// Integrates positions: f64 positions advance, f32 deltas track.
static void IntegratePositions(m2World* world, float h, float invH)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] == (uint8_t)m2_staticBody ||
            world->bodies.asleep[i] != 0 || world->bodies.disabled[i] != 0)
        {
            continue;
        }
        // Reference velocity caps (b2 maximumLinearSpeed 400 m/s and
        // B2_MAX_ROTATION, a quarter turn per substep), applied to the
        // STORED dynamic velocity at the point the integrator consumes
        // it. The reference caps in IntegrateVelocities; Maul caps here
        // because a warm-started joint accumulation can spike a body past
        // the cap AFTER that stage, and the guard must bound what
        // m2MakeRot and the f64 position actually see. Capping the stored
        // value each substep also breaks an over-constrained scene's
        // exponential velocity growth at the source, so it can never
        // reach inf and hand a kinematic neighbour 0 * inf = NaN. Ratio
        // scaling matches the reference bit-form; tame scenes reach
        // neither cap, so the gated hashes are untouched.
        if (world->bodies.types[i] == (uint8_t)m2_dynamicBody)
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
            float wv = world->bodies.angularVelocities[i];
            if (wv * wv > maxW * maxW)
            {
                float ratio = maxW / m2AbsF(wv);
                world->bodies.angularVelocities[i] = wv * ratio;
            }
        }
        // Motion locks again at the point the position consumes the
        // velocity: whatever the solve pushed along
        // a locked axis, the body does not move along it.
        uint8_t plocks = world->bodies.motionLocks[i];
        if (plocks & M2_LOCK_LINEAR_X)
        {
            world->bodies.linearVelocities[i].x = 0.0f;
        }
        if (plocks & M2_LOCK_LINEAR_Y)
        {
            world->bodies.linearVelocities[i].y = 0.0f;
        }
        // The center of mass is what the velocity moves; the origin
        // swings around it. With the COM on the origin both extra
        // terms are exact zeros and the old bits fall out.
        m2Vec2 lc = world->bodies.localCenters[i];
        m2Vec2 rlcOld = m2RotateVec2(world->bodies.transforms[i].q, lc);
        world->bodies.transforms[i].p.x += (double)world->bodies.linearVelocities[i].x * (double)h;
        world->bodies.transforms[i].p.y += (double)world->bodies.linearVelocities[i].y * (double)h;
        world->solver.deltaPositions[i].x += world->bodies.linearVelocities[i].x * h;
        world->solver.deltaPositions[i].y += world->bodies.linearVelocities[i].y * h;
        m2Rot dq = m2MakeRot(world->bodies.angularVelocities[i] * h);
        world->bodies.transforms[i].q = m2MulRot(world->bodies.transforms[i].q, dq);
        world->solver.deltaRotations[i] = m2MulRot(world->solver.deltaRotations[i], dq);
        m2Vec2 rlcNew = m2RotateVec2(world->bodies.transforms[i].q, lc);
        world->bodies.transforms[i].p.x += (double)(rlcOld.x - rlcNew.x);
        world->bodies.transforms[i].p.y += (double)(rlcOld.y - rlcNew.y);
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
            e->jointId.world0 = world->worldIndex0;
            e->jointId.generation = world->joints.jointGenerations[j];
            e->step = world->stepCount;
            e->force = force;
            e->torque = torque;
        }
        m2DestroyJointInternal(world, j);
    }
}

void m2SolveStep(m2World* world, float dt, int32_t substepCount)
{
    float h = dt / (float)substepCount;
    float invH = h > 0.0f ? 1.0f / h : 0.0f;
    world->lastInvH = invH;

    // Deltas are step-transient: zero at prepare, folded into f64
    // positions at each integrate-positions stage.
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        world->solver.deltaPositions[i] = (m2Vec2){0.0f, 0.0f};
        world->solver.deltaRotations[i] = (m2Rot){1.0f, 0.0f};
    }

    m2ContactPlan plan;
    plan.constraints = (m2ContactConstraint*)world->solver.constraintScratch;
    plan.count = m2PrepareContacts(world, plan.constraints, h);
    plan.invH = invH;
    plan.reversed = false;
    m2JointConstraint* joints =
        (m2JointConstraint*)((uint8_t*)world->solver.constraintScratch +
                             (size_t)world->contacts.pairCapacity * sizeof(m2ContactConstraint));
    int32_t jointCount = m2PrepareJoints(world, joints, h);

    m2PlanContacts(world, &plan);
    world->solver.lastConstraintCount = plan.count;
    world->solver.lastOverflow =
        plan.colorStart[M2_GRAPH_COLORS + 1] - plan.colorStart[M2_GRAPH_COLORS];
    world->solver.lastGraphColors = 0;
    for (int32_t c = 0; c < M2_GRAPH_COLORS; ++c)
    {
        world->solver.lastGraphColors += plan.colorStart[c + 1] > plan.colorStart[c] ? 1 : 0;
    }

    for (int32_t sub = 0; sub < substepCount; ++sub)
    {
        IntegrateVelocities(world, h);
        plan.reversed = (sub & 1) != 0;

        m2WarmStartJoints(world, joints, jointCount);
        m2RunContactStage(world, &plan, m2_stageWarmStart);
        m2SolveJoints(world, joints, jointCount, true, invH); // joints before contacts
        m2RunContactStage(world, &plan, m2_stageSolve);

        CaptureBulletOrigins(world);
        IntegratePositions(world, h, invH);

        m2SolveContinuous(world); // the last pass that moves transforms
        m2SolveJoints(world, joints, jointCount, false, invH);
        m2RunContactStage(world, &plan, m2_stageRelax);
    }

    m2RunContactStage(world, &plan, m2_stageRestitution);
    m2RunContactStage(world, &plan, m2_stageStore);
    m2StoreJointImpulses(world, joints, jointCount);

    BreakJoints(world, invH);
}
