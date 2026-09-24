// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint solver's stages. Prepare fills what every joint shares and
// hands the rest to its kind. Each pass then loads the two bodies,
// works out where the substep has taken the joint, lets the kind solve
// its rows against that pose, and stores the dynamic bodies back.
// Joints run serially in slot order.

#include "joint_solver.h"

#include "joint.h"
#include "solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

static const m2JointKind* const s_kinds[] = {
    [m2_distanceJoint] = &m2_distanceJointKind,   [m2_revoluteJoint] = &m2_revoluteJointKind,
    [m2_prismaticJoint] = &m2_prismaticJointKind, [m2_weldJoint] = &m2_weldJointKind,
    [m2_wheelJoint] = &m2_wheelJointKind,         [m2_filterJoint] = &m2_filterJointKind,
    [m2_motorJoint] = &m2_motorJointKind,         [m2_mouseJoint] = &m2_mouseJointKind,
    [m2_gearJoint] = &m2_gearJointKind,           [m2_pulleyJoint] = &m2_pulleyJointKind,
    [m2_ratchetJoint] = &m2_ratchetJointKind,
};

m2Softness m2StiffJointSoftness(float h)
{
    return m2MakeSoft(M2_JOINT_HERTZ, M2_JOINT_DAMPING_RATIO, h);
}

static bool Awake(const m2World* world, int32_t body)
{
    return world->bodies.types[body] == (uint8_t)m2_dynamicBody && world->bodies.asleep[body] == 0;
}

// True when the joint has rows to solve this step.
static bool JointSolves(const m2World* world, int32_t j)
{
    int32_t bodyA = world->joints.jointBodyA[j];
    int32_t bodyB = world->joints.jointBodyB[j];
    if (world->joints.jointAlive[j] == 0 || s_kinds[world->joints.jointType[j]]->prepare == NULL)
    {
        return false; // gone, or a kind without rows (the filter joint)
    }
    if (world->bodies.disabled[bodyA] != 0 || world->bodies.disabled[bodyB] != 0)
    {
        return false; // a dormant end pauses the whole joint
    }
    if (world->joints.jointType[j] == (uint8_t)m2_mouseJoint)
    {
        return Awake(world, bodyB); // a mouse joint only ever moves body B
    }
    return Awake(world, bodyA) || Awake(world, bodyB);
}

static m2Vec2 WorldArm(const m2World* world, int32_t body, m2Vec2 localAnchor)
{
    m2Vec2 center = world->bodies.localCenters[body];
    m2Vec2 arm = {localAnchor.x - center.x, localAnchor.y - center.y};
    return m2RotateVec2(world->bodies.transforms[body].q, arm);
}

// Anchor B minus anchor A, the body origins subtracted in double first.
static m2Vec2 AnchorGap(const m2World* world, const m2JointConstraint* c)
{
    m2Transform tA = world->bodies.transforms[c->bodyA];
    m2Transform tB = world->bodies.transforms[c->bodyB];
    m2Vec2 centerA = m2RotateVec2(tA.q, world->bodies.localCenters[c->bodyA]);
    m2Vec2 centerB = m2RotateVec2(tB.q, world->bodies.localCenters[c->bodyB]);
    return (m2Vec2){(float)(tB.p.x - tA.p.x) + (centerB.x - centerA.x) + (c->armB.x - c->armA.x),
                    (float)(tB.p.y - tA.p.y) + (centerB.y - centerA.y) + (c->armB.y - c->armA.y)};
}

static void PrepareCommon(const m2World* world, m2JointConstraint* c, int32_t j, float h)
{
    const m2Joints* joints = &world->joints;
    c->jointIndex = j;
    c->bodyA = joints->jointBodyA[j];
    c->bodyB = joints->jointBodyB[j];
    c->type = joints->jointType[j];
    c->flags = joints->jointFlags[j];
    c->soft = joints->jointHertz[j] > 0.0f
                  ? m2MakeSoft(joints->jointHertz[j], joints->jointDamping[j], h)
                  : m2StiffJointSoftness(h);
    c->spring = c->soft;
    c->linearSpring = false;
    c->angularSpring = false;
    c->motorSpeed = joints->jointMotorSpeed[j];
    c->maxMotorImpulse = h * joints->jointMaxMotor[j];
    c->lower = joints->jointLower[j];
    c->upper = joints->jointUpper[j];
    c->impulse = joints->jointImpulse[j];
    c->motorImpulse = joints->jointMotorImpulse[j];
    c->lowerImpulse = joints->jointLowerImpulse[j];
    c->upperImpulse = joints->jointUpperImpulse[j];
    c->springImpulse = joints->jointSpringImpulse[j];
    c->armA = WorldArm(world, c->bodyA, joints->jointLocalAnchorA[j]);
    c->armB = WorldArm(world, c->bodyB, joints->jointLocalAnchorB[j]);
    c->gap = AnchorGap(world, c);
    m2JointBodies masses = {{0.0f, 0.0f},
                            0.0f,
                            {0.0f, 0.0f},
                            0.0f,
                            world->bodies.invMass[c->bodyA],
                            world->bodies.invInertia[c->bodyA],
                            world->bodies.invMass[c->bodyB],
                            world->bodies.invInertia[c->bodyB]};
    c->pointMass = m2MakePointMass(c->armA, c->armB, &masses);
    m2Rot qA = world->bodies.transforms[c->bodyA].q;
    m2Rot qB = world->bodies.transforms[c->bodyB].q;
    c->angle = m2UnwindAngle(m2RelativeJointAngle(qA, qB) - joints->jointRefAngle[j]);
}

int32_t m2PrepareJoints(m2World* world, m2JointConstraint* joints, float h)
{
    int32_t count = 0;
    for (int32_t j = 0; j < world->joints.maxJointIndex; ++j)
    {
        if (!JointSolves(world, j))
        {
            continue;
        }
        m2JointConstraint* c = joints + count;
        count += 1;
        PrepareCommon(world, c, j, h);
        m2JointFrame frame = {j, h, world->bodies.transforms[c->bodyA].q,
                              world->bodies.transforms[c->bodyB].q};
        s_kinds[c->type]->prepare(world, c, &frame);
    }
    return count;
}

static m2JointBodies LoadBodies(const m2World* world, const m2JointConstraint* c)
{
    m2JointBodies b = {
        world->bodies.linearVelocities[c->bodyA], world->bodies.angularVelocities[c->bodyA],
        world->bodies.linearVelocities[c->bodyB], world->bodies.angularVelocities[c->bodyB],
        world->bodies.invMass[c->bodyA],          world->bodies.invInertia[c->bodyA],
        world->bodies.invMass[c->bodyB],          world->bodies.invInertia[c->bodyB],
    };
    return b;
}

static void StoreBodies(m2World* world, const m2JointConstraint* c, const m2JointBodies* b)
{
    if (world->bodies.types[c->bodyA] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyA] = b->vA;
        world->bodies.angularVelocities[c->bodyA] = b->wA;
    }
    if (world->bodies.types[c->bodyB] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.linearVelocities[c->bodyB] = b->vB;
        world->bodies.angularVelocities[c->bodyB] = b->wB;
    }
}

static m2JointPose CurrentPose(const m2World* world, const m2JointConstraint* c)
{
    m2JointPose pose;
    pose.moveA = world->solver.deltaPositions[c->bodyA];
    pose.moveB = world->solver.deltaPositions[c->bodyB];
    pose.turnA = world->solver.deltaRotations[c->bodyA];
    pose.turnB = world->solver.deltaRotations[c->bodyB];
    pose.armA = m2RotateVec2(pose.turnA, c->armA);
    pose.armB = m2RotateVec2(pose.turnB, c->armB);
    return pose;
}

void m2WarmStartJoints(m2World* world, m2JointConstraint* joints, int32_t count)
{
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        m2JointPose pose = CurrentPose(world, c);
        m2JointBodies b = LoadBodies(world, c);
        s_kinds[c->type]->warmStart(c, &pose, &b);
        StoreBodies(world, c, &b);
    }
}

void m2SolveJoints(m2World* world, m2JointConstraint* joints, int32_t count, bool biased,
                   float invH)
{
    m2JointPass pass = {biased, invH};
    for (int32_t i = 0; i < count; ++i)
    {
        m2JointConstraint* c = joints + i;
        m2JointPose pose = CurrentPose(world, c);
        m2JointBodies b = LoadBodies(world, c);
        s_kinds[c->type]->solve(c, &pose, &b, &pass);
        StoreBodies(world, c, &b);
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

m2Vec2 m2PoseGap(const m2JointConstraint* c, const m2JointPose* pose)
{
    return (m2Vec2){c->gap.x + (pose->moveB.x - pose->moveA.x) + (pose->armB.x - c->armB.x) -
                        (pose->armA.x - c->armA.x),
                    c->gap.y + (pose->moveB.y - pose->moveA.y) + (pose->armB.y - c->armB.y) -
                        (pose->armA.y - c->armA.y)};
}

m2JointRow m2AxisRow(const m2JointPose* pose, m2Vec2 gap, m2Vec2 axis)
{
    m2Vec2 lever = {pose->armA.x + gap.x, pose->armA.y + gap.y};
    return m2LineRow(lever, pose->armB, axis);
}

float m2PoseAngle(const m2JointConstraint* c, const m2JointPose* pose)
{
    return m2UnwindAngle(c->angle + m2RelativeJointAngle(pose->turnA, pose->turnB));
}

void m2SolveLimits(m2JointConstraint* c, const m2JointRow* row, float value, m2Softness soft,
                   m2JointBodies* b, const m2JointPass* pass)
{
    m2RowDrive lower = m2LimitDrive(soft, value - c->lower, pass->invH, pass->biased);
    m2SolveRow(row, b, lower, &c->lowerImpulse, 0.0f, M2_ROW_FREE);
    m2JointRow back = m2ScaleRow(*row, -1.0f);
    m2RowDrive upper = m2LimitDrive(soft, c->upper - value, pass->invH, pass->biased);
    m2SolveRow(&back, b, upper, &c->upperImpulse, 0.0f, M2_ROW_FREE);
}

void m2WarmStartLimits(const m2JointConstraint* c, const m2JointRow* row, m2JointBodies* b)
{
    m2PushRow(row, b, c->lowerImpulse - c->upperImpulse);
}
