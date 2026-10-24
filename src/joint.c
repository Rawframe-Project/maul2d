// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joints: creation, destruction, the per-body joint lists, parameters
// and readback for every joint type.

#include "joint.h"

#include "broadphase.h"
#include "island.h"
#include "joint_edges.h"
#include "joint_row.h"
#include "joint_solver.h"
#include "journal.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

uint64_t m2Joint_GetUserData(const m2World* world, m2JointId jointId)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->joints.jointUserData[index];
}

void m2Joint_SetUserData(m2World* world, m2JointId jointId, uint64_t userData)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpJointUserData record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.userData = userData;
        m2JournalRecord(world, m2_opJointUserData, &record, (int32_t)sizeof(record));
    }
    world->joints.jointUserData[index] = userData;
}

float m2Joint_GetLinearSeparation(const m2World* world, m2JointId jointId)
{
    int32_t j = (int32_t)jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->joints.jointCapacity ||
        world->joints.jointAlive[j] == 0 || world->joints.jointGenerations[j] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    int32_t bodyA = world->joints.jointBodyA[j];
    int32_t bodyB = world->joints.jointBodyB[j];
    m2Transform xfA = world->bodies.transforms[bodyA];
    m2Transform xfB = world->bodies.transforms[bodyB];
    m2Vec2 aA = world->joints.jointLocalAnchorA[j];
    m2Vec2 aB = world->joints.jointLocalAnchorB[j];
    m2Vec2 wA = {xfA.q.c * aA.x - xfA.q.s * aA.y, xfA.q.s * aA.x + xfA.q.c * aA.y};
    m2Vec2 wB = {xfB.q.c * aB.x - xfB.q.s * aB.y, xfB.q.s * aB.x + xfB.q.c * aB.y};
    float dx = (float)(xfB.p.x - xfA.p.x) + wB.x - wA.x;
    float dy = (float)(xfB.p.y - xfA.p.y) + wB.y - wA.y;
    switch (world->joints.jointType[j])
    {
    case 0: // distance: length error along the rod
        return m2AbsF(sqrtf(dx * dx + dy * dy) - world->joints.jointLength[j]);
    case 2: // prismatic: the off-axis gap
    case 4: // wheel: same slider geometry
    {
        m2Vec2 axis = world->joints.jointLocalAxisA[j];
        m2Vec2 worldAxis = {xfA.q.c * axis.x - xfA.q.s * axis.y,
                            xfA.q.s * axis.x + xfA.q.c * axis.y};
        float perp = dx * -worldAxis.y + dy * worldAxis.x;
        return m2AbsF(perp);
    }
    case 5: // filter: pins nothing
        return 0.0f;
    case 6: // motor: distance from the commanded offset
    {
        m2Vec2 off = world->joints.jointLocalAxisA[j];
        m2Vec2 worldOff = {xfA.q.c * off.x - xfA.q.s * off.y, xfA.q.s * off.x + xfA.q.c * off.y};
        float ex = dx - worldOff.x;
        float ey = dy - worldOff.y;
        return sqrtf(ex * ex + ey * ey);
    }
    case 7: // mouse: gap between grab point and target
    {
        m2Pos2 grab = m2Body_GetWorldPoint(
            world, (m2BodyId){(uint32_t)(bodyB + 1), world->bodies.generations[bodyB]}, aB);
        float gx = (float)(grab.x - world->joints.jointTargets[j].x);
        float gy = (float)(grab.y - world->joints.jointTargets[j].y);
        return sqrtf(gx * gx + gy * gy);
    }
    default: // revolute, weld: the pinned point's gap
        return sqrtf(dx * dx + dy * dy);
    }
}

float m2Joint_GetAngularSeparation(const m2World* world, m2JointId jointId)
{
    int32_t j = (int32_t)jointId.index1 - 1;
    if (world == NULL || j < 0 || j >= world->joints.jointCapacity ||
        world->joints.jointAlive[j] == 0 || world->joints.jointGenerations[j] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    uint8_t type = world->joints.jointType[j];
    if (type != (uint8_t)m2_weldJoint && type != (uint8_t)m2_motorJoint &&
        type != (uint8_t)m2_prismaticJoint)
    {
        return 0.0f; // no angle is pinned
    }
    m2Rot qA = world->bodies.transforms[world->joints.jointBodyA[j]].q;
    m2Rot qB = world->bodies.transforms[world->joints.jointBodyB[j]].q;
    return m2AbsF(m2UnwindAngle(m2RelativeJointAngle(qA, qB) - world->joints.jointRefAngle[j]));
}

int32_t m2AllocateJoint(m2World* world)
{
    if (world->joints.jointFreeCount == 0)
    {
        return -1;
    }
    int32_t index = world->joints.jointFreeQueue[world->joints.jointFreeHead];
    world->joints.jointFreeHead = (world->joints.jointFreeHead + 1) % world->joints.jointCapacity;
    world->joints.jointFreeCount -= 1;
    if (index + 1 > world->joints.maxJointIndex)
    {
        world->joints.maxJointIndex = index + 1;
    }
    return index;
}

m2JointId m2FinishJoint(m2World* world, int32_t index, uint8_t type, int32_t bodyA, int32_t bodyB,
                        m2Vec2 anchorA, m2Vec2 anchorB, float length, float hertz, float damping)
{
    world->joints.jointType[index] = type;
    world->joints.jointBodyA[index] = bodyA;
    world->joints.jointBodyB[index] = bodyB;
    world->joints.jointLocalAnchorA[index] = anchorA;
    world->joints.jointLocalAnchorB[index] = anchorB;
    world->joints.jointLength[index] = length;
    world->joints.jointHertz[index] = hertz;
    world->joints.jointDamping[index] = damping;
    world->joints.jointHertz2[index] = 0.0f;
    world->joints.jointDamping2[index] = 0.0f;
    world->joints.jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
    world->joints.jointFlags[index] = 0;
    world->joints.jointMotorSpeed[index] = 0.0f;
    world->joints.jointMaxMotor[index] = 0.0f;
    world->joints.jointLower[index] = 0.0f;
    world->joints.jointUpper[index] = 0.0f;
    world->joints.jointLocalAxisA[index] = (m2Vec2){1.0f, 0.0f};
    world->joints.jointRefAngle[index] = 0.0f;
    world->joints.jointMotorImpulse[index] = 0.0f;
    world->joints.jointLowerImpulse[index] = 0.0f;
    world->joints.jointUpperImpulse[index] = 0.0f;
    world->joints.jointSpringImpulse[index] = 0.0f;
    world->joints.jointBreakForce[index] = 0.0f;
    world->joints.jointBreakTorque[index] = 0.0f;
    world->joints.jointCollide[index] = 1;
    world->joints.jointTargets[index] = (m2Pos2){0.0, 0.0};
    world->joints.jointTargetsB[index] = (m2Pos2){0.0, 0.0};
    world->joints.jointUserData[index] = 0;
    world->joints.jointAlive[index] = 1;
    m2LinkJoint(world, index);
    // A new constraint wakes both ends.
    world->bodies.asleep[bodyA] = 0;
    world->bodies.sleepTimes[bodyA] = 0.0f;
    world->bodies.asleep[bodyB] = 0;
    world->bodies.sleepTimes[bodyB] = 0.0f;
    m2JointId id = {(uint32_t)(index + 1), world->joints.jointGenerations[index]};
    return id;
}

// One journaled channel for every runtime joint parameter: replay
// re-drives the same setters through the same records.
// The contract of each joint parameter channel: the joint kinds that
// carry the parameter and the values it takes. Live setters and replay
// both pass through it, so a tape can never write what the API refuses.
static bool JointParamValid(uint8_t type, uint8_t param, float value)
{
    switch (param)
    {
    case m2_jointParamEnableMotor:
    case m2_jointParamEnableLimit:
        return value == 0.0f || value == 1.0f;
    case m2_jointParamMotorSpeed:
    case m2_jointParamLower:
    case m2_jointParamUpper:
        return m2FiniteF(value);
    case m2_jointParamMaxMotor:
        return m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamBreakForce:
    case m2_jointParamBreakTorque:
        return value >= 0.0f; // zero turns breaking off; infinity never breaks
    case m2_jointParamHertz:
    case m2_jointParamDamping:
        return type != (uint8_t)m2_filterJoint && type != (uint8_t)m2_motorJoint &&
               m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamAngularHertz:
    case m2_jointParamAngularDamping:
        return (type == (uint8_t)m2_weldJoint || type == (uint8_t)m2_revoluteJoint) &&
               m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamLength:
        return type == (uint8_t)m2_distanceJoint && m2FiniteF(value) && value > 0.0f;
    case m2_jointParamMinLength:
    case m2_jointParamMaxLength:
        return type == (uint8_t)m2_distanceJoint && m2FiniteF(value) && value >= 0.0f;
    case m2_jointParamGearRatio:
        return type == (uint8_t)m2_gearJoint && m2FiniteF(value) && value != 0.0f;
    case m2_jointParamPulleyRatio:
        return type == (uint8_t)m2_pulleyJoint && m2FiniteF(value) && value > 0.0f;
    default:
        return false;
    }
}

// Writes one validated parameter; distance and pulley retargets drop
// the joint's accumulated impulses.
static void ApplyJointParam(m2World* world, int32_t index, uint8_t param, float value)
{
    m2Joints* j = &world->joints;
    switch (param)
    {
    case m2_jointParamMotorSpeed:
        j->jointMotorSpeed[index] = value;
        break;
    case m2_jointParamMaxMotor:
        j->jointMaxMotor[index] = value;
        break;
    case m2_jointParamEnableMotor:
        j->jointFlags[index] = value != 0.0f ? (j->jointFlags[index] | M2_JOINT_MOTOR)
                                             : (j->jointFlags[index] & ~M2_JOINT_MOTOR);
        break;
    case m2_jointParamEnableLimit:
        j->jointFlags[index] = value != 0.0f ? (j->jointFlags[index] | M2_JOINT_LIMIT)
                                             : (j->jointFlags[index] & ~M2_JOINT_LIMIT);
        break;
    case m2_jointParamLower:
        j->jointLower[index] = value;
        break;
    case m2_jointParamBreakForce:
        j->jointBreakForce[index] = value;
        break;
    case m2_jointParamBreakTorque:
        j->jointBreakTorque[index] = value;
        break;
    case m2_jointParamHertz:
        j->jointHertz[index] = value;
        break;
    case m2_jointParamDamping:
        j->jointDamping[index] = value;
        break;
    case m2_jointParamAngularHertz:
        j->jointHertz2[index] = value;
        if (value == 0.0f)
        {
            j->jointSpringImpulse[index] = 0.0f; // disable drops memory
        }
        break;
    case m2_jointParamAngularDamping:
        j->jointDamping2[index] = value;
        break;
    case m2_jointParamLength:
        j->jointLength[index] = value;
        j->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        j->jointLowerImpulse[index] = 0.0f;
        j->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMinLength:
        j->jointLower[index] = value;
        j->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        j->jointLowerImpulse[index] = 0.0f;
        j->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamMaxLength:
        j->jointUpper[index] = value;
        j->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        j->jointLowerImpulse[index] = 0.0f;
        j->jointUpperImpulse[index] = 0.0f;
        break;
    case m2_jointParamGearRatio:
        j->jointLength[index] = value; // phase carries over on purpose
        break;
    case m2_jointParamPulleyRatio:
        // Recapture the rope total from current geometry so the
        // machine does not snap; drop memory like a distance retarget.
        j->jointRefAngle[index] =
            m2PulleyLiveLength(world, index, 0) + value * m2PulleyLiveLength(world, index, 1);
        j->jointLength[index] = value;
        j->jointImpulse[index] = (m2Vec2){0.0f, 0.0f};
        break;
    case m2_jointParamUpper:
        j->jointUpper[index] = value;
        break;
    default:
        M2_ASSERT(false); // JointParamValid admits no other channel
        break;
    }
}

// One journaled channel for every joint parameter. Refuses a stale id
// (world may be NULL) or a parameter outside the channel's contract.
bool m2SetJointParamInternal(m2World* world, m2JointId jointId, uint8_t param, float value)
{
    int32_t index = world != NULL ? m2JointSlotChecked(world, jointId) : -1;
    if (index < 0 || !JointParamValid(world->joints.jointType[index], param, value))
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpJointParam record;
        memset(&record, 0, sizeof(record));
        record.joint = jointId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opSetJointParam, &record, (int32_t)sizeof(record));
    }
    ApplyJointParam(world, index, param, value);
    // Any parameter change wakes both ends.
    int32_t bodyA = world->joints.jointBodyA[index];
    int32_t bodyB = world->joints.jointBodyB[index];
    m2WakeIfDynamic(world, bodyA);
    m2WakeIfDynamic(world, bodyB);
    return true;
}

void m2Joint_SetMotorSpeed(m2World* world, m2JointId jointId, float speed)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamMotorSpeed, speed);
}

void m2Joint_SetMaxMotor(m2World* world, m2JointId jointId, float maxTorqueOrForce)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamMaxMotor, maxTorqueOrForce);
}

void m2Joint_EnableMotor(m2World* world, m2JointId jointId, bool enable)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamEnableMotor, enable ? 1.0f : 0.0f);
}

void m2Joint_EnableLimit(m2World* world, m2JointId jointId, bool enable)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamEnableLimit, enable ? 1.0f : 0.0f);
}

void m2Joint_SetLimits(m2World* world, m2JointId jointId, float lower, float upper)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (!(lower <= upper) || !m2FiniteF(lower) || !m2FiniteF(upper))
    {
        m2Refuse(world, m2_errorInvalid); // both or neither: never half a range
        return;
    }
    if (m2SetJointParamInternal(world, jointId, m2_jointParamLower, lower))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamUpper, upper);
    }
}

void m2Joint_SetSpringHertz(m2World* world, m2JointId jointId, float hertz)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamHertz, hertz);
}

void m2Joint_SetSpringDampingRatio(m2World* world, m2JointId jointId, float dampingRatio)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamDamping, dampingRatio);
}

void m2Joint_SetAngularSpringHertz(m2World* world, m2JointId jointId, float hertz)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamAngularHertz, hertz);
}

void m2Joint_SetAngularSpringDampingRatio(m2World* world, m2JointId jointId, float dampingRatio)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetJointParamInternal(world, jointId, m2_jointParamAngularDamping, dampingRatio);
}

void m2Joint_SetBreakLimits(m2World* world, m2JointId jointId, float maxForce, float maxTorque)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (!(maxForce >= 0.0f) || !(maxTorque >= 0.0f))
    {
        m2Refuse(world, m2_errorInvalid); // both or neither
        return;
    }
    if (m2SetJointParamInternal(world, jointId, m2_jointParamBreakForce, maxForce))
    {
        m2SetJointParamInternal(world, jointId, m2_jointParamBreakTorque, maxTorque);
    }
}

m2Result m2DestroyJoint(m2World* world, m2JointId jointId)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    int32_t index = m2JointSlotChecked(world, jointId);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    m2JournalRecord(world, m2_opDestroyJoint, &jointId, (int32_t)sizeof(jointId));
    m2DestroyJointInternal(world, index);
    return m2_success;
}

// The guts, shared with the solver's break pass (which must not
// journal: breaking is derived from state and replays by itself).
void m2DestroyJointInternal(m2World* world, int32_t index)
{
    // Both ends wake: a constraint vanished.
    int32_t bodyA = world->joints.jointBodyA[index];
    int32_t bodyB = world->joints.jointBodyB[index];
    m2WakeIfDynamic(world, bodyA);
    m2WakeIfDynamic(world, bodyB);
    world->joints.jointAlive[index] = 0;
    m2UnlinkJoint(world, index);
    if (world->joints.jointCollide[index] == 0)
    {
        m2RefilterJointedBodies(world, bodyA, bodyB); // pairs may return
    }
    if (world->joints.jointGenerations[index] == UINT16_MAX)
    {
        world->joints.jointRetiredCount += 1;
        return;
    }
    world->joints.jointGenerations[index] += 1;
    world->joints.jointFreeQueue[world->joints.jointFreeTail] = index;
    world->joints.jointFreeTail = (world->joints.jointFreeTail + 1) % world->joints.jointCapacity;
    world->joints.jointFreeCount += 1;
}

int32_t m2TypedJointSlot(const m2World* world, m2JointId jointId, uint8_t type)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation ||
        world->joints.jointType[index] != type)
    {
        m2Refuse(world, m2_errorInvalid); // stale id or wrong type on a typed path
        return -1;
    }
    return index;
}

bool m2Joint_GetCollideConnected(const m2World* world, m2JointId jointId)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->joints.jointCollide[index] != 0;
}

float m2Joint_GetReactionForce(const m2World* world, m2JointId jointId)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return force;
}

float m2Joint_GetReactionTorque(const m2World* world, m2JointId jointId)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->joints.jointCapacity ||
        world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    float force = 0.0f;
    float torque = 0.0f;
    m2JointReactionMagnitudes(world, index, world->lastInvH, &force, &torque);
    return torque;
}

bool m2Joint_IsValid(const m2World* world, m2JointId jointId)
{
    if (world == NULL)
    {
        return false;
    }
    int32_t index = (int32_t)jointId.index1 - 1;
    return index >= 0 && index < world->joints.jointCapacity &&
           world->joints.jointAlive[index] != 0 &&
           world->joints.jointGenerations[index] == jointId.generation;
}

// Introspection and enumeration: pure readers for the
// editor and engine-integration walk. Every list is ascending slot
// order (the canonical order everywhere else in Maul) and returns the
// TRUE total even when it exceeds capacity, so callers can size and
// retry instead of silently missing objects.

int32_t m2JointSlotChecked(const m2World* world, m2JointId jointId)
{
    int32_t index = (int32_t)jointId.index1 - 1;
    if (index < 0 || index >= world->joints.jointCapacity || world->joints.jointAlive[index] == 0 ||
        world->joints.jointGenerations[index] != jointId.generation)
    {
        return -1;
    }
    return index;
}

m2JointType m2Joint_GetType(const m2World* world, m2JointId jointId)
{
    int32_t index = world != NULL ? m2JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_distanceJoint;
    }
    return (m2JointType)world->joints.jointType[index];
}

m2BodyId m2Joint_GetBodyA(const m2World* world, m2JointId jointId)
{
    int32_t index = world != NULL ? m2JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t b = world->joints.jointBodyA[index];
    m2BodyId id = {(uint32_t)(b + 1), world->bodies.generations[b]};
    return id;
}

m2BodyId m2Joint_GetBodyB(const m2World* world, m2JointId jointId)
{
    int32_t index = world != NULL ? m2JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t b = world->joints.jointBodyB[index];
    m2BodyId id = {(uint32_t)(b + 1), world->bodies.generations[b]};
    return id;
}

// Joint parameter readback: with these, a world is
// reconstructible from public getters alone; the mirror test in
// test_world.c holds that promise to hash equality.

static int32_t JointSlotRefusing(const m2World* world, m2JointId jointId)
{
    int32_t index = world != NULL ? m2JointSlotChecked(world, jointId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
    }
    return index;
}

// Like JointSlotRefusing, but the joint must also be one of the
// kinds in the mask (bit n = m2JointType n). Refuses exactly once.
static int32_t JointSlotOfKind(const m2World* world, m2JointId jointId, uint32_t kindMask)
{
    int32_t index = JointSlotRefusing(world, jointId);
    if (index >= 0 && (kindMask & (1u << world->joints.jointType[index])) == 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return -1;
    }
    return index;
}

m2Vec2 m2Joint_GetLocalAnchorA(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAnchorA[index] : zero;
}

m2Vec2 m2Joint_GetLocalAnchorB(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAnchorB[index] : zero;
}

m2Vec2 m2Joint_GetLocalAxisA(const m2World* world, m2JointId jointId)
{
    int32_t index =
        JointSlotOfKind(world, jointId, (1u << m2_prismaticJoint) | (1u << m2_wheelJoint));
    m2Vec2 zero = {0.0f, 0.0f};
    if (index < 0)
    {
        return zero;
    }
    return world->joints.jointLocalAxisA[index];
}

float m2Joint_GetLength(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, 1u << m2_distanceJoint);
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointLength[index];
}

float m2Joint_GetHertz(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointHertz[index] : 0.0f;
}

float m2Joint_GetDampingRatio(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointDamping[index] : 0.0f;
}

float m2Joint_GetAngularHertz(const m2World* world, m2JointId jointId)
{
    int32_t index =
        JointSlotOfKind(world, jointId, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointHertz2[index];
}

float m2Joint_GetAngularDampingRatio(const m2World* world, m2JointId jointId)
{
    int32_t index =
        JointSlotOfKind(world, jointId, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointDamping2[index];
}

float m2Joint_GetMotorSpeed(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointMotorSpeed[index] : 0.0f;
}

float m2Joint_GetMaxMotor(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointMaxMotor[index] : 0.0f;
}

bool m2Joint_IsMotorEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_MOTOR) != 0;
}

bool m2Joint_IsLimitEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_LIMIT) != 0;
}

bool m2Joint_IsSpringEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_SPRING) != 0;
}

void m2Joint_GetLimits(const m2World* world, m2JointId jointId, float* lowerOut, float* upperOut)
{
    int32_t index = JointSlotRefusing(world, jointId);
    if (lowerOut != NULL)
    {
        *lowerOut = index >= 0 ? world->joints.jointLower[index] : 0.0f;
    }
    if (upperOut != NULL)
    {
        *upperOut = index >= 0 ? world->joints.jointUpper[index] : 0.0f;
    }
}

void m2Joint_GetBreakLimits(const m2World* world, m2JointId jointId, float* maxForceOut,
                            float* maxTorqueOut)
{
    int32_t index = JointSlotRefusing(world, jointId);
    if (maxForceOut != NULL)
    {
        *maxForceOut = index >= 0 ? world->joints.jointBreakForce[index] : 0.0f;
    }
    if (maxTorqueOut != NULL)
    {
        *maxTorqueOut = index >= 0 ? world->joints.jointBreakTorque[index] : 0.0f;
    }
}

// Spring-named getter aliases: the setters say Spring, the readers
// now can too. Same slots, same validation.
float m2Joint_GetSpringHertz(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetHertz(world, jointId);
}

float m2Joint_GetSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetDampingRatio(world, jointId);
}

float m2Joint_GetAngularSpringHertz(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetAngularHertz(world, jointId);
}

float m2Joint_GetAngularSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetAngularDampingRatio(world, jointId);
}
