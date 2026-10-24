// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint readback: anchors, axes, lengths, springs, motors, limits and
// break limits. Pure readers.

#include "fpenv.h"
#include "joint.h"
#include "world_internal.h"

#include "maul2d/base.h"

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

static M2_NOINLINE m2Vec2 JointGetLocalAnchorAWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAnchorA[index] : zero;
}

m2Vec2 m2Joint_GetLocalAnchorA(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = JointGetLocalAnchorAWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 JointGetLocalAnchorBWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    m2Vec2 zero = {0.0f, 0.0f};
    return index >= 0 ? world->joints.jointLocalAnchorB[index] : zero;
}

m2Vec2 m2Joint_GetLocalAnchorB(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = JointGetLocalAnchorBWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 JointGetLocalAxisAWorker(const m2World* world, m2JointId jointId)
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

m2Vec2 m2Joint_GetLocalAxisA(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = JointGetLocalAxisAWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetLengthWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, 1u << m2_distanceJoint);
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointLength[index];
}

float m2Joint_GetLength(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetLengthWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetHertzWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointHertz[index] : 0.0f;
}

float m2Joint_GetHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetHertzWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetDampingRatioWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 ? world->joints.jointDamping[index] : 0.0f;
}

float m2Joint_GetDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetDampingRatioWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetAngularHertzWorker(const m2World* world, m2JointId jointId)
{
    int32_t index =
        JointSlotOfKind(world, jointId, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointHertz2[index];
}

float m2Joint_GetAngularHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetAngularHertzWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetAngularDampingRatioWorker(const m2World* world, m2JointId jointId)
{
    int32_t index =
        JointSlotOfKind(world, jointId, (1u << m2_weldJoint) | (1u << m2_revoluteJoint));
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointDamping2[index];
}

float m2Joint_GetAngularDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetAngularDampingRatioWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetMotorSpeedWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, M2_MOTOR_LIMIT_KINDS);
    return index >= 0 ? world->joints.jointMotorSpeed[index] : 0.0f;
}

float m2Joint_GetMotorSpeed(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetMotorSpeedWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetMaxMotorWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, M2_MAX_MOTOR_KINDS);
    return index >= 0 ? world->joints.jointMaxMotor[index] : 0.0f;
}

float m2Joint_GetMaxMotor(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetMaxMotorWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

bool m2Joint_IsMotorEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, M2_MOTOR_LIMIT_KINDS);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_MOTOR) != 0;
}

bool m2Joint_IsLimitEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, M2_MOTOR_LIMIT_KINDS);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_LIMIT) != 0;
}

bool m2Joint_IsSpringEnabled(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotRefusing(world, jointId);
    return index >= 0 && (world->joints.jointFlags[index] & M2_JOINT_SPRING) != 0;
}

static M2_NOINLINE void JointGetLimitsWorker(const m2World* world, m2JointId jointId,
                                             float* lowerOut, float* upperOut)
{
    // A distance joint reads its length range here.
    int32_t index =
        JointSlotOfKind(world, jointId, M2_MOTOR_LIMIT_KINDS | (1u << m2_distanceJoint));
    if (lowerOut != NULL)
    {
        *lowerOut = index >= 0 ? world->joints.jointLower[index] : 0.0f;
    }
    if (upperOut != NULL)
    {
        *upperOut = index >= 0 ? world->joints.jointUpper[index] : 0.0f;
    }
}

void m2Joint_GetLimits(const m2World* world, m2JointId jointId, float* lowerOut, float* upperOut)
{
    uint64_t env = m2FloatEnvEnter();
    JointGetLimitsWorker(world, jointId, lowerOut, upperOut);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void JointGetBreakLimitsWorker(const m2World* world, m2JointId jointId,
                                                  float* maxForceOut, float* maxTorqueOut)
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

void m2Joint_GetBreakLimits(const m2World* world, m2JointId jointId, float* maxForceOut,
                            float* maxTorqueOut)
{
    uint64_t env = m2FloatEnvEnter();
    JointGetBreakLimitsWorker(world, jointId, maxForceOut, maxTorqueOut);
    m2FloatEnvLeave(env);
}

// Spring-named getter aliases: the setters say Spring, the readers
// now can too. Same slots, same validation.
static M2_NOINLINE float JointGetSpringHertzWorker(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetHertz(world, jointId);
}

float m2Joint_GetSpringHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetSpringHertzWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetSpringDampingRatioWorker(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetDampingRatio(world, jointId);
}

float m2Joint_GetSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetSpringDampingRatioWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetAngularSpringHertzWorker(const m2World* world, m2JointId jointId)
{
    return m2Joint_GetAngularHertz(world, jointId);
}

float m2Joint_GetAngularSpringHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetAngularSpringHertzWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float JointGetAngularSpringDampingRatioWorker(const m2World* world,
                                                                 m2JointId jointId)
{
    return m2Joint_GetAngularDampingRatio(world, jointId);
}

float m2Joint_GetAngularSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetAngularSpringDampingRatioWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}
