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

static M2_NOINLINE float DistanceJointGetLengthWorker(const m2World* world, m2JointId jointId)
{
    int32_t index = JointSlotOfKind(world, jointId, 1u << m2_distanceJoint);
    if (index < 0)
    {
        return 0.0f;
    }
    return world->joints.jointLength[index];
}

float m2DistanceJoint_GetLength(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = DistanceJointGetLengthWorker(world, jointId);
    m2FloatEnvLeave(env);
    return result;
}

// Reads one stiffness value a public tuning names, from the pair it uses
// on this kind.
static M2_NOINLINE float JointGetTuningWorker(const m2World* world, m2JointId jointId,
                                              m2JointTuning tuning, bool damping)
{
    bool second = false;
    int32_t index = m2JointTuningSlot(world, jointId, tuning, &second);
    if (index < 0)
    {
        return 0.0f;
    }
    const m2Joints* j = &world->joints;
    if (second)
    {
        return damping ? j->jointDamping2[index] : j->jointHertz2[index];
    }
    return damping ? j->jointDamping[index] : j->jointHertz[index];
}

float m2Joint_GetConstraintHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneConstraint, false);
    m2FloatEnvLeave(env);
    return result;
}

float m2Joint_GetConstraintDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneConstraint, true);
    m2FloatEnvLeave(env);
    return result;
}

float m2Joint_GetSpringHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneSpring, false);
    m2FloatEnvLeave(env);
    return result;
}

float m2Joint_GetSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneSpring, true);
    m2FloatEnvLeave(env);
    return result;
}

float m2WeldJoint_GetLinearSpringHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneWeldLinear, false);
    m2FloatEnvLeave(env);
    return result;
}

float m2WeldJoint_GetLinearSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneWeldLinear, true);
    m2FloatEnvLeave(env);
    return result;
}

float m2WeldJoint_GetAngularSpringHertz(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneWeldAngular, false);
    m2FloatEnvLeave(env);
    return result;
}

float m2WeldJoint_GetAngularSpringDampingRatio(const m2World* world, m2JointId jointId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = JointGetTuningWorker(world, jointId, m2_tuneWeldAngular, true);
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
    int32_t index = JointSlotOfKind(world, jointId, M2_SPRING_KINDS);
    if (index < 0)
    {
        return false;
    }
    uint32_t bit = world->joints.jointType[index] == (uint8_t)m2_distanceJoint ? M2_JOINT_ROPE
                                                                               : M2_JOINT_SPRING;
    return (world->joints.jointFlags[index] & bit) != 0;
}

static M2_NOINLINE void JointGetLimitsWorker(const m2World* world, m2JointId jointId,
                                             float* lowerOut, float* upperOut)
{
    int32_t index = JointSlotOfKind(world, jointId, M2_MOTOR_LIMIT_KINDS);
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

static M2_NOINLINE void DistanceJointGetLengthRangeWorker(const m2World* world, m2JointId jointId,
                                                          float* minLengthOut, float* maxLengthOut)
{
    int32_t index = JointSlotOfKind(world, jointId, 1u << m2_distanceJoint);
    if (minLengthOut != NULL)
    {
        *minLengthOut = index >= 0 ? world->joints.jointLower[index] : 0.0f;
    }
    if (maxLengthOut != NULL)
    {
        *maxLengthOut = index >= 0 ? world->joints.jointUpper[index] : 0.0f;
    }
}

void m2DistanceJoint_GetLengthRange(const m2World* world, m2JointId jointId, float* minLengthOut,
                                    float* maxLengthOut)
{
    uint64_t env = m2FloatEnvEnter();
    DistanceJointGetLengthRangeWorker(world, jointId, minLengthOut, maxLengthOut);
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
