// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body readback: mass, frames, velocities at points, bounds and the
// shapes and joints on a body. Pure readers.

#include "body.h"
#include "fpenv.h"
#include "object_ids.h"
#include "shape.h"
#include "world_internal.h"

#include "maul2d/base.h"

static M2_NOINLINE float BodyGetMassWorker(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->bodies.invMass[index] > 0.0f ? 1.0f / world->bodies.invMass[index] : 0.0f;
}

float m2Body_GetMass(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = BodyGetMassWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

bool m2Body_IsAwake(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->bodies.types[index] == (uint8_t)m2_dynamicBody ? world->bodies.asleep[index] == 0
                                                                 : true;
}

static M2_NOINLINE m2Pos2 BodyGetWorldPointWorker(const m2World* world, m2BodyId bodyId,
                                                  m2Vec2 localPoint)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 r = {xf.q.c * localPoint.x - xf.q.s * localPoint.y,
                xf.q.s * localPoint.x + xf.q.c * localPoint.y};
    return (m2Pos2){xf.p.x + (double)r.x, xf.p.y + (double)r.y};
}

m2Pos2 m2Body_GetWorldPoint(const m2World* world, m2BodyId bodyId, m2Vec2 localPoint)
{
    uint64_t env = m2FloatEnvEnter();
    m2Pos2 result = BodyGetWorldPointWorker(world, bodyId, localPoint);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 BodyGetLocalPointWorker(const m2World* world, m2BodyId bodyId,
                                                  m2Pos2 worldPoint)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 rel = {(float)(worldPoint.x - xf.p.x), (float)(worldPoint.y - xf.p.y)};
    return (m2Vec2){xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
}

m2Vec2 m2Body_GetLocalPoint(const m2World* world, m2BodyId bodyId, m2Pos2 worldPoint)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetLocalPointWorker(world, bodyId, worldPoint);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 BodyGetWorldVectorWorker(const m2World* world, m2BodyId bodyId,
                                                   m2Vec2 localVector)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->bodies.transforms[index].q;
    return (m2Vec2){q.c * localVector.x - q.s * localVector.y,
                    q.s * localVector.x + q.c * localVector.y};
}

m2Vec2 m2Body_GetWorldVector(const m2World* world, m2BodyId bodyId, m2Vec2 localVector)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetWorldVectorWorker(world, bodyId, localVector);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 BodyGetLocalVectorWorker(const m2World* world, m2BodyId bodyId,
                                                   m2Vec2 worldVector)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->bodies.transforms[index].q;
    return (m2Vec2){q.c * worldVector.x + q.s * worldVector.y,
                    -q.s * worldVector.x + q.c * worldVector.y};
}

m2Vec2 m2Body_GetLocalVector(const m2World* world, m2BodyId bodyId, m2Vec2 worldVector)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetLocalVectorWorker(world, bodyId, worldVector);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Vec2 BodyGetWorldPointVelocityWorker(const m2World* world, m2BodyId bodyId,
                                                          m2Pos2 worldPoint)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    // v + w x r, arm from the center of mass (one f64 crossing).
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 rlc = {xf.q.c * world->bodies.localCenters[index].x -
                      xf.q.s * world->bodies.localCenters[index].y,
                  xf.q.s * world->bodies.localCenters[index].x +
                      xf.q.c * world->bodies.localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    float w = world->bodies.angularVelocities[index];
    m2Vec2 v = world->bodies.linearVelocities[index];
    return (m2Vec2){v.x - w * r.y, v.y + w * r.x};
}

m2Vec2 m2Body_GetWorldPointVelocity(const m2World* world, m2BodyId bodyId, m2Pos2 worldPoint)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetWorldPointVelocityWorker(world, bodyId, worldPoint);
    m2FloatEnvLeave(env);
    return result;
}

int32_t m2Body_GetJoints(const m2World* world, m2BodyId bodyId, m2JointId* ids, int32_t capacity)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t e = world->joints.bodyJointHead[index]; e != -1;
         e = world->joints.jointEdgeNext[e])
    {
        int32_t j = e >> 1;
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {(uint32_t)(j + 1), world->joints.jointGenerations[j]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

static M2_NOINLINE m2Vec2 BodyGetLocalPointVelocityWorker(const m2World* world, m2BodyId bodyId,
                                                          m2Vec2 localPoint)
{
    return m2Body_GetWorldPointVelocity(world, bodyId,
                                        m2Body_GetWorldPoint(world, bodyId, localPoint));
}

m2Vec2 m2Body_GetLocalPointVelocity(const m2World* world, m2BodyId bodyId, m2Vec2 localPoint)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetLocalPointVelocityWorker(world, bodyId, localPoint);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Pos2 BodyGetWorldCenterOfMassWorker(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    return m2Body_GetWorldPoint(world, bodyId, world->bodies.localCenters[index]);
}

m2Pos2 m2Body_GetWorldCenterOfMass(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Pos2 result = BodyGetWorldCenterOfMassWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float BodyGetRotationalInertiaWorker(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    float invI = world->bodies.invInertia[index];
    return invI > 0.0f ? 1.0f / invI : 0.0f;
}

float m2Body_GetRotationalInertia(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = BodyGetRotationalInertiaWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2AabbResult BodyComputeAabbWorker(const m2World* world, m2BodyId bodyId)
{
    m2AabbResult result = {{0.0, 0.0}, {0.0, 0.0}};
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    result.lowerBound = world->bodies.transforms[index].p;
    result.upperBound = world->bodies.transforms[index].p;
    bool first = true;
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2Aabb tight =
            m2ComputeShapeAabb(&world->shapes.shapeGeometry[s], world->bodies.transforms[index]);
        if (first)
        {
            result.lowerBound = tight.lowerBound;
            result.upperBound = tight.upperBound;
            first = false;
            continue;
        }
        result.lowerBound.x =
            tight.lowerBound.x < result.lowerBound.x ? tight.lowerBound.x : result.lowerBound.x;
        result.lowerBound.y =
            tight.lowerBound.y < result.lowerBound.y ? tight.lowerBound.y : result.lowerBound.y;
        result.upperBound.x =
            tight.upperBound.x > result.upperBound.x ? tight.upperBound.x : result.upperBound.x;
        result.upperBound.y =
            tight.upperBound.y > result.upperBound.y ? tight.upperBound.y : result.upperBound.y;
    }
    return result;
}

m2AabbResult m2Body_ComputeAabb(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2AabbResult result = BodyComputeAabbWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

int8_t m2Body_GetDominance(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->bodies.dominances[index];
}

bool m2Body_IsEnabled(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->bodies.disabled[index] == 0;
}

static M2_NOINLINE m2MassData BodyGetMassDataWorker(const m2World* world, m2BodyId bodyId)
{
    m2MassData data = {0.0f, {0.0f, 0.0f}, 0.0f};
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return data;
    }
    float invMass = world->bodies.invMass[index];
    data.mass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
    data.center = world->bodies.localCenters[index];
    float invI = world->bodies.invInertia[index];
    data.centralInertia = invI > 0.0f ? 1.0f / invI : 0.0f;
    return data;
}

m2MassData m2Body_GetMassData(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2MassData result = BodyGetMassDataWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

m2BodyType m2Body_GetType(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_staticBody;
    }
    return (m2BodyType)world->bodies.types[index];
}

static M2_NOINLINE m2Vec2 BodyGetLocalCenterWorker(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        m2Vec2 zero = {0.0f, 0.0f};
        return zero;
    }
    return world->bodies.localCenters[index];
}

m2Vec2 m2Body_GetLocalCenter(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Vec2 result = BodyGetLocalCenterWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

bool m2Body_IsBullet(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->bodies.bullets[index] != 0;
}

static M2_NOINLINE float BodyGetGravityScaleWorker(const m2World* world, m2BodyId bodyId)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->bodies.gravityScales[index];
}

float m2Body_GetGravityScale(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = BodyGetGravityScaleWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

int32_t m2Body_GetShapes(const m2World* world, m2BodyId bodyId, m2ShapeId* ids, int32_t capacity)
{
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        if (world->shapes.shapeAlive[i] == 0 || world->shapes.shapeBody[i] != bodyIndex)
        {
            continue;
        }
        if (ids != NULL && total < capacity)
        {
            ids[total] = m2MakeShapeId(world, i);
        }
        total += 1;
    }
    return total;
}
