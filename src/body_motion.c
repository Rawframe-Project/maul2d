// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body motion: velocities, impulses, forces and teleports.

#include "body.h"
#include "fpenv.h"
#include "island.h"
#include "joint.h"
#include "joint_row.h"
#include "journal.h"
#include "math_functions.h"
#include "object_ids.h"
#include "proxies.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

// Adds a load to a body's accumulated force and torque, each held within
// M2_FORCE_LIMIT however many calls add up.
static void AddLoad(m2World* world, int32_t index, m2Vec2 force, float torque)
{
    m2Vec2* f = &world->bodies.forces[index];
    f->x = m2SaturateF(f->x + force.x, M2_FORCE_LIMIT);
    f->y = m2SaturateF(f->y + force.y, M2_FORCE_LIMIT);
    world->bodies.torques[index] =
        m2SaturateF(world->bodies.torques[index] + torque, M2_FORCE_LIMIT);
}

// The arm from a body's center of mass to a world point, each axis
// clamped to M2_REACH_LIMIT.
static m2Vec2 ArmToPoint(const m2World* world, int32_t index, m2Pos2 worldPoint)
{
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 rlc = {xf.q.c * world->bodies.localCenters[index].x -
                      xf.q.s * world->bodies.localCenters[index].y,
                  xf.q.s * world->bodies.localCenters[index].x +
                      xf.q.c * world->bodies.localCenters[index].y};
    return (m2Vec2){m2WorldOffset(worldPoint.x - xf.p.x) - rlc.x,
                    m2WorldOffset(worldPoint.y - xf.p.y) - rlc.y};
}

static M2_NOINLINE void BodySetLinearVelocityWorker(m2World* world, m2BodyId bodyId,
                                                    m2Vec2 velocity)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] == (uint8_t)m2_staticBody ||
        !m2ValidVelocity(velocity))
    {
        m2Refuse(world, m2_errorInvalid); // a static body never moves
        return;
    }
    world->bodies.linearVelocities[index] = velocity;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyVec record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetLinearVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_SetLinearVelocity(m2World* world, m2BodyId bodyId, m2Vec2 velocity)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetLinearVelocityWorker(world, bodyId, velocity);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetAngularVelocityWorker(m2World* world, m2BodyId bodyId,
                                                     float velocity)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] == (uint8_t)m2_staticBody ||
        !m2ValidSpeed(velocity))
    {
        m2Refuse(world, m2_errorInvalid); // a static body never moves
        return;
    }
    world->bodies.angularVelocities[index] = velocity;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyFloat record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = velocity;
        m2JournalRecord(world, m2_opSetAngularVelocity, &record, (int32_t)sizeof(record));
    }
}

void m2Body_SetAngularVelocity(m2World* world, m2BodyId bodyId, float velocity)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetAngularVelocityWorker(world, bodyId, velocity);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyLinearImpulseAtPointWorker(m2World* world, m2BodyId bodyId,
                                                            m2Vec2 impulse, m2Pos2 worldPoint)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2ValidImpulseVec(impulse) || !m2FinitePos2(worldPoint))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyPoint record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    m2Vec2 r = ArmToPoint(world, index, worldPoint);
    world->bodies.linearVelocities[index].x += world->bodies.invMass[index] * impulse.x;
    world->bodies.linearVelocities[index].y += world->bodies.invMass[index] * impulse.y;
    world->bodies.angularVelocities[index] +=
        world->bodies.invInertia[index] * (r.x * impulse.y - r.y * impulse.x);
    m2SaturateBodyVelocity(world, index);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyLinearImpulseAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 impulse,
                                      m2Pos2 worldPoint)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyLinearImpulseAtPointWorker(world, bodyId, impulse, worldPoint);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyForceAtPointWorker(m2World* world, m2BodyId bodyId, m2Vec2 force,
                                                    m2Pos2 worldPoint)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2ValidForceVec(force) || !m2FinitePos2(worldPoint))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyPoint record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = force;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyForce, &record, (int32_t)sizeof(record));
    }
    m2Vec2 r = ArmToPoint(world, index, worldPoint);
    AddLoad(world, index, force, r.x * force.y - r.y * force.x);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForceAtPoint(m2World* world, m2BodyId bodyId, m2Vec2 force, m2Pos2 worldPoint)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyForceAtPointWorker(world, bodyId, force, worldPoint);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyForceWorker(m2World* world, m2BodyId bodyId, m2Vec2 force)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2ValidForceVec(force))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyVec record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = force;
        m2JournalRecord(world, m2_opApplyForceCenter, &record, (int32_t)sizeof(record));
    }
    AddLoad(world, index, force, 0.0f);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForce(m2World* world, m2BodyId bodyId, m2Vec2 force)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyForceWorker(world, bodyId, force);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyTorqueWorker(m2World* world, m2BodyId bodyId, float torque)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody || !m2ValidForce(torque))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyFloat record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = torque;
        m2JournalRecord(world, m2_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    AddLoad(world, index, (m2Vec2){0.0f, 0.0f}, torque);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyTorque(m2World* world, m2BodyId bodyId, float torque)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyTorqueWorker(world, bodyId, torque);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyAngularImpulseWorker(m2World* world, m2BodyId bodyId,
                                                      float impulse)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2ValidImpulse(impulse))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyFloat record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        m2JournalRecord(world, m2_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    world->bodies.angularVelocities[index] += world->bodies.invInertia[index] * impulse;
    m2SaturateBodyVelocity(world, index);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyAngularImpulse(m2World* world, m2BodyId bodyId, float impulse)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyAngularImpulseWorker(world, bodyId, impulse);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyLinearImpulseWorker(m2World* world, m2BodyId bodyId,
                                                     m2Vec2 impulse)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2ValidImpulseVec(impulse))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyVec record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        m2JournalRecord(world, m2_opImpulseCenter, &record, (int32_t)sizeof(record));
    }
    world->bodies.linearVelocities[index].x += world->bodies.invMass[index] * impulse.x;
    world->bodies.linearVelocities[index].y += world->bodies.invMass[index] * impulse.y;
    m2SaturateBodyVelocity(world, index);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyLinearImpulse(m2World* world, m2BodyId bodyId, m2Vec2 impulse)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyLinearImpulseWorker(world, bodyId, impulse);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetTransformWorker(m2World* world, m2BodyId bodyId, m2Pos2 position,
                                               m2Rot rotation)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !m2FinitePos2(position) || !m2UnitRot(rotation))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpSetTransform record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.position = position;
        record.rotation = rotation;
        m2JournalRecord(world, m2_opSetTransform, &record, (int32_t)sizeof(record));
    }

    // Whatever this body was resting on - or holding up - must notice.
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = world->shapes.shapeBody[(int32_t)(world->contacts.pairKeys[i] >> 32)];
        int32_t b = world->shapes.shapeBody[(int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu)];
        if (a != index && b != index)
        {
            continue;
        }
        int32_t other = a == index ? b : a;
        m2WakeIfDynamic(world, other);
    }

    world->bodies.transforms[index].p = position;
    world->bodies.transforms[index].q = rotation;
    m2WakeIfDynamic(world, index);

    // Broadphase refresh right now: the next step's pair update must
    // see the new home, not the old one.
    for (int32_t shape = world->bodies.bodyShapeHead[index]; shape != -1;
         shape = world->shapes.shapeNext[shape])
    {
        if (world->broadphase.proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        (void)m2RefitProxy(world, shape);
        m2PushMoved(world, shape);
    }
}

void m2Body_SetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position, m2Rot rotation)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetTransformWorker(world, bodyId, position, rotation);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetTargetTransformWorker(m2World* world, m2BodyId bodyId,
                                                     m2Pos2 position, m2Rot rotation, float dt)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] == (uint8_t)m2_staticBody || !m2ValidTimeStep(dt) ||
        !m2FinitePos2(position) || !m2UnitRot(rotation))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Velocities that land the pose in one step; applied through the
    // journaled setters, so replays get this for free. A pose too far to
    // reach within M2_SPEED_LIMIT is refused before either is set.
    float invDt = 1.0f / dt;
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 v = {m2WorldOffset(position.x - xf.p.x) * invDt,
                m2WorldOffset(position.y - xf.p.y) * invDt};
    float w = m2UnwindAngleWork(m2RelativeJointAngle(xf.q, rotation)) * invDt;
    if (!m2ValidVelocity(v) || !m2ValidSpeed(w))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2Body_SetLinearVelocity(world, bodyId, v);
    m2Body_SetAngularVelocity(world, bodyId, w);
}

void m2Body_SetTargetTransform(m2World* world, m2BodyId bodyId, m2Pos2 position, m2Rot rotation,
                               float dt)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetTargetTransformWorker(world, bodyId, position, rotation, dt);
    m2FloatEnvLeave(env);
}
