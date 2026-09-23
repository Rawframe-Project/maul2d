// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Body motion: velocities, impulses, forces and teleports.

#include "body.h"

#include "broadphase.h"
#include "joint.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !m2FiniteVec2(velocity))
    {
        m2Refuse(world, m2_errorInvalid);
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

void m2Body_SetAngularVelocity(m2BodyId bodyId, float velocity)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !m2FiniteF(velocity))
    {
        m2Refuse(world, m2_errorInvalid);
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

void m2Body_ApplyLinearImpulse(m2BodyId bodyId, m2Vec2 impulse, m2Pos2 worldPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2FiniteVec2(impulse) || !m2FinitePos2(worldPoint))
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
    // Arm from the center of mass; the single f64 crossing.
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 rlc = {xf.q.c * world->bodies.localCenters[index].x -
                      xf.q.s * world->bodies.localCenters[index].y,
                  xf.q.s * world->bodies.localCenters[index].x +
                      xf.q.c * world->bodies.localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->bodies.linearVelocities[index].x += world->bodies.invMass[index] * impulse.x;
    world->bodies.linearVelocities[index].y += world->bodies.invMass[index] * impulse.y;
    world->bodies.angularVelocities[index] +=
        world->bodies.invInertia[index] * (r.x * impulse.y - r.y * impulse.x);
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForce(m2BodyId bodyId, m2Vec2 force, m2Pos2 worldPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2FiniteVec2(force) || !m2FinitePos2(worldPoint))
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
    // Arm from the center of mass, exactly like the impulse path.
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 rlc = {xf.q.c * world->bodies.localCenters[index].x -
                      xf.q.s * world->bodies.localCenters[index].y,
                  xf.q.s * world->bodies.localCenters[index].x +
                      xf.q.c * world->bodies.localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->bodies.forces[index].x += force.x;
    world->bodies.forces[index].y += force.y;
    world->bodies.torques[index] += r.x * force.y - r.y * force.x;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForceToCenter(m2BodyId bodyId, m2Vec2 force)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody || !m2FiniteVec2(force))
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
    world->bodies.forces[index].x += force.x;
    world->bodies.forces[index].y += force.y;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyTorque(m2BodyId bodyId, float torque)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody || !m2FiniteF(torque))
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
    world->bodies.torques[index] += torque;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody || !m2FiniteF(impulse))
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
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_ApplyLinearImpulseToCenter(m2BodyId bodyId, m2Vec2 impulse)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !m2FiniteVec2(impulse))
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
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_SetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation)
{
    m2World* world = m2GetBodyWorld(bodyId);
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
        m2AABB tight = m2ShapeTightAABB(world, shape);
        int32_t tree = m2ShapeTreeIndex(world, shape);
        if (!m2AABB_Contains(
                world->broadphase.treeNodes[tree][world->broadphase.proxyIds[shape]].aabb, tight))
        {
            m2TreeMove(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                       world->broadphase.proxyIds[shape], m2Fatten(tight));
        }
        m2PushMoved(world, shape);
    }
}

void m2Body_SetTargetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation, float dt)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !(dt > 0.0f) || !m2FinitePos2(position) || !m2UnitRot(rotation) ||
        !m2FiniteF(dt))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Velocities that land the pose in one step; applied through the
    // journaled setters, so replays get this for free.
    float invDt = 1.0f / dt;
    m2Transform xf = world->bodies.transforms[index];
    m2Vec2 v = {(float)(position.x - xf.p.x) * invDt, (float)(position.y - xf.p.y) * invDt};
    float w = m2UnwindAngle(m2RelativeJointAngle(xf.q, rotation)) * invDt;
    m2Body_SetLinearVelocity(bodyId, v);
    m2Body_SetAngularVelocity(bodyId, w);
}
