// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation and destruction, motion, forces and impulses, mass,
// sleep, enable and disable, and body readback.

#include "body.h"

#include "broadphase.h"
#include "chain.h"
#include "joint.h"
#include "shape.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

m2World* m2GetBodyWorld(m2BodyId id)
{
    return m2WorldFromIndex(id.world0);
}

int32_t m2BodySlot(const m2World* world, m2BodyId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->bodyCapacity)
    {
        return -1;
    }
    if (world->alive[index] == 0 || world->generations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

// --- Mass ---------------------------------------------------------------------

// Deterministic: walks the body's shape list in stored (insertion) order,
// which the snapshot preserves. Dynamic bodies get the minimum-mass floor
// (RT1-NUM-2: a zero-density dynamic body must not mint an infinite
// inverse mass).
void m2RecomputeMass(m2World* world, int32_t bodyIndex)
{
    if (world->types[bodyIndex] != (uint8_t)m2_dynamicBody)
    {
        world->invMass[bodyIndex] = 0.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    // First pass: total mass and the mass-weighted center. Shape mass now
    // reports inertia about the SHAPE centroid, so the shift to the body
    // COM below is a sum of non-negative parallel-axis terms with no
    // big-minus-big cancellation (reference b2 #955).
    float mass = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        m2MassData data = m2ComputeShapeMass(&world->shapeGeometry[s], world->shapeDensity[s]);
        mass += data.mass;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        world->invMass[bodyIndex] = 1.0f;
        world->invInertia[bodyIndex] = 0.0f;
        world->localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;

    // Second pass: accumulate inertia about the body COM. Each shape's
    // centroid inertia plus mass times the (small) offset from the COM,
    // in the same canonical shape-list order.
    float inertiaCenter = 0.0f;
    for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
    {
        m2MassData data = m2ComputeShapeMass(&world->shapeGeometry[s], world->shapeDensity[s]);
        float ox = center.x - data.center.x;
        float oy = center.y - data.center.y;
        inertiaCenter += data.rotationalInertia + data.mass * (ox * ox + oy * oy);
    }
    world->invMass[bodyIndex] = invMass;
    world->invInertia[bodyIndex] =
        world->fixedRotations[bodyIndex] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->localCenters[bodyIndex] = center;
}

// --- Bodies ---------------------------------------------------------------------

m2BodyDef m2DefaultBodyDef(void)
{
    m2BodyDef def;
    memset(&def, 0, sizeof(def));
    def.type = m2_staticBody;
    def.rotation = (m2Rot){1.0f, 0.0f};
    def.gravityScale = 1.0f;
    def.enableSleep = true;
    def.isEnabled = true;
    def.internalValue = M2_BODY_COOKIE;
    return def;
}

m2BodyId m2CreateBody(m2WorldId worldId, const m2BodyDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_BODY_COOKIE)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    if (world->freeCount == 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullBodyId;
    }

    int32_t index = world->freeQueue[world->freeHead];
    world->freeHead = (world->freeHead + 1) % world->bodyCapacity;
    world->freeCount -= 1;

    world->transforms[index].p = def->position;
    world->transforms[index].q = m2NormalizeRot(def->rotation);
    world->linearVelocities[index] = def->linearVelocity;
    // A rotation-locked body never spins, so a fixed-rotation (or
    // angularZ-locked) body drops any initial angular velocity at birth,
    // the same as the runtime setter does.
    world->angularVelocities[index] =
        (def->fixedRotation || def->motionLocks.angularZ) ? 0.0f : def->angularVelocity;
    world->gravityScales[index] = def->gravityScale;
    world->linearDampings[index] = def->linearDamping;
    world->angularDampings[index] = def->angularDamping;
    world->fixedRotations[index] = (def->fixedRotation || def->motionLocks.angularZ) ? 1 : 0;
    world->motionLocks[index] =
        (uint8_t)((def->motionLocks.linearX ? 1u : 0u) | (def->motionLocks.linearY ? 2u : 0u));
    world->sleepEnables[index] = def->enableSleep ? 1 : 0;
    world->forces[index] = (m2Vec2){0.0f, 0.0f};
    world->torques[index] = 0.0f;
    world->disabled[index] = def->isEnabled ? 0 : 1;
    world->dominances[index] = def->dominance;
    world->userData[index] = def->userData;
    world->types[index] = (uint8_t)def->type;
    world->alive[index] = 1;
    world->bodyShapeHead[index] = -1;
    world->invMass[index] = def->type == m2_dynamicBody ? 1.0f : 0.0f; // shapeless floor
    world->localCenters[index] = (m2Vec2){0.0f, 0.0f};
    world->invInertia[index] = 0.0f;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    world->sleepStreak[index] = 0;
    world->bullets[index] = def->isBullet ? 1 : 0;
    if (index + 1 > world->maxBodyIndex)
    {
        world->maxBodyIndex = index + 1;
    }

    m2BodyId id = {index + 1, worldId.index1, world->generations[index]};

    if (world->journalActive != 0)

    {

        struct

        {

            m2BodyDef def;

            m2BodyId expected;

        } record;
        memset(&record, 0, sizeof(record));
        record.def = *def;
        record.expected = id;

        m2JournalRecord(world, m2_opCreateBody, &record, (int32_t)sizeof(record));
    }
    return id;
}

void m2DestroyBody(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world == NULL)
    {
        return;
    }
    int32_t index = m2BodySlot(world, bodyId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opDestroyBody, &bodyId, (int32_t)sizeof(bodyId));

    // Joints attached to this body die with it; the counterpart body
    // wakes (a support vanished).
    // The body's list is in ascending joint order, the order a full
    // scan would free the slots in.
    while (world->bodyJointHead[index] != -1)
    {
        int32_t j = world->bodyJointHead[index] >> 1;
        int32_t other = world->jointBodyA[j] == index ? world->jointBodyB[j] : world->jointBodyA[j];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
        world->jointAlive[j] = 0;
        m2UnlinkJoint(world, j);
        if (world->jointGenerations[j] == UINT16_MAX)
        {
            world->jointRetiredCount += 1;
        }
        else
        {
            world->jointGenerations[j] += 1;
            world->jointFreeQueue[world->jointFreeTail] = j;
            world->jointFreeTail = (world->jointFreeTail + 1) % world->jointCapacity;
            world->jointFreeCount += 1;
        }
    }

    // Chains die with their body; their slots retire so stale chain
    // ids miss on generation, same as every other id kind.
    for (int32_t c = 0; c < world->maxChainIndex; ++c)
    {
        if (world->chainAlive[c] != 0 && world->chainBody[c] == index)
        {
            m2RetireChainSlot(world, c);
        }
    }

    // Cascade: destroy the shape list (the contact-killing path that event
    // bookending will hang end-touch emission on, registry M19).
    int32_t s = world->bodyShapeHead[index];
    while (s != -1)
    {
        int32_t next = world->shapeNext[s];
        m2DestroyShapeInternal(world, s);
        s = next;
    }
    world->bodyShapeHead[index] = -1;

    world->alive[index] = 0;
    if (world->generations[index] == UINT16_MAX)
    {
        world->retiredCount += 1;
        return;
    }
    world->generations[index] += 1;
    world->freeQueue[world->freeTail] = index;
    world->freeTail = (world->freeTail + 1) % world->bodyCapacity;
    world->freeCount += 1;
}

bool m2Body_IsValid(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    return world != NULL && m2BodySlot(world, bodyId) >= 0;
}

#define M2_BODY_GETTER(returnType, name, expr, fallback)                                           \
    returnType name(m2BodyId bodyId)                                                               \
    {                                                                                              \
        m2World* world = m2GetBodyWorld(bodyId);                                                   \
        int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;                            \
        if (index < 0)                                                                             \
        {                                                                                          \
            m2Refuse(world, m2_errorInvalid);                                                      \
            return fallback;                                                                       \
        }                                                                                          \
        return expr;                                                                               \
    }

M2_BODY_GETTER(m2Transform, m2Body_GetTransform, world->transforms[index],
               ((m2Transform){{0.0, 0.0}, {1.0f, 0.0f}}))

M2_BODY_GETTER(m2Pos2, m2Body_GetPosition, world->transforms[index].p, ((m2Pos2){0.0, 0.0}))

M2_BODY_GETTER(m2Rot, m2Body_GetRotation, world->transforms[index].q, ((m2Rot){1.0f, 0.0f}))

M2_BODY_GETTER(m2Vec2, m2Body_GetLinearVelocity, world->linearVelocities[index],
               ((m2Vec2){0.0f, 0.0f}))

M2_BODY_GETTER(float, m2Body_GetAngularVelocity, world->angularVelocities[index], 0.0f)

M2_BODY_GETTER(uint64_t, m2Body_GetUserData, world->userData[index], 0)

float m2Body_GetMass(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->invMass[index] > 0.0f ? 1.0f / world->invMass[index] : 0.0f;
}

void m2Body_SetLinearVelocity(m2BodyId bodyId, m2Vec2 velocity)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    world->linearVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 value;
        } record;
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
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    world->angularVelocities[index] = velocity;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
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
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 impulse;
            m2Pos2 point;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.impulse = impulse;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyLinearImpulse, &record, (int32_t)sizeof(record));
    }
    // Arm from the center of mass; the single f64 crossing.
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->linearVelocities[index].x += world->invMass[index] * impulse.x;
    world->linearVelocities[index].y += world->invMass[index] * impulse.y;
    world->angularVelocities[index] +=
        world->invInertia[index] * (r.x * impulse.y - r.y * impulse.x);
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForce(m2BodyId bodyId, m2Vec2 force, m2Pos2 worldPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 force;
            m2Pos2 point;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.force = force;
        record.point = worldPoint;
        m2JournalRecord(world, m2_opApplyForce, &record, (int32_t)sizeof(record));
    }
    // Arm from the center of mass, exactly like the impulse path.
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    world->forces[index].x += force.x;
    world->forces[index].y += force.y;
    world->torques[index] += r.x * force.y - r.y * force.x;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyForceToCenter(m2BodyId bodyId, m2Vec2 force)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 force;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.force = force;
        m2JournalRecord(world, m2_opApplyForceCenter, &record, (int32_t)sizeof(record));
    }
    world->forces[index].x += force.x;
    world->forces[index].y += force.y;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_ApplyTorque(m2BodyId bodyId, float torque)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float torque;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.torque = torque;
        m2JournalRecord(world, m2_opApplyTorque, &record, (int32_t)sizeof(record));
    }
    world->torques[index] += torque;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

// One journaled channel for the body dynamics parameters, mirroring
// the shape and joint channels.
void m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value)
{
    int32_t index = m2BodySlot(world, bodyId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct m2OpBodyParam
        {
            m2BodyId body;
            float value;
            uint8_t param;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opBodyParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case 0:
        world->linearDampings[index] = value;
        break;
    case 1:
        world->angularDampings[index] = value;
        break;
    case 2:
        world->gravityScales[index] = value;
        break;
    case 3:
        // Fixed rotation is a mass property: inertia recomputes, spin
        // stops now (a frozen axis with leftover spin is a lie).
        world->fixedRotations[index] = value != 0.0f ? 1 : 0;
        world->angularVelocities[index] = 0.0f;
        m2RecomputeMass(world, index);
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 4:
        world->sleepEnables[index] = value != 0.0f ? 1 : 0;
        if (value == 0.0f && world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0; // must not stay asleep illegally
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 5:
        // Lock linear X: a locked axis holds still, so stop it now and
        // wake the body (a frozen axis with leftover velocity is a lie,
        // same discipline as fixed rotation).
        world->motionLocks[index] = value != 0.0f ? (uint8_t)(world->motionLocks[index] | 1u)
                                                  : (uint8_t)(world->motionLocks[index] & ~1u);
        world->linearVelocities[index].x = value != 0.0f ? 0.0f : world->linearVelocities[index].x;
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    case 6:
        // Lock linear Y.
        world->motionLocks[index] = value != 0.0f ? (uint8_t)(world->motionLocks[index] | 2u)
                                                  : (uint8_t)(world->motionLocks[index] & ~2u);
        world->linearVelocities[index].y = value != 0.0f ? 0.0f : world->linearVelocities[index].y;
        if (world->types[index] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[index] = 0;
            world->sleepTimes[index] = 0.0f;
        }
        break;
    default:
        M2_ASSERT(false); // unknown body param
        break;
    }
}

void m2Body_SetLinearDamping(m2BodyId bodyId, float damping)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 0, m2MaxF(damping, 0.0f));
    }
}

float m2Body_GetLinearDamping(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->linearDampings[index] : 0.0f;
}

void m2Body_SetAngularDamping(m2BodyId bodyId, float damping)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 1, m2MaxF(damping, 0.0f));
    }
}

float m2Body_GetAngularDamping(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->angularDampings[index] : 0.0f;
}

void m2Body_SetGravityScale(m2BodyId bodyId, float scale)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 2, scale);
    }
}

void m2Body_SetFixedRotation(m2BodyId bodyId, bool flag)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 3, flag ? 1.0f : 0.0f);
    }
}

bool m2Body_IsFixedRotation(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->fixedRotations[index] != 0;
}

void m2Body_SetMotionLocks(m2BodyId bodyId, m2MotionLocks locks)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        // Each axis rides the journaled body-param channel so a replay
        // reproduces the locks. angularZ is the fixed-rotation lock.
        m2SetBodyParamInternal(world, bodyId, 5, locks.linearX ? 1.0f : 0.0f);
        m2SetBodyParamInternal(world, bodyId, 6, locks.linearY ? 1.0f : 0.0f);
        m2SetBodyParamInternal(world, bodyId, 3, locks.angularZ ? 1.0f : 0.0f);
    }
}

m2MotionLocks m2Body_GetMotionLocks(m2BodyId bodyId)
{
    m2MotionLocks locks = {false, false, false};
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index >= 0)
    {
        locks.linearX = (world->motionLocks[index] & 1u) != 0;
        locks.linearY = (world->motionLocks[index] & 2u) != 0;
        locks.angularZ = world->fixedRotations[index] != 0;
    }
    return locks;
}

void m2Body_EnableSleep(m2BodyId bodyId, bool flag)
{
    m2World* world = m2GetBodyWorld(bodyId);
    if (world != NULL)
    {
        m2SetBodyParamInternal(world, bodyId, 4, flag ? 1.0f : 0.0f);
    }
}

bool m2Body_IsSleepEnabled(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->sleepEnables[index] != 0;
}

void m2Body_ApplyAngularImpulse(m2BodyId bodyId, float impulse)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = impulse;
        m2JournalRecord(world, m2_opApplyAngularImpulse, &record, (int32_t)sizeof(record));
    }
    world->angularVelocities[index] += world->invInertia[index] * impulse;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Pos2 position;
            m2Rot rotation;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.position = position;
        record.rotation = rotation;
        m2JournalRecord(world, m2_opSetTransform, &record, (int32_t)sizeof(record));
    }

    // Whatever this body was resting on - or holding up - must notice.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = world->shapeBody[(int32_t)(world->pairKeys[i] >> 32)];
        int32_t b = world->shapeBody[(int32_t)(world->pairKeys[i] & 0xFFFFFFFFu)];
        if (a != index && b != index)
        {
            continue;
        }
        int32_t other = a == index ? b : a;
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }

    world->transforms[index].p = position;
    world->transforms[index].q = rotation;
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }

    // Broadphase refresh right now: the next step's pair update must
    // see the new home, not the old one.
    for (int32_t shape = world->bodyShapeHead[index]; shape != -1; shape = world->shapeNext[shape])
    {
        if (world->proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        m2AABB tight = m2ShapeTightAABB(world, shape);
        int32_t tree = m2ShapeTreeIndex(world, shape);
        if (!m2AABB_Contains(world->treeNodes[tree][world->proxyIds[shape]].aabb, tight))
        {
            m2TreeMove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[shape],
                       m2Fatten(tight));
        }
        m2PushMoved(world, shape);
    }
}

void m2Body_SetType(m2BodyId bodyId, m2BodyType type)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || (int32_t)type < 0 || (int32_t)type >= M2_TREE_COUNT)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->types[index] == (uint8_t)type)
    {
        return; // no-op, and deliberately not journaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t type;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.type = (uint8_t)type;
        m2JournalRecord(world, m2_opSetType, &record, (int32_t)sizeof(record));
    }

    // Contacts change meaning: wake every touching partner while the
    // old type is still in effect.
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = world->shapeBody[(int32_t)(world->pairKeys[i] >> 32)];
        int32_t b = world->shapeBody[(int32_t)(world->pairKeys[i] & 0xFFFFFFFFu)];
        if (a != index && b != index)
        {
            continue;
        }
        int32_t other = a == index ? b : a;
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }

    // Proxies move between the per-type trees; marking them moved also
    // purges stale pairs and re-forms the valid ones, which is exactly
    // the M19 path for pairs that stop making sense.
    int32_t oldTree = world->types[index];
    world->types[index] = (uint8_t)type;
    for (int32_t shape = world->bodyShapeHead[index]; shape != -1; shape = world->shapeNext[shape])
    {
        if (world->proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        m2TreeRemove(&world->trees[oldTree], world->treeNodes[oldTree], world->proxyIds[shape]);
        world->proxyIds[shape] = m2TreeInsert(&world->trees[type], world->treeNodes[type],
                                              m2Fatten(m2ShapeTightAABB(world, shape)), shape);
        M2_ASSERT(world->proxyIds[shape] != M2_NULL_NODE);
        m2PushMoved(world, shape);
    }

    m2RecomputeMass(world, index);
    if (type == m2_staticBody)
    {
        world->linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->angularVelocities[index] = 0.0f;
    }
    // Fresh start for the sleep ledger under the new identity.
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
    world->sleepStreak[index] = 0;
}

bool m2Body_IsAwake(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->types[index] == (uint8_t)m2_dynamicBody ? world->asleep[index] == 0 : true;
}

m2Pos2 m2Body_GetWorldPoint(m2BodyId bodyId, m2Vec2 localPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    m2Transform xf = world->transforms[index];
    m2Vec2 r = {xf.q.c * localPoint.x - xf.q.s * localPoint.y,
                xf.q.s * localPoint.x + xf.q.c * localPoint.y};
    return (m2Pos2){xf.p.x + (double)r.x, xf.p.y + (double)r.y};
}

m2Vec2 m2Body_GetLocalPoint(m2BodyId bodyId, m2Pos2 worldPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Transform xf = world->transforms[index];
    m2Vec2 rel = {(float)(worldPoint.x - xf.p.x), (float)(worldPoint.y - xf.p.y)};
    return (m2Vec2){xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
}

m2Vec2 m2Body_GetWorldVector(m2BodyId bodyId, m2Vec2 localVector)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->transforms[index].q;
    return (m2Vec2){q.c * localVector.x - q.s * localVector.y,
                    q.s * localVector.x + q.c * localVector.y};
}

m2Vec2 m2Body_GetLocalVector(m2BodyId bodyId, m2Vec2 worldVector)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    m2Rot q = world->transforms[index].q;
    return (m2Vec2){q.c * worldVector.x + q.s * worldVector.y,
                    -q.s * worldVector.x + q.c * worldVector.y};
}

m2Vec2 m2Body_GetWorldPointVelocity(m2BodyId bodyId, m2Pos2 worldPoint)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Vec2){0.0f, 0.0f};
    }
    // v + w x r, arm from the center of mass (one f64 crossing).
    m2Transform xf = world->transforms[index];
    m2Vec2 rlc = {xf.q.c * world->localCenters[index].x - xf.q.s * world->localCenters[index].y,
                  xf.q.s * world->localCenters[index].x + xf.q.c * world->localCenters[index].y};
    m2Vec2 r = {(float)(worldPoint.x - xf.p.x) - rlc.x, (float)(worldPoint.y - xf.p.y) - rlc.y};
    float w = world->angularVelocities[index];
    m2Vec2 v = world->linearVelocities[index];
    return (m2Vec2){v.x - w * r.y, v.y + w * r.x};
}

int32_t m2Body_GetJoints(m2BodyId bodyId, m2JointId* ids, int32_t capacity)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t e = world->bodyJointHead[index]; e != -1; e = world->jointEdgeNext[e])
    {
        int32_t j = e >> 1;
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {j + 1, world->worldIndex0, world->jointGenerations[j]};
            ids[total] = id;
        }
        total += 1;
    }
    return total;
}

void m2Body_ApplyLinearImpulseToCenter(m2BodyId bodyId, m2Vec2 impulse)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2Vec2 impulse;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.impulse = impulse;
        m2JournalRecord(world, m2_opImpulseCenter, &record, (int32_t)sizeof(record));
    }
    world->linearVelocities[index].x += world->invMass[index] * impulse.x;
    world->linearVelocities[index].y += world->invMass[index] * impulse.y;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetAwake(m2BodyId bodyId, bool awake)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody || world->disabled[index] != 0)
    {
        return;
    }
    uint8_t sleeping = awake ? 0 : 1;
    if (world->asleep[index] == sleeping)
    {
        return; // no-op stays unjournaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t awake;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.awake = awake ? 1 : 0;
        m2JournalRecord(world, m2_opSetAwake, &record, (int32_t)sizeof(record));
    }
    world->asleep[index] = sleeping;
    world->sleepTimes[index] = 0.0f;
    if (!awake)
    {
        // Forced sleep stills the body, reference-style.
        world->linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->angularVelocities[index] = 0.0f;
        world->forces[index] = (m2Vec2){0.0f, 0.0f};
        world->torques[index] = 0.0f;
    }
}

void m2Body_SetBullet(m2BodyId bodyId, bool flag)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    uint8_t next = flag ? 1 : 0;
    if (world->bullets[index] == next)
    {
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint8_t flag;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.flag = next;
        m2JournalRecord(world, m2_opSetBullet, &record, (int32_t)sizeof(record));
    }
    world->bullets[index] = next;
}

void m2Body_SetUserData(m2BodyId bodyId, uint64_t userData)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            uint64_t userData;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.userData = userData;
        m2JournalRecord(world, m2_opBodyUserData, &record, (int32_t)sizeof(record));
    }
    world->userData[index] = userData;
}

void m2Body_SetTargetTransform(m2BodyId bodyId, m2Pos2 position, m2Rot rotation, float dt)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !(dt > 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    // Velocities that land the pose in one step; applied through the
    // journaled setters, so replays get this for free.
    float invDt = 1.0f / dt;
    m2Transform xf = world->transforms[index];
    m2Vec2 v = {(float)(position.x - xf.p.x) * invDt, (float)(position.y - xf.p.y) * invDt};
    float w = m2UnwindAngle(m2RelativeJointAngle(xf.q, rotation)) * invDt;
    m2Body_SetLinearVelocity(bodyId, v);
    m2Body_SetAngularVelocity(bodyId, w);
}

m2Vec2 m2Body_GetLocalPointVelocity(m2BodyId bodyId, m2Vec2 localPoint)
{
    return m2Body_GetWorldPointVelocity(bodyId, m2Body_GetWorldPoint(bodyId, localPoint));
}

m2Pos2 m2Body_GetWorldCenterOfMass(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    return m2Body_GetWorldPoint(bodyId, world->localCenters[index]);
}

float m2Body_GetRotationalInertia(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    float invI = world->invInertia[index];
    return invI > 0.0f ? 1.0f / invI : 0.0f;
}

m2WorldId m2Body_GetWorld(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return id;
    }
    id.index1 = world->worldIndex0;
    id.generation = world->worldGeneration;
    return id;
}

m2AABBResult m2Body_ComputeAABB(m2BodyId bodyId)
{
    m2AABBResult result = {{0.0, 0.0}, {0.0, 0.0}};
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    result.lowerBound = world->transforms[index].p;
    result.upperBound = world->transforms[index].p;
    bool first = true;
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        m2AABB tight = m2ComputeShapeAABB(&world->shapeGeometry[s], world->transforms[index]);
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

void m2Body_Disable(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->disabled[index] != 0)
    {
        return; // already dormant: no-op stays unjournaled
    }
    m2JournalRecord(world, m2_opDisableBody, &bodyId, (int32_t)sizeof(bodyId));
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        m2RetireShapeFromBroadphase(world, s);
    }
    world->disabled[index] = 1;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_Enable(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->disabled[index] == 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opEnableBody, &bodyId, (int32_t)sizeof(bodyId));
    world->disabled[index] = 0;
    int32_t tree = world->types[index];
    for (int32_t s = world->bodyShapeHead[index]; s != -1; s = world->shapeNext[s])
    {
        world->proxyIds[s] = m2TreeInsert(&world->trees[tree], world->treeNodes[tree],
                                          m2Fatten(m2ShapeTightAABB(world, s)), s);
        m2PushMoved(world, s);
    }
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

void m2Body_SetDominance(m2BodyId bodyId, int8_t dominance)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->dominances[index] == dominance)
    {
        return; // no-op stays unjournaled
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            int8_t dominance;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.dominance = dominance;
        m2JournalRecord(world, m2_opSetDominance, &record, (int32_t)sizeof(record));
    }
    world->dominances[index] = dominance;
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }
}

int8_t m2Body_GetDominance(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->dominances[index];
}

bool m2Body_IsEnabled(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->disabled[index] == 0;
}

void m2Body_SetMassData(m2BodyId bodyId, m2MassData massData)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->types[index] != (uint8_t)m2_dynamicBody || !(massData.mass > 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
    {
        struct
        {
            m2BodyId body;
            m2MassData data;
        } record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.data = massData;
        m2JournalRecord(world, m2_opSetMassData, &record, (int32_t)sizeof(record));
    }
    world->invMass[index] = 1.0f / massData.mass;
    float inertiaCenter =
        massData.rotationalInertia - massData.mass * (massData.center.x * massData.center.x +
                                                      massData.center.y * massData.center.y);
    world->invInertia[index] =
        world->fixedRotations[index] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter : 0.0f;
    world->localCenters[index] = massData.center;
    world->asleep[index] = 0;
    world->sleepTimes[index] = 0.0f;
}

m2MassData m2Body_GetMassData(m2BodyId bodyId)
{
    m2MassData data = {0.0f, {0.0f, 0.0f}, 0.0f};
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return data;
    }
    float invMass = world->invMass[index];
    data.mass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
    data.center = world->localCenters[index];
    float invI = world->invInertia[index];
    float inertiaCenter = invI > 0.0f ? 1.0f / invI : 0.0f;
    data.rotationalInertia =
        inertiaCenter + data.mass * (data.center.x * data.center.x + data.center.y * data.center.y);
    return data;
}

void m2Body_ApplyMassFromShapes(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opMassFromShapes, &bodyId, (int32_t)sizeof(bodyId));
    m2RecomputeMass(world, index);
    if (world->types[index] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[index] = 0;
        world->sleepTimes[index] = 0.0f;
    }
}

m2BodyType m2Body_GetType(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_staticBody;
    }
    return (m2BodyType)world->types[index];
}

m2Vec2 m2Body_GetLocalCenter(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        m2Vec2 zero = {0.0f, 0.0f};
        return zero;
    }
    return world->localCenters[index];
}

bool m2Body_IsBullet(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->bullets[index] != 0;
}

float m2Body_GetGravityScale(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->gravityScales[index];
}

int32_t m2Body_GetShapes(m2BodyId bodyId, m2ShapeId* ids, int32_t capacity)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0 || world->shapeBody[i] != bodyIndex)
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
