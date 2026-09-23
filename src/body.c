// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation and destruction, motion, forces and impulses, mass,
// sleep, enable and disable, and body readback.

#include "body.h"

#include "broadphase.h"
#include "chain.h"
#include "joint.h"
#include "journal.h"
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
    if (index < 0 || index >= world->bodies.bodyCapacity)
    {
        return -1;
    }
    if (world->bodies.alive[index] == 0 || world->bodies.generations[index] != id.generation)
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
    if (world->bodies.types[bodyIndex] != (uint8_t)m2_dynamicBody)
    {
        world->bodies.invMass[bodyIndex] = 0.0f;
        world->bodies.invInertia[bodyIndex] = 0.0f;
        world->bodies.localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    // First pass: total mass and the mass-weighted center. Shape mass now
    // reports inertia about the SHAPE centroid, so the shift to the body
    // COM below is a sum of non-negative parallel-axis terms with no
    // big-minus-big cancellation (reference b2 #955).
    float mass = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        m2MassData data =
            m2ComputeShapeMass(&world->shapes.shapeGeometry[s], world->shapes.shapeDensity[s]);
        mass += data.mass;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        world->bodies.invMass[bodyIndex] = 1.0f;
        world->bodies.invInertia[bodyIndex] = 0.0f;
        world->bodies.localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;

    // Second pass: accumulate inertia about the body COM. Each shape's
    // centroid inertia plus mass times the (small) offset from the COM,
    // in the same canonical shape-list order.
    float inertiaCenter = 0.0f;
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        m2MassData data =
            m2ComputeShapeMass(&world->shapes.shapeGeometry[s], world->shapes.shapeDensity[s]);
        float ox = center.x - data.center.x;
        float oy = center.y - data.center.y;
        inertiaCenter += data.rotationalInertia + data.mass * (ox * ox + oy * oy);
    }
    world->bodies.invMass[bodyIndex] = invMass;
    world->bodies.invInertia[bodyIndex] =
        world->bodies.fixedRotations[bodyIndex] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter
                                                                             : 0.0f;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->bodies.localCenters[bodyIndex] = center;
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
    if (world->bodies.freeCount == 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullBodyId;
    }

    int32_t index = world->bodies.freeQueue[world->bodies.freeHead];
    world->bodies.freeHead = (world->bodies.freeHead + 1) % world->bodies.bodyCapacity;
    world->bodies.freeCount -= 1;

    world->bodies.transforms[index].p = def->position;
    world->bodies.transforms[index].q = m2NormalizeRot(def->rotation);
    world->bodies.linearVelocities[index] = def->linearVelocity;
    // A rotation-locked body never spins, so a fixed-rotation (or
    // angularZ-locked) body drops any initial angular velocity at birth,
    // the same as the runtime setter does.
    world->bodies.angularVelocities[index] =
        (def->fixedRotation || def->motionLocks.angularZ) ? 0.0f : def->angularVelocity;
    world->bodies.gravityScales[index] = def->gravityScale;
    world->bodies.linearDampings[index] = def->linearDamping;
    world->bodies.angularDampings[index] = def->angularDamping;
    world->bodies.fixedRotations[index] = (def->fixedRotation || def->motionLocks.angularZ) ? 1 : 0;
    world->bodies.motionLocks[index] =
        (uint8_t)((def->motionLocks.linearX ? M2_LOCK_LINEAR_X : 0u) |
                  (def->motionLocks.linearY ? M2_LOCK_LINEAR_Y : 0u));
    world->bodies.sleepEnables[index] = def->enableSleep ? 1 : 0;
    world->bodies.forces[index] = (m2Vec2){0.0f, 0.0f};
    world->bodies.torques[index] = 0.0f;
    world->bodies.disabled[index] = def->isEnabled ? 0 : 1;
    world->bodies.dominances[index] = def->dominance;
    world->bodies.userData[index] = def->userData;
    world->bodies.types[index] = (uint8_t)def->type;
    world->bodies.alive[index] = 1;
    world->bodies.bodyShapeHead[index] = -1;
    world->bodies.invMass[index] = def->type == m2_dynamicBody ? 1.0f : 0.0f; // shapeless floor
    world->bodies.localCenters[index] = (m2Vec2){0.0f, 0.0f};
    world->bodies.invInertia[index] = 0.0f;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.sleepStreak[index] = 0;
    world->bodies.bullets[index] = def->isBullet ? 1 : 0;
    if (index + 1 > world->bodies.maxBodyIndex)
    {
        world->bodies.maxBodyIndex = index + 1;
    }

    m2BodyId id = {index + 1, worldId.index1, world->bodies.generations[index]};

    if (world->recorder.journalActive != 0)
    {
        m2OpCreateBody record;
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
    while (world->joints.bodyJointHead[index] != -1)
    {
        int32_t j = world->joints.bodyJointHead[index] >> 1;
        int32_t other = world->joints.jointBodyA[j] == index ? world->joints.jointBodyB[j]
                                                             : world->joints.jointBodyA[j];
        if (world->bodies.types[other] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[other] = 0;
            world->bodies.sleepTimes[other] = 0.0f;
        }
        world->joints.jointAlive[j] = 0;
        m2UnlinkJoint(world, j);
        if (world->joints.jointGenerations[j] == UINT16_MAX)
        {
            world->joints.jointRetiredCount += 1;
        }
        else
        {
            world->joints.jointGenerations[j] += 1;
            world->joints.jointFreeQueue[world->joints.jointFreeTail] = j;
            world->joints.jointFreeTail =
                (world->joints.jointFreeTail + 1) % world->joints.jointCapacity;
            world->joints.jointFreeCount += 1;
        }
    }

    // Chains die with their body; their slots retire so stale chain
    // ids miss on generation, same as every other id kind.
    for (int32_t c = 0; c < world->chains.maxChainIndex; ++c)
    {
        if (world->chains.chainAlive[c] != 0 && world->chains.chainBody[c] == index)
        {
            m2RetireChainSlot(world, c);
        }
    }

    // Cascade: destroy the shape list (the contact-killing path that event
    // bookending will hang end-touch emission on, registry M19).
    int32_t s = world->bodies.bodyShapeHead[index];
    while (s != -1)
    {
        int32_t next = world->shapes.shapeNext[s];
        m2DestroyShapeInternal(world, s);
        s = next;
    }
    world->bodies.bodyShapeHead[index] = -1;

    world->bodies.alive[index] = 0;
    if (world->bodies.generations[index] == UINT16_MAX)
    {
        world->bodies.retiredCount += 1;
        return;
    }
    world->bodies.generations[index] += 1;
    world->bodies.freeQueue[world->bodies.freeTail] = index;
    world->bodies.freeTail = (world->bodies.freeTail + 1) % world->bodies.bodyCapacity;
    world->bodies.freeCount += 1;
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

M2_BODY_GETTER(m2Transform, m2Body_GetTransform, world->bodies.transforms[index],
               ((m2Transform){{0.0, 0.0}, {1.0f, 0.0f}}))

M2_BODY_GETTER(m2Pos2, m2Body_GetPosition, world->bodies.transforms[index].p, ((m2Pos2){0.0, 0.0}))

M2_BODY_GETTER(m2Rot, m2Body_GetRotation, world->bodies.transforms[index].q, ((m2Rot){1.0f, 0.0f}))

M2_BODY_GETTER(m2Vec2, m2Body_GetLinearVelocity, world->bodies.linearVelocities[index],
               ((m2Vec2){0.0f, 0.0f}))

M2_BODY_GETTER(float, m2Body_GetAngularVelocity, world->bodies.angularVelocities[index], 0.0f)

M2_BODY_GETTER(uint64_t, m2Body_GetUserData, world->bodies.userData[index], 0)

float m2Body_GetMass(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->bodies.invMass[index] > 0.0f ? 1.0f / world->bodies.invMass[index] : 0.0f;
}

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

// The value contract of each body parameter channel. Live setters and
// replay both pass through it, so a tape can never write what the API
// would refuse.
static bool BodyParamValid(uint8_t param, float value)
{
    switch (param)
    {
    case m2_bodyParamLinearDamping:
    case m2_bodyParamAngularDamping:
        return m2FiniteF(value) && value >= 0.0f;
    case m2_bodyParamGravityScale:
        return m2FiniteF(value);
    case m2_bodyParamFixedRotation:
    case m2_bodyParamEnableSleep:
    case m2_bodyParamLockLinearX:
    case m2_bodyParamLockLinearY:
        return value == 0.0f || value == 1.0f;
    default:
        return false;
    }
}

// One journaled channel for the body dynamics parameters, mirroring
// the shape and joint channels. Refuses a stale id (world may be NULL)
// or a value outside the channel's contract.
bool m2SetBodyParamInternal(m2World* world, m2BodyId bodyId, uint8_t param, float value)
{
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || !BodyParamValid(param, value))
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyParam record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opBodyParam, &record, (int32_t)sizeof(record));
    }
    switch (param)
    {
    case m2_bodyParamLinearDamping:
        world->bodies.linearDampings[index] = value;
        break;
    case m2_bodyParamAngularDamping:
        world->bodies.angularDampings[index] = value;
        break;
    case m2_bodyParamGravityScale:
        world->bodies.gravityScales[index] = value;
        break;
    case m2_bodyParamFixedRotation:
        // Fixed rotation is a mass property: inertia recomputes, spin
        // stops now (a frozen axis with leftover spin is a lie).
        world->bodies.fixedRotations[index] = value != 0.0f ? 1 : 0;
        world->bodies.angularVelocities[index] = 0.0f;
        m2RecomputeMass(world, index);
        if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[index] = 0;
            world->bodies.sleepTimes[index] = 0.0f;
        }
        break;
    case m2_bodyParamEnableSleep:
        world->bodies.sleepEnables[index] = value != 0.0f ? 1 : 0;
        if (value == 0.0f && world->bodies.types[index] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[index] = 0; // must not stay asleep illegally
            world->bodies.sleepTimes[index] = 0.0f;
        }
        break;
    case m2_bodyParamLockLinearX:
        // Lock linear X: a locked axis holds still, so stop it now and
        // wake the body (a frozen axis with leftover velocity is a lie,
        // same discipline as fixed rotation).
        world->bodies.motionLocks[index] =
            value != 0.0f ? (uint8_t)(world->bodies.motionLocks[index] | M2_LOCK_LINEAR_X)
                          : (uint8_t)(world->bodies.motionLocks[index] & ~M2_LOCK_LINEAR_X);
        world->bodies.linearVelocities[index].x =
            value != 0.0f ? 0.0f : world->bodies.linearVelocities[index].x;
        if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[index] = 0;
            world->bodies.sleepTimes[index] = 0.0f;
        }
        break;
    case m2_bodyParamLockLinearY:
        // Lock linear Y.
        world->bodies.motionLocks[index] =
            value != 0.0f ? (uint8_t)(world->bodies.motionLocks[index] | M2_LOCK_LINEAR_Y)
                          : (uint8_t)(world->bodies.motionLocks[index] & ~M2_LOCK_LINEAR_Y);
        world->bodies.linearVelocities[index].y =
            value != 0.0f ? 0.0f : world->bodies.linearVelocities[index].y;
        if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[index] = 0;
            world->bodies.sleepTimes[index] = 0.0f;
        }
        break;
    default:
        M2_ASSERT(false); // BodyParamValid admits no other channel
        break;
    }
    return true;
}

void m2Body_SetLinearDamping(m2BodyId bodyId, float damping)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamLinearDamping,
                           damping < 0.0f ? 0.0f : damping);
}

float m2Body_GetLinearDamping(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->bodies.linearDampings[index] : 0.0f;
}

void m2Body_SetAngularDamping(m2BodyId bodyId, float damping)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamAngularDamping,
                           damping < 0.0f ? 0.0f : damping);
}

float m2Body_GetAngularDamping(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 ? world->bodies.angularDampings[index] : 0.0f;
}

void m2Body_SetGravityScale(m2BodyId bodyId, float scale)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamGravityScale, scale);
}

void m2Body_SetFixedRotation(m2BodyId bodyId, bool flag)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamFixedRotation, flag ? 1.0f : 0.0f);
}

bool m2Body_IsFixedRotation(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->bodies.fixedRotations[index] != 0;
}

void m2Body_SetMotionLocks(m2BodyId bodyId, m2MotionLocks locks)
{
    m2World* world = m2GetBodyWorld(bodyId);
    // Each axis rides the journaled body-param channel so a replay
    // reproduces the locks. angularZ is the fixed-rotation lock.
    // A stale id refuses once, on the first channel.
    if (m2SetBodyParamInternal(world, bodyId, m2_bodyParamLockLinearX, locks.linearX ? 1.0f : 0.0f))
    {
        m2SetBodyParamInternal(world, bodyId, m2_bodyParamLockLinearY, locks.linearY ? 1.0f : 0.0f);
        m2SetBodyParamInternal(world, bodyId, m2_bodyParamFixedRotation,
                               locks.angularZ ? 1.0f : 0.0f);
    }
}

m2MotionLocks m2Body_GetMotionLocks(m2BodyId bodyId)
{
    m2MotionLocks locks = {false, false, false};
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index >= 0)
    {
        locks.linearX = (world->bodies.motionLocks[index] & M2_LOCK_LINEAR_X) != 0;
        locks.linearY = (world->bodies.motionLocks[index] & M2_LOCK_LINEAR_Y) != 0;
        locks.angularZ = world->bodies.fixedRotations[index] != 0;
    }
    return locks;
}

void m2Body_EnableSleep(m2BodyId bodyId, bool flag)
{
    m2World* world = m2GetBodyWorld(bodyId);
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamEnableSleep, flag ? 1.0f : 0.0f);
}

bool m2Body_IsSleepEnabled(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->bodies.sleepEnables[index] != 0;
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
        if (world->bodies.types[other] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[other] = 0;
            world->bodies.sleepTimes[other] = 0.0f;
        }
    }

    world->bodies.transforms[index].p = position;
    world->bodies.transforms[index].q = rotation;
    if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[index] = 0;
        world->bodies.sleepTimes[index] = 0.0f;
    }

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

void m2Body_SetType(m2BodyId bodyId, m2BodyType type)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || (int32_t)type < 0 || (int32_t)type >= M2_TREE_COUNT)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->bodies.types[index] == (uint8_t)type)
    {
        return; // no-op, and deliberately not journaled
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = (uint8_t)type;
        m2JournalRecord(world, m2_opSetType, &record, (int32_t)sizeof(record));
    }

    // Contacts change meaning: wake every touching partner while the
    // old type is still in effect.
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
        if (world->bodies.types[other] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.asleep[other] = 0;
            world->bodies.sleepTimes[other] = 0.0f;
        }
    }

    // Proxies move between the per-type trees; marking them moved also
    // purges stale pairs and re-forms the valid ones, which is exactly
    // the M19 path for pairs that stop making sense.
    int32_t oldTree = world->bodies.types[index];
    world->bodies.types[index] = (uint8_t)type;
    for (int32_t shape = world->bodies.bodyShapeHead[index]; shape != -1;
         shape = world->shapes.shapeNext[shape])
    {
        if (world->broadphase.proxyIds[shape] == M2_NULL_NODE)
        {
            continue;
        }
        m2TreeRemove(&world->broadphase.trees[oldTree], world->broadphase.treeNodes[oldTree],
                     world->broadphase.proxyIds[shape]);
        world->broadphase.proxyIds[shape] =
            m2TreeInsert(&world->broadphase.trees[type], world->broadphase.treeNodes[type],
                         m2Fatten(m2ShapeTightAABB(world, shape)), shape);
        M2_ASSERT(world->broadphase.proxyIds[shape] != M2_NULL_NODE);
        m2PushMoved(world, shape);
    }

    m2RecomputeMass(world, index);
    if (type == m2_staticBody)
    {
        world->bodies.linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.angularVelocities[index] = 0.0f;
    }
    // Fresh start for the sleep ledger under the new identity.
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.sleepStreak[index] = 0;
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
    return world->bodies.types[index] == (uint8_t)m2_dynamicBody ? world->bodies.asleep[index] == 0
                                                                 : true;
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
    m2Transform xf = world->bodies.transforms[index];
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
    m2Transform xf = world->bodies.transforms[index];
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
    m2Rot q = world->bodies.transforms[index].q;
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
    m2Rot q = world->bodies.transforms[index].q;
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
    for (int32_t e = world->joints.bodyJointHead[index]; e != -1;
         e = world->joints.jointEdgeNext[e])
    {
        int32_t j = e >> 1;
        if (ids != NULL && total < capacity)
        {
            m2JointId id = {j + 1, world->worldIndex0, world->joints.jointGenerations[j]};
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

void m2Body_SetAwake(m2BodyId bodyId, bool awake)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        world->bodies.disabled[index] != 0)
    {
        return;
    }
    uint8_t sleeping = awake ? 0 : 1;
    if (world->bodies.asleep[index] == sleeping)
    {
        return; // no-op stays unjournaled
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = awake ? 1 : 0;
        m2JournalRecord(world, m2_opSetAwake, &record, (int32_t)sizeof(record));
    }
    world->bodies.asleep[index] = sleeping;
    world->bodies.sleepTimes[index] = 0.0f;
    if (!awake)
    {
        // Forced sleep stills the body, reference-style.
        world->bodies.linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.angularVelocities[index] = 0.0f;
        world->bodies.forces[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.torques[index] = 0.0f;
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
    if (world->bodies.bullets[index] == next)
    {
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = next;
        m2JournalRecord(world, m2_opSetBullet, &record, (int32_t)sizeof(record));
    }
    world->bodies.bullets[index] = next;
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
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyUserData record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.userData = userData;
        m2JournalRecord(world, m2_opBodyUserData, &record, (int32_t)sizeof(record));
    }
    world->bodies.userData[index] = userData;
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
    return m2Body_GetWorldPoint(bodyId, world->bodies.localCenters[index]);
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
    float invI = world->bodies.invInertia[index];
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
    result.lowerBound = world->bodies.transforms[index].p;
    result.upperBound = world->bodies.transforms[index].p;
    bool first = true;
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2AABB tight =
            m2ComputeShapeAABB(&world->shapes.shapeGeometry[s], world->bodies.transforms[index]);
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
    if (index < 0 || world->bodies.disabled[index] != 0)
    {
        return; // already dormant: no-op stays unjournaled
    }
    m2JournalRecord(world, m2_opDisableBody, &bodyId, (int32_t)sizeof(bodyId));
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2RetireShapeFromBroadphase(world, s);
    }
    world->bodies.disabled[index] = 1;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_Enable(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.disabled[index] == 0)
    {
        return;
    }
    m2JournalRecord(world, m2_opEnableBody, &bodyId, (int32_t)sizeof(bodyId));
    world->bodies.disabled[index] = 0;
    int32_t tree = world->bodies.types[index];
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        world->broadphase.proxyIds[s] =
            m2TreeInsert(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                         m2Fatten(m2ShapeTightAABB(world, s)), s);
        m2PushMoved(world, s);
    }
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
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
    if (world->bodies.dominances[index] == dominance)
    {
        return; // no-op stays unjournaled
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpBodyByte record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.value = (uint8_t)dominance;
        m2JournalRecord(world, m2_opSetDominance, &record, (int32_t)sizeof(record));
    }
    world->bodies.dominances[index] = dominance;
    if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[index] = 0;
        world->bodies.sleepTimes[index] = 0.0f;
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
    return world->bodies.dominances[index];
}

bool m2Body_IsEnabled(m2BodyId bodyId)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    return index >= 0 && world->bodies.disabled[index] == 0;
}

void m2Body_SetMassData(m2BodyId bodyId, m2MassData massData)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !(massData.mass > 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpSetMassData record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.data = massData;
        m2JournalRecord(world, m2_opSetMassData, &record, (int32_t)sizeof(record));
    }
    world->bodies.invMass[index] = 1.0f / massData.mass;
    float inertiaCenter =
        massData.rotationalInertia - massData.mass * (massData.center.x * massData.center.x +
                                                      massData.center.y * massData.center.y);
    world->bodies.invInertia[index] =
        world->bodies.fixedRotations[index] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter
                                                                         : 0.0f;
    world->bodies.localCenters[index] = massData.center;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
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
    float invMass = world->bodies.invMass[index];
    data.mass = invMass > 0.0f ? 1.0f / invMass : 0.0f;
    data.center = world->bodies.localCenters[index];
    float invI = world->bodies.invInertia[index];
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
    if (world->bodies.types[index] == (uint8_t)m2_dynamicBody)
    {
        world->bodies.asleep[index] = 0;
        world->bodies.sleepTimes[index] = 0.0f;
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
    return (m2BodyType)world->bodies.types[index];
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
    return world->bodies.localCenters[index];
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
    return world->bodies.bullets[index] != 0;
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
    return world->bodies.gravityScales[index];
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
