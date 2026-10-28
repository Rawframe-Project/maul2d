// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bodies: creation and destruction, mass, parameters, type, sleep,
// enable and disable. Motion lives in body_motion.c, readback in
// body_read.c.

#include "body.h"

#include "chain.h"
#include "fpenv.h"
#include "island.h"
#include "joint_edges.h"
#include "journal.h"
#include "mass.h"
#include "math_functions.h"
#include "object_ids.h"
#include "proxies.h"
#include "shape.h"
#include "world_internal.h"
#include "world_state.h"

#include "maul2d/base.h"

#include <string.h>

// --- Mass ---------------------------------------------------------------------

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

// A def every field of which a snapshot of the world will restore: a
// known type, a finite position, a rotation with a direction, and the
// velocities, gravity scale and dampings within their bounds.
static m2Result BodyDefCheck(const m2BodyDef* def)
{
    m2Result result =
        m2CheckValid((int32_t)def->type >= (int32_t)m2_staticBody &&
                     (int32_t)def->type <= (int32_t)m2_dynamicBody && m2FinitePos2(def->position) &&
                     m2FiniteF(def->rotation.c) && m2FiniteF(def->rotation.s) &&
                     (def->rotation.c != 0.0f || def->rotation.s != 0.0f));
    result = m2BothChecks(result, m2CheckVelocity(def->linearVelocity));
    result = m2BothChecks(result, m2CheckSpeed(def->angularVelocity));
    result = m2BothChecks(result, m2CheckGravityScale(def->gravityScale));
    result = m2BothChecks(result, m2CheckRate(def->linearDamping));
    return m2BothChecks(result, m2CheckRate(def->angularDamping));
}

// A static body never moves, so its def carries no velocity, as its
// velocity setters take none.
static bool StaticDefIsStill(const m2BodyDef* def)
{
    return def->type != m2_staticBody ||
           (def->linearVelocity.x == 0.0f && def->linearVelocity.y == 0.0f &&
            def->angularVelocity == 0.0f);
}

// The create-body record holds the def's fields and zero bytes between
// them: copied field by field into the zeroed record, so no padding of
// the caller's struct reaches the tape.
_Static_assert(sizeof(m2OpCreateBody) == 88, "the create-body record is 88 bytes");

static void RecordBodyDef(m2BodyDef* out, const m2BodyDef* def)
{
    out->type = def->type;
    out->position = def->position;
    out->rotation = def->rotation;
    out->linearVelocity = def->linearVelocity;
    out->angularVelocity = def->angularVelocity;
    out->gravityScale = def->gravityScale;
    out->linearDamping = def->linearDamping;
    out->angularDamping = def->angularDamping;
    out->motionLocks.linearX = def->motionLocks.linearX;
    out->motionLocks.linearY = def->motionLocks.linearY;
    out->motionLocks.angularZ = def->motionLocks.angularZ;
    out->enableSleep = def->enableSleep;
    out->isEnabled = def->isEnabled;
    out->dominance = def->dominance;
    out->isBullet = def->isBullet;
    out->userData = def->userData;
    out->internalValue = def->internalValue;
}

static M2_NOINLINE m2Result CreateBodyWorker(m2World* world, const m2BodyDef* def,
                                             m2BodyId* bodyOut)
{
    if (bodyOut == NULL)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    *bodyOut = m2_nullBodyId;
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    if (def == NULL || def->internalValue != M2_BODY_COOKIE)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    m2Result checked = m2BothChecks(BodyDefCheck(def), m2CheckValid(StaticDefIsStill(def)));
    if (checked != m2_success)
    {
        return m2Refuse(world, checked);
    }
    if (world->bodies.freeCount == 0)
    {
        return m2Refuse(world, m2_errorCapacity);
    }

    int32_t index = world->bodies.freeQueue[world->bodies.freeHead];
    world->bodies.freeHead = (world->bodies.freeHead + 1) % world->bodies.bodyCapacity;
    world->bodies.freeCount -= 1;

    world->bodies.transforms[index].p = def->position;
    world->bodies.transforms[index].q = m2NormalizeRotWork(def->rotation);
    world->bodies.linearVelocities[index] = def->linearVelocity;
    // A rotation-locked body never spins, so a fixed-rotation (or
    // angularZ-locked) body drops any initial angular velocity at birth,
    // the same as the runtime setter does.
    world->bodies.angularVelocities[index] =
        def->motionLocks.angularZ ? 0.0f : def->angularVelocity;
    world->bodies.gravityScales[index] = def->gravityScale;
    world->bodies.linearDampings[index] = def->linearDamping;
    world->bodies.angularDampings[index] = def->angularDamping;
    world->bodies.fixedRotations[index] = def->motionLocks.angularZ ? 1 : 0;
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
    world->bodies.massOverrides[index] = 0;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.sleepStreak[index] = 0;
    world->bodies.bullets[index] = def->isBullet ? 1 : 0;
    if (index + 1 > world->bodies.maxBodyIndex)
    {
        world->bodies.maxBodyIndex = index + 1;
    }
    M2_ASSERT(m2StateBodyValid(world, index)); // BodyDefCheck admits what a restore does

    m2BodyId id = {(uint32_t)(index + 1), world->bodies.generations[index]};

    if (world->recorder.journalActive != 0)
    {
        m2OpCreateBody record;
        memset(&record, 0, sizeof(record));
        RecordBodyDef(&record.def, def);
        record.expected = id;
        m2JournalRecord(world, m2_opCreateBody, &record, (int32_t)sizeof(record));
    }
    *bodyOut = id;
    return m2_success;
}

m2Result m2CreateBody(m2World* world, const m2BodyDef* def, m2BodyId* bodyOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = CreateBodyWorker(world, def, bodyOut);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Result DestroyBodyWorker(m2World* world, m2BodyId bodyId)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    int32_t index = m2BodySlot(world, bodyId);
    if (index < 0)
    {
        return m2Refuse(world, m2_errorStale);
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
        m2WakeIfDynamic(world, other);
        world->joints.jointAlive[j] = 0;
        m2UnlinkJoint(world, j);
        if (world->joints.jointGenerations[j] == M2_LAST_GENERATION)
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

    // Cascade: destroy the shape list; every path that kills a touching
    // contact emits its end event.
    int32_t s = world->bodies.bodyShapeHead[index];
    while (s != -1)
    {
        int32_t next = world->shapes.shapeNext[s];
        m2DestroyShapeInternal(world, s);
        s = next;
    }
    world->bodies.bodyShapeHead[index] = -1;

    world->bodies.alive[index] = 0;
    if (world->bodies.generations[index] == M2_LAST_GENERATION)
    {
        world->bodies.retiredCount += 1;
        return m2_success;
    }
    world->bodies.generations[index] += 1;
    world->bodies.freeQueue[world->bodies.freeTail] = index;
    world->bodies.freeTail = (world->bodies.freeTail + 1) % world->bodies.bodyCapacity;
    world->bodies.freeCount += 1;
    return m2_success;
}

m2Result m2DestroyBody(m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = DestroyBodyWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

bool m2Body_IsValid(const m2World* world, m2BodyId bodyId)
{
    return world != NULL && m2BodySlot(world, bodyId) >= 0;
}

#define M2_BODY_GETTER(returnType, name, expr, fallback)                                           \
    returnType name(const m2World* world, m2BodyId bodyId)                                         \
    {                                                                                              \
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

// The value contract of each body parameter channel. Live setters and
// replay both pass through it, so a tape can never write what the API
// would refuse.
static bool BodyParamValid(uint8_t param, float value)
{
    switch (param)
    {
    case m2_bodyParamLinearDamping:
    case m2_bodyParamAngularDamping:
        return m2ValidRate(value);
    case m2_bodyParamGravityScale:
        return m2ValidGravityScale(value);
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
        m2WakeIfDynamic(world, index);
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
        m2WakeIfDynamic(world, index);
        break;
    case m2_bodyParamLockLinearY:
        // Lock linear Y.
        world->bodies.motionLocks[index] =
            value != 0.0f ? (uint8_t)(world->bodies.motionLocks[index] | M2_LOCK_LINEAR_Y)
                          : (uint8_t)(world->bodies.motionLocks[index] & ~M2_LOCK_LINEAR_Y);
        world->bodies.linearVelocities[index].y =
            value != 0.0f ? 0.0f : world->bodies.linearVelocities[index].y;
        m2WakeIfDynamic(world, index);
        break;
    default:
        M2_ASSERT(false); // BodyParamValid admits no other channel
        break;
    }
    return true;
}

static M2_NOINLINE void BodySetLinearDampingWorker(m2World* world, m2BodyId bodyId, float damping)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamLinearDamping, damping);
}

void m2Body_SetLinearDamping(m2World* world, m2BodyId bodyId, float damping)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetLinearDampingWorker(world, bodyId, damping);
    m2FloatEnvLeave(env);
}

M2_BODY_GETTER(float, m2Body_GetLinearDamping, world->bodies.linearDampings[index], 0.0f)

static M2_NOINLINE void BodySetAngularDampingWorker(m2World* world, m2BodyId bodyId, float damping)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamAngularDamping, damping);
}

void m2Body_SetAngularDamping(m2World* world, m2BodyId bodyId, float damping)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetAngularDampingWorker(world, bodyId, damping);
    m2FloatEnvLeave(env);
}

M2_BODY_GETTER(float, m2Body_GetAngularDamping, world->bodies.angularDampings[index], 0.0f)

static M2_NOINLINE void BodySetGravityScaleWorker(m2World* world, m2BodyId bodyId, float scale)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamGravityScale, scale);
}

void m2Body_SetGravityScale(m2World* world, m2BodyId bodyId, float scale)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetGravityScaleWorker(world, bodyId, scale);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetMotionLocksWorker(m2World* world, m2BodyId bodyId,
                                                 m2MotionLocks locks)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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

void m2Body_SetMotionLocks(m2World* world, m2BodyId bodyId, m2MotionLocks locks)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetMotionLocksWorker(world, bodyId, locks);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE m2MotionLocks BodyGetMotionLocksWorker(const m2World* world, m2BodyId bodyId)
{
    m2MotionLocks locks = {false, false, false};
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
    }
    else
    {
        locks.linearX = (world->bodies.motionLocks[index] & M2_LOCK_LINEAR_X) != 0;
        locks.linearY = (world->bodies.motionLocks[index] & M2_LOCK_LINEAR_Y) != 0;
        locks.angularZ = world->bodies.fixedRotations[index] != 0;
    }
    return locks;
}

m2MotionLocks m2Body_GetMotionLocks(const m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    m2MotionLocks result = BodyGetMotionLocksWorker(world, bodyId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void BodyEnableSleepWorker(m2World* world, m2BodyId bodyId, bool flag)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetBodyParamInternal(world, bodyId, m2_bodyParamEnableSleep, flag ? 1.0f : 0.0f);
}

void m2Body_EnableSleep(m2World* world, m2BodyId bodyId, bool flag)
{
    uint64_t env = m2FloatEnvEnter();
    BodyEnableSleepWorker(world, bodyId, flag);
    m2FloatEnvLeave(env);
}

M2_BODY_GETTER(bool, m2Body_IsSleepEnabled, world->bodies.sleepEnables[index] != 0, false)

static M2_NOINLINE void BodySetTypeWorker(m2World* world, m2BodyId bodyId, m2BodyType type)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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
        m2WakeIfDynamic(world, other);
    }

    // Proxies move between the per-type trees; marking them moved also
    // purges stale pairs and re-forms the valid ones, which is exactly
    // the end-event path for pairs that stop making sense.
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
                         m2Fatten(m2ShapeTightAabb(world, shape)), shape);
        M2_ASSERT(world->broadphase.proxyIds[shape] != M2_NULL_NODE);
        m2PushMoved(world, shape);
    }

    m2RecomputeMass(world, index);
    if (type == m2_staticBody)
    {
        world->bodies.linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.angularVelocities[index] = 0.0f;
    }
    // Fresh sleep state under the new identity.
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
    world->bodies.sleepStreak[index] = 0;
}

void m2Body_SetType(m2World* world, m2BodyId bodyId, m2BodyType type)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetTypeWorker(world, bodyId, type);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetAwakeWorker(m2World* world, m2BodyId bodyId, bool awake)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorStale);
        return;
    }
    if (world->bodies.types[index] != (uint8_t)m2_dynamicBody || world->bodies.disabled[index] != 0)
    {
        return; // only an enabled dynamic body sleeps
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
    // A body put to sleep counts its sleep streak from now, as the sleep
    // pass does.
    world->bodies.sleepStreak[index] = 0;
    if (!awake)
    {
        // Forced sleep stills the body.
        world->bodies.linearVelocities[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.angularVelocities[index] = 0.0f;
        world->bodies.forces[index] = (m2Vec2){0.0f, 0.0f};
        world->bodies.torques[index] = 0.0f;
        // The step refits no sleeping body's boxes; it may have moved
        // since its boxes were last refitted.
        m2RefitBodyProxies(world, index);
    }
}

void m2Body_SetAwake(m2World* world, m2BodyId bodyId, bool awake)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetAwakeWorker(world, bodyId, awake);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetBulletWorker(m2World* world, m2BodyId bodyId, bool flag)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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

void m2Body_SetBullet(m2World* world, m2BodyId bodyId, bool flag)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetBulletWorker(world, bodyId, flag);
    m2FloatEnvLeave(env);
}

void m2Body_SetUserData(m2World* world, m2BodyId bodyId, uint64_t userData)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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

// Takes a live, enabled body's shapes out of the broadphase.
static void DisableBody(m2World* world, m2BodyId bodyId, int32_t index)
{
    m2JournalRecord(world, m2_opDisableBody, &bodyId, (int32_t)sizeof(bodyId));
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        m2RetireShapeFromBroadphase(world, s);
    }
    world->bodies.disabled[index] = 1;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

// Puts a live, disabled body's shapes back into the broadphase.
static void EnableBody(m2World* world, m2BodyId bodyId, int32_t index)
{
    m2JournalRecord(world, m2_opEnableBody, &bodyId, (int32_t)sizeof(bodyId));
    world->bodies.disabled[index] = 0;
    int32_t tree = world->bodies.types[index];
    for (int32_t s = world->bodies.bodyShapeHead[index]; s != -1; s = world->shapes.shapeNext[s])
    {
        world->broadphase.proxyIds[s] =
            m2TreeInsert(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                         m2Fatten(m2ShapeTightAabb(world, s)), s);
        m2PushMoved(world, s);
    }
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

static M2_NOINLINE void BodySetEnabledWorker(m2World* world, m2BodyId bodyId, bool flag)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = m2BodySlot(world, bodyId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorStale);
        return;
    }
    if ((world->bodies.disabled[index] == 0) == flag)
    {
        return; // no change: a no-op stays unjournaled
    }
    if (flag)
    {
        EnableBody(world, bodyId, index);
    }
    else
    {
        DisableBody(world, bodyId, index);
    }
}

void m2Body_SetEnabled(m2World* world, m2BodyId bodyId, bool flag)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetEnabledWorker(world, bodyId, flag);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodySetDominanceWorker(m2World* world, m2BodyId bodyId, int8_t dominance)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
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
    m2WakeIfDynamic(world, index);
}

void m2Body_SetDominance(m2World* world, m2BodyId bodyId, int8_t dominance)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetDominanceWorker(world, bodyId, dominance);
    m2FloatEnvLeave(env);
}

// The inverses a body keeps for mass data, or false when the data has
// none a float holds: a mass outside its range, a center beyond M2_LOCAL_LIMIT,
// a central inertia that is neither 0 nor within its range, or inverses
// m2MassInversesValid refuses: one that overflows, or one whose own
// inverse does, as the inverse of a mass of FLT_MAX does. The inertia is
// taken about the center as given, so no shift from another point cancels
// a small one away; 0 gives a body that cannot turn.
static bool MassDataInverses(m2MassData massData, float* invMass, float* invInertia)
{
    if (!m2ValidMass(massData.mass) || !m2LocalVec2(massData.center) ||
        !m2ValidInertia(massData.centralInertia))
    {
        return false;
    }
    *invMass = 1.0f / massData.mass;
    *invInertia = massData.centralInertia > 0.0f ? 1.0f / massData.centralInertia : 0.0f;
    return m2MassInversesValid(*invMass, *invInertia);
}

static M2_NOINLINE void BodySetMassDataWorker(m2World* world, m2BodyId bodyId, m2MassData massData)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    float invMass = 0.0f;
    float invInertia = 0.0f;
    if (index < 0 || world->bodies.types[index] != (uint8_t)m2_dynamicBody ||
        !MassDataInverses(massData, &invMass, &invInertia))
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
    world->bodies.invMass[index] = invMass;
    world->bodies.invInertia[index] = world->bodies.fixedRotations[index] == 0 ? invInertia : 0.0f;
    world->bodies.localCenters[index] = massData.center;
    world->bodies.massOverrides[index] = 1;
    world->bodies.asleep[index] = 0;
    world->bodies.sleepTimes[index] = 0.0f;
}

void m2Body_SetMassData(m2World* world, m2BodyId bodyId, m2MassData massData)
{
    uint64_t env = m2FloatEnvEnter();
    BodySetMassDataWorker(world, bodyId, massData);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void BodyApplyMassFromShapesWorker(m2World* world, m2BodyId bodyId)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opMassFromShapes, &bodyId, (int32_t)sizeof(bodyId));
    m2RecomputeMass(world, index);
    m2WakeIfDynamic(world, index);
}

void m2Body_ApplyMassFromShapes(m2World* world, m2BodyId bodyId)
{
    uint64_t env = m2FloatEnvEnter();
    BodyApplyMassFromShapesWorker(world, bodyId);
    m2FloatEnvLeave(env);
}
