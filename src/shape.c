// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: creation and destruction, materials, filters and geometry
// changes, and the readers that share their checks; shape_read.c holds
// the rest of the readback.

#include "shape.h"

#include "broadphase.h"
#include "distance.h"
#include "fpenv.h"
#include "island.h"
#include "journal.h"
#include "mass.h"
#include "object_ids.h"
#include "proxies.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

// Queues the end event of a touching pair whose shape leaves between
// steps; the next step's event window flushes it.
static void QueueEndEvent(m2World* world, int32_t a, int32_t b)
{
    if (world->shapes.shapeSensor[a] != 0 || world->shapes.shapeSensor[b] != 0)
    {
        if (world->events.pendingSensorEndCount < world->contacts.pairCapacity)
        {
            m2SensorEndEvent* e =
                &world->events.pendingSensorEnd[world->events.pendingSensorEndCount++];
            m2SensorPairIds(world, a, b, &e->sensorShapeId, &e->visitorShapeId);
            e->step = world->stepCount;
        }
    }
    else if (world->events.pendingEndCount < world->contacts.pairCapacity)
    {
        m2ContactEndEvent* e = &world->events.pendingEndEvents[world->events.pendingEndCount++];
        e->shapeIdA = m2MakeShapeId(world, a);
        e->shapeIdB = m2MakeShapeId(world, b);
        e->step = world->stepCount;
    }
}

// Shared by destroy and disable: end every touching
// contact of this shape, wake its riders, drop the proxy, prune pairs.
void m2RetireShapeFromBroadphase(m2World* world, int32_t shapeIndex)
{
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            continue;
        }
        // The same law as teleports and type changes: whoever was
        // resting on this shape must notice it vanish, or sleepers
        // float on a memory. (Caught by the floor-yank probe.)
        int32_t partner = world->shapes.shapeBody[a == shapeIndex ? b : a];
        m2WakeIfDynamic(world, partner);
        QueueEndEvent(world, a, b);
    }

    if (world->broadphase.proxyIds[shapeIndex] != M2_NULL_NODE)
    {
        int32_t tree = m2ShapeTreeIndex(world, shapeIndex);
        m2TreeRemove(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                     world->broadphase.proxyIds[shapeIndex]);
        world->broadphase.proxyIds[shapeIndex] = M2_NULL_NODE;
    }
    m2PrunePairsOfShape(world, shapeIndex);
}

void m2DestroyShapeInternal(m2World* world, int32_t shapeIndex)
{
    m2RetireShapeFromBroadphase(world, shapeIndex);
    world->shapes.shapeAlive[shapeIndex] = 0;
    if (world->shapes.shapeGenerations[shapeIndex] == M2_LAST_GENERATION)
    {
        world->shapes.shapeRetiredCount += 1;
        return;
    }
    world->shapes.shapeGenerations[shapeIndex] += 1;
    world->shapes.shapeFreeQueue[world->shapes.shapeFreeTail] = shapeIndex;
    world->shapes.shapeFreeTail = (world->shapes.shapeFreeTail + 1) % world->shapes.shapeCapacity;
    world->shapes.shapeFreeCount += 1;
}

static M2_NOINLINE m2Result DestroyShapeWorker(m2World* world, m2ShapeId shapeId)
{
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    int32_t index = (int32_t)shapeId.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity || world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation)
    {
        return m2Refuse(world, m2_errorStale);
    }
    m2JournalRecord(world, m2_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));

    int32_t bodyIndex = world->shapes.shapeBody[index];
    // Unlink from the body's shape list (insertion-ordered, singly
    // linked - the walk is canonical).
    if (world->bodies.bodyShapeHead[bodyIndex] == index)
    {
        world->bodies.bodyShapeHead[bodyIndex] = world->shapes.shapeNext[index];
    }
    else
    {
        for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
             s = world->shapes.shapeNext[s])
        {
            if (world->shapes.shapeNext[s] == index)
            {
                world->shapes.shapeNext[s] = world->shapes.shapeNext[index];
                break;
            }
        }
    }
    world->shapes.shapeNext[index] = -1;

    m2DestroyShapeInternal(world, index);
    m2RecomputeMass(world, bodyIndex);
    m2WakeIfDynamic(world, bodyIndex);
    return m2_success;
}

m2Result m2DestroyShape(m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result = DestroyShapeWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

// --- Shapes ---------------------------------------------------------------------

m2ShapeDef m2DefaultShapeDef(void)
{
    m2ShapeDef def;
    memset(&def, 0, sizeof(def));
    def.density = 1.0f;
    def.friction = 0.6f;
    def.restitution = 0.0f;
    def.categoryBits = 1;
    def.maskBits = UINT64_MAX;
    def.internalValue = M2_SHAPE_COOKIE;
    return def;
}

static m2Result ShapeDefCheck(const m2ShapeDef* def)
{
    if (def == NULL || def->internalValue != M2_SHAPE_COOKIE)
    {
        return m2_errorInvalid;
    }
    m2Result result = m2BothChecks(m2CheckDensity(def->density), m2CheckCoefficient(def->friction));
    result = m2BothChecks(result, m2CheckRestitution(def->restitution));
    return m2BothChecks(result, m2CheckSpeed(def->tangentSpeed));
}

static void WriteShapeSlot(m2World* world, int32_t index, int32_t bodyIndex, const m2ShapeDef* def,
                           const m2ShapeGeometry* geometry)
{
    m2Shapes* sh = &world->shapes;
    // memset first: deterministic union tail bytes in the snapshot.
    memset(&sh->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    sh->shapeGeometry[index] = *geometry;
    sh->shapeDensity[index] = def->density;
    sh->shapeFriction[index] = def->friction;
    sh->shapeRestitution[index] = def->restitution;
    sh->shapeTangentSpeed[index] = def->tangentSpeed;
    sh->shapeUserData[index] = def->userData;
    sh->shapeCategory[index] = def->categoryBits;
    sh->shapeMask[index] = def->maskBits;
    sh->shapeGroup[index] = def->groupIndex;
    sh->shapeSensor[index] = def->isSensor ? 1 : 0;
    sh->shapeChain[index] = -1;
    sh->shapeBody[index] = bodyIndex;
    sh->shapeNext[index] = world->bodies.bodyShapeHead[bodyIndex];
    world->bodies.bodyShapeHead[bodyIndex] = index;
    sh->shapeAlive[index] = 1;
}

// Enters the shape into its body's tree. A full node pool undoes the
// create: the slot goes back to the head of the free ring with its
// generation untouched, since a refused create moves no later id.
static bool InsertShapeProxy(m2World* world, int32_t index, int32_t oldMaxShapeIndex)
{
    int32_t bodyIndex = world->shapes.shapeBody[index];
    int32_t tree = world->bodies.types[bodyIndex];
    world->broadphase.proxyIds[index] =
        m2TreeInsert(&world->broadphase.trees[tree], world->broadphase.treeNodes[tree],
                     m2Fatten(m2ShapeTightAabb(world, index)), index);
    if (world->broadphase.proxyIds[index] != M2_NULL_NODE)
    {
        m2PushMoved(world, index);
        return true;
    }
    m2Shapes* sh = &world->shapes;
    world->bodies.bodyShapeHead[bodyIndex] = sh->shapeNext[index];
    sh->shapeAlive[index] = 0;
    sh->maxShapeIndex = oldMaxShapeIndex;
    sh->shapeFreeHead = (sh->shapeFreeHead + sh->shapeCapacity - 1) % sh->shapeCapacity;
    sh->shapeFreeQueue[sh->shapeFreeHead] = index;
    sh->shapeFreeCount += 1;
    return false;
}

// The create record holds the def's fields and zero bytes between them:
// copied field by field into the zeroed record, so no padding of the
// caller's struct reaches the tape.
static void RecordShapeDef(m2ShapeDef* out, const m2ShapeDef* def)
{
    out->density = def->density;
    out->friction = def->friction;
    out->tangentSpeed = def->tangentSpeed;
    out->restitution = def->restitution;
    out->categoryBits = def->categoryBits;
    out->maskBits = def->maskBits;
    out->groupIndex = def->groupIndex;
    out->isSensor = def->isSensor;
    out->userData = def->userData;
    out->internalValue = def->internalValue;
}

m2Result m2CreateShape(m2World* world, m2BodyId bodyId, const m2ShapeDef* def,
                       const m2ShapeGeometry* geometry, m2ShapeId* shapeOut)
{
    *shapeOut = m2_nullShapeId;
    int32_t bodyIndex = m2BodySlot(world, bodyId);
    if (bodyIndex < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    m2Result checked = ShapeDefCheck(def);
    if (checked != m2_success)
    {
        return m2Refuse(world, checked);
    }
    if (world->shapes.shapeFreeCount == 0)
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    int32_t index = world->shapes.shapeFreeQueue[world->shapes.shapeFreeHead];
    world->shapes.shapeFreeHead = (world->shapes.shapeFreeHead + 1) % world->shapes.shapeCapacity;
    world->shapes.shapeFreeCount -= 1;
    WriteShapeSlot(world, index, bodyIndex, def, geometry);
    int32_t oldMaxShapeIndex = world->shapes.maxShapeIndex;
    world->shapes.maxShapeIndex = index + 1 > oldMaxShapeIndex ? index + 1 : oldMaxShapeIndex;
    // Dormant bodies keep the shape out of the trees until Enable, but
    // everything else (mass, journaling, the id) proceeds normally so
    // replays mint identical worlds.
    if (world->bodies.disabled[bodyIndex] == 0 && !InsertShapeProxy(world, index, oldMaxShapeIndex))
    {
        return m2Refuse(world, m2_errorCapacity);
    }
    m2RecomputeMass(world, bodyIndex);
    m2ShapeId id = {(uint32_t)(index + 1), world->shapes.shapeGenerations[index]};
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateShape record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        RecordShapeDef(&record.def, def);
        record.geometry = *geometry;
        record.expected = id;
        m2JournalRecord(world, m2_opCreateShape, &record, (int32_t)sizeof(record));
    }
    *shapeOut = id;
    return m2_success;
}

// Each constructor enters the floating-point environment around a worker
// that does its work, as every public function that computes does.
#define M2_SHAPE_CTOR(name, geomType, enumValue, validator, member)                                \
    static M2_NOINLINE m2Result name##Worker(m2World* world, m2BodyId bodyId,                      \
                                             const m2ShapeDef* def, const geomType* geom,          \
                                             m2ShapeId* shapeOut)                                  \
    {                                                                                              \
        if (shapeOut == NULL)                                                                      \
        {                                                                                          \
            return m2Refuse(world, m2_errorInvalid);                                               \
        }                                                                                          \
        *shapeOut = m2_nullShapeId;                                                                \
        m2Result writer = m2CheckWriter(world);                                                    \
        if (writer != m2_success)                                                                  \
        {                                                                                          \
            return writer;                                                                         \
        }                                                                                          \
        m2Result checked = validator(geom);                                                        \
        if (checked != m2_success)                                                                 \
        {                                                                                          \
            return m2Refuse(world, checked);                                                       \
        }                                                                                          \
        m2ShapeGeometry geometry;                                                                  \
        memset(&geometry, 0, sizeof(geometry));                                                    \
        geometry.type = enumValue;                                                                 \
        geometry.member = *geom;                                                                   \
        if (!m2CanonicalizeGeometry(&geometry))                                                    \
        {                                                                                          \
            return m2Refuse(world, m2_errorInvalid);                                               \
        }                                                                                          \
        return m2CreateShape(world, bodyId, def, &geometry, shapeOut);                             \
    }                                                                                              \
    m2Result name(m2World* world, m2BodyId bodyId, const m2ShapeDef* def, const geomType* geom,    \
                  m2ShapeId* shapeOut)                                                             \
    {                                                                                              \
        uint64_t env = m2FloatEnvEnter();                                                          \
        m2Result result = name##Worker(world, bodyId, def, geom, shapeOut);                        \
        m2FloatEnvLeave(env);                                                                      \
        return result;                                                                             \
    }

M2_SHAPE_CTOR(m2CreateCircleShape, m2Circle, m2_circleShape, m2CheckCircle, circle)

M2_SHAPE_CTOR(m2CreateCapsuleShape, m2Capsule, m2_capsuleShape, m2CheckCapsule, capsule)

M2_SHAPE_CTOR(m2CreatePolygonShape, m2Polygon, m2_polygonShape, m2CheckPolygon, polygon)

M2_SHAPE_CTOR(m2CreateSegmentShape, m2Segment, m2_segmentShape, m2CheckSegment, segment)

bool m2Shape_IsValid(const m2World* world, m2ShapeId shapeId)
{
    return world != NULL && m2ShapeSlot(world, shapeId) >= 0;
}

m2BodyId m2Shape_GetBody(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t bodyIndex = world->shapes.shapeBody[index];
    m2BodyId id = {(uint32_t)(bodyIndex + 1), world->bodies.generations[bodyIndex]};
    return id;
}

uint64_t m2Shape_GetUserData(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->shapes.shapeUserData[index];
}

static int32_t ShapeSlotChecked(const m2World* world, m2ShapeId shapeId)
{
    if (world == NULL)
    {
        return -1;
    }
    int32_t index = (int32_t)shapeId.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity || world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation)
    {
        return -1;
    }
    return index;
}

// One journaled channel for shape materials (op 22).
// The value contract of each shape parameter channel, shared by the
// live setters and replay.
static bool ShapeParamValid(uint8_t param, float value)
{
    switch (param)
    {
    case m2_shapeParamFriction:
        return m2ValidCoefficient(value);
    case m2_shapeParamRestitution:
        return m2ValidRestitution(value);
    case m2_shapeParamTangentSpeed:
        return m2ValidSpeed(value);
    default:
        return false;
    }
}

// One journaled channel for the shape material parameters. Refuses a
// stale id (world may be NULL) or a value outside the contract.
bool m2SetShapeParamInternal(m2World* world, m2ShapeId shapeId, uint8_t param, float value)
{
    int32_t index = (int32_t)shapeId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->shapes.shapeCapacity ||
        world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation ||
        !ShapeParamValid(param, value))
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpShapeParam record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.value = value;
        record.param = param;
        m2JournalRecord(world, m2_opShapeParam, &record, (int32_t)sizeof(record));
    }
    if (param == m2_shapeParamFriction)
    {
        world->shapes.shapeFriction[index] = value;
    }
    else if (param == m2_shapeParamTangentSpeed)
    {
        world->shapes.shapeTangentSpeed[index] = value;
        // A belt that changes speed must wake its riders, and the wake must
        // live HERE, inside the journaled channel, so a replay reproduces
        // it exactly. When it lived only in the public wrapper the replay
        // set the speed but left a sleeping rider asleep, and the recorded
        // and replayed worlds diverged (a fuzz seed caught this once the
        // velocity cap let it run far enough to reach the replay check).
        int32_t body = world->shapes.shapeBody[index];
        for (int32_t i = 0; i < world->contacts.pairCount; ++i)
        {
            int32_t a = (int32_t)(world->contacts.pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
            if (a != index && b != index)
            {
                continue;
            }
            int32_t otherBody = world->shapes.shapeBody[a == index ? b : a];
            m2WakeIfDynamic(world, otherBody);
        }
        m2WakeIfDynamic(world, body);
    }
    else if (param == m2_shapeParamRestitution)
    {
        world->shapes.shapeRestitution[index] = value;
    }
    else
    {
        M2_ASSERT(false); // ShapeParamValid admits no other channel
    }
    return true;
}

static M2_NOINLINE void ShapeSetTangentSpeedWorker(m2World* world, m2ShapeId shapeId, float speed)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetShapeParamInternal(world, shapeId, m2_shapeParamTangentSpeed, speed);
}

void m2Shape_SetTangentSpeed(m2World* world, m2ShapeId shapeId, float speed)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetTangentSpeedWorker(world, shapeId, speed);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE float ShapeGetTangentSpeedWorker(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = (int32_t)shapeId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->shapes.shapeCapacity ||
        world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapes.shapeTangentSpeed[index];
}

float m2Shape_GetTangentSpeed(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = ShapeGetTangentSpeedWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void ShapeSetFrictionWorker(m2World* world, m2ShapeId shapeId, float friction)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetShapeParamInternal(world, shapeId, m2_shapeParamFriction, friction);
}

void m2Shape_SetFriction(m2World* world, m2ShapeId shapeId, float friction)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetFrictionWorker(world, shapeId, friction);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void ShapeSetRestitutionWorker(m2World* world, m2ShapeId shapeId,
                                                  float restitution)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    m2SetShapeParamInternal(world, shapeId, m2_shapeParamRestitution, restitution);
}

void m2Shape_SetRestitution(m2World* world, m2ShapeId shapeId, float restitution)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetRestitutionWorker(world, shapeId, restitution);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE float ShapeGetFrictionWorker(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapes.shapeFriction[index];
}

float m2Shape_GetFriction(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = ShapeGetFrictionWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE float ShapeGetRestitutionWorker(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapes.shapeRestitution[index];
}

float m2Shape_GetRestitution(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = ShapeGetRestitutionWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void ShapeSetFilterWorker(m2World* world, m2ShapeId shapeId,
                                             uint64_t categoryBits, uint64_t maskBits,
                                             int32_t groupIndex)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpSetFilter record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.categoryBits = categoryBits;
        record.maskBits = maskBits;
        record.groupIndex = groupIndex;
        m2JournalRecord(world, m2_opSetFilter, &record, (int32_t)sizeof(record));
    }

    // Whoever this shape was touching must notice its allegiance
    // change, exactly like a teleport or a type flip.
    int32_t body = world->shapes.shapeBody[index];
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapes.shapeBody[sa == index ? sb : sa];
        m2WakeIfDynamic(world, other);
    }
    m2WakeIfDynamic(world, body);

    world->shapes.shapeCategory[index] = categoryBits;
    world->shapes.shapeMask[index] = maskBits;
    world->shapes.shapeGroup[index] = groupIndex;
    m2PushMoved(world, index); // the pair rebuild purges and re-collects
}

void m2Shape_SetFilter(m2World* world, m2ShapeId shapeId, uint64_t categoryBits, uint64_t maskBits,
                       int32_t groupIndex)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetFilterWorker(world, shapeId, categoryBits, maskBits, groupIndex);
    m2FloatEnvLeave(env);
}

// One shared road for runtime geometry: validate outside, then swap
// the union (memset first: deterministic tail bytes), wake whoever
// was touching it, refresh mass and broadphase.
static void SetGeometryInternal(m2World* world, m2ShapeId shapeId, int32_t index,
                                const m2ShapeGeometry* geometry)
{
    if (world->recorder.journalActive != 0)
    {
        m2OpSetGeometry record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.geometry = *geometry;
        m2JournalRecord(world, m2_opSetGeometry, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->shapes.shapeBody[index];
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapes.shapeBody[sa == index ? sb : sa];
        m2WakeIfDynamic(world, other);
    }
    memset(&world->shapes.shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapes.shapeGeometry[index] = *geometry;
    m2RecomputeMass(world, body);
    m2WakeIfDynamic(world, body);
    // The box is refitted now, as a teleport refits it: the step refits
    // only awake bodies' boxes, so a static shape's would keep its old
    // extent.
    if (world->broadphase.proxyIds[index] != M2_NULL_NODE)
    {
        (void)m2RefitProxy(world, index);
        m2PushMoved(world, index);
    }
}

static M2_NOINLINE void ShapeSetCircleWorker(m2World* world, m2ShapeId shapeId,
                                             const m2Circle* circle)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0 || circle == NULL || !m2ValidateCircle(circle) ||
        world->shapes.shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_circleShape;
    g.circle = *circle;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetCircle(m2World* world, m2ShapeId shapeId, const m2Circle* circle)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetCircleWorker(world, shapeId, circle);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void ShapeSetCapsuleWorker(m2World* world, m2ShapeId shapeId,
                                              const m2Capsule* capsule)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0 || capsule == NULL || !m2ValidateCapsule(capsule) ||
        world->shapes.shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_capsuleShape;
    g.capsule = *capsule;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetCapsule(m2World* world, m2ShapeId shapeId, const m2Capsule* capsule)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetCapsuleWorker(world, shapeId, capsule);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void ShapeSetPolygonWorker(m2World* world, m2ShapeId shapeId,
                                              const m2Polygon* polygon)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0 || polygon == NULL || !m2ValidatePolygon(polygon) ||
        world->shapes.shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_polygonShape;
    g.polygon = m2CanonicalPolygon(polygon); // the caller's normals are not kept
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetPolygon(m2World* world, m2ShapeId shapeId, const m2Polygon* polygon)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetPolygonWorker(world, shapeId, polygon);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE void ShapeSetSegmentWorker(m2World* world, m2ShapeId shapeId,
                                              const m2Segment* segment)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0 || segment == NULL || !m2ValidateSegment(segment) ||
        world->shapes.shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_segmentShape;
    g.segment = *segment;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetSegment(m2World* world, m2ShapeId shapeId, const m2Segment* segment)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetSegmentWorker(world, shapeId, segment);
    m2FloatEnvLeave(env);
}

static M2_NOINLINE bool ShapeTestPointWorker(const m2World* world, m2ShapeId shapeId, m2Pos2 point)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    int32_t body = world->shapes.shapeBody[index];
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapes.shapeGeometry[index]);
    m2DistanceProxy probe;
    probe.points[0] = local;
    probe.count = 1;
    probe.radius = 0.0f;
    m2DistanceResult d = m2ShapeDistance(&target, &probe);
    // Touching within the slop skin counts (the overlap law).
    return d.distance - target.radius <= 0.005f;
}

bool m2Shape_TestPoint(const m2World* world, m2ShapeId shapeId, m2Pos2 point)
{
    uint64_t env = m2FloatEnvEnter();
    bool result = ShapeTestPointWorker(world, shapeId, point);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2Pos2 ShapeGetClosestPointWorker(const m2World* world, m2ShapeId shapeId,
                                                     m2Pos2 point)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    int32_t body = world->shapes.shapeBody[index];
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapes.shapeGeometry[index]);
    m2DistanceProxy probe;
    probe.points[0] = local;
    probe.count = 1;
    probe.radius = 0.0f;
    m2DistanceResult d = m2ShapeDistance(&target, &probe);
    if (d.distance - target.radius <= 0.0f)
    {
        return point; // inside: the query point is its own closest
    }
    m2Vec2 surf = {d.pointA.x + target.radius * d.normal.x,
                   d.pointA.y + target.radius * d.normal.y};
    m2Vec2 out = {xf.q.c * surf.x - xf.q.s * surf.y, xf.q.s * surf.x + xf.q.c * surf.y};
    return (m2Pos2){xf.p.x + (double)out.x, xf.p.y + (double)out.y};
}

m2Pos2 m2Shape_GetClosestPoint(const m2World* world, m2ShapeId shapeId, m2Pos2 point)
{
    uint64_t env = m2FloatEnvEnter();
    m2Pos2 result = ShapeGetClosestPointWorker(world, shapeId, point);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2ChainId ShapeGetParentChainWorker(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    m2ChainId id = {0, 0};
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return id;
    }
    int32_t chain = world->shapes.shapeChain[index];
    if (chain < 0)
    {
        return id;
    }
    id.index1 = chain + 1;
    id.generation = world->chains.chainGenerations[chain];
    return id;
}

m2ChainId m2Shape_GetParentChain(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    m2ChainId result = ShapeGetParentChainWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE m2WorldAabb ShapeGetAabbWorker(const m2World* world, m2ShapeId shapeId)
{
    m2WorldAabb result = {{0.0, 0.0}, {0.0, 0.0}};
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    m2Aabb tight = m2ComputeShapeAabb(&world->shapes.shapeGeometry[index],
                                      world->bodies.transforms[world->shapes.shapeBody[index]]);
    result.lowerBound = tight.lowerBound;
    result.upperBound = tight.upperBound;
    return result;
}

m2WorldAabb m2Shape_GetAabb(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    m2WorldAabb result = ShapeGetAabbWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}

static M2_NOINLINE void ShapeSetDensityWorker(m2World* world, m2ShapeId shapeId, float density)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0 || !m2ValidDensity(density))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpShapeFloat record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.value = density;
        m2JournalRecord(world, m2_opSetDensity, &record, (int32_t)sizeof(record));
    }
    world->shapes.shapeDensity[index] = density;
    int32_t body = world->shapes.shapeBody[index];
    m2RecomputeMass(world, body);
    m2WakeIfDynamic(world, body);
}

void m2Shape_SetDensity(m2World* world, m2ShapeId shapeId, float density)
{
    uint64_t env = m2FloatEnvEnter();
    ShapeSetDensityWorker(world, shapeId, density);
    m2FloatEnvLeave(env);
}

void m2Shape_SetUserData(m2World* world, m2ShapeId shapeId, uint64_t userData)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    int32_t index = ShapeSlotChecked(world, shapeId);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->recorder.journalActive != 0)
    {
        m2OpShapeUserData record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.userData = userData;
        m2JournalRecord(world, m2_opShapeUserData, &record, (int32_t)sizeof(record));
    }
    world->shapes.shapeUserData[index] = userData;
}
