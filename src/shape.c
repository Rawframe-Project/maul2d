// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: creation and destruction, materials, filters, geometry
// changes and shape readback.

#include "shape.h"

#include "body.h"
#include "broadphase.h"
#include "distance.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

static int32_t ShapeSlot(const m2World* world, m2ShapeId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity)
    {
        return -1;
    }
    if (world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ShapeId m2MakeShapeId(const m2World* world, int32_t shapeIndex)
{
    m2ShapeId id = {shapeIndex + 1, world->idWorld, world->shapes.shapeGenerations[shapeIndex]};
    return id;
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
        bool sensor = world->shapes.shapeSensor[a] != 0 || world->shapes.shapeSensor[b] != 0;
        m2ContactEndEvent* queue =
            sensor ? world->events.pendingSensorEnd : world->events.pendingEndEvents;
        int32_t* queueCount =
            sensor ? &world->events.pendingSensorEndCount : &world->events.pendingEndCount;
        if (*queueCount < world->contacts.pairCapacity)
        {
            m2ContactEndEvent* e = &queue[(*queueCount)++];
            e->shapeIdA = m2MakeShapeId(world, a);
            e->shapeIdB = m2MakeShapeId(world, b);
            e->step = world->stepCount;
        }
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
    if (world->shapes.shapeGenerations[shapeIndex] == UINT16_MAX)
    {
        world->shapes.shapeRetiredCount += 1;
        return;
    }
    world->shapes.shapeGenerations[shapeIndex] += 1;
    world->shapes.shapeFreeQueue[world->shapes.shapeFreeTail] = shapeIndex;
    world->shapes.shapeFreeTail = (world->shapes.shapeFreeTail + 1) % world->shapes.shapeCapacity;
    world->shapes.shapeFreeCount += 1;
}

void m2DestroyShape(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    if (world == NULL)
    {
        return;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity || world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation)
    {
        return;
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

static bool ShapeDefValid(const m2ShapeDef* def)
{
    return def != NULL && def->internalValue == M2_SHAPE_COOKIE && m2FiniteF(def->density) &&
           def->density >= 0.0f && m2FiniteF(def->friction) && def->friction >= 0.0f &&
           def->restitution >= 0.0f && def->restitution <= 1.0f && m2FiniteF(def->tangentSpeed);
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

m2ShapeId m2CreateShape(m2BodyId bodyId, const m2ShapeDef* def, const m2ShapeGeometry* geometry)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || !ShapeDefValid(def))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullShapeId;
    }
    if (world->shapes.shapeFreeCount == 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullShapeId;
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
        m2Refuse(world, m2_errorCapacity);
        return m2_nullShapeId;
    }
    m2RecomputeMass(world, bodyIndex);
    m2ShapeId id = {index + 1, bodyId.world, world->shapes.shapeGenerations[index]};
    if (world->recorder.journalActive != 0)
    {
        m2OpCreateShape record;
        memset(&record, 0, sizeof(record));
        record.body = bodyId;
        record.def = *def;
        record.geometry = *geometry;
        record.expected = id;
        m2JournalRecord(world, m2_opCreateShape, &record, (int32_t)sizeof(record));
    }
    return id;
}

#define M2_SHAPE_CTOR(name, geomType, enumValue, validator, member)                                \
    m2ShapeId name(m2BodyId bodyId, const m2ShapeDef* def, const geomType* geom)                   \
    {                                                                                              \
        if (!validator(geom))                                                                      \
        {                                                                                          \
            m2Refuse(m2GetBodyWorld(bodyId), m2_errorInvalid);                                     \
            return m2_nullShapeId;                                                                 \
        }                                                                                          \
        m2ShapeGeometry geometry;                                                                  \
        memset(&geometry, 0, sizeof(geometry));                                                    \
        geometry.type = enumValue;                                                                 \
        geometry.member = *geom;                                                                   \
        return m2CreateShape(bodyId, def, &geometry);                                              \
    }

M2_SHAPE_CTOR(m2CreateCircleShape, m2Circle, m2_circleShape, m2ValidateCircle, circle)

M2_SHAPE_CTOR(m2CreateCapsuleShape, m2Capsule, m2_capsuleShape, m2ValidateCapsule, capsule)

M2_SHAPE_CTOR(m2CreatePolygonShape, m2Polygon, m2_polygonShape, m2ValidatePolygon, polygon)

M2_SHAPE_CTOR(m2CreateSegmentShape, m2Segment, m2_segmentShape, m2ValidateSegment, segment)

bool m2Shape_IsValid(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    return world != NULL && ShapeSlot(world, shapeId) >= 0;
}

m2BodyId m2Shape_GetBody(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t bodyIndex = world->shapes.shapeBody[index];
    m2BodyId id = {bodyIndex + 1, shapeId.world, world->bodies.generations[bodyIndex]};
    return id;
}

uint64_t m2Shape_GetUserData(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->shapes.shapeUserData[index];
}

static int32_t ShapeSlotChecked(m2ShapeId shapeId, m2World** outWorld)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    *outWorld = world;
    if (world == NULL)
    {
        return -1;
    }
    int32_t index = shapeId.index1 - 1;
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
        return m2FiniteF(value) && value >= 0.0f;
    case m2_shapeParamRestitution:
        return value >= 0.0f && value <= 1.0f;
    case m2_shapeParamTangentSpeed:
        return m2FiniteF(value);
    default:
        return false;
    }
}

// One journaled channel for the shape material parameters. Refuses a
// stale id (world may be NULL) or a value outside the contract.
bool m2SetShapeParamInternal(m2World* world, m2ShapeId shapeId, uint8_t param, float value)
{
    int32_t index = shapeId.index1 - 1;
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

void m2Shape_SetTangentSpeed(m2ShapeId shapeId, float speed)
{
    m2SetShapeParamInternal(m2WorldFromTag(shapeId.world), shapeId, m2_shapeParamTangentSpeed,
                            speed);
}

float m2Shape_GetTangentSpeed(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->shapes.shapeCapacity ||
        world->shapes.shapeAlive[index] == 0 ||
        world->shapes.shapeGenerations[index] != shapeId.generation)
    {
        return 0.0f;
    }
    return world->shapes.shapeTangentSpeed[index];
}

void m2Shape_SetFriction(m2ShapeId shapeId, float friction)
{
    m2SetShapeParamInternal(m2WorldFromTag(shapeId.world), shapeId, m2_shapeParamFriction,
                            friction);
}

void m2Shape_SetRestitution(m2ShapeId shapeId, float restitution)
{
    m2SetShapeParamInternal(m2WorldFromTag(shapeId.world), shapeId, m2_shapeParamRestitution,
                            restitution);
}

float m2Shape_GetFriction(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapes.shapeFriction[index] : 0.0f;
}

float m2Shape_GetRestitution(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapes.shapeRestitution[index] : 0.0f;
}

void m2Shape_SetFilter(m2ShapeId shapeId, uint64_t categoryBits, uint64_t maskBits,
                       int32_t groupIndex)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
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
    if (world->broadphase.proxyIds[index] != M2_NULL_NODE)
    {
        m2PushMoved(world, index);
    }
}

void m2Shape_SetCircle(m2ShapeId shapeId, const m2Circle* circle)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
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

void m2Shape_SetCapsule(m2ShapeId shapeId, const m2Capsule* capsule)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
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

void m2Shape_SetPolygon(m2ShapeId shapeId, const m2Polygon* polygon)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || polygon == NULL || !m2ValidatePolygon(polygon) ||
        world->shapes.shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2ShapeGeometry g;
    memset(&g, 0, sizeof(g));
    g.type = m2_polygonShape;
    g.polygon = *polygon;
    SetGeometryInternal(world, shapeId, index, &g);
}

void m2Shape_SetSegment(m2ShapeId shapeId, const m2Segment* segment)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
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

bool m2Shape_TestPoint(m2ShapeId shapeId, m2Pos2 point)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
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

m2Pos2 m2Shape_GetClosestPoint(m2ShapeId shapeId, m2Pos2 point)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
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

m2WorldId m2Shape_GetWorld(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    m2WorldId id = {0, 0};
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return id;
    }
    id.index1 = (uint16_t)(world->slot + 1);
    id.generation = world->worldGeneration;
    return id;
}

m2ChainId m2Shape_GetParentChain(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    m2ChainId id = {0, 0, 0};
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
    id.world = world->idWorld;
    id.generation = world->chains.chainGenerations[chain];
    return id;
}

m2AabbResult m2Shape_GetAabb(m2ShapeId shapeId)
{
    m2AabbResult result = {{0.0, 0.0}, {0.0, 0.0}};
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
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

void m2Shape_SetDensity(m2ShapeId shapeId, float density)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || !m2FiniteF(density) || density < 0.0f)
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

void m2Shape_SetUserData(m2ShapeId shapeId, uint64_t userData)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
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

m2ShapeType m2Shape_GetType(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_circleShape;
    }
    return (m2ShapeType)world->shapes.shapeGeometry[index].type;
}

bool m2Shape_IsSensor(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->shapes.shapeSensor[index] != 0;
}

void m2Shape_GetFilter(m2ShapeId shapeId, uint64_t* categoryBits, uint64_t* maskBits,
                       int32_t* groupIndex)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    uint64_t category = 0;
    uint64_t mask = 0;
    int32_t group = 0;
    if (index >= 0)
    {
        category = world->shapes.shapeCategory[index];
        mask = world->shapes.shapeMask[index];
        group = world->shapes.shapeGroup[index];
    }
    else
    {
        m2Refuse(world, m2_errorInvalid);
    }
    if (categoryBits != NULL)
    {
        *categoryBits = category;
    }
    if (maskBits != NULL)
    {
        *maskBits = mask;
    }
    if (groupIndex != NULL)
    {
        *groupIndex = group;
    }
}

// Geometry readback: exact stored bits, loud on a type mismatch.
#define M2_GEOMETRY_GETTER(name, fieldType, field, enumValue)                                      \
    fieldType name(m2ShapeId shapeId)                                                              \
    {                                                                                              \
        fieldType zero;                                                                            \
        memset(&zero, 0, sizeof(zero));                                                            \
        m2World* world = m2WorldFromTag(shapeId.world);                                            \
        int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;                            \
        if (index < 0 || world->shapes.shapeGeometry[index].type != (int32_t)(enumValue))          \
        {                                                                                          \
            m2Refuse(world, m2_errorInvalid);                                                      \
            return zero;                                                                           \
        }                                                                                          \
        return world->shapes.shapeGeometry[index].field;                                           \
    }

M2_GEOMETRY_GETTER(m2Shape_GetCircle, m2Circle, circle, m2_circleShape)

M2_GEOMETRY_GETTER(m2Shape_GetCapsule, m2Capsule, capsule, m2_capsuleShape)

M2_GEOMETRY_GETTER(m2Shape_GetPolygon, m2Polygon, polygon, m2_polygonShape)

M2_GEOMETRY_GETTER(m2Shape_GetSegment, m2Segment, segment, m2_segmentShape)

M2_GEOMETRY_GETTER(m2Shape_GetChainSegment, m2ChainSegment, chainSegment, m2_chainSegmentShape)

#undef M2_GEOMETRY_GETTER

float m2Shape_GetDensity(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromTag(shapeId.world);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapes.shapeDensity[index];
}
