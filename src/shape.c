// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shapes: creation and destruction, materials, filters, geometry
// changes and shape readback.

#include "shape.h"

#include "body.h"
#include "broadphase.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

static int32_t ShapeSlot(const m2World* world, m2ShapeId id)
{
    int32_t index = id.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity)
    {
        return -1;
    }
    if (world->shapeAlive[index] == 0 || world->shapeGenerations[index] != id.generation)
    {
        return -1;
    }
    return index;
}

m2ShapeId m2MakeShapeId(const m2World* world, int32_t shapeIndex)
{
    m2ShapeId id = {shapeIndex + 1, world->worldIndex0, world->shapeGenerations[shapeIndex]};
    return id;
}

// M19 bookending shared by destroy and disable: end every touching
// contact of this shape, wake its riders, drop the proxy, prune pairs.
void m2RetireShapeFromBroadphase(m2World* world, int32_t shapeIndex)
{
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t a = (int32_t)(world->pairKeys[i] >> 32);
        int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (a != shapeIndex && b != shapeIndex)
        {
            continue;
        }
        // The same law as teleports and type changes: whoever was
        // resting on this shape must notice it vanish, or sleepers
        // float on a memory. (Caught by the floor-yank probe.)
        int32_t partner = world->shapeBody[a == shapeIndex ? b : a];
        if (world->types[partner] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[partner] = 0;
            world->sleepTimes[partner] = 0.0f;
        }
        bool sensor = world->shapeSensor[a] != 0 || world->shapeSensor[b] != 0;
        m2ContactEndEvent* queue = sensor ? world->pendingSensorEnd : world->pendingEndEvents;
        int32_t* queueCount = sensor ? &world->pendingSensorEndCount : &world->pendingEndCount;
        if (*queueCount < world->pairCapacity)
        {
            m2ContactEndEvent* e = &queue[(*queueCount)++];
            e->shapeIdA = m2MakeShapeId(world, a);
            e->shapeIdB = m2MakeShapeId(world, b);
            e->step = world->stepCount;
        }
    }

    if (world->proxyIds[shapeIndex] != M2_NULL_NODE)
    {
        int32_t tree = m2ShapeTreeIndex(world, shapeIndex);
        m2TreeRemove(&world->trees[tree], world->treeNodes[tree], world->proxyIds[shapeIndex]);
        world->proxyIds[shapeIndex] = M2_NULL_NODE;
    }
    m2PrunePairsOfShape(world, shapeIndex);
}

void m2DestroyShapeInternal(m2World* world, int32_t shapeIndex)
{
    m2RetireShapeFromBroadphase(world, shapeIndex);
    world->shapeAlive[shapeIndex] = 0;
    if (world->shapeGenerations[shapeIndex] == UINT16_MAX)
    {
        world->shapeRetiredCount += 1;
        return;
    }
    world->shapeGenerations[shapeIndex] += 1;
    world->shapeFreeQueue[world->shapeFreeTail] = shapeIndex;
    world->shapeFreeTail = (world->shapeFreeTail + 1) % world->shapeCapacity;
    world->shapeFreeCount += 1;
}

void m2DestroyShape(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    if (world == NULL)
    {
        return;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
    {
        return;
    }
    m2JournalRecord(world, m2_opDestroyShape, &shapeId, (int32_t)sizeof(shapeId));

    int32_t bodyIndex = world->shapeBody[index];
    // Unlink from the body's shape list (insertion-ordered, singly
    // linked - the walk is canonical).
    if (world->bodyShapeHead[bodyIndex] == index)
    {
        world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
    }
    else
    {
        for (int32_t s = world->bodyShapeHead[bodyIndex]; s != -1; s = world->shapeNext[s])
        {
            if (world->shapeNext[s] == index)
            {
                world->shapeNext[s] = world->shapeNext[index];
                break;
            }
        }
    }
    world->shapeNext[index] = -1;

    m2DestroyShapeInternal(world, index);
    m2RecomputeMass(world, bodyIndex);
    if (world->types[bodyIndex] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyIndex] = 0;
        world->sleepTimes[bodyIndex] = 0.0f;
    }
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
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_SHAPE_COOKIE;
    return def;
}

m2ShapeId m2CreateShape(m2BodyId bodyId, const m2ShapeDef* def, const m2ShapeGeometry* geometry)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_SHAPE_COOKIE ||
        !(def->density >= 0.0f) || !(def->friction >= 0.0f) ||
        !(def->restitution >= 0.0f && def->restitution <= 1.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullShapeId;
    }
    if (world->shapeFreeCount == 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullShapeId;
    }

    int32_t index = world->shapeFreeQueue[world->shapeFreeHead];
    world->shapeFreeHead = (world->shapeFreeHead + 1) % world->shapeCapacity;
    world->shapeFreeCount -= 1;

    // memset first: deterministic union tail bytes in the snapshot.
    memset(&world->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapeGeometry[index] = *geometry;
    world->shapeDensity[index] = def->density;
    world->shapeFriction[index] = def->friction;
    world->shapeRestitution[index] = def->restitution;
    world->shapeTangentSpeed[index] = def->tangentSpeed;
    world->shapeUserData[index] = def->userData;
    world->shapeCategory[index] = def->categoryBits;
    world->shapeMask[index] = def->maskBits;
    world->shapeGroup[index] = def->groupIndex;
    world->shapeSensor[index] = def->isSensor ? 1 : 0;
    world->shapeChain[index] = -1;
    world->shapeBody[index] = bodyIndex;
    world->shapeNext[index] = world->bodyShapeHead[bodyIndex];
    world->bodyShapeHead[bodyIndex] = index;
    world->shapeAlive[index] = 1;
    if (index + 1 > world->maxShapeIndex)
    {
        world->maxShapeIndex = index + 1;
    }

    if (world->disabled[bodyIndex] == 0)
    {
        int32_t tree = world->types[bodyIndex];
        world->proxyIds[index] = m2TreeInsert(&world->trees[tree], world->treeNodes[tree],
                                              m2Fatten(m2ShapeTightAABB(world, index)), index);
        if (world->proxyIds[index] == M2_NULL_NODE)
        {
            // Node pool exhausted: undo everything; capacity error, not UB.
            m2Refuse(world, m2_errorCapacity);
            world->bodyShapeHead[bodyIndex] = world->shapeNext[index];
            world->shapeAlive[index] = 0;
            world->shapeFreeHead =
                (world->shapeFreeHead + world->shapeCapacity - 1) % world->shapeCapacity;
            world->shapeFreeQueue[world->shapeFreeHead] = index;
            world->shapeFreeCount += 1;
            return m2_nullShapeId;
        }
        m2PushMoved(world, index);
    }
    // Dormant bodies keep the shape out of the trees until Enable, but
    // EVERYTHING else (mass, journaling, the id) proceeds normally so
    // replays mint identical worlds.
    m2RecomputeMass(world, bodyIndex);

    m2ShapeId id = {index + 1, bodyId.world0, world->shapeGenerations[index]};

    if (world->journalActive != 0)
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
    m2World* world = m2WorldFromIndex(shapeId.world0);
    return world != NULL && ShapeSlot(world, shapeId) >= 0;
}

m2BodyId m2Shape_GetBody(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullBodyId;
    }
    int32_t bodyIndex = world->shapeBody[index];
    m2BodyId id = {bodyIndex + 1, shapeId.world0, world->generations[bodyIndex]};
    return id;
}

uint64_t m2Shape_GetUserData(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    return world->shapeUserData[index];
}

static int32_t ShapeSlotChecked(m2ShapeId shapeId, m2World** outWorld)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    *outWorld = world;
    if (world == NULL)
    {
        return -1;
    }
    int32_t index = shapeId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->shapeAlive[index] == 0 ||
        world->shapeGenerations[index] != shapeId.generation)
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
    if (world == NULL || index < 0 || index >= world->shapeCapacity ||
        world->shapeAlive[index] == 0 || world->shapeGenerations[index] != shapeId.generation ||
        !ShapeParamValid(param, value))
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    if (world->journalActive != 0)
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
        world->shapeFriction[index] = value;
    }
    else if (param == m2_shapeParamTangentSpeed)
    {
        world->shapeTangentSpeed[index] = value;
        // A belt that changes speed must wake its riders, and the wake must
        // live HERE, inside the journaled channel, so a replay reproduces
        // it exactly. When it lived only in the public wrapper the replay
        // set the speed but left a sleeping rider asleep, and the recorded
        // and replayed worlds diverged (a fuzz seed caught this once the
        // velocity cap let it run far enough to reach the replay check).
        int32_t body = world->shapeBody[index];
        for (int32_t i = 0; i < world->pairCount; ++i)
        {
            int32_t a = (int32_t)(world->pairKeys[i] >> 32);
            int32_t b = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
            if (a != index && b != index)
            {
                continue;
            }
            int32_t otherBody = world->shapeBody[a == index ? b : a];
            if (world->types[otherBody] == (uint8_t)m2_dynamicBody)
            {
                world->asleep[otherBody] = 0;
                world->sleepTimes[otherBody] = 0.0f;
            }
        }
        if (world->types[body] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[body] = 0;
            world->sleepTimes[body] = 0.0f;
        }
    }
    else if (param == m2_shapeParamRestitution)
    {
        world->shapeRestitution[index] = value;
    }
    else
    {
        M2_ASSERT(false); // ShapeParamValid admits no other channel
    }
    return true;
}

void m2Shape_SetTangentSpeed(m2ShapeId shapeId, float speed)
{
    m2SetShapeParamInternal(m2WorldFromIndex(shapeId.world0), shapeId, m2_shapeParamTangentSpeed,
                            speed);
}

float m2Shape_GetTangentSpeed(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = shapeId.index1 - 1;
    if (world == NULL || index < 0 || index >= world->shapeCapacity ||
        world->shapeAlive[index] == 0 || world->shapeGenerations[index] != shapeId.generation)
    {
        return 0.0f;
    }
    return world->shapeTangentSpeed[index];
}

void m2Shape_SetFriction(m2ShapeId shapeId, float friction)
{
    m2SetShapeParamInternal(m2WorldFromIndex(shapeId.world0), shapeId, m2_shapeParamFriction,
                            friction);
}

void m2Shape_SetRestitution(m2ShapeId shapeId, float restitution)
{
    m2SetShapeParamInternal(m2WorldFromIndex(shapeId.world0), shapeId, m2_shapeParamRestitution,
                            restitution);
}

float m2Shape_GetFriction(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapeFriction[index] : 0.0f;
}

float m2Shape_GetRestitution(m2ShapeId shapeId)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    return index >= 0 ? world->shapeRestitution[index] : 0.0f;
}

void m2Shape_SetFilter(m2ShapeId shapeId, uint32_t categoryBits, uint32_t maskBits,
                       int32_t groupIndex)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    if (world->journalActive != 0)
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
    int32_t body = world->shapeBody[index];
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapeBody[sa == index ? sb : sa];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }

    world->shapeCategory[index] = categoryBits;
    world->shapeMask[index] = maskBits;
    world->shapeGroup[index] = groupIndex;
    m2PushMoved(world, index); // pair rebuild purges and re-collects (M19)
}

// One shared road for runtime geometry: validate outside, then swap
// the union (memset first: deterministic tail bytes), wake whoever
// was touching it, refresh mass and broadphase.
static void SetGeometryInternal(m2World* world, m2ShapeId shapeId, int32_t index,
                                const m2ShapeGeometry* geometry)
{
    if (world->journalActive != 0)
    {
        m2OpSetGeometry record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.geometry = *geometry;
        m2JournalRecord(world, m2_opSetGeometry, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->shapeBody[index];
    for (int32_t i = 0; i < world->pairCount; ++i)
    {
        if (world->pairTouching[i] == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->pairKeys[i] & 0xFFFFFFFFu);
        if (sa != index && sb != index)
        {
            continue;
        }
        int32_t other = world->shapeBody[sa == index ? sb : sa];
        if (world->types[other] == (uint8_t)m2_dynamicBody)
        {
            world->asleep[other] = 0;
            world->sleepTimes[other] = 0.0f;
        }
    }
    memset(&world->shapeGeometry[index], 0, sizeof(m2ShapeGeometry));
    world->shapeGeometry[index] = *geometry;
    m2RecomputeMass(world, body);
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
    if (world->proxyIds[index] != M2_NULL_NODE)
    {
        m2PushMoved(world, index);
    }
}

void m2Shape_SetCircle(m2ShapeId shapeId, const m2Circle* circle)
{
    m2World* world = NULL;
    int32_t index = ShapeSlotChecked(shapeId, &world);
    if (index < 0 || circle == NULL || !m2ValidateCircle(circle) ||
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
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
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
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
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
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
        world->shapeGeometry[index].type == (int32_t)m2_chainSegmentShape)
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
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    int32_t body = world->shapeBody[index];
    m2Transform xf = world->transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[index]);
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
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return (m2Pos2){0.0, 0.0};
    }
    int32_t body = world->shapeBody[index];
    m2Transform xf = world->transforms[body];
    m2Vec2 rel = {(float)(point.x - xf.p.x), (float)(point.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[index]);
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
    m2World* world = m2WorldFromIndex(shapeId.world0);
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

m2ChainId m2Shape_GetParentChain(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    m2ChainId id = {0, 0, 0};
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return id;
    }
    int32_t chain = world->shapeChain[index];
    if (chain < 0)
    {
        return id;
    }
    id.index1 = chain + 1;
    id.world0 = world->worldIndex0;
    id.generation = world->chainGenerations[chain];
    return id;
}

m2AABBResult m2Shape_GetAABB(m2ShapeId shapeId)
{
    m2AABBResult result = {{0.0, 0.0}, {0.0, 0.0}};
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return result;
    }
    m2AABB tight = m2ComputeShapeAABB(&world->shapeGeometry[index],
                                      world->transforms[world->shapeBody[index]]);
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
    if (world->journalActive != 0)
    {
        m2OpShapeFloat record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.value = density;
        m2JournalRecord(world, m2_opSetDensity, &record, (int32_t)sizeof(record));
    }
    world->shapeDensity[index] = density;
    int32_t body = world->shapeBody[index];
    m2RecomputeMass(world, body);
    if (world->types[body] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
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
    if (world->journalActive != 0)
    {
        m2OpShapeUserData record;
        memset(&record, 0, sizeof(record));
        record.shape = shapeId;
        record.userData = userData;
        m2JournalRecord(world, m2_opShapeUserData, &record, (int32_t)sizeof(record));
    }
    world->shapeUserData[index] = userData;
}

m2ShapeType m2Shape_GetType(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_circleShape;
    }
    return (m2ShapeType)world->shapeGeometry[index].type;
}

bool m2Shape_IsSensor(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->shapeSensor[index] != 0;
}

void m2Shape_GetFilter(m2ShapeId shapeId, uint32_t* categoryBits, uint32_t* maskBits,
                       int32_t* groupIndex)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    uint32_t category = 0;
    uint32_t mask = 0;
    int32_t group = 0;
    if (index >= 0)
    {
        category = world->shapeCategory[index];
        mask = world->shapeMask[index];
        group = world->shapeGroup[index];
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
        m2World* world = m2WorldFromIndex(shapeId.world0);                                         \
        int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;                            \
        if (index < 0 || world->shapeGeometry[index].type != (int32_t)(enumValue))                 \
        {                                                                                          \
            m2Refuse(world, m2_errorInvalid);                                                      \
            return zero;                                                                           \
        }                                                                                          \
        return world->shapeGeometry[index].field;                                                  \
    }

M2_GEOMETRY_GETTER(m2Shape_GetCircle, m2Circle, circle, m2_circleShape)

M2_GEOMETRY_GETTER(m2Shape_GetCapsule, m2Capsule, capsule, m2_capsuleShape)

M2_GEOMETRY_GETTER(m2Shape_GetPolygon, m2Polygon, polygon, m2_polygonShape)

M2_GEOMETRY_GETTER(m2Shape_GetSegment, m2Segment, segment, m2_segmentShape)

M2_GEOMETRY_GETTER(m2Shape_GetChainSegment, m2ChainSegment, chainSegment, m2_chainSegmentShape)

#undef M2_GEOMETRY_GETTER

float m2Shape_GetDensity(m2ShapeId shapeId)
{
    m2World* world = m2WorldFromIndex(shapeId.world0);
    int32_t index = world != NULL ? ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapeDensity[index];
}
