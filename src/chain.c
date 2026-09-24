// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Chains: groups of one-sided segment shapes with shared materials.

#include "chain.h"

#include "body.h"
#include "geometry.h"
#include "journal.h"
#include "shape.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

void m2RetireChainSlot(m2World* world, int32_t chainIndex)
{
    world->chains.chainAlive[chainIndex] = 0;
    if (world->chains.chainGenerations[chainIndex] == UINT16_MAX)
    {
        world->chains.chainRetiredCount += 1;
        return;
    }
    world->chains.chainGenerations[chainIndex] += 1;
    world->chains.chainFreeQueue[world->chains.chainFreeTail] = chainIndex;
    world->chains.chainFreeTail = (world->chains.chainFreeTail + 1) % world->shapes.shapeCapacity;
    world->chains.chainFreeCount += 1;
}

m2ChainDef m2DefaultChainDef(void)
{
    m2ChainDef def;
    memset(&def, 0, sizeof(def));
    def.friction = 0.6f;
    def.categoryBits = 1;
    def.maskBits = UINT64_MAX;
    def.internalValue = M2_CHAIN_COOKIE;
    return def;
}

// The segment i of a chain with its two ghost neighbors. An open chain's
// first and last points are ghosts only.
static m2ChainSegment ChainSegmentAt(const m2ChainDef* def, int32_t i)
{
    m2ChainSegment seg;
    int32_t n = def->count;
    int32_t first = def->isLoop ? i + n - 1 : i;
    seg.ghost1 = def->points[first % n];
    seg.segment.point1 = def->points[(first + 1) % n];
    seg.segment.point2 = def->points[(first + 2) % n];
    seg.ghost2 = def->points[(first + 3) % n];
    return seg;
}

static int32_t ChainSegmentCount(const m2ChainDef* def)
{
    return def->isLoop ? def->count : def->count - 3;
}

// Finite points and a real length on every segment, so the segment
// creates below can only fail for capacity.
static bool ChainDefValid(const m2ChainDef* def)
{
    if (def->points == NULL || (def->isLoop ? def->count < 3 : def->count < 4))
    {
        return false;
    }
    for (int32_t i = 0; i < def->count; ++i)
    {
        if (!m2FiniteVec2(def->points[i]))
        {
            return false;
        }
    }
    for (int32_t i = 0; i < ChainSegmentCount(def); ++i)
    {
        m2ChainSegment seg = ChainSegmentAt(def, i);
        if (!m2ValidateSegment(&seg.segment))
        {
            return false;
        }
    }
    return true;
}

m2ChainId m2CreateChain(m2BodyId bodyId, const m2ChainDef* def)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_CHAIN_COOKIE ||
        !ChainDefValid(def))
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullChainId;
    }
    if (world->chains.chainFreeCount == 0 || world->shapes.shapeFreeCount == 0)
    {
        m2Refuse(world, m2_errorCapacity);
        return m2_nullChainId;
    }

    // Claim the chain slot before making shapes so each segment can be
    // tagged with its owner as it is born.
    int32_t chainIndex = world->chains.chainFreeQueue[world->chains.chainFreeHead];
    world->chains.chainFreeHead = (world->chains.chainFreeHead + 1) % world->shapes.shapeCapacity;
    world->chains.chainFreeCount -= 1;

    // One journal op describes the whole chain; the per-shape creates
    // below must not double-record.
    uint8_t journalWasActive = world->recorder.journalActive;
    world->recorder.journalActive = 0;

    m2ShapeDef shapeDef = m2DefaultShapeDef();
    shapeDef.density = 0.0f;
    shapeDef.friction = def->friction;
    shapeDef.restitution = def->restitution;
    shapeDef.categoryBits = def->categoryBits;
    shapeDef.maskBits = def->maskBits;
    shapeDef.groupIndex = def->groupIndex;
    shapeDef.userData = def->userData;

    int32_t created = 0;
    for (int32_t i = 0; i < ChainSegmentCount(def); ++i)
    {
        m2ShapeGeometry geometry;
        memset(&geometry, 0, sizeof(geometry));
        geometry.type = m2_chainSegmentShape;
        geometry.chainSegment = ChainSegmentAt(def, i);
        m2ShapeId shape = m2CreateShape(bodyId, &shapeDef, &geometry);
        if (shape.index1 == 0)
        {
            break; // capacity: partial chain, loud via the segment count
        }
        world->shapes.shapeChain[shape.index1 - 1] = chainIndex;
        created += 1;
    }

    world->recorder.journalActive = journalWasActive;
    if (created == 0)
    {
        // Unreachable while the shape pool had room: hand the claimed slot
        // back to the head of the free ring, generation untouched, since a
        // refused create is not journaled and must move no later id.
        world->chains.chainFreeHead =
            (world->chains.chainFreeHead + world->shapes.shapeCapacity - 1) %
            world->shapes.shapeCapacity;
        world->chains.chainFreeQueue[world->chains.chainFreeHead] = chainIndex;
        world->chains.chainFreeCount += 1;
        m2Refuse(world, m2_errorCapacity);
        return m2_nullChainId;
    }
    world->chains.chainAlive[chainIndex] = 1;
    world->chains.chainBody[chainIndex] = bodyIndex;
    if (chainIndex + 1 > world->chains.maxChainIndex)
    {
        world->chains.maxChainIndex = chainIndex + 1;
    }
    m2JournalRecordChain(world, bodyId, def, created);
    m2ChainId id = {chainIndex + 1, world->idWorld, world->chains.chainGenerations[chainIndex]};
    return id;
}

static int32_t ChainSlot(const m2World* world, m2ChainId chainId)
{
    int32_t index = chainId.index1 - 1;
    if (index < 0 || index >= world->shapes.shapeCapacity || world->chains.chainAlive[index] == 0 ||
        world->chains.chainGenerations[index] != chainId.generation)
    {
        return -1;
    }
    return index;
}

void m2DestroyChain(m2ChainId chainId)
{
    m2World* world = m2WorldFromTag(chainId.world);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opDestroyChain, &chainId, (int32_t)sizeof(chainId));

    // One canonical walk over the body's insertion-ordered shape list,
    // unlinking members in place; m2DestroyShapeInternal ends contacts
    // and wakes whoever was resting on each segment.
    int32_t bodyIndex = world->chains.chainBody[index];
    int32_t prev = -1;
    int32_t s = world->bodies.bodyShapeHead[bodyIndex];
    while (s != -1)
    {
        int32_t next = world->shapes.shapeNext[s];
        if (world->shapes.shapeChain[s] == index)
        {
            if (prev == -1)
            {
                world->bodies.bodyShapeHead[bodyIndex] = next;
            }
            else
            {
                world->shapes.shapeNext[prev] = next;
            }
            world->shapes.shapeNext[s] = -1;
            m2DestroyShapeInternal(world, s);
        }
        else
        {
            prev = s;
        }
        s = next;
    }
    m2RetireChainSlot(world, index);
    m2RecomputeMass(world, bodyIndex);
    m2WakeIfDynamic(world, bodyIndex);
}

bool m2Chain_IsValid(m2ChainId chainId)
{
    m2World* world = m2WorldFromTag(chainId.world);
    return world != NULL && ChainSlot(world, chainId) >= 0;
}

int32_t m2Chain_GetSegmentCount(m2ChainId chainId)
{
    m2World* world = m2WorldFromTag(chainId.world);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t count = 0;
    for (int32_t s = world->bodies.bodyShapeHead[world->chains.chainBody[index]]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        count += world->shapes.shapeChain[s] == index ? 1 : 0;
    }
    return count;
}

static void ChainMaterialInternal(m2World* world, m2ChainId chainId, int32_t chainIndex, uint8_t op,
                                  float value)
{
    if (world->recorder.journalActive != 0)
    {
        m2OpChainFloat record;
        memset(&record, 0, sizeof(record));
        record.chain = chainId;
        record.value = value;
        m2JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->chains.chainBody[chainIndex];
    for (int32_t s = world->bodies.bodyShapeHead[body]; s != -1; s = world->shapes.shapeNext[s])
    {
        if (world->shapes.shapeChain[s] != chainIndex)
        {
            continue;
        }
        if (op == m2_opChainFriction)
        {
            world->shapes.shapeFriction[s] = value;
        }
        else
        {
            world->shapes.shapeRestitution[s] = value;
        }
    }
}

void m2Chain_SetFriction(m2ChainId chainId, float friction)
{
    m2World* world = m2WorldFromTag(chainId.world);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !m2FiniteF(friction) || friction < 0.0f)
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainFriction, friction);
}

void m2Chain_SetRestitution(m2ChainId chainId, float restitution)
{
    m2World* world = m2WorldFromTag(chainId.world);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !(restitution >= 0.0f && restitution <= 1.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainRestitution, restitution);
}

m2WorldId m2Chain_GetWorld(m2ChainId chainId)
{
    m2World* world = m2WorldFromTag(chainId.world);
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

int32_t m2Chain_GetShapes(m2ChainId chainId, m2ShapeId* ids, int32_t capacity)
{
    m2World* world = m2WorldFromTag(chainId.world);
    int32_t chainIndex = world != NULL ? ChainSlot(world, chainId) : -1;
    if (chainIndex < 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        if (world->shapes.shapeAlive[i] == 0 || world->shapes.shapeChain[i] != chainIndex)
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
