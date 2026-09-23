// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Chains: groups of one-sided segment shapes with shared materials.

#include "chain.h"

#include "body.h"
#include "shape.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

void m2RetireChainSlot(m2World* world, int32_t chainIndex)
{
    world->chainAlive[chainIndex] = 0;
    if (world->chainGenerations[chainIndex] == UINT16_MAX)
    {
        world->chainRetiredCount += 1;
        return;
    }
    world->chainGenerations[chainIndex] += 1;
    world->chainFreeQueue[world->chainFreeTail] = chainIndex;
    world->chainFreeTail = (world->chainFreeTail + 1) % world->shapeCapacity;
    world->chainFreeCount += 1;
}

m2ChainDef m2DefaultChainDef(void)
{
    m2ChainDef def;
    memset(&def, 0, sizeof(def));
    def.friction = 0.6f;
    def.categoryBits = 1;
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_CHAIN_COOKIE;
    return def;
}

m2ChainId m2CreateChain(m2BodyId bodyId, const m2ChainDef* def)
{
    m2World* world = m2GetBodyWorld(bodyId);
    int32_t bodyIndex = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (bodyIndex < 0 || def == NULL || def->internalValue != M2_CHAIN_COOKIE ||
        def->points == NULL || (def->isLoop ? def->count < 3 : def->count < 4) ||
        world->chainFreeCount == 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_nullChainId;
    }

    // Claim the chain slot before making shapes so each segment can be
    // tagged with its owner as it is born.
    int32_t chainIndex = world->chainFreeQueue[world->chainFreeHead];
    world->chainFreeHead = (world->chainFreeHead + 1) % world->shapeCapacity;
    world->chainFreeCount -= 1;

    // One journal op describes the whole chain; the per-shape creates
    // below must not double-record.
    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    m2ShapeDef shapeDef = m2DefaultShapeDef();
    shapeDef.density = 0.0f;
    shapeDef.friction = def->friction;
    shapeDef.restitution = def->restitution;
    shapeDef.categoryBits = def->categoryBits;
    shapeDef.maskBits = def->maskBits;
    shapeDef.groupIndex = def->groupIndex;
    shapeDef.userData = def->userData;

    int32_t segmentCount = def->isLoop ? def->count : def->count - 3;
    int32_t created = 0;
    for (int32_t i = 0; i < segmentCount; ++i)
    {
        m2ShapeGeometry geometry;
        memset(&geometry, 0, sizeof(geometry));
        geometry.type = m2_chainSegmentShape;
        if (def->isLoop)
        {
            int32_t n = def->count;
            geometry.chainSegment.ghost1 = def->points[(i + n - 1) % n];
            geometry.chainSegment.segment.point1 = def->points[i];
            geometry.chainSegment.segment.point2 = def->points[(i + 1) % n];
            geometry.chainSegment.ghost2 = def->points[(i + 2) % n];
        }
        else
        {
            geometry.chainSegment.ghost1 = def->points[i];
            geometry.chainSegment.segment.point1 = def->points[i + 1];
            geometry.chainSegment.segment.point2 = def->points[i + 2];
            geometry.chainSegment.ghost2 = def->points[i + 3];
        }
        m2ShapeId shape = m2CreateShape(bodyId, &shapeDef, &geometry);
        if (shape.index1 == 0)
        {
            break; // capacity: partial chain, loud via the segment count
        }
        world->shapeChain[shape.index1 - 1] = chainIndex;
        created += 1;
    }

    world->journalActive = journalWasActive;
    if (created == 0)
    {
        // Nothing was made: retire the claimed slot. The generation
        // burns, which keeps the id sequence append-only either way.
        m2RetireChainSlot(world, chainIndex);
        m2Refuse(world, m2_errorCapacity);
        return m2_nullChainId;
    }
    world->chainAlive[chainIndex] = 1;
    world->chainBody[chainIndex] = bodyIndex;
    if (chainIndex + 1 > world->maxChainIndex)
    {
        world->maxChainIndex = chainIndex + 1;
    }
    m2JournalRecordChain(world, bodyId, def, created);
    m2ChainId id = {chainIndex + 1, world->worldIndex0, world->chainGenerations[chainIndex]};
    return id;
}

static int32_t ChainSlot(const m2World* world, m2ChainId chainId)
{
    int32_t index = chainId.index1 - 1;
    if (index < 0 || index >= world->shapeCapacity || world->chainAlive[index] == 0 ||
        world->chainGenerations[index] != chainId.generation)
    {
        return -1;
    }
    return index;
}

void m2DestroyChain(m2ChainId chainId)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
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
    int32_t bodyIndex = world->chainBody[index];
    int32_t prev = -1;
    int32_t s = world->bodyShapeHead[bodyIndex];
    while (s != -1)
    {
        int32_t next = world->shapeNext[s];
        if (world->shapeChain[s] == index)
        {
            if (prev == -1)
            {
                world->bodyShapeHead[bodyIndex] = next;
            }
            else
            {
                world->shapeNext[prev] = next;
            }
            world->shapeNext[s] = -1;
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
    if (world->types[bodyIndex] == (uint8_t)m2_dynamicBody)
    {
        world->asleep[bodyIndex] = 0;
        world->sleepTimes[bodyIndex] = 0.0f;
    }
}

bool m2Chain_IsValid(m2ChainId chainId)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
    return world != NULL && ChainSlot(world, chainId) >= 0;
}

int32_t m2Chain_GetSegmentCount(m2ChainId chainId)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    int32_t count = 0;
    for (int32_t s = world->bodyShapeHead[world->chainBody[index]]; s != -1;
         s = world->shapeNext[s])
    {
        count += world->shapeChain[s] == index ? 1 : 0;
    }
    return count;
}

static void ChainMaterialInternal(m2World* world, m2ChainId chainId, int32_t chainIndex, uint8_t op,
                                  float value)
{
    if (world->journalActive != 0)
    {
        struct
        {
            m2ChainId chain;
            float value;
        } record;
        memset(&record, 0, sizeof(record));
        record.chain = chainId;
        record.value = value;
        m2JournalRecord(world, op, &record, (int32_t)sizeof(record));
    }
    int32_t body = world->chainBody[chainIndex];
    for (int32_t s = world->bodyShapeHead[body]; s != -1; s = world->shapeNext[s])
    {
        if (world->shapeChain[s] != chainIndex)
        {
            continue;
        }
        if (op == m2_opChainFriction)
        {
            world->shapeFriction[s] = value;
        }
        else
        {
            world->shapeRestitution[s] = value;
        }
    }
}

void m2Chain_SetFriction(m2ChainId chainId, float friction)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !(friction >= 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainFriction, friction);
}

void m2Chain_SetRestitution(m2ChainId chainId, float restitution)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
    int32_t index = world != NULL ? ChainSlot(world, chainId) : -1;
    if (index < 0 || !(restitution >= 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    ChainMaterialInternal(world, chainId, index, m2_opChainRestitution, restitution);
}

m2WorldId m2Chain_GetWorld(m2ChainId chainId)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
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

int32_t m2Chain_GetShapes(m2ChainId chainId, m2ShapeId* ids, int32_t capacity)
{
    m2World* world = m2WorldFromIndex(chainId.world0);
    int32_t chainIndex = world != NULL ? ChainSlot(world, chainId) : -1;
    if (chainIndex < 0)
    {
        return 0;
    }
    int32_t total = 0;
    for (int32_t i = 0; i < world->maxShapeIndex; ++i)
    {
        if (world->shapeAlive[i] == 0 || world->shapeChain[i] != chainIndex)
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
