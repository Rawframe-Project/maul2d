// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The translation between public body and shape ids and their slots.

#include "object_ids.h"

int32_t m2BodySlot(const m2World* world, m2BodyId id)
{
    int32_t index = (int32_t)id.index1 - 1;
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

int32_t m2ShapeSlot(const m2World* world, m2ShapeId id)
{
    int32_t index = (int32_t)id.index1 - 1;
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
    m2ShapeId id = {(uint32_t)(shapeIndex + 1), world->shapes.shapeGenerations[shapeIndex]};
    return id;
}
