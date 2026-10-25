// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Shape readback: type, sensor flag, filter, geometry and density.
// Pure readers.

#include "fpenv.h"
#include "object_ids.h"
#include "shape.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <string.h>

m2ShapeType m2Shape_GetType(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return m2_circleShape;
    }
    return (m2ShapeType)world->shapes.shapeGeometry[index].type;
}

bool m2Shape_IsSensor(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return world->shapes.shapeSensor[index] != 0;
}

void m2Shape_GetFilter(const m2World* world, m2ShapeId shapeId, uint64_t* categoryBitsOut,
                       uint64_t* maskBitsOut, int32_t* groupIndexOut)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
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
    if (categoryBitsOut != NULL)
    {
        *categoryBitsOut = category;
    }
    if (maskBitsOut != NULL)
    {
        *maskBitsOut = mask;
    }
    if (groupIndexOut != NULL)
    {
        *groupIndexOut = group;
    }
}

// Geometry readback: exact stored bits, loud on a type mismatch.
#define M2_GEOMETRY_GETTER(name, fieldType, field, enumValue)                                      \
    fieldType name(const m2World* world, m2ShapeId shapeId)                                        \
    {                                                                                              \
        fieldType zero;                                                                            \
        memset(&zero, 0, sizeof(zero));                                                            \
        int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;                          \
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

static M2_NOINLINE float ShapeGetDensityWorker(const m2World* world, m2ShapeId shapeId)
{
    int32_t index = world != NULL ? m2ShapeSlot(world, shapeId) : -1;
    if (index < 0)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0.0f;
    }
    return world->shapes.shapeDensity[index];
}

float m2Shape_GetDensity(const m2World* world, m2ShapeId shapeId)
{
    uint64_t env = m2FloatEnvEnter();
    float result = ShapeGetDensityWorker(world, shapeId);
    m2FloatEnvLeave(env);
    return result;
}
