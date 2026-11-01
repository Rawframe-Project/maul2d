// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fuzzes the geometry builders that take caller points: m2MakePolygon,
// m2ComputeHull and m2DecomposeOutline. The first input byte picks the
// radius and how the rest becomes points: raw floats (NaN, infinity and
// huge values included) or small grid coordinates, which form outlines
// random floats rarely do. The promises:
//   - a refusal is a status, never a crash;
//   - every polygon a builder accepts makes a shape;
//   - no point lies further outside an accepted hull than welding (0.02
//     m) and merging (0.01 m) allow;
//   - an accepted outline gives at least one piece, and its pieces lie
//     within its area: they may drop slivers of near-zero area, never add
//     any.

#include "helpers.h"
#include "world_internal.h"

#include "maul2d/maul2d.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    FUZZ_MAX_POINTS = 64,
    FUZZ_MAX_PIECES = 64,
    FUZZ_SHAPE_SLOTS = 4
};

// The shapes the fuzz world makes before every slot has retired: each
// slot names one per generation.
static const uint64_t k_fuzzShapeLife =
    (uint64_t)FUZZ_SHAPE_SLOTS * ((uint64_t)M2_LAST_GENERATION + 1u);

static m2World* s_world;
static m2BodyId s_body;
static uint64_t s_shapesMade; // since the fuzz world was built

static void Fail(const char* promise)
{
    printf("broken promise: %s\n", promise);
    fflush(stdout);
    abort();
}

// The fuzz world: one dynamic body whose shapes come and go.
static void BuildFuzzWorld(void)
{
    if (s_world != NULL)
    {
        m2DestroyWorld(s_world);
    }
    m2WorldDef def = m2DefaultWorldDef();
    def.shapeCapacity = FUZZ_SHAPE_SLOTS;
    s_world = NewWorld(&def);
    s_shapesMade = 0;
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    s_body = NewBody(s_world, &bodyDef);
    if (s_world == NULL || s_body.index1 == 0)
    {
        Fail("the fuzz body builds");
    }
}

// The shape a polygon makes on the fuzz body, destroyed again at once.
// Once every slot has retired the pool is full for good, and the fuzz
// world is built again; a full pool any earlier is a leaked or retired
// slot.
static void MustMakeShape(const m2Polygon* polygon)
{
    m2ShapeDef def = m2DefaultShapeDef();
    m2ShapeId shape = m2_nullShapeId;
    m2Result made = m2CreatePolygonShape(s_world, s_body, &def, polygon, &shape);
    if (made == m2_errorCapacity && s_shapesMade == k_fuzzShapeLife)
    {
        BuildFuzzWorld();
        made = m2CreatePolygonShape(s_world, s_body, &def, polygon, &shape);
    }
    if (made != m2_success)
    {
        Fail("an accepted polygon makes a shape");
    }
    s_shapesMade += 1;
    if (m2DestroyShape(s_world, shape) != m2_success)
    {
        Fail("the shape goes again");
    }
}

static double OutlineArea(const m2Vec2* points, int32_t count)
{
    double twice = 0.0;
    for (int32_t i = 0; i < count; ++i)
    {
        const m2Vec2 a = points[i];
        const m2Vec2 b = points[(i + 1) % count];
        twice += (double)a.x * (double)b.y - (double)b.x * (double)a.y;
    }
    return 0.5 * twice;
}

static double PolygonArea(const m2Polygon* polygon)
{
    return OutlineArea(polygon->vertices, polygon->count);
}

// How far the farthest point lies outside the polygon's edges, and the
// largest coordinate among the points, whose float rounding the bound
// allows for.
static double FarthestOutside(const m2Polygon* polygon, const m2Vec2* points, int32_t count,
                              double* scale)
{
    double farthest = 0.0;
    *scale = 0.0;
    for (int32_t i = 0; i < count; ++i)
    {
        double x = fabs((double)points[i].x);
        double y = fabs((double)points[i].y);
        *scale = x > *scale ? x : *scale;
        *scale = y > *scale ? y : *scale;
        for (int32_t e = 0; e < polygon->count; ++e)
        {
            const m2Vec2 v = polygon->vertices[e];
            const m2Vec2 n = polygon->normals[e];
            double out = ((double)points[i].x - (double)v.x) * (double)n.x +
                         ((double)points[i].y - (double)v.y) * (double)n.y;
            farthest = out > farthest ? out : farthest;
        }
    }
    return farthest;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_world == NULL)
    {
        BuildFuzzWorld();
    }
    if (size == 0)
    {
        return 0;
    }

    m2Vec2 points[FUZZ_MAX_POINTS];
    int32_t count = 0;
    bool grid = (data[0] & 1u) != 0;
    float radius = (float)((data[0] >> 1) & 7u) * 0.05f;
    size_t stride = grid ? 2 : 8;
    for (size_t at = 1; at + stride <= size && count < FUZZ_MAX_POINTS; at += stride)
    {
        if (grid)
        {
            points[count].x = (float)((int32_t)data[at] - 128) * 0.125f;
            points[count].y = (float)((int32_t)data[at + 1] - 128) * 0.125f;
        }
        else
        {
            memcpy(&points[count].x, data + at, sizeof(float));
            memcpy(&points[count].y, data + at + 4, sizeof(float));
        }
        count += 1;
    }

    m2Polygon polygon;
    if (m2MakePolygon(points, count, radius, &polygon) == m2_success)
    {
        MustMakeShape(&polygon);
    }
    if (m2ComputeHull(points, count, radius, &polygon) == m2_success)
    {
        MustMakeShape(&polygon);
        double scale = 0.0;
        if (FarthestOutside(&polygon, points, count, &scale) > 0.03 + 1.0e-5 * scale)
        {
            Fail("no point lies more than 0.03 m outside its hull");
        }
    }

    m2Polygon pieces[FUZZ_MAX_PIECES];
    int32_t made = 0;
    if (m2DecomposeOutline(points, count, pieces, FUZZ_MAX_PIECES, &made) == m2_success)
    {
        if (made < 1 || made > FUZZ_MAX_PIECES)
        {
            Fail("a decomposition of 64 points makes 1 to 64 pieces");
        }
        double covered = 0.0;
        for (int32_t i = 0; i < made; ++i)
        {
            MustMakeShape(&pieces[i]);
            covered += PolygonArea(&pieces[i]);
        }
        double area = OutlineArea(points, count);
        if (!(area > 0.0) || covered > area + 1.0e-3 * area + 1.0e-4)
        {
            Fail("the pieces of a counter-clockwise outline lie within its area");
        }
    }
    return 0;
}
