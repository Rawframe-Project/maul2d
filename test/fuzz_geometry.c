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
//   - the pieces of a decomposed outline lie within its area: they may
//     drop slivers of near-zero area, never add any.

#include "helpers.h"

#include "maul2d/maul2d.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    FUZZ_MAX_POINTS = 64,
    FUZZ_MAX_PIECES = 64
};

static m2World* s_world;
static m2BodyId s_body;

static void Fail(const char* promise)
{
    printf("broken promise: %s\n", promise);
    fflush(stdout);
    abort();
}

// The shape a polygon makes on the fuzz body, destroyed again at once.
static void MustMakeShape(const m2Polygon* polygon)
{
    m2ShapeDef def = m2DefaultShapeDef();
    m2ShapeId shape = m2_nullShapeId;
    if (m2CreatePolygonShape(s_world, s_body, &def, polygon, &shape) != m2_success)
    {
        Fail("an accepted polygon makes a shape");
    }
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

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_world == NULL)
    {
        m2WorldDef def = m2DefaultWorldDef();
        def.shapeCapacity = 4;
        s_world = NewWorld(&def);
        m2BodyDef bodyDef = m2DefaultBodyDef();
        bodyDef.type = m2_dynamicBody;
        s_body = NewBody(s_world, &bodyDef);
        if (s_world == NULL || s_body.index1 == 0)
        {
            Fail("the fuzz body builds");
        }
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
    }

    m2Polygon pieces[FUZZ_MAX_PIECES];
    int32_t made = 0;
    if (m2DecomposeOutline(points, count, pieces, FUZZ_MAX_PIECES, &made) == m2_success)
    {
        if (made < 0 || made > FUZZ_MAX_PIECES)
        {
            Fail("a decomposition of 64 points fits 64 pieces");
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
