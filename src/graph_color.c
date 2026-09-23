// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph coloring: constraints of one color share no dynamic body, so a
// color can be solved in parallel.

#include "graph_color.h"

#include "solver.h"
#include "world_internal.h"

#include "maul2d/base.h"

void m2ColorConstraints(m2World* world, m2ContactConstraint* constraints, int32_t count,
                        int32_t* colorStart)
{
    uint32_t* masks = world->solver.colorMasks;
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        masks[i] = 0;
    }
    int32_t counts[M2_GRAPH_COLORS + 1];
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        counts[c] = 0;
    }

    for (int32_t i = 0; i < count; ++i)
    {
        int32_t bodyA = constraints[i].bodyA;
        int32_t bodyB = constraints[i].bodyB;
        bool dynA = world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody;
        bool dynB = world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody;
        uint32_t used = (dynA ? masks[bodyA] : 0u) | (dynB ? masks[bodyB] : 0u);
        int32_t color = 0;
        while (color < M2_GRAPH_COLORS && (used & (1u << color)) != 0)
        {
            color += 1;
        }
        if (color < M2_GRAPH_COLORS && counts[color] >= 256)
        {
            color = M2_GRAPH_COLORS; // full color: spill to the scalar bucket
        }
        if (color < M2_GRAPH_COLORS)
        {
            if (dynA)
            {
                masks[bodyA] |= 1u << color;
            }
            if (dynB)
            {
                masks[bodyB] |= 1u << color;
            }
        }
        world->solver.constraintColors[i] = (uint8_t)color;
        counts[color] += 1;
    }

    colorStart[0] = 0;
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        colorStart[c + 1] = colorStart[c] + counts[c];
    }
    int32_t cursor[M2_GRAPH_COLORS + 1];
    for (int32_t c = 0; c <= M2_GRAPH_COLORS; ++c)
    {
        cursor[c] = colorStart[c];
    }
    for (int32_t i = 0; i < count; ++i)
    {
        int32_t color = world->solver.constraintColors[i];
        world->solver.colorOrder[cursor[color]] = i;
        cursor[color] += 1;
    }
}
