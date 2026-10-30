// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world the snapshot and journal fuzz targets start from: one of each
// kind of object, stepped until its contacts, joints, particles and
// volume all hold state, so a valid snapshot or tape of it reaches every
// array of the state table.

#ifndef MAUL2D_TEST_FUZZ_SCENE_H
#define MAUL2D_TEST_FUZZ_SCENE_H

#include "helpers.h"

#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

// A broken promise: say which, and crash so libFuzzer saves the input.
static void FuzzFail(const char* promise)
{
    printf("broken promise: %s\n", promise);
    fflush(stdout);
    abort();
}

static m2WorldDef FuzzSceneDef(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    def.particleCapacity = 64;
    def.waterVolumeCapacity = 2;
    return def;
}

// Fills a world made from FuzzSceneDef. Returns false when a call the
// scene relies on was refused.
static bool BuildFuzzScene(m2World* world)
{
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = NewBody(world, &groundDef);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(8.0f, 0.5f);
    m2Vec2 ridge[5] = {{-9.0f, 3.0f}, {-8.0f, 1.0f}, {-6.0f, 1.5f}, {-4.0f, 1.0f}, {-3.0f, 2.0f}};
    m2ChainDef chainDef = m2DefaultChainDef();
    chainDef.points = ridge;
    chainDef.count = 5;
    bool built = NewPolygonShape(world, ground, &shapeDef, &slab).index1 != 0 &&
                 NewChain(world, ground, &chainDef).index1 != 0;

    m2BodyDef boxDef = m2DefaultBodyDef();
    boxDef.type = m2_dynamicBody;
    m2BodyId bodies[3];
    for (int32_t i = 0; i < 3; ++i)
    {
        boxDef.position = (m2Pos2){-1.0 + (double)i, 1.0 + 0.6 * (double)i};
        bodies[i] = NewBody(world, &boxDef);
    }
    m2Polygon box = m2MakeBox(0.3f, 0.3f);
    m2Circle ball = {{0.0f, 0.0f}, 0.3f};
    m2Capsule pill = {{-0.2f, 0.0f}, {0.2f, 0.0f}, 0.15f};
    built = built && NewPolygonShape(world, bodies[0], &shapeDef, &box).index1 != 0 &&
            NewCircleShape(world, bodies[1], &shapeDef, &ball).index1 != 0 &&
            NewCapsuleShape(world, bodies[2], &shapeDef, &pill).index1 != 0;

    m2RevoluteJointDef hinge = m2DefaultRevoluteJointDef();
    hinge.bodyIdA = bodies[0];
    hinge.bodyIdB = bodies[1];
    hinge.enableLimit = true;
    hinge.lowerAngle = -0.5f;
    hinge.upperAngle = 0.5f;
    m2DistanceJointDef rope = m2DefaultDistanceJointDef();
    rope.bodyIdA = bodies[1];
    rope.bodyIdB = bodies[2];
    rope.length = 1.0f;
    built = built && NewRevoluteJoint(world, &hinge).index1 != 0 &&
            NewDistanceJoint(world, &rope).index1 != 0;

    for (int32_t i = 0; i < 8; ++i)
    {
        m2Pos2 at = {2.0 + 0.1 * (double)i, 0.5};
        built = built && EmitParticle(world, at, (m2Vec2){0.0f, 0.0f}, 0).index1 != 0;
    }
    // A small spring fill, so a tape of the scene carries a fill record
    // and its polygon.
    m2Polygon drop = m2MakeBox(0.1f, 0.1f);
    built = built && FillParticles(world, &drop, (m2Pos2){3.0, 1.0}, (m2Vec2){0.0f, 0.0f},
                                   m2_springParticle) > 0;
    m2WaterVolumeDef pool = m2DefaultWaterVolumeDef();
    pool.bounds = (m2WorldAabb){{-4.0, -0.5}, {4.0, 0.5}};
    pool.surface = 0.2;
    built = built && NewWaterVolume(world, &pool).index1 != 0;

    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    return built;
}

#endif // MAUL2D_TEST_FUZZ_SCENE_H
