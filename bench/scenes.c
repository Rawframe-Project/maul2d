// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark scenes, shared by the benchmark, which times them, and the
// golden test, which checks their final world hashes.

#include "scenes.h"

#include <math.h>

static m2WorldId MakeWorld(int32_t bodies, int32_t joints, int32_t particles)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = bodies;
    def.shapeCapacity = bodies;
    def.jointCapacity = joints > 0 ? joints : 1;
    def.particleCapacity = particles;
    return m2CreateWorld(&def);
}

static void AddGround(m2WorldId world, float halfWidth)
{
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(halfWidth, 1.0f);
    m2CreatePolygonShape(ground, &sd, &slab);
}

static m2WorldId BuildPyramid(int32_t rows)
{
    m2WorldId world = MakeWorld(rows * (rows + 1) / 2 + 1, 0, 0);
    AddGround(world, 2.0f * (float)rows + 10.0f);
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.6f;
    for (int32_t i = 0; i < rows; ++i)
    {
        for (int32_t j = i; j < rows; ++j)
        {
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){(double)j - 0.5 * (double)(rows + i), 0.55 + 1.05 * (double)i};
            m2CreatePolygonShape(m2CreateBody(world, &bd), &sd, &box);
        }
    }
    return world;
}

static m2WorldId ScenePyramid15(void)
{
    return BuildPyramid(15);
}

static m2WorldId ScenePyramid30(void)
{
    return BuildPyramid(30);
}

// A revolute chain hanging from a static anchor.
static m2WorldId BuildChain(int32_t links, float halfLength, float density)
{
    m2WorldId world = MakeWorld(links + 1, links, 0);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 20.0};
    m2BodyId prev = m2CreateBody(world, &ad);
    m2Polygon link = m2MakeBox(halfLength, 0.125f);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = density;
    sd.friction = 0.2f;
    for (int32_t i = 0; i < links; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){(double)halfLength * (double)(2 * i + 1), 20.0};
        m2BodyId body = m2CreateBody(world, &bd);
        m2CreatePolygonShape(body, &sd, &link);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = prev;
        jd.bodyIdB = body;
        jd.localAnchorA = i == 0 ? (m2Vec2){0.0f, 0.0f} : (m2Vec2){halfLength, 0.0f};
        jd.localAnchorB = (m2Vec2){-halfLength, 0.0f};
        m2CreateRevoluteJoint(world, &jd);
        prev = body;
    }
    return world;
}

static m2WorldId SceneChain30(void)
{
    return BuildChain(30, 0.5f, 20.0f);
}

static m2WorldId SceneChain2000(void)
{
    return BuildChain(2000, 0.25f, 1.0f);
}

// Forty motorized windmills and forty limited swing arms.
static m2WorldId SceneJointFarm(void)
{
    m2WorldId world = MakeWorld(160, 80, 0);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 40; ++i)
    {
        double x = 4.0 * (double)i;
        m2BodyDef td = m2DefaultBodyDef();
        td.position = (m2Pos2){x, 5.0};
        m2BodyId tower = m2CreateBody(world, &td);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){x, 5.0};
        m2BodyId blade = m2CreateBody(world, &wd);
        m2Polygon bar = m2MakeBox(1.0f, 0.08f);
        m2CreatePolygonShape(blade, &sd, &bar);
        m2RevoluteJointDef mill = m2DefaultRevoluteJointDef();
        mill.bodyIdA = tower;
        mill.bodyIdB = blade;
        mill.enableMotor = true;
        mill.motorSpeed = 2.5f;
        mill.maxMotorTorque = 30.0f;
        m2CreateRevoluteJoint(world, &mill);

        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){x + 2.0, 8.0};
        m2BodyId pivot = m2CreateBody(world, &pd);
        m2BodyDef rd = m2DefaultBodyDef();
        rd.type = m2_dynamicBody;
        rd.position = (m2Pos2){x + 2.6, 8.0};
        m2BodyId rod = m2CreateBody(world, &rd);
        m2Polygon rodBox = m2MakeBox(0.6f, 0.05f);
        m2CreatePolygonShape(rod, &sd, &rodBox);
        m2RevoluteJointDef swing = m2DefaultRevoluteJointDef();
        swing.bodyIdA = pivot;
        swing.bodyIdB = rod;
        swing.localAnchorB = (m2Vec2){-0.6f, 0.0f};
        swing.enableLimit = true;
        swing.lowerAngle = -0.6f;
        swing.upperAngle = 0.6f;
        m2CreateRevoluteJoint(world, &swing);
    }
    return world;
}

// A block of water particles poured into a box.
static m2WorldId SceneWater(void)
{
    m2WorldId world = MakeWorld(4, 0, 2048);
    AddGround(world, 12.0f);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t side = -1; side <= 1; side += 2)
    {
        m2BodyDef wd = m2DefaultBodyDef();
        wd.position = (m2Pos2){6.0 * (double)side, 4.0};
        m2Polygon wall = m2MakeBox(0.25f, 5.0f);
        m2CreatePolygonShape(m2CreateBody(world, &wd), &sd, &wall);
    }
    for (int32_t i = 0; i < 2000; ++i)
    {
        m2Pos2 p = {-4.0 + 0.2 * (double)(i % 40), 1.0 + 0.2 * (double)(i / 40)};
        m2World_EmitParticle(world, p, (m2Vec2){0.0f, 0.0f}, 0);
    }
    return world;
}

static const BenchScene kScenes[] = {
    {"pyramid15", ScenePyramid15, 600, 1.0f},  {"pyramid30", ScenePyramid30, 600, 1.0f},
    {"chain30", SceneChain30, 1200, 50.0f},    {"chain2000", SceneChain2000, 120, 50.0f},
    {"jointfarm", SceneJointFarm, 300, 20.0f}, {"water", SceneWater, 300, 50.0f},
};

float BenchMaxBodySpeed(m2WorldId world)
{
    m2BodyId bodies[4096];
    int32_t count = m2World_GetBodies(world, bodies, 4096);
    float best = 0.0f;
    for (int32_t i = 0; i < count && i < 4096; ++i)
    {
        m2Vec2 v = m2Body_GetLinearVelocity(bodies[i]);
        float speed = sqrtf(v.x * v.x + v.y * v.y);
        best = speed > best ? speed : best;
    }
    return best;
}

const BenchScene* BenchScenes(int32_t* count)
{
    *count = (int32_t)(sizeof(kScenes) / sizeof(kScenes[0]));
    return kScenes;
}
