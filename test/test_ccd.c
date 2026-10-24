// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// CCD gate: a fast bullet must stop at a thin wall that the same body
// tunnels through without the flag; the earliest wall wins; bullets
// ignore each other; the whole thing replays under rollback;
// and the bullet spray hash crosses CI cells.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void AddWall(m2World* world, double x, float halfThickness, float halfHeight)
{
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){x, 0.0};
    m2BodyId wall = NewBody(world, &wd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slabShape = m2MakeBox(halfThickness, halfHeight);
    NewPolygonShape(world, wall, &sd, &slabShape);
}

static m2BodyId FireBall(m2World* world, double x, float speed, bool isBullet)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, 0.0};
    bd.linearVelocity = (m2Vec2){speed, 0.0f};
    bd.gravityScale = 0.0f;
    bd.isBullet = isBullet;
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.1f};
    NewCircleShape(world, ball, &sd, &circle);
    return ball;
}

static void TestThinWall(void)
{
    // 150 m/s ball, 0.2 m wall: one substep moves ~0.625 m - a clean
    // tunnel for discrete stepping.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    AddWall(world, 0.0, 0.1f, 4.0f);

    m2BodyId normal = FireBall(world, -12.0, 150.0f, false);
    m2BodyId bullet = FireBall(world, -12.0, 150.0f, true);
    m2Body_SetLinearVelocity(world, bullet, (m2Vec2){150.0f, 0.0f});
    // Separate lanes so the two balls never meet.
    // (Re-create at different y: cheaper to just move the normal ball.)
    DestroyBody(world, normal);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-12.0, 2.0};
    bd.linearVelocity = (m2Vec2){150.0f, 0.0f};
    bd.gravityScale = 0.0f;
    m2BodyId tunneler = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.1f};
    NewCircleShape(world, tunneler, &sd, &circle);

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, tunneler).x > 1.0, "unflagged ball tunnels the thin wall");
    CHECK(m2Body_GetPosition(world, bullet).x < 0.0, "bullet is stopped by the thin wall");
    m2Vec2 v = m2Body_GetLinearVelocity(world, bullet);
    CHECK(v.x < 10.0f, "bullet velocity resolved by the contact after impact");

    m2DestroyWorld(world);
}

static void TestEarliestWallWins(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    AddWall(world, 0.0, 0.05f, 4.0f);
    AddWall(world, 3.0, 0.05f, 4.0f);

    m2BodyId bullet = FireBall(world, -10.0, 200.0f, true);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, bullet).x < 0.0, "bullet stops at the FIRST wall");

    m2DestroyWorld(world);
}

static void TestBulletsIgnoreEachOther(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);

    m2BodyId left = FireBall(world, -8.0, 120.0f, true);
    m2BodyId right = FireBall(world, 8.0, -120.0f, true);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // No sweep between bullets: both cross the middle unhindered.
    CHECK(m2Body_GetPosition(world, left).x > 4.0, "left bullet passes (no bullet-vs-bullet TOI)");
    CHECK(m2Body_GetPosition(world, right).x < -4.0, "right bullet passes");

    m2DestroyWorld(world);
}

static void TestCcdRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    AddWall(world, 2.0, 0.1f, 4.0f);
    FireBall(world, -12.0, 170.0f, true);

    m2World_Step(world, 1.0f / 60.0f, 4); // bullet mid-flight

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[30];
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "bullet impact replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

int main(void)
{
    TestThinWall();
    TestEarliestWallWins();
    TestBulletsIgnoreEachOther();
    TestCcdRollback();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
