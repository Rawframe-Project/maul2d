// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase seen through the counters: a wide mover pairs with
// everything it overlaps, a full pair table counts what it drops, and
// pairs met from both ends are kept once.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <stdio.h>

static void TestWideMoverPairsEverything(void)
{
    // One dynamic slab over 300 static posts: every post pairs with it,
    // past the 256 hits a single tree query once kept.
    enum
    {
        POSTS = 300
    };
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = POSTS + 4;
    def.shapeCapacity = POSTS + 4;
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < POSTS; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){0.1 * (double)i, 0.0};
        m2Circle post = {{0.0f, 0.0f}, 0.05f};
        NewCircleShape(world, NewBody(world, &bd), &sd, &post);
    }
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.gravityScale = 0.0f;
    bd.position = (m2Pos2){15.0, 0.0};
    m2Polygon slab = m2MakeBox(16.0f, 0.2f);
    NewPolygonShape(world, NewBody(world, &bd), &sd, &slab);
    Step(world, 1.0f / 60.0f, 4);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.contactCount == POSTS, "the slab pairs with all 300 posts");
    CHECK(c.pairsDropped == 0, "and nothing overflowed");
    m2DestroyWorld(world);
}

static void TestFullPairTableIsCounted(void)
{
    // Twenty discs stacked on one spot want 190 pairs, more than a
    // world of twenty shapes keeps. The pairs that do not fit are
    // counted, not silently lost.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 20;
    def.shapeCapacity = 20;
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 20; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.001 * (double)i, 0.0};
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        NewCircleShape(world, NewBody(world, &bd), &sd, &disc);
    }
    Step(world, 1.0f / 60.0f, 1);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.pairsDropped > 0, "some pairs do not fit");
    CHECK(c.contactCount + c.pairsDropped == 190,
          "and every one of the 190 is either kept or counted");
    m2DestroyWorld(world);
}

static void TestDuplicateCandidatesDoNotCrowdTheTable(void)
{
    // Seventeen discs on one spot make 136 distinct pairs, which fit a
    // table of 144. Every pair is met from both ends while all of them
    // move; recording it once keeps the duplicates from crowding out
    // pairs that fit.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 18;
    def.shapeCapacity = 18;
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 17; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.001 * (double)i, 0.0};
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        NewCircleShape(world, NewBody(world, &bd), &sd, &disc);
    }
    Step(world, 1.0f / 60.0f, 1);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.contactCount == 136 && c.pairsDropped == 0, "all 136 distinct pairs fit");
    m2DestroyWorld(world);
}

// Every shape slot fits every tree: a full shape pool in one tree, then
// moved into another body type's tree a body at a time, disabled and
// enabled again, and refilled after destroys. No insert fails (the world
// asserts it), every shape keeps its proxy and the world validates.
static void TestFullShapePoolFitsEveryTree(void)
{
    enum
    {
        SHAPES = 16
    };
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = SHAPES;
    def.shapeCapacity = SHAPES;
    m2World* world = NewWorld(&def);
    m2BodyId bodies[SHAPES];
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    for (int32_t i = 0; i < SHAPES; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){(double)i, 0.0};
        bodies[i] = NewBody(world, &bd);
        NewCircleShape(world, bodies[i], &sd, &ball);
    }
    CHECK(m2World_GetCounters(world).shapeCount == SHAPES && m2World_Validate(world),
          "a full shape pool fits the static tree");
    for (int32_t i = 0; i < SHAPES; ++i)
    {
        m2Body_SetType(world, bodies[i], m2_kinematicBody);
    }
    CHECK(m2World_Validate(world), "and moves into the kinematic tree a body at a time");
    for (int32_t i = 0; i < SHAPES; ++i)
    {
        m2Body_SetType(world, bodies[i], m2_dynamicBody);
        m2Body_Enable(world, bodies[i], false);
    }
    for (int32_t i = 0; i < SHAPES; ++i)
    {
        m2Body_Enable(world, bodies[i], true);
    }
    Step(world, 1.0f / 60.0f, 4);
    CHECK(m2World_Validate(world), "then into the dynamic tree, out of it and back");
    for (int32_t i = 0; i < SHAPES; i += 2)
    {
        CHECK(DestroyBody(world, bodies[i]) == m2_success, "half the bodies go");
    }
    for (int32_t i = 0; i < SHAPES; i += 2)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){(double)i, 2.0};
        bodies[i] = NewBody(world, &bd);
        NewCircleShape(world, bodies[i], &sd, &ball);
    }
    Step(world, 1.0f / 60.0f, 4);
    CHECK(m2World_GetCounters(world).shapeCount == SHAPES && m2World_Validate(world),
          "and the refilled pool fits again");
    m2DestroyWorld(world);
}

int main(void)
{
    TestWideMoverPairsEverything();
    TestFullPairTableIsCounted();
    TestDuplicateCandidatesDoNotCrowdTheTable();
    TestFullShapePoolFitsEveryTree();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
