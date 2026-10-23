// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The broadphase seen through the counters: a wide mover pairs with
// everything it overlaps, a full pair table counts what it drops, and
// pairs met from both ends are kept once.

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
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < POSTS; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.position = (m2Pos2){0.1 * (double)i, 0.0};
        m2Circle post = {{0.0f, 0.0f}, 0.05f};
        m2CreateCircleShape(m2CreateBody(world, &bd), &sd, &post);
    }
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.gravityScale = 0.0f;
    bd.position = (m2Pos2){15.0, 0.0};
    m2Polygon slab = m2MakeBox(16.0f, 0.2f);
    m2CreatePolygonShape(m2CreateBody(world, &bd), &sd, &slab);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.pairs == POSTS, "the slab pairs with all 300 posts");
    CHECK(c.pairOverflow == 0, "and nothing overflowed");
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
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 20; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.001 * (double)i, 0.0};
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        m2CreateCircleShape(m2CreateBody(world, &bd), &sd, &disc);
    }
    m2World_Step(world, 1.0f / 60.0f, 1);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.pairOverflow > 0, "some pairs do not fit");
    CHECK(c.pairs + c.pairOverflow == 190, "and every one of the 190 is either kept or counted");
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
    m2WorldId world = m2CreateWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 17; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.001 * (double)i, 0.0};
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        m2CreateCircleShape(m2CreateBody(world, &bd), &sd, &disc);
    }
    m2World_Step(world, 1.0f / 60.0f, 1);
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.pairs == 136 && c.pairOverflow == 0, "all 136 distinct pairs fit");
    m2DestroyWorld(world);
}

int main(void)
{
    TestWideMoverPairsEverything();
    TestFullPairTableIsCounted();
    TestDuplicateCandidatesDoNotCrowdTheTable();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
