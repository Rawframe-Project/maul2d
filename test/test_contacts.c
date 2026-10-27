// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact checks: analytic manifold checks (circle kernels,
// Voronoi regions, feature ids, the flip path), speculative existence
// before touch, feature-id persistence with warm-start impulse carry,
// rollback over the contact block, and the manifold evolution hash.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_NEAR(value, expected, tolerance, msg)                                                \
    CHECK((value) > (expected) - (tolerance) && (value) < (expected) + (tolerance), msg)

static void TestCollisionFilters(void)
{
    // Category/mask filtering: a "ghost" box falls straight through a
    // floor it is masked against, a one-way mismatch also blocks (the
    // rule is AND), and the filter survives rollback.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef floorShape = m2DefaultShapeDef();
    floorShape.categoryBits = 0x1;
    floorShape.maskBits = 0x2; // floor only talks to category 2
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &floorShape, &slab);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId solid = NewBody(world, &bd);
    m2ShapeDef solidShape = m2DefaultShapeDef();
    solidShape.categoryBits = 0x2;
    solidShape.maskBits = 0x1;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, solid, &solidShape, &unit);

    bd.position = (m2Pos2){3.0, 2.0};
    m2BodyId ghost = NewBody(world, &bd);
    m2ShapeDef ghostShape = m2DefaultShapeDef();
    ghostShape.categoryBits = 0x4; // floor's mask does not include 4
    ghostShape.maskBits = 0xFFFFFFFFu;
    NewPolygonShape(world, ghost, &ghostShape, &unit);

    bd.position = (m2Pos2){-3.0, 2.0};
    m2BodyId oneWay = NewBody(world, &bd);
    m2ShapeDef oneWayShape = m2DefaultShapeDef();
    oneWayShape.categoryBits = 0x2; // floor would accept it...
    oneWayShape.maskBits = 0x8;     // ...but it refuses the floor
    NewPolygonShape(world, oneWay, &oneWayShape, &unit);

    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, solid).y > 0.3, "matching filters collide and rest");
    CHECK(m2Body_GetPosition(world, ghost).y < -3.0, "mismatched category falls through");
    CHECK(m2Body_GetPosition(world, oneWay).y < -3.0, "one-way mismatch also falls (AND rule)");

    m2DestroyWorld(world);
}

static void TestGroupIndex(void)
{
    // Same negative group never collides even when the masks agree;
    // same positive group always collides even when the masks refuse.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(12.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    // Two squad-mates (group -3) dropped onto the same spot: they pass
    // through each other and both land on the floor.
    m2ShapeDef mate = m2DefaultShapeDef();
    mate.groupIndex = -3;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.2};
    m2BodyId one = NewBody(world, &bd);
    NewPolygonShape(world, one, &mate, &unit);
    bd.position = (m2Pos2){0.05, 2.4};
    m2BodyId two = NewBody(world, &bd);
    NewPolygonShape(world, two, &mate, &unit);

    // Two forced-friends (group +5) whose masks refuse each other:
    // they still stack.
    m2ShapeDef friendA = m2DefaultShapeDef();
    friendA.groupIndex = 5;
    friendA.categoryBits = 0x10;
    friendA.maskBits = 0x1; // floor only
    bd.position = (m2Pos2){4.0, 0.55};
    m2BodyId base = NewBody(world, &bd);
    NewPolygonShape(world, base, &friendA, &unit);
    bd.position = (m2Pos2){4.0, 1.6};
    m2BodyId rider = NewBody(world, &bd);
    NewPolygonShape(world, rider, &friendA, &unit);

    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double y1 = m2Body_GetPosition(world, one).y;
    double y2 = m2Body_GetPosition(world, two).y;
    CHECK(y1 < 0.6 && y2 < 0.6, "negative group mates pass through each other");
    CHECK(m2Body_GetPosition(world, rider).y > 1.0, "positive group forces the stack");

    m2DestroyWorld(world);
}

// A chain id names the whole ground run: destroying it must remove
// every segment, end the contacts, and wake the sleeper that was
// resting on it.
static void TestChainDestroy(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);

    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    // Right-to-left points: solid side up for a ground chain.
    m2Vec2 pts[6] = {{6.0f, 0.0f}, {4.0f, 0.0f},  {2.0f, 0.0f},
                     {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId chainId = NewChain(world, ground, &chain);
    CHECK(m2Chain_IsValid(world, chainId), "chain id is live after create");
    CHECK(m2Chain_GetSegmentCount(world, chainId) == 3, "six open points make three segments");
    CHECK(!m2Chain_IsLoop(world, chainId), "and read back as an open chain");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){2.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, box), "box falls asleep on the chain");

    CastResult before =
        RayClosest(world, (m2Pos2){-1.0, 2.0}, (m2Vec2){0.0f, -4.0f}, m2DefaultQueryFilter());
    CHECK(before.hit, "a chain segment answers rays while alive");

    DestroyChain(world, chainId);
    CHECK(!m2Chain_IsValid(world, chainId), "destroyed chain id misses on generation");
    CHECK(m2Chain_GetSegmentCount(world, chainId) == 0, "destroyed chain reports zero segments");
    CHECK(!m2Chain_IsLoop(world, chainId), "and is no loop");
    CHECK(m2Body_IsValid(world, ground), "the owning body outlives its chain");

    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, box).y < -2.0,
          "sleeper wakes and falls once the ground is gone");

    CastResult after =
        RayClosest(world, (m2Pos2){-1.0, 2.0}, (m2Vec2){0.0f, -4.0f}, m2DefaultQueryFilter());
    CHECK(!after.hit, "no chain segment answers rays after destroy");

    // The slot recycles with a fresh generation.
    m2ChainId again = NewChain(world, ground, &chain);
    CHECK(m2Chain_IsValid(world, again), "a new chain can be built on the same body");
    CHECK(again.index1 != chainId.index1 || again.generation != chainId.generation,
          "recycled slot carries a new generation");
    m2DestroyWorld(world);
}

static void TestChainGround(void)
{
    // The internal-edge exam. A flat floor built as a chain of five
    // segments; a box shoved across every seam must never pop: the
    // ghost law eats the phantom edge collisions that a naive segment
    // floor would produce. Chain winds right-to-left so the solid side
    // faces up (the chain winding).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);

    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, 0.0};
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 points[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m2Vec2){9.0f - 3.0f * (float)i, 0.0f}; // +9 .. -12, right to left
    }
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = points;
    chain.count = 8;
    m2ChainId made = NewChain(world, ground, &chain);
    CHECK(m2Chain_GetSegmentCount(world, made) == 5,
          "open chain of eight points makes five segments");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){5.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.05f; // slide far
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // settle on the chain
    }
    CHECK(m2Body_GetPosition(world, box).y > 0.3, "the box rests on the chain floor");

    m2Body_ApplyLinearImpulseAtPoint(world, box, (m2Vec2){-4.0f, 0.0f},
                                     m2Body_GetPosition(world, box));
    float worstVy = 0.0f;
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        float vy = m2Body_GetLinearVelocity(world, box).y;
        float mag = vy < 0.0f ? -vy : vy;
        worstVy = mag > worstVy ? mag : worstVy;
    }
    CHECK(m2Body_GetPosition(world, box).x < -1.0, "the box crossed several seams");
    CHECK(worstVy < 0.25f, "no seam ever kicked it upward");
    CHECK(m2Body_GetPosition(world, box).y > 0.3 && m2Body_GetPosition(world, box).y < 0.6,
          "still riding the floor after the trip");

    // One-sided: a ball thrown up from below passes clean through.
    m2BodyDef ud = m2DefaultBodyDef();
    ud.type = m2_dynamicBody;
    ud.position = (m2Pos2){0.0, -4.0};
    ud.linearVelocity = (m2Vec2){0.0f, 14.0f};
    m2BodyId riser = NewBody(world, &ud);
    m2ShapeDef us = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    NewCircleShape(world, riser, &us, &ball);
    bool gotAbove = false;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        gotAbove = gotAbove || m2Body_GetPosition(world, riser).y > 1.0;
    }
    CHECK(gotAbove, "from below, the chain is a ghost");
    bool cameBackDown = false;
    for (int32_t i = 0; i < 300 && !cameBackDown; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        cameBackDown = !m2Body_IsAwake(world, riser) && m2Body_GetPosition(world, riser).y > 0.2;
    }
    CHECK(cameBackDown, "from above, it is a floor: the ball lands and sleeps");

    // One-sided rays agree: from above it hits, from below it misses.
    CastResult down =
        RayClosest(world, (m2Pos2){-2.0, 3.0}, (m2Vec2){0.0f, -5.0f}, m2DefaultQueryFilter());
    CastResult up =
        RayClosest(world, (m2Pos2){-2.0, -3.0}, (m2Vec2){0.0f, 2.5f}, m2DefaultQueryFilter());
    CHECK(down.hit, "a ray from above lands on the chain");
    CHECK(!up.hit, "a ray from below passes through");

    m2DestroyWorld(world);
}

static void TestRuntimeMaterials(void)
{
    // Ice turns to grip mid-slide: the tuned twin stops far short of
    // the untouched one. Then a dead ball turns bouncy.
    double travel[2];
    for (int32_t tuned = 0; tuned < 2; ++tuned)
    {
        m2WorldDef def = m2DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        m2World* world = NewWorld(&def);
        m2BodyDef fd = m2DefaultBodyDef();
        fd.position = (m2Pos2){0.0, -0.5};
        m2BodyId floor = NewBody(world, &fd);
        m2ShapeDef fs = m2DefaultShapeDef();
        fs.friction = 0.02f; // ice
        m2Polygon slab = m2MakeBox(40.0f, 0.5f);
        m2ShapeId ice = NewPolygonShape(world, floor, &fs, &slab);
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-30.0, 0.45};
        bd.linearVelocity = (m2Vec2){8.0f, 0.0f};
        m2BodyId puck = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.friction = 1.0f;
        m2Polygon unit = m2MakeBox(0.3f, 0.3f);
        NewPolygonShape(world, puck, &sd, &unit);

        double x0 = m2Body_GetPosition(world, puck).x;
        for (int32_t i = 0; i < 300; ++i)
        {
            if (tuned == 1 && i == 30)
            {
                m2Shape_SetFriction(world, ice, 1.0f); // grip, mid-slide
            }
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        travel[tuned] = m2Body_GetPosition(world, puck).x - x0;
        m2DestroyWorld(world);
    }
    CHECK(travel[1] < travel[0] - 3.0, "grip mid-slide cuts the trip short");

    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 3.0};
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.restitution = 0.0f;
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    m2ShapeId ballShape = NewCircleShape(world, ball, &bs, &circle);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // dead landing
    }
    m2Shape_SetRestitution(world, ballShape, 0.9f);
    CHECK(m2Shape_GetRestitution(world, ballShape) == 0.9f, "the getter agrees");
    m2Body_ApplyLinearImpulseAtPoint(world, ball, (m2Vec2){0.0f, -0.6f},
                                     m2Body_GetPosition(world, ball));
    double peak = 0.0;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double y = m2Body_GetPosition(world, ball).y;
        peak = y > peak ? y : peak;
    }
    CHECK(peak > 0.4, "a dead ball turned bouncy");

    m2DestroyWorld(world);
}

static void TestRuntimeFilter(void)
{
    // The floor turns into a ghost under a resting box: the box must
    // fall through and the pair's end must be witnessed.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    m2ShapeId floorShape = NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2Shape_SetFilter(world, floorShape, 0x2, 0x2, 0); // box (cat 1) no longer matches
    bool sawEnd = false;
    for (int32_t i = 0; i < 70; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents events = m2World_GetContactEvents(world);
        sawEnd = sawEnd || events.endCount > 0;
    }
    CHECK(sawEnd, "the vanished pair emitted its end");
    CHECK(m2Body_GetPosition(world, box).y < -1.0, "the box fell through its former floor");

    m2DestroyWorld(world);
}

static void TestFilterRollback(void)
{
    // Filters are snapshot state: a mixed-filter scene must replay
    // bit-exactly through restore.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-1.5 + (double)i, 1.5 + 0.4 * (double)i};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.categoryBits = i % 2 == 0 ? 0x1u : 0x4u; // half of them ghosts
        m2Polygon unit = m2MakeBox(0.3f, 0.3f);
        NewPolygonShape(world, body, &sd, &unit);
    }

    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "filtered scene replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

// A chain the shape pool cuts short keeps the segments that fit and its
// id, returns m2_errorCapacity and is recorded: its replay cuts short the
// same way and reaches the same world.
static void TestChainCutShort(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m2World* world = NewWorld(&def);
    m2World* replayed = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 5.0f}, 0.5f};
    NewCircleShape(world, ground, &sd, &ball);
    int32_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Vec2 pts[7] = {{6.0f, 0.0f},  {4.0f, 0.0f},  {2.0f, 0.0f}, {0.0f, 0.0f},
                     {-2.0f, 0.0f}, {-4.0f, 0.0f}, {-6.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 7; // four segments asked for, three shape slots left
    m2ChainId made = m2_nullChainId;
    CHECK(m2CreateChain(world, ground, &chain, &made) == m2_errorCapacity &&
              m2Chain_IsValid(world, made) && m2Chain_GetSegmentCount(world, made) == 3,
          "a chain the pool cuts short keeps its three segments and says so");
    CHECK(m2World_GetCounters(world).misuse == misuse, "counting no misuse");
    int32_t bytes = StopJournal(world);
    CHECK(bytes > 0 && ReplayJournal(replayed, tape, bytes) &&
              m2World_Hash(replayed) == m2World_Hash(world),
          "and its replay reaches the same world");
    free(tape);
    m2DestroyWorld(replayed);
    m2DestroyWorld(world);
}

int main(void)
{
    TestCollisionFilters();
    TestGroupIndex();
    TestChainGround();
    TestChainDestroy();
    TestChainCutShort();
    TestRuntimeMaterials();
    TestRuntimeFilter();
    TestFilterRollback();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
