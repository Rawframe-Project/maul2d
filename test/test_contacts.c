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
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4); // settle on the chain
    }
    CHECK(m2Body_GetPosition(world, box).y > 0.3, "the box rests on the chain floor");

    m2Body_ApplyLinearImpulseAtPoint(world, box, (m2Vec2){-4.0f, 0.0f},
                                     m2Body_GetPosition(world, box));
    float worstVy = 0.0f;
    for (int32_t i = 0; i < 240; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
        gotAbove = gotAbove || m2Body_GetPosition(world, riser).y > 1.0;
    }
    CHECK(gotAbove, "from below, the chain is a ghost");
    bool cameBackDown = false;
    for (int32_t i = 0; i < 300 && !cameBackDown; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
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

// A ball at rest beside a seam of a flat chain, on either side, touches
// only the segment under it. The next segment's end lies within reach,
// but its point there belongs to the segment the ball rests on: the chain
// lets go of it, so no contact pushes the ball along a slanted normal.
static void TestBallBesideASeamTouchesOneSegment(void)
{
    for (int32_t side = 0; side < 2; ++side)
    {
        m2WorldDef def = m2DefaultWorldDef();
        def.bodyCapacity = 4;
        def.shapeCapacity = 8;
        m2World* world = NewWorld(&def);
        m2BodyDef gd = m2DefaultBodyDef();
        m2BodyId ground = NewBody(world, &gd);
        m2Vec2 points[6] = {{10.0f, 0.0f}, {6.0f, 0.0f},  {2.0f, 0.0f},
                            {-2.0f, 0.0f}, {-6.0f, 0.0f}, {-10.0f, 0.0f}};
        m2ChainDef chainDef = m2DefaultChainDef();
        chainDef.points = points;
        chainDef.count = 6;
        NewChain(world, ground, &chainDef);
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){side == 0 ? 1.9 : 2.1, 0.25};
        m2BodyId ball = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle circle = {{0.0f, 0.0f}, 0.25f};
        NewCircleShape(world, ball, &sd, &circle);
        for (int32_t i = 0; i < 60; ++i)
        {
            Step(world, 1.0f / 60.0f, 4);
        }
        m2ContactData contacts[4];
        CHECK(m2World_GetContactData(world, contacts, 4) == 1 && contacts[0].normal.y > 0.999f,
              "a ball beside a seam touches one segment, straight up");
        m2DestroyWorld(world);
    }
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
            Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4); // dead landing
    }
    m2Shape_SetRestitution(world, ballShape, 0.9f);
    CHECK(m2Shape_GetRestitution(world, ballShape) == 0.9f, "the getter agrees");
    m2Body_ApplyLinearImpulseAtPoint(world, ball, (m2Vec2){0.0f, -0.6f},
                                     m2Body_GetPosition(world, ball));
    double peak = 0.0;
    for (int32_t i = 0; i < 90; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
    }

    m2Shape_SetFilter(world, floorShape, 0x2, 0x2, 0); // box (cat 1) no longer matches
    bool sawEnd = false;
    for (int32_t i = 0; i < 70; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
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
        Step(world, 1.0f / 60.0f, 4);
    }
    int64_t size = m2World_GetSnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(Restore(world, snap, size), "restore");
    for (int32_t i = 0; i < 60; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
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
    int64_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    m2Vec2 pts[7] = {{6.0f, 0.0f},  {4.0f, 0.0f},  {2.0f, 0.0f}, {0.0f, 0.0f},
                     {-2.0f, 0.0f}, {-4.0f, 0.0f}, {-6.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 7; // four segments asked for, three shape slots left
    m2ChainId made = m2_nullChainId;
    CHECK(m2CreateChain(world, ground, &chain, &made) == m2_errorCapacity &&
              m2Chain_IsValid(world, made) && m2Chain_GetSegmentCount(world, made) == 3,
          "a chain the pool cuts short keeps its three segments and says so");
    CHECK(m2World_GetCounters(world).misuseCount == misuse, "counting no misuse");
    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0 && ReplayJournal(replayed, tape, bytes) &&
              m2World_Hash(replayed) == m2World_Hash(world),
          "and its replay reaches the same world");
    free(tape);
    m2DestroyWorld(replayed);
    m2DestroyWorld(world);
}

// Drops a box onto a ground made of a lone segment or of a chain, with
// the box in the lower shape slot when `boxFirst` (a pair names its lower
// slot first), and returns where the box comes to rest.
static double BoxRestsOn(bool chainGround, bool boxFirst)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef groundDef = m2DefaultBodyDef();
    m2BodyDef boxDef = m2DefaultBodyDef();
    boxDef.type = m2_dynamicBody;
    boxDef.position = (m2Pos2){0.0, 1.5};
    m2Polygon block = m2MakeBox(0.5f, 0.5f);
    m2BodyId box = m2_nullBodyId;
    for (int32_t turn = 0; turn < 2; ++turn)
    {
        if ((turn == 0) == boxFirst)
        {
            box = NewBody(world, &boxDef);
            NewPolygonShape(world, box, &sd, &block);
            continue;
        }
        m2BodyId ground = NewBody(world, &groundDef);
        if (chainGround)
        {
            // Points run right to left, so the solid side faces up.
            m2Vec2 floor[4] = {{6.0f, 0.0f}, {3.0f, 0.0f}, {-3.0f, 0.0f}, {-6.0f, 0.0f}};
            m2ChainDef cd = m2DefaultChainDef();
            cd.points = floor;
            cd.count = 4;
            NewChain(world, ground, &cd);
        }
        else
        {
            m2Segment floor = {{-3.0f, 0.0f}, {3.0f, 0.0f}};
            NewSegmentShape(world, ground, &sd, &floor);
        }
    }
    for (int32_t i = 0; i < 120; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    double y = m2Body_GetPosition(world, box).y;
    m2DestroyWorld(world);
    return y;
}

// A box lands on a lone segment and on a chain link, whichever of the two
// holds the lower shape slot, and rests on top within the contact skin.
static void TestBoxRestsOnSegmentsInEitherSlot(void)
{
    const char* what[4] = {
        "a box above a segment's slot rests on it", "a box below a segment's slot rests on it",
        "a box above a chain's slots rests on it", "a box below a chain's slots rests on it"};
    for (int32_t k = 0; k < 4; ++k)
    {
        double y = BoxRestsOn(k >= 2, (k % 2) == 1);
        CHECK(y > 0.49 && y < 0.52, what[k]);
    }
}

// Two shapes without area never collide. A segment or a chain plank set
// down 2 mm above a segment or a chain floor makes no contact, not even on
// the first step, and falls through, where a box plank lands on either
// floor. A segment sensor still detects a segment.
static void TestShapesWithoutAreaNeverCollide(void)
{
    for (int32_t floorKind = 0; floorKind < 2; ++floorKind)
    {
        for (int32_t plankKind = 0; plankKind < 3; ++plankKind)
        {
            m2WorldDef def = m2DefaultWorldDef();
            def.bodyCapacity = 4;
            def.shapeCapacity = 8;
            m2World* world = NewWorld(&def);
            m2BodyDef gd = m2DefaultBodyDef();
            m2BodyId ground = NewBody(world, &gd);
            m2ShapeDef sd = m2DefaultShapeDef();
            m2Vec2 floorPoints[4] = {{12.0f, 0.0f}, {8.0f, 0.0f}, {-8.0f, 0.0f}, {-12.0f, 0.0f}};
            m2Segment floorLine = {{-8.0f, 0.0f}, {8.0f, 0.0f}};
            m2ChainDef chainDef = m2DefaultChainDef();
            chainDef.points = floorPoints;
            chainDef.count = 4;
            if (floorKind == 0)
            {
                NewSegmentShape(world, ground, &sd, &floorLine);
            }
            else
            {
                NewChain(world, ground, &chainDef);
            }
            m2BodyDef bd = m2DefaultBodyDef();
            bd.type = m2_dynamicBody;
            bd.position = (m2Pos2){0.0, plankKind == 2 ? 0.102 : 0.002};
            m2BodyId plank = NewBody(world, &bd);
            m2Segment plankLine = {{-1.0f, 0.0f}, {1.0f, 0.0f}};
            m2Vec2 plankPoints[4] = {{-2.0f, 0.0f}, {-1.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}};
            m2ChainDef plankChain = m2DefaultChainDef();
            plankChain.points = plankPoints;
            plankChain.count = 4;
            m2Polygon board = m2MakeBox(1.0f, 0.1f);
            if (plankKind == 0)
            {
                NewSegmentShape(world, plank, &sd, &plankLine);
            }
            else if (plankKind == 1)
            {
                NewChain(world, plank, &plankChain);
            }
            else
            {
                NewPolygonShape(world, plank, &sd, &board);
            }
            Step(world, 1.0f / 60.0f, 4);
            int32_t first = m2World_GetContactData(world, NULL, 0);
            for (int32_t i = 0; i < 60; ++i)
            {
                Step(world, 1.0f / 60.0f, 4);
            }
            double y = m2Body_GetPosition(world, plank).y;
            int32_t last = m2World_GetContactData(world, NULL, 0);
            if (plankKind == 2)
            {
                CHECK(first == 1 && last == 1 && y > 0.09 && y < 0.11,
                      "a box lands on a segment or a chain");
            }
            else
            {
                CHECK(first == 0 && last == 0 && y < -1.0,
                      "a segment or a chain never touches a segment or a chain");
            }
            m2DestroyWorld(world);
        }
    }

    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2Segment gate = {{0.0f, -1.0f}, {0.0f, 1.0f}};
    m2ShapeId sensor = NewSegmentShape(world, ground, &sensorDef, &gate);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId rod = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Segment across = {{-1.0f, 0.0f}, {1.0f, 0.0f}};
    NewSegmentShape(world, rod, &sd, &across);
    Step(world, 1.0f / 60.0f, 4);
    m2ShapeId visitors[2];
    CHECK(m2Shape_GetSensorOverlaps(world, sensor, visitors, 2) == 1,
          "a segment sensor detects a segment crossing it");
    m2DestroyWorld(world);
}

// With a null world, the three event getters return no events. A sensor
// beside a box it does not touch, close enough that their broadphase
// boxes overlap, lists no overlap. Contact data lists the box resting on
// the floor but not the sensor the box sits inside.
static void TestEventEdgesAndSensorPairs(void)
{
    m2ContactEvents contacts = m2World_GetContactEvents(NULL);
    m2SensorEvents sensors = m2World_GetSensorEvents(NULL);
    m2JointEvents joints = m2World_GetJointEvents(NULL);
    CHECK(contacts.beginCount == 0 && contacts.endCount == 0 && sensors.beginCount == 0 &&
              sensors.endCount == 0 && joints.breakCount == 0,
          "a null world has no contact, sensor or joint events");

    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &sd, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.5};
    m2BodyId box = NewBody(world, &bd);
    m2Polygon crate = m2MakeBox(0.5f, 0.5f);
    m2ShapeId crateShape = NewPolygonShape(world, box, &sd, &crate);

    // One sensor around the box, one 15 cm to its right: their boxes'
    // 0.1 m margins overlap the crate's, but the second never touches it.
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){0.0, 0.5};
    m2Polygon zone = m2MakeBox(1.0f, 1.0f);
    m2ShapeId zoneShape = NewPolygonShape(world, NewBody(world, &zd), &sensorDef, &zone);
    zd.position = (m2Pos2){0.75, 0.5};
    m2Polygon strip = m2MakeBox(0.1f, 0.3f);
    m2ShapeId beside = NewPolygonShape(world, NewBody(world, &zd), &sensorDef, &strip);
    for (int32_t i = 0; i < 10; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    m2ShapeId overlaps[4];
    CHECK(m2Shape_GetSensorOverlaps(world, beside, overlaps, 4) == 0,
          "a sensor beside a box it does not touch lists no overlap");
    m2ContactData data[8];
    int32_t count = m2World_GetContactData(world, data, 8);
    // The crate touches the floor and sits inside the zone; only the floor
    // contact is physical.
    bool solid = count == 1 && data[0].pointCount > 0 &&
                 (data[0].shapeIdA.index1 == crateShape.index1 ||
                  data[0].shapeIdB.index1 == crateShape.index1) &&
                 data[0].shapeIdA.index1 != zoneShape.index1 &&
                 data[0].shapeIdB.index1 != zoneShape.index1;
    CHECK(solid, "contact data lists the crate on the floor and not the sensor around it");
    m2DestroyWorld(world);
}

// What the step after a static wall moves into a sleeping box reports.
typedef struct WallContact
{
    m2ContactData data; // the box-wall contact, zeros without one
    int32_t begins;     // begin events naming the wall
    int32_t touching;   // touching pairs, from the counters
    bool woke;          // the box wakes when the wall moves again
} WallContact;

// A unit box resting on the ground at the origin, its shape made with its
// body.
static m2BodyId SleeperBox(m2World* world)
{
    m2BodyDef def = m2DefaultBodyDef();
    def.type = m2_dynamicBody;
    def.position = (m2Pos2){0.0, 1.0};
    m2BodyId box = NewBody(world, &def);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon square = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, box, &shapeDef, &square);
    return box;
}

// A box sleeps on the ground, its right edge at x 0.5. A static wall box
// starts beside it at x 1.15, close enough to pair but not to touch, or
// far away at x 5. One host call then makes it overlap the box by 0.1 m:
// a teleport to x 0.9, or, at x 1.15, a polygon of half width 0.75. The
// wall is made before the box or after it, so it takes either end of the
// pair.
static WallContact WallIntoSleeper(bool beside, bool widen, bool wallFirst)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, NewBody(world, &bodyDef), &shapeDef, &floor);
    m2BodyId box = wallFirst ? m2_nullBodyId : SleeperBox(world);
    bodyDef.position = (m2Pos2){beside ? 1.15 : 5.0, 1.0};
    m2BodyId wall = NewBody(world, &bodyDef);
    m2Polygon square = m2MakeBox(0.5f, 0.5f);
    m2ShapeId wallShape = NewPolygonShape(world, wall, &shapeDef, &square);
    if (wallFirst)
    {
        box = SleeperBox(world);
    }
    for (int32_t i = 0; i < 200; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, box), "the box falls asleep beside the wall");

    m2Rot upright = {1.0f, 0.0f};
    if (widen)
    {
        m2Polygon wide = m2MakeBox(0.75f, 0.5f);
        m2Shape_SetPolygon(world, wallShape, &wide);
        if (!beside)
        {
            m2Body_SetTransform(world, wall, (m2Pos2){1.15, 1.0}, upright);
        }
    }
    else
    {
        m2Body_SetTransform(world, wall, (m2Pos2){0.9, 1.0}, upright);
    }
    Step(world, 1.0f / 60.0f, 4);

    WallContact report;
    memset(&report, 0, sizeof(report));
    m2ContactData data[4];
    int32_t count = m2World_GetContactData(world, data, 4);
    for (int32_t i = 0; i < count && i < 4; ++i)
    {
        if (data[i].shapeIdA.index1 == wallShape.index1 ||
            data[i].shapeIdB.index1 == wallShape.index1)
        {
            report.data = data[i];
        }
    }
    m2ContactEvents events = m2World_GetContactEvents(world);
    for (int32_t i = 0; i < events.beginCount; ++i)
    {
        report.begins += events.beginEvents[i].shapeIdA.index1 == wallShape.index1 ||
                         events.beginEvents[i].shapeIdB.index1 == wallShape.index1;
    }
    report.touching = m2World_GetCounters(world).touchingCount;
    m2Body_SetTransform(world, wall, m2Body_GetPosition(world, wall), upright);
    report.woke = m2Body_IsAwake(world, box);
    m2DestroyWorld(world);
    return report;
}

static void ExpectWallTouches(bool widen, bool wallFirst, const char* fromAfar,
                              const char* fromBeside)
{
    WallContact far = WallIntoSleeper(false, widen, wallFirst);
    WallContact near = WallIntoSleeper(true, widen, wallFirst);
    CHECK(far.data.pointCount == 2 && far.begins == 1 && far.touching == 2 && far.woke, fromAfar);
    CHECK(memcmp(&near.data, &far.data, sizeof(near.data)) == 0 && near.begins == 1 &&
              near.touching == 2 && near.woke,
          fromBeside);
}

// The step copies a pair's manifold forward while both its bodies are
// static or asleep. A host moves a static shape without waking a sleeper
// it does not touch, so a pair the two already held gets a fresh manifold,
// the one a pair new to the step gets: the same points, the touching flag
// and the begin event, and a later move of the wall wakes the box.
static void TestStaticShapeMovedIntoASleeper(void)
{
    ExpectWallTouches(false, true, "a wall teleported in from afar touches the sleeping box",
                      "a wall teleported from beside it touches it the same way");
    ExpectWallTouches(false, false, "a wall made after the box, teleported in, touches it",
                      "a wall made after the box, teleported from beside it, touches it");
    ExpectWallTouches(true, true, "a wall from afar, widened into the sleeping box, touches it",
                      "a wall widened where it stands touches it the same way");
    ExpectWallTouches(true, false, "a wall made after the box, widened from afar, touches it",
                      "a wall made after the box, widened where it stands, touches it");
}

int main(void)
{
    TestCollisionFilters();
    TestEventEdgesAndSensorPairs();
    TestGroupIndex();
    TestChainGround();
    TestBallBesideASeamTouchesOneSegment();
    TestChainDestroy();
    TestChainCutShort();
    TestRuntimeMaterials();
    TestRuntimeFilter();
    TestFilterRollback();
    TestBoxRestsOnSegmentsInEitherSlot();
    TestShapesWithoutAreaNeverCollide();
    TestStaticShapeMovedIntoASleeper();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
