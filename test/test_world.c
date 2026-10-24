// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// World suite: the rollback identity (hash-sequence equality plus
// snapshot -> restore -> re-snapshot byte-compare), id semantics
// (generation staleness, FIFO reuse, replay re-minting, a stale world
// id), refusals and out-of-memory creation, counters and validation,
// and the world-level behavior of bodies: teleports, type changes,
// gravity, wind, explosions, sleep, mass, shatter, dominance, motion
// locks and fluid volumes.

#include "helpers.h"
#include "test_harness.h"
#include "test_task_pool.h"

#include "maul2d/maul2d.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2World* MakeTestWorld(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 256;
    return NewWorld(&def);
}

static void SpawnScene(m2World* world)
{
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -1.0};
    NewBody(world, &groundDef);

    for (int32_t i = 0; i < 50; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){1.0e6 + 0.1 * (double)(i % 10), 5.0 + 1.1 * (double)(i / 10)};
        bd.rotation = m2MakeRot(0.05f * (float)i);
        bd.linearVelocity = (m2Vec2){0.25f * (float)(i % 5) - 0.5f, 0.0f};
        bd.angularVelocity = 0.3f * (float)(i % 3) - 0.3f;
        bd.userData = (uint64_t)i;
        NewBody(world, &bd);
    }
}

static void TestRollbackIdentity(void)
{
    m2World* world = MakeTestWorld();
    CHECK(world != NULL, "world creation");
    SpawnScene(world);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snapA = malloc((size_t)size);
    void* snapB = malloc((size_t)size);
    CHECK(Snapshot(world, snapA, size) == size, "snapshot A");

    // Immediate restore + re-snapshot must be byte-identical (oracle 2).
    CHECK(Restore(world, snapA, size), "restore A");
    CHECK(Snapshot(world, snapB, size) == size, "snapshot B");
    CHECK(memcmp(snapA, snapB, (size_t)size) == 0, "restore->re-snapshot byte-compare");

    // Trajectory identity: continue, record hashes; restore, re-run, compare.
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(Restore(world, snapA, size), "restore for replay");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "rollback identity: hash sequence must match");
    }

    // Post-restore create/destroy determinism:
    // the same commands after restore must mint bit-identical ids.
    CHECK(Restore(world, snapA, size), "restore for id re-mint");
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId mintedA = NewBody(world, &bd);
    CHECK(Restore(world, snapA, size), "restore again");
    m2BodyId mintedB = NewBody(world, &bd);
    CHECK(mintedA.index1 == mintedB.index1 && mintedA.generation == mintedB.generation,
          "replay re-mints identical ids");

    free(snapA);
    free(snapB);
    m2DestroyWorld(world);
}

// Destroying the null world does nothing.
static void TestDestroyNullWorld(void)
{
    m2DestroyWorld(NULL);
    CHECK(true, "destroying the null world returns");
}

static void TestIdSemantics(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId a = NewBody(world, &bd);
    m2BodyId b = NewBody(world, &bd);
    CHECK(m2Body_IsValid(world, a) && m2Body_IsValid(world, b), "created ids valid");

    DestroyBody(world, a);
    CHECK(!m2Body_IsValid(world, a), "destroyed id is stale");

    // FIFO reuse: the freed slot goes to the queue tail, so the next two
    // creates use fresh slots before slot 0 comes around again.
    m2BodyId c = NewBody(world, &bd);
    m2BodyId d = NewBody(world, &bd);
    CHECK(c.index1 != a.index1 && d.index1 != a.index1, "FIFO reuse distance");
    m2BodyId e = NewBody(world, &bd);
    CHECK(e.index1 == a.index1 && e.generation != a.generation,
          "recycled slot carries a bumped generation");
    CHECK(!m2Body_IsValid(world, a), "stale id stays stale after slot reuse");
    CHECK(NewBody(world, &bd).index1 == 0, "capacity exhaustion returns the null id");

    m2World_Step(world, 1.0f / 60.0f, 1); // step with dead slots present
    m2DestroyWorld(world);
}

static void TestCountersAndProfile(void)
{
    // Counters are state-derived and deterministic; neither they nor the
    // wall-clock profile may touch the simulation hash.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.5 + (double)i, 0.55};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.45f, 0.45f);
        NewPolygonShape(world, body, &sd, &box);
    }

    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2Counters counters = m2World_GetCounters(world);
    CHECK(counters.bodies == 7 && counters.shapes == 7, "body and shape counts");
    CHECK(counters.awakeBodies == 7, "everyone still awake early on");
    CHECK(counters.joints == 0, "no joints in this scene");
    CHECK(counters.touchingPairs == 6, "six boxes rest on the slab");
    CHECK(counters.pairs >= counters.touchingPairs,
          "speculative pairs may outnumber touching ones");
    CHECK(counters.constraints == 6, "one constraint per touching pair");
    CHECK(counters.graphColors >= 1 && counters.overflowConstraints == 0,
          "colored without overflow");
    CHECK(counters.stepCount == 10, "step count mirrors the world");

    // Diagnostics never touch the simulation: hash before == after.
    uint64_t before = m2World_Hash(world);
    for (int32_t i = 0; i < 25; ++i)
    {
        m2World_GetCounters(world);
        m2World_GetProfile(world);
    }
    CHECK(m2World_Hash(world) == before, "diagnostics are read-only");

    // Twin world, same ops: counters identical (determinism).
    m2World* twin = NewWorld(&def);
    m2BodyId floor2 = NewBody(twin, &fd);
    NewPolygonShape(twin, floor2, &fs, &slab);
    for (int32_t i = 0; i < 6; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.5 + (double)i, 0.55};
        m2BodyId body = NewBody(twin, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.45f, 0.45f);
        NewPolygonShape(twin, body, &sd, &box);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(twin, 1.0f / 60.0f, 4);
    }
    m2Counters twinCounters = m2World_GetCounters(twin);
    CHECK(memcmp(&counters, &twinCounters, sizeof(m2Counters)) == 0,
          "twin worlds report identical counters");

    m2DestroyWorld(twin);
    m2DestroyWorld(world);
}

static void TestKineticEnergy(void)
{
    // Analytic: one ball, known mass, known velocity. Then the energy
    // must be twin-identical and vanish with sleep.
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, 0.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.0};
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    NewCircleShape(world, ball, &sd, &circle);
    m2Body_SetLinearVelocity(world, ball, (m2Vec2){3.0f, 4.0f}); // speed 5

    double mass = 3.14159265358979 * 0.25; // rho=1, r=0.5
    double expected = 0.5 * mass * 25.0;
    double energy = m2World_GetKineticEnergy(world);
    double err = energy - expected;
    CHECK(err > -1.0e-4 && err < 1.0e-4, "kinetic energy matches the hand computation");

    m2DestroyWorld(world);

    // A settling stack: energy decays to exactly zero once asleep,
    // and twin worlds agree to the bit the whole way down.
    m2WorldDef def2 = m2DefaultWorldDef();
    def2.bodyCapacity = 16;
    def2.shapeCapacity = 16;
    m2World* a = NewWorld(&def2);
    m2World* b = NewWorld(&def2);
    for (int32_t twin = 0; twin < 2; ++twin)
    {
        m2World* w = twin == 0 ? a : b;
        m2BodyDef fd = m2DefaultBodyDef();
        fd.position = (m2Pos2){0.0, -0.5};
        m2BodyId floor = NewBody(w, &fd);
        m2ShapeDef fs = m2DefaultShapeDef();
        m2Polygon slab = m2MakeBox(10.0f, 0.5f);
        NewPolygonShape(w, floor, &fs, &slab);
        for (int32_t i = 0; i < 4; ++i)
        {
            m2BodyDef box = m2DefaultBodyDef();
            box.type = m2_dynamicBody;
            box.position = (m2Pos2){0.0, 0.7 + 1.05 * (double)i};
            m2BodyId body = NewBody(w, &box);
            m2ShapeDef bs = m2DefaultShapeDef();
            m2Polygon unit = m2MakeBox(0.45f, 0.45f);
            NewPolygonShape(w, body, &bs, &unit);
        }
    }
    bool everAwake = false;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(a, 1.0f / 60.0f, 4);
        m2World_Step(b, 1.0f / 60.0f, 4);
        double ea = m2World_GetKineticEnergy(a);
        double eb = m2World_GetKineticEnergy(b);
        CHECK(ea == eb, "twin worlds agree on energy to the bit");
        everAwake = everAwake || ea > 0.0;
    }
    CHECK(everAwake, "the stack had energy while falling");
    CHECK(m2World_GetKineticEnergy(a) == 0.0, "sleep zeroes the ledger");
    m2DestroyWorld(a);
    m2DestroyWorld(b);
}

static void TestTeleport(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    bd.linearVelocity = (m2Vec2){1.5f, 0.0f};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, 0.4f};
    NewCircleShape(world, body, &sd, &c);

    m2Body_SetTransform(world, body, (m2Pos2){100.0, -3.0}, m2MakeRot(0.5f));
    m2Pos2 p = m2Body_GetPosition(world, body);
    CHECK(p.x == 100.0 && p.y == -3.0, "teleport moves the origin exactly");
    m2Vec2 v = m2Body_GetLinearVelocity(world, body);
    CHECK(v.x == 1.5f && v.y == 0.0f, "teleport leaves velocity untouched");

    // Ray finds it at the new home immediately (broadphase refreshed).
    m2RayCastResult hit = m2World_CastRayClosest(world, (m2Pos2){100.0, 2.0}, (m2Vec2){0.0f, -6.0f},
                                                 m2DefaultQueryFilter());
    CHECK(hit.hit, "queries see the teleported body at once");

    CHECK(m2World_GetJournalBaseSize(world) == 32 + m2World_SnapshotSize(world),
          "journal base size is header plus snapshot");

    m2DestroyWorld(world);
}

static void TestSetType(void)
{
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

    // A falling box freezes mid-air when it turns static...
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId shelf = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.1f);
    NewPolygonShape(world, shelf, &sd, &unit);
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Body_SetType(world, shelf, m2_staticBody);
    double frozenY = m2Body_GetPosition(world, shelf).y;
    m2Vec2 v = m2Body_GetLinearVelocity(world, shelf);
    CHECK(v.x == 0.0f && v.y == 0.0f, "becoming static zeroes velocity");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, shelf).y == frozenY, "a static shelf hangs mid-air");

    // ...carries a dynamic guest on its migrated proxy...
    m2BodyDef gd = m2DefaultBodyDef();
    gd.type = m2_dynamicBody;
    gd.position = (m2Pos2){0.0, frozenY + 0.5};
    m2BodyId guest = NewBody(world, &gd);
    m2Polygon g = m2MakeBox(0.3f, 0.3f);
    NewPolygonShape(world, guest, &sd, &g);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, guest).y > frozenY + 0.3, "a guest rests on the frozen shelf");

    // ...and drops like anyone else once it is dynamic again.
    m2Body_SetType(world, shelf, m2_dynamicBody);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, shelf).y < frozenY - 0.5, "dynamic again, it falls");
    CHECK(m2Body_GetPosition(world, shelf).y > 0.0, "and lands on the real floor");

    m2DestroyWorld(world);
}

static void TestRuntimeGravity(void)
{
    // Flip gravity over a sleeping box: it must wake and rise, not
    // float against the new gravity.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, box), "asleep under the old sky");

    m2World_SetGravity(world, (m2Vec2){0.0f, 10.0f});
    m2Vec2 g = m2World_GetGravity(world);
    CHECK(g.y == 10.0f, "the getter reads the new sky");
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, box), "the sleeper feels the flip");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, box).y > 3.0, "and falls upward");

    m2DestroyWorld(world);
}

// Destroying a body must wake whoever was resting on it, the same
// rule teleports and type changes already obey; otherwise sleepers
// would float on nothing.
static void TestDestroyWakesSleepers(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.45};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, box, &sd, &unit);
    for (int32_t i = 0; i < 250; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, box), "box sleeps on the floor");

    DestroyBody(world, floor);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, box), "yanking the floor wakes the sleeper");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, box).y < -5.0, "and it falls");
    m2DestroyWorld(world);
}

// The integrator walk: an editor or engine layer must be able to
// enumerate every live object from a bare world id, read types and
// filters back, and see destroys reflected. Ascending slot order,
// truthful totals.
static void TestEnumerationWalk(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Polygon slabPoly = m2MakeBox(8.0f, 0.5f);
    m2ShapeId slab = NewPolygonShape(world, ground, &gs, &slabPoly);
    m2Vec2 pts[6] = {{6.0f, 2.0f}, {4.0f, 2.0f},  {2.0f, 2.0f},
                     {0.0f, 2.0f}, {-2.0f, 2.0f}, {-4.0f, 2.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId ledge = NewChain(world, ground, &chain);

    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 6.0};
    m2BodyId hook = NewBody(world, &hd);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId boxA = NewBody(world, &ad);
    m2ShapeDef asd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, boxA, &asd, &unit);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){3.0, 0.5};
    m2BodyId boxB = NewBody(world, &bd);
    NewPolygonShape(world, boxB, &asd, &unit);
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2Circle aura = {{0.0f, 0.0f}, 1.5f};
    m2ShapeId auraShape = NewCircleShape(world, boxB, &sensorDef, &aura);

    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = boxA;
    m2JointId rope = NewDistanceJoint(world, &dj);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = boxB;
    m2JointId pin = NewRevoluteJoint(world, &rj);

    // Totals are truthful regardless of capacity.
    CHECK(m2World_GetBodies(world, NULL, 0) == 4, "four live bodies");
    m2BodyId two[2];
    CHECK(m2World_GetBodies(world, two, 2) == 4, "small capacity still reports four");
    m2BodyId bodies[8];
    int32_t n = m2World_GetBodies(world, bodies, 8);
    CHECK(n == 4, "the full walk finds four");
    bool ascending = true;
    for (int32_t i = 1; i < n; ++i)
    {
        ascending = ascending && bodies[i - 1].index1 < bodies[i].index1;
    }
    CHECK(ascending, "bodies arrive in ascending slot order");
    CHECK(m2World_GetJoints(world, NULL, 0) == 2, "two joints");
    CHECK(m2World_GetChains(world, NULL, 0) == 1, "one chain");

    // Per-body shape walks.
    CHECK(m2Body_GetShapes(world, ground, NULL, 0) == 4, "slab plus three chain segments");
    m2ShapeId groundShapes[8];
    int32_t gn = m2Body_GetShapes(world, ground, groundShapes, 8);
    int32_t segments = 0;
    for (int32_t i = 0; i < gn; ++i)
    {
        segments += m2Shape_GetType(world, groundShapes[i]) == m2_chainSegmentShape ? 1 : 0;
    }
    CHECK(segments == 3, "and the walk can tell which are chain links");
    CHECK(m2Body_GetShapes(world, boxB, NULL, 0) == 2, "box B carries body and aura");

    // Types and flags read back.
    CHECK(m2Body_GetType(world, ground) == m2_staticBody, "ground reads static");
    CHECK(m2Body_GetType(world, boxA) == m2_dynamicBody, "box A reads dynamic");
    m2Body_SetType(world, boxA, m2_kinematicBody);
    CHECK(m2Body_GetType(world, boxA) == m2_kinematicBody, "and follows SetType");
    m2Body_SetType(world, boxA, m2_dynamicBody);
    CHECK(m2Shape_GetType(world, slab) == m2_polygonShape, "slab reads polygon");
    CHECK(m2Shape_GetType(world, auraShape) == m2_circleShape, "aura reads circle");
    CHECK(m2Shape_IsSensor(world, auraShape), "aura reads sensor");
    CHECK(!m2Shape_IsSensor(world, slab), "slab does not");

    // Filter round-trip, including NULL out-params.
    m2Shape_SetFilter(world, slab, 0x4u, 0xF0u, -3);
    uint64_t category = 0;
    uint64_t mask = 0;
    int32_t group = 0;
    m2Shape_GetFilter(world, slab, &category, &mask, &group);
    CHECK(category == 0x4u && mask == 0xF0u && group == -3, "filter reads back exactly");
    m2Shape_GetFilter(world, slab, NULL, NULL, &group);
    CHECK(group == -3, "partial reads are fine");

    // Joint introspection.
    CHECK(m2Joint_GetType(world, rope) == m2_distanceJoint, "rope reads distance");
    CHECK(m2Joint_GetType(world, pin) == m2_revoluteJoint, "pin reads revolute");
    m2BodyId ja = m2Joint_GetBodyA(world, rope);
    m2BodyId jb = m2Joint_GetBodyB(world, rope);
    CHECK(ja.index1 == hook.index1 && ja.generation == hook.generation, "rope body A is the hook");
    CHECK(jb.index1 == boxA.index1 && jb.generation == boxA.generation, "rope body B is box A");

    // Destroys are reflected immediately.
    DestroyBody(world, boxA);
    CHECK(m2World_GetBodies(world, NULL, 0) == 3, "three bodies after the destroy");
    CHECK(m2World_GetJoints(world, NULL, 0) == 1, "the rope died with its body");
    DestroyChain(world, ledge);
    CHECK(m2World_GetChains(world, NULL, 0) == 0, "no chains after demolition");
    CHECK(m2Body_GetShapes(world, ground, NULL, 0) == 1, "the slab is all that remains");

    m2DestroyWorld(world);
}

// THE MIRROR TEST: rebuild a world using nothing but the public
// readback surface, then demand hash equality at creation and through
// 90 steps. Any parameter the getters cannot carry breaks the mirror,
// so this test IS the completeness proof of the integrator surface.
// Bit-identity requires replaying the source's creation order, which
// the ascending walks reproduce for destroy-free worlds.

static m2BodyId MirrorOf(const m2BodyId* src, const m2BodyId* dst, int32_t n, m2BodyId find)
{
    for (int32_t i = 0; i < n; ++i)
    {
        if (src[i].index1 == find.index1 && src[i].generation == find.generation)
        {
            return dst[i];
        }
    }
    m2BodyId null = {0, 0};
    return null;
}

static m2World* MirrorWorld(m2World* source, const m2WorldDef* def)
{
    m2World* mirror = NewWorld(def);
    m2BodyId bodies[16];
    m2BodyId mirrored[16];
    int32_t bodyCount = m2World_GetBodies(source, bodies, 16);
    m2ChainId chains[8];
    int32_t chainCount = m2World_GetChains(source, chains, 8);

    for (int32_t i = 0; i < bodyCount; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2Body_GetType(source, bodies[i]);
        m2Transform tf = m2Body_GetTransform(source, bodies[i]);
        bd.position = tf.p;
        bd.rotation = tf.q;
        bd.linearVelocity = m2Body_GetLinearVelocity(source, bodies[i]);
        bd.angularVelocity = m2Body_GetAngularVelocity(source, bodies[i]);
        bd.gravityScale = m2Body_GetGravityScale(source, bodies[i]);
        bd.isBullet = m2Body_IsBullet(source, bodies[i]);
        bd.userData = m2Body_GetUserData(source, bodies[i]);
        bd.dominance = m2Body_GetDominance(source, bodies[i]);
        mirrored[i] = NewBody(mirror, &bd);

        m2ShapeId shapes[16];
        int32_t shapeCount = m2Body_GetShapes(source, bodies[i], shapes, 16);
        for (int32_t s = 0; s < shapeCount; ++s)
        {
            m2ShapeType type = m2Shape_GetType(source, shapes[s]);
            if (type == m2_chainSegmentShape)
            {
                continue; // rebuilt with its chain below
            }
            m2ShapeDef sd = m2DefaultShapeDef();
            sd.density = m2Shape_GetDensity(source, shapes[s]);
            sd.friction = m2Shape_GetFriction(source, shapes[s]);
            sd.restitution = m2Shape_GetRestitution(source, shapes[s]);
            sd.tangentSpeed = m2Shape_GetTangentSpeed(source, shapes[s]);
            sd.isSensor = m2Shape_IsSensor(source, shapes[s]);
            m2Shape_GetFilter(source, shapes[s], &sd.categoryBits, &sd.maskBits, &sd.groupIndex);
            sd.userData = m2Shape_GetUserData(source, shapes[s]);
            if (type == m2_circleShape)
            {
                m2Circle g = m2Shape_GetCircle(source, shapes[s]);
                NewCircleShape(mirror, mirrored[i], &sd, &g);
            }
            else if (type == m2_capsuleShape)
            {
                m2Capsule g = m2Shape_GetCapsule(source, shapes[s]);
                NewCapsuleShape(mirror, mirrored[i], &sd, &g);
            }
            else if (type == m2_polygonShape)
            {
                m2Polygon g = m2Shape_GetPolygon(source, shapes[s]);
                NewPolygonShape(mirror, mirrored[i], &sd, &g);
            }
            else
            {
                m2Segment g = m2Shape_GetSegment(source, shapes[s]);
                NewSegmentShape(mirror, mirrored[i], &sd, &g);
            }
        }

        for (int32_t c = 0; c < chainCount; ++c)
        {
            m2ShapeId links[8];
            int32_t linkCount = m2Chain_GetShapes(source, chains[c], links, 8);
            if (linkCount == 0)
            {
                continue;
            }
            m2BodyId owner = m2Shape_GetBody(source, links[0]);
            if (owner.index1 != bodies[i].index1)
            {
                continue;
            }
            // Open-chain polyline from the links: leading ghost, each
            // link's first point, trailing point, trailing ghost.
            m2Vec2 pts[16];
            m2ChainSegment firstLink = m2Shape_GetChainSegment(source, links[0]);
            m2ChainSegment lastLink = m2Shape_GetChainSegment(source, links[linkCount - 1]);
            pts[0] = firstLink.ghost1;
            for (int32_t k = 0; k < linkCount; ++k)
            {
                pts[k + 1] = m2Shape_GetChainSegment(source, links[k]).segment.point1;
            }
            pts[linkCount + 1] = lastLink.segment.point2;
            pts[linkCount + 2] = lastLink.ghost2;
            m2ChainDef cd = m2DefaultChainDef();
            cd.points = pts;
            cd.count = linkCount + 3;
            cd.friction = m2Shape_GetFriction(source, links[0]);
            cd.restitution = m2Shape_GetRestitution(source, links[0]);
            m2Shape_GetFilter(source, links[0], &cd.categoryBits, &cd.maskBits, &cd.groupIndex);
            cd.userData = m2Shape_GetUserData(source, links[0]);
            NewChain(mirror, mirrored[i], &cd);
        }
    }

    m2JointId joints[8];
    int32_t jointCount = m2World_GetJoints(source, joints, 8);
    for (int32_t j = 0; j < jointCount; ++j)
    {
        m2JointId id = joints[j];
        m2BodyId a = MirrorOf(bodies, mirrored, bodyCount, m2Joint_GetBodyA(source, id));
        m2BodyId b = MirrorOf(bodies, mirrored, bodyCount, m2Joint_GetBodyB(source, id));
        m2JointId made = {0, 0};
        m2JointType type = m2Joint_GetType(source, id);
        if (type == m2_filterJoint)
        {
            m2FilterJointDef jd = m2DefaultFilterJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            made = NewFilterJoint(mirror, &jd);
        }
        else if (type == m2_ratchetJoint)
        {
            m2RatchetJointDef jd = m2DefaultRatchetJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.ratchet = m2RatchetJoint_GetRatchet(source, id);
            jd.phase = m2RatchetJoint_GetPhase(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewRatchetJoint(mirror, &jd);
        }
        else if (type == m2_pulleyJoint)
        {
            m2PulleyJointDef jd = m2DefaultPulleyJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.groundAnchorA = m2PulleyJoint_GetGroundAnchorA(source, id);
            jd.groundAnchorB = m2PulleyJoint_GetGroundAnchorB(source, id);
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.ratio = m2PulleyJoint_GetRatio(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewPulleyJoint(mirror, &jd);
        }
        else if (type == m2_gearJoint)
        {
            m2GearJointDef jd = m2DefaultGearJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.ratio = m2GearJoint_GetRatio(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewGearJoint(mirror, &jd);
        }
        else if (type == m2_motorJoint)
        {
            m2MotorJointDef jd = m2DefaultMotorJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.linearOffset = m2MotorJoint_GetLinearOffset(source, id);
            jd.angularOffset = m2MotorJoint_GetAngularOffset(source, id);
            jd.maxForce = m2MotorJoint_GetMaxForce(source, id);
            jd.maxTorque = m2Joint_GetMaxMotor(source, id);
            jd.correctionFactor = m2MotorJoint_GetCorrectionFactor(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewMotorJoint(mirror, &jd);
        }
        else if (type == m2_mouseJoint)
        {
            m2MouseJointDef jd = m2DefaultMouseJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.target = m2MouseJoint_GetTarget(source, id);
            jd.hertz = m2Joint_GetHertz(source, id);
            jd.dampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.maxForce = m2MouseJoint_GetMaxForce(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewMouseJoint(mirror, &jd);
        }
        else if (type == m2_distanceJoint)
        {
            m2DistanceJointDef jd = m2DefaultDistanceJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.length = m2Joint_GetLength(source, id);
            m2Joint_GetLimits(source, id, &jd.minLength, &jd.maxLength);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            jd.hertz = m2Joint_GetHertz(source, id);
            jd.dampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.enableSpring = m2DistanceJoint_IsSpringEnabled(source, id);
            made = NewDistanceJoint(mirror, &jd);
        }
        else if (type == m2_revoluteJoint)
        {
            m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.hertz = m2Joint_GetHertz(source, id);
            jd.dampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.springHertz = m2Joint_GetAngularHertz(source, id);
            jd.springDampingRatio = m2Joint_GetAngularDampingRatio(source, id);
            jd.enableMotor = m2Joint_IsMotorEnabled(source, id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(source, id);
            jd.maxMotorTorque = m2Joint_GetMaxMotor(source, id);
            jd.enableLimit = m2Joint_IsLimitEnabled(source, id);
            m2Joint_GetLimits(source, id, &jd.lowerAngle, &jd.upperAngle);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewRevoluteJoint(mirror, &jd);
        }
        else if (type == m2_prismaticJoint)
        {
            m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.localAxisA = m2Joint_GetLocalAxisA(source, id);
            jd.hertz = m2Joint_GetHertz(source, id);
            jd.dampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.enableMotor = m2Joint_IsMotorEnabled(source, id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(source, id);
            jd.maxMotorForce = m2Joint_GetMaxMotor(source, id);
            jd.enableLimit = m2Joint_IsLimitEnabled(source, id);
            m2Joint_GetLimits(source, id, &jd.lowerTranslation, &jd.upperTranslation);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewPrismaticJoint(mirror, &jd);
        }
        else if (type == m2_weldJoint)
        {
            m2WeldJointDef jd = m2DefaultWeldJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.linearHertz = m2Joint_GetHertz(source, id);
            jd.linearDampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.angularHertz = m2Joint_GetAngularHertz(source, id);
            jd.angularDampingRatio = m2Joint_GetAngularDampingRatio(source, id);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewWeldJoint(mirror, &jd);
        }
        else
        {
            m2WheelJointDef jd = m2DefaultWheelJointDef();
            jd.bodyIdA = a;
            jd.bodyIdB = b;
            jd.localAnchorA = m2Joint_GetLocalAnchorA(source, id);
            jd.localAnchorB = m2Joint_GetLocalAnchorB(source, id);
            jd.localAxisA = m2Joint_GetLocalAxisA(source, id);
            jd.enableSpring = m2Joint_IsSpringEnabled(source, id);
            jd.hertz = m2Joint_GetHertz(source, id);
            jd.dampingRatio = m2Joint_GetDampingRatio(source, id);
            jd.enableMotor = m2Joint_IsMotorEnabled(source, id);
            jd.motorSpeed = m2Joint_GetMotorSpeed(source, id);
            jd.maxMotorTorque = m2Joint_GetMaxMotor(source, id);
            jd.enableLimit = m2Joint_IsLimitEnabled(source, id);
            m2Joint_GetLimits(source, id, &jd.lowerTranslation, &jd.upperTranslation);
            jd.collideConnected = m2Joint_GetCollideConnected(source, id);
            made = NewWheelJoint(mirror, &jd);
        }
        float breakForce = 0.0f;
        float breakTorque = 0.0f;
        m2Joint_GetBreakLimits(source, id, &breakForce, &breakTorque);
        if (breakForce != 0.0f || breakTorque != 0.0f)
        {
            m2Joint_SetBreakLimits(mirror, made, breakForce, breakTorque);
        }
    }
    return mirror;
}

static void TestMirrorRebuild(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    // One of everything, with deliberately non-default numbers, built
    // body by body so the mirror's walk replays the creation order.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -1.0};
    gd.userData = 11;
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef gs = m2DefaultShapeDef();
    gs.friction = 0.8f;
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, ground, &gs, &slab);
    m2Vec2 pts[6] = {{6.0f, 1.0f}, {4.0f, 1.0f},  {2.0f, 1.0f},
                     {0.0f, 1.0f}, {-2.0f, 1.0f}, {-4.0f, 1.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    chain.friction = 0.9f;
    chain.restitution = 0.1f;
    chain.categoryBits = 0x2u;
    chain.maskBits = 0xFFu;
    chain.groupIndex = 5;
    chain.userData = 77;
    NewChain(world, ground, &chain);

    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 8.0};
    m2BodyId hook = NewBody(world, &hd);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){0.0, 5.0};
    ad.linearVelocity = (m2Vec2){1.5f, 2.0f};
    ad.angularVelocity = 3.0f;
    ad.gravityScale = 0.7f;
    ad.isBullet = true;
    ad.userData = 21;
    ad.dominance = 2;
    m2BodyId boxA = NewBody(world, &ad);
    m2ShapeDef asd = m2DefaultShapeDef();
    asd.density = 2.0f;
    asd.friction = 0.4f;
    asd.restitution = 0.3f;
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, boxA, &asd, &unit);

    m2BodyDef bd2 = m2DefaultBodyDef();
    bd2.type = m2_dynamicBody;
    bd2.position = (m2Pos2){3.0, 2.0};
    bd2.rotation = (m2Rot){0.9553365f, 0.29552022f}; // cos/sin of 0.3, pinned
    m2BodyId boxB = NewBody(world, &bd2);
    m2Capsule pill = {{-0.4f, 0.0f}, {0.4f, 0.0f}, 0.25f};
    NewCapsuleShape(world, boxB, &asd, &pill);
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    sensorDef.userData = 42;
    m2Circle aura = {{0.0f, 0.0f}, 1.2f};
    NewCircleShape(world, boxB, &sensorDef, &aura);

    m2BodyDef cd2 = m2DefaultBodyDef();
    cd2.type = m2_dynamicBody;
    cd2.position = (m2Pos2){-3.0, 2.0};
    m2BodyId boxC = NewBody(world, &cd2);
    NewPolygonShape(world, boxC, &asd, &unit);

    m2BodyDef dd = m2DefaultBodyDef();
    dd.type = m2_dynamicBody;
    dd.position = (m2Pos2){-6.0, 2.0};
    m2BodyId boxD = NewBody(world, &dd);
    m2Circle wheelDisc = {{0.0f, 0.0f}, 0.35f};
    NewCircleShape(world, boxD, &asd, &wheelDisc);

    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = boxA;
    dj.hertz = 4.0f;
    dj.dampingRatio = 0.6f;
    dj.minLength = 0.5f;
    dj.maxLength = 4.0f;
    m2JointId rope = NewDistanceJoint(world, &dj);
    m2Joint_SetBreakLimits(world, rope, 60.0f, 0.0f);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = boxB;
    rj.enableMotor = true;
    rj.motorSpeed = 1.2f;
    rj.maxMotorTorque = 20.0f;
    rj.enableLimit = true;
    rj.lowerAngle = -0.5f;
    rj.upperAngle = 0.7f;
    NewRevoluteJoint(world, &rj);
    m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
    pj.bodyIdA = ground;
    pj.bodyIdB = boxC;
    pj.localAxisA = (m2Vec2){0.0f, 1.0f};
    pj.enableMotor = true;
    pj.motorSpeed = -0.5f;
    pj.maxMotorForce = 30.0f;
    pj.enableLimit = true;
    pj.lowerTranslation = -1.0f;
    pj.upperTranslation = 2.0f;
    NewPrismaticJoint(world, &pj);
    m2WeldJointDef wj = m2DefaultWeldJointDef();
    wj.bodyIdA = boxA;
    wj.bodyIdB = boxB;
    wj.linearHertz = 3.0f;
    wj.linearDampingRatio = 0.5f;
    wj.angularHertz = 6.0f;
    wj.angularDampingRatio = 0.8f;
    NewWeldJoint(world, &wj);
    m2RatchetJointDef pawl = m2DefaultRatchetJointDef();
    pawl.bodyIdA = boxB;
    pawl.bodyIdB = boxD;
    pawl.ratchet = 0.4f;
    pawl.phase = 0.1f;
    NewRatchetJoint(world, &pawl);
    m2PulleyJointDef hoist = m2DefaultPulleyJointDef();
    hoist.bodyIdA = boxB;
    hoist.bodyIdB = boxD;
    hoist.groundAnchorA = (m2Pos2){-1.0, 9.0};
    hoist.groundAnchorB = (m2Pos2){3.0, 9.0};
    hoist.ratio = 1.5f;
    NewPulleyJoint(world, &hoist);
    m2GearJointDef cog = m2DefaultGearJointDef();
    cog.bodyIdA = boxB;
    cog.bodyIdB = boxD;
    cog.ratio = 1.5f;
    NewGearJoint(world, &cog);
    m2MotorJointDef chase = m2DefaultMotorJointDef();
    chase.bodyIdA = hook;
    chase.bodyIdB = boxC;
    chase.linearOffset = (m2Vec2){0.5f, -0.25f};
    chase.angularOffset = 0.2f;
    chase.maxForce = 77.0f;
    chase.maxTorque = 33.0f;
    chase.correctionFactor = 0.4f;
    NewMotorJoint(world, &chase);
    m2MouseJointDef grip = m2DefaultMouseJointDef();
    grip.bodyIdA = ground;
    grip.bodyIdB = boxD;
    grip.target = (m2Pos2){-6.0, 2.5};
    grip.hertz = 5.0f;
    grip.dampingRatio = 0.8f;
    grip.maxForce = 44.0f;
    grip.collideConnected = true;
    NewMouseJoint(world, &grip);
    m2FilterJointDef pact = m2DefaultFilterJointDef();
    pact.bodyIdA = boxA;
    pact.bodyIdB = boxC;
    NewFilterJoint(world, &pact);
    m2WheelJointDef whj = m2DefaultWheelJointDef();
    whj.bodyIdA = ground;
    whj.bodyIdB = boxD;
    whj.localAxisA = (m2Vec2){0.0f, 1.0f};
    whj.enableSpring = true;
    whj.hertz = 2.5f;
    whj.dampingRatio = 0.7f;
    whj.enableMotor = true;
    whj.motorSpeed = 4.0f;
    whj.maxMotorTorque = 15.0f;
    whj.enableLimit = true;
    whj.lowerTranslation = -0.3f;
    whj.upperTranslation = 0.3f;
    whj.collideConnected = true;
    NewWheelJoint(world, &whj);

    m2World* mirror = MirrorWorld(world, &def);
    CHECK(m2World_Hash(mirror) == m2World_Hash(world), "the mirror is bit-identical at creation");

    bool tracked = true;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2World_Step(mirror, 1.0f / 60.0f, 4);
        tracked = tracked && m2World_Hash(mirror) == m2World_Hash(world);
    }
    CHECK(tracked, "and tracks the original for 90 steps");

    m2DestroyWorld(mirror);
    m2DestroyWorld(world);
}

// The body dynamics pack: forces with one-step lifetime,
// Pade damping, fixed rotation as a mass property, and sleep control
// at both scopes, through the journaled channels and the snapshot.
static void TestBodyDynamicsPack(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f}); // isolate the pack

    // Force: F = m*a, integrated over a second.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 10.0};
    m2BodyId pushed = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f); // 0.64 kg at density 1
    NewPolygonShape(world, pushed, &sd, &unit);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2Body_ApplyForce(world, pushed, (m2Vec2){6.4f, 0.0f}); // a = 10
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float vx = m2Body_GetLinearVelocity(world, pushed).x;
    CHECK(vx > 9.0f && vx < 11.0f, "one second of F=ma reaches ten");

    // One-step lifetime: no further force, velocity coasts.
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float coast = m2Body_GetLinearVelocity(world, pushed).x;
    CHECK(coast > vx - 0.01f && coast < vx + 0.01f, "forces die with their step");

    // Damping: exp(-c*t) via Pade, c=4 over one second leaves ~e^-4.
    m2Body_SetLinearDamping(world, pushed, 4.0f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float damped = m2Body_GetLinearVelocity(world, pushed).x;
    CHECK(damped > 0.05f && damped < 0.5f, "damping decays as v / (1 + h d) per substep");

    // Torque spins, angular damping stills.
    m2Body_ApplyTorque(world, pushed, 5.0f);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(world, pushed) > 0.0f, "torque spins");
    m2Body_SetAngularDamping(world, pushed, 8.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float spin = m2Body_GetAngularVelocity(world, pushed);
    CHECK(spin < 0.05f, "angular damping stills the spin");

    // Fixed rotation: an off-center impulse must not rotate it.
    m2BodyDef fdd = m2DefaultBodyDef();
    fdd.type = m2_dynamicBody;
    fdd.position = (m2Pos2){5.0, 10.0};
    fdd.fixedRotation = true;
    m2BodyId upright = NewBody(world, &fdd);
    NewPolygonShape(world, upright, &sd, &unit);
    m2Body_ApplyLinearImpulseAtPoint(world, upright, (m2Vec2){0.0f, 2.0f}, (m2Pos2){5.4, 10.0});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(world, upright) == 0.0f, "fixed rotation never spins");
    CHECK(m2Body_IsFixedRotation(world, upright), "and reads back");
    m2Body_SetFixedRotation(world, upright, false);
    m2Body_ApplyLinearImpulseAtPoint(world, upright, (m2Vec2){0.0f, 2.0f}, (m2Pos2){5.4, 10.0});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_GetAngularVelocity(world, upright) != 0.0f, "released, it spins again");

    m2DestroyWorld(world);

    // Sleep control at both scopes, on a world with real gravity.
    m2World* nap = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(nap, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(nap, floor, &fs, &slab);
    m2BodyDef nd = m2DefaultBodyDef();
    nd.type = m2_dynamicBody;
    nd.position = (m2Pos2){-2.0, 0.45};
    m2BodyId sleeper = NewBody(nap, &nd);
    NewPolygonShape(nap, sleeper, &sd, &unit);
    m2BodyDef id2 = nd;
    id2.position = (m2Pos2){2.0, 0.45};
    id2.enableSleep = false;
    m2BodyId insomniac = NewBody(nap, &id2);
    NewPolygonShape(nap, insomniac, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(nap, sleeper), "the plain twin sleeps");
    CHECK(m2Body_IsAwake(nap, insomniac), "the flagged twin never does");
    CHECK(!m2Body_IsSleepEnabled(nap, insomniac), "and reads back");

    m2World_EnableSleeping(nap, false);
    CHECK(m2Body_IsAwake(nap, sleeper), "killing the master switch wakes the sleeper");
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_IsAwake(nap, sleeper), "and nobody sleeps while it is off");
    m2World_EnableSleeping(nap, true);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(nap, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(nap, sleeper), "switch back on, naps resume");
    m2DestroyWorld(nap);

    // The same master switch, given at creation.
    m2WorldDef awakeDef = def;
    awakeDef.enableSleeping = false;
    m2World* wakeful = NewWorld(&awakeDef);
    NewPolygonShape(wakeful, NewBody(wakeful, &gd), &fs, &slab);
    m2BodyId restless = NewBody(wakeful, &nd);
    NewPolygonShape(wakeful, restless, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(wakeful, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_IsAwake(wakeful, restless), "a world created without sleep never naps");
    m2DestroyWorld(wakeful);
}

// Integration extras (parity sprint 4a): dormancy that ends contacts
// and wakes riders, mass overrides with a documented lifetime, and
// one deterministic blast.
static void TestDormancyMassAndExplosions(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    // Dormancy: a pedestal with a sleeper on top goes dormant; the
    // rider wakes and falls (the destroy-path wake law, third user).
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 0.75};
    m2BodyId pedestal = NewBody(world, &pd);
    m2Polygon post = m2MakeBox(0.6f, 0.75f);
    NewPolygonShape(world, pedestal, &fs, &post);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.95};
    m2BodyId rider = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, rider, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, rider), "the rider sleeps on the pedestal");
    m2Body_Disable(world, pedestal);
    CHECK(!m2Body_IsEnabled(world, pedestal), "dormancy reads back");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, rider).y < 0.6, "the rider wakes and lands on the floor");
    m2RayCastResult probe = m2World_CastRayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -4.0f},
                                                   m2DefaultQueryFilter());
    CHECK(!probe.hit, "a dormant body is invisible to queries");
    m2Body_Enable(world, pedestal);
    probe = m2World_CastRayClosest(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, -4.5f},
                                   m2DefaultQueryFilter());
    CHECK(probe.hit, "and reappears where it stood");

    // Mass override: twice the mass, same impulse, half the speed.
    m2BodyDef md = m2DefaultBodyDef();
    md.type = m2_dynamicBody;
    md.position = (m2Pos2){8.0, 5.0};
    m2BodyId heavy = NewBody(world, &md);
    NewPolygonShape(world, heavy, &sd, &unit);
    m2MassData original = m2Body_GetMassData(world, heavy);
    m2MassData doubled = original;
    doubled.mass = original.mass * 2.0f;
    doubled.rotationalInertia = original.rotationalInertia * 2.0f;
    m2Body_SetMassData(world, heavy, doubled);
    CHECK(m2Body_GetMass(world, heavy) > original.mass * 1.9f, "the override sticks");
    m2Body_ApplyLinearImpulseAtPoint(world, heavy, (m2Vec2){original.mass * 2.0f, 0.0f},
                                     m2Body_GetPosition(world, heavy));
    float vx = m2Body_GetLinearVelocity(world, heavy).x;
    CHECK(vx > 0.9f && vx < 1.1f, "twice the mass takes half the speed");
    m2Body_ApplyMassFromShapes(world, heavy);
    m2MassData back = m2Body_GetMassData(world, heavy);
    CHECK(back.mass < original.mass * 1.1f, "shapes take the mass back on request");

    // Explosion: two crates flank the blast inside the radius, a third
    // sits in the falloff band, a fourth is filtered out by mask.
    double bx = 20.0;
    m2BodyId crates[4];
    double at[4] = {bx - 1.0, bx + 1.0, bx + 3.0, bx - 1.0};
    double ay[4] = {0.45, 0.45, 0.45, 1.6};
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){at[i], ay[i]};
        crates[i] = NewBody(world, &cd);
        m2ShapeDef xs = m2DefaultShapeDef();
        if (i == 3)
        {
            xs.categoryBits = 0x8000u; // outside the blast mask
        }
        NewPolygonShape(world, crates[i], &xs, &unit);
    }
    m2ExplosionDef boom = m2DefaultExplosionDef();
    boom.position = (m2Pos2){bx, 0.45};
    boom.radius = 1.5f;
    boom.falloff = 2.0f;
    boom.impulse = 3.0f;
    boom.maskBits = 0x1u;
    m2World_Explode(world, &boom);
    float vLeft = m2Body_GetLinearVelocity(world, crates[0]).x;
    float vRight = m2Body_GetLinearVelocity(world, crates[1]).x;
    float vFar = m2Body_GetLinearVelocity(world, crates[2]).x;
    CHECK(vLeft < -1.0f, "the left crate flies left");
    CHECK(vRight > 1.0f, "the right crate flies right");
    CHECK(vFar > 0.1f && vFar < vRight, "the falloff crate gets a weaker push");
    float vMasked = m2Body_GetLinearVelocity(world, crates[3]).x;
    CHECK(vMasked == 0.0f, "the masked crate never feels it");

    m2DestroyWorld(world);
}

// Small-basket readers and mutators (parity sprint 4b): frame
// helpers, the per-body joint walk, runtime geometry, and chain
// materials.
static void TestSmallBasket(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    // Frame helpers on a rotated, offset body.
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){100.0, 50.0};
    bd.rotation = (m2Rot){0.0f, 1.0f}; // ninety degrees
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, body, &sd, &unit);
    m2Pos2 wp = m2Body_GetWorldPoint(world, body, (m2Vec2){1.0f, 0.0f});
    CHECK(wp.x > 99.9 && wp.x < 100.1 && wp.y > 50.9 && wp.y < 51.1,
          "a local x step becomes a world y step under ninety degrees");
    m2Vec2 back = m2Body_GetLocalPoint(world, body, wp);
    CHECK(back.x > 0.99f && back.x < 1.01f && back.y > -0.01f && back.y < 0.01f,
          "and comes back local");
    m2Vec2 wv = m2Body_GetWorldVector(world, body, (m2Vec2){1.0f, 0.0f});
    CHECK(wv.y > 0.99f, "vectors rotate the same way");
    m2Body_SetAngularVelocity(world, body, 2.0f);
    m2Vec2 pv = m2Body_GetWorldPointVelocity(
        world, body, m2Body_GetWorldPoint(world, body, (m2Vec2){1.0f, 0.0f}));
    CHECK(pv.x < -1.9f && pv.x > -2.1f, "point velocity is w cross r");

    // Per-body joint walk.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){104.0, 50.0};
    m2BodyId post = NewBody(world, &ad);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = post;
    dj.bodyIdB = body;
    m2JointId rope = NewDistanceJoint(world, &dj);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = post;
    mj.bodyIdB = body;
    m2JointId chase = NewMotorJoint(world, &mj);
    CHECK(m2Body_GetJoints(world, body, NULL, 0) == 2, "the body names both joints");
    m2JointId list[4];
    int32_t n = m2Body_GetJoints(world, post, list, 4);
    CHECK(n == 2 && list[0].index1 == rope.index1 && list[1].index1 == chase.index1,
          "ascending and correct");

    // Runtime geometry: a floor shrinks under a sleeper.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    m2ShapeId ground = NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){4.0, 0.45};
    m2BodyId crate = NewBody(world, &cd);
    NewPolygonShape(world, crate, &sd, &unit);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, crate), "the crate sleeps at the slab's edge");
    m2Polygon stub = m2MakeBox(1.0f, 0.5f);
    m2Shape_SetPolygon(world, ground, &stub);
    CHECK(m2Shape_GetPolygon(world, ground).count == 4, "geometry reads back after the swap");
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, crate).y < -1.0, "the sleeper wakes and falls off the stub");

    // Type change: the stub becomes a circle and reports it.
    m2Circle dome = {{0.0f, 0.0f}, 0.8f};
    m2Shape_SetCircle(world, ground, &dome);
    CHECK(m2Shape_GetType(world, ground) == m2_circleShape, "runtime type change reads back");

    // Chain materials: every link picks up the new values.
    m2Vec2 pts[5] = {{22.0f, 0.0f}, {20.0f, 0.0f}, {18.0f, 0.0f}, {16.0f, 0.0f}, {14.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    m2ChainId ledge = NewChain(world, floor, &chain);
    m2Chain_SetFriction(world, ledge, 0.85f);
    m2Chain_SetRestitution(world, ledge, 0.35f);
    m2ShapeId links[4];
    int32_t linkCount = m2Chain_GetShapes(world, ledge, links, 4);
    bool allSet = linkCount == 2;
    for (int32_t i = 0; i < linkCount; ++i)
    {
        allSet = allSet && m2Shape_GetFriction(world, links[i]) == 0.85f &&
                 m2Shape_GetRestitution(world, links[i]) == 0.35f;
    }
    CHECK(allSet, "chain materials reach every link");

    m2DestroyWorld(world);
}

// The leftover basket (audit appendix bucket 1): every small reader
// and setter of the full surface, exercised end to end.
static void TestLeftoverBasket(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2ShapeId boxShape = NewPolygonShape(world, box, &sd, &unit);

    // Impulse to center: speed without spin.
    m2Body_ApplyLinearImpulse(world, box, (m2Vec2){0.64f, 0.0f});
    CHECK(m2Body_GetLinearVelocity(world, box).x > 0.9f &&
              m2Body_GetAngularVelocity(world, box) == 0.0f,
          "center impulse moves without spin");

    // Manual sleep: stilled instantly, then woken on demand.
    m2Body_SetAwake(world, box, false);
    CHECK(!m2Body_IsAwake(world, box) && m2Body_GetLinearVelocity(world, box).x == 0.0f,
          "forced sleep stills the body");
    m2Body_SetAwake(world, box, true);
    CHECK(m2Body_IsAwake(world, box), "and it wakes on demand");

    // Runtime flags and data.
    m2Body_SetBullet(world, box, true);
    CHECK(m2Body_IsBullet(world, box), "bullet flag flips at runtime");
    m2Body_SetUserData(world, box, 777);
    m2Shape_SetUserData(world, boxShape, 888);
    CHECK(m2Body_GetUserData(world, box) == 777 && m2Shape_GetUserData(world, boxShape) == 888,
          "user data round-trips at runtime");
    float massBefore = m2Body_GetMass(world, box);
    m2Shape_SetDensity(world, boxShape, 2.0f);
    CHECK(m2Body_GetMass(world, box) > massBefore * 1.9f, "density doubles the mass");

    // Mass and frame readers.
    m2Pos2 com = m2Body_GetWorldCenterOfMass(world, box);
    CHECK(com.x > -0.01 && com.x < 0.01 && com.y > 4.99 && com.y < 5.01,
          "world center of mass sits on the centered box");
    CHECK(m2Body_GetRotationalInertia(world, box) > 0.0f, "inertia reads positive");
    m2AabbResult ab = m2Body_ComputeAabb(world, box);
    CHECK(ab.lowerBound.x < -0.39 && ab.upperBound.x > 0.39 && ab.upperBound.y > 5.39,
          "the body AABB hugs the shape");
    m2AabbResult sab = m2Shape_GetAabb(world, boxShape);
    CHECK(sab.lowerBound.y < 4.61 && sab.upperBound.y > 5.39, "the shape AABB agrees");

    // One-shape queries.
    CHECK(m2Shape_TestPoint(world, boxShape, (m2Pos2){0.0, 5.0}), "the center is inside");
    CHECK(!m2Shape_TestPoint(world, boxShape, (m2Pos2){2.0, 5.0}), "two meters out is not");
    m2Pos2 closest = m2Shape_GetClosestPoint(world, boxShape, (m2Pos2){3.0, 5.0});
    CHECK(closest.x > 0.35 && closest.x < 0.45, "closest point sits on the face");
    m2RayCastResult ray =
        m2Shape_CastRay(world, boxShape, (m2Pos2){3.0, 5.0}, (m2Vec2){-5.0f, 0.0f});
    CHECK(ray.hit && ray.normal.x > 0.99f, "the one-shape ray hits the right face");

    // Chain parentage.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, 0.0};
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 pts[5] = {{4.0f, 0.0f}, {2.0f, 0.0f}, {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5;
    m2ChainId ledge = NewChain(world, ground, &chain);
    m2ShapeId links[4];
    m2Chain_GetShapes(world, ledge, links, 4);
    m2ChainId parent = m2Shape_GetParentChain(world, links[0]);
    CHECK(parent.index1 == ledge.index1 && parent.generation == ledge.generation,
          "links name their chain");
    CHECK(m2Shape_GetParentChain(world, boxShape).index1 == 0, "plain shapes name none");

    // Joint extras: user data and drift readers.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){0.0, 8.0};
    m2BodyId hook = NewBody(world, &hd);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = box;
    dj.userData = 4242;
    m2JointId rope = NewDistanceJoint(world, &dj);
    CHECK(m2Joint_GetUserData(world, rope) == 4242, "joint user data arrives from the def");
    m2Joint_SetUserData(world, rope, 5151);
    CHECK(m2Joint_GetUserData(world, rope) == 5151, "and flips at runtime");
    // Teleport the box a meter outward: the rope reads that meter.
    m2Body_SetTransform(world, box, (m2Pos2){0.0, 1.0}, (m2Rot){1.0f, 0.0f});
    float stretch = m2Joint_GetLinearSeparation(world, rope);
    CHECK(stretch > 3.9f && stretch < 4.1f, "linear separation reads the stretch");
    CHECK(m2Joint_GetAngularSeparation(world, rope) == 0.0f, "a rope pins no angle");

    // Kinematic follow: one step to the target pose.
    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){20.0, 0.0};
    m2BodyId mover = NewBody(world, &kd);
    NewPolygonShape(world, mover, &sd, &unit);
    m2Body_SetTargetTransform(world, mover, (m2Pos2){21.0, 0.5}, (m2Rot){1.0f, 0.0f}, 1.0f / 60.0f);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Pos2 landed = m2Body_GetPosition(world, mover);
    CHECK(landed.x > 20.9 && landed.x < 21.1 && landed.y > 0.4 && landed.y < 0.6,
          "target transform lands in one step");

    m2DestroyWorld(world);
}

// Dominance: the higher body cannot be
// pushed by the lower one in contacts, statics outrank everyone,
// joints stay symmetric.
// The diagnostics surface: a healthy world validates, quiet facts
// count, and typed-path misuse counts without asserting.
static void TestValidateAndCounters(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(4.0f, 0.5f);
    NewPolygonShape(world, NewBody(world, &fd), &fs, &slab);
    m2BodyDef ad = m2DefaultBodyDef();
    m2BodyId hook = NewBody(world, &ad);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.0};
    m2BodyId bob = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle disc = {{0.0f, 0.0f}, 0.3f};
    NewCircleShape(world, bob, &sd, &disc);
    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = hook;
    dj.bodyIdB = bob;
    m2JointId rope = NewDistanceJoint(world, &dj);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Validate(world), "a healthy world validates");
    uint64_t before = m2World_Hash(world);
    m2World_Validate(world);
    CHECK(m2World_Hash(world) == before, "the validator is a pure reader");

    // Quiet facts count: fill the pool past its brim.
    for (int32_t i = 0; i < 12; ++i)
    {
        EmitParticle(world, (m2Pos2){(double)i, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    m2Counters counters = m2World_GetCounters(world);
    CHECK(counters.particlePoolFull == 4, "refused emits are on the scoreboard");

    // Typed-path misuse counts without asserting: a gear getter on a
    // distance joint is quiet in every build, and countable.
    CHECK(m2GearJoint_GetRatio(world, rope) == 0.0f, "the wrong-type getter answers zero");
    counters = m2World_GetCounters(world);
    CHECK(counters.misuse >= 1, "the wrong-type read is on the scoreboard");
    m2DestroyWorld(world);
}

// A host allocator that refuses once a budget of successful calls is
// spent, and counts live blocks so a leak shows up as a nonzero total.
static int32_t s_allocBudget = 0;
static int32_t s_allocLive = 0;

static void* FailingAlloc(size_t bytes, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    if (s_allocBudget <= 0)
    {
        return NULL;
    }
    s_allocBudget -= 1;
    void* memory = malloc(bytes);
    s_allocLive += memory != NULL ? 1 : 0;
    return memory;
}

static void FailingFree(void* memory, size_t bytes, size_t alignment, void* context)
{
    (void)bytes;
    (void)alignment;
    (void)context;
    if (memory != NULL)
    {
        s_allocLive -= 1;
        free(memory);
    }
}

static void TestCreateWorldOutOfMemory(void)
{
    // Refuse each allocation of world creation in turn, with fluids on so
    // every array kind is allocated. Every attempt must refuse cleanly
    // with nothing left allocated, or succeed and destroy cleanly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.particleCapacity = 32;
    def.fluidVolumeCapacity = 2;
    def.allocator.alloc = FailingAlloc;
    def.allocator.free = FailingFree;
    int32_t refusals = 0;
    bool created = false;
    for (int32_t budget = 0; budget < 1024 && !created; ++budget)
    {
        s_allocBudget = budget;
        s_allocLive = 0;
        m2World* world = NULL;
        m2Result result = m2CreateWorld(&def, &world);
        if (result == m2_success)
        {
            created = true;
            m2DestroyWorld(world);
        }
        else
        {
            refusals += 1;
            CHECK(result == m2_errorCapacity && world == NULL,
                  "a refused allocation reports capacity and the null world");
        }
        if (s_allocLive != 0)
        {
            printf("FAIL: %d block(s) leaked with an allocation budget of %d\n", s_allocLive,
                   budget);
            s_failures += 1;
            break;
        }
    }
    CHECK(created, "world creation succeeds once the allocator stops refusing");
    CHECK(refusals > 100, "every allocation of world creation was refused once");
}

static void TestInfiniteInputIsRefused(void)
{
    // NaN was refused before; infinity slipped through the x == x
    // check. Both are refused now.
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 8;
    m2World* world = NewWorld(&def);
    float inf = INFINITY;
    m2ParticleId p = m2_nullParticleId;
    CHECK(m2World_EmitParticle(world, (m2Pos2){0.0, (double)inf}, (m2Vec2){0.0f, 0.0f}, 0, &p) ==
                  m2_errorInvalid &&
              p.index1 == 0,
          "an infinite position is refused");
    m2World_SetWind(world, (m2Vec2){inf, 0.0f}, 0.1f);
    m2Vec2 wind = {1.0f, 1.0f};
    m2World_GetWind(world, &wind, NULL);
    CHECK(wind.x == 0.0f && wind.y == 0.0f, "an infinite wind is refused");
    m2DestroyWorld(world);
}

// A def field a snapshot would refuse is refused at creation, so a world
// never writes a snapshot it cannot restore. Each bad def changes one
// field; the edge values that pass round-trip through a snapshot.
static m2BodyDef BadBodyDef(int32_t field)
{
    m2BodyDef def = m2DefaultBodyDef();
    def.type = m2_dynamicBody;
    switch (field)
    {
    case 0:
        def.type = (m2BodyType)-1;
        break;
    case 1:
        def.type = (m2BodyType)3;
        break;
    case 2:
        def.position.x = (double)NAN;
        break;
    case 3:
        def.position.y = (double)INFINITY;
        break;
    case 4:
        def.rotation.c = NAN;
        break;
    case 5:
        def.rotation.s = -INFINITY;
        break;
    case 6:
        def.linearVelocity.x = NAN;
        break;
    case 7:
        def.linearVelocity.y = INFINITY;
        break;
    case 8:
        def.angularVelocity = NAN;
        break;
    case 9:
        def.gravityScale = INFINITY;
        break;
    case 10:
        def.linearDamping = -1.0f;
        break;
    case 11:
        def.linearDamping = NAN;
        break;
    case 12:
        def.angularDamping = -INFINITY;
        break;
    case 13:
        def.angularDamping = INFINITY;
        break;
    default:
        def.rotation = (m2Rot){0.0f, -0.0f}; // no direction
        break;
    }
    return def;
}

static m2WorldDef BadWorldDef(int32_t field)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 8;
    float* strengths[] = {&def.particlePressureStrength, &def.particleDampingStrength,
                          &def.particleViscousStrength,  &def.particlePowderStrength,
                          &def.particleSpringStrength,   &def.particleElasticStrength,
                          &def.particleCohesionStrength, &def.particleNearPressureStrength};
    switch (field)
    {
    case 0:
        def.gravity.x = NAN;
        break;
    case 1:
        def.gravity.y = -INFINITY;
        break;
    case 2:
        def.particleRadius = INFINITY;
        break;
    case 3:
        def.particleDensity = INFINITY;
        break;
    case 4:
        def.particleGravityScale = NAN;
        break;
    default:
        *strengths[field - 5] = INFINITY;
        break;
    }
    return def;
}

static void TestDefsMatchWhatARestoreAccepts(void)
{
    for (int32_t field = 0; field < 13; ++field)
    {
        m2WorldDef def = BadWorldDef(field);
        m2World* refused = NULL;
        CHECK(m2CreateWorld(&def, &refused) == m2_errorInvalid && refused == NULL,
              "a world def with a field out of range is refused");
        if (refused != NULL)
        {
            m2DestroyWorld(refused);
        }
    }

    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){-FLT_MAX, FLT_MAX};
    def.particleCapacity = 8;
    def.particleRadius = 0.02f;
    def.particleDensity = FLT_MIN;
    def.particleViscousStrength = FLT_MAX;
    def.particleCohesionStrength = 0.0f;
    m2World* world = NewWorld(&def);
    for (int32_t field = 0; field < 15; ++field)
    {
        m2BodyDef bodyDef = BadBodyDef(field);
        uint64_t misuse = m2World_GetCounters(world).misuse;
        m2BodyId body = {7, 7};
        CHECK(m2CreateBody(world, &bodyDef, &body) == m2_errorInvalid && body.index1 == 0 &&
                  m2World_GetCounters(world).misuse == misuse + 1,
              "a body def with a field out of range is refused and counted");
    }

    m2BodyDef edge = m2DefaultBodyDef();
    edge.position = (m2Pos2){DBL_MAX, -DBL_MAX};
    edge.rotation = (m2Rot){0.0f, 1.0e-30f};
    edge.gravityScale = -FLT_MAX;
    edge.linearDamping = -0.0f;
    edge.angularDamping = FLT_MAX;
    edge.dominance = INT8_MIN;
    for (int32_t type = m2_staticBody; type <= m2_dynamicBody; ++type)
    {
        edge.type = (m2BodyType)type;
        bool still = edge.type == m2_staticBody; // a static def has no velocity
        edge.linearVelocity = still ? (m2Vec2){-0.0f, 0.0f} : (m2Vec2){FLT_MAX, -FLT_MAX};
        edge.angularVelocity = still ? -0.0f : -FLT_MAX;
        m2Rot q = m2Body_GetRotation(world, NewBody(world, &edge));
        CHECK(q.c == 0.0f && q.s == 1.0f, "a tiny rotation keeps its direction");
    }
    edge.rotation = (m2Rot){FLT_MAX, -FLT_MAX};
    m2Rot huge = m2Body_GetRotation(world, NewBody(world, &edge));
    edge.rotation = (m2Rot){1.0e20f, 1.0e20f};
    m2Rot diagonal = m2Body_GetRotation(world, NewBody(world, &edge));
    CHECK(huge.c == diagonal.c && huge.s == -diagonal.s && m2IsNormalizedRot(diagonal) &&
              fabsf(diagonal.c - 0.70710678f) < 1.0e-6f && diagonal.c == diagonal.s,
          "a huge rotation keeps its direction");

    int32_t size = m2World_SnapshotSize(world);
    uint8_t* first = (uint8_t*)malloc((size_t)size);
    uint8_t* second = (uint8_t*)malloc((size_t)size);
    CHECK(first != NULL && second != NULL, "snapshot buffers");
    int32_t written = Snapshot(world, first, size);
    CHECK(written == size && Restore(world, first, written), "edge values restore");
    CHECK(Snapshot(world, second, size) == size && memcmp(first, second, (size_t)size) == 0,
          "a restored world snapshots the same bytes");
    CHECK(m2World_Validate(world), "edge values keep the world valid");
    free(second);
    free(first);
    m2DestroyWorld(world);
}

// A snapshot whose body rotation is not of unit length is refused like
// any other bad bytes: no count, nothing changed. The body's transform is
// found in the snapshot by its bytes.
static void TestRestoreRefusesANonUnitRotation(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.position = (m2Pos2){123.25, -77.5};
    bodyDef.rotation = (m2Rot){0.6f, 0.8f};
    m2BodyId body = NewBody(world, &bodyDef);
    m2Transform xf = {m2Body_GetPosition(world, body), m2Body_GetRotation(world, body)};

    int32_t size = m2World_SnapshotSize(world);
    uint8_t* bytes = (uint8_t*)malloc((size_t)size);
    CHECK(bytes != NULL && Snapshot(world, bytes, size) == size, "the world snapshots");
    int32_t at = -1;
    for (int32_t i = 0; i + (int32_t)sizeof(xf) <= size; ++i)
    {
        if (memcmp(bytes + i, &xf, sizeof(xf)) == 0)
        {
            CHECK(at < 0, "the transform appears once");
            at = i;
        }
    }
    CHECK(at >= 0, "the transform is in the snapshot");

    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuse;
    const m2Rot bad[] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {0.6f, 0.6f}};
    for (size_t k = 0; at >= 0 && k < sizeof(bad) / sizeof(bad[0]); ++k)
    {
        memcpy(bytes + at + (int32_t)sizeof(m2Pos2), &bad[k], sizeof(bad[k]));
        CHECK(m2World_Restore(world, bytes, size) == m2_errorInvalid,
              "a rotation that is not of unit length is refused");
    }
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuse == misuse,
          "the refusal changes nothing and counts nothing");
    CHECK(m2World_Validate(world), "the world stays valid");
    free(bytes);
    m2DestroyWorld(world);
}

// A step takes at most M2_MAX_SUBSTEPS substeps, from a program or from
// a tape. fuzz_journal found a one-byte edit of a tape's step record that
// asked for millions and stalled the replay.
static void TestSubstepCap(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2World_Step(world, 1.0f / 60.0f, M2_MAX_SUBSTEPS + 1);
    CHECK(m2World_GetStepCount(world) == 0 && m2World_GetCounters(world).misuse == misuse + 1,
          "a step above the cap changes nothing and counts one misuse");
    m2World_Step(world, 1.0f / 60.0f, M2_MAX_SUBSTEPS);
    CHECK(m2World_GetStepCount(world) == 1 && m2World_GetCounters(world).misuse == misuse + 1,
          "a step at the cap runs");

    int32_t capacity = m2World_GetJournalBaseSize(world) + 1024;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    m2World_Step(world, 1.0f / 60.0f, 4);
    int32_t bytes = StopJournal(world);
    // The step record's payload: dt, then the substep count.
    struct
    {
        float dt;
        int32_t substepCount;
    } record = {1.0f / 60.0f, 4};
    int32_t at = -1;
    for (int32_t i = 0; i + (int32_t)sizeof(record) <= bytes; ++i)
    {
        if (memcmp(tape + i, &record, sizeof(record)) == 0)
        {
            at = i;
        }
    }
    CHECK(at >= 0, "the tape holds the step record");
    record.substepCount = 0x00B30004;
    if (at >= 0)
    {
        memcpy(tape + at, &record, sizeof(record)); // the fuzz input's edit
    }
    m2World* replica = NewWorld(&def);
    misuse = m2World_GetCounters(replica).misuse;
    (void)m2World_ReplayJournal(replica, tape, bytes);
    CHECK(m2World_GetStepCount(replica) == 1, "the tape's oversized step is skipped");
    CHECK(m2World_GetCounters(replica).misuse == misuse, "and counts nothing");
    m2DestroyWorld(replica);
    free(tape);
    m2DestroyWorld(world);
}

// Host code a world runs, a task hook or a debug draw callback, tries to
// change the world it serves. Every writer refuses: a status writer with
// m2_errorState, a void writer by counting one misuse.
typedef struct Intruder
{
    m2World* world;
    m2BodyId body;
    int32_t calls;
    m2Result created;
    m2Result destroyed;
    m2Result restored;
    uint64_t stepsSeen;
} Intruder;

static void Intrude(Intruder* in)
{
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId made = m2_nullBodyId;
    in->created = m2CreateBody(in->world, &bd, &made);
    in->destroyed = m2DestroyBody(in->world, in->body);
    uint8_t snapshot[16] = {0};
    in->restored = m2World_Restore(in->world, snapshot, (int32_t)sizeof(snapshot));
    m2Body_SetLinearVelocity(in->world, in->body, (m2Vec2){5.0f, 0.0f});
    m2World_Step(in->world, 1.0f / 60.0f, 1);
    in->stepsSeen = m2World_GetStepCount(in->world); // readers stay open
}

static void* IntrudingEnqueue(m2TaskFn* task, int32_t itemCount, int32_t minRange,
                              void* taskContext, void* userContext)
{
    (void)minRange;
    Intruder* in = (Intruder*)userContext;
    if (in->calls++ == 0)
    {
        Intrude(in);
    }
    task(0, itemCount, taskContext);
    return NULL;
}

static void IntrudingFinish(void* userTask, void* userContext)
{
    (void)userTask;
    (void)userContext;
}

static void IntrudingDraw(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context)
{
    (void)p1;
    (void)p2;
    (void)color;
    Intruder* in = (Intruder*)context;
    if (in->calls++ == 0)
    {
        Intrude(in);
    }
}

static void TestHostCodeCannotWrite(void)
{
    Intruder in = {NULL, m2_nullBodyId, 0, m2_success, m2_success, m2_success, 0};
    m2WorldDef def = m2DefaultWorldDef();
    def.enqueueTask = IntrudingEnqueue;
    def.finishTask = IntrudingFinish;
    def.userTaskContext = &in;
    in.world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2Polygon slab = m2MakeBox(4.0f, 0.5f);
    m2ShapeDef sd = m2DefaultShapeDef();
    NewPolygonShape(in.world, NewBody(in.world, &gd), &sd, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.99};
    in.body = NewBody(in.world, &bd);
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    NewCircleShape(in.world, in.body, &sd, &ball);

    // The ball rests on the slab, so the step has contacts to solve and
    // hands them to the task hook.
    uint64_t misuse = m2World_GetCounters(in.world).misuse;
    m2World_Step(in.world, 1.0f / 60.0f, 4);
    CHECK(in.calls > 0, "the step ran the task hook");
    CHECK(in.created == m2_errorState, "a create inside a task refuses with m2_errorState");
    CHECK(in.destroyed == m2_errorState, "so does a destroy");
    CHECK(in.restored == m2_errorState, "and a restore, before it reads the bytes");
    CHECK(in.stepsSeen == 0, "a nested step does not run, and readers still answer");
    CHECK(m2World_GetCounters(in.world).misuse == misuse + 5,
          "each refusal, the void ones too, counts one misuse");
    CHECK(m2Body_IsValid(in.world, in.body), "the body survived its destroy");
    CHECK(m2Body_GetLinearVelocity(in.world, in.body).x < 1.0f, "the velocity never landed");
    CHECK(m2World_GetStepCount(in.world) == 1, "only the outer step counts");

    // A debug draw is host code too, and the world is writable again after.
    in.calls = 0;
    in.stepsSeen = 99;
    m2DebugDraw draw = {0};
    draw.drawSegment = IntrudingDraw;
    draw.drawAabbs = true;
    draw.context = &in;
    m2World_Draw(in.world, &draw);
    CHECK(in.calls > 0, "the draw ran its callback");
    CHECK(in.created == m2_errorState && in.destroyed == m2_errorState,
          "writers refuse inside a draw callback");
    CHECK(in.stepsSeen == 1, "a step inside a draw does not run");

    m2BodyId after = m2_nullBodyId;
    CHECK(m2CreateBody(in.world, &bd, &after) == m2_success, "outside host code writers work");
    m2DestroyWorld(in.world);
}

// The allocator is host code too: one that tries to change the world it
// serves, from inside alloc or free, is refused, during a replay's scratch
// allocations and a destroy's frees alike.
typedef struct Meddler
{
    m2World* target;
    m2BodyId body;
    int32_t allocTries;
    int32_t allocRefused;
    int32_t freeTries;
    int32_t freeRefused;
} Meddler;

static void* MeddlingAlloc(size_t size, size_t alignment, void* context)
{
    (void)alignment;
    Meddler* m = (Meddler*)context;
    if (m->target != NULL)
    {
        m->allocTries += 1;
        m2BodyDef bd = m2DefaultBodyDef();
        m2BodyId made = m2_nullBodyId;
        m->allocRefused += m2CreateBody(m->target, &bd, &made) == m2_errorState ? 1 : 0;
    }
    return malloc(size);
}

static void MeddlingFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    Meddler* m = (Meddler*)context;
    if (m->target != NULL)
    {
        m->freeTries += 1;
        m->freeRefused += m2DestroyBody(m->target, m->body) == m2_errorState ? 1 : 0;
    }
    free(memory);
}

static void TestAllocatorCannotWrite(void)
{
    Meddler quiet = {NULL, m2_nullBodyId, 0, 0, 0, 0};
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 32;
    def.allocator.alloc = MeddlingAlloc;
    def.allocator.free = MeddlingFree;
    def.allocator.context = &quiet;
    m2World* world = NewWorld(&def);

    // A tape whose replay allocates scratch: a chain and a shatter.
    int32_t capacity = m2World_GetJournalBaseSize(world) + (1 << 14);
    void* tape = malloc((size_t)capacity);
    CHECK(StartJournal(world, tape, capacity), "the journal starts");
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 ledge[4] = {{-4.0f, 0.0f}, {-2.0f, 0.0f}, {2.0f, 0.0f}, {4.0f, 0.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = ledge;
    cd.count = 4;
    CHECK(NewChain(world, ground, &cd).index1 != 0, "the chain is recorded");
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId crate = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, crate, &sd, &box);
    m2Vec2 left[4] = {{-0.5f, -0.5f}, {0.0f, -0.5f}, {0.0f, 0.5f}, {-0.5f, 0.5f}};
    m2Vec2 right[4] = {{0.0f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {0.0f, 0.5f}};
    m2Polygon halves[2] = {MakePolygon(left, 4, 0.0f), MakePolygon(right, 4, 0.0f)};
    CHECK(ShatterBody(world, crate, halves, 2, NULL, 0) == 2, "the shatter is recorded");
    int32_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the journal stops");

    Meddler meddler = {NULL, m2_nullBodyId, 0, 0, 0, 0};
    def.allocator.context = &meddler;
    m2World* fresh = NewWorld(&def);
    meddler.body = NewBody(fresh, &gd);
    meddler.target = fresh;
    uint64_t misuse = m2World_GetCounters(fresh).misuse;
    CHECK(ReplayJournal(fresh, tape, bytes), "the tape replays");
    CHECK(meddler.allocTries >= 3, "the replay allocated its backup, the chain and the pieces");
    CHECK(meddler.allocRefused == meddler.allocTries, "every create from inside alloc refused");
    CHECK(meddler.freeTries >= 3 && meddler.freeRefused == meddler.freeTries,
          "every destroy from inside free refused");
    CHECK(m2World_GetCounters(fresh).misuse ==
              misuse + (uint64_t)(meddler.allocTries + meddler.freeTries),
          "host code calling back counts as misuse, mid-replay too");
    CHECK(m2World_Hash(fresh) == m2World_Hash(world), "and the replay is untouched by it");

    // Destroying the world frees through the same allocator.
    meddler.freeTries = 0;
    meddler.freeRefused = 0;
    m2DestroyWorld(fresh);
    CHECK(meddler.freeTries > 1 && meddler.freeRefused == meddler.freeTries,
          "a destroy's frees cannot change the world either");
    m2DestroyWorld(world);
    free(tape);
}

// Host code a replay runs is the caller's: a task hook that reads a stale
// id while a tape's step replays counts misuse as it would anywhere else.
typedef struct Peeker
{
    m2World* world;
    m2BodyId stale;
    int32_t calls;
} Peeker;

static void* PeekingEnqueue(m2TaskFn* task, int32_t itemCount, int32_t minRange, void* taskContext,
                            void* userContext)
{
    (void)minRange;
    Peeker* peek = (Peeker*)userContext;
    if (peek->world != NULL)
    {
        peek->calls += 1;
        (void)m2Body_GetLinearVelocity(peek->world, peek->stale);
    }
    task(0, itemCount, taskContext);
    return NULL;
}

static void PeekingFinish(void* userTask, void* userContext)
{
    (void)userTask;
    (void)userContext;
}

static void TestHostCodeMistakesCountMidReplay(void)
{
    Peeker peek = {NULL, {9u, 1u}, 0};
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.enqueueTask = PeekingEnqueue;
    def.finishTask = PeekingFinish;
    def.userTaskContext = &peek;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2Polygon slab = m2MakeBox(4.0f, 0.5f);
    m2ShapeDef sd = m2DefaultShapeDef();
    NewPolygonShape(world, NewBody(world, &gd), &sd, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.99};
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    NewCircleShape(world, NewBody(world, &bd), &sd, &ball);

    // The ball rests on the slab, so the recorded step hands work to the
    // task hook, and so does its replay.
    int32_t capacity = m2World_GetJournalBaseSize(world) + 1024;
    void* tape = malloc((size_t)capacity);
    CHECK(StartJournal(world, tape, capacity), "the journal starts");
    m2World_Step(world, 1.0f / 60.0f, 4);
    int32_t bytes = StopJournal(world);
    CHECK(bytes > 0, "the journal stops");

    m2World* fresh = NewWorld(&def);
    peek.world = fresh;
    uint64_t misuse = m2World_GetCounters(fresh).misuse;
    CHECK(ReplayJournal(fresh, tape, bytes), "the tape replays");
    CHECK(peek.calls > 0, "the replayed step ran the task hook");
    CHECK(m2World_GetCounters(fresh).misuse == misuse + (uint64_t)peek.calls,
          "each stale read from the hook counts, mid-replay too");
    peek.world = NULL;
    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(tape);
}

// Untrusted bytes are refused with a status and count no misuse: a
// corrupt snapshot, a corrupt tape, and a tape op whose value the setter
// refuses during the replay.
static void TestCorruptInputIsNotMisuse(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId box = NewBody(world, &bd);

    int32_t size = m2World_SnapshotSize(world);
    uint8_t* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "a snapshot is taken");
    int32_t capacity = m2World_GetJournalBaseSize(world) + 256;
    uint8_t* tape = malloc((size_t)capacity);
    CHECK(StartJournal(world, tape, capacity), "the journal starts");
    m2Body_SetLinearDamping(world, box, 0.25f);
    int32_t bytes = StopJournal(world);

    uint64_t misuse = m2World_GetCounters(world).misuse;
    snap[0] ^= 0xFF;
    CHECK(m2World_Restore(world, snap, size) == m2_errorInvalid, "a corrupt snapshot refuses");
    CHECK(m2World_Restore(world, snap, 3) == m2_errorInvalid, "so does a truncated one");
    // The last record is the body-param op; its channel byte sits four
    // bytes from the end. An unknown channel is refused by the setter the
    // replay calls.
    tape[bytes - 4] = 200;
    CHECK(m2World_ReplayJournal(world, tape, bytes) == m2_errorInvalid, "a corrupt tape refuses");
    tape[0] ^= 0xFF;
    CHECK(m2World_ReplayJournal(world, tape, bytes) == m2_errorInvalid,
          "and so does one with a broken header");
    CHECK(m2World_GetCounters(world).misuse == misuse, "none of it counts as misuse");

    CHECK(m2World_Restore(world, NULL, size) == m2_errorInvalid, "a null buffer refuses");
    CHECK(m2World_GetCounters(world).misuse == misuse + 1, "and that is misuse");
    free(tape);
    free(snap);
    m2DestroyWorld(world);
}

// A call that returns a status reports a stale id as m2_errorStale and
// counts it; a getter returns the zeroed value and counts it.
static void TestStaleIdsInStatusCalls(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 4;
    def.fluidVolumeCapacity = 1;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId body = NewBody(world, &bd);
    m2BodyId other = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeId shape = NewCircleShape(world, body, &sd, &ball);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = body;
    jd.bodyIdB = other;
    m2JointId joint = NewDistanceJoint(world, &jd);
    m2ParticleId particle = EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    m2FluidVolumeDef vd = m2DefaultFluidVolumeDef();
    m2FluidVolumeId volume = NewFluidVolume(world, &vd);

    CHECK(m2DestroyShape(world, shape) == m2_success, "a live shape destroys");
    CHECK(m2DestroyShape(world, shape) == m2_errorStale, "a stale shape is m2_errorStale");
    CHECK(m2DestroyJoint(world, joint) == m2_success, "a live joint destroys");
    CHECK(m2DestroyJoint(world, joint) == m2_errorStale, "a stale joint is m2_errorStale");
    CHECK(m2DestroyParticle(world, particle) == m2_success, "a live particle destroys");
    CHECK(m2DestroyParticle(world, particle) == m2_errorStale, "a stale particle is m2_errorStale");
    CHECK(m2DestroyFluidVolume(world, volume) == m2_success, "a live volume destroys");
    CHECK(m2DestroyFluidVolume(world, volume) == m2_errorStale, "a stale volume is m2_errorStale");

    uint64_t misuse = m2World_GetCounters(world).misuse;
    CHECK(m2DestroyBody(world, body) == m2_success, "a live body destroys");
    m2ShapeId made = shape;
    CHECK(m2CreateCircleShape(world, body, &sd, &ball, &made) == m2_errorStale,
          "a shape on a stale body is m2_errorStale");
    CHECK(made.index1 == 0, "and leaves the null id");
    jd.bodyIdA = body;
    m2JointId madeJoint = joint;
    CHECK(m2CreateDistanceJoint(world, &jd, &madeJoint) == m2_errorStale,
          "a joint to a stale body is m2_errorStale");
    CHECK(madeJoint.index1 == 0, "and leaves the null id");
    CHECK(m2Body_GetPosition(world, body).x == 0.0 && m2Body_GetMass(world, body) == 0.0f,
          "a stale getter returns the zeroed value");
    CHECK(!m2Body_IsValid(world, body), "a validity query counts nothing");
    CHECK(m2World_GetCounters(world).misuse == misuse + 4, "each stale use counts once");
    m2DestroyWorld(world);
}

// A static body never moves: the velocity setters refuse it and count one
// misuse each, while a kinematic body takes them.
static void TestStaticBodiesHaveNoVelocity(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId wall = NewBody(world, &bd);
    bd.type = m2_kinematicBody;
    m2BodyId lift = NewBody(world, &bd);

    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Body_SetLinearVelocity(world, wall, (m2Vec2){1.0f, 0.0f});
    m2Body_SetAngularVelocity(world, wall, 1.0f);
    m2Body_SetTargetTransform(world, wall, (m2Pos2){1.0, 0.0}, (m2Rot){1.0f, 0.0f}, 1.0f / 60.0f);
    CHECK(m2World_GetCounters(world).misuse == misuse + 3, "each call on a static body counts");
    CHECK(m2Body_GetLinearVelocity(world, wall).x == 0.0f &&
              m2Body_GetAngularVelocity(world, wall) == 0.0f,
          "and leaves it still");

    m2Body_SetLinearVelocity(world, lift, (m2Vec2){0.0f, 1.0f});
    m2Body_SetAngularVelocity(world, lift, 0.5f);
    CHECK(m2World_GetCounters(world).misuse == misuse + 3, "a kinematic body takes them");
    CHECK(m2Body_GetLinearVelocity(world, lift).y == 1.0f, "and moves");

    m2BodyDef moving = m2DefaultBodyDef();
    moving.linearVelocity = (m2Vec2){0.0f, 1.0f};
    m2BodyId made = {7, 7};
    CHECK(m2CreateBody(world, &moving, &made) == m2_errorInvalid && made.index1 == 0,
          "a static def with a linear velocity is refused");
    moving.linearVelocity = (m2Vec2){0.0f, 0.0f};
    moving.angularVelocity = 1.0f;
    CHECK(m2CreateBody(world, &moving, &made) == m2_errorInvalid && made.index1 == 0,
          "and one with an angular velocity");
    CHECK(m2World_GetCounters(world).misuse == misuse + 5, "each counts one misuse");
    moving.type = m2_kinematicBody;
    CHECK(m2CreateBody(world, &moving, &made) == m2_success, "a kinematic def keeps its velocity");
    m2DestroyWorld(world);
}

// Every call that takes an id counts a stale one, the getters and void
// writers included; only the validity queries stay silent.
static void TestStaleIdsCountInEveryCall(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.fluidVolumeCapacity = 1;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bd);
    m2BodyId other = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.isSensor = true;
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeId sensor = NewCircleShape(world, body, &sd, &ball);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = body;
    jd.bodyIdB = other;
    m2JointId joint = NewDistanceJoint(world, &jd);
    m2Vec2 ledge[4] = {{-4.0f, 0.0f}, {-2.0f, 0.0f}, {2.0f, 0.0f}, {4.0f, 0.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = ledge;
    cd.count = 4;
    m2ChainId chain = NewChain(world, other, &cd);
    m2FluidVolumeDef vd = m2DefaultFluidVolumeDef();
    m2FluidVolumeId volume = NewFluidVolume(world, &vd);
    CHECK(m2DestroyFluidVolume(world, volume) == m2_success, "the volume goes");
    CHECK(m2DestroyJoint(world, joint) == m2_success, "the joint goes");
    CHECK(m2DestroyChain(world, chain) == m2_success, "the chain goes");
    CHECK(m2DestroyBody(world, body) == m2_success, "the body goes, and its sensor with it");

    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Body_SetAwake(world, body, true);
    m2Body_Disable(world, body);
    m2Body_Enable(world, body);
    CHECK(!m2Body_IsEnabled(world, body), "a stale body is not enabled");
    CHECK(m2Body_GetLinearDamping(world, body) == 0.0f, "nor damped");
    CHECK(m2Body_GetAngularDamping(world, body) == 0.0f, "in either way");
    CHECK(!m2Body_IsFixedRotation(world, body), "nor fixed");
    CHECK(!m2Body_GetMotionLocks(world, body).linearX, "nor locked");
    CHECK(!m2Body_IsSleepEnabled(world, body), "nor sleepy");
    CHECK(m2Shape_GetSensorOverlaps(world, sensor, NULL, 0) == 0, "a stale sensor lists nothing");
    CHECK(m2Shape_GetTangentSpeed(world, sensor) == 0.0f, "a stale shape reads zero");
    CHECK(m2Shape_GetFriction(world, sensor) == 0.0f, "for friction");
    CHECK(m2Shape_GetRestitution(world, sensor) == 0.0f, "and restitution");
    CHECK(m2Joint_GetReactionForce(world, joint) == 0.0f, "a stale joint carries no force");
    CHECK(m2Joint_GetReactionTorque(world, joint) == 0.0f, "nor torque");
    CHECK(m2FluidVolume_GetSurface(world, volume) == 0.0, "a stale volume has no surface");
    CHECK(m2FluidVolume_GetUserData(world, volume) == 0, "nor data");
    CHECK(m2Chain_GetShapes(world, chain, NULL, 0) == 0, "a stale chain has no links");
    CHECK(m2World_GetCounters(world).misuse == misuse + 18, "each of the 18 calls counts once");
    m2DestroyWorld(world);
}

static void TestEveryRefusalHasAReason(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 2;
    def.shapeCapacity = 2;
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId a = NewBody(world, &bd);
    m2BodyId b = NewBody(world, &bd);
    CHECK(m2Body_IsValid(world, a) && m2Body_IsValid(world, b), "two bodies fit");

    m2BodyId third = m2_nullBodyId;
    CHECK(m2CreateBody(world, &bd, &third) == m2_errorCapacity,
          "a third body is refused because the pool is full");
    CHECK(third.index1 == 0, "and the out id is the null id");

    uint64_t misuseBefore = m2World_GetCounters(world).misuse;
    DestroyBody(world, b);
    CHECK(!m2Body_IsValid(world, b), "a validity query on a stale id is not misuse");
    CHECK(m2World_GetCounters(world).misuse == misuseBefore, "so it is not counted");
    CHECK(m2DestroyBody(world, b) == m2_errorStale, "destroying a stale id is refused as stale");
    m2Body_SetLinearVelocity(world, b, (m2Vec2){1.0f, 0.0f});
    CHECK(m2World_GetCounters(world).misuse == misuseBefore + 2, "and both refusals count");

    m2BodyDef badDef = m2DefaultBodyDef();
    badDef.internalValue = 0;
    CHECK(m2CreateBody(world, &badDef, &third) == m2_errorInvalid,
          "an uninitialized def is refused as invalid");

    // Bad step arguments and stale joint reads refuse in every build;
    // they are caller input, not internal invariants.
    misuseBefore = m2World_GetCounters(world).misuse;
    uint64_t stepsBefore = m2World_GetStepCount(world);
    m2World_Step(world, 0.0f, 4);
    m2World_Step(world, 1.0f / 60.0f, 0);
    CHECK(m2World_GetStepCount(world) == stepsBefore, "a zero dt and zero substeps are refused");
    m2JointId staleJoint = {1, 1};
    CHECK(m2Joint_GetLength(world, staleJoint) == 0.0f, "a stale joint read returns zero");
    CHECK(m2World_GetCounters(world).misuse == misuseBefore + 3, "and each refusal counts once");
    m2DestroyWorld(world);
}

// Subsystem hashes: twins agree part by part, and a nudge to one
// subsystem splits exactly that part.
static void TestHashParts(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 4;
    def.particleCapacity = 32;
    m2World* a = NewWorld(&def);
    m2World* b = NewWorld(&def);
    for (int32_t w = 0; w < 2; ++w)
    {
        m2World* world = w == 0 ? a : b;
        m2BodyDef fd = m2DefaultBodyDef();
        fd.position = (m2Pos2){0.0, -0.5};
        m2ShapeDef fs = m2DefaultShapeDef();
        m2Polygon slab = m2MakeBox(5.0f, 0.5f);
        NewPolygonShape(world, NewBody(world, &fd), &fs, &slab);
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.0, 1.0};
        m2BodyId box = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon crate = m2MakeBox(0.4f, 0.4f);
        NewPolygonShape(world, box, &sd, &crate);
        EmitParticle(world, (m2Pos2){50.0, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
    }
    m2WorldHashParts pa = m2World_GetHashParts(a);
    m2WorldHashParts pb = m2World_GetHashParts(b);
    CHECK(pa.world == pb.world && pa.bodies == pb.bodies && pa.contacts == pb.contacts &&
              pa.joints == pb.joints && pa.particles == pb.particles,
          "twin worlds agree part by part");

    // Nudge one subsystem in one twin: only its part splits (before
    // any step folds the change into the rest).
    m2BodyId bodies[4];
    m2World_GetBodies(b, bodies, 4);
    m2Body_SetLinearVelocity(b, bodies[1], (m2Vec2){0.5f, 0.0f});
    pb = m2World_GetHashParts(b);
    CHECK(pa.bodies != pb.bodies, "a body nudge splits the bodies part");
    CHECK(pa.contacts == pb.contacts, "the contacts part holds");
    CHECK(pa.particles == pb.particles, "the particles part holds");
    CHECK(pa.world == pb.world, "the world part holds");

    m2ParticleId drops[4];
    m2World_GetParticles(a, drops, 4);
    m2Particle_SetVelocity(a, drops[0], (m2Vec2){0.0f, 1.0f});
    m2WorldHashParts pa2 = m2World_GetHashParts(a);
    CHECK(pa2.particles != pa.particles, "a particle nudge splits the particles part");
    CHECK(pa2.bodies == pa.bodies, "the bodies part holds against a particle nudge");

    // The read-only law: a parts storm moves nothing.
    uint64_t before = m2World_Hash(b);
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_GetHashParts(b);
    }
    CHECK(m2World_Hash(b) == before, "the parts storm leaves the world untouched");
    m2DestroyWorld(a);
    m2DestroyWorld(b);
}

// The destruction road: a spinning crate shatters into four
// triangles that carry its rigid velocity field, the wreck rolls
// back out of existence, and the whole thing replays from tape.
// Particle-free water: a light box floats, a dense box sinks, a
// flow current carries a drifter, the waterline moves at runtime,
// and the whole thing rolls back and replays from tape.
// Archimedes at equilibrium: a body of half the water's density floats
// with half its area under the surface. A symmetric shape then sits
// with its center on the waterline, whatever the shape: a circle, a
// capsule and a rounded box (whose rounded corners count at their true
// area).
static void TestFloatingDraft(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.fluidVolumeCapacity = 2;
    m2World* world = NewWorld(&def);
    m2FluidVolumeDef fd = m2DefaultFluidVolumeDef();
    fd.regionLower = (m2Pos2){-20.0, -20.0};
    fd.regionUpper = (m2Pos2){20.0, 20.0};
    fd.surface = 0.0;
    fd.density = 2.0f;
    fd.linearDrag = 3.0f;
    fd.angularDrag = 3.0f;
    NewFluidVolume(world, &fd);

    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 1.0f;
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.fixedRotation = true;
    bd.position = (m2Pos2){-4.0, 0.3};
    m2BodyId ball = NewBody(world, &bd);
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    NewCircleShape(world, ball, &sd, &circle);
    bd.position = (m2Pos2){0.0, 0.3};
    m2BodyId log = NewBody(world, &bd);
    m2Capsule capsule = {{-0.6f, 0.0f}, {0.6f, 0.0f}, 0.25f};
    NewCapsuleShape(world, log, &sd, &capsule);
    bd.position = (m2Pos2){4.0, 0.3};
    m2BodyId crate = NewBody(world, &bd);
    m2Vec2 core[4] = {{-0.25f, -0.15f}, {0.25f, -0.15f}, {0.25f, 0.15f}, {-0.25f, 0.15f}};
    m2Polygon rounded = MakePolygon(core, 4, 0.15f);
    NewPolygonShape(world, crate, &sd, &rounded);

    for (int32_t i = 0; i < 900; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(fabs(m2Body_GetPosition(world, ball).y) < 0.005,
          "a half-density circle floats half under");
    CHECK(fabs(m2Body_GetPosition(world, log).y) < 0.005,
          "a half-density capsule floats half under");
    CHECK(fabs(m2Body_GetPosition(world, crate).y) < 0.005,
          "a half-density rounded box floats half under, corners at their true area");
    m2DestroyWorld(world);
}

static void TestFluidVolume(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.fluidVolumeCapacity = 4;
    m2World* world = NewWorld(&def);
    m2FluidVolumeDef fd = m2DefaultFluidVolumeDef();
    fd.regionLower = (m2Pos2){-10.0, -10.0};
    fd.regionUpper = (m2Pos2){10.0, 10.0};
    fd.surface = 0.0;
    fd.density = 2.0f;
    fd.linearDrag = 2.0f;
    fd.userData = 42;
    m2FluidVolumeId vol = NewFluidVolume(world, &fd);
    CHECK(m2FluidVolume_IsValid(world, vol), "the volume is live");
    CHECK(m2FluidVolume_GetSurface(world, vol) == 0.0, "the surface reads back");
    CHECK(m2FluidVolume_GetUserData(world, vol) == 42, "userData reads back");

    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    m2BodyDef ld = m2DefaultBodyDef();
    ld.type = m2_dynamicBody;
    ld.position = (m2Pos2){-2.0, -3.0};
    m2BodyId light = NewBody(world, &ld);
    m2ShapeDef ls = m2DefaultShapeDef();
    ls.density = 0.5f;
    NewPolygonShape(world, light, &ls, &box);
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){2.0, -1.0};
    m2BodyId heavy = NewBody(world, &hd);
    m2ShapeDef hs = m2DefaultShapeDef();
    hs.density = 5.0f;
    NewPolygonShape(world, heavy, &hs, &box);

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    Snapshot(world, snap, size);

    for (int32_t i = 0; i < 400; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, light).y > -0.6 && m2Body_GetPosition(world, light).y < 0.6,
          "the light box floats at the waterline");
    CHECK(m2Body_GetPosition(world, heavy).y < -3.0, "the dense box sinks past the light one");

    // Rollback: the boxes return to their launch and re-float exactly.
    uint64_t future = m2World_Hash(world);
    CHECK(Restore(world, snap, size), "the pre-float snapshot restores");
    for (int32_t i = 0; i < 400; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(world) == future, "buoyancy replays bit for bit after rollback");

    // Runtime waterline: drain it below the boxes and they fall.
    m2FluidVolume_SetSurface(world, vol, -20.0);
    CHECK(m2FluidVolume_GetSurface(world, vol) == -20.0, "the drained surface reads back");
    double lightBefore = m2Body_GetPosition(world, light).y;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, light).y < lightBefore - 0.5, "a drained tank drops its cargo");
    free(snap);
    m2DestroyWorld(world);

    // Tape: record a volume plus a float, replay onto identical bits.
    m2World* rec = NewWorld(&def);
    int32_t cap = 1 << 20;
    void* tape = malloc((size_t)cap);
    StartJournal(rec, tape, cap);
    m2FluidVolumeDef rfd = m2DefaultFluidVolumeDef();
    rfd.regionLower = (m2Pos2){-10.0, -10.0};
    rfd.regionUpper = (m2Pos2){10.0, 10.0};
    rfd.surface = 0.0;
    m2FluidVolumeId rv = NewFluidVolume(rec, &rfd);
    m2BodyDef rb = m2DefaultBodyDef();
    rb.type = m2_dynamicBody;
    rb.position = (m2Pos2){0.0, -2.0};
    m2BodyId rbody = NewBody(rec, &rb);
    m2ShapeDef rs = m2DefaultShapeDef();
    rs.density = 0.5f;
    NewPolygonShape(rec, rbody, &rs, &box);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(rec, 1.0f / 60.0f, 4);
    }
    m2FluidVolume_SetSurface(rec, rv, 1.0);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(rec, 1.0f / 60.0f, 4);
    }
    int32_t used = StopJournal(rec);
    uint64_t recorded = m2World_Hash(rec);
    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, used), "the buoyancy tape replays");
    CHECK(m2World_Hash(fresh) == recorded, "the recorded float replays bit for bit");
    m2DestroyWorld(fresh);
    m2DestroyWorld(rec);
    free(tape);
}

static void TestShatterBody(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 4;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){10.0, 5.0};
    bd.linearVelocity = (m2Vec2){2.0f, 1.0f};
    bd.angularVelocity = 3.0f;
    m2BodyId crate = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.9f;
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, crate, &sd, &box);

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    Snapshot(world, snap, size);

    // Four triangles fanned around the center.
    m2Vec2 c[4] = {{-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
    m2Polygon shards[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        m2Vec2 tri[3] = {{0.0f, 0.0f}, c[i], c[(i + 1) % 4]};
        shards[i] = MakePolygon(tri, 3, 0.0f);
        CHECK(shards[i].count == 3, "each shard validates");
    }
    m2BodyId out[4];
    int32_t made = 0;
    CHECK(m2World_ShatterBody(world, crate, shards, 4, out, 4, &made) == m2_success,
          "the crate shatters");
    CHECK(made == 4, "into four");
    CHECK(!m2Body_IsValid(world, crate), "the parent is gone");
    float mass = 0.0f;
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m2Body_IsValid(world, out[i]), "every shard lives");
        CHECK(m2Body_GetAngularVelocity(world, out[i]) == 3.0f, "shards inherit the spin");
        mass += m2Body_GetMass(world, out[i]);
        m2ShapeId shardShapes[2];
        CHECK(m2Body_GetShapes(world, out[i], shardShapes, 2) == 1, "one shape per shard");
        CHECK(m2Shape_GetFriction(world, shardShapes[0]) == 0.9f,
              "shards wear the parent's material");
        // The rigid field: v_piece = v + w x r about the parent COM.
        m2Pos2 com = m2Body_GetWorldCenterOfMass(world, out[i]);
        float rx = (float)(com.x - 10.0);
        float ry = (float)(com.y - 5.0);
        m2Vec2 v = m2Body_GetLinearVelocity(world, out[i]);
        CHECK(v.x > 2.0f - 3.0f * ry - 0.001f && v.x < 2.0f - 3.0f * ry + 0.001f,
              "the shard rides the field in x");
        CHECK(v.y > 1.0f + 3.0f * rx - 0.001f && v.y < 1.0f + 3.0f * rx + 0.001f,
              "the shard rides the field in y");
    }
    CHECK(mass > 0.99f && mass < 1.01f, "the shards keep the parent's mass");

    // Step the wreck, then roll it back out of existence.
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(Restore(world, snap, size), "the pre-wreck snapshot restores");
    CHECK(m2Body_IsValid(world, crate), "the crate is whole again");
    CHECK(!m2Body_IsValid(world, out[0]), "the shards never happened");

    // The tape: record a shatter, replay it onto the same bits.
    int32_t cap = 1 << 20;
    void* tape = malloc((size_t)cap);
    CHECK(StartJournal(world, tape, cap), "the tape starts");
    made = ShatterBody(world, crate, shards, 4, NULL, 0);
    CHECK(made == 4, "the recorded shatter lands");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t used = StopJournal(world);
    uint64_t recorded = m2World_Hash(world);
    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, used), "the shatter tape replays");
    CHECK(m2World_Hash(fresh) == recorded, "the wreck replays bit for bit");
    m2DestroyWorld(fresh);
    free(tape);
    free(snap);

    // All or nothing: a world too full to seat the pieces refuses
    // without touching anything.
    m2WorldDef tight = m2DefaultWorldDef();
    tight.bodyCapacity = 2;
    tight.shapeCapacity = 4;
    tight.jointCapacity = 4;
    m2World* small = NewWorld(&tight);
    m2BodyDef sb = m2DefaultBodyDef();
    sb.type = m2_dynamicBody;
    sb.position = (m2Pos2){0.0, 1.0};
    m2BodyId lone = NewBody(small, &sb);
    m2ShapeDef ss = m2DefaultShapeDef();
    NewPolygonShape(small, lone, &ss, &box);
    uint64_t before = m2World_Hash(small);
    uint64_t misuseBefore = m2World_GetCounters(small).misuse;
    m2BodyId kept[2] = {lone, lone};
    int32_t count = 7;
    CHECK(m2World_ShatterBody(small, lone, shards, 4, kept, 2, &count) == m2_errorCapacity,
          "a full world refuses with m2_errorCapacity");
    CHECK(count == 0 && kept[0].index1 == 0 && kept[1].index1 == 0, "and clears the outputs");
    CHECK(m2World_GetCounters(small).misuse == misuseBefore, "a full pool is not misuse");
    m2Polygon bent = shards[0];
    bent.count = M2_MAX_POLYGON_VERTICES + 1;
    CHECK(m2World_ShatterBody(small, lone, &bent, 1, NULL, 0, &count) == m2_errorInvalid,
          "a malformed piece refuses with m2_errorInvalid");
    CHECK(m2World_GetCounters(small).misuse == misuseBefore + 1, "as misuse");
    m2BodyDef fixed = m2DefaultBodyDef();
    m2BodyId wall = NewBody(small, &fixed);
    CHECK(m2World_ShatterBody(small, wall, shards, 1, NULL, 0, &count) == m2_errorInvalid,
          "a static body does not shatter");
    DestroyBody(small, wall);
    CHECK(m2World_ShatterBody(small, wall, shards, 1, NULL, 0, &count) == m2_errorStale,
          "a stale body is m2_errorStale");
    CHECK(m2World_ShatterBody(small, lone, shards, 1, NULL, 0, NULL) == m2_errorInvalid,
          "and so is a missing count");
    count = 7;
    CHECK(m2World_ShatterBody(small, lone, shards, 1, NULL, -1, &count) == m2_errorInvalid &&
              count == 0,
          "a negative capacity is refused and clears the count");
    count = 7;
    CHECK(m2World_ShatterBody(small, lone, shards, 1, NULL, 2, &count) == m2_errorInvalid &&
              count == 0,
          "a null bodiesOut with room is refused and clears the count");
    CHECK(m2World_GetCounters(small).misuse == misuseBefore + 6, "each a misuse");
    CHECK(m2Body_IsValid(small, lone), "the refused parent is untouched");
    CHECK(m2World_Hash(small) == before, "a refusal moves no bits");
    m2DestroyWorld(small);
    m2DestroyWorld(world);
}

static void TestDominance(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.2f;
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    // A heavy runaway crate slams into a plain twin and into a
    // dominant twin; only the plain one gets shoved.
    m2BodyId targets[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 12.0;
        m2BodyDef td = m2DefaultBodyDef();
        td.type = m2_dynamicBody;
        td.position = (m2Pos2){x, 0.45};
        if (i == 1)
        {
            td.dominance = 5;
        }
        targets[i] = NewBody(world, &td);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.friction = 0.2f;
        m2Polygon unit = m2MakeBox(0.4f, 0.4f);
        NewPolygonShape(world, targets[i], &sd, &unit);

        m2BodyDef rd = m2DefaultBodyDef();
        rd.type = m2_dynamicBody;
        rd.position = (m2Pos2){x - 3.0, 0.45};
        rd.linearVelocity = (m2Vec2){8.0f, 0.0f};
        m2BodyId ram = NewBody(world, &rd);
        m2ShapeDef rs = m2DefaultShapeDef();
        rs.density = 4.0f;
        rs.friction = 0.2f;
        m2Polygon heavy = m2MakeBox(0.5f, 0.4f);
        NewPolygonShape(world, ram, &rs, &heavy);
    }
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double plainMoved = m2Body_GetPosition(world, targets[0]).x - 0.0;
    double bossMoved = m2Body_GetPosition(world, targets[1]).x - 12.0;
    CHECK(plainMoved > 0.5, "the plain crate gets shoved");
    CHECK(bossMoved < 0.05 && bossMoved > -0.05, "the dominant crate does not budge");
    CHECK(m2Body_GetDominance(world, targets[1]) == 5, "dominance reads back");

    // The dominant crate still falls (gravity is not a contact) and
    // still rests on the floor (statics outrank it).
    m2Body_SetTransform(world, targets[1], (m2Pos2){12.0, 3.0}, (m2Rot){1.0f, 0.0f});
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, targets[1]).y < 0.6, "the boss still lands on the floor");

    // Runtime demotion: drop the crown and the crate shoves again.
    m2Body_SetDominance(world, targets[1], 0);
    m2BodyDef rd2 = m2DefaultBodyDef();
    rd2.type = m2_dynamicBody;
    rd2.position = (m2Pos2){9.0, 0.45};
    rd2.linearVelocity = (m2Vec2){8.0f, 0.0f};
    m2BodyId ram2 = NewBody(world, &rd2);
    m2ShapeDef rs2 = m2DefaultShapeDef();
    rs2.density = 4.0f;
    m2Polygon heavy2 = m2MakeBox(0.5f, 0.4f);
    NewPolygonShape(world, ram2, &rs2, &heavy2);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, targets[1]).x > 12.3, "demoted, it gets shoved like anyone");

    m2DestroyWorld(world);
}

static m2BodyId LockedBody(m2World* world, m2MotionLocks locks)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){5.0, 10.0};
    bd.linearVelocity = (m2Vec2){20.0f, 0.0f}; // free y falls under gravity
    bd.angularVelocity = 8.0f;
    bd.motionLocks = locks;
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, body, &sd, &box);
    return body;
}

static void TestMotionLocks(void)
{
    // A locked axis holds still: the velocity is zeroed each substep so
    // gravity, contacts and the initial shove cannot move the body along
    // it, while the free axes behave normally.
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyId xLock = LockedBody(world, (m2MotionLocks){true, false, false});
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 p = m2Body_GetPosition(world, xLock);
    CHECK(p.x == 5.0, "a linear-X lock holds the x position exactly");
    CHECK(p.y < 8.0, "the free y axis still falls under gravity");
    CHECK(m2Body_GetLinearVelocity(world, xLock).x == 0.0f, "the locked-x velocity is zero");
    m2MotionLocks got = m2Body_GetMotionLocks(world, xLock);
    CHECK(got.linearX && !got.linearY && !got.angularZ, "the locks read back as set");
    m2DestroyWorld(world);

    // angularZ is the fixed-rotation lock: it aliases m2Body_IsFixedRotation.
    world = NewWorld(&def);
    m2BodyId aLock = LockedBody(world, (m2MotionLocks){false, false, true});
    CHECK(m2Body_IsFixedRotation(world, aLock), "angularZ lock reads as fixed rotation");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetAngularVelocity(world, aLock) == 0.0f, "the angular lock stops the spin");
    m2DestroyWorld(world);

    // Runtime setter through the journal, plus a snapshot round-trip on the
    // Snapshot round trip and a worker-count twin: bit-exact.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef d2 = m2DefaultWorldDef();
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&d2, &s_twinWorkers[wc]);
        m2World* w = NewWorld(&d2);
        m2BodyId b = LockedBody(w, (m2MotionLocks){false, false, false});
        for (int32_t i = 0; i < 20; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        m2Body_SetMotionLocks(w, b,
                              (m2MotionLocks){true, true, false}); // lock both linear axes now
        int32_t size = m2World_SnapshotSize(w);
        void* snap = malloc((size_t)size);
        CHECK(Snapshot(w, snap, size) == size, "the snapshot fits");
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        uint64_t after = m2World_Hash(w);
        CHECK(Restore(w, snap, size), "the snapshot restores");
        // The locks survive the round-trip, so the frozen body stays frozen.
        CHECK(m2Body_GetMotionLocks(w, b).linearX && m2Body_GetMotionLocks(w, b).linearY,
              "the restored body keeps its locks");
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(w) == after, "a locked-body timeline replays bit for bit");
        hashes[wc] = after;
        free(snap);
        m2DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "motion locks are worker-count deterministic");
}

static m2BodyId WindBody(m2World* world, m2Pos2 pos)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = pos;
    bd.gravityScale = 0.0f; // isolate wind from gravity in the test
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, body, &sd, &box);
    return body;
}

static void TestWind(void)
{
    // With gravity off, a still body in a +x wind drifts downwind and its
    // speed approaches the wind speed without passing it. Wind set before
    // the first step, so the body stays awake and feels it (asleep bodies
    // skip wind by design).
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2BodyId body = WindBody(world, (m2Pos2){0.0, 0.0});
    m2World_SetWind(world, (m2Vec2){10.0f, 0.0f}, 2.0f);
    m2Vec2 gv;
    float gd;
    m2World_GetWind(world, &gv, &gd);
    CHECK(gv.x == 10.0f && gv.y == 0.0f && gd == 2.0f, "wind reads back as set");
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Vec2 vel = m2Body_GetLinearVelocity(world, body);
    CHECK(m2Body_GetPosition(world, body).x > 0.5, "the body drifts downwind");
    CHECK(vel.x > 0.0f && vel.x < 10.0f, "it accelerates toward but not past the wind speed");
    CHECK(vel.y > -1.0e-4f && vel.y < 1.0e-4f, "a pure +x wind adds no y motion");
    m2DestroyWorld(world);

    // Wind off (the default) is a true no-op: a still body never drifts.
    world = NewWorld(&def);
    m2BodyId calm = WindBody(world, (m2Pos2){3.0, 0.0});
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, calm).x == 3.0, "no wind, no drift");
    m2DestroyWorld(world);

    // The snapshot round trip carries the wind; the worker-count twin is
    // bit-exact.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef d2 = m2DefaultWorldDef();
        d2.gravity = (m2Vec2){0.0f, 0.0f};
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&d2, &s_twinWorkers[wc]);
        m2World* w = NewWorld(&d2);
        m2BodyId b = WindBody(w, (m2Pos2){0.0, 0.0});
        (void)b;
        m2World_SetWind(w, (m2Vec2){0.0f, -8.0f}, 1.5f);
        for (int32_t i = 0; i < 20; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        int32_t size = m2World_SnapshotSize(w);
        void* snap = malloc((size_t)size);
        CHECK(Snapshot(w, snap, size) == size, "the snapshot fits");
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        uint64_t after = m2World_Hash(w);
        CHECK(Restore(w, snap, size), "the snapshot restores");
        m2Vec2 rv;
        float rd;
        m2World_GetWind(w, &rv, &rd);
        CHECK(rv.y == -8.0f && rd == 1.5f, "the restored world keeps its wind");
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(w) == after, "a windy timeline replays bit for bit");
        hashes[wc] = after;
        free(snap);
        m2DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "wind is worker-count deterministic");
}

int main(void)
{
    TestCreateWorldOutOfMemory();
    TestInfiniteInputIsRefused();
    TestDefsMatchWhatARestoreAccepts();
    TestSubstepCap();
    TestRestoreRefusesANonUnitRotation();
    TestEveryRefusalHasAReason();
    TestHostCodeCannotWrite();
    TestAllocatorCannotWrite();
    TestCorruptInputIsNotMisuse();
    TestHostCodeMistakesCountMidReplay();
    TestStaleIdsInStatusCalls();
    TestStaleIdsCountInEveryCall();
    TestStaticBodiesHaveNoVelocity();
    TestRuntimeGravity();
    TestDominance();
    TestShatterBody();
    TestFluidVolume();
    TestFloatingDraft();
    TestHashParts();
    TestValidateAndCounters();
    TestLeftoverBasket();
    TestSmallBasket();
    TestDormancyMassAndExplosions();
    TestBodyDynamicsPack();
    TestMirrorRebuild();
    TestEnumerationWalk();
    TestDestroyWakesSleepers();
    TestSetType();
    TestTeleport();
    TestKineticEnergy();
    TestCountersAndProfile();
    TestRollbackIdentity();
    TestIdSemantics();
    TestDestroyNullWorld();
    TestMotionLocks();
    TestWind();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
