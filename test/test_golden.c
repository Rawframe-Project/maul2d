// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The golden scenes: a few fixed sessions whose final hashes are part of
// the specification, so every platform, compiler, backend and thread
// count must reach the same bits. Each scene covers several subsystems at
// once, and the benchmark scenes (bench/scenes.c) are checked the same
// way. The committed values come from test/hashes.txt, which
// test/CMakeLists.txt turns into the M2_GOLDEN_<NAME> definitions; a
// scene prints its line, so a deliberate change of results is one
// regeneration:
//
//   out/<preset>/test_golden | grep -E '^M2_[A-Z0-9_]+_HASH=' > test/hashes.txt
//
// The scenes then run again on a hostile host, each under another of the
// floating-point settings test_fenv.h plays (every rounding mode, with
// flush-to-zero and denormals-are-zero off and on): the library computes
// in its own setting, so they reach the same values, and the host's
// setting is as the host left it after each scene.

#include "helpers.h"
#include "test_fenv.h"
#include "test_harness.h"
#include "test_task_pool.h"
#include "walk.h"

#include "../bench/scenes.h"
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A value missing from test/hashes.txt reads as zero, so its scene fails
// and prints the line to commit.
#ifndef M2_GOLDEN_STACK
#define M2_GOLDEN_STACK 0
#endif
#ifndef M2_GOLDEN_CONTACTS
#define M2_GOLDEN_CONTACTS 0
#endif
#ifndef M2_GOLDEN_FLUID
#define M2_GOLDEN_FLUID 0
#endif
#ifndef M2_GOLDEN_ROLLBACK
#define M2_GOLDEN_ROLLBACK 0
#endif
#ifndef M2_GOLDEN_JOURNAL
#define M2_GOLDEN_JOURNAL 0
#endif
#ifndef M2_GOLDEN_JOURNAL_TAPE
#define M2_GOLDEN_JOURNAL_TAPE 0
#endif
#ifndef M2_GOLDEN_ROLLBACK_SNAPSHOT
#define M2_GOLDEN_ROLLBACK_SNAPSHOT 0
#endif
#ifndef M2_GOLDEN_BENCH_PYRAMID15
#define M2_GOLDEN_BENCH_PYRAMID15 0
#endif
#ifndef M2_GOLDEN_BENCH_PYRAMID30
#define M2_GOLDEN_BENCH_PYRAMID30 0
#endif
#ifndef M2_GOLDEN_BENCH_CHAIN30
#define M2_GOLDEN_BENCH_CHAIN30 0
#endif
#ifndef M2_GOLDEN_BENCH_CHAIN2000
#define M2_GOLDEN_BENCH_CHAIN2000 0
#endif
#ifndef M2_GOLDEN_BENCH_JOINTFARM
#define M2_GOLDEN_BENCH_JOINTFARM 0
#endif
#ifndef M2_GOLDEN_BENCH_WATER
#define M2_GOLDEN_BENCH_WATER 0
#endif
#ifndef M2_GOLDEN_LINES
#define M2_GOLDEN_LINES 0
#endif

#define DT (1.0f / 60.0f)

static uint64_t Mix(uint64_t hash, uint64_t value)
{
    return m2Hash64(hash, &value, (int32_t)sizeof(value));
}

static int32_t s_expected = 0;

// The host setting of the second pass, -1 in the first, which alone
// prints and counts the lines.
static int s_host = -1;

static void Expect(const char* name, uint64_t actual, uint64_t expected)
{
    if (s_host >= 0)
    {
        if (actual != expected)
        {
            printf("FAIL: %s moved to %016llx on host setting %d\n", name,
                   (unsigned long long)actual, s_host);
            s_failures += 1;
        }
        return;
    }
    s_expected += 1;
    printf("M2_%s_HASH=%016llx\n", name, (unsigned long long)actual);
    if (expected == 0)
    {
        printf("FAIL: %s has no line in test/hashes.txt\n", name);
        s_failures += 1;
    }
    else if (actual != expected)
    {
        printf("FAIL: %s moved from %016llx; if the change is meant, regenerate test/hashes.txt\n",
               name, (unsigned long long)expected);
        s_failures += 1;
    }
}

static int32_t s_workers = 4;

static m2WorldDef WorldDef(int32_t bodies, int32_t joints, int32_t particles, bool threaded)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = bodies;
    def.shapeCapacity = 2 * bodies;
    def.jointCapacity = joints;
    def.particleCapacity = particles;
    def.fluidVolumeCapacity = 2;
    if (threaded)
    {
        def.enqueueTask = TpEnqueue;
        def.finishTask = TpFinish;
        def.userTaskContext = &s_workers;
    }
    return def;
}

static m2BodyId AddStatic(m2World* world, double x, double y, float halfWidth, float halfHeight)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(halfWidth, halfHeight);
    NewPolygonShape(world, body, &sd, &box);
    return body;
}

static m2BodyId AddBox(m2World* world, double x, double y, float halfSize)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.6f;
    m2Polygon box = m2MakeBox(halfSize, halfSize);
    NewPolygonShape(world, body, &sd, &box);
    return body;
}

// STACK: a pyramid that settles and sleeps, a revolute chain, a sprung
// two-wheeler on wheel joints, a prismatic elevator, a welded beam and a
// crate swinging from the mouse that grips its corner, far from the
// origin, where positions need their 64 bits.
static void BuildStack(m2World* world)
{
    const double ox = 1.0e6;
    AddStatic(world, ox, -1.0, 40.0f, 1.0f);
    for (int32_t row = 0; row < 10; ++row)
    {
        for (int32_t i = row; i < 10; ++i)
        {
            AddBox(world, ox - 12.0 + 1.05 * (double)i - 0.525 * (double)row,
                   0.5 + 1.0 * (double)row, 0.5f);
        }
    }

    m2BodyId anchor = AddStatic(world, ox + 2.0, 12.0, 0.2f, 0.2f);
    m2BodyId prev = anchor;
    for (int32_t i = 0; i < 12; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){ox + 2.5 + (double)i, 12.0};
        m2BodyId link = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Capsule capsule = {{-0.5f, 0.0f}, {0.5f, 0.0f}, 0.12f};
        NewCapsuleShape(world, link, &sd, &capsule);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = prev;
        jd.bodyIdB = link;
        jd.localAnchorA = i == 0 ? (m2Vec2){0.0f, 0.0f} : (m2Vec2){0.5f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        NewRevoluteJoint(world, &jd);
        prev = link;
    }

    m2BodyId chassis = AddBox(world, ox + 20.0, 1.2, 0.4f);
    for (int32_t side = -1; side <= 1; side += 2)
    {
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){ox + 20.0 + 0.8 * (double)side, 0.4};
        m2BodyId wheel = NewBody(world, &wd);
        m2ShapeDef ws = m2DefaultShapeDef();
        ws.friction = 0.9f;
        m2Circle tire = {{0.0f, 0.0f}, 0.35f};
        NewCircleShape(world, wheel, &ws, &tire);
        m2WheelJointDef jd = m2DefaultWheelJointDef();
        jd.bodyIdA = chassis;
        jd.bodyIdB = wheel;
        jd.localAnchorA = (m2Vec2){0.8f * (float)side, -0.8f};
        jd.localAxisA = (m2Vec2){0.0f, 1.0f};
        jd.enableMotor = true;
        jd.motorSpeed = -6.0f;
        jd.maxMotorEffort = 8.0f;
        NewWheelJoint(world, &jd);
    }

    m2BodyId shaft = AddStatic(world, ox - 30.0, 0.0, 0.1f, 0.1f);
    m2BodyId cabin = AddBox(world, ox - 30.0, 1.0, 0.6f);
    m2PrismaticJointDef pd = m2DefaultPrismaticJointDef();
    pd.bodyIdA = shaft;
    pd.bodyIdB = cabin;
    pd.localAxisA = (m2Vec2){0.0f, 1.0f};
    pd.enableMotor = true;
    pd.motorSpeed = 1.5f;
    pd.maxMotorEffort = 400.0f;
    pd.enableLimit = true;
    pd.lowerTranslation = 0.0f;
    pd.upperTranslation = 4.0f;
    NewPrismaticJoint(world, &pd);

    m2BodyId post = AddStatic(world, ox + 30.0, 3.0, 0.2f, 0.2f);
    m2BodyId beam = AddBox(world, ox + 31.0, 3.0, 0.3f);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = post;
    weld.bodyIdB = beam;
    weld.localAnchorA = (m2Vec2){0.5f, 0.0f};
    weld.localAnchorB = (m2Vec2){-0.5f, 0.0f};
    NewWeldJoint(world, &weld);

    m2BodyId crate = AddBox(world, ox + 34.0, 5.0, 0.4f);
    m2MouseJointDef grip = m2DefaultMouseJointDef();
    grip.bodyIdA = post;
    grip.bodyIdB = crate;
    grip.target = (m2Pos2){ox + 34.4, 5.4};
    grip.maxForce = 200.0f;
    NewMouseJoint(world, &grip);
}

static uint64_t RunStack(bool threaded)
{
    m2WorldDef def = WorldDef(128, 32, 0, threaded);
    m2World* world = NewWorld(&def);
    BuildStack(world);
    uint64_t hash = M2_HASH_INIT;
    for (int32_t i = 0; i < 360; ++i)
    {
        m2World_Step(world, DT, 4);
        if (i % 60 == 59)
        {
            hash = Mix(hash, m2World_Hash(world));
        }
    }
    m2DestroyWorld(world);
    return hash;
}

static void SceneStack(void)
{
    uint64_t serial = RunStack(false);
    CHECK(RunStack(true) == serial, "STACK: four task-system workers reach the serial bits");
    Expect("STACK", serial, M2_GOLDEN_STACK);
}

// CONTACTS: bullets at a thin wall, boxes sliding over a one-way chain
// floor, a sensor zone, a filtered pair, a bouncy and a sliding body, and
// a conveyor; the contact and sensor event streams go into the hash with
// the world state.
static void BuildContacts(m2World* world)
{
    AddStatic(world, 0.0, -1.0, 30.0f, 1.0f);
    AddStatic(world, 12.0, 3.0, 0.05f, 3.0f); // the thin wall
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.isBullet = i % 2 == 0;
        bd.position = (m2Pos2){2.0, 1.0 + 1.2 * (double)i};
        bd.linearVelocity = (m2Vec2){300.0f, 0.0f};
        bd.gravityScale = 0.0f;
        m2BodyId bullet = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle ball = {{0.0f, 0.0f}, 0.1f};
        NewCircleShape(world, bullet, &sd, &ball);
    }

    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){-20.0, 2.0};
    m2BodyId chainBody = NewBody(world, &cd);
    m2Vec2 points[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        points[i] = (m2Vec2){8.0f - 2.0f * (float)i, 0.0f};
    }
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = points;
    chain.count = 8;
    NewChain(world, chainBody, &chain);
    for (int32_t i = 0; i < 3; ++i)
    {
        m2BodyId box = AddBox(world, -26.0 + 2.0 * (double)i, 3.0, 0.3f);
        m2Body_SetLinearVelocity(world, box, (m2Vec2){4.0f, 0.0f});
    }
    m2BodyDef riser = m2DefaultBodyDef();
    riser.type = m2_dynamicBody;
    riser.position = (m2Pos2){-17.0, 1.0};
    riser.linearVelocity = (m2Vec2){0.0f, 9.0f};
    m2BodyId ghost = NewBody(world, &riser);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Circle pebble = {{0.0f, 0.0f}, 0.2f};
    NewCircleShape(world, ghost, &gs, &pebble);

    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){5.0, 2.0};
    m2BodyId zone = NewBody(world, &zd);
    m2ShapeDef zs = m2DefaultShapeDef();
    zs.isSensor = true;
    m2Polygon area = m2MakeBox(1.5f, 1.5f);
    NewPolygonShape(world, zone, &zs, &area);
    for (int32_t i = 0; i < 3; ++i)
    {
        AddBox(world, 4.0 + 1.0 * (double)i, 6.0 + 1.5 * (double)i, 0.25f);
    }

    m2BodyDef fd = m2DefaultBodyDef();
    fd.type = m2_dynamicBody;
    fd.position = (m2Pos2){-4.0, 1.0};
    m2BodyId lower = NewBody(world, &fd);
    fd.position = (m2Pos2){-4.0, 3.0};
    m2BodyId upper = NewBody(world, &fd);
    m2ShapeDef a = m2DefaultShapeDef();
    a.categoryBits = 2;
    a.maskBits = ~(uint64_t)4;
    m2ShapeDef b = m2DefaultShapeDef();
    b.categoryBits = 4;
    m2Polygon crate = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, lower, &a, &crate);
    NewPolygonShape(world, upper, &b, &crate);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-8.0, 4.0};
    m2BodyId bouncer = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.restitution = 0.8f;
    m2Circle ball = {{0.0f, 0.0f}, 0.4f};
    NewCircleShape(world, bouncer, &bs, &ball);
    bd.position = (m2Pos2){-12.0, 0.5};
    bd.linearVelocity = (m2Vec2){6.0f, 0.0f};
    m2BodyId slider = NewBody(world, &bd);
    m2ShapeDef ss = m2DefaultShapeDef();
    ss.friction = 0.05f;
    m2Polygon flat = m2MakeBox(0.5f, 0.25f);
    NewPolygonShape(world, slider, &ss, &flat);

    m2BodyDef beltDef = m2DefaultBodyDef();
    beltDef.position = (m2Pos2){20.0, 0.25};
    m2BodyId belt = NewBody(world, &beltDef);
    m2ShapeDef beltShape = m2DefaultShapeDef();
    beltShape.tangentSpeed = 2.0f;
    m2Polygon strip = m2MakeBox(4.0f, 0.25f);
    NewPolygonShape(world, belt, &beltShape, &strip);
    AddBox(world, 18.0, 1.0, 0.3f);
}

static uint64_t HashShapeId(uint64_t hash, m2ShapeId id)
{
    hash = Mix(hash, (uint64_t)(uint32_t)id.index1);
    return Mix(hash, id.generation);
}

static void SceneContacts(void)
{
    m2WorldDef def = WorldDef(64, 8, 0, false);
    m2World* world = NewWorld(&def);
    BuildContacts(world);
    uint64_t hash = M2_HASH_INIT;
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, DT, 4);
        m2ContactEvents contacts = m2World_GetContactEvents(world);
        for (int32_t e = 0; e < contacts.beginCount; ++e)
        {
            const m2ContactBeginEvent* begin = &contacts.beginEvents[e];
            hash = HashShapeId(HashShapeId(hash, begin->shapeIdA), begin->shapeIdB);
            hash = m2Hash64(hash, &begin->approachSpeed, (int32_t)sizeof(float));
            hash = m2Hash64(hash, &begin->normal, (int32_t)sizeof(m2Vec2));
        }
        for (int32_t e = 0; e < contacts.endCount; ++e)
        {
            hash = HashShapeId(HashShapeId(hash, contacts.endEvents[e].shapeIdA),
                               contacts.endEvents[e].shapeIdB);
        }
        m2SensorEvents sensors = m2World_GetSensorEvents(world);
        for (int32_t e = 0; e < sensors.beginCount; ++e)
        {
            hash = HashShapeId(HashShapeId(hash, sensors.beginEvents[e].sensorShapeId),
                               sensors.beginEvents[e].visitorShapeId);
        }
        for (int32_t e = 0; e < sensors.endCount; ++e)
        {
            hash = HashShapeId(HashShapeId(hash, sensors.endEvents[e].sensorShapeId),
                               sensors.endEvents[e].visitorShapeId);
        }
    }
    hash = Mix(hash, m2World_Hash(world));
    m2DestroyWorld(world);
    Expect("CONTACTS", hash, M2_GOLDEN_CONTACTS);
}

// FLUID: water poured into a basin, a tensile droplet, a jelly block
// held together by springs and triads, a buoyancy volume carrying a
// light crate and a heavy one, and a steady wind.
static void BuildFluid(m2World* world)
{
    AddStatic(world, 0.0, -0.5, 8.0f, 0.5f);
    AddStatic(world, -6.0, 3.0, 0.25f, 3.0f);
    AddStatic(world, 2.0, 3.0, 0.25f, 3.0f);
    for (int32_t i = 0; i < 300; ++i)
    {
        m2Pos2 p = {-5.0 + 0.15 * (double)(i % 30), 0.5 + 0.15 * (double)(i / 30)};
        EmitParticle(world, p, (m2Vec2){0.0f, 0.0f}, m2_waterParticle);
    }
    for (int32_t i = 0; i < 12; ++i)
    {
        m2Pos2 p = {-1.0 + 0.08 * (double)(i % 4), 5.0 + 0.08 * (double)(i / 4)};
        EmitParticle(world, p, (m2Vec2){0.0f, 0.0f}, m2_tensileParticle);
    }
    m2Polygon block = m2MakeBox(0.6f, 0.6f);
    FillParticles(world, &block, (m2Pos2){-3.0, 4.5}, (m2Vec2){0.0f, 0.0f},
                  m2_springParticle | m2_elasticParticle);

    AddStatic(world, 12.0, -0.5, 4.0f, 0.5f);
    m2FluidVolumeDef pool = m2DefaultFluidVolumeDef();
    pool.regionLower = (m2Pos2){8.0, 0.0};
    pool.regionUpper = (m2Pos2){16.0, 4.0};
    pool.surface = 2.0;
    pool.density = 2.0f;
    pool.flow = (m2Vec2){0.4f, 0.0f};
    NewFluidVolume(world, &pool);
    AddBox(world, 10.0, 3.5, 0.4f);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){13.0, 3.5};
    m2BodyId heavy = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 6.0f;
    m2Polygon box = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, heavy, &sd, &box);
    m2World_SetWind(world, (m2Vec2){1.5f, 0.0f}, 0.05f);
}

static uint64_t RunFluid(bool threaded)
{
    m2WorldDef def = WorldDef(32, 4, 1024, threaded);
    def.particleRadius = 0.06f;
    m2World* world = NewWorld(&def);
    BuildFluid(world);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, DT, 4);
    }
    uint64_t hash = Mix(m2World_Hash(world), (uint64_t)m2World_GetParticleCount(world));
    m2DestroyWorld(world);
    return hash;
}

static void SceneFluid(void)
{
    uint64_t serial = RunFluid(false);
    CHECK(RunFluid(true) == serial, "FLUID: four task-system workers reach the serial bits");
    Expect("FLUID", serial, M2_GOLDEN_FLUID);
}

// ROLLBACK: the stack and the fluid in one world, snapshot mid-flight,
// a divergent future, restore, and the same future again: the hash
// sequence repeats and a re-snapshot is byte-equal. The snapshot's bytes
// are pinned on their own line (ROLLBACK_SNAPSHOT), since they are a
// format: a format change moves that line and leaves the results alone.
// The release's ABI word is left out (HashStateBytes), so a release bump
// moves nothing.
static void SceneRollback(void)
{
    m2WorldDef def = WorldDef(192, 32, 1024, false);
    def.particleRadius = 0.06f;
    m2World* world = NewWorld(&def);
    BuildStack(world);
    BuildFluid(world);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, DT, 4);
    }
    int64_t size = m2World_SnapshotSize(world);
    uint8_t* snapshot = malloc((size_t)size);
    uint8_t* again = malloc((size_t)size);
    CHECK(Snapshot(world, snapshot, size) == size, "ROLLBACK: the snapshot writes");

    uint64_t first[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, DT, 4);
        first[i] = m2World_Hash(world);
    }
    m2World_SetGravity(world, (m2Vec2){3.0f, -4.0f}); // a future to roll back from
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, DT, 4);
    }
    CHECK(Restore(world, snapshot, size), "ROLLBACK: the snapshot restores");
    CHECK(Snapshot(world, again, size) == size, "ROLLBACK: the re-snapshot writes");
    CHECK(memcmp(snapshot, again, (size_t)size) == 0, "ROLLBACK: the re-snapshot is byte-equal");
    bool same = true;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, DT, 4);
        same = same && m2World_Hash(world) == first[i];
    }
    CHECK(same, "ROLLBACK: the restored world repeats its hash sequence");
    uint64_t hash = m2World_Hash(world);
    uint64_t bytes = HashStateBytes(M2_HASH_INIT, snapshot, size, 0);
    free(snapshot);
    free(again);
    m2DestroyWorld(world);
    Expect("ROLLBACK", hash, M2_GOLDEN_ROLLBACK);
    Expect("ROLLBACK_SNAPSHOT", bytes, M2_GOLDEN_ROLLBACK_SNAPSHOT);
}

// JOURNAL: the seeded API walk's ten canonical seeds, each recorded and
// replayed, with an unjournaled and a threaded twin (walk.h). The tapes'
// bytes are pinned on their own line (JOURNAL_TAPE), since they are a
// format, without the release's ABI words (HashTapeBytes).
#define WALK_SEEDS ((int32_t)(sizeof(s_walkSeeds) / sizeof(s_walkSeeds[0])))

// The walks' tapes, kept for the hostile host to replay.
static uint8_t* s_tapes[WALK_SEEDS];
static int64_t s_tapeSizes[WALK_SEEDS];

static uint64_t JournalHash(uint64_t hash, uint64_t walk)
{
    return hash ^ (walk + 0x9E3779B97F4A7C15ull + (hash << 6) + (hash >> 2));
}

static void SceneJournal(void)
{
    uint8_t* journal = malloc(WALK_JOURNAL_CAPACITY);
    uint64_t hash = 0x9E3779B97F4A7C15ull;
    s_walkTapeHash = M2_HASH_INIT;
    for (int32_t i = 0; i < WALK_SEEDS; ++i)
    {
        hash = JournalHash(hash, WalkSeed(s_walkSeeds[i], journal));
        free(s_tapes[i]);
        s_tapes[i] = malloc((size_t)s_walkTapeSize);
        memcpy(s_tapes[i], journal, (size_t)s_walkTapeSize);
        s_tapeSizes[i] = s_walkTapeSize;
    }
    free(journal);
    Expect("JOURNAL", hash, M2_GOLDEN_JOURNAL);
    Expect("JOURNAL_TAPE", s_walkTapeHash, M2_GOLDEN_JOURNAL_TAPE);
}

// The benchmark scenes at their own step counts.
static void SceneBench(void)
{
    static const uint64_t expected[] = {
        M2_GOLDEN_BENCH_PYRAMID15, M2_GOLDEN_BENCH_PYRAMID30, M2_GOLDEN_BENCH_CHAIN30,
        M2_GOLDEN_BENCH_CHAIN2000, M2_GOLDEN_BENCH_JOINTFARM, M2_GOLDEN_BENCH_WATER,
    };
    int32_t count = 0;
    const BenchScene* scenes = BenchScenes(&count);
    CHECK(count == (int32_t)(sizeof(expected) / sizeof(expected[0])),
          "one golden value per bench scene");
    for (int32_t k = 0; k < count && k < (int32_t)(sizeof(expected) / sizeof(expected[0])); ++k)
    {
        m2World* world = scenes[k].build();
        for (int32_t i = 0; i < scenes[k].steps; ++i)
        {
            m2World_Step(world, BENCH_DT, BENCH_SUBSTEPS);
        }
        char name[32];
        snprintf(name, sizeof(name), "BENCH_%s", scenes[k].name);
        for (char* c = name; *c != '\0'; ++c)
        {
            *c = *c >= 'a' && *c <= 'z' ? (char)(*c - 'a' + 'A') : *c;
        }
        Expect(name, m2World_Hash(world), expected[k]);
        m2DestroyWorld(world);
    }
}

// Steps a replayed walk takes through the public m2World_Step.
#define WALK_TAIL 30

typedef struct WalkRun
{
    uint64_t replay; // the walks' hashes as JOURNAL folds them
    uint64_t tail;   // the same, after WALK_TAIL more steps
} WalkRun;

// The walks' tapes replayed on the thread's setting, on the given number
// of workers, whose threads the host starts in its setting: the calls are
// the ones recorded, so the host's own rounding cannot reach their
// arguments. A replay steps through the journal's own path, inside one
// entry, so each world then steps on through the public m2World_Step.
static WalkRun ReplayWalks(int32_t workers)
{
    WalkRun run = {0x9E3779B97F4A7C15ull, 0x9E3779B97F4A7C15ull};
    for (int32_t i = 0; i < WALK_SEEDS; ++i)
    {
        m2WorldDef def = WalkWorldDef(workers);
        m2World* world = NewWorld(&def);
        CHECK(ReplayJournal(world, s_tapes[i], s_tapeSizes[i]), "a walk replays");
        run.replay = JournalHash(run.replay, m2World_Hash(world));
        for (int32_t k = 0; k < WALK_TAIL; ++k)
        {
            m2World_Step(world, DT, 4);
        }
        run.tail = JournalHash(run.tail, m2World_Hash(world));
        m2DestroyWorld(world);
    }
    return run;
}

// The replays on host setting s_host must reach JOURNAL, and their tails
// the tail the default setting reaches.
static void CheckWalks(uint64_t expectedTail)
{
    for (int32_t workers = 1; workers <= 3; workers += 2)
    {
        WalkRun run = ReplayWalks(workers);
        Expect("JOURNAL", run.replay, M2_GOLDEN_JOURNAL);
        if (run.tail != expectedTail)
        {
            printf("FAIL: the walks stepped on to %016llx on host setting %d, %d worker(s)\n",
                   (unsigned long long)run.tail, s_host, (int)workers);
            s_failures += 1;
        }
    }
}

// The scenes on a hostile host. The test's own arithmetic rounds in the
// host's mode too, so the scenes run whole only under the settings that
// keep its rounding mode and add flush-to-zero and denormals-are-zero,
// which none of their inputs reach. Under every setting, the walks'
// recorded calls replay and step on. After each, the host's setting is as
// it was.
static void HostilePass(void)
{
    void (*const scenes[])(void) = {SceneStack,    SceneContacts, SceneFluid,
                                    SceneRollback, SceneJournal,  SceneBench};
    int sceneCount = (int)(sizeof(scenes) / sizeof(scenes[0]));
    TestFloatSetting start = TestFloatGet();
    uint64_t expectedTail = ReplayWalks(1).tail;
    for (s_host = 0; s_host < TestFloatHostileCount(); ++s_host)
    {
        TestFloatSetting host = TestFloatHostileAt(s_host);
        bool whole = host.rounding == start.rounding && !TestFloatSame(host, start);
        for (int k = 0; whole && k < sceneCount; ++k)
        {
            TestFloatSet(host);
            scenes[k]();
            TestFloatSetting after = TestFloatGet();
            TestFloatSet(start);
            CHECK(TestFloatSame(after, host), "the host's setting after a scene");
        }
        TestFloatSet(host);
        CheckWalks(expectedTail);
        TestFloatSetting after = TestFloatGet();
        TestFloatSet(start);
        CHECK(TestFloatSame(after, host), "the host's setting after the replays");
    }
    s_host = -1;
    for (int32_t i = 0; i < WALK_SEEDS; ++i)
    {
        free(s_tapes[i]);
    }
}

int main(void)
{
    SceneStack();
    SceneContacts();
    SceneFluid();
    SceneRollback();
    SceneJournal();
    SceneBench();
    HostilePass();
    // Every checked value is present (a missing one reads as zero and
    // fails), so equal counts mean no line of test/hashes.txt is stale.
    if (s_expected != M2_GOLDEN_LINES)
    {
        printf("FAIL: test/hashes.txt has %d lines and the scenes check %d values; regenerate it\n",
               (int)M2_GOLDEN_LINES, (int)s_expected);
        s_failures += 1;
    }
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
