// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluid gate (storage slice): emit/destroy lifecycle with FIFO slot
// recycling under fresh generations, capacity overflow as a quiet
// fact, free-fall integration, rollback identity over particle
// state, journal replay of the particle ops, and the churn hash
// compared across CI cells as the 16th gated line.

#include "helpers.h"
#include "test_harness.h"
#include "test_task_pool.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2WorldDef FluidWorldDef(int32_t particleCapacity)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = particleCapacity;
    return def;
}

static void TestLifecycle(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2World* world = NewWorld(&def);

    // Emit a small grid; count and enumeration agree.
    m2ParticleId ids[64];
    for (int32_t i = 0; i < 20; ++i)
    {
        double x = (double)(i % 5) * 0.1;
        double y = 4.0 + (double)(i / 5) * 0.1;
        ids[i] = EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, 0);
        CHECK(m2Particle_IsValid(world, ids[i]), "each emitted particle is live");
    }
    CHECK(m2World_GetParticleCount(world) == 20, "the count sees every emit");
    m2ParticleId listed[64];
    int32_t total = m2World_GetParticles(world, listed, 64);
    CHECK(total == 20, "enumeration total is truthful");
    CHECK(listed[0].index1 == ids[0].index1, "ascending slot order starts at the first emit");

    // Free fall: gravity moves every particle down, nothing else.
    m2Pos2 before = m2Particle_GetPosition(world, ids[7]);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 after = m2Particle_GetPosition(world, ids[7]);
    CHECK(after.y < before.y - 0.5, "particles free-fall under gravity");
    CHECK(after.x == before.x, "no lateral force exists yet");
    CHECK(m2Particle_GetVelocity(world, ids[7]).y < -4.0f, "velocity integrates");

    // Destroy one third; slots recycle FIFO under fresh generations.
    for (int32_t i = 0; i < 20; i += 3)
    {
        DestroyParticle(world, ids[i]);
    }
    CHECK(m2World_GetParticleCount(world) == 13, "destroys drop the count");
    CHECK(!m2Particle_IsValid(world, ids[0]), "a destroyed id goes stale");
    // FIFO promise: freed slots return at the
    // END of the queue, never next; drain the 44 untouched slots and
    // the 45th emit lands on the first freed slot, fresh generation.
    m2ParticleId reborn = m2_nullParticleId;
    for (int32_t i = 0; i < 45; ++i)
    {
        reborn = EmitParticle(world, (m2Pos2){0.0, 9.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    CHECK(m2Particle_IsValid(world, reborn), "the pool recycles");
    CHECK(reborn.index1 == ids[0].index1 && reborn.generation == ids[0].generation + 1,
          "the first freed slot returns last, under a fresh generation");
    CHECK(!m2Particle_IsValid(world, ids[0]), "the old id stays stale after rebirth");

    // Retargeting velocity is immediate (an isolated particle: the
    // coincident tower above is exactly where the velocity-limit law
    // dominates, by design).
    m2ParticleId lone = EmitParticle(world, (m2Pos2){20.0, 9.0}, (m2Vec2){0.0f, 0.0f}, 0);
    m2Particle_SetVelocity(world, lone, (m2Vec2){3.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Particle_GetPosition(world, lone).x > 20.04, "a set velocity moves the particle");

    m2DestroyWorld(world);
}

static void TestOverflowIsQuiet(void)
{
    m2WorldDef def = FluidWorldDef(8);
    m2World* world = NewWorld(&def);
    int32_t granted = 0;
    for (int32_t i = 0; i < 12; ++i)
    {
        m2ParticleId id = EmitParticle(world, (m2Pos2){(double)i, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
        granted += id.index1 != 0 ? 1 : 0;
    }
    CHECK(granted == 8, "a full pool grants exactly its capacity");
    CHECK(m2World_GetParticleCount(world) == 8, "overflow leaves the count at capacity");
    m2DestroyWorld(world);
}

static void TestRollbackIdentity(void)
{
    m2WorldDef def = FluidWorldDef(128);
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < 60; ++i)
    {
        double x = (double)(i % 10) * 0.11;
        double y = 3.0 + (double)(i / 10) * 0.11;
        EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.1f, 0.0f}, 0);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot writes its full size");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t first = m2World_Hash(world);
    CHECK(Restore(world, snap, size), "restore accepts its own bytes");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t second = m2World_Hash(world);
    CHECK(first == second, "the fluid timeline replays bit-exactly after rollback");
    free(snap);
    m2DestroyWorld(world);
}

static void TestJournalReplay(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2World* world = NewWorld(&def);
    int32_t cap = 1 << 20;
    void* tape = malloc((size_t)cap);
    CHECK(StartJournal(world, tape, cap), "journal starts");

    m2ParticleId ids[32];
    for (int32_t i = 0; i < 16; ++i)
    {
        ids[i] = EmitParticle(world, (m2Pos2){(double)i * 0.09, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Particle_SetVelocity(world, ids[3], (m2Vec2){-2.0f, 1.0f});
    DestroyParticle(world, ids[8]);
    DestroyParticle(world, ids[9]);
    EmitParticle(world, (m2Pos2){0.5, 8.0}, (m2Vec2){0.0f, -1.0f}, 0);
    m2Polygon jellyTub = m2MakeBox(0.2f, 0.15f);
    FillParticles(world, &jellyTub, (m2Pos2){2.0, 6.0}, (m2Vec2){0.0f, 0.0f},
                  m2_springParticle | m2_elasticParticle); // op 57
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t used = StopJournal(world);
    CHECK(used > 0, "journal closed with bytes");
    uint64_t recorded = m2World_Hash(world);

    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, used), "replay accepts the tape");
    CHECK(m2World_Hash(fresh) == recorded, "the particle session replays onto the same bits");

    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(tape);
}

// The relaxation acceptance: a dense blob spreads toward rest
// spacing without exploding, the velocity limit holds, and the
// system calms down; viscosity kills tangential shear that plain
// water keeps.
static void TestRelaxation(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2ParticleId blob[25];
    for (int32_t i = 0; i < 25; ++i)
    {
        double x = (double)(i % 5) * 0.04;
        double y = (double)(i / 5) * 0.04;
        blob[i] = EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    float peakSpeed = 0.0f;
    for (int32_t step = 0; step < 120; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (step == 4)
        {
            for (int32_t i = 0; i < 25; ++i)
            {
                m2Vec2 v = m2Particle_GetVelocity(world, blob[i]);
                float speed = sqrtf(v.x * v.x + v.y * v.y);
                peakSpeed = peakSpeed > speed ? peakSpeed : speed;
            }
        }
    }
    double spreadSq = 0.0;
    float lateSpeed = 0.0f;
    for (int32_t i = 0; i < 25; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, blob[i]);
        CHECK(p.x == p.x && p.y == p.y, "positions stay finite");
        double dx = p.x - 0.08;
        double dy = p.y - 0.08;
        double d2 = dx * dx + dy * dy;
        spreadSq = d2 > spreadSq ? d2 : spreadSq;
        m2Vec2 v = m2Particle_GetVelocity(world, blob[i]);
        float speed = sqrtf(v.x * v.x + v.y * v.y);
        lateSpeed = lateSpeed > speed ? lateSpeed : speed;
    }
    CHECK(spreadSq > 0.013, "pressure spreads the blob past its packed extent");
    CHECK(peakSpeed > 0.0f, "the packed blob actually moves");
    // Free particles in zero gravity coast (walls arrive with the
    // rigid coupling slice); the promise here is the speed law.
    CHECK(peakSpeed <= 6.001f && lateSpeed <= 6.001f, "no speed ever beats one diameter per step");

    // Approach damping: two particles closing at 2 m/s inside one
    // diameter lose closing speed in a single step but do not stick.
    m2ParticleId left = EmitParticle(world, (m2Pos2){5.0, 5.0}, (m2Vec2){1.0f, 0.0f}, 0);
    m2ParticleId right = EmitParticle(world, (m2Pos2){5.08, 5.0}, (m2Vec2){-1.0f, 0.0f}, 0);
    m2World_Step(world, 1.0f / 60.0f, 4);
    float closing = m2Particle_GetVelocity(world, left).x - m2Particle_GetVelocity(world, right).x;
    CHECK(closing < 1.7f, "damping eats approach speed");
    CHECK(closing > 0.0f, "damping does not reverse the approach by itself");

    // The stability law: nothing crosses one diameter per step.
    m2ParticleId bullet = EmitParticle(world, (m2Pos2){3.0, 3.0}, (m2Vec2){1000.0f, 0.0f}, 0);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Vec2 bv = m2Particle_GetVelocity(world, bullet);
    CHECK(sqrtf(bv.x * bv.x + bv.y * bv.y) <= 6.001f,
          "the velocity limit clamps to one diameter per step");
    m2DestroyWorld(world);

    // Tangential shear: plain water keeps it, the viscous flag eats it.
    // Same world def, the flag alone decides. The pair normal is y and
    // the shear x: pressure and approach damping never touch it.
    m2WorldDef shearDef = FluidWorldDef(8);
    shearDef.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* shearWorld = NewWorld(&shearDef);
    m2ParticleId wSlide = EmitParticle(shearWorld, (m2Pos2){0.0, 0.0}, (m2Vec2){1.0f, 0.0f}, 0);
    EmitParticle(shearWorld, (m2Pos2){0.0, 0.06}, (m2Vec2){0.0f, 0.0f}, 0);
    m2ParticleId sSlide =
        EmitParticle(shearWorld, (m2Pos2){10.0, 0.0}, (m2Vec2){1.0f, 0.0f}, m2_viscousParticle);
    EmitParticle(shearWorld, (m2Pos2){10.0, 0.06}, (m2Vec2){0.0f, 0.0f}, m2_viscousParticle);
    for (int32_t s = 0; s < 10; ++s)
    {
        m2World_Step(shearWorld, 1.0f / 60.0f, 4);
    }
    CHECK(m2Particle_GetVelocity(shearWorld, wSlide).x > 0.95f,
          "plain water keeps tangential shear");
    CHECK(m2Particle_GetVelocity(shearWorld, sSlide).x < 0.7f,
          "the viscous flag eats tangential shear");

    // Powder: grains packed tighter than the rest stride push apart;
    // water at the same spacing is below rest weight and never moves.
    m2ParticleId g1 =
        EmitParticle(shearWorld, (m2Pos2){20.0, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_powderParticle);
    m2ParticleId g2 =
        EmitParticle(shearWorld, (m2Pos2){20.05, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_powderParticle);
    m2ParticleId w1 = EmitParticle(shearWorld, (m2Pos2){30.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    m2ParticleId w2 = EmitParticle(shearWorld, (m2Pos2){30.05, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    for (int32_t s = 0; s < 90; ++s)
    {
        m2World_Step(shearWorld, 1.0f / 60.0f, 4);
    }
    double grainGap =
        m2Particle_GetPosition(shearWorld, g2).x - m2Particle_GetPosition(shearWorld, g1).x;
    double waterGap =
        m2Particle_GetPosition(shearWorld, w2).x - m2Particle_GetPosition(shearWorld, w1).x;
    CHECK(grainGap > 0.06, "packed powder pushes apart past the stride");
    CHECK(waterGap > 0.049 && waterGap < 0.051, "sparse water below rest weight never moves");
    m2Vec2 gv = m2Particle_GetVelocity(shearWorld, g1);
    CHECK(gv.x == gv.x && gv.y == gv.y, "powder stays finite");
    m2DestroyWorld(shearWorld);
}

// Overflow truncates deterministically and counts what it dropped.
static void TestPairOverflow(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < 64; ++i)
    {
        EmitParticle(world, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    m2World_Step(world, 1.0f / 60.0f, 4);
    // 64 coincident particles make 2016 pairs, past the pair budget.
    m2Counters c = m2World_GetCounters(world);
    CHECK(c.particlePairOverflow > 0, "the dropped pairs are counted");
    CHECK(c.particlePairOverflow < 64 * 63 / 2, "and the budget keeps the rest");
    m2DestroyWorld(world);
}

// The coupling acceptance: water poured into a static basin stays
// inside, calms down, and stacks a surface; the slice-84 promise
// that needed walls.
static void TestBasin(void)
{
    m2WorldDef def = FluidWorldDef(128);
    m2World* world = NewWorld(&def);

    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){100.0, 49.9};
    m2Polygon floorBox = m2MakeBox(0.7f, 0.1f);
    NewPolygonShape(world, NewBody(world, &floorDef), &sd, &floorBox);
    m2Polygon wallBox = m2MakeBox(0.1f, 0.8f);
    m2BodyDef leftDef = m2DefaultBodyDef();
    leftDef.position = (m2Pos2){99.3, 50.7};
    NewPolygonShape(world, NewBody(world, &leftDef), &sd, &wallBox);
    m2BodyDef rightDef = m2DefaultBodyDef();
    rightDef.position = (m2Pos2){100.7, 50.7};
    NewPolygonShape(world, NewBody(world, &rightDef), &sd, &wallBox);

    m2ParticleId drops[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        double x = 100.0 - 0.44 + (double)(i % 10) * 0.09;
        double y = 50.3 + (double)(i / 10) * 0.09;
        drops[i] = EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    for (int32_t step = 0; step < 360; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float lateSpeed = 0.0f;
    double lowest = 1.0e9;
    double highest = -1.0e9;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, drops[i]);
        CHECK(p.x == p.x && p.y == p.y, "water stays finite");
        CHECK(p.x > 99.15 && p.x < 100.85, "no particle leaks through a wall");
        CHECK(p.y > 49.85, "no particle leaks through the floor");
        m2Vec2 v = m2Particle_GetVelocity(world, drops[i]);
        float speed = sqrtf(v.x * v.x + v.y * v.y);
        lateSpeed = lateSpeed > speed ? lateSpeed : speed;
        lowest = p.y < lowest ? p.y : lowest;
        highest = p.y > highest ? p.y : highest;
    }
    CHECK(lateSpeed < 1.0f, "the pool calms down");
    CHECK(lowest < 50.1, "water reaches the floor");
    CHECK(highest < 51.0, "water pools instead of climbing the walls");
    m2DestroyWorld(world);
}

// One-way platforms are one-way for water; sensors are invisible.
static void TestWaterChainAndSensor(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2World* world = NewWorld(&def);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.position = (m2Pos2){0.0, 5.0};
    m2BodyId ground = NewBody(world, &bd);
    // Solid side up: chain runs right to left (the winding law).
    m2Vec2 points[4] = {{2.0f, 0.0f}, {1.0f, 0.0f}, {-1.0f, 0.0f}, {-2.0f, 0.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = points;
    cd.count = 4;
    NewChain(world, ground, &cd);

    // A sensor box hangs above the platform, in the fall path.
    m2BodyDef sensorBody = m2DefaultBodyDef();
    sensorBody.position = (m2Pos2){0.0, 6.5};
    m2ShapeDef sensorDef = m2DefaultShapeDef();
    sensorDef.isSensor = true;
    m2Polygon sensorBox = m2MakeBox(0.5f, 0.2f);
    NewPolygonShape(world, NewBody(world, &sensorBody), &sensorDef, &sensorBox);

    m2ParticleId rain[10];
    for (int32_t i = 0; i < 10; ++i)
    {
        rain[i] =
            EmitParticle(world, (m2Pos2){-0.45 + (double)i * 0.09, 7.5}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    m2ParticleId spray = EmitParticle(world, (m2Pos2){0.0, 3.0}, (m2Vec2){0.0f, 5.9f}, 0);
    for (int32_t step = 0; step < 240; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t i = 0; i < 10; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, rain[i]);
        CHECK(p.y > 4.95 && p.y < 5.6, "rain rests on the solid side, through the sensor");
    }
    m2Pos2 sp = m2Particle_GetPosition(world, spray);
    CHECK(sp.y < 4.0, "spray from below passed the ghost side and fell back through");
    m2DestroyWorld(world);
}

// Two-way coupling: what floats, floats; what sinks, sinks; and
// water landing on a sleeper wakes it through the island law.
static void TestBuoyancy(void)
{
    m2WorldDef def = FluidWorldDef(128);
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef floorDef = m2DefaultBodyDef();
    floorDef.position = (m2Pos2){100.0, 49.9};
    m2Polygon floorBox = m2MakeBox(0.7f, 0.1f);
    NewPolygonShape(world, NewBody(world, &floorDef), &sd, &floorBox);
    m2Polygon wallBox = m2MakeBox(0.1f, 0.8f);
    m2BodyDef leftDef = m2DefaultBodyDef();
    leftDef.position = (m2Pos2){99.3, 50.7};
    NewPolygonShape(world, NewBody(world, &leftDef), &sd, &wallBox);
    m2BodyDef rightDef = m2DefaultBodyDef();
    rightDef.position = (m2Pos2){100.7, 50.7};
    NewPolygonShape(world, NewBody(world, &rightDef), &sd, &wallBox);
    for (int32_t i = 0; i < 100; ++i)
    {
        double x = 100.0 - 0.44 + (double)(i % 10) * 0.09;
        double y = 50.25 + (double)(i / 10) * 0.09;
        EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2Polygon crate = m2MakeBox(0.1f, 0.1f);
    m2BodyDef lightDef = m2DefaultBodyDef();
    lightDef.type = m2_dynamicBody;
    lightDef.position = (m2Pos2){99.7, 50.9};
    m2BodyId light = NewBody(world, &lightDef);
    m2ShapeDef lightShape = m2DefaultShapeDef();
    lightShape.density = 0.05f;
    NewPolygonShape(world, light, &lightShape, &crate);
    m2BodyDef heavyDef = m2DefaultBodyDef();
    heavyDef.type = m2_dynamicBody;
    heavyDef.position = (m2Pos2){100.3, 50.9};
    m2BodyId heavy = NewBody(world, &heavyDef);
    m2ShapeDef heavyShape = m2DefaultShapeDef();
    heavyShape.density = 3.0f;
    NewPolygonShape(world, heavy, &heavyShape, &crate);
    for (int32_t i = 0; i < 500; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 lp = m2Body_GetPosition(world, light);
    m2Pos2 hp = m2Body_GetPosition(world, heavy);
    CHECK(lp.y > 50.3, "the light crate floats on the water body");
    CHECK(hp.y < 50.15, "the heavy crate sinks to the floor");
    CHECK(lp.y > hp.y + 0.2, "density decides who floats");
    m2DestroyWorld(world);

    // Wake: a crate asleep in a dry basin, then rain.
    m2World* dry = NewWorld(&def);
    m2BodyDef groundDef = m2DefaultBodyDef();
    m2Polygon slab = m2MakeBox(1.0f, 0.1f);
    NewPolygonShape(dry, NewBody(dry, &groundDef), &sd, &slab);
    m2BodyDef boxDef = m2DefaultBodyDef();
    boxDef.type = m2_dynamicBody;
    boxDef.position = (m2Pos2){0.0, 0.35};
    m2BodyId box = NewBody(dry, &boxDef);
    m2ShapeDef boxShape = m2DefaultShapeDef();
    NewPolygonShape(dry, box, &boxShape, &crate);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(dry, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(dry, box), "the crate sleeps in the dry basin");
    for (int32_t i = 0; i < 5; ++i)
    {
        EmitParticle(dry, (m2Pos2){-0.1 + (double)i * 0.05, 0.55}, (m2Vec2){0.0f, -1.0f}, 0);
    }
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(dry, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_IsAwake(dry, box), "rain wakes the sleeper");
    m2DestroyWorld(dry);
}

// Surface tension: a sparse tensile pair pulls itself together and
// rests; a plain pair at the same spacing never moves. A mixed blob
// stays finite and inside the speed law.
static void TestSurfaceTension(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2ParticleId t1 =
        EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_tensileParticle);
    m2ParticleId t2 =
        EmitParticle(world, (m2Pos2){0.08, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_tensileParticle);
    CHECK(m2Particle_GetFlags(world, t1) == m2_tensileParticle, "flags read back");
    m2ParticleId p1 = EmitParticle(world, (m2Pos2){10.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    m2ParticleId p2 = EmitParticle(world, (m2Pos2){10.08, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    for (int32_t s = 0; s < 120; ++s)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double td = m2Particle_GetPosition(world, t2).x - m2Particle_GetPosition(world, t1).x;
    double pd = m2Particle_GetPosition(world, p2).x - m2Particle_GetPosition(world, p1).x;
    CHECK(td > 0.02 && td < 0.065, "the tensile pair pulls together and rests");
    CHECK(pd > 0.079 && pd < 0.081, "the plain pair never moves");
    m2Vec2 tv = m2Particle_GetVelocity(world, t1);
    CHECK(sqrtf(tv.x * tv.x + tv.y * tv.y) < 1.0e-3f,
          "the droplet settles where cohesion meets its near pressure");

    // A mixed dense blob stays finite and lawful: above rest density a
    // tensile particle pushes like water; cohesion is the sparse story
    // above.
    for (int32_t i = 0; i < 16; ++i)
    {
        double x = 20.0 + (double)(i % 4) * 0.06;
        double y = (double)(i / 4) * 0.06;
        uint32_t flags = i % 2 == 0 ? m2_tensileParticle : 0;
        EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, flags);
    }
    for (int32_t s = 0; s < 120; ++s)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2ParticleId all[64];
    int32_t n = m2World_GetParticles(world, all, 64);
    for (int32_t i = 0; i < n; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, all[i]);
        m2Vec2 v = m2Particle_GetVelocity(world, all[i]);
        CHECK(p.x == p.x && p.y == p.y, "tension never makes a NaN");
        CHECK(sqrtf(v.x * v.x + v.y * v.y) <= 6.001f, "tension never beats the speed law");
    }
    m2DestroyWorld(world);
}

// The fill helper: one call, one pool, deterministic layout.
static void TestParticleFill(void)
{
    m2WorldDef def = FluidWorldDef(128);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2Polygon tub = m2MakeBox(0.5f, 0.25f);
    int32_t made = FillParticles(world, &tub, (m2Pos2){5.0, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(made > 70 && made < 110, "the fill lands near area over stride squared");
    CHECK(made == m2World_GetParticleCount(world), "every fill emit is a live particle");
    m2ParticleId ids[128];
    int32_t total = m2World_GetParticles(world, ids, 128);
    for (int32_t i = 0; i < total; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, ids[i]);
        CHECK(p.x > 4.49 && p.x < 5.51 && p.y > 4.74 && p.y < 5.26,
              "every drop lands inside the polygon bounds");
    }
    m2World* twin = NewWorld(&def);
    int32_t madeTwin = FillParticles(twin, &tub, (m2Pos2){5.0, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(madeTwin == made, "twin fills agree exactly");
    CHECK(m2World_Hash(twin) == m2World_Hash(world), "twin fills agree to the bit");
    m2DestroyWorld(twin);
    m2DestroyWorld(world);

    // A small pool fills to capacity and stops quietly.
    m2WorldDef tiny = FluidWorldDef(16);
    m2World* small = NewWorld(&tiny);
    int32_t granted = FillParticles(small, &tub, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(granted == 16, "a full pool grants exactly its capacity");
    m2DestroyWorld(small);
}

// A triangle 2e30 m tall with its apex at the bottom: past M2_LOCAL_LIMIT, so a
// fill refuses it as a shape would.
static m2Polygon FarTriangle(void)
{
    m2Polygon distant;
    memset(&distant, 0, sizeof(distant));
    distant.count = 3;
    distant.vertices[0] = (m2Vec2){0.0f, -1.0e30f};
    distant.vertices[1] = (m2Vec2){0.29f, 1.0e30f};
    distant.vertices[2] = (m2Vec2){-0.29f, 1.0e30f};
    distant.normals[0] = (m2Vec2){1.0f, 0.0f};
    distant.normals[1] = (m2Vec2){0.0f, 1.0f};
    distant.normals[2] = (m2Vec2){-1.0f, 0.0f};
    return distant;
}

static void TestFillChecksItsPolygon(void)
{
    m2WorldDef def = FluidWorldDef(64);
    m2World* world = NewWorld(&def);
    m2Polygon distant = FarTriangle();
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2ParticleDef particleDef = m2DefaultParticleDef();
    int32_t count = -1;
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &distant, &count) == m2_errorLimit &&
              count == 0,
          "a polygon past M2_LOCAL_LIMIT fills nothing");
    CHECK(m2World_GetCounters(world).misuse == misuse + 1 && m2World_GetParticleCount(world) == 0,
          "and is refused as a misuse");

    // The normals are recomputed from the vertices: a box with its normals
    // turned around fills like the box.
    m2Polygon box = m2MakeBox(0.3f, 0.2f);
    m2Polygon turned = box;
    for (int32_t i = 0; i < turned.count; ++i)
    {
        turned.normals[i] = (m2Vec2){-box.normals[i].x, -box.normals[i].y};
    }
    m2World* twin = NewWorld(&def);
    int32_t made = FillParticles(world, &box, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
    int32_t madeTurned = FillParticles(twin, &turned, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(made > 0 && madeTurned == made && m2World_Hash(twin) == m2World_Hash(world),
          "a box with turned normals fills like the box");
    m2DestroyWorld(twin);
    m2DestroyWorld(world);

    // A tape whose fill record carries the distant triangle is refused whole.
    m2World* recorder = NewWorld(&def);
    int64_t capacity = m2World_GetJournalBaseSize(recorder) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(recorder, tape, capacity), "the journal starts");
    FillParticles(recorder, &box, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, 0.0f}, 0);
    int64_t bytes = StopJournal(recorder);
    int32_t at = -1;
    for (int32_t i = 0; at < 0 && i + (int32_t)sizeof(box) <= bytes; ++i)
    {
        at = memcmp(tape + i, &box, sizeof(box)) == 0 ? i : -1;
    }
    CHECK(at > 0, "the fill record carries the box");
    memcpy(tape + at, &distant, sizeof(distant));
    m2World* fresh = NewWorld(&def);
    uint64_t before = m2World_Hash(fresh);
    CHECK(m2World_ReplayJournal(fresh, tape, bytes) == m2_errorFormat,
          "replay refuses the distant triangle's fill");
    CHECK(m2World_Hash(fresh) == before && m2World_GetCounters(fresh).misuse == 0,
          "and changes nothing, counting no misuse");
    m2DestroyWorld(fresh);
    m2DestroyWorld(recorder);
    free(tape);
}

// Jelly: spring nets remember their spawn lengths, elastic triads
// their spawn shape; the state persists, journals, and dies with
// its particles.
static void TestJelly(void)
{
    // Stretch recovery in zero gravity: fling every particle outward
    // at eight times its offset and the net pulls the blob back to
    // its spawn extent.
    m2WorldDef def = FluidWorldDef(128);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2Polygon blob = m2MakeBox(0.3f, 0.3f);
    int32_t made =
        FillParticles(world, &blob, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_springParticle);
    CHECK(made == 64, "the lattice fills its 8 by 8");
    m2ParticleId ids[128];
    int32_t n = m2World_GetParticles(world, ids, 128);
    for (int32_t i = 0; i < n; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, ids[i]);
        m2Particle_SetVelocity(world, ids[i], (m2Vec2){(float)(p.x * 8.0), (float)(p.y * 8.0)});
    }
    for (int32_t s = 0; s < 240; ++s)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double x0 = 1.0e9, x1 = -1.0e9;
    for (int32_t i = 0; i < n; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, ids[i]);
        x0 = p.x < x0 ? p.x : x0;
        x1 = p.x > x1 ? p.x : x1;
    }
    CHECK(x1 - x0 > 0.4 && x1 - x0 < 0.8, "the stretched net springs back to its extent");
    m2DestroyWorld(world);

    // Cohesion under gravity: a jelly blob slumps but holds while a
    // plain water twin runs off across the whole slab.
    m2WorldDef gdef = FluidWorldDef(256);
    m2World* gw = NewWorld(&gdef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.2};
    m2Polygon slab = m2MakeBox(10.0f, 0.2f);
    NewPolygonShape(gw, NewBody(gw, &fd), &sd, &slab);
    FillParticles(gw, &blob, (m2Pos2){-2.0, 0.42}, (m2Vec2){0.0f, 0.0f},
                  m2_springParticle | m2_elasticParticle);
    FillParticles(gw, &blob, (m2Pos2){2.0, 0.42}, (m2Vec2){0.0f, 0.0f}, 0);
    for (int32_t s = 0; s < 600; ++s)
    {
        m2World_Step(gw, 1.0f / 60.0f, 4);
    }
    m2ParticleId all[256];
    int32_t total = m2World_GetParticles(gw, all, 256);
    double jx0 = 1.0e9, jx1 = -1.0e9, wx0 = 1.0e9, wx1 = -1.0e9;
    for (int32_t i = 0; i < total; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(gw, all[i]);
        uint32_t flags = m2Particle_GetFlags(gw, all[i]);
        if ((flags & m2_springParticle) != 0)
        {
            jx0 = p.x < jx0 ? p.x : jx0;
            jx1 = p.x > jx1 ? p.x : jx1;
        }
        else
        {
            wx0 = p.x < wx0 ? p.x : wx0;
            wx1 = p.x > wx1 ? p.x : wx1;
        }
    }
    CHECK(jx1 - jx0 < 4.0, "jelly slumps but holds together");
    CHECK(wx1 - wx0 > 8.0, "the water twin runs off across the slab");
    CHECK(jx1 - jx0 < (wx1 - wx0) * 0.5, "the nets at least halve the spread");

    m2World_Step(gw, 1.0f / 60.0f, 4);
    m2DestroyWorld(gw);
}

// The region query: exact membership, truthful totals, and the
// read-only law.
static void TestOverlapParticles(void)
{
    m2WorldDef def = FluidWorldDef(64);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < 25; ++i)
    {
        double x = (double)(i % 5) * 1.0;
        double y = (double)(i / 5) * 1.0;
        m2ParticleId made = EmitParticle(world, (m2Pos2){x, y}, (m2Vec2){0.0f, 0.0f}, 0);
        CHECK(made.index1 != 0, "the grid emits");
    }
    m2ParticleId found[64];
    int32_t n =
        m2World_OverlapParticlesAabb(world, (m2WorldAabb){{0.5, 0.5}, {3.5, 3.5}}, found, 64);
    CHECK(n == 9, "the inner three-by-three answers the box");
    for (int32_t i = 0; i < n; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, found[i]);
        CHECK(p.x > 0.5 && p.x < 3.5 && p.y > 0.5 && p.y < 3.5, "every answer is inside");
        CHECK(m2Particle_IsValid(world, found[i]), "every answer is live");
    }
    int32_t total =
        m2World_OverlapParticlesAabb(world, (m2WorldAabb){{0.5, 0.5}, {3.5, 3.5}}, found, 4);
    CHECK(total == 9, "capacity pressure never lies about the total");
    total = m2World_OverlapParticlesAabb(world, (m2WorldAabb){{0.5, 0.5}, {3.5, 3.5}}, NULL, 0);
    CHECK(total == 9, "the count query agrees");
    total =
        m2World_OverlapParticlesAabb(world, (m2WorldAabb){{50.0, 50.0}, {51.0, 51.0}}, found, 64);
    CHECK(total == 0, "an empty region answers zero");

    // Read-only law: a query storm moves no bits.
    m2World_Step(world, 1.0f / 60.0f, 4);
    uint64_t before = m2World_Hash(world);
    for (int32_t i = 0; i < 50; ++i)
    {
        m2World_OverlapParticlesAabb(world, (m2WorldAabb){{-1.0, -1.0}, {5.0, 5.0}}, found, 64);
    }
    CHECK(m2World_Hash(world) == before, "the query storm leaves the world untouched");
    m2DestroyWorld(world);
}

// Threaded fluids: a pool past the threading threshold reaches the
// same bits at one worker and at four. The build passes thread; the
// choice is scheduling, never physics.
static void TestFluidThreadingParity(void)
{
    uint64_t hashes[2];
    int32_t workers[2] = {1, 4};
    for (int32_t run = 0; run < 2; ++run)
    {
        m2WorldDef def = m2DefaultWorldDef();
        def.bodyCapacity = 16;
        def.shapeCapacity = 16;
        def.jointCapacity = 4;
        def.particleCapacity = 6000;
        if (workers[run] > 1)
        {
            def.enqueueTask = TpEnqueue;
            def.finishTask = TpFinish;
            def.userTaskContext = &workers[run];
        }
        m2World* world = NewWorld(&def);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2BodyDef fd = m2DefaultBodyDef();
        fd.position = (m2Pos2){0.0, -0.2};
        m2Polygon slab = m2MakeBox(6.0f, 0.2f);
        NewPolygonShape(world, NewBody(world, &fd), &sd, &slab);
        m2Polygon wall = m2MakeBox(0.2f, 4.0f);
        m2BodyDef ld = m2DefaultBodyDef();
        ld.position = (m2Pos2){-6.0, 3.8};
        NewPolygonShape(world, NewBody(world, &ld), &sd, &wall);
        m2BodyDef rd = m2DefaultBodyDef();
        rd.position = (m2Pos2){6.0, 3.8};
        NewPolygonShape(world, NewBody(world, &rd), &sd, &wall);
        m2Polygon pool = m2MakeBox(5.6f, 2.0f);
        int32_t made = FillParticles(world, &pool, (m2Pos2){0.0, 2.2}, (m2Vec2){0.0f, 0.0f}, 0);
        CHECK(made >= 4096, "the pool clears the threading threshold");
        for (int32_t i = 0; i < 90; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        hashes[run] = m2World_Hash(world);
        m2DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "one worker and four reach the same fluid bits");
}

// Lifetime and userData: a spray that cleans itself up on schedule,
// data that rides snapshots and tapes, and a death that rolls back.
static void TestLifetimeAndUserData(void)
{
    m2WorldDef def = FluidWorldDef(32);
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);

    // Ten drops, half of them mortal at a quarter second (15 steps at
    // 1/60), tagged with their index in userData.
    m2ParticleId drops[10];
    for (int32_t i = 0; i < 10; ++i)
    {
        drops[i] = EmitParticle(world, (m2Pos2){(double)i, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
        m2Particle_SetUserData(world, drops[i], (uint64_t)(1000 + i));
        if (i % 2 == 0)
        {
            m2Particle_SetLifetime(world, drops[i], 0.25f);
        }
    }
    CHECK(m2Particle_GetUserData(world, drops[3]) == 1003, "userData reads back");
    CHECK(m2Particle_GetLifetime(world, drops[0]) == 0.25f, "lifetime reads back");
    CHECK(m2Particle_GetLifetime(world, drops[1]) == 0.0f, "immortals report zero");

    // Snapshot before the reaping, then run past it.
    int64_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    Snapshot(world, snap, size);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_GetParticleCount(world) == 5, "the mortal half reaped itself on schedule");
    CHECK(!m2Particle_IsValid(world, drops[0]), "an expired drop is gone");
    CHECK(m2Particle_IsValid(world, drops[1]) && m2Particle_GetUserData(world, drops[1]) == 1001,
          "an immortal keeps its data past the reaping");

    // A particle the world reaped leaves its id behind in the program, so
    // a stale particle id is no misuse: every call answers and counts
    // nothing.
    uint64_t misuse = m2World_GetCounters(world).misuse;
    CHECK(m2Particle_GetPosition(world, drops[0]).x == 0.0, "an expired drop reads as zero");
    m2Particle_SetVelocity(world, drops[0], (m2Vec2){1.0f, 0.0f});
    m2Particle_SetLifetime(world, drops[0], 1.0f);
    m2Particle_SetUserData(world, drops[0], 7);
    CHECK(m2DestroyParticle(world, drops[0]) == m2_errorStale, "destroying it is m2_errorStale");
    CHECK(m2World_GetCounters(world).misuse == misuse, "and none of it counts as misuse");

    // Rollback: the reaped drops return with their data intact.
    CHECK(Restore(world, snap, size), "the pre-reaping snapshot restores");
    CHECK(m2World_GetParticleCount(world) == 10, "the reaped drops are back");
    CHECK(m2Particle_IsValid(world, drops[0]) && m2Particle_GetUserData(world, drops[0]) == 1000,
          "a resurrected drop keeps its userData and lifetime");
    CHECK(m2Particle_GetLifetime(world, drops[0]) == 0.25f, "and its unspent lifetime");
    free(snap);
    m2DestroyWorld(world);
}

// Particles anywhere a finite position can name step without undefined
// behavior: the grid cell of a position far beyond its reach (1e129 m)
// used to overflow an integer cast. fuzz_snapshot found it through a
// snapshot; the API reaches it too.
static void TestFarParticlesStep(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 8;
    m2World* world = NewWorld(&def);
    const m2Pos2 distant[] = {{1.0e129, 0.0}, {-1.0e300, 5.0}, {0.0, 1.0e200}, {3.0e9, -3.0e9}};
    for (size_t i = 0; i < sizeof(distant) / sizeof(distant[0]); ++i)
    {
        CHECK(EmitParticle(world, distant[i], (m2Vec2){0.0f, 0.0f}, 0).index1 != 0,
              "a far particle is emitted");
    }
    EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    for (int32_t i = 0; i < 4; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Validate(world), "far particles step into a valid world");
    m2DestroyWorld(world);
}

// Gives a world one of four tree histories. Every body is teleported in
// place, which marks it moved; with history 1 to 3 it first goes far away
// and back, in reverse, forward or interleaved order, so its leaf is
// reinserted and the tree takes another shape while every pose and fat
// box comes back the same. The worlds then hash the same and differ only
// in their trees.
static void Reshuffle(m2World* world, const m2BodyId* ids, int32_t count, int32_t history)
{
    m2World_Step(world, 1.0f / 60.0f, 4); // the creation's moved flags clear
    m2Transform homes[64];
    for (int32_t i = 0; i < count; ++i)
    {
        homes[i] = m2Body_GetTransform(world, ids[i]);
    }
    for (int32_t k = 0; k < count && history != 0; ++k)
    {
        int32_t i = history == 1 ? count - 1 - k : history == 3 ? (k * 7) % count : k;
        m2Pos2 away = {homes[i].p.x + 1000.0, homes[i].p.y};
        m2Body_SetTransform(world, ids[i], away, homes[i].q);
    }
    for (int32_t i = 0; i < count; ++i)
    {
        m2Body_SetTransform(world, ids[i], homes[i].p, homes[i].q);
    }
}

static m2World* FieldWorld(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.particleCapacity = 64;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    return NewWorld(&def);
}

// 64 thin overlapping walls 2 mm apart, created in a scrambled, mirrored
// order so slots do not follow position: a particle among them has more
// candidate shapes than the cap.
static m2World* WallField(int32_t history)
{
    m2World* world = FieldWorld();
    m2BodyId ids[64];
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(0.01f, 2.0f);
    for (int32_t i = 0; i < 64; ++i)
    {
        m2BodyDef wd = m2DefaultBodyDef();
        wd.position = (m2Pos2){0.002 * (double)(63 - (i * 37) % 64), 0.0};
        ids[i] = NewBody(world, &wd);
        NewPolygonShape(world, ids[i], &sd, &slab);
    }
    Reshuffle(world, ids, 64, history);
    for (int32_t k = 0; k < 8; ++k)
    {
        double y = -1.0 + 0.25 * (double)k;
        EmitParticle(world, (m2Pos2){-0.2, y}, (m2Vec2){20.0f, 0.0f}, 0);
        EmitParticle(world, (m2Pos2){0.06, y}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    return world;
}

// A wall across the diagonal at (0.25, 0.25), created after 40 small
// circles 9 cm off the diagonal beside it, whose fat boxes reach into a
// diagonal sweep's box near the wall: a particle swept along the diagonal
// has more candidate shapes than the cap, and only the wall stops it.
static m2World* CrossingField(int32_t history)
{
    m2World* world = FieldWorld();
    m2BodyId ids[41];
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle dot = {{0.0f, 0.0f}, 0.008f};
    for (int32_t i = 0; i < 40; ++i)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.position = (m2Pos2){0.30, 0.17}; // 9 cm off the diagonal, by the wall
        ids[i] = NewBody(world, &cd);
        NewCircleShape(world, ids[i], &sd, &dot);
    }
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){0.25, 0.25};
    wd.rotation = m2MakeRot(0.25f * M2_PI); // its long side across the diagonal
    ids[40] = NewBody(world, &wd);
    m2Polygon slab = m2MakeBox(0.01f, 0.3f);
    NewPolygonShape(world, ids[40], &sd, &slab);
    Reshuffle(world, ids, 41, history);
    EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){30.0f, 30.0f}, 0);
    return world;
}

// Particles with more candidate shapes than the cap, resting among them
// or sweeping past them, keep the same candidates whatever the tree's
// history: the two fields of each kind hash the same and step on alike.
static void TestCappedCandidatesIgnoreTreeHistory(void)
{
    m2World* (*const fields[2])(int32_t) = {WallField, CrossingField};
    for (int32_t k = 0; k < 2; ++k)
    {
        for (int32_t history = 1; history < 4; ++history)
        {
            m2World* plain = fields[k](0);
            m2World* reshuffled = fields[k](history);
            CHECK(m2World_Hash(plain) == m2World_Hash(reshuffled), "the fields hash the same");
            bool same = true;
            for (int32_t i = 0; i < 30 && same; ++i)
            {
                m2World_Step(plain, 1.0f / 60.0f, 4);
                m2World_Step(reshuffled, 1.0f / 60.0f, 4);
                same = m2World_Hash(plain) == m2World_Hash(reshuffled);
            }
            if (!same)
            {
                printf("field %d, history %d diverged\n", (int)k, (int)history);
            }
            CHECK(same, "and step on to the same bits");
            m2DestroyWorld(plain);
            m2DestroyWorld(reshuffled);
        }
    }
}

// A particle swept diagonally to a wall created after 40 small circles
// beside it, half on static and half on kinematic bodies: more candidate
// shapes than 32, yet fewer than 32 in each tree, so the particle stops at
// the wall.
static void TestCrowdedCrossingStops(void)
{
    m2World* world = FieldWorld();
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle dot = {{0.0f, 0.0f}, 0.008f};
    for (int32_t i = 0; i < 40; ++i)
    {
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = i % 2 == 0 ? m2_staticBody : m2_kinematicBody;
        cd.position = (m2Pos2){0.30, 0.17}; // 9 cm off the diagonal, by the wall
        NewCircleShape(world, NewBody(world, &cd), &sd, &dot);
    }
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){0.25, 0.25};
    wd.rotation = m2MakeRot(0.25f * M2_PI); // its long side across the diagonal
    m2Polygon slab = m2MakeBox(0.01f, 0.3f);
    NewPolygonShape(world, NewBody(world, &wd), &sd, &slab);
    m2ParticleId particle = EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){30.0f, 30.0f}, 0);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 at = m2Particle_GetPosition(world, particle);
    CHECK(at.x + at.y < 0.5, "the particle stops at the wall");
    m2DestroyWorld(world);
}

// A program may hold the id of a particle that is gone. A slot is reused
// every capacity emits, so one slot reused 65536 times would bring a 16-bit
// generation back to the held id: it would name the new particle. Volumes
// share the pattern.
static void TestDeadIdsStayDead(void)
{
    m2WorldDef def = FluidWorldDef(1);
    def.fluidVolumeCapacity = 1;
    m2World* world = NewWorld(&def);
    m2ParticleId first = EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(DestroyParticle(world, first) == m2_success, "the first particle goes");
    int32_t revived = 0;
    for (int32_t i = 0; i < 65537; ++i)
    {
        m2ParticleId next = EmitParticle(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
        revived += m2Particle_IsValid(world, first) ? 1 : 0;
        CHECK(DestroyParticle(world, next) == m2_success, "each later particle goes");
    }
    CHECK(revived == 0, "a held particle id never names a later particle");

    m2FluidVolumeDef pool = m2DefaultFluidVolumeDef();
    pool.regionLower = (m2Pos2){-1.0, -1.0};
    pool.regionUpper = (m2Pos2){1.0, 1.0};
    m2FluidVolumeId tank = NewFluidVolume(world, &pool);
    CHECK(DestroyFluidVolume(world, tank) == m2_success, "the first volume goes");
    revived = 0;
    for (int32_t i = 0; i < 65537; ++i)
    {
        m2FluidVolumeId next = NewFluidVolume(world, &pool);
        revived += m2FluidVolume_IsValid(world, tank) ? 1 : 0;
        CHECK(DestroyFluidVolume(world, next) == m2_success, "each later volume goes");
    }
    CHECK(revived == 0, "a held volume id never names a later volume");
    m2DestroyWorld(world);
}

// A particle is created from a def like every other object: a create
// or fill without a def from m2DefaultParticleDef, or without its out
// argument, is refused as a misuse; a full pool refuses a create with
// m2_errorCapacity and cuts a fill short with it, neither counting
// misuse.
static void TestParticleDefs(void)
{
    m2WorldDef def = FluidWorldDef(4);
    m2World* world = NewWorld(&def);
    m2Polygon tub = m2MakeBox(0.5f, 0.5f);
    m2ParticleDef particleDef = m2DefaultParticleDef();
    m2ParticleDef unset;
    memset(&unset, 0, sizeof(unset));
    m2ParticleId id = {1, 1};
    int32_t count = -1;
    uint64_t misuse = m2World_GetCounters(world).misuse;
    CHECK(m2CreateParticle(world, NULL, &id) == m2_errorInvalid && id.index1 == 0,
          "a create without a def is refused");
    CHECK(m2CreateParticle(world, &unset, &id) == m2_errorInvalid && id.index1 == 0,
          "and one whose def skipped m2DefaultParticleDef");
    CHECK(m2CreateParticle(world, &particleDef, NULL) == m2_errorInvalid,
          "and one without particleOut");
    CHECK(m2CreateParticlesInPolygon(world, NULL, &tub, &count) == m2_errorInvalid && count == 0,
          "a fill without a def is refused");
    count = -1;
    CHECK(m2CreateParticlesInPolygon(world, &unset, &tub, &count) == m2_errorInvalid && count == 0,
          "and one whose def skipped m2DefaultParticleDef");
    count = -1;
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, NULL, &count) == m2_errorInvalid &&
              count == 0,
          "and one without a polygon");
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &tub, NULL) == m2_errorInvalid,
          "and one without countOut");
    CHECK(m2World_GetCounters(world).misuse == misuse + 7 && m2World_GetParticleCount(world) == 0,
          "each counting one misuse and creating nothing");

    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &tub, &count) == m2_errorCapacity &&
              count == 4 && m2World_GetParticleCount(world) == 4,
          "a fill larger than the pool keeps the particles that fit and says it was cut short");
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &tub, &count) == m2_errorCapacity &&
              count == 0,
          "and one into a full pool creates none");
    CHECK(m2CreateParticle(world, &particleDef, &id) == m2_errorCapacity && id.index1 == 0,
          "a create into a full pool is refused");
    CHECK(m2World_GetCounters(world).misuse == misuse + 7, "neither counting misuse");
    m2DestroyWorld(world);
}

// A fill the pool cuts short keeps the particles that fit, returns
// m2_errorCapacity with their count and is recorded: its replay cuts
// short the same way and reaches the same world.
static void TestFillCutShortReplays(void)
{
    m2WorldDef def = FluidWorldDef(8);
    m2World* world = NewWorld(&def);
    for (int32_t i = 0; i < 5; ++i)
    {
        EmitParticle(world, (m2Pos2){(double)i, 3.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    m2World* replayed = NewWorld(&def);
    int64_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    m2Counters before = m2World_GetCounters(world);
    m2Polygon tub = m2MakeBox(0.5f, 0.5f);
    m2ParticleDef particleDef = m2DefaultParticleDef();
    particleDef.flags = m2_springParticle;
    int32_t count = -1;
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &tub, &count) == m2_errorCapacity &&
              count == 3 && m2World_GetParticleCount(world) == 8,
          "a fill into a nearly full pool creates the 3 that fit and says so");
    m2Counters after = m2World_GetCounters(world);
    CHECK(after.misuse == before.misuse && after.particlePoolFull == before.particlePoolFull + 1,
          "counting no misuse and one full pool");
    m2World_Step(world, 1.0f / 60.0f, 4);
    int64_t bytes = StopJournal(world);
    CHECK(bytes > 0 && ReplayJournal(replayed, tape, bytes) &&
              m2World_Hash(replayed) == m2World_Hash(world),
          "and its replay reaches the same world");
    free(tape);
    m2DestroyWorld(replayed);
    m2DestroyWorld(world);
}

int main(void)
{
    TestCrowdedCrossingStops();
    TestCappedCandidatesIgnoreTreeHistory();
    TestLifecycle();
    TestParticleDefs();
    TestFillCutShortReplays();
    TestFarParticlesStep();
    TestOverflowIsQuiet();
    TestPairOverflow();
    TestRelaxation();
    TestBasin();
    TestWaterChainAndSensor();
    TestBuoyancy();
    TestSurfaceTension();
    TestParticleFill();
    TestFillChecksItsPolygon();
    TestJelly();
    TestOverlapParticles();
    TestFluidThreadingParity();
    TestLifetimeAndUserData();
    TestDeadIdsStayDead();
    TestRollbackIdentity();
    TestJournalReplay();
    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
