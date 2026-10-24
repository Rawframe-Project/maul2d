// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Island/sleep gate: a settled pyramid must sleep with zeroed
// velocities and frozen positions; waking is island-transitive; a
// kinematic sweep into a sleeping stack wakes it (end to
// end); sleep counters survive rollback bit-exactly; and the sleep and
// wake cycle hash crosses CI cells.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2BodyId AddSlab(m2World* world, double y)
{
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, y};
    m2BodyId body = NewBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, body, &sd, &slab);
    return body;
}

static m2BodyId AddBox(m2World* world, double x, double y)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, body, &sd, &unit);
    return body;
}

static void TestPyramidSleeps(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    AddSlab(world, -0.5);

    m2BodyId boxes[6];
    int32_t count = 0;
    for (int32_t row = 0; row < 3; ++row)
    {
        for (int32_t col = row; col < 3; ++col)
        {
            boxes[count++] = AddBox(world, (double)col - 0.5 * (double)(3 + row) + 0.5,
                                    0.55 + 1.02 * (double)row);
        }
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    for (int32_t i = 0; i < count; ++i)
    {
        CHECK(!m2Body_IsAwake(world, boxes[i]), "pyramid box sleeps");
        m2Vec2 v = m2Body_GetLinearVelocity(world, boxes[i]);
        CHECK(v.x == 0.0f && v.y == 0.0f, "sleeping velocity zeroed");
    }

    // Frozen: positions must not change at all while asleep.
    m2Pos2 before = m2Body_GetPosition(world, boxes[0]);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 after = m2Body_GetPosition(world, boxes[0]);
    CHECK(before.x == after.x && before.y == after.y, "sleeping bodies are frozen");

    // Island-transitive wake: shove the top box; the whole pile wakes.
    m2Body_SetLinearVelocity(world, boxes[count - 1], (m2Vec2){2.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    int32_t awakeCount = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        awakeCount += m2Body_IsAwake(world, boxes[i]) ? 1 : 0;
    }
    CHECK(awakeCount == count, "waking is island-transitive");

    m2DestroyWorld(world);
}

static void TestKinematicWakesSleepers(void)
{
    // End to end: a stack sleeps, a kinematic platform
    // sweeps into it, the stack must wake and get pushed.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    AddSlab(world, -0.5);
    m2BodyId a = AddBox(world, 0.0, 0.55);
    m2BodyId b = AddBox(world, 0.0, 1.57);

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, a) && !m2Body_IsAwake(world, b), "stack asleep before the sweep");

    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){-6.0, 0.55};
    kd.linearVelocity = (m2Vec2){8.0f, 0.0f};
    m2BodyId sweeper = NewBody(world, &kd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon blade = m2MakeBox(0.5f, 0.4f);
    NewPolygonShape(world, sweeper, &sd, &blade);

    bool woke = false;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (m2Body_IsAwake(world, a))
        {
            woke = true;
            break;
        }
    }
    CHECK(woke, "kinematic sweep wakes the sleeping stack");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, a).x > 0.2, "woken box actually gets pushed");

    m2DestroyWorld(world);
}

static void TestSleepRollback(void)
{
    // Snapshot mid-doze (counters running), restore, replay: the sleep
    // moment must land on the identical step, under rollback.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    AddSlab(world, -0.5);
    m2BodyId box = AddBox(world, 0.0, 1.2);

    for (int32_t i = 0; i < 45; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // landed, counters counting
    }
    CHECK(m2Body_IsAwake(world, box), "still dozing off at snapshot time");

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");

    int32_t sleepStepA = -1;
    uint64_t hashes[120];
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
        if (sleepStepA < 0 && !m2Body_IsAwake(world, box))
        {
            sleepStepA = i;
        }
    }
    CHECK(sleepStepA >= 0, "box eventually sleeps");

    CHECK(Restore(world, snap, size), "restore");
    int32_t sleepStepB = -1;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "sleep cycle replays bit-exactly");
        if (sleepStepB < 0 && !m2Body_IsAwake(world, box))
        {
            sleepStepB = i;
        }
    }
    CHECK(sleepStepA == sleepStepB, "sleep lands on the identical step after rollback");

    free(snap);
    m2DestroyWorld(world);
}

static void TestTowerSleeps(void)
{
    // Regression tripwire for solver-schedule changes: a plain 8-box
    // tower must fall asleep promptly. Some solver schedules leave
    // exactly this scene grinding forever.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.friction = 0.6f;
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2BodyId boxes[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.0, 0.55 + 1.02 * (double)i};
        boxes[i] = NewBody(world, &bd);
        NewPolygonShape(world, boxes[i], &sd, &unit);
    }
    int32_t sleepStep = -1;
    for (int32_t step = 0; step < 300 && sleepStep < 0; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        bool allAsleep = true;
        for (int32_t i = 0; i < 8; ++i)
        {
            allAsleep = allAsleep && !m2Body_IsAwake(world, boxes[i]);
        }
        if (allAsleep)
        {
            sleepStep = step;
        }
    }
    CHECK(sleepStep >= 0 && sleepStep < 250, "an eight-box tower sleeps promptly");
    m2DestroyWorld(world);
}

static void TestTeleportWakesBystanders(void)
{
    // Pull the support out from under a sleeping stack by teleporting
    // it away: the sleepers must wake and fall, not float.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    AddSlab(world, -0.5);
    m2BodyId support = AddBox(world, 0.0, 0.55);
    m2BodyId rider = AddBox(world, 0.0, 1.57);

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, support) && !m2Body_IsAwake(world, rider), "stack asleep");
    double riderY = m2Body_GetPosition(world, rider).y;

    m2Body_SetTransform(world, support, (m2Pos2){50.0, 0.55}, (m2Rot){1.0f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, rider), "the rider notices its support vanish");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, rider).y < riderY - 0.5, "and falls");

    m2DestroyWorld(world);
}

int main(void)
{
    TestTeleportWakesBystanders();
    TestTowerSleeps();
    TestPyramidSleeps();
    TestKinematicWakesSleepers();
    TestSleepRollback();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
