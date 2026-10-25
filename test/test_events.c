// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Event gate: begin/end symmetry across the full contact lifecycle
// (land, launch, destroy-while-touching, between-step destroy), canonical
// order, restore-clears-buffers with identical re-emission on replay,
// and the event stream hash across CI cells.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2BodyId AddSlab(m2World* world)
{
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId body = NewBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, body, &sd, &slab);
    return body;
}

static void TestLifecycleSymmetry(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    AddSlab(world);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    m2ShapeId ballShape = NewCircleShape(world, ball, &sd, &circle);

    // Fall until begin fires; exactly one begin, no end yet.
    int32_t begins = 0;
    int32_t ends = 0;
    int32_t beginStep = -1;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        begins += ev.beginCount;
        ends += ev.endCount;
        if (ev.beginCount > 0 && beginStep < 0)
        {
            beginStep = i;
            CHECK(ev.beginEvents[0].shapeIdB.index1 == ballShape.index1 ||
                      ev.beginEvents[0].shapeIdA.index1 == ballShape.index1,
                  "begin event names the ball shape");
        }
    }
    CHECK(begins == 1 && ends == 0, "landing produces exactly one begin");

    // Launch it away: exactly one end.
    m2Body_SetLinearVelocity(world, ball, (m2Vec2){0.0f, 30.0f});
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        begins += ev.beginCount;
        ends += ev.endCount;
    }
    CHECK(begins == 1 && ends == 1, "launch produces exactly one end");

    m2DestroyWorld(world);
}

static void TestDestroyBookending(void)
{
    // Destroying a touching body must emit the end event, with the
    // ids as they were - and a between-step destroy must surface in the
    // next drained window, never vanish.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    AddSlab(world);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.85};
    m2BodyId box = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.5f, 0.5f);
    m2ShapeId boxShape = NewPolygonShape(world, box, &sd, &unit);

    int32_t begins = 0;
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        begins += m2World_GetContactEvents(world).beginCount;
    }
    CHECK(begins == 1, "box landed");

    // Destroy BETWEEN steps, while touching.
    uint32_t generationAtDestroy = boxShape.generation;
    DestroyBody(world, box);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2ContactEvents ev = m2World_GetContactEvents(world);
    CHECK(ev.endCount == 1, "destroy-while-touching emits the end event");
    if (ev.endCount == 1)
    {
        bool namesBox = ev.endEvents[0].shapeIdA.index1 == boxShape.index1 ||
                        ev.endEvents[0].shapeIdB.index1 == boxShape.index1;
        CHECK(namesBox, "end event names the destroyed shape");
        uint32_t gen = ev.endEvents[0].shapeIdA.index1 == boxShape.index1
                           ? ev.endEvents[0].shapeIdA.generation
                           : ev.endEvents[0].shapeIdB.generation;
        CHECK(gen == generationAtDestroy, "end event carries the pre-destroy generation");
    }

    m2DestroyWorld(world);
}

static void TestRestoreClearsAndReplays(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    AddSlab(world);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.restitution = 0.6f;
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.0 + 1.4 * (double)i, 1.0 + 0.8 * (double)i};
        m2BodyId body = NewBody(world, &bd);
        m2Circle c = {{0.0f, 0.0f}, 0.35f};
        NewCircleShape(world, body, &sd, &c);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");

    // Record the event stream (bouncy balls churn begins/ends).
    uint64_t streamA = M2_HASH_INIT;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        streamA =
            m2Hash64(streamA, ev.beginEvents, ev.beginCount * (int32_t)sizeof(m2ContactBeginEvent));
        streamA = m2Hash64(streamA, ev.endEvents, ev.endCount * (int32_t)sizeof(m2ContactEndEvent));
    }

    CHECK(Restore(world, snap, size), "restore");
    m2ContactEvents cleared = m2World_GetContactEvents(world);
    CHECK(cleared.beginCount == 0 && cleared.endCount == 0, "restore clears event buffers");

    uint64_t streamB = M2_HASH_INIT;
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents ev = m2World_GetContactEvents(world);
        streamB =
            m2Hash64(streamB, ev.beginEvents, ev.beginCount * (int32_t)sizeof(m2ContactBeginEvent));
        streamB = m2Hash64(streamB, ev.endEvents, ev.endCount * (int32_t)sizeof(m2ContactEndEvent));
    }
    CHECK(streamA == streamB, "re-simulation re-emits the identical event stream");

    free(snap);
    m2DestroyWorld(world);
}

static void TestDestroyShapeBookends(void)
{
    // Destroying one shape of a
    // two-shape body must end its touching contact and lighten the
    // body, while the body itself lives on.
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
    bd.position = (m2Pos2){0.0, 0.55};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon foot = m2MakeBox(0.4f, 0.4f);
    m2ShapeId footId = NewPolygonShape(world, body, &sd, &foot);
    m2Circle hat = {{0.0f, 0.9f}, 0.3f};
    m2ShapeId hatId = NewCircleShape(world, body, &sd, &hat);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    // Destroy the touching foot between steps.
    DestroyShape(world, footId);
    CHECK(!m2Shape_IsValid(world, footId), "destroyed shape id dies");
    CHECK(m2Shape_IsValid(world, hatId), "sibling shape lives");
    CHECK(m2Body_IsValid(world, body), "body survives losing a shape");

    m2World_Step(world, 1.0f / 60.0f, 4);
    m2ContactEvents events = m2World_GetContactEvents(world);
    bool sawEnd = false;
    for (int32_t i = 0; i < events.endCount; ++i)
    {
        sawEnd = sawEnd || events.endEvents[i].shapeIdA.index1 == footId.index1 ||
                 events.endEvents[i].shapeIdB.index1 == footId.index1;
    }
    CHECK(sawEnd, "destroying a touching shape emits its end event");

    m2DestroyWorld(world);
}

static void TestTypeChangeBookends(void)
{
    // Two touching dynamics; turn BOTH static and the pair stops
    // making sense, and its end must be witnessed.
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
    bd.position = (m2Pos2){0.0, 0.55};
    m2BodyId a = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    m2ShapeId shapeA = NewPolygonShape(world, a, &sd, &unit);
    bd.position = (m2Pos2){0.0, 1.37};
    m2BodyId b = NewBody(world, &bd);
    m2ShapeId shapeB = NewPolygonShape(world, b, &sd, &unit);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2Body_SetType(world, a, m2_staticBody);
    m2Body_SetType(world, b, m2_staticBody);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2ContactEvents events = m2World_GetContactEvents(world);
    bool sawEnd = false;
    for (int32_t i = 0; i < events.endCount; ++i)
    {
        bool hasA = events.endEvents[i].shapeIdA.index1 == shapeA.index1 ||
                    events.endEvents[i].shapeIdB.index1 == shapeA.index1;
        bool hasB = events.endEvents[i].shapeIdA.index1 == shapeB.index1 ||
                    events.endEvents[i].shapeIdB.index1 == shapeB.index1;
        sawEnd = sawEnd || (hasA && hasB);
    }
    CHECK(sawEnd, "a pair that stops making sense emits its end");

    m2DestroyWorld(world);
}

static void TestSensors(void)
{
    // A checkpoint zone: the runner falls through it (no response),
    // the zone reports enter and exit on the SENSOR stream only, the
    // contact stream never hears about it, and a body parked inside a
    // sensor still earns its sleep.
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

    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){0.0, 3.0};
    m2BodyId zoneBody = NewBody(world, &zd);
    m2ShapeDef zs = m2DefaultShapeDef();
    zs.isSensor = true;
    m2Polygon zone = m2MakeBox(1.5f, 0.5f);
    m2ShapeId zoneId = NewPolygonShape(world, zoneBody, &zs, &zone);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 6.0};
    m2BodyId runner = NewBody(world, &bd);
    m2ShapeDef rs = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.3f};
    NewCircleShape(world, runner, &rs, &ball);

    bool sawEnter = false;
    bool sawExit = false;
    bool contactStreamPolluted = false;
    int32_t enterPointCount = -1;
    double enterPointY = 0.0;
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2SensorEvents sensor = m2World_GetSensorEvents(world);
        for (int32_t k = 0; k < sensor.beginCount; ++k)
        {
            bool isZone = sensor.beginEvents[k].shapeIdA.index1 == zoneId.index1 ||
                          sensor.beginEvents[k].shapeIdB.index1 == zoneId.index1;
            sawEnter = sawEnter || isZone;
            if (isZone && enterPointCount < 0)
            {
                // The overlap hit point rides the begin event.
                enterPointCount = sensor.beginEvents[k].pointCount;
                enterPointY = sensor.beginEvents[k].points[0].y;
            }
        }
        for (int32_t k = 0; k < sensor.endCount; ++k)
        {
            sawExit = sawExit || sensor.endEvents[k].shapeIdA.index1 == zoneId.index1 ||
                      sensor.endEvents[k].shapeIdB.index1 == zoneId.index1;
        }
        m2ContactEvents contact = m2World_GetContactEvents(world);
        for (int32_t k = 0; k < contact.beginCount; ++k)
        {
            contactStreamPolluted = contactStreamPolluted ||
                                    contact.beginEvents[k].shapeIdA.index1 == zoneId.index1 ||
                                    contact.beginEvents[k].shapeIdB.index1 == zoneId.index1;
        }
    }
    CHECK(sawEnter, "the zone reports the runner entering");
    CHECK(sawExit, "and leaving out the bottom");
    CHECK(!contactStreamPolluted, "the contact stream never hears about sensors");
    CHECK(m2Body_GetPosition(world, runner).y < 1.0, "the runner fell straight through the zone");
    CHECK(enterPointCount > 0, "the sensor begin carries the overlap hit point");
    CHECK(enterPointY > 2.0 && enterPointY < 4.0, "the hit point sits at the zone overlap");

    // A box parked inside a sensor zone still sleeps (sensors never
    // hold anyone awake).
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){5.0, 0.6};
    m2BodyId parkZoneBody = NewBody(world, &pd);
    m2ShapeDef ps = m2DefaultShapeDef();
    ps.isSensor = true;
    m2Polygon parkZone = m2MakeBox(1.0f, 1.0f);
    NewPolygonShape(world, parkZoneBody, &ps, &parkZone);
    bd.position = (m2Pos2){5.0, 0.45};
    m2BodyId parked = NewBody(world, &bd);
    m2ShapeDef qs = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, parked, &qs, &unit);
    bool slept = false;
    for (int32_t i = 0; i < 300 && !slept; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        slept = !m2Body_IsAwake(world, parked);
    }
    CHECK(slept, "a body parked inside a sensor still sleeps");

    // For sensors: destroy the park zone while overlapped - the
    // sensor end must be witnessed in the next window.
    DestroyBody(world, parkZoneBody);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2SensorEvents after = m2World_GetSensorEvents(world);
    bool sawDestroyEnd = false;
    for (int32_t k = 0; k < after.endCount; ++k)
    {
        sawDestroyEnd = true;
    }
    CHECK(sawDestroyEnd, "destroying an overlapped sensor emits its end");

    m2DestroyWorld(world);
}

static void TestBeginEventPayload(void)
{
    // A ball dropped from a known height: the begin event must say
    // where it hit, which way, and how hard - and the physics answer
    // sqrt(2gh) is checkable to a few percent.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){50.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(10.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){50.0, 2.25}; // ball bottom 2m above the floor top
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    NewCircleShape(world, ball, &bs, &circle);

    bool sawBegin = false;
    m2ContactBeginEvent hit;
    memset(&hit, 0, sizeof(hit));
    for (int32_t i = 0; i < 120 && !sawBegin; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents events = m2World_GetContactEvents(world);
        if (events.beginCount > 0)
        {
            hit = events.beginEvents[0];
            sawBegin = true;
        }
    }
    CHECK(sawBegin, "the drop produces a begin event");
    CHECK(hit.pointCount >= 1, "the event carries contact points");
    float ny = hit.normal.y;
    CHECK(ny == 1.0f || ny == -1.0f, "the normal is vertical");
    CHECK(hit.points[0].x > 49.9 && hit.points[0].x < 50.1, "the hit lands where it should");
    CHECK(hit.points[0].y > -0.15 && hit.points[0].y < 0.15, "on the floor's surface");
    // v = sqrt(2*10*2) = 6.32 m/s; speculative contact fires a touch
    // early, so accept a small undershoot band.
    CHECK(hit.approachSpeed > 5.6f && hit.approachSpeed < 6.6f,
          "approach speed tells the impact story");

    m2DestroyWorld(world);
}

static void TestSensorOverlapQuery(void)
{
    // Two visitors inside the zone, one outside: the list names
    // exactly the two, in canonical order, with a truthful total.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    def.bodyCapacity = 8;

    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){0.0, 0.0};
    m2BodyId zoneBody = NewBody(world, &zd);
    m2ShapeDef zs = m2DefaultShapeDef();
    zs.isSensor = true;
    m2Polygon zone = m2MakeBox(2.0f, 2.0f);
    m2ShapeId zoneId = NewPolygonShape(world, zoneBody, &zs, &zone);

    m2ShapeId visitors[3];
    double xs[3] = {-0.5, 0.5, 10.0}; // third is far away
    for (int32_t i = 0; i < 3; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_kinematicBody; // hang in place, no gravity fall
        bd.position = (m2Pos2){xs[i], 0.0};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle c = {{0.0f, 0.0f}, 0.3f};
        visitors[i] = NewCircleShape(world, body, &sd, &c);
    }
    for (int32_t i = 0; i < 5; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    m2ShapeId inside[4];
    int32_t total = m2Shape_GetSensorOverlaps(world, zoneId, inside, 4);
    CHECK(total == 2, "two visitors inside");
    CHECK(inside[0].index1 == visitors[0].index1 && inside[1].index1 == visitors[1].index1,
          "canonical order");
    CHECK(m2Shape_GetSensorOverlaps(world, zoneId, inside, 1) == 2, "truthful beyond capacity");
    CHECK(m2Shape_GetSensorOverlaps(world, visitors[0], inside, 4) == 0,
          "a non-sensor shape reports nothing");

    m2DestroyWorld(world);
}

int main(void)
{
    TestSensorOverlapQuery();
    TestBeginEventPayload();
    TestSensors();
    TestTypeChangeBookends();
    TestDestroyShapeBookends();
    TestLifecycleSymmetry();
    TestDestroyBookending();
    TestRestoreClearsAndReplays();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
