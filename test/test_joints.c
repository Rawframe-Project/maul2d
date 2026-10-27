// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint gate: a distance-joint pendulum holds its rod length through the
// swing; a revolute chain keeps its pivots pinned; joints couple sleep
// islands without any contact; destroying a body kills its joints and
// wakes the counterpart; joint impulses replay under rollback; and the
// chain-swing hash crosses CI cells.

#include "helpers.h"
#include "test_harness.h"
#include "test_task_pool.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2BodyId AddBall(m2World* world, double x, double y, float radius)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, radius};
    NewCircleShape(world, body, &sd, &c);
    return body;
}

static double Distance(m2Pos2 a, m2Pos2 b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return dx * dx + dy * dy;
}

static void TestDistancePendulum(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 5.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyId bob = AddBall(world, 2.0, 5.0, 0.2f); // horizontal start: big swing

    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    m2JointId joint = NewDistanceJoint(world, &jd);
    CHECK(m2Joint_IsValid(world, joint), "distance joint created");

    double worst = 0.0;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double lenSq = Distance(m2Body_GetPosition(world, anchor), m2Body_GetPosition(world, bob));
        double err = lenSq - 4.0; // rod length 2 derived at create
        err = err < 0.0 ? -err : err;
        worst = err > worst ? err : worst;
    }
    // |len^2 - 4| < 0.12 keeps the rod within ~1.5% of its length.
    CHECK(worst < 0.12, "pendulum rod length holds through the swing");

    m2DestroyWorld(world);
}

static void TestDistanceRope(void)
{
    // enableSpring with zero stiffness and a maxLength is a slack rope:
    // the ball falls freely until the rope goes taut at maxLength, then
    // hangs there. A rigid joint (the default) would pin it at length.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 10.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyId ball = AddBall(world, 0.0, 9.0, 0.2f); // one below the anchor

    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = ball;
    jd.enableSpring = true; // rope: free until a limit bites
    jd.springHertz = 0.0f;
    jd.maxLength = 3.0f;
    m2JointId rope = NewDistanceJoint(world, &jd);
    CHECK(m2Joint_IsValid(world, rope), "rope joint created");

    // Early: still falling, so it has dropped below its start. A rigid
    // joint would have pinned it at the spawn length of one.
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, ball).y < 8.5, "the slack rope lets the ball fall freely");

    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double d = sqrt(Distance(m2Body_GetPosition(world, anchor), m2Body_GetPosition(world, ball)));
    CHECK(d > 2.9 && d < 3.05, "the taut rope hangs at maxLength, not past it");
    m2DestroyWorld(world);

    // Contrast: the default joint is a rigid rod that holds the spawn length.
    world = NewWorld(&def);
    anchor = NewBody(world, &ad);
    ball = AddBall(world, 0.0, 9.0, 0.2f);
    m2DistanceJointDef rd = m2DefaultDistanceJointDef();
    rd.bodyIdA = anchor;
    rd.bodyIdB = ball; // default: rigid at the spawn length of one
    NewDistanceJoint(world, &rd);
    for (int32_t i = 0; i < 100; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double rigid =
        sqrt(Distance(m2Body_GetPosition(world, anchor), m2Body_GetPosition(world, ball)));
    CHECK(rigid < 1.1, "the rigid joint holds the spawn length");
    m2DestroyWorld(world);

    // Determinism: snapshot round-trip and a worker-count twin, bit-exact.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef dw = m2DefaultWorldDef();
        dw.bodyCapacity = 8;
        dw.shapeCapacity = 8;
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&dw, &s_twinWorkers[wc]);
        m2World* w = NewWorld(&dw);
        m2BodyId a = NewBody(w, &ad);
        m2BodyId b = AddBall(w, 0.0, 9.0, 0.2f);
        m2DistanceJointDef j2 = m2DefaultDistanceJointDef();
        j2.bodyIdA = a;
        j2.bodyIdB = b;
        j2.enableSpring = true;
        j2.springHertz = 0.0f;
        j2.maxLength = 3.0f;
        NewDistanceJoint(w, &j2);
        for (int32_t i = 0; i < 40; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        int32_t size = m2World_SnapshotSize(w);
        void* snap = malloc((size_t)size);
        CHECK(Snapshot(w, snap, size) == size, "rope snapshot fits");
        for (int32_t i = 0; i < 60; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        uint64_t after = m2World_Hash(w);
        CHECK(Restore(w, snap, size), "rope snapshot restores");
        for (int32_t i = 0; i < 60; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(w) == after, "rope timeline replays bit for bit");
        hashes[wc] = after;
        free(snap);
        m2DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "rope is worker-count deterministic");
}

static void TestMotorSpring(void)
{
    // A spring-driven motor (hertz > 0): the platform eases to the
    // offset instead of snapping, and still holds it against gravity.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){0.0, 4.0};
    m2BodyId platform = NewBody(world, &pd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.5f, 0.1f);
    NewPolygonShape(world, platform, &sd, &deck);

    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = anchor;
    mj.bodyIdB = platform;
    mj.linearOffset = (m2Vec2){2.0f, 1.0f};
    mj.maxForce = 500.0f;
    mj.maxTorque = 50.0f;
    mj.springHertz = 3.0f; // soft spring drive
    mj.springDampingRatio = 1.0f;
    NewMotorJoint(world, &mj);

    // Early: the soft drive is still easing in, not snapped to target.
    for (int32_t i = 0; i < 6; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, platform).x < 1.5, "the soft spring eases in, not snaps");

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 at = m2Body_GetPosition(world, platform);
    CHECK(at.x > 1.8 && at.x < 2.2, "the spring motor settles at the x offset");
    CHECK(at.y > 4.7 && at.y < 5.2, "and holds y against gravity");
    m2DestroyWorld(world);

    // Determinism: snapshot round-trip and a worker-count twin, bit-exact.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef dw = m2DefaultWorldDef();
        dw.bodyCapacity = 8;
        dw.shapeCapacity = 8;
        dw.jointCapacity = 4;
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&dw, &s_twinWorkers[wc]);
        m2World* w = NewWorld(&dw);
        m2BodyId a = NewBody(w, &ad);
        m2BodyId p = NewBody(w, &pd);
        m2ShapeDef s2 = m2DefaultShapeDef();
        m2Polygon d2 = m2MakeBox(0.5f, 0.1f);
        NewPolygonShape(w, p, &s2, &d2);
        m2MotorJointDef mj2 = m2DefaultMotorJointDef();
        mj2.bodyIdA = a;
        mj2.bodyIdB = p;
        mj2.linearOffset = (m2Vec2){2.0f, 1.0f};
        mj2.maxForce = 500.0f;
        mj2.maxTorque = 50.0f;
        mj2.springHertz = 3.0f;
        mj2.springDampingRatio = 1.0f;
        NewMotorJoint(w, &mj2);
        for (int32_t i = 0; i < 30; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        int32_t size = m2World_SnapshotSize(w);
        void* snap = malloc((size_t)size);
        CHECK(Snapshot(w, snap, size) == size, "spring motor snapshot fits");
        for (int32_t i = 0; i < 60; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        uint64_t after = m2World_Hash(w);
        CHECK(Restore(w, snap, size), "spring motor snapshot restores");
        for (int32_t i = 0; i < 60; ++i)
        {
            m2World_Step(w, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(w) == after, "spring motor replays bit for bit");
        hashes[wc] = after;
        free(snap);
        m2DestroyWorld(w);
    }
    CHECK(hashes[0] == hashes[1], "spring motor is worker-count deterministic");
}

static void TestRevoluteChain(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 8.0};
    m2BodyId previous = NewBody(world, &ad);

    m2BodyId links[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        links[i] = AddBall(world, 1.0 + (double)i, 8.0, 0.15f);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = previous;
        jd.bodyIdB = links[i];
        jd.localAnchorA = i == 0 ? (m2Vec2){0.5f, 0.0f} : (m2Vec2){0.5f, 0.0f};
        jd.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        // This scene predates enableConnectedCollision and its gap band assumes
        // adjacent links bounce off each other; opt into that explicitly.
        jd.enableConnectedCollision = true;
        CHECK(m2Joint_IsValid(world, NewRevoluteJoint(world, &jd)), "link joint created");
        previous = links[i];
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // Pivot integrity: each pair of pinned anchors stays together within a
    // soft tolerance while the chain swings and folds.
    for (int32_t i = 1; i < 4; ++i)
    {
        m2Pos2 pinA = m2Body_GetWorldPoint(world, links[i - 1], (m2Vec2){0.5f, 0.0f});
        m2Pos2 pinB = m2Body_GetWorldPoint(world, links[i], (m2Vec2){-0.5f, 0.0f});
        CHECK(Distance(pinA, pinB) < 0.05 * 0.05, "revolute pivots hold the chain together");
    }

    m2DestroyWorld(world);
}

static void TestJointsCoupleIslands(void)
{
    // Two balls joined by a rod, never touching each other, resting on
    // separate pillars: they must sleep as one island and wake as one.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);

    for (int32_t i = 0; i < 2; ++i)
    {
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){3.0 * (double)i, 0.0};
        m2BodyId pillar = NewBody(world, &pd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon top = m2MakeBox(0.4f, 0.4f);
        NewPolygonShape(world, pillar, &sd, &top);
    }
    m2BodyId left = AddBall(world, 0.0, 0.62, 0.2f);
    m2BodyId right = AddBall(world, 3.0, 0.62, 0.2f);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = left;
    jd.bodyIdB = right;
    NewDistanceJoint(world, &jd);

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, left) && !m2Body_IsAwake(world, right),
          "joined pair sleeps together");

    m2Body_SetLinearVelocity(world, left, (m2Vec2){0.5f, 0.0f});
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, right), "wake crosses the joint, no contact needed");

    // Destroy one end: the joint dies, the survivor wakes.
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    DestroyBody(world, left);
    CHECK(m2Body_IsAwake(world, right), "destroying a jointed body wakes the counterpart");

    m2DestroyWorld(world);
}

static void TestJointRollback(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 6.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyId bob = AddBall(world, 1.5, 6.0, 0.2f);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    NewDistanceJoint(world, &jd);

    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // mid-swing, impulses loaded
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
        CHECK(m2World_Hash(world) == hashes[i], "pendulum replays bit-exactly");
    }
    free(snap);
    m2DestroyWorld(world);
}

static float BodyAngle(const m2World* world, m2BodyId body)
{
    m2Rot q = m2Body_GetRotation(world, body);
    return m2Atan2(q.s, q.c);
}

static void TestRevoluteMotor(void)
{
    // A strong motor spins a pinned wheel up to its target speed; a
    // starved motor cannot (the torque budget is real).
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 3.0};
    m2BodyId base = NewBody(world, &ad);
    m2BodyId wheel = AddBall(world, 0.0, 3.0, 0.5f);
    m2BodyId starvedWheel = AddBall(world, 5.0, 3.0, 0.5f);
    m2BodyDef ad2 = m2DefaultBodyDef();
    ad2.position = (m2Pos2){5.0, 3.0};
    m2BodyId base2 = NewBody(world, &ad2);

    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = base;
    jd.bodyIdB = wheel;
    jd.enableMotor = true;
    jd.motorSpeed = 4.0f;
    jd.maxMotorTorque = 50.0f;
    NewRevoluteJoint(world, &jd);

    m2RevoluteJointDef weak = m2DefaultRevoluteJointDef();
    weak.bodyIdA = base2;
    weak.bodyIdB = starvedWheel;
    weak.enableMotor = true;
    weak.motorSpeed = 4.0f;
    weak.maxMotorTorque = 0.0001f;
    NewRevoluteJoint(world, &weak);

    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w = m2Body_GetAngularVelocity(world, wheel);
    CHECK(w > 3.9f && w < 4.1f, "motor reaches target speed");
    float starved = m2Body_GetAngularVelocity(world, starvedWheel);
    CHECK(starved < 1.0f, "starved motor stays under budget");

    m2DestroyWorld(world);
}

static void TestRevoluteLimit(void)
{
    // A horizontal rod on a limited hinge: gravity swings it down, the
    // lower limit catches it and it rests there without sagging through.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 5.0};
    m2BodyId pivot = NewBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.6, 5.0};
    m2BodyId rod = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon bar = m2MakeBox(0.6f, 0.05f);
    NewPolygonShape(world, rod, &sd, &bar);

    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = pivot;
    jd.bodyIdB = rod;
    jd.localAnchorB = (m2Vec2){-0.6f, 0.0f};
    jd.enableLimit = true;
    jd.lowerAngle = -0.5f;
    jd.upperAngle = 0.5f;
    NewRevoluteJoint(world, &jd);

    float minAngle = 0.0f;
    for (int32_t i = 0; i < 400; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        float angle = BodyAngle(world, rod);
        minAngle = angle < minAngle ? angle : minAngle;
    }
    CHECK(minAngle > -0.55f, "limit never overshoots badly");
    float rest = BodyAngle(world, rod);
    CHECK(rest > -0.55f && rest < -0.45f, "rod rests on the lower limit");

    m2DestroyWorld(world);
}

static void TestPrismaticSlider(void)
{
    // Gravity elevator: the axis lock kills sideways drift and spin,
    // the lower translation limit catches the fall.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId frame = NewBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 4.0};
    m2BodyId platform = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.5f, 0.1f);
    NewPolygonShape(world, platform, &sd, &deck);

    m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
    jd.bodyIdA = frame;
    jd.bodyIdB = platform;
    jd.localAxisA = (m2Vec2){0.0f, 1.0f};
    jd.enableLimit = true;
    jd.lowerTranslation = -1.5f;
    jd.upperTranslation = 2.0f;
    NewPrismaticJoint(world, &jd);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pos = m2Body_GetPosition(world, platform);
    CHECK(pos.y > 4.0 - 1.55 && pos.y < 4.0 - 1.42, "platform rests on the lower stop");
    CHECK(pos.x > -0.01 && pos.x < 0.01, "axis lock kills sideways drift");
    m2Rot q = m2Body_GetRotation(world, platform);
    CHECK(q.s > -0.01f && q.s < 0.01f, "angle lock kills spin");

    m2DestroyWorld(world);
}

static void TestPrismaticMotor(void)
{
    // The motor hoists the platform against gravity to the upper stop
    // and holds it there.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 1.0};
    m2BodyId frame = NewBody(world, &ad);

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 1.0};
    m2BodyId platform = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.5f, 0.1f);
    NewPolygonShape(world, platform, &sd, &deck);

    m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
    jd.bodyIdA = frame;
    jd.bodyIdB = platform;
    jd.localAxisA = (m2Vec2){0.0f, 1.0f};
    jd.enableMotor = true;
    jd.motorSpeed = 1.5f;
    jd.maxMotorForce = 50.0f;
    jd.enableLimit = true;
    jd.lowerTranslation = -0.5f;
    jd.upperTranslation = 2.0f;
    NewPrismaticJoint(world, &jd);

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Vec2 vel = m2Body_GetLinearVelocity(world, platform);
    CHECK(vel.y > 1.3f && vel.y < 1.7f, "elevator cruises at motor speed");

    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pos = m2Body_GetPosition(world, platform);
    CHECK(pos.y > 1.0 + 1.95 && pos.y < 1.0 + 2.05, "elevator holds at the upper stop");

    m2DestroyWorld(world);
}

static void TestWeldRigid(void)
{
    // Two boxes welded into a bar, dropped onto a slab: the weld holds
    // through the impact and the pair lands as one rigid piece.
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

    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(0.4f, 0.4f);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){-0.4, 3.0};
    m2BodyId left = NewBody(world, &bd);
    NewPolygonShape(world, left, &sd, &box);
    bd.position = (m2Pos2){0.4, 3.0};
    m2BodyId right = NewBody(world, &bd);
    NewPolygonShape(world, right, &sd, &box);

    m2WeldJointDef wd = m2DefaultWeldJointDef();
    wd.bodyIdA = left;
    wd.bodyIdB = right;
    wd.localAnchorA = (m2Vec2){0.4f, 0.0f};
    wd.localAnchorB = (m2Vec2){-0.4f, 0.0f};
    NewWeldJoint(world, &wd);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float relAngle = BodyAngle(world, right) - BodyAngle(world, left);
    CHECK(relAngle > -0.01f && relAngle < 0.01f, "weld holds the relative angle");
    m2Pos2 pl = m2Body_GetPosition(world, left);
    m2Pos2 pr = m2Body_GetPosition(world, right);
    double gap = pr.x - pl.x;
    CHECK(gap > 0.78 && gap < 0.82, "weld holds the separation");
    CHECK(pl.y > 0.35 && pl.y < 0.45, "welded bar rests on the slab");

    m2DestroyWorld(world);
}

static void TestWheelSuspension(void)
{
    // A chassis on one sprung wheel: drop it, the spring absorbs the
    // fall and the chassis settles above the wheel inside the travel
    // limits instead of slamming rigidly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 2.5};
    m2BodyId chassis = NewBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 2.0f;
    m2Polygon deck = m2MakeBox(0.8f, 0.15f);
    NewPolygonShape(world, chassis, &cs, &deck);

    m2BodyId wheel = AddBall(world, 0.0, 1.9, 0.3f);

    m2WheelJointDef wd = m2DefaultWheelJointDef();
    wd.bodyIdA = chassis;
    wd.bodyIdB = wheel;
    wd.localAnchorA = (m2Vec2){0.0f, -0.6f};
    wd.localAxisA = (m2Vec2){0.0f, 1.0f};
    wd.springHertz = 3.0f;
    wd.springDampingRatio = 0.7f;
    wd.enableLimit = true;
    wd.lowerTranslation = -0.25f;
    wd.upperTranslation = 0.25f;
    NewWheelJoint(world, &wd);

    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 wheelPos = m2Body_GetPosition(world, wheel);
    m2Pos2 chassisPos = m2Body_GetPosition(world, chassis);
    CHECK(wheelPos.y > 0.25 && wheelPos.y < 0.35, "wheel rests on the ground");
    double ride = chassisPos.y - wheelPos.y;
    CHECK(ride > 0.3 && ride < 0.9, "spring carries the chassis inside travel");
    CHECK(chassisPos.x > -0.05 && chassisPos.x < 0.05, "suspension does not walk sideways");

    m2DestroyWorld(world);
}

static void TestWheelCar(void)
{
    // The showcase: a two-wheel car drives itself with wheel-joint
    // motors - suspension, friction and the motor all working together.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId road = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    fs.friction = 0.9f;
    m2Polygon slab = m2MakeBox(60.0f, 0.5f);
    NewPolygonShape(world, road, &fs, &slab);

    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.0};
    m2BodyId chassis = NewBody(world, &cd);
    m2ShapeDef cs = m2DefaultShapeDef();
    cs.density = 1.0f;
    m2Polygon deck = m2MakeBox(0.9f, 0.15f);
    NewPolygonShape(world, chassis, &cs, &deck);

    m2BodyId wheels[2];
    double xs[2] = {-0.6, 0.6};
    for (int32_t i = 0; i < 2; ++i)
    {
        wheels[i] = AddBall(world, xs[i], 0.55, 0.3f);
        m2WheelJointDef wd = m2DefaultWheelJointDef();
        wd.bodyIdA = chassis;
        wd.bodyIdB = wheels[i];
        wd.localAnchorA = (m2Vec2){(float)xs[i], -0.45f};
        wd.localAxisA = (m2Vec2){0.0f, 1.0f};
        wd.springHertz = 4.0f;
        wd.springDampingRatio = 0.7f;
        wd.enableLimit = true;
        wd.lowerTranslation = -0.2f;
        wd.upperTranslation = 0.2f;
        wd.enableMotor = true;
        wd.motorSpeed = -10.0f; // clockwise spin rolls the car +x
        wd.maxMotorTorque = 15.0f;
        NewWheelJoint(world, &wd);
    }

    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // settle onto the road
    }
    double startX = m2Body_GetPosition(world, chassis).x;
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double traveled = m2Body_GetPosition(world, chassis).x - startX;
    CHECK(traveled > 3.0, "the car drives itself forward");
    m2Rot q = m2Body_GetRotation(world, chassis);
    CHECK(q.c > 0.95f, "chassis stays upright while driving");

    m2DestroyWorld(world);
}

static void TestJointRuntimeTuning(void)
{
    // Setters retarget a running motor, toggle limits, and wake
    // sleeping machines.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 3.0};
    m2BodyId base = NewBody(world, &ad);
    m2BodyId wheel = AddBall(world, 0.0, 3.0, 0.5f);
    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = base;
    jd.bodyIdB = wheel;
    jd.enableMotor = true;
    jd.motorSpeed = 3.0f;
    jd.maxMotorTorque = 50.0f;
    m2JointId motor = NewRevoluteJoint(world, &jd);

    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w1 = m2Body_GetAngularVelocity(world, wheel);
    CHECK(w1 > 2.9f && w1 < 3.1f, "initial motor speed reached");

    m2Joint_SetMotorSpeed(world, motor, -5.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float w2 = m2Body_GetAngularVelocity(world, wheel);
    CHECK(w2 > -5.1f && w2 < -4.9f, "retargeted motor speed reached");

    m2Joint_EnableMotor(world, motor, false);
    m2Joint_EnableLimit(world, motor, true);
    m2Joint_SetLimits(world, motor, -0.2f, 0.2f);
    bool settled = false;
    for (int32_t i = 0; i < 300 && !settled; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        settled = !m2Body_IsAwake(world, wheel);
    }
    CHECK(settled, "limited free wheel eventually sleeps inside its limits");

    // A setter on a sleeping machine wakes it.
    m2Joint_SetMotorSpeed(world, motor, 1.0f);
    m2Joint_EnableMotor(world, motor, true);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, wheel), "runtime tuning wakes the machine");

    m2DestroyWorld(world);
}

static void TestSoftWeld(void)
{
    // Twin cantilevers off static walls: the rigid weld holds its
    // angle, the angular-soft weld droops under the same load like the
    // spring it now is.
    m2WorldDef def = m2DefaultWorldDef();
    def.gravity = (m2Vec2){0.0f, -10.0f};
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    for (int32_t soft = 0; soft < 2; ++soft)
    {
        double x = soft == 0 ? 0.0 : 10.0;
        m2BodyDef wd = m2DefaultBodyDef();
        wd.position = (m2Pos2){x, 4.0};
        m2BodyId wall = NewBody(world, &wd);
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){x + 0.9, 4.0};
        m2BodyId beam = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 2.0f;
        m2Polygon bar = m2MakeBox(0.9f, 0.06f);
        NewPolygonShape(world, beam, &sd, &bar);
        m2WeldJointDef weld = m2DefaultWeldJointDef();
        weld.bodyIdA = wall;
        weld.bodyIdB = beam;
        weld.localAnchorB = (m2Vec2){-0.9f, 0.0f};
        if (soft == 1)
        {
            weld.angularSpringHertz = 1.5f;
            weld.angularSpringDampingRatio = 0.3f;
        }
        NewWeldJoint(world, &weld);
    }

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    // Compare the beam tips by looking the bodies up via queries.
    m2RayCastResult rigid = m2World_CastRayClosest(world, (m2Pos2){1.7, 6.0}, (m2Vec2){0.0f, -6.0f},
                                                   m2DefaultQueryFilter());
    m2RayCastResult soft = m2World_CastRayClosest(world, (m2Pos2){11.7, 6.0}, (m2Vec2){0.0f, -6.0f},
                                                  m2DefaultQueryFilter());
    CHECK(rigid.hit, "rigid beam tip is where it was built");
    CHECK(rigid.point.y > 3.85, "the rigid weld holds its angle");
    CHECK(!soft.hit || soft.point.y < rigid.point.y - 0.05,
          "the angular-soft weld droops under the same load");

    m2DestroyWorld(world);
}

// The getters hand back the same numbers the scissors compare: read
// the load, set a limit just under it, and the joint MUST snap; set
// one well over it and it MUST hold. Plus analytic checks: a hanging
// crate reads its own weight, a cantilever reads its lever torque.
static void TestReactionGetters(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    m2JointId ropes[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 6.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 6.0};
        m2BodyId hook = NewBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 4.0};
        m2BodyId crate = NewBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon box = m2MakeBox(0.5f, 0.5f);
        NewPolygonShape(world, crate, &sd, &box);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crate;
        ropes[i] = NewDistanceJoint(world, &jd);
    }
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    // Load = m*g = 5 * 1 * 10 = 50 N steady.
    float load = m2Joint_GetReactionForce(world, ropes[0]);
    CHECK(load > 45.0f && load < 55.0f, "hanging crate reads its own weight");
    CHECK(m2Joint_GetReactionTorque(world, ropes[0]) == 0.0f, "a rope carries no torque");

    // Agreement with the scissors, both directions.
    m2Joint_SetBreakLimits(world, ropes[0], load * 0.95f, 0.0f);
    m2Joint_SetBreakLimits(world, ropes[1], load * 2.0f, 0.0f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Joint_IsValid(world, ropes[0]), "a limit under the reading snaps");
    CHECK(m2Joint_IsValid(world, ropes[1]), "a limit over the reading holds");
    CHECK(m2Joint_GetReactionForce(world, ropes[0]) == 0.0f, "a dead id reads zero");

    // Cantilever: beam mass 4*2*0.16 = 1.28 kg, COM one meter out from
    // the anchor: torque = 12.8 N*m, force = the 12.8 N weight.
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){12.0, 4.0};
    m2BodyId wall = NewBody(world, &wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){13.0, 4.0};
    m2BodyId beam = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 4.0f;
    m2Polygon bar = m2MakeBox(1.0f, 0.08f);
    NewPolygonShape(world, beam, &bs, &bar);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = wall;
    weld.bodyIdB = beam;
    weld.localAnchorB = (m2Vec2){-1.0f, 0.0f};
    m2JointId cantilever = NewWeldJoint(world, &weld);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float tipTorque = m2Joint_GetReactionTorque(world, cantilever);
    float holdForce = m2Joint_GetReactionForce(world, cantilever);
    CHECK(tipTorque > 10.0f && tipTorque < 16.0f, "cantilever reads its lever torque");
    CHECK(holdForce > 10.0f && holdForce < 16.0f, "and carries the beam weight");

    m2DestroyWorld(world);
}

// enableConnectedCollision: the default is that
// jointed bodies pass through each other; the flag restores contact,
// and the filter joint is nothing but this switch with a lifetime.
static void TestConnectedCollisionAndFilterJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(12.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    // A crate resting on a pedestal; a filter joint makes the pedestal
    // intangible to it and the crate falls through to the floor.
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 0.5};
    m2BodyId pedestal = NewBody(world, &pd);
    m2Polygon top = m2MakeBox(1.0f, 0.5f);
    NewPolygonShape(world, pedestal, &fs, &top);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.45};
    m2BodyId crate = NewBody(world, &cd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, crate, &sd, &unit);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, crate).y > 1.2, "the crate rests on the pedestal");

    m2FilterJointDef fj = m2DefaultFilterJointDef();
    fj.bodyIdA = pedestal;
    fj.bodyIdB = crate;
    m2JointId ghostPact = NewFilterJoint(world, &fj);
    CHECK(m2Joint_GetType(world, ghostPact) == m2_filterJoint, "the pact reads as a filter joint");
    CHECK(!m2Joint_IsConnectedCollisionEnabled(world, ghostPact), "and never collides");
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, crate).y < 0.9, "the crate falls through the pedestal");
    CHECK(m2Body_GetPosition(world, crate).y > 0.2, "and lands on the floor");
    CHECK(m2Joint_GetReactionForce(world, ghostPact) == 0.0f, "a filter joint carries nothing");

    // Destroy the pact: collision returns and the crate pops back out
    // on top the next time it is dropped onto the pedestal.
    DestroyJoint(world, ghostPact);
    m2Body_SetTransform(world, crate, (m2Pos2){0.0, 2.0}, (m2Rot){1.0f, 0.0f});
    m2Body_SetLinearVelocity(world, crate, (m2Vec2){0.0f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, crate).y > 1.2, "with the pact gone it rests on top again");

    // The flag on a real joint: two touching crates tied by a slack
    // rope. Default false lets them interpenetrate; true keeps them
    // apart. Same scene, one flag.
    for (int32_t pass = 0; pass < 2; ++pass)
    {
        m2BodyDef ad = m2DefaultBodyDef();
        ad.type = m2_dynamicBody;
        ad.position = (m2Pos2){6.0 + 4.0 * (double)pass, 0.45};
        m2BodyId left = NewBody(world, &ad);
        NewPolygonShape(world, left, &sd, &unit);
        m2BodyDef bd2 = ad;
        bd2.position = (m2Pos2){6.0 + 4.0 * (double)pass, 1.25};
        m2BodyId right = NewBody(world, &bd2);
        NewPolygonShape(world, right, &sd, &unit);
        m2DistanceJointDef dj = m2DefaultDistanceJointDef();
        dj.bodyIdA = left;
        dj.bodyIdB = right;
        dj.length = 0.1f; // a short soft rod pulling them together
        dj.enableSpring = true;
        dj.springHertz = 5.0f;
        dj.springDampingRatio = 0.7f;
        dj.enableConnectedCollision = pass == 1;
        NewDistanceJoint(world, &dj);
        for (int32_t i = 0; i < 120; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        double gap = m2Body_GetPosition(world, right).y - m2Body_GetPosition(world, left).y;
        if (pass == 0)
        {
            CHECK(gap < 0.5, "default false: the top crate sinks into the bottom one");
        }
        else
        {
            CHECK(gap > 0.7, "true: contact keeps them stacked");
        }
    }
    m2DestroyWorld(world);
}

// Motor and mouse joints (parity sprint 3b): a platform that chases
// offsets under gravity, and a crate dragged around by a soft target
// spring, solved on the step's delta tracking.
static void TestMotorAndMouseJoints(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    // Motor platform: anchor at the origin, platform driven to an
    // offset and held there against gravity.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 4.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.type = m2_dynamicBody;
    pd.position = (m2Pos2){0.0, 4.0};
    m2BodyId platform = NewBody(world, &pd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon deck = m2MakeBox(0.8f, 0.1f);
    NewPolygonShape(world, platform, &sd, &deck);
    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = anchor;
    mj.bodyIdB = platform;
    mj.linearOffset = (m2Vec2){2.0f, 1.0f};
    mj.maxForce = 200.0f;
    mj.maxTorque = 50.0f;
    m2JointId mover = NewMotorJoint(world, &mj);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 at = m2Body_GetPosition(world, platform);
    CHECK(at.x > 1.8 && at.x < 2.2, "the platform chases the x offset");
    CHECK(at.y > 4.7 && at.y < 5.2, "and holds the y offset against gravity");
    m2Vec2 off = m2MotorJoint_GetLinearOffset(world, mover);
    CHECK(off.x == 2.0f && off.y == 1.0f, "offsets read back");
    CHECK(m2MotorJoint_GetMaxForce(world, mover) == 200.0f, "and the force budget too");

    // Retarget: the platform follows.
    m2MotorJoint_SetOffsets(world, mover, (m2Vec2){-2.0f, 0.5f}, 0.0f);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    at = m2Body_GetPosition(world, platform);
    CHECK(at.x < -1.7, "retargeted platform crosses over");

    // Mouse drag: a crate on the floor gets pulled up and sideways.
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){10.0, -0.5};
    m2BodyId floor = NewBody(world, &gd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(6.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){10.0, 0.45};
    m2BodyId crate = NewBody(world, &cd);
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, crate, &sd, &unit);
    m2MouseJointDef sj = m2DefaultMouseJointDef();
    sj.bodyIdA = floor;
    sj.bodyIdB = crate;
    sj.target = (m2Pos2){10.0, 0.45};
    sj.maxForce = 200.0f;
    m2JointId grip = NewMouseJoint(world, &sj);
    m2MouseJoint_SetTarget(world, grip, (m2Pos2){12.0, 3.0});
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    at = m2Body_GetPosition(world, crate);
    CHECK(at.x > 11.3 && at.x < 12.7, "the drag pulls the crate sideways");
    CHECK(at.y > 2.2 && at.y < 3.6, "and lifts it off the floor");
    m2Pos2 target = m2MouseJoint_GetTarget(world, grip);
    CHECK(target.x == 12.0 && target.y == 3.0, "the target reads back");
    CHECK(m2Joint_GetReactionForce(world, grip) > 1.0f, "the spring carries real load");
    CHECK(m2Joint_GetType(world, mover) == m2_motorJoint &&
              m2Joint_GetType(world, grip) == m2_mouseJoint,
          "types read back");

    m2DestroyWorld(world);
}

// The richer distance joint (parity bucket 2): a hard length range
// over the spring, runtime retargeting, and runtime softness.
// A slack rope: a distance joint with its spring toggle on, hertz 0 and a
// maximum length, holding a ball that starts inside the rope's reach.
static m2World* RopeWorld(const m2DistanceJointDef* rope, m2JointId* ropeOut)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyDef hookDef = m2DefaultBodyDef();
    hookDef.position = (m2Pos2){0.0, 5.0};
    m2BodyId hook = NewBody(world, &hookDef);
    m2BodyDef ballDef = m2DefaultBodyDef();
    ballDef.type = m2_dynamicBody;
    ballDef.position = (m2Pos2){1.0, 5.0};
    m2BodyId ball = NewBody(world, &ballDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle disc = {{0.0f, 0.0f}, 0.25f};
    NewCircleShape(world, ball, &sd, &disc);
    m2DistanceJointDef jd = *rope;
    jd.bodyIdA = hook;
    jd.bodyIdB = ball;
    *ropeOut = NewDistanceJoint(world, &jd);
    return world;
}

// Every setting of a distance joint reads back, the spring toggle
// included: a rope rebuilt from its getters swings like the original.
static void TestDistanceSpringToggleReadsBack(void)
{
    m2DistanceJointDef rope = m2DefaultDistanceJointDef();
    rope.enableSpring = true;
    rope.maxLength = 3.0f;
    m2JointId original;
    m2World* world = RopeWorld(&rope, &original);
    CHECK(m2Joint_IsSpringEnabled(world, original), "the rope's toggle reads back");

    m2DistanceJointDef copy = m2DefaultDistanceJointDef();
    copy.length = m2DistanceJoint_GetLength(world, original);
    m2DistanceJoint_GetLengthRange(world, original, &copy.minLength, &copy.maxLength);
    copy.springHertz = m2Joint_GetSpringHertz(world, original);
    copy.springDampingRatio = m2Joint_GetSpringDampingRatio(world, original);
    copy.enableSpring = m2Joint_IsSpringEnabled(world, original);
    m2JointId rebuilt;
    m2World* twin = RopeWorld(&copy, &rebuilt);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2World_Step(twin, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(world) == m2World_Hash(twin), "the rebuilt rope falls slack the same way");

    m2DistanceJointDef rod = m2DefaultDistanceJointDef();
    m2JointId plain;
    m2World* rodWorld = RopeWorld(&rod, &plain);
    CHECK(!m2Joint_IsSpringEnabled(rodWorld, plain), "a plain rod reads off");
    m2WeldJointDef glue = m2DefaultWeldJointDef();
    glue.bodyIdA = m2Joint_GetBodyA(rodWorld, plain);
    glue.bodyIdB = m2Joint_GetBodyB(rodWorld, plain);
    m2JointId weld = NewWeldJoint(rodWorld, &glue);
    uint64_t misuse = m2World_GetCounters(rodWorld).misuse;
    CHECK(!m2Joint_IsSpringEnabled(rodWorld, weld), "a weld has no spring switch");
    CHECK(m2World_GetCounters(rodWorld).misuse == misuse + 1, "and asking counts one misuse");
    m2DestroyWorld(rodWorld);
    m2DestroyWorld(twin);
    m2DestroyWorld(world);
}

static void TestDistanceRange(void)
{
    // The hard range rows run on the stiff default softness, so when
    // the range disagrees with a soft rod, the range wins. That is
    // the observable contract: a squishy rope reeled hard.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    m2JointId ropes[2];
    m2BodyId crates[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 8.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 8.0};
        m2BodyId hook = NewBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 6.0};
        crates[i] = NewBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon crate = m2MakeBox(0.5f, 0.5f);
        NewPolygonShape(world, crates[i], &sd, &crate);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crates[i];
        jd.length = 2.0f;
        jd.enableSpring = true;
        jd.springHertz = 0.5f; // a squishy rod that converges near 2
        jd.springDampingRatio = 0.7f;
        if (i == 1)
        {
            jd.minLength = 0.5f;
            jd.maxLength = 1.5f; // fights the rod, and must win
        }
        ropes[i] = NewDistanceJoint(world, &jd);
    }
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double hangFree = 8.0 - m2Body_GetPosition(world, crates[0]).y;
    double hangClamped = 8.0 - m2Body_GetPosition(world, crates[1]).y;
    CHECK(hangFree > 1.9, "the free rope settles at its length");
    CHECK(hangClamped < 1.7, "the ranged rope is held at max length instead");
    float lo = 0.0f;
    float hi = 0.0f;
    m2DistanceJoint_GetLengthRange(world, ropes[1], &lo, &hi);
    CHECK(lo == 0.5f && hi == 1.5f, "the range reads back");
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Joint_GetLimits(world, ropes[1], &lo, &hi);
    CHECK(lo == 0.0f && hi == 0.0f && m2World_GetCounters(world).misuse == misuse + 1,
          "a distance joint has no limits to read, only its range");

    // Runtime: reel the free one in the same way.
    m2DistanceJoint_SetLengthRange(world, ropes[0], 0.5f, 1.5f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double reeled = 8.0 - m2Body_GetPosition(world, crates[0]).y;
    CHECK(reeled < 1.7, "a runtime range reels the crate in");

    // Min side: force the clamped one OUT beyond its rod.
    m2DistanceJoint_SetLengthRange(world, ropes[1], 2.6f, 5.0f);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double pushed = 8.0 - m2Body_GetPosition(world, crates[1]).y;
    CHECK(pushed > 2.4, "the min bound pushes the load out past the rod");

    // Runtime spring retuning is journaled and takes effect: soften
    // the free rope hugely and give it a shove; it wobbles (peak
    // deviation from its clamp grows), then stiffen and it pins.
    m2Joint_SetSpringHertz(world, ropes[0], 0.1f);
    m2Joint_SetSpringDampingRatio(world, ropes[0], 0.05f);
    m2Body_ApplyLinearImpulse(world, crates[0], (m2Vec2){30.0f, 0.0f});
    double wobble = 0.0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        double sway = m2Body_GetPosition(world, crates[0]).x;
        sway = sway < 0.0 ? -sway : sway;
        wobble = sway > wobble ? sway : wobble;
    }
    CHECK(wobble > 0.5, "a shoved pendulum sways");

    // Retarget a rigid rod: the crate follows the new length.
    m2BodyDef ad2 = m2DefaultBodyDef();
    ad2.position = (m2Pos2){16.0, 8.0};
    m2BodyId hook2 = NewBody(world, &ad2);
    m2BodyDef cd2 = m2DefaultBodyDef();
    cd2.type = m2_dynamicBody;
    cd2.position = (m2Pos2){16.0, 7.0};
    m2BodyId weight = NewBody(world, &cd2);
    m2ShapeDef sd2 = m2DefaultShapeDef();
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, weight, &sd2, &unit);
    m2DistanceJointDef rig = m2DefaultDistanceJointDef();
    rig.bodyIdA = hook2;
    rig.bodyIdB = weight;
    m2JointId rod = NewDistanceJoint(world, &rig); // rigid, length 1
    m2DistanceJoint_SetLength(world, rod, 2.5f);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    double drop = 8.0 - m2Body_GetPosition(world, weight).y;
    CHECK(drop > 2.3 && drop < 2.7, "a retargeted rigid rod lowers the load");

    m2DestroyWorld(world);
}

// The revolute angular spring: a horizontal bar on a pin either
// droops to hanging (no spring) or holds near its spawn angle
// (strong spring); killing the spring at runtime lets it fall.
static void TestRevoluteSpring(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, -10.0f});

    m2Circle bobDisc = {{0.0f, 0.0f}, 0.2f};
    m2BodyId bobs[2];
    m2JointId pins[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 4.0;
        m2BodyDef postDef = m2DefaultBodyDef();
        postDef.position = (m2Pos2){x, 5.0};
        m2BodyId post = NewBody(world, &postDef);
        m2BodyDef bobDef = m2DefaultBodyDef();
        bobDef.type = m2_dynamicBody;
        bobDef.position = (m2Pos2){x + 1.0, 5.0};
        bobDef.linearDamping = 1.0f;
        bobs[i] = NewBody(world, &bobDef);
        m2ShapeDef sd = m2DefaultShapeDef();
        NewCircleShape(world, bobs[i], &sd, &bobDisc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = bobs[i];
        rj.localAnchorB = (m2Vec2){-1.0f, 0.0f};
        if (i == 1)
        {
            rj.enableSpring = true;
            rj.springHertz = 12.0f;
            rj.springDampingRatio = 1.0f;
        }
        pins[i] = NewRevoluteJoint(world, &rj);
    }
    CHECK(m2Joint_GetSpringHertz(world, pins[1]) == 12.0f, "the spring hertz reads back");
    CHECK(m2Joint_GetSpringDampingRatio(world, pins[1]) == 1.0f,
          "the spring damping ratio reads back");

    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 limp = m2Body_GetPosition(world, bobs[0]);
    m2Pos2 held = m2Body_GetPosition(world, bobs[1]);
    CHECK(limp.x < 0.4 && limp.y < 4.4, "the springless bar droops to hanging");
    CHECK(held.x > 4.7 && held.y > 4.7, "the sprung bar holds near its spawn angle");
    CHECK(m2Joint_GetReactionTorque(world, pins[1]) > 0.0f, "the spring carries the bar's weight");

    // Kill the spring at runtime: the held bar falls like its twin.
    m2Joint_EnableSpring(world, pins[1], false);
    CHECK(!m2Joint_IsSpringEnabled(world, pins[1]) &&
              m2Joint_GetSpringHertz(world, pins[1]) == 12.0f,
          "the disabled spring reads back, its frequency kept");
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    held = m2Body_GetPosition(world, bobs[1]);
    CHECK(held.x < 4.4 && held.y < 4.4, "the bar droops once the spring is gone");

    m2DestroyWorld(world);
}

// The pulley: a rope over two fixed world points. Heavy side sinks,
// light side rises, the rope total is conserved; a ratio-2 machine
// balances double the mass; a live retune recaptures the total and
// the machine re-decides without snapping.
static void TestPulleyJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, -10.0f});

    m2Circle disc = {{0.0f, 0.0f}, 0.4f};

    // Machine 1: equal ratio, unequal masses. The heavy side wins.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.type = m2_dynamicBody;
    hd.position = (m2Pos2){-2.0, 2.0};
    m2BodyId heavy = NewBody(world, &hd);
    m2ShapeDef hs = m2DefaultShapeDef();
    hs.density = 4.0f;
    NewCircleShape(world, heavy, &hs, &disc);
    m2BodyDef ld = m2DefaultBodyDef();
    ld.type = m2_dynamicBody;
    ld.position = (m2Pos2){2.0, 2.0};
    m2BodyId light = NewBody(world, &ld);
    m2ShapeDef ls = m2DefaultShapeDef();
    ls.density = 1.0f;
    NewCircleShape(world, light, &ls, &disc);
    m2PulleyJointDef pd = m2DefaultPulleyJointDef();
    pd.bodyIdA = heavy;
    pd.bodyIdB = light;
    pd.groundAnchorA = (m2Pos2){-2.0, 4.0};
    pd.groundAnchorB = (m2Pos2){2.0, 4.0};
    m2JointId rope = NewPulleyJoint(world, &pd);
    CHECK(m2PulleyJoint_GetRatio(world, rope) == 1.0f, "the ratio reads back");
    float spawnA = m2PulleyJoint_GetLengthA(world, rope);
    CHECK(spawnA > 1.9f && spawnA < 2.1f, "the A rope measures its spawn length");
    m2Pos2 gA = m2PulleyJoint_GetGroundAnchorA(world, rope);
    CHECK(gA.x == -2.0 && gA.y == 4.0, "the ground anchor reads back");

    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pHeavy = m2Body_GetPosition(world, heavy);
    m2Pos2 pLight = m2Body_GetPosition(world, light);
    CHECK(pHeavy.y < 1.7, "the heavy side sinks");
    CHECK(pLight.y > 2.3, "the light side rises");
    float total = m2PulleyJoint_GetLengthA(world, rope) + m2PulleyJoint_GetLengthB(world, rope);
    CHECK(total > 3.95f && total < 4.05f, "the rope total is conserved");
    CHECK(m2Joint_GetReactionForce(world, rope) > 0.0f, "the rope carries tension");

    // Machine 2: ratio 2 balances double the mass (the B side feels
    // twice the A-side tension).
    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){6.0, 2.0};
    m2BodyId counter = NewBody(world, &ad);
    m2ShapeDef as = m2DefaultShapeDef();
    as.density = 1.0f;
    NewCircleShape(world, counter, &as, &disc);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){10.0, 2.0};
    m2BodyId load = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 2.0f;
    NewCircleShape(world, load, &bs, &disc);
    m2PulleyJointDef bal = m2DefaultPulleyJointDef();
    bal.bodyIdA = counter;
    bal.bodyIdB = load;
    bal.groundAnchorA = (m2Pos2){6.0, 4.0};
    bal.groundAnchorB = (m2Pos2){10.0, 4.0};
    bal.ratio = 2.0f;
    m2JointId scale = NewPulleyJoint(world, &bal);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pCounter = m2Body_GetPosition(world, counter);
    m2Pos2 pLoad = m2Body_GetPosition(world, load);
    CHECK(pCounter.y > 1.85 && pCounter.y < 2.15, "the counterweight holds its height");
    CHECK(pLoad.y > 1.85 && pLoad.y < 2.15, "double mass balances at ratio 2");

    // Live retune: at ratio 1 the double-mass side wins; the total is
    // recaptured at the retune so the machine re-decides, no snap.
    m2PulleyJoint_SetRatio(world, scale, 1.0f);
    CHECK(m2PulleyJoint_GetRatio(world, scale) == 1.0f, "the new ratio reads back");
    float c0 = m2PulleyJoint_GetLengthA(world, scale) + m2PulleyJoint_GetLengthB(world, scale);
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 pCounter2 = m2Body_GetPosition(world, counter);
    m2Pos2 pLoad2 = m2Body_GetPosition(world, load);
    CHECK(pLoad2.y < pLoad.y - 0.1, "the heavy side wins after the retune");
    CHECK(pCounter2.y > pCounter.y + 0.1, "the counterweight gives way");
    float c1 = m2PulleyJoint_GetLengthA(world, scale) + m2PulleyJoint_GetLengthB(world, scale);
    CHECK(c1 > c0 - 0.05f && c1 < c0 + 0.05f, "the recaptured total is conserved");

    m2DestroyWorld(world);
}

// Unwound relative rotation magnitude between two rotations.
static float m2Atan2Test(m2Rot before, m2Rot after)
{
    float sin = before.c * after.s - before.s * after.c;
    float cos = before.c * after.c + before.s * after.s;
    float angle = atan2f(sin, cos);
    return angle < 0.0f ? -angle : angle;
}

// The ratchet: forward spins free and clicks tooth by tooth;
// forced back-spin holds at the last engaged tooth, multi-turn.
static void TestRatchetJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});
    m2BodyDef postDef = m2DefaultBodyDef();
    postDef.position = (m2Pos2){0.0, 2.0};
    m2BodyId post = NewBody(world, &postDef);
    m2BodyDef wheelDef = m2DefaultBodyDef();
    wheelDef.type = m2_dynamicBody;
    wheelDef.position = (m2Pos2){0.0, 2.0};
    m2BodyId wheel = NewBody(world, &wheelDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle disc = {{0.0f, 0.0f}, 0.5f};
    NewCircleShape(world, wheel, &sd, &disc);
    m2RevoluteJointDef pinDef = m2DefaultRevoluteJointDef();
    pinDef.bodyIdA = post;
    pinDef.bodyIdB = wheel;
    NewRevoluteJoint(world, &pinDef);
    m2RatchetJointDef ratchetDef = m2DefaultRatchetJointDef();
    ratchetDef.bodyIdA = post;
    ratchetDef.bodyIdB = wheel;
    ratchetDef.ratchet = 0.5f;
    m2JointId ratchet = NewRatchetJoint(world, &ratchetDef);
    CHECK(m2RatchetJoint_GetRatchet(world, ratchet) == 0.5f, "the tooth angle reads back");
    CHECK(m2RatchetJoint_GetPhase(world, ratchet) == 0.0f, "the phase reads back");

    // Forward: free spin, more than a full turn (multi-turn law).
    m2Body_SetAngularVelocity(world, wheel, 3.0f);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetAngularVelocity(world, wheel) > 2.9f, "the free direction never engages");

    // Reverse under a CONTINUOUS torque (an instantaneous shove is
    // consumed in the first substep and reads zero at step end, the
    // exact bookkeeping of speculative rows): the wheel gives back
    // less than one tooth, stalls on it, and carries the load.
    m2Rot before = m2Body_GetRotation(world, wheel);
    for (int32_t i = 0; i < 180; ++i)
    {
        m2Body_ApplyTorque(world, wheel, -2.0f);
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Rot after = m2Body_GetRotation(world, wheel);
    float backSpin = m2Atan2Test(before, after);
    CHECK(backSpin < 0.51f, "back-spin never passes one tooth");
    CHECK(m2Body_GetAngularVelocity(world, wheel) > -0.05f &&
              m2Body_GetAngularVelocity(world, wheel) < 0.05f,
          "the tooth stalls the loaded reverse");
    float hold = m2Joint_GetReactionTorque(world, ratchet);
    CHECK(hold > 1.5f && hold < 2.5f, "the ratchet reports the load it carries");
    m2DestroyWorld(world);
}

// A pinned wheel on a ratchet, spun forward past several teeth.
static m2World* RatchetScene(m2JointId* ratchetOut, m2BodyId* wheelOut)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 8;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);
    m2BodyDef postDef = m2DefaultBodyDef();
    m2BodyId post = NewBody(world, &postDef);
    m2BodyDef wheelDef = m2DefaultBodyDef();
    wheelDef.type = m2_dynamicBody;
    *wheelOut = NewBody(world, &wheelDef);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle disc = {{0.0f, 0.0f}, 0.5f};
    NewCircleShape(world, *wheelOut, &sd, &disc);
    m2RevoluteJointDef pinDef = m2DefaultRevoluteJointDef();
    pinDef.bodyIdA = post;
    pinDef.bodyIdB = *wheelOut;
    NewRevoluteJoint(world, &pinDef);
    m2RatchetJointDef ratchetDef = m2DefaultRatchetJointDef();
    ratchetDef.bodyIdA = post;
    ratchetDef.bodyIdB = *wheelOut;
    ratchetDef.ratchet = 0.5f;
    *ratchetOut = NewRatchetJoint(world, &ratchetDef);
    m2Body_SetAngularVelocity(world, *wheelOut, 3.0f);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    return world;
}

// Motor and limit parameters belong to the revolute, prismatic and wheel
// joints (the motor joint's torque budget rides the max motor channel).
// On any other joint they are refused and counted: a ratchet keeps its
// tooth state where a hinge keeps its limits, so a limit call on it must
// not reach the ratchet.
static void TestMotorAndLimitCallsNeedTheirJoint(void)
{
    m2JointId ratchet;
    m2BodyId wheel;
    m2World* world = RatchetScene(&ratchet, &wheel);
    m2JointId twinRatchet;
    m2BodyId twinWheel;
    m2World* twin = RatchetScene(&twinRatchet, &twinWheel);

    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Joint_SetLimits(world, ratchet, -10.0f, 10.0f);
    m2Joint_EnableLimit(world, ratchet, true);
    m2Joint_EnableMotor(world, ratchet, true);
    m2Joint_SetMotorSpeed(world, ratchet, -4.0f);
    m2Joint_SetMaxMotor(world, ratchet, 100.0f);
    CHECK(m2World_GetCounters(world).misuse == misuse + 5, "each call on a ratchet counts");
    float lower = 1.0f;
    float upper = 1.0f;
    m2Joint_GetLimits(world, ratchet, &lower, &upper);
    CHECK(lower == 0.0f && upper == 0.0f, "a ratchet has no limits to read");
    CHECK(m2Joint_GetMotorSpeed(world, ratchet) == 0.0f, "nor a motor");
    lower = 1.0f;
    upper = 1.0f;
    m2DistanceJoint_GetLengthRange(world, ratchet, &lower, &upper);
    CHECK(lower == 0.0f && upper == 0.0f, "nor a length range");
    CHECK(m2World_GetCounters(world).misuse == misuse + 8, "and the reads count too");

    for (int32_t i = 0; i < 120; ++i)
    {
        m2Body_ApplyTorque(world, wheel, -2.0f);
        m2Body_ApplyTorque(twin, twinWheel, -2.0f);
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2World_Step(twin, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(world) == m2World_Hash(twin), "the ratchet holds as if never called");

    // The joints that own the channels still take them.
    m2WorldDef def = m2DefaultWorldDef();
    m2World* hinges = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId base = NewBody(hinges, &bd);
    bd.type = m2_dynamicBody;
    m2BodyId arm = NewBody(hinges, &bd);
    m2RevoluteJointDef rd = m2DefaultRevoluteJointDef();
    rd.bodyIdA = base;
    rd.bodyIdB = arm;
    m2JointId hinge = NewRevoluteJoint(hinges, &rd);
    m2MotorJointDef md = m2DefaultMotorJointDef();
    md.bodyIdA = base;
    md.bodyIdB = arm;
    m2JointId motor = NewMotorJoint(hinges, &md);
    misuse = m2World_GetCounters(hinges).misuse;
    m2Joint_SetLimits(hinges, hinge, -1.0f, 1.0f);
    m2Joint_EnableLimit(hinges, hinge, true);
    m2Joint_SetMotorSpeed(hinges, hinge, 2.0f);
    m2Joint_SetMaxMotor(hinges, motor, 50.0f);
    CHECK(m2World_GetCounters(hinges).misuse == misuse, "a hinge and a motor joint take them");
    CHECK(m2Joint_GetMaxMotor(hinges, motor) == 50.0f,
          "the motor joint's torque budget reads back");
    m2DestroyWorld(hinges);
    m2DestroyWorld(twin);
    m2DestroyWorld(world);
}

// The gear joint: two pinned flywheels coupled at a ratio; drive
// one, the other counter-spins scaled; retune the ratio live; the
// phase survives many full turns.
static void TestGearJoint(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, 0.0f});

    m2BodyId wheels[2];
    m2JointId pins[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 3.0;
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){x, 2.0};
        m2BodyId post = NewBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){x, 2.0};
        wheels[i] = NewBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle disc = {{0.0f, 0.0f}, 0.5f};
        NewCircleShape(world, wheels[i], &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheels[i];
        if (i == 0)
        {
            rj.enableMotor = true;
            rj.motorSpeed = 3.0f;
            rj.maxMotorTorque = 50.0f;
        }
        pins[i] = NewRevoluteJoint(world, &rj);
    }
    (void)pins;
    m2GearJointDef gj = m2DefaultGearJointDef();
    gj.bodyIdA = wheels[0];
    gj.bodyIdB = wheels[1];
    gj.ratio = 2.0f;
    m2JointId gear = NewGearJoint(world, &gj);

    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float wA = m2Body_GetAngularVelocity(world, wheels[0]);
    float wB = m2Body_GetAngularVelocity(world, wheels[1]);
    CHECK(wA > 2.7f && wA < 3.3f, "the driven wheel reaches its motor speed");
    CHECK(wB < -5.4f && wB > -6.6f, "the geared wheel counter-spins at twice the rate");
    CHECK(m2Joint_GetReactionTorque(world, gear) > 0.0f, "the gear carries torque");
    CHECK(m2GearJoint_GetRatio(world, gear) == 2.0f, "the ratio reads back");

    // Many full turns later the coupling still holds exactly: cut the
    // motor and the ratio persists through the coast-down.
    m2Joint_EnableMotor(world, pins[0], false);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    wA = m2Body_GetAngularVelocity(world, wheels[0]);
    wB = m2Body_GetAngularVelocity(world, wheels[1]);
    CHECK(wA > 0.5f, "the pair coasts");
    CHECK(wB / wA < -1.9f && wB / wA > -2.1f, "still locked at the ratio");

    // Live retune halves the coupling.
    m2GearJoint_SetRatio(world, gear, 1.0f);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    wA = m2Body_GetAngularVelocity(world, wheels[0]);
    wB = m2Body_GetAngularVelocity(world, wheels[1]);
    CHECK(wA != 0.0f && wB / wA < -0.9f && wB / wA > -1.1f, "the new ratio takes over");

    m2DestroyWorld(world);
}

static void TestJointBreaking(void)
{
    // Twin ropes carry the same heavy crate; one has a break limit
    // under the load, one above it. The weak one snaps, reports the
    // force that did it, and the crate falls. Then a weld cantilever
    // breaks by torque alone. Snapping replays bit-exactly through a
    // pre-break snapshot because it is a pure function of state.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    m2BodyId crates[2];
    m2JointId ropes[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        double x = (double)i * 6.0;
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){x, 6.0};
        m2BodyId hook = NewBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){x, 4.0};
        crates[i] = NewBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon crate = m2MakeBox(0.5f, 0.5f);
        NewPolygonShape(world, crates[i], &sd, &crate);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crates[i];
        ropes[i] = NewDistanceJoint(world, &jd);
    }
    // Load = m*g = 5*1*10 = 50 N steady. Weak rope: 30 N. Strong: 500 N.
    m2Joint_SetBreakLimits(world, ropes[0], 30.0f, 0.0f);
    m2Joint_SetBreakLimits(world, ropes[1], 500.0f, 0.0f);

    bool snapped = false;
    float reportedForce = 0.0f;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2JointEvents events = m2World_GetJointEvents(world);
        if (events.breakCount > 0 && !snapped)
        {
            snapped = true;
            reportedForce = events.breakEvents[0].force;
            CHECK(events.breakEvents[0].jointId.index1 == ropes[0].index1,
                  "the weak rope is the one that snapped");
        }
    }
    CHECK(snapped, "the overloaded rope snaps");
    CHECK(reportedForce > 30.0f, "the event reports the force that broke it");
    CHECK(!m2Joint_IsValid(world, ropes[0]), "the snapped rope is gone");
    CHECK(m2Joint_IsValid(world, ropes[1]), "the strong rope holds");
    CHECK(m2Body_GetPosition(world, crates[0]).y < 2.0, "the crate fell");
    CHECK(m2Body_GetPosition(world, crates[1]).y > 3.0, "the safe crate hangs");

    // Torque break: a weld cantilever with a heavy tip.
    m2BodyDef wd = m2DefaultBodyDef();
    wd.position = (m2Pos2){12.0, 4.0};
    m2BodyId wall = NewBody(world, &wd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){13.0, 4.0};
    m2BodyId beam = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.density = 4.0f;
    m2Polygon bar = m2MakeBox(1.0f, 0.08f);
    NewPolygonShape(world, beam, &bs, &bar);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = wall;
    weld.bodyIdB = beam;
    weld.localAnchorB = (m2Vec2){-1.0f, 0.0f};
    m2JointId cantilever = NewWeldJoint(world, &weld);
    m2Joint_SetBreakLimits(world, cantilever, 0.0f, 4.0f); // gravity torque ~ 6.4 Nm
    bool cracked = false;
    for (int32_t i = 0; i < 120 && !cracked; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        cracked = m2World_GetJointEvents(world).breakCount > 0;
    }
    CHECK(cracked, "the weld cracks under torque alone");
    CHECK(!m2Joint_IsValid(world, cantilever), "and is gone");

    m2DestroyWorld(world);
}

static void TestBreakRollback(void)
{
    // Snapshot before the snap, replay through it: the joint must
    // break on the identical step with identical bits.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 6.0};
    m2BodyId hook = NewBody(world, &ad);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 4.0};
    m2BodyId crate = NewBody(world, &cd);
    m2ShapeDef sd = m2DefaultShapeDef();
    sd.density = 5.0f;
    m2Polygon box = m2MakeBox(0.5f, 0.5f);
    NewPolygonShape(world, crate, &sd, &box);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = hook;
    jd.bodyIdB = crate;
    m2JointId rope = NewDistanceJoint(world, &jd);
    m2Joint_SetBreakLimits(world, rope, 30.0f, 0.0f);

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot before the snap");

    int32_t breakStepA = -1;
    uint64_t hashes[60];
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
        if (breakStepA < 0 && m2World_GetJointEvents(world).breakCount > 0)
        {
            breakStepA = i;
        }
    }
    CHECK(breakStepA >= 0, "the rope snapped in the recorded run");

    CHECK(Restore(world, snap, size), "restore");
    int32_t breakStepB = -1;
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "the snap replays bit-exactly");
        if (breakStepB < 0 && m2World_GetJointEvents(world).breakCount > 0)
        {
            breakStepB = i;
        }
    }
    CHECK(breakStepA == breakStepB, "and lands on the identical step");

    free(snap);
    m2DestroyWorld(world);
}

static void TestPrismaticStressStability(void)
{
    // An over-constrained slider on a spinning base: two prismatic joints
    // pull a dynamic body along fighting axes while the kinematic base
    // rotates. With the effective mass frozen at prepare the slider builds
    // up speed (peaked near 20 in this scene); the fresh per-substep mass
    // keeps it far lower. The peak stays well below the
    // velocity cap, so this is the solver's own stability, not the cap. It
    // must stay bounded, finite, and bit-identical across worker counts and
    // a rollback.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef def = m2DefaultWorldDef();
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&def, &s_twinWorkers[wc]);
        def.bodyCapacity = 8;
        def.jointCapacity = 8;
        m2World* world = NewWorld(&def);
        m2BodyDef kd = m2DefaultBodyDef();
        kd.type = m2_kinematicBody;
        kd.angularVelocity = 8.0f;
        m2BodyId base = NewBody(world, &kd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon bx = m2MakeBox(0.3f, 0.3f);
        NewPolygonShape(world, base, &sd, &bx);
        m2BodyDef dd = m2DefaultBodyDef();
        dd.type = m2_dynamicBody;
        dd.position = (m2Pos2){1.5, 0.0};
        m2BodyId body = NewBody(world, &dd);
        NewPolygonShape(world, body, &sd, &bx);
        m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
        pj.bodyIdA = base;
        pj.bodyIdB = body;
        pj.localAxisA = (m2Vec2){1.0f, 0.0f};
        pj.enableLimit = true;
        pj.lowerTranslation = -5.0f;
        pj.upperTranslation = 5.0f;
        NewPrismaticJoint(world, &pj);
        m2PrismaticJointDef pj2 = pj;
        pj2.localAxisA = (m2Vec2){0.0f, 1.0f};
        NewPrismaticJoint(world, &pj2);

        float peak = 0.0f;
        const int32_t snapAt = 150;
        int32_t size = 0;
        void* snap = NULL;
        for (int32_t i = 0; i < 300; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
            m2Vec2 v = m2Body_GetLinearVelocity(world, body);
            float av = m2Body_GetAngularVelocity(world, body);
            float s = sqrtf(v.x * v.x + v.y * v.y) + fabsf(av);
            if (s > peak)
            {
                peak = s;
            }
            CHECK(v.x == v.x && v.y == v.y && av == av, "the stressed slider stays finite");
            if (i == snapAt)
            {
                size = m2World_SnapshotSize(world);
                snap = malloc((size_t)size);
                Snapshot(world, snap, size);
            }
        }
        CHECK(peak < 15.0f, "the fresh effective mass keeps the stressed slider bounded");
        uint64_t endHash = m2World_Hash(world);
        CHECK(Restore(world, snap, size), "the stressed slider restores");
        for (int32_t i = snapAt + 1; i < 300; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(world) == endHash, "the stressed slider replays bit for bit");
        hashes[wc] = endHash;
        free(snap);
        m2DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the stressed slider is worker-count deterministic");
}

static int32_t JointIndices(const m2World* world, m2BodyId body, uint32_t* out, int32_t capacity)
{
    m2JointId ids[16];
    int32_t count = m2Body_GetJoints(world, body, ids, 16);
    for (int32_t i = 0; i < count && i < capacity; ++i)
    {
        out[i] = ids[i].index1;
    }
    return count;
}

static void TestJointAdjacency(void)
{
    // A hub body with four spokes. Body joint lists come back in slot
    // order, follow destruction, and are rebuilt by a restore.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyId hub = AddBall(world, 0.0, 0.0, 0.5f);
    m2BodyId spokes[4];
    m2JointId joints[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        spokes[i] = AddBall(world, 0.5 * (double)(i + 1), 0.0, 0.5f);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = (i % 2 == 0) ? hub : spokes[i];
        jd.bodyIdB = (i % 2 == 0) ? spokes[i] : hub;
        jd.length = 0.5f * (float)(i + 1);
        joints[i] = NewDistanceJoint(world, &jd);
    }

    uint32_t got[8];
    CHECK(JointIndices(world, hub, got, 8) == 4, "the hub lists all four joints");
    CHECK(got[0] < got[1] && got[1] < got[2] && got[2] < got[3], "in ascending slot order");
    CHECK(JointIndices(world, spokes[2], got, 8) == 1 && got[0] == joints[2].index1,
          "each spoke lists its own joint");

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot taken");

    DestroyJoint(world, joints[1]);
    CHECK(JointIndices(world, hub, got, 8) == 3 && got[1] == joints[2].index1,
          "a destroyed joint leaves the hub list");
    CHECK(JointIndices(world, spokes[1], got, 8) == 0, "and the spoke list");
    DestroyBody(world, spokes[3]);
    CHECK(JointIndices(world, hub, got, 8) == 2, "a destroyed body takes its joint along");

    CHECK(Restore(world, snap, size), "restore");
    CHECK(JointIndices(world, hub, got, 8) == 4 && got[1] == joints[1].index1 &&
              got[3] == joints[3].index1,
          "the restore rebuilds the hub list");
    CHECK(JointIndices(world, spokes[1], got, 8) == 1, "and every spoke list");

    // The spokes overlap the hub, yet jointed bodies never touch: the
    // pair filter reads the rebuilt lists. Spokes still touch each
    // other, which proves contacts are being generated at all.
    int32_t hubTouches = 0;
    int32_t spokeTouches = 0;
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents events = m2World_GetContactEvents(world);
        for (int32_t e = 0; e < events.beginCount; ++e)
        {
            uint32_t a = m2Shape_GetBody(world, events.beginEvents[e].shapeIdA).index1;
            uint32_t b = m2Shape_GetBody(world, events.beginEvents[e].shapeIdB).index1;
            if (a == hub.index1 || b == hub.index1)
            {
                hubTouches += 1;
            }
            else
            {
                spokeTouches += 1;
            }
        }
    }
    CHECK(hubTouches == 0, "jointed bodies never touch after a restore");
    CHECK(spokeTouches > 0, "unjointed overlapping bodies do touch");

    free(snap);
    m2DestroyWorld(world);
}

// Joint anchors, axes and the motor's offset are body-local, so they stop
// at M2_HUGE like shape coordinates: each create past it is refused, and
// so is a motor offset set past it.
static void TestLocalAnchorsAreBounded(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyId a = AddBall(world, 0.0, 0.0, 0.5f);
    m2BodyId b = AddBall(world, 2.0, 0.0, 0.5f);
    const m2Vec2 beyond = {0.0f, 2.0f * M2_HUGE};
    m2JointId joint = m2_nullJointId;
    uint64_t misuse = m2World_GetCounters(world).misuse;

    m2DistanceJointDef distance = m2DefaultDistanceJointDef();
    distance.bodyIdA = a;
    distance.bodyIdB = b;
    distance.localAnchorA = beyond;
    m2RevoluteJointDef revolute = m2DefaultRevoluteJointDef();
    revolute.bodyIdA = a;
    revolute.bodyIdB = b;
    revolute.localAnchorB = beyond;
    m2PrismaticJointDef prismatic = m2DefaultPrismaticJointDef();
    prismatic.bodyIdA = a;
    prismatic.bodyIdB = b;
    prismatic.localAxisA = beyond;
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = a;
    weld.bodyIdB = b;
    weld.localAnchorA = beyond;
    m2WheelJointDef wheel = m2DefaultWheelJointDef();
    wheel.bodyIdA = a;
    wheel.bodyIdB = b;
    wheel.localAnchorB = beyond;
    m2PulleyJointDef pulley = m2DefaultPulleyJointDef();
    pulley.bodyIdA = a;
    pulley.bodyIdB = b;
    pulley.groundAnchorA = (m2Pos2){0.0, 5.0};
    pulley.groundAnchorB = (m2Pos2){2.0, 5.0};
    pulley.localAnchorA = beyond;
    m2MotorJointDef motor = m2DefaultMotorJointDef();
    motor.bodyIdA = a;
    motor.bodyIdB = b;
    motor.linearOffset = beyond;
    CHECK(m2CreateDistanceJoint(world, &distance, &joint) == m2_errorInvalid &&
              m2CreateRevoluteJoint(world, &revolute, &joint) == m2_errorInvalid &&
              m2CreatePrismaticJoint(world, &prismatic, &joint) == m2_errorInvalid &&
              m2CreateWeldJoint(world, &weld, &joint) == m2_errorInvalid &&
              m2CreateWheelJoint(world, &wheel, &joint) == m2_errorInvalid &&
              m2CreatePulleyJoint(world, &pulley, &joint) == m2_errorInvalid &&
              m2CreateMotorJoint(world, &motor, &joint) == m2_errorInvalid,
          "every joint def past M2_HUGE is refused");
    CHECK(m2World_GetCounters(world).misuse == misuse + 7, "each refusal counts one misuse");

    // The mouse joint's grab point is where the target lies in B's frame:
    // a target past M2_HUGE from B would make an anchor no snapshot keeps.
    m2MouseJointDef mouse = m2DefaultMouseJointDef();
    mouse.bodyIdA = a;
    mouse.bodyIdB = b;
    mouse.target = (m2Pos2){2.0 + 2.0 * (double)M2_HUGE, 0.0};
    CHECK(m2CreateMouseJoint(world, &mouse, &joint) == m2_errorInvalid &&
              m2World_GetCounters(world).misuse == misuse + 8,
          "a mouse target past M2_HUGE from its body is refused");
    mouse.target = (m2Pos2){2.0 + (double)M2_HUGE, 0.0};
    CHECK(m2CreateMouseJoint(world, &mouse, &joint) == m2_success, "one at M2_HUGE is created");
    misuse += 1;
    int32_t size = m2World_SnapshotSize(world);
    void* snapshot = malloc((size_t)size);
    CHECK(snapshot != NULL && Snapshot(world, snapshot, size) == size &&
              Restore(world, snapshot, size),
          "and its world's snapshot restores");
    free(snapshot);

    motor.linearOffset = (m2Vec2){1.0f, 0.0f};
    m2JointId mover = NewMotorJoint(world, &motor);
    m2MotorJoint_SetOffsets(world, mover, beyond, 0.0f);
    m2Vec2 kept = m2MotorJoint_GetLinearOffset(world, mover);
    CHECK(m2World_GetCounters(world).misuse == misuse + 8 && kept.x == 1.0f && kept.y == 0.0f,
          "a motor offset past M2_HUGE changes nothing and counts one misuse");
    m2DestroyWorld(world);
}

// One joint of the given type between a and b, every row it has switched
// on: limits, motors and springs. Returns false for a type with no def
// here.
static bool FarJoint(m2World* world, m2JointType type, m2BodyId a, m2BodyId b)
{
    m2JointId joint = m2_nullJointId;
    switch (type)
    {
    case m2_distanceJoint:
    {
        m2DistanceJointDef def = m2DefaultDistanceJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.minLength = 0.5f;
        def.maxLength = 2.0f;
        def.enableSpring = true;
        def.springHertz = 4.0f;
        joint = NewDistanceJoint(world, &def);
        break;
    }
    case m2_revoluteJoint:
    {
        m2RevoluteJointDef def = m2DefaultRevoluteJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.localAnchorA = (m2Vec2){0.5f, 0.0f};
        def.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        def.springHertz = 2.0f;
        def.enableMotor = true;
        def.motorSpeed = 1.0f;
        def.maxMotorTorque = 10.0f;
        def.enableLimit = true;
        def.lowerAngle = -0.5f;
        def.upperAngle = 0.5f;
        joint = NewRevoluteJoint(world, &def);
        break;
    }
    case m2_prismaticJoint:
    {
        m2PrismaticJointDef def = m2DefaultPrismaticJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.enableMotor = true;
        def.motorSpeed = 1.0f;
        def.maxMotorForce = 10.0f;
        def.enableLimit = true;
        def.lowerTranslation = -1.0f;
        def.upperTranslation = 1.0f;
        joint = NewPrismaticJoint(world, &def);
        break;
    }
    case m2_weldJoint:
    {
        m2WeldJointDef def = m2DefaultWeldJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.localAnchorA = (m2Vec2){0.5f, 0.0f};
        def.localAnchorB = (m2Vec2){-0.5f, 0.0f};
        joint = NewWeldJoint(world, &def);
        break;
    }
    case m2_wheelJoint:
    {
        m2WheelJointDef def = m2DefaultWheelJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.enableSpring = true;
        def.springHertz = 4.0f;
        def.enableMotor = true;
        def.motorSpeed = 1.0f;
        def.maxMotorTorque = 10.0f;
        def.enableLimit = true;
        def.lowerTranslation = -1.0f;
        def.upperTranslation = 1.0f;
        joint = NewWheelJoint(world, &def);
        break;
    }
    case m2_filterJoint:
    {
        m2FilterJointDef def = m2DefaultFilterJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        joint = NewFilterJoint(world, &def);
        break;
    }
    case m2_motorJoint:
    {
        m2MotorJointDef def = m2DefaultMotorJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.linearOffset = (m2Vec2){1.0f, 0.0f};
        def.maxForce = 100.0f;
        def.maxTorque = 10.0f;
        joint = NewMotorJoint(world, &def);
        break;
    }
    case m2_mouseJoint:
    {
        m2MouseJointDef def = m2DefaultMouseJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.target = (m2Pos2){1.0, 0.0};
        def.maxForce = 100.0f;
        joint = NewMouseJoint(world, &def);
        break;
    }
    case m2_gearJoint:
    {
        m2GearJointDef def = m2DefaultGearJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.ratio = 2.0f;
        joint = NewGearJoint(world, &def);
        break;
    }
    case m2_pulleyJoint:
    {
        m2PulleyJointDef def = m2DefaultPulleyJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.groundAnchorA = (m2Pos2){0.0, 3.0};
        def.groundAnchorB = (m2Pos2){1.0, 3.0};
        joint = NewPulleyJoint(world, &def);
        break;
    }
    case m2_ratchetJoint:
    {
        m2RatchetJointDef def = m2DefaultRatchetJointDef();
        def.bodyIdA = a;
        def.bodyIdB = b;
        def.ratchet = 0.5f;
        joint = NewRatchetJoint(world, &def);
        break;
    }
    default:
        return false;
    }
    return m2Joint_IsValid(world, joint);
}

static bool BodyFinite(const m2World* world, m2BodyId body)
{
    m2Pos2 p = m2Body_GetPosition(world, body);
    m2Vec2 v = m2Body_GetLinearVelocity(world, body);
    float w = m2Body_GetAngularVelocity(world, body);
    return isfinite(p.x) && isfinite(p.y) && isfinite(v.x) && isfinite(v.y) && isfinite(w);
}

// Two jointed dynamic bodies moved far apart with m2Body_SetTransform:
// world positions are 64-bit and unbounded, so their separation can pass
// what a float holds. With type -1 there is no joint, the control.
static bool FarApartStaysFinite(int32_t type, int32_t placement)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    m2World* world = NewWorld(&def);
    m2World_SetGravity(world, (m2Vec2){0.0f, -10.0f});
    m2Polygon box = m2MakeBox(0.25f, 0.25f);
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    m2BodyId a = NewBody(world, &bodyDef);
    NewPolygonShape(world, a, &shapeDef, &box);
    bodyDef.position = (m2Pos2){1.0, 0.0};
    m2BodyId b = NewBody(world, &bodyDef);
    NewPolygonShape(world, b, &shapeDef, &box);
    bool made = type < 0 || FarJoint(world, (m2JointType)type, a, b);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Rot identity = {1.0f, 0.0f};
    if (placement == 0)
    {
        m2Body_SetTransform(world, b, (m2Pos2){1e40, 1e40}, identity);
    }
    else if (placement == 1)
    {
        m2Body_SetTransform(world, b, (m2Pos2){DBL_MAX, DBL_MAX}, identity);
    }
    else
    {
        m2Body_SetTransform(world, a, (m2Pos2){-1e40, 0.0}, identity);
        m2Body_SetTransform(world, b, (m2Pos2){1e40, 0.0}, identity);
    }
    bool finite = made;
    for (int32_t i = 0; i < 8 && finite; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        finite = m2World_Validate(world) && BodyFinite(world, a) && BodyFinite(world, b);
    }
    m2DestroyWorld(world);
    return finite;
}

static void TestFarApartJointsStayFinite(void)
{
    static const char* const placements[3] = {"1e40", "the largest double", "-1e40 and +1e40"};
    for (int32_t placement = 0; placement < 3; ++placement)
    {
        for (int32_t type = -1; type <= (int32_t)m2_ratchetJoint; ++type)
        {
            if (!FarApartStaysFinite(type, placement))
            {
                printf("joint type %d at %s\n", (int)type, placements[placement]);
                CHECK(false, "jointed bodies far apart stay valid and finite");
            }
        }
    }
}

// ---- Long arms against small inertias ---------------------------------------

// The revolute, weld, motor and mouse joints pin a point of body B. When
// B's arm is long against its inertia, the mass the pin sees turns sharply
// with the arm, so each substep builds it from the arm it has reached, in
// double: the point pair's 2x2, the weld's 3x3 with its angle, the mouse's
// 2x2 with its spring.

typedef enum
{
    PIN_REVOLUTE,
    PIN_WELD,
    PIN_MOTOR,
    PIN_MOUSE
} PinKind;

// A 1 kg body of the given central inertia, its point `arm` (in its own
// frame, from its center) pinned to the world origin by a joint of the
// given kind and falling from rest. The motor joint pins B's origin, so
// there the center sits at -arm from it. Returns the joint.
static m2JointId PinBody(m2World* world, PinKind kind, float inertia, m2Vec2 arm, m2BodyId* out)
{
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bool atOrigin = kind == PIN_MOTOR;
    bd.position = atOrigin ? (m2Pos2){0.0, 0.0} : (m2Pos2){(double)-arm.x, (double)-arm.y};
    m2BodyId body = NewBody(world, &bd);
    m2MassData mass = {1.0f, {0.0f, 0.0f}, inertia};
    if (atOrigin)
    {
        mass.center = (m2Vec2){-arm.x, -arm.y};
    }
    m2Body_SetMassData(world, body, mass);
    *out = body;
    if (kind == PIN_REVOLUTE)
    {
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = body;
        jd.localAnchorB = arm;
        return NewRevoluteJoint(world, &jd);
    }
    if (kind == PIN_WELD)
    {
        m2WeldJointDef jd = m2DefaultWeldJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = body;
        jd.localAnchorB = arm;
        return NewWeldJoint(world, &jd);
    }
    if (kind == PIN_MOTOR)
    {
        m2MotorJointDef jd = m2DefaultMotorJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = body;
        jd.maxForce = 1000.0f;
        jd.maxTorque = 0.0f; // free to swing
        return NewMotorJoint(world, &jd);
    }
    m2MouseJointDef jd = m2DefaultMouseJointDef();
    jd.bodyIdA = ground;
    jd.bodyIdB = body;
    jd.target = (m2Pos2){0.0, 0.0};
    jd.springHertz = 10.0f;
    jd.maxForce = 1000.0f;
    return NewMouseJoint(world, &jd);
}

// The farthest the pinned point strays from the origin over two seconds.
static double PinnedDrift(PinKind kind, float inertia, m2Vec2 arm)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    m2BodyId body;
    m2JointId joint = PinBody(world, kind, inertia, arm, &body);
    CHECK(joint.index1 != 0, "the pinning joint is made");
    m2Vec2 point = kind == PIN_MOTOR ? (m2Vec2){0.0f, 0.0f} : arm;
    double worst = 0.0;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2Pos2 p = m2Body_GetPosition(world, body);
        m2Rot q = m2Body_GetRotation(world, body);
        double x = p.x + (double)(q.c * point.x - q.s * point.y);
        double y = p.y + (double)(q.s * point.x + q.c * point.y);
        double d = sqrt(x * x + y * y);
        worst = d > worst || isnan(d) ? d : worst; // a NaN sticks
    }
    m2DestroyWorld(world);
    return worst;
}

static void TestBallPendulumHolds(void)
{
    // A ball on a 1 m revolute pendulum, released level: radii 5 cm and
    // 1 cm put mass * arm^2 / inertia at 800 and 2e4.
    const float radii[2] = {0.05f, 0.01f};
    for (int32_t k = 0; k < 2; ++k)
    {
        m2WorldDef def = m2DefaultWorldDef();
        m2World* world = NewWorld(&def);
        m2BodyDef gd = m2DefaultBodyDef();
        m2BodyId ground = NewBody(world, &gd);
        m2BodyId ball = AddBall(world, 1.0, 0.0, radii[k]);
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = ball;
        jd.localAnchorB = (m2Vec2){-1.0f, 0.0f};
        NewRevoluteJoint(world, &jd);
        double worst = 0.0;
        for (int32_t i = 0; i < 300; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
            double stretch =
                fabs(sqrt(Distance((m2Pos2){0.0, 0.0}, m2Body_GetPosition(world, ball))) - 1.0);
            worst = stretch > worst || isnan(stretch) ? stretch : worst;
        }
        if (!(worst < 0.01))
        {
            printf("ball of radius %g strayed %g m\n", (double)radii[k], worst);
        }
        CHECK(worst < 0.01, "a small ball on a long pendulum keeps its arm");
        m2DestroyWorld(world);
    }
}

static void TestLongArmsHold(void)
{
    // Each joint up to the ratio mass * arm^2 / inertia it holds, with the
    // arm at 0, 15, 30, 45 and 60 degrees. The revolute and the motor pin
    // the point with the point pair, the weld with its 3x3 block and the
    // mouse with a spring of the body's own mass; the mouse's bound is
    // that 10 Hz spring's own give.
    static const struct
    {
        PinKind kind;
        const char* name;
        float maxRatio;
        double bound;
    } joints[4] = {{PIN_REVOLUTE, "revolute", 1.0e7f, 0.01},
                   {PIN_MOTOR, "motor", 1.0e8f, 0.01},
                   {PIN_MOUSE, "mouse", 1.0e7f, 0.02},
                   {PIN_WELD, "weld", 1.0e8f, 0.01}};
    const float degrees[5] = {0.0f, 15.0f, 30.0f, 45.0f, 60.0f};
    for (int32_t k = 0; k < 4; ++k)
    {
        for (float ratio = 10.0f; ratio <= joints[k].maxRatio; ratio *= 10.0f)
        {
            for (int32_t a = 0; a < 5; ++a)
            {
                float turn = degrees[a] * (M2_PI / 180.0f);
                m2Vec2 arm = {cosf(turn), sinf(turn)};
                double drift = PinnedDrift(joints[k].kind, 1.0f / ratio, arm);
                if (!(drift < joints[k].bound))
                {
                    printf("%s at ratio %g, %g degrees: drift %g m\n", joints[k].name,
                           (double)ratio, (double)degrees[a], drift);
                }
                CHECK(drift < joints[k].bound, "a long arm against a small inertia stays pinned");
            }
        }
    }
}

static void TestNearSingularPointPairActs(void)
{
    // At 45 degrees with mass * arm^2 / inertia at 1e8, the point mass's
    // two squares k11 k22 and k12^2 are nearly equal, and the expanded
    // determinant keeps it positive. One step from rest, the revolute
    // carries the weight's share along the arm, m g cos 45 = 7.07 N,
    // while the body turns freely across it, and so does the motor; the
    // weld does not let it turn and carries the whole weight. The mouse is
    // a spring, which takes more than one step to carry the load.
    const PinKind kinds[3] = {PIN_REVOLUTE, PIN_WELD, PIN_MOTOR};
    const float inertias[2] = {1.0e-8f, 1.0e-12f};
    for (int32_t k = 0; k < 3; ++k)
    {
        for (int32_t i = 0; i < 2; ++i)
        {
            m2WorldDef def = m2DefaultWorldDef();
            m2World* world = NewWorld(&def);
            m2BodyId body;
            m2JointId joint =
                PinBody(world, kinds[k], inertias[i], (m2Vec2){0.70710678f, 0.70710678f}, &body);
            m2World_Step(world, 1.0f / 60.0f, 4);
            float carried = m2Joint_GetReactionForce(world, joint);
            float share = 1.0f * -def.gravity.y * (kinds[k] == PIN_WELD ? 1.0f : 0.70710678f);
            if (!(fabsf(carried - share) < 0.1f * share))
            {
                printf("joint kind %d, inertia %g: carries %g N of %g\n", (int)kinds[k],
                       (double)inertias[i], (double)carried, (double)share);
            }
            CHECK(fabsf(carried - share) < 0.1f * share,
                  "a near-singular pin still carries its load");
            m2DestroyWorld(world);
        }
    }
    // And it holds: at 45 degrees and a ratio of 1e8 the anchor stays
    // within 1 cm for two seconds.
    double drift = PinnedDrift(PIN_REVOLUTE, 1.0e-8f, (m2Vec2){0.70710678f, 0.70710678f});
    if (!(drift < 0.01))
    {
        printf("revolute at ratio 1e8, 45 degrees: drift %g m\n", drift);
    }
    CHECK(drift < 0.01, "a near-singular point pair holds its anchor");
}

// One joint of each kind with a spring, every hertz at the smallest
// float a rate takes and every damping ratio 0, between the ground and a
// ball.
static m2JointId VanishingSpring(m2World* world, int32_t kind, m2BodyId ground, m2BodyId box)
{
    const float hertz = FLT_TRUE_MIN;
    if (kind == 0)
    {
        m2WeldJointDef jd = m2DefaultWeldJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.linearSpringHertz = hertz;
        jd.linearSpringDampingRatio = 0.0f;
        jd.angularSpringHertz = hertz;
        jd.angularSpringDampingRatio = 0.0f;
        return NewWeldJoint(world, &jd);
    }
    if (kind == 1)
    {
        m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.constraintHertz = hertz;
        jd.constraintDampingRatio = 0.0f;
        jd.enableSpring = true;
        jd.springHertz = hertz;
        jd.springDampingRatio = 0.0f;
        return NewRevoluteJoint(world, &jd);
    }
    if (kind == 2)
    {
        m2PrismaticJointDef jd = m2DefaultPrismaticJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.constraintHertz = hertz;
        jd.constraintDampingRatio = 0.0f;
        return NewPrismaticJoint(world, &jd);
    }
    if (kind == 3)
    {
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.enableSpring = true;
        jd.springHertz = hertz;
        jd.springDampingRatio = 0.0f;
        return NewDistanceJoint(world, &jd);
    }
    if (kind == 4)
    {
        m2WheelJointDef jd = m2DefaultWheelJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.enableSpring = true;
        jd.springHertz = hertz;
        jd.springDampingRatio = 0.0f;
        return NewWheelJoint(world, &jd);
    }
    if (kind == 5)
    {
        m2MotorJointDef jd = m2DefaultMotorJointDef();
        jd.bodyIdA = ground;
        jd.bodyIdB = box;
        jd.springHertz = hertz;
        jd.springDampingRatio = 0.0f;
        return NewMotorJoint(world, &jd);
    }
    m2MouseJointDef jd = m2DefaultMouseJointDef();
    jd.bodyIdA = ground;
    jd.bodyIdB = box;
    jd.target = (m2Pos2){0.0, 2.0};
    jd.springHertz = hertz;
    jd.springDampingRatio = 0.0f;
    return NewMouseJoint(world, &jd);
}

static void TestVanishingSpringsStayFinite(void)
{
    // h w rounds to zero at such a hertz, and the bias was 0 * inf: the
    // first step turned the box's position into a NaN. A spring that
    // weak pulls with nothing.
    static const char* const names[7] = {"weld",  "revolute", "prismatic", "distance",
                                         "wheel", "motor",    "mouse"};
    for (int32_t kind = 0; kind < 7; ++kind)
    {
        m2WorldDef def = m2DefaultWorldDef();
        m2World* world = NewWorld(&def);
        m2BodyDef gd = m2DefaultBodyDef();
        m2BodyId ground = NewBody(world, &gd);
        m2BodyId box = AddBall(world, 0.0, 2.0, 0.25f);
        m2JointId joint = VanishingSpring(world, kind, ground, box);
        CHECK(joint.index1 != 0, "the smallest hertz is a rate a def takes");
        for (int32_t i = 0; i < 10; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        m2Pos2 p = m2Body_GetPosition(world, box);
        if (!(isfinite(p.x) && isfinite(p.y) && m2World_Validate(world)))
        {
            printf("%s joint with a vanishing spring\n", names[kind]);
        }
        CHECK(isfinite(p.x) && isfinite(p.y) && m2World_Validate(world),
              "a vanishing spring leaves the world finite");
        m2DestroyWorld(world);
    }
}

// The switches a def sets can change on a live joint: connected
// collision on any kind but the filter joint, and the spring on distance
// and wheel joints. Each reads back, and another kind counts a misuse.
static void TestJointSwitches(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 0.5};
    m2BodyId pedestal = NewBody(world, &pd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon top = m2MakeBox(1.0f, 0.5f);
    NewPolygonShape(world, pedestal, &fs, &top);
    m2Polygon floorBox = m2MakeBox(12.0f, 0.5f);
    pd.position = (m2Pos2){0.0, -1.5};
    NewPolygonShape(world, NewBody(world, &pd), &fs, &floorBox);
    m2BodyDef cd = m2DefaultBodyDef();
    cd.type = m2_dynamicBody;
    cd.position = (m2Pos2){0.0, 1.45};
    m2BodyId crate = NewBody(world, &cd);
    m2Polygon unit = m2MakeBox(0.4f, 0.4f);
    NewPolygonShape(world, crate, &fs, &unit);

    // A slack rope (spring on, hertz 0, no range) that lets its bodies
    // touch: the crate rests on the pedestal.
    m2DistanceJointDef rope = m2DefaultDistanceJointDef();
    rope.bodyIdA = pedestal;
    rope.bodyIdB = crate;
    rope.enableSpring = true;
    rope.enableConnectedCollision = true;
    m2JointId tie = NewDistanceJoint(world, &rope);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Joint_IsConnectedCollisionEnabled(world, tie) &&
              m2Body_GetPosition(world, crate).y > 1.2,
          "a rope that lets its bodies collide keeps the crate on the pedestal");
    m2Joint_EnableConnectedCollision(world, tie, false);
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Joint_IsConnectedCollisionEnabled(world, tie) &&
              m2Body_GetPosition(world, crate).y < 0.0,
          "turned off, the crate falls through the pedestal");
    m2Joint_EnableConnectedCollision(world, tie, true);
    m2Body_SetTransform(world, crate, (m2Pos2){0.0, 2.0}, (m2Rot){1.0f, 0.0f});
    m2Body_SetLinearVelocity(world, crate, (m2Vec2){0.0f, 0.0f});
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Joint_IsConnectedCollisionEnabled(world, tie) &&
              m2Body_GetPosition(world, crate).y > 1.2,
          "turned back on, a dropped crate rests on the pedestal again");

    CHECK(m2Joint_IsSpringEnabled(world, tie), "the rope's spring reads on");
    m2Joint_EnableSpring(world, tie, false);
    CHECK(!m2Joint_IsSpringEnabled(world, tie), "and off once turned off");

    m2FilterJointDef fj = m2DefaultFilterJointDef();
    fj.bodyIdA = pedestal;
    fj.bodyIdB = crate;
    m2JointId pact = NewFilterJoint(world, &fj);
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Joint_EnableConnectedCollision(world, pact, true);
    m2Joint_EnableSpring(world, pact, true);
    CHECK(!m2Joint_IsConnectedCollisionEnabled(world, pact) &&
              !m2Joint_IsSpringEnabled(world, pact) &&
              m2World_GetCounters(world).misuse == misuse + 3,
          "a filter joint refuses both switches and has no spring, each a misuse");
    CHECK(m2World_Validate(world), "the world stays valid");
    m2DestroyWorld(world);
}

int main(void)
{
    TestFarApartJointsStayFinite();
    TestVanishingSpringsStayFinite();
    TestBallPendulumHolds();
    TestLongArmsHold();
    TestNearSingularPointPairActs();
    TestLocalAnchorsAreBounded();
    TestJointAdjacency();
    TestDistancePendulum();
    TestDistanceRope();
    TestRevoluteChain();
    TestJointsCoupleIslands();
    TestJointRollback();
    TestRevoluteMotor();
    TestRevoluteLimit();
    TestPrismaticSlider();
    TestPrismaticMotor();
    TestPrismaticStressStability();
    TestWeldRigid();
    TestWheelSuspension();
    TestWheelCar();
    TestJointRuntimeTuning();
    TestSoftWeld();
    TestReactionGetters();
    TestConnectedCollisionAndFilterJoint();
    TestJointSwitches();
    TestMotorAndMouseJoints();
    TestMotorSpring();
    TestDistanceRange();
    TestDistanceSpringToggleReadsBack();
    TestGearJoint();
    TestRatchetJoint();
    TestMotorAndLimitCallsNeedTheirJoint();
    TestPulleyJoint();
    TestRevoluteSpring();
    TestJointBreaking();
    TestBreakRollback();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
