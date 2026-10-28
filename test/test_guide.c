// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The C snippets of docs/user/, built and run as written. Each test sets
// up what its snippet assumes, runs the snippet unchanged, and checks the
// outcome the page describes. tools/check_guide.py fails when a snippet on
// a page no longer appears here.

#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <stdlib.h>

// A world with a static floor whose top is y = 0.
static m2World* FloorWorld(const m2WorldDef* def)
{
    m2World* world = NULL;
    if (m2CreateWorld(def, &world) != m2_success)
    {
        return NULL;
    }
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -0.5};
    m2BodyId ground = m2_nullBodyId;
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(20.0f, 0.5f);
    m2ShapeId floorShape = m2_nullShapeId;
    if (m2CreateBody(world, &groundDef, &ground) != m2_success ||
        m2CreatePolygonShape(world, ground, &shapeDef, &floor, &floorShape) != m2_success)
    {
        m2DestroyWorld(world);
        return NULL;
    }
    return world;
}

// A dynamic ball of radius 0.5 at a height.
static m2BodyId DropBall(m2World* world, double height)
{
    m2BodyDef ballDef = m2DefaultBodyDef();
    ballDef.type = m2_dynamicBody;
    ballDef.position = (m2Pos2){0.0, height};
    m2BodyId ball = m2_nullBodyId;
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2ShapeId shape = m2_nullShapeId;
    if (m2CreateBody(world, &ballDef, &ball) != m2_success ||
        m2CreateCircleShape(world, ball, &shapeDef, &circle, &shape) != m2_success)
    {
        return m2_nullBodyId;
    }
    return ball;
}

// getting-started.md: a first world. The snippet returns early when the
// world is refused, so the test reports whether it ran to the end.
static void FirstWorld(bool* finished)
{
    m2WorldDef worldDef = m2DefaultWorldDef();
    m2World* world = NULL;
    if (m2CreateWorld(&worldDef, &world) != m2_success)
    {
        return; // the CPU, the def or the allocator refused
    }

    m2BodyDef groundDef = m2DefaultBodyDef();
    m2BodyId ground = m2_nullBodyId;
    m2ShapeDef shapeDef = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(10.0f, 0.5f);
    m2ShapeId floorShape = m2_nullShapeId;

    m2BodyDef ballDef = m2DefaultBodyDef();
    ballDef.type = m2_dynamicBody;
    ballDef.position = (m2Pos2){0.0, 4.0};
    m2BodyId ball = m2_nullBodyId;
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    m2ShapeId ballShape = m2_nullShapeId;

    bool built =
        m2CreateBody(world, &groundDef, &ground) == m2_success &&
        m2CreatePolygonShape(world, ground, &shapeDef, &floor, &floorShape) == m2_success &&
        m2CreateBody(world, &ballDef, &ball) == m2_success &&
        m2CreateCircleShape(world, ball, &shapeDef, &circle, &ballShape) == m2_success;

    for (int32_t i = 0; built && i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2Pos2 resting = m2Body_GetPosition(world, ball);
    m2DestroyWorld(world);

    *finished = true;
    CHECK(built, "the first world builds");
    CHECK(resting.y > 0.9 && resting.y < 1.1, "the ball rests on the floor");
}

static void TestFirstWorld(void)
{
    bool finished = false;
    FirstWorld(&finished);
    CHECK(finished, "the first world is created");
}

// bodies-and-shapes.md: an outline into pieces, then a shatter.
static void TestOutlineAndShatter(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    CHECK(world != NULL, "the world builds");
    m2BodyDef wallDef = m2DefaultBodyDef();
    wallDef.type = m2_dynamicBody;
    m2BodyId wall = m2_nullBodyId;
    CHECK(m2CreateBody(world, &wallDef, &wall) == m2_success, "the wall body builds");
    m2ShapeDef shapeDef = m2DefaultShapeDef();

    m2Vec2 outline[6] = {{-2.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f},
                         {0.5f, 1.0f},  {0.5f, 2.0f}, {-2.0f, 2.0f}};
    m2Polygon pieces[8];
    int32_t pieceCount = 0;
    bool whole =
        m2DecomposeOutline(outline, 6, pieces, 8, &pieceCount) == m2_success && pieceCount <= 8;
    for (int32_t i = 0; whole && i < pieceCount; ++i)
    {
        m2ShapeId piece = m2_nullShapeId;
        whole = m2CreatePolygonShape(world, wall, &shapeDef, &pieces[i], &piece) == m2_success;
    }

    // Later, when the wall breaks:
    m2BodyId fragments[8];
    int32_t fragmentCount = 0;
    m2Result broke =
        m2World_ShatterBody(world, wall, pieces, pieceCount, fragments, 8, &fragmentCount);

    CHECK(whole && pieceCount >= 2, "the L outline becomes convex pieces on the wall");
    CHECK(broke == m2_success && fragmentCount == pieceCount,
          "the wall breaks into one body per piece");
    CHECK(!m2Body_IsValid(world, wall), "and the wall itself is gone");
    m2DestroyWorld(world);
}

// events-and-queries.md: counting hard hits from the contact events.
static void TestHardHits(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    CHECK(world != NULL, "the world builds");
    CHECK(DropBall(world, 3.0).index1 != 0, "a ball drops from three meters");
    int32_t hardHits = 0;
    for (int32_t step = 0; step < 90; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2ContactEvents contacts = m2World_GetContactEvents(world);
        for (int32_t i = 0; i < contacts.beginCount; ++i)
        {
            const m2ContactBeginEvent* begin = &contacts.beginEvents[i];
            if (begin->approachSpeed > 2.0f)
            {
                hardHits += 1; // a sound at begin->points[0], louder for faster hits
            }
        }
    }
    CHECK(hardHits == 1, "the landing is one hard hit");
    m2DestroyWorld(world);
}

// events-and-queries.md: one frame of the character mover.
static void TestMoverFrame(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    CHECK(world != NULL, "the world builds");
    m2Transform pose = {{0.0, 0.81}, {1.0f, 0.0f}};
    m2Vec2 wish = {0.2f, -0.5f};
    m2Vec2 velocity = {2.0f, -5.0f};

    m2Capsule capsule = {{0.0f, -0.5f}, {0.0f, 0.5f}, 0.3f};
    m2MoverPlane planes[M2_MAX_MOVER_PLANES];
    int32_t planeCount = m2World_CollideMover(world, &capsule, pose, m2DefaultQueryFilter(), planes,
                                              M2_MAX_MOVER_PLANES);
    if (planeCount > M2_MAX_MOVER_PLANES)
    {
        planeCount = M2_MAX_MOVER_PLANES;
    }
    m2MoverMove move = m2SolveMover(wish, planes, planeCount);
    pose.p.x += (double)move.translation.x;
    pose.p.y += (double)move.translation.y;
    velocity = m2ClipMoverVelocity(velocity, planes, planeCount, move.pressed);

    CHECK(planeCount >= 1, "the floor is a plane");
    CHECK(pose.p.y > 0.79, "the floor stops the downward wish");
    CHECK(pose.p.x > 0.19, "and the sideways part goes through");
    CHECK(velocity.y > -0.01f && velocity.x > 1.99f, "the velocity into the floor is spent");
    m2DestroyWorld(world);
}

// fluids.md: a particle with a lifetime.
static void TestExpiringDrop(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.particleCapacity = 16;
    m2World* world = FloorWorld(&def);
    CHECK(world != NULL, "the world builds");
    uint64_t misuse = m2World_GetCounters(world).misuse;

    m2ParticleDef dropDef = m2DefaultParticleDef();
    dropDef.position = (m2Pos2){0.0, 2.0};
    dropDef.velocity = (m2Vec2){0.0f, -1.0f};
    dropDef.flags = m2_tensileParticle;
    m2ParticleId drop = m2_nullParticleId;
    m2Result created = m2CreateParticle(world, &dropDef, &drop);
    if (created == m2_success)
    {
        m2Particle_SetLifetime(world, drop, 0.5f); // gone half a second from now
    }
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    bool stillThere = m2Particle_IsValid(world, drop); // false: it expired

    CHECK(created == m2_success, "the drop is created");
    CHECK(!stillThere, "and expires on its own");
    CHECK(m2Particle_GetPosition(world, drop).x == 0.0, "its id reads as zero");
    CHECK(m2World_GetCounters(world).misuse == misuse, "without counting misuse");
    m2DestroyWorld(world);
}

// rollback-and-replay.md: a mispredicted frame rolled back.
static void TestRollbackFrame(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    m2World* twin = FloorWorld(&def);
    CHECK(world != NULL && twin != NULL, "two worlds build");
    m2BodyId ball = DropBall(world, 2.0);
    CHECK(DropBall(twin, 2.0).index1 != 0 && ball.index1 != 0, "with a ball each");

    int64_t size = m2World_SnapshotSize(world);
    void* frame = malloc((size_t)size);
    int64_t written = 0;
    bool saved = frame != NULL && m2World_Snapshot(world, frame, size, &written) == m2_success;

    m2Body_ApplyLinearImpulse(world, ball, (m2Vec2){3.0f, 0.0f}); // a mispredicted input
    m2World_Step(world, 1.0f / 60.0f, 4);

    if (saved && m2World_Restore(world, frame, written) == m2_success)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // the corrected frame
    }
    free(frame);

    m2World_Step(twin, 1.0f / 60.0f, 4);
    CHECK(saved, "the frame is saved");
    CHECK(m2World_Hash(world) == m2World_Hash(twin), "the rollback lands on the right frame");
    m2DestroyWorld(twin);
    m2DestroyWorld(world);
}

// rollback-and-replay.md: a session recorded and replayed.
static void TestRecordAndReplay(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    m2World* replica = NULL;
    CHECK(world != NULL && m2CreateWorld(&def, &replica) == m2_success, "two worlds build");
    CHECK(DropBall(world, 2.0).index1 != 0, "a ball drops");

    int64_t capacity = m2World_GetJournalBaseSize(world) + 64 * 1024;
    void* tape = malloc((size_t)capacity);
    int64_t tapeSize = 0;
    bool recorded = tape != NULL && m2World_StartJournal(world, tape, capacity) == m2_success;
    for (int32_t i = 0; recorded && i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    recorded = recorded && m2World_StopJournal(world, &tapeSize) == m2_success;

    // In another world built from the same def:
    bool same = recorded && m2World_ReplayJournal(replica, tape, tapeSize) == m2_success &&
                m2World_Hash(replica) == m2World_Hash(world);
    free(tape);

    CHECK(same, "the replica replays onto the same bits");
    m2DestroyWorld(replica);
    m2DestroyWorld(world);
}

// diagnostics.md: watching the misuse counter.
static void TestMisuseWatch(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = FloorWorld(&def);
    CHECK(world != NULL, "the world builds");
    m2BodyId ball = DropBall(world, 1.0);
    CHECK(m2DestroyBody(world, ball) == m2_success, "a ball comes and goes");
    m2Body_SetLinearVelocity(world, ball, (m2Vec2){1.0f, 0.0f}); // a bug: the ball is gone
    uint64_t lastMisuse = 0;

    m2Counters counters = m2World_GetCounters(world);
    if (counters.misuse > lastMisuse)
    {
        lastMisuse = counters.misuse; // log the frame: some call this frame was a bug
    }

    CHECK(lastMisuse == 1, "the stale call shows in the counter");
    m2DestroyWorld(world);
}

int main(void)
{
    TestFirstWorld();
    TestOutlineAndShatter();
    TestHardHits();
    TestMoverFrame();
    TestExpiringDrop();
    TestRollbackFrame();
    TestRecordAndReplay();
    TestMisuseWatch();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
