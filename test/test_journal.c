// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Journal gate: a recorded session (spawns, shoves, joints, destroys,
// steps) replays into an identical world hash; recorded ids re-mint
// bit-identically; overflow is loud; and the journal BYTES themselves
// hash identically across CI cells - the wire format is part of the
// determinism contract.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2WorldDef TestDef(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 32;
    def.particleCapacity = 64;   // the parade drives the particle ops too
    def.waterVolumeCapacity = 4; // and the buoyancy ops
    return def;
}

// A session with every op type: floor, rain of shapes, a pendulum, a
// shove, a destroy, steps in two dt flavors (markers carry dt unchanged).
static void RunSession(m2World* world, uint8_t* journal, int64_t capacity, int64_t* outSize,
                       uint64_t* outHash)
{
    if (journal != NULL)
    {
        CHECK(StartJournal(world, journal, capacity), "journal starts");
    }

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    m2BodyId victim = m2_nullBodyId;
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-4.0 + 1.1 * (double)i, 2.0 + 0.4 * (double)i};
        m2BodyId body = NewBody(world, &bd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.restitution = 0.3f;
        if (i % 2 == 0)
        {
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            NewCircleShape(world, body, &sd, &c);
        }
        else
        {
            m2Polygon box = m2MakeBox(0.3f, 0.25f);
            NewPolygonShape(world, body, &sd, &box);
        }
        if (i == 3)
        {
            victim = body;
        }
        if (i == 5)
        {
            m2Body_SetLinearVelocity(world, body, (m2Vec2){-2.5f, 1.0f});
            m2Body_SetAngularVelocity(world, body, 3.0f);
        }
    }

    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){5.0, 5.0};
    m2BodyId anchor = NewBody(world, &ad);
    m2BodyDef bobDef = m2DefaultBodyDef();
    bobDef.type = m2_dynamicBody;
    bobDef.position = (m2Pos2){6.5, 5.0};
    m2BodyId bob = NewBody(world, &bobDef);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle bc = {{0.0f, 0.0f}, 0.2f};
    NewCircleShape(world, bob, &bs, &bc);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = anchor;
    jd.bodyIdB = bob;
    m2JointId rod = NewDistanceJoint(world, &jd);

    // A motorized slider exercises the prismatic create op (op 10).
    m2BodyDef ramDef = m2DefaultBodyDef();
    ramDef.type = m2_dynamicBody;
    ramDef.position = (m2Pos2){9.0, 2.0};
    m2BodyId ram = NewBody(world, &ramDef);
    NewCircleShape(world, ram, &bs, &bc);
    m2BodyDef cylDef = m2DefaultBodyDef();
    cylDef.position = (m2Pos2){9.0, 2.0};
    m2BodyId cylinder = NewBody(world, &cylDef);
    m2PrismaticJointDef press = m2DefaultPrismaticJointDef();
    press.bodyIdA = cylinder;
    press.bodyIdB = ram;
    press.localAxisA = (m2Vec2){0.0f, 1.0f};
    press.enableMotor = true;
    press.motorSpeed = -0.5f;
    press.maxMotorEffort = 20.0f;
    press.enableLimit = true;
    press.lowerTranslation = -0.8f;
    press.upperTranslation = 0.3f;
    m2JointId pressJoint = NewPrismaticJoint(world, &press);

    // Weld and wheel creates put ops 11 and 12 into the gated bytes.
    m2BodyDef sideDef = m2DefaultBodyDef();
    sideDef.type = m2_dynamicBody;
    sideDef.position = (m2Pos2){9.6, 2.0};
    m2BodyId sidecar = NewBody(world, &sideDef);
    NewCircleShape(world, sidecar, &bs, &bc);
    m2Circle spare = {{0.3f, 0.2f}, 0.1f};
    m2ShapeId sacrificial = NewCircleShape(world, sidecar, &bs, &spare);
    m2WeldJointDef weld = m2DefaultWeldJointDef();
    weld.bodyIdA = ram;
    weld.bodyIdB = sidecar;
    NewWeldJoint(world, &weld);
    m2WheelJointDef ride = m2DefaultWheelJointDef();
    ride.bodyIdA = sidecar;
    ride.bodyIdB = bob;
    ride.enableMotor = true;
    ride.motorSpeed = 2.0f;
    ride.maxMotorEffort = 5.0f;
    NewWheelJoint(world, &ride);

    for (int32_t i = 0; i < 45; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    // Shape destruction, impulses and joint tuning all ride the journal
    // too.
    m2Body_ApplyLinearImpulseAtPoint(world, ram, (m2Vec2){0.4f, 0.9f}, (m2Pos2){9.1, 2.0});
    m2Body_ApplyAngularImpulse(world, ram, 0.3f);
    m2Joint_SetMotorSpeed(world, pressJoint, 0.7f);
    m2Body_SetTransform(world, ram, (m2Pos2){8.6, 2.4}, m2MakeRot(0.3f)); // op 17
    m2Body_SetType(world, sidecar, m2_kinematicBody);                     // op 18
    m2Vec2 chainPts[5] = {{12.0f, 0.5f}, {11.0f, 0.0f}, {10.0f, 0.0f}, {9.0f, 0.0f}, {8.0f, 0.5f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = chainPts;
    chain.count = 5;
    m2ChainId chainId = NewChain(world, cylinder, &chain);      // op 20
    m2World_SetGravity(world, (m2Vec2){0.3f, -9.5f});           // op 21
    m2Shape_SetFriction(world, sacrificial, 0.9f);              // op 22 (before it dies)
    m2Shape_SetTangentSpeed(world, sacrificial, 1.25f);         // op 22 param 2
    m2Shape_SetFilter(world, sacrificial, 0x2, 0xFFFFFFFFu, 0); // op 23
    DestroyShape(world, sacrificial);
    for (int32_t i = 0; i < 15; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    DestroyBody(world, victim);
    DestroyJoint(world, rod);
    DestroyChain(world, chainId);                                           // op 24
    m2Body_ApplyForce(world, bob, (m2Vec2){3.0f, 1.0f});                    // op 28
    m2Body_ApplyTorque(world, bob, 0.8f);                                   // op 29
    m2Body_SetLinearDamping(world, bob, 0.2f);                              // op 25
    m2Body_SetMotionLocks(world, bob, (m2MotionLocks){false, false, true}); // op 25
    m2World_EnableSleep(world, false);                                      // op 26
    m2World_EnableSleep(world, true);
    m2MotorJointDef chase = m2DefaultMotorJointDef(); // op 31
    chase.bodyIdA = anchor;
    chase.bodyIdB = bob;
    chase.maxForce = 60.0f;
    chase.maxTorque = 20.0f;
    m2JointId chaser = NewMotorJoint(world, &chase);
    m2MotorJoint_SetOffsets(world, chaser, (m2Vec2){0.5f, 0.5f}, 0.1f); // op 33
    m2MouseJointDef grip = m2DefaultMouseJointDef();                    // op 32
    grip.bodyIdA = floor;
    grip.bodyIdB = ram;
    grip.target = (m2Pos2){2.0, 1.0};
    m2JointId gripper = NewMouseJoint(world, &grip);
    m2MouseJoint_SetTarget(world, gripper, (m2Pos2){3.0, 2.0}); // op 34
    m2Body_Enable(world, ram, false);                           // op 35
    m2Body_Enable(world, ram, true);                            // op 36
    m2MassData heavier = m2Body_GetMassData(world, bob);
    heavier.mass *= 2.0f;
    heavier.centralInertia *= 2.0f;
    m2Body_SetMassData(world, bob, heavier); // op 37
    m2Body_ApplyMassFromShapes(world, bob);  // op 38
    m2ExplosionDef boom = m2DefaultExplosionDef();
    boom.position = (m2Pos2){1.0, 0.5};
    boom.radius = 2.0f;
    boom.impulse = 1.5f;
    m2World_Explode(world, &boom); // op 39
    m2BodyDef rsd = m2DefaultBodyDef();
    rsd.type = m2_dynamicBody;
    rsd.position = (m2Pos2){5.0, 3.0};
    m2BodyId reshapeBody = NewBody(world, &rsd);
    m2ShapeDef rss = m2DefaultShapeDef();
    m2Polygon rsp = m2MakeBox(0.3f, 0.3f);
    m2ShapeId reshapeShape = NewPolygonShape(world, reshapeBody, &rss, &rsp);
    m2Circle reshaped = {{0.0f, 0.0f}, 0.3f};
    m2Shape_SetCircle(world, reshapeShape, &reshaped); // op 40
    m2Vec2 matPts[5] = {{9.0f, 2.0f}, {8.0f, 2.0f}, {7.0f, 2.0f}, {6.0f, 2.0f}, {5.0f, 2.0f}};
    m2ChainDef matChain = m2DefaultChainDef();
    matChain.points = matPts;
    matChain.count = 5;
    m2ChainId matLedge = NewChain(world, cylinder, &matChain);
    m2Chain_SetFriction(world, matLedge, 0.75f);                 // op 41
    m2Chain_SetRestitution(world, matLedge, 0.15f);              // op 42
    m2Body_ApplyLinearImpulse(world, bob, (m2Vec2){0.5f, 0.2f}); // op 43
    m2Body_SetAwake(world, bob, false);                          // op 44
    m2Body_SetAwake(world, bob, true);
    m2Body_EnableBullet(world, bob, true);         // op 45
    m2Shape_SetDensity(world, reshapeShape, 1.5f); // op 46
    m2Body_SetUserData(world, bob, 999);           // op 47
    m2Body_SetDominance(world, bob, 3);            // op 50
    m2Shape_SetUserData(world, reshapeShape, 111); // op 48
    m2Joint_SetUserData(world, gripper, 222);      // op 49
    m2GearJointDef cogDef = m2DefaultGearJointDef();
    cogDef.bodyIdA = bob;
    cogDef.bodyIdB = ram;
    cogDef.ratio = 2.0f;
    m2JointId cog = NewGearJoint(world, &cogDef); // op 51
    m2GearJoint_SetRatio(world, cog, 1.5f);       // param 15
    m2PulleyJointDef hoistDef = m2DefaultPulleyJointDef();
    hoistDef.bodyIdA = cylinder;
    hoistDef.bodyIdB = sidecar;
    hoistDef.groundAnchorA = (m2Pos2){-1.0, 9.0};
    hoistDef.groundAnchorB = (m2Pos2){3.0, 9.0};
    m2JointId hoist = NewPulleyJoint(world, &hoistDef); // op 52
    m2PulleyJoint_SetRatio(world, hoist, 2.0f);         // param 16
    m2RatchetJointDef pawlDef = m2DefaultRatchetJointDef();
    pawlDef.bodyIdA = anchor;
    pawlDef.bodyIdB = sidecar;
    pawlDef.toothAngle = -0.3f;
    NewRatchetJoint(world, &pawlDef); // op 56
    m2RevoluteJointDef swingDef = m2DefaultRevoluteJointDef();
    swingDef.bodyIdA = anchor;
    swingDef.bodyIdB = cylinder;
    swingDef.enableSpring = true;
    swingDef.springHertz = 3.0f; // the def echo carries the spring pair
    swingDef.springDampingRatio = 0.5f;
    m2JointId swing = NewRevoluteJoint(world, &swingDef);
    m2Joint_SetSpringHertz(world, swing, 1.5f);            // param 10 on a revolute
    m2Joint_SetSpringDampingRatio(world, swing, 0.25f);    // param 11 on a revolute
    m2Joint_SetConstraintHertz(world, swing, 40.0f);       // param 8 on a revolute
    m2Joint_SetConstraintDampingRatio(world, swing, 2.0f); // param 9 on a revolute
    m2ParticleId drop = EmitParticle(world, (m2Pos2){0.5, 6.0}, (m2Vec2){0.0f, 0.0f}, 0); // op 53
    m2Particle_SetVelocity(world, drop, (m2Vec2){0.2f, 0.0f});                            // op 55
    m2ParticleId drop2 = EmitParticle(world, (m2Pos2){0.7, 6.0}, (m2Vec2){0.0f, 0.0f}, 0);
    DestroyParticle(world, drop); // op 54
    m2Vec2 shardTri[3] = {{0.0f, 0.0f}, {0.3f, 0.0f}, {0.0f, 0.3f}};
    m2Polygon shard = MakePolygon(shardTri, 3, 0.0f);
    m2Polygon shardPair[2];
    shardPair[0] = shard;
    shardPair[1] = shard;
    ShatterBody(world, ram, shardPair, 2, NULL, 0); // op 58 (ram is dynamic; its gear dies with it)
    m2Particle_SetLifetime(world, drop2, 0.5f);     // op 59
    m2Particle_SetUserData(world, drop2, 777);      // op 60
    m2WaterVolumeDef poolDef = m2DefaultWaterVolumeDef();
    poolDef.bounds = (m2WorldAabb){{-5.0, -5.0}, {5.0, 5.0}};
    poolDef.surface = -1.0;
    m2WaterVolumeId pool = NewWaterVolume(world, &poolDef); // op 61
    m2WaterVolume_SetSurface(world, pool, 0.5);             // op 63
    m2BodyDef drd = m2DefaultBodyDef();
    drd.position = (m2Pos2){-4.0, 6.0};
    m2BodyId drHook = NewBody(world, &drd);
    m2DistanceJointDef drj = m2DefaultDistanceJointDef();
    drj.bodyIdA = drHook;
    drj.bodyIdB = bob;
    drj.enableSpring = true;
    drj.springHertz = 1.0f;
    m2JointId leash = NewDistanceJoint(world, &drj);
    m2DistanceJoint_SetLength(world, leash, 1.8f);            // param 12
    m2DistanceJoint_SetLengthRange(world, leash, 0.4f, 2.2f); // params 13+14
    m2Joint_SetSpringHertz(world, leash, 6.0f);               // param 8
    m2Joint_SetSpringDampingRatio(world, leash, 0.5f);        // param 9
    m2Joint_EnableSpring(world, leash, false);                // param 17
    m2Joint_EnableConnectedCollision(world, leash, true);     // param 18
    m2Pos2 bobAt = m2Body_GetPosition(world, bob);
    m2Body_ApplyForceAtPoint(world, bob, (m2Vec2){-2.0f, 0.5f},
                             (m2Pos2){bobAt.x + 0.25, bobAt.y}); // op 27
    m2World_SetWind(world, (m2Vec2){1.5f, 0.0f}, 0.1f);          // op 64
    for (int32_t i = 0; i < 30; ++i)
    {
        Step(world, 1.0f / 120.0f, 2); // second dt flavor
    }
    CHECK(DestroyWaterVolume(world, pool) == m2_success, "the pool drains"); // op 62
    Step(world, 1.0f / 120.0f, 2);

    if (journal != NULL)
    {
        *outSize = StopJournal(world);
        CHECK(*outSize > 0, "journal stops without overflow");
    }
    *outHash = m2World_Hash(world);
}

static void TestRecordReplay(void)
{
    enum
    {
        CAPACITY = 1 << 20
    };
    uint8_t* journal = malloc(CAPACITY);

    m2WorldDef def = TestDef();
    m2World* original = NewWorld(&def);
    int64_t journalSize = 0;
    uint64_t originalHash = 0;
    RunSession(original, journal, CAPACITY, &journalSize, &originalHash);

    // Replay into a FRESH world of the same shape.
    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, journal, journalSize), "replay accepts the journal");
    CHECK(m2World_Hash(fresh) == originalHash, "replayed world hash matches bit-exactly");
    CHECK(m2World_GetStepCount(fresh) == m2World_GetStepCount(original), "step counts match");

    // Replay is repeatable: run it again on the same world.
    CHECK(ReplayJournal(fresh, journal, journalSize), "replay is idempotent from t0");
    CHECK(m2World_Hash(fresh) == originalHash, "second replay matches too");

    // Wrong-shaped world refuses the journal.
    m2WorldDef other = TestDef();
    other.bodyCapacity = 128;
    m2World* mismatched = NewWorld(&other);
    CHECK(!ReplayJournal(mismatched, journal, journalSize), "capacity mismatch is rejected");

    m2DestroyWorld(original);
    m2DestroyWorld(fresh);
    m2DestroyWorld(mismatched);
    free(journal);
}

// A negative capacity describes no buffer: the journal does not start,
// and the call counts one misuse.
static void TestNegativeCapacityIsRefused(void)
{
    m2WorldDef def = TestDef();
    m2World* world = NewWorld(&def);
    uint8_t byte = 0;
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    CHECK(m2World_StartJournal(world, &byte, -1) == m2_errorInvalid,
          "a capacity of -1 starts no journal");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 1, "and counts one misuse");
    m2DestroyWorld(world);
}

static void TestOverflowIsLoud(void)
{
    m2WorldDef def = TestDef();
    m2World* world = NewWorld(&def);
    int64_t snapshotSize = m2World_GetSnapshotSize(world);
    int64_t capacity = snapshotSize + 64; // room for the header + a crumb
    uint8_t* journal = malloc((size_t)capacity + sizeof(void*));
    CHECK(StartJournal(world, journal, capacity), "tiny journal starts");
    for (int32_t i = 0; i < 8; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        NewBody(world, &bd); // blows past the crumb
    }
    CHECK(StopJournal(world) == 0, "overflowed journal reports zero, never truncates");
    m2DestroyWorld(world);
    free(journal);
}

static void TestRollbackWhileRecording(void)
{
    // The rollback-netcode flow, now first class: record, snapshot,
    // mispredict, RESTORE while the tape is still rolling, continue.
    // The tape carries the restore and the whole timeline replays.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 32;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    int64_t tapeCapacity = 2 * m2World_GetJournalBaseSize(world) + (1 << 16);
    unsigned char* tape = malloc((size_t)tapeCapacity);
    CHECK(StartJournal(world, tape, tapeCapacity), "tape starts");

    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(12.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyDef ballDef = m2DefaultBodyDef();
    ballDef.type = m2_dynamicBody;
    ballDef.position = (m2Pos2){0.0, 3.0};
    m2BodyId ball = NewBody(world, &ballDef);
    m2ShapeDef bs = m2DefaultShapeDef();
    bs.restitution = 0.5f;
    m2Circle circle = {{0.0f, 0.0f}, 0.3f};
    NewCircleShape(world, ball, &bs, &circle);

    for (int32_t i = 0; i < 40; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }
    int64_t snapSize = m2World_GetSnapshotSize(world);
    void* snap = malloc((size_t)snapSize);
    CHECK(Snapshot(world, snap, snapSize) == snapSize, "mid-recording snapshot");

    // The misprediction: a shove that will be rolled back.
    m2Body_ApplyLinearImpulseAtPoint(world, ball, (m2Vec2){2.0f, 1.0f},
                                     m2Body_GetPosition(world, ball));
    for (int32_t i = 0; i < 25; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
    }

    CHECK(Restore(world, snap, snapSize), "rollback with the tape still rolling");
    for (int32_t i = 0; i < 30; ++i)
    {
        Step(world, 1.0f / 60.0f, 4); // the corrected timeline
    }

    int64_t bytes = StopJournal(world);
    CHECK(bytes > snapSize, "the tape survived the rollback and kept recording");
    uint64_t recorded = m2World_Hash(world);

    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, bytes), "the rollback-bearing tape replays");
    CHECK(m2World_Hash(fresh) == recorded, "the whole timeline lands on the same bits");

    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(snap);
    free(tape);
}

static void TestReplayIsAtomic(void)
{
    // Record a session, then offer the tape cut short in the middle of
    // an op to a world with a history of its own: the replay refuses
    // and the world is exactly as it was.
    enum
    {
        CAPACITY = 1 << 20
    };
    uint8_t* tape = malloc(CAPACITY);
    m2WorldDef def = TestDef();
    m2World* recorder = NewWorld(&def);
    int64_t size = 0;
    uint64_t recordedHash = 0;
    RunSession(recorder, tape, CAPACITY, &size, &recordedHash);
    m2DestroyWorld(recorder);

    m2World* target = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){1.0, 7.0};
    m2BodyId own = NewBody(target, &bd);
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeDef sd = m2DefaultShapeDef();
    NewCircleShape(target, own, &sd, &ball);
    Step(target, 1.0f / 60.0f, 4);
    uint64_t before = m2World_Hash(target);

    CHECK(!ReplayJournal(target, tape, size - 3), "a truncated tape is refused");
    CHECK(m2World_Hash(target) == before, "and the world is untouched");
    CHECK(ReplayJournal(target, tape, size), "the whole tape replays");
    CHECK(m2World_Hash(target) == recordedHash, "onto the recorded bits");

    m2DestroyWorld(target);
    free(tape);
}

static bool SameShape(m2ShapeId a, m2ShapeId b)
{
    return a.index1 == b.index1 && a.generation == b.generation;
}

static void TestRefusedReplayKeepsEvents(void)
{
    // A ball lands on a floor; then its shape is destroyed between steps,
    // which queues the contact's end for the next step. A refused replay
    // in between leaves the last step's begin readable and the queued end
    // on its way, as if the replay had never been called.
    enum
    {
        CAPACITY = 1 << 16
    };
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(world, floor, &sd, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.8};
    m2BodyId ball = NewBody(world, &bd);
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    m2ShapeId shape = NewCircleShape(world, ball, &sd, &circle);

    uint8_t* tape = malloc(CAPACITY);
    CHECK(StartJournal(world, tape, CAPACITY), "the journal starts");
    Step(world, 1.0f / 60.0f, 4);
    int64_t size = StopJournal(world);
    CHECK(size > 0, "the tape is recorded");

    int32_t begins = 0;
    for (int32_t i = 0; i < 120 && begins == 0; ++i)
    {
        Step(world, 1.0f / 60.0f, 4);
        begins = m2World_GetContactEvents(world).beginCount;
    }
    CHECK(begins == 1, "the ball lands");
    m2ContactBeginEvent begin = m2World_GetContactEvents(world).beginEvents[0];
    uint64_t before = m2World_Hash(world);

    // A tape whose embedded snapshot claims a negative size is refused
    // after the replay has taken its back-out.
    uint8_t* edited = malloc((size_t)size);
    memcpy(edited, tape, (size_t)size);
    int64_t negative = -1;
    memcpy(edited + 16, &negative, sizeof(negative));
    CHECK(m2World_ReplayJournal(world, edited, size) == m2_errorFormat,
          "a negative snapshot size is refused");
    m2ContactEvents events = m2World_GetContactEvents(world);
    CHECK(events.beginCount == 1 && SameShape(events.beginEvents[0].shapeIdA, begin.shapeIdA) &&
              SameShape(events.beginEvents[0].shapeIdB, begin.shapeIdB) &&
              events.beginEvents[0].step == begin.step,
          "the last step's begin is still readable");
    CHECK(m2World_Hash(world) == before, "the world is untouched");

    CHECK(DestroyShape(world, shape) == m2_success, "the ball's shape is destroyed");
    uint64_t destroyed = m2World_Hash(world);
    CHECK(m2World_ReplayJournal(world, tape, size - 1) == m2_errorFormat,
          "a truncated tape is refused");
    CHECK(m2World_Hash(world) == destroyed, "and the destroy stands");
    Step(world, 1.0f / 60.0f, 4);
    events = m2World_GetContactEvents(world);
    CHECK(events.endCount == 1 && SameShape(events.endEvents[0].shapeIdA, begin.shapeIdA) &&
              SameShape(events.endEvents[0].shapeIdB, begin.shapeIdB),
          "the queued end reaches the next step");

    m2DestroyWorld(world);
    free(edited);
    free(tape);
}

static void TestRefusedReplayKeepsCounters(void)
{
    // The session's tape steps a scene of its own before its cut-short
    // last record fails it. The counts those steps set do not outlive the
    // refusal: the world reports what it reported before the call.
    enum
    {
        CAPACITY = 1 << 20
    };
    uint8_t* tape = malloc(CAPACITY);
    m2WorldDef def = TestDef();
    m2World* recorder = NewWorld(&def);
    int64_t size = 0;
    uint64_t recordedHash = 0;
    RunSession(recorder, tape, CAPACITY, &size, &recordedHash);
    m2Counters recorded = m2World_GetCounters(recorder);
    m2DestroyWorld(recorder);

    m2World* target = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    m2BodyId floor = NewBody(target, &fd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(target, floor, &sd, &slab);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.8};
    m2BodyId ball = NewBody(target, &bd);
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    NewCircleShape(target, ball, &sd, &circle);
    for (int32_t i = 0; i < 30; ++i)
    {
        Step(target, 1.0f / 60.0f, 4);
    }
    m2Counters before = m2World_GetCounters(target);
    CHECK(before.constraintCount == 1 && recorded.constraintCount != before.constraintCount,
          "the tape's last step solved other constraints than the target's");

    CHECK(m2World_ReplayJournal(target, tape, size - 3) == m2_errorFormat,
          "a tape cut short in its last record is refused");
    m2Counters after = m2World_GetCounters(target);
    CHECK(memcmp(&before, &after, sizeof(m2Counters)) == 0,
          "every count is as it was before the call");

    m2DestroyWorld(target);
    free(tape);
}

// Replays tape with the 4-byte word at offset (from the tape's start)
// replaced by value, after checking it held expected: the call it feeds
// refuses, so the tape is refused, changes nothing and counts nothing,
// even where the call returns void.
static void ExpectWordRefused(m2World* world, const uint8_t* tape, int64_t bytes, int64_t offset,
                              const void* expected, const void* value, const char* what)
{
    uint8_t* edited = (uint8_t*)malloc((size_t)bytes);
    CHECK(edited != NULL, "a copy of the tape");
    if (edited == NULL)
    {
        return;
    }
    memcpy(edited, tape, (size_t)bytes);
    CHECK(memcmp(edited + offset, expected, 4) == 0, "the record holds the value at its offset");
    memcpy(edited + offset, value, 4);
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    CHECK(m2World_ReplayJournal(world, edited, bytes) == m2_errorFormat, what);
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuseCount == misuse,
          "a refused tape changes nothing and counts nothing");
    free(edited);
}

static void ExpectFloatRefused(m2World* world, const uint8_t* tape, int64_t bytes, int64_t offset,
                               float expected, float value, const char* what)
{
    ExpectWordRefused(world, tape, bytes, offset, &expected, &value, what);
}

// A void writer records itself only after its checks pass, so a tape
// whose value that writer refuses is not one a recording wrote: a
// velocity set to NaN, a step of no time, a particle id that names no live
// particle. None replays as a skipped op; the whole tape is refused.
static void TestRefusedVoidOpsFailTheTape(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.particleCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bodyDef);
    int64_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);

    // The record: an op byte, the body's id, then the velocity.
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    int64_t recordStart = m2World_GetJournalBaseSize(world);
    m2Body_SetLinearVelocity(world, body, (m2Vec2){1.5f, 0.0f});
    int64_t bytes = StopJournal(world);
    CHECK(ReplayJournal(world, tape, bytes), "the recorded tape replays");
    ExpectFloatRefused(world, tape, bytes, recordStart + 1 + (int64_t)sizeof(m2BodyId), 1.5f, NAN,
                       "a velocity set to NaN on a tape is refused");

    // The record: an op byte, then the step's dt.
    CHECK(StartJournal(world, tape, capacity), "the journal starts again");
    recordStart = m2World_GetJournalBaseSize(world);
    Step(world, 0.25f, 1);
    bytes = StopJournal(world);
    CHECK(ReplayJournal(world, tape, bytes), "the recorded step replays");
    ExpectFloatRefused(world, tape, bytes, recordStart + 1, 0.25f, 0.0f,
                       "a step of no time on a tape is refused");

    // The record: an op byte, the particle's id (slot, then generation),
    // then the velocity. A dead particle's setters count nothing, but a
    // recording never writes one, so a tape that names one is refused.
    m2ParticleId particle = EmitParticle(world, (m2Pos2){0.0, 5.0}, (m2Vec2){0.0f, 0.0f}, 0);
    CHECK(StartJournal(world, tape, capacity), "the journal starts a third time");
    recordStart = m2World_GetJournalBaseSize(world);
    m2Particle_SetVelocity(world, particle, (m2Vec2){0.5f, 0.0f});
    bytes = StopJournal(world);
    CHECK(ReplayJournal(world, tape, bytes), "the recorded particle velocity replays");
    uint32_t generation = particle.generation;
    uint32_t stale = generation + 1u;
    ExpectWordRefused(world, tape, bytes, recordStart + 1 + (int64_t)sizeof(uint32_t), &generation,
                      &stale, "a particle velocity for no live particle on a tape is refused");
    free(tape);
    m2DestroyWorld(world);
}

static void TestRestoreRefusesBadCounters(void)
{
    // A snapshot whose header counters point past the capacities is
    // refused before anything is overwritten.
    m2WorldDef def = TestDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bd);
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeDef sd = m2DefaultShapeDef();
    NewCircleShape(world, body, &sd, &ball);
    int64_t size = m2World_GetSnapshotSize(world);
    uint8_t* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot taken");
    Step(world, 1.0f / 60.0f, 4);
    uint64_t before = m2World_Hash(world);

    // maxBodyIndex follows the header and the step count, gravity, wind
    // velocity and wind drag (world_state.c); the header's size is the
    // prefix's last word.
    uint32_t headerBytes = 0;
    memcpy(&headerBytes, snap + 12, sizeof(headerBytes));
    int32_t hostile = def.bodyCapacity + 1000;
    memcpy(snap + headerBytes + 28, &hostile, sizeof(hostile));
    CHECK(!Restore(world, snap, size), "an out-of-range body count is refused");
    CHECK(m2World_Hash(world) == before, "and the world is untouched");

    m2DestroyWorld(world);
    free(snap);
}

// Replays a tape whose one record, made at the start of a recording, has
// the byte at payloadOffset replaced: it must be refused, change nothing
// and count nothing.
static void ExpectByteRefused(m2World* world, const uint8_t* tape, int64_t bytes,
                              int64_t recordStart, size_t payloadOffset, const char* what)
{
    uint8_t* edited = (uint8_t*)malloc((size_t)bytes);
    CHECK(edited != NULL, "a copy of the tape");
    if (edited == NULL)
    {
        return;
    }
    memcpy(edited, tape, (size_t)bytes);
    edited[recordStart + 1 + (int64_t)payloadOffset] = 29;
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    CHECK(m2World_ReplayJournal(world, edited, bytes) == m2_errorFormat, what);
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuseCount == misuse,
          "a refused tape changes nothing and counts nothing");
    free(edited);
}

// Replays a tape whose one record, made at the start of a recording, has
// the int32 at payloadOffset replaced by value; the record held expected
// there, which checks the offset.
static void ExpectI32Refused(m2World* world, const uint8_t* tape, int64_t bytes,
                             int64_t recordStart, size_t payloadOffset, int32_t expected,
                             int32_t value, const char* what)
{
    uint8_t* edited = (uint8_t*)malloc((size_t)bytes);
    CHECK(edited != NULL, "a copy of the tape");
    if (edited == NULL)
    {
        return;
    }
    memcpy(edited, tape, (size_t)bytes);
    int32_t held = 0;
    memcpy(&held, edited + recordStart + 1 + (int64_t)payloadOffset, sizeof(held));
    CHECK(held == expected, "the record holds the field at its offset");
    memcpy(edited + recordStart + 1 + (int64_t)payloadOffset, &value, sizeof(value));
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuseCount;
    CHECK(m2World_ReplayJournal(world, edited, bytes) == m2_errorFormat, what);
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuseCount == misuse,
          "a refused tape changes nothing and counts nothing");
    free(edited);
}

// A body def's type and a particle's flags on a tape are refused below
// zero and past the defined bits, as their creates refuse them.
static void TestTapeKindsAreRefused(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.particleCapacity = 4;
    m2World* world = NewWorld(&def);
    int64_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    int64_t recordStart = StopJournal(world);

    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    CHECK(StartJournal(world, tape, capacity), "the journal starts again");
    NewBody(world, &bodyDef);
    int64_t bytes = StopJournal(world);
    const int32_t negatives[2] = {-1, INT32_MIN};
    for (int32_t i = 0; i < 2; ++i)
    {
        ExpectI32Refused(world, tape, bytes, recordStart, offsetof(m2BodyDef, type), m2_dynamicBody,
                         negatives[i], "a negative body type on a tape is refused");
    }

    // The particle record holds the def's position, velocity and flags, in
    // that order.
    CHECK(StartJournal(world, tape, capacity), "the journal starts a third time");
    recordStart = m2World_GetJournalBaseSize(world);
    m2ParticleDef particleDef = m2DefaultParticleDef();
    particleDef.flags = m2_tensileParticle;
    m2ParticleId particle = m2_nullParticleId;
    CHECK(m2CreateParticle(world, &particleDef, &particle) == m2_success, "the particle creates");
    bytes = StopJournal(world);
    size_t flagsAt = sizeof(m2Pos2) + sizeof(m2Vec2);
    for (int32_t i = 0; i < 2; ++i)
    {
        ExpectI32Refused(world, tape, bytes, recordStart, flagsAt, (int32_t)m2_tensileParticle,
                         negatives[i],
                         "particle flags past the defined bits on a tape are refused");
    }
    free(tape);
    m2DestroyWorld(world);
}

// A record's bool flags hold 0 or 1; any other byte is refused before the
// payload is read, since reading it as a bool is undefined. fuzz_journal
// found a body def whose enable byte was 29.
static void TestTapeBoolsAreZeroOrOne(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    m2World* world = NewWorld(&def);
    int64_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    int64_t recordStart = StopJournal(world); // the header and snapshot alone

    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    CHECK(StartJournal(world, tape, capacity), "the journal starts again");
    m2BodyId body = NewBody(world, &bodyDef);
    int64_t bytes = StopJournal(world);
    const size_t bodyBools[] = {offsetof(m2BodyDef, enableSleep), offsetof(m2BodyDef, enable),
                                offsetof(m2BodyDef, enableBullet),
                                offsetof(m2BodyDef, motionLocks.angularZ)};
    for (size_t i = 0; i < sizeof(bodyBools) / sizeof(bodyBools[0]); ++i)
    {
        ExpectByteRefused(world, tape, bytes, recordStart, bodyBools[i],
                          "a body def's bool of 29 is refused");
    }

    bodyDef.position = (m2Pos2){1.0, 0.0};
    m2BodyId other = NewBody(world, &bodyDef);
    m2RevoluteJointDef pin = m2DefaultRevoluteJointDef();
    pin.bodyIdA = body;
    pin.bodyIdB = other;
    CHECK(StartJournal(world, tape, capacity), "the journal starts a third time");
    recordStart = m2World_GetJournalBaseSize(world);
    NewRevoluteJoint(world, &pin);
    bytes = StopJournal(world);
    ExpectByteRefused(world, tape, bytes, recordStart, offsetof(m2RevoluteJointDef, enableMotor),
                      "a joint def's bool of 29 is refused");
    free(tape);
    m2DestroyWorld(world);
}

// Each Filled*Def sets a def's fields to from's over bytes that all start
// as fill, so only the bytes between the fields keep the fill.
static void FilledBodyDef(m2BodyDef* to, const m2BodyDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->type = from->type;
    to->position = from->position;
    to->rotation = from->rotation;
    to->linearVelocity = from->linearVelocity;
    to->angularVelocity = from->angularVelocity;
    to->gravityScale = from->gravityScale;
    to->linearDamping = from->linearDamping;
    to->angularDamping = from->angularDamping;
    to->motionLocks = from->motionLocks;
    to->enableSleep = from->enableSleep;
    to->enable = from->enable;
    to->dominance = from->dominance;
    to->enableBullet = from->enableBullet;
    to->userData = from->userData;
    to->internalValue = from->internalValue;
}

static void FilledShapeDef(m2ShapeDef* to, const m2ShapeDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->density = from->density;
    to->friction = from->friction;
    to->tangentSpeed = from->tangentSpeed;
    to->restitution = from->restitution;
    to->categoryBits = from->categoryBits;
    to->maskBits = from->maskBits;
    to->groupIndex = from->groupIndex;
    to->isSensor = from->isSensor;
    to->userData = from->userData;
    to->internalValue = from->internalValue;
}

static void FilledChainDef(m2ChainDef* to, const m2ChainDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->points = from->points;
    to->count = from->count;
    to->isLoop = from->isLoop;
    to->friction = from->friction;
    to->restitution = from->restitution;
    to->categoryBits = from->categoryBits;
    to->maskBits = from->maskBits;
    to->groupIndex = from->groupIndex;
    to->userData = from->userData;
    to->internalValue = from->internalValue;
}

static void FilledWaterVolumeDef(m2WaterVolumeDef* to, const m2WaterVolumeDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bounds = from->bounds;
    to->surface = from->surface;
    to->density = from->density;
    to->linearDrag = from->linearDrag;
    to->angularDrag = from->angularDrag;
    to->flow = from->flow;
    to->userData = from->userData;
    to->internalValue = from->internalValue;
}

static void FilledExplosionDef(m2ExplosionDef* to, const m2ExplosionDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->position = from->position;
    to->radius = from->radius;
    to->falloff = from->falloff;
    to->impulse = from->impulse;
    to->maskBits = from->maskBits;
    to->internalValue = from->internalValue;
}

static void FilledDistanceJointDef(m2DistanceJointDef* to, const m2DistanceJointDef* from,
                                   uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->length = from->length;
    to->minLength = from->minLength;
    to->maxLength = from->maxLength;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->enableSpring = from->enableSpring;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledRevoluteJointDef(m2RevoluteJointDef* to, const m2RevoluteJointDef* from,
                                   uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->constraintHertz = from->constraintHertz;
    to->constraintDampingRatio = from->constraintDampingRatio;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->enableSpring = from->enableSpring;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorEffort = from->maxMotorEffort;
    to->enableLimit = from->enableLimit;
    to->lowerAngle = from->lowerAngle;
    to->upperAngle = from->upperAngle;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledPrismaticJointDef(m2PrismaticJointDef* to, const m2PrismaticJointDef* from,
                                    uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->localAxisA = from->localAxisA;
    to->constraintHertz = from->constraintHertz;
    to->constraintDampingRatio = from->constraintDampingRatio;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorEffort = from->maxMotorEffort;
    to->enableLimit = from->enableLimit;
    to->lowerTranslation = from->lowerTranslation;
    to->upperTranslation = from->upperTranslation;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledWeldJointDef(m2WeldJointDef* to, const m2WeldJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->linearSpringHertz = from->linearSpringHertz;
    to->linearSpringDampingRatio = from->linearSpringDampingRatio;
    to->angularSpringHertz = from->angularSpringHertz;
    to->angularSpringDampingRatio = from->angularSpringDampingRatio;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledWheelJointDef(m2WheelJointDef* to, const m2WheelJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->localAxisA = from->localAxisA;
    to->enableSpring = from->enableSpring;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorEffort = from->maxMotorEffort;
    to->enableLimit = from->enableLimit;
    to->lowerTranslation = from->lowerTranslation;
    to->upperTranslation = from->upperTranslation;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledFilterJointDef(m2FilterJointDef* to, const m2FilterJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->userData = from->userData;
    to->internalValue = from->internalValue;
}

static void FilledMotorJointDef(m2MotorJointDef* to, const m2MotorJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->linearOffset = from->linearOffset;
    to->angularOffset = from->angularOffset;
    to->maxForce = from->maxForce;
    to->maxTorque = from->maxTorque;
    to->correctionFactor = from->correctionFactor;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledMouseJointDef(m2MouseJointDef* to, const m2MouseJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->target = from->target;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->maxForce = from->maxForce;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledGearJointDef(m2GearJointDef* to, const m2GearJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->ratio = from->ratio;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledPulleyJointDef(m2PulleyJointDef* to, const m2PulleyJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->groundAnchorA = from->groundAnchorA;
    to->groundAnchorB = from->groundAnchorB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->ratio = from->ratio;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

static void FilledRatchetJointDef(m2RatchetJointDef* to, const m2RatchetJointDef* from,
                                  uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->toothAngle = from->toothAngle;
    to->phase = from->phase;
    to->userData = from->userData;
    to->enableConnectedCollision = from->enableConnectedCollision;
    to->internalValue = from->internalValue;
}

// Creates a joint of kind between bodies a and b from its default def,
// with the def's bytes starting as fill.
#define FILLED_JOINT(kind, a, b)                                                                   \
    do                                                                                             \
    {                                                                                              \
        m2##kind##JointDef values = m2Default##kind##JointDef();                                   \
        values.bodyIdA = (a);                                                                      \
        values.bodyIdB = (b);                                                                      \
        m2##kind##JointDef def;                                                                    \
        Filled##kind##JointDef(&def, &values, fill);                                               \
        CHECK(m2Joint_IsValid(world, New##kind##Joint(world, &def)), #kind " joint creates");      \
    } while (0)

static m2World* NewDefWorld(void)
{
    m2WorldDef worldDef = m2DefaultWorldDef();
    worldDef.bodyCapacity = 8;
    worldDef.shapeCapacity = 16;
    worldDef.jointCapacity = 16;
    worldDef.waterVolumeCapacity = 2;
    worldDef.particleCapacity = 64;
    return NewWorld(&worldDef);
}

// A polygon with from's vertices, count and radius over bytes that all
// start as fill: its normals and unused slots, which the library computes
// and zeroes itself, keep the fill.
static m2Polygon FilledPolygon(const m2Polygon* from, uint8_t fill)
{
    m2Polygon to;
    memset(&to, fill, sizeof(to));
    for (int32_t i = 0; i < from->count; ++i)
    {
        to.vertices[i] = from->vertices[i];
    }
    to.count = from->count;
    to.radius = from->radius;
    return to;
}

// Records a create from every def the journal records into a fresh
// world's tape, each def's bytes starting as fill, and gives the world's
// hash at the end.
static uint8_t* TapeOfFilledDefs(uint8_t fill, int64_t* bytesOut, uint64_t* hashOut)
{
    m2World* world = NewDefWorld();
    m2BodyDef groundValues = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &groundValues);
    int64_t capacity = m2World_GetJournalBaseSize(world) + 16384;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");

    m2BodyDef bodyValues = m2DefaultBodyDef();
    bodyValues.type = m2_dynamicBody;
    bodyValues.motionLocks.angularZ = true;
    bodyValues.dominance = -3;
    bodyValues.userData = 77;
    m2BodyDef bodyDef;
    m2BodyId bodies[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        bodyValues.position = (m2Pos2){(double)i, 2.0};
        FilledBodyDef(&bodyDef, &bodyValues, fill);
        bodies[i] = NewBody(world, &bodyDef);
        CHECK(m2Body_IsValid(world, bodies[i]), "a body creates");
    }
    m2ShapeDef shapeValues = m2DefaultShapeDef();
    shapeValues.isSensor = true;
    m2ShapeDef shapeDef;
    FilledShapeDef(&shapeDef, &shapeValues, fill);
    m2Circle circle = {{0.0f, 0.0f}, 0.25f};
    CHECK(m2Shape_IsValid(world, NewCircleShape(world, bodies[0], &shapeDef, &circle)),
          "a shape creates");
    m2Vec2 points[4] = {{-3.0f, 0.0f}, {-1.0f, 0.0f}, {1.0f, 0.0f}, {3.0f, 0.0f}};
    m2ChainDef chainValues = m2DefaultChainDef();
    chainValues.points = points;
    chainValues.count = 4;
    m2ChainDef chainDef;
    FilledChainDef(&chainDef, &chainValues, fill);
    CHECK(m2Chain_IsValid(world, NewChain(world, ground, &chainDef)), "a chain creates");
    m2WaterVolumeDef volumeValues = m2DefaultWaterVolumeDef();
    volumeValues.bounds = (m2WorldAabb){{-1.0, -1.0}, {1.0, 1.0}};
    m2WaterVolumeDef volumeDef;
    FilledWaterVolumeDef(&volumeDef, &volumeValues, fill);
    CHECK(m2WaterVolume_IsValid(world, NewWaterVolume(world, &volumeDef)),
          "a water volume creates");
    m2ExplosionDef explosionValues = m2DefaultExplosionDef();
    m2ExplosionDef explosionDef;
    FilledExplosionDef(&explosionDef, &explosionValues, fill);
    m2World_Explode(world, &explosionDef);

    FILLED_JOINT(Distance, bodies[0], bodies[1]);
    FILLED_JOINT(Revolute, bodies[0], bodies[1]);
    FILLED_JOINT(Prismatic, bodies[0], bodies[1]);
    FILLED_JOINT(Weld, bodies[0], bodies[1]);
    FILLED_JOINT(Wheel, bodies[0], bodies[1]);
    FILLED_JOINT(Filter, bodies[0], bodies[1]);
    FILLED_JOINT(Motor, bodies[0], bodies[1]);
    FILLED_JOINT(Mouse, ground, bodies[1]);
    FILLED_JOINT(Gear, bodies[0], bodies[1]);
    FILLED_JOINT(Pulley, bodies[0], bodies[1]);
    FILLED_JOINT(Ratchet, bodies[0], bodies[1]);

    // A fill and a shatter, from polygons whose normals and unused slots
    // hold the fill.
    m2Polygon box = m2MakeBox(0.2f, 0.2f);
    m2Polygon region = FilledPolygon(&box, fill);
    m2ParticleDef particleDef = m2DefaultParticleDef();
    particleDef.position = (m2Pos2){0.0, 4.0};
    int32_t made = 0;
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &region, &made) == m2_success && made > 0,
          "a fill records");
    m2BodyDef crateDef = m2DefaultBodyDef();
    crateDef.type = m2_dynamicBody;
    crateDef.position = (m2Pos2){4.0, 2.0};
    m2BodyId crate = NewBody(world, &crateDef);
    m2ShapeDef crateShape = m2DefaultShapeDef();
    NewPolygonShape(world, crate, &crateShape, &box);
    m2Vec2 halves[2][3] = {{{-0.2f, -0.2f}, {0.2f, -0.2f}, {0.2f, 0.2f}},
                           {{-0.2f, -0.2f}, {0.2f, 0.2f}, {-0.2f, 0.2f}}};
    m2Polygon pieces[2];
    for (int32_t i = 0; i < 2; ++i)
    {
        m2Polygon half = MakePolygon(halves[i], 3, 0.0f);
        pieces[i] = FilledPolygon(&half, fill);
    }
    CHECK(ShatterBody(world, crate, pieces, 2, NULL, 0) == 2, "a shatter records");
    CHECK(m2World_GetCounters(world).misuseCount == 0,
          "every call passes its checks, the explosion too");
    *bytesOut = StopJournal(world);
    *hashOut = m2World_Hash(world);
    m2DestroyWorld(world);
    return tape;
}

// The bytes between a def's fields are not part of it, nor are a
// polygon's normals and unused slots: two runs of the same calls record
// the same tape whatever those bytes hold, and the tape replays onto the
// world that recorded it.
static void TestDefPaddingStaysOffTheTape(void)
{
    int64_t bytesA = 0;
    int64_t bytesB = 0;
    uint64_t hashA = 0;
    uint64_t hashB = 0;
    uint8_t* tapeA = TapeOfFilledDefs(0xA5, &bytesA, &hashA);
    uint8_t* tapeB = TapeOfFilledDefs(0x5A, &bytesB, &hashB);
    CHECK(bytesA > 0 && bytesA == bytesB && memcmp(tapeA, tapeB, (size_t)bytesA) == 0,
          "no padding or unused polygon slot reaches the tape");
    m2World* replayed = NewDefWorld();
    CHECK(ReplayJournal(replayed, tapeA, bytesA) && m2World_Hash(replayed) == hashA &&
              hashA == hashB,
          "and the tape replays onto the recorded world");
    m2DestroyWorld(replayed);
    free(tapeA);
    free(tapeB);
}

// Host code the world runs, here a debug draw callback, stops the journal
// the world is recording. The stop is refused as busy and leaves bytesOut,
// which the host may be filling, and the journal keeps recording.
typedef struct JournalIntruder
{
    m2World* world;
    int32_t calls;
    m2Result stopped;
    int64_t bytes;
} JournalIntruder;

static void StopJournalWhileDrawing(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context)
{
    (void)p1;
    (void)p2;
    (void)color;
    JournalIntruder* in = (JournalIntruder*)context;
    if (in->calls++ == 0)
    {
        in->stopped = m2World_StopJournal(in->world, &in->bytes);
    }
}

static void TestBusyStopKeepsBytes(void)
{
    m2WorldDef def = TestDef();
    JournalIntruder in = {NewWorld(&def), 0, m2_success, 7};
    m2BodyDef bd = m2DefaultBodyDef();
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(1.0f, 1.0f);
    NewPolygonShape(in.world, NewBody(in.world, &bd), &sd, &box);
    int64_t capacity = m2World_GetSnapshotSize(in.world) + 4096;
    uint8_t* tape = malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(in.world, tape, capacity), "the journal starts");

    uint64_t misuse = m2World_GetCounters(in.world).misuseCount;
    m2DebugDraw draw = {0};
    draw.drawSegment = StopJournalWhileDrawing;
    draw.drawAabbs = true;
    draw.context = &in;
    m2World_Draw(in.world, &draw);
    CHECK(in.calls > 0 && in.stopped == m2_errorBusy, "a stop inside a draw callback is busy");
    CHECK(in.bytes == 7, "and leaves bytesOut as it was");
    CHECK(m2World_GetCounters(in.world).misuseCount == misuse + 1, "the refusal counts one misuse");
    CHECK(StopJournal(in.world) > 0, "the journal was still recording");
    m2DestroyWorld(in.world);
    free(tape);
}

// Each journal call refuses out of turn: a start below the base size or
// while recording, a stop with nowhere to write its size or with nothing
// recording, and a replay of no data or while recording. A refusal that
// is not m2_errorCapacity counts one misuse, and none of them changes the
// world or ends a recording in progress.
static void TestJournalCallsRefuseOutOfTurn(void)
{
    m2WorldDef def = TestDef();
    m2World* world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    m2BodyId body = NewBody(world, &bd);
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeDef sd = m2DefaultShapeDef();
    NewCircleShape(world, body, &sd, &ball);
    int64_t base = m2World_GetJournalBaseSize(world);
    int64_t capacity = base + 4096;
    uint8_t* tape = malloc((size_t)capacity);
    uint8_t* other = malloc((size_t)capacity);
    CHECK(tape != NULL && other != NULL, "the buffers exist");
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuseCount;

    CHECK(m2World_StartJournal(world, tape, base - 1) == m2_errorCapacity,
          "a buffer below the base size is m2_errorCapacity");
    CHECK(m2World_GetCounters(world).misuseCount == misuse, "which counts no misuse");
    int64_t bytes = 7;
    CHECK(m2World_StopJournal(world, &bytes) == m2_errorState && bytes == 0,
          "so nothing is recording: a stop is m2_errorState and gives 0");
    CHECK(m2World_StopJournal(world, NULL) == m2_errorInvalid,
          "a stop with nowhere to write its size is m2_errorInvalid");
    CHECK(m2World_ReplayJournal(world, NULL, 64) == m2_errorInvalid,
          "a replay of no data is m2_errorInvalid");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 3, "each counting one misuse");

    CHECK(StartJournal(world, tape, capacity), "the journal starts");
    m2Body_SetLinearVelocity(world, body, (m2Vec2){1.0f, 0.0f});
    Step(world, 1.0f / 60.0f, 4);
    misuse = m2World_GetCounters(world).misuseCount;
    CHECK(m2World_StartJournal(world, other, capacity) == m2_errorState,
          "a second start while recording is m2_errorState");
    CHECK(m2World_ReplayJournal(world, tape, capacity) == m2_errorState,
          "and so is a replay");
    CHECK(m2World_StopJournal(world, NULL) == m2_errorInvalid,
          "a stop with nowhere to write its size is refused while recording too");
    CHECK(m2World_GetCounters(world).misuseCount == misuse + 3, "each counting one misuse");
    uint64_t recorded = m2World_Hash(world);
    bytes = StopJournal(world);
    CHECK(bytes > 0, "the first recording went on through every refusal");

    m2World* replayed = NewWorld(&def);
    CHECK(ReplayJournal(replayed, tape, bytes) && m2World_Hash(replayed) == recorded,
          "and its tape replays onto the recorded world");
    CHECK(hash != recorded, "the session changed the world");
    m2DestroyWorld(replayed);
    m2DestroyWorld(world);
    free(other);
    free(tape);
}

int main(void)
{
    TestTapeBoolsAreZeroOrOne();
    TestTapeKindsAreRefused();
    TestDefPaddingStaysOffTheTape();
    TestReplayIsAtomic();
    TestRefusedReplayKeepsEvents();
    TestRefusedReplayKeepsCounters();
    TestRefusedVoidOpsFailTheTape();
    TestRestoreRefusesBadCounters();
    TestRollbackWhileRecording();
    TestRecordReplay();
    TestOverflowIsLoud();
    TestNegativeCapacityIsRefused();
    TestBusyStopKeepsBytes();
    TestJournalCallsRefuseOutOfTurn();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
