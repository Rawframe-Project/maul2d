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

#include <stddef.h>
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
    def.fluidVolumeCapacity = 4; // and the buoyancy ops
    return def;
}

// A session with every op type: floor, rain of shapes, a pendulum, a
// shove, a destroy, steps in two dt flavors (markers carry dt unchanged).
static void RunSession(m2World* world, uint8_t* journal, int32_t capacity, int32_t* outSize,
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
    press.maxMotorForce = 20.0f;
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
    ride.maxMotorTorque = 5.0f;
    NewWheelJoint(world, &ride);

    for (int32_t i = 0; i < 45; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
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
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    DestroyBody(world, victim);
    DestroyJoint(world, rod);
    DestroyChain(world, chainId);                                           // op 24
    m2Body_ApplyForce(world, bob, (m2Vec2){3.0f, 1.0f});                    // op 28
    m2Body_ApplyTorque(world, bob, 0.8f);                                   // op 29
    m2Body_SetLinearDamping(world, bob, 0.2f);                              // op 25
    m2Body_SetMotionLocks(world, bob, (m2MotionLocks){false, false, true}); // op 25
    m2World_EnableSleeping(world, false);                                   // op 26
    m2World_EnableSleeping(world, true);
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
    m2Body_Disable(world, ram);                                 // op 35
    m2Body_Enable(world, ram);                                  // op 36
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
    m2Body_SetBullet(world, bob, true);            // op 45
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
    pawlDef.ratchet = -0.3f;
    NewRatchetJoint(world, &pawlDef); // op 56
    m2RevoluteJointDef swingDef = m2DefaultRevoluteJointDef();
    swingDef.bodyIdA = anchor;
    swingDef.bodyIdB = cylinder;
    swingDef.springHertz = 3.0f; // the def echo carries the spring pair
    swingDef.springDampingRatio = 0.5f;
    m2JointId swing = NewRevoluteJoint(world, &swingDef);
    m2Joint_SetAngularSpringHertz(world, swing, 1.5f);         // param 10 on a revolute
    m2Joint_SetAngularSpringDampingRatio(world, swing, 0.25f); // param 11 on a revolute
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
    m2FluidVolumeDef poolDef = m2DefaultFluidVolumeDef();
    poolDef.regionLower = (m2Pos2){-5.0, -5.0};
    poolDef.regionUpper = (m2Pos2){5.0, 5.0};
    poolDef.surface = -1.0;
    m2FluidVolumeId pool = NewFluidVolume(world, &poolDef); // op 61
    m2FluidVolume_SetSurface(world, pool, 0.5);             // op 63
    m2BodyDef drd = m2DefaultBodyDef();
    drd.position = (m2Pos2){-4.0, 6.0};
    m2BodyId drHook = NewBody(world, &drd);
    m2DistanceJointDef drj = m2DefaultDistanceJointDef();
    drj.bodyIdA = drHook;
    drj.bodyIdB = bob;
    drj.hertz = 1.0f;
    m2JointId leash = NewDistanceJoint(world, &drj);
    m2DistanceJoint_SetLength(world, leash, 1.8f);            // param 12
    m2DistanceJoint_SetLengthRange(world, leash, 0.4f, 2.2f); // params 13+14
    m2Joint_SetSpringHertz(world, leash, 6.0f);               // param 8
    m2Joint_SetSpringDampingRatio(world, leash, 0.5f);        // param 9
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 120.0f, 2); // second dt flavor
    }

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
    int32_t journalSize = 0;
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

static void TestOverflowIsLoud(void)
{
    m2WorldDef def = TestDef();
    m2World* world = NewWorld(&def);
    int32_t snapshotSize = m2World_SnapshotSize(world);
    int32_t capacity = snapshotSize + 64; // room for the header + a crumb
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
    int32_t tapeCapacity = 2 * m2World_GetJournalBaseSize(world) + (1 << 16);
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
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t snapSize = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)snapSize);
    CHECK(Snapshot(world, snap, snapSize) == snapSize, "mid-recording snapshot");

    // The misprediction: a shove that will be rolled back.
    m2Body_ApplyLinearImpulseAtPoint(world, ball, (m2Vec2){2.0f, 1.0f},
                                     m2Body_GetPosition(world, ball));
    for (int32_t i = 0; i < 25; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    CHECK(Restore(world, snap, snapSize), "rollback with the tape still rolling");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // the corrected timeline
    }

    int32_t bytes = StopJournal(world);
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
    int32_t size = 0;
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
    m2World_Step(target, 1.0f / 60.0f, 4);
    uint64_t before = m2World_Hash(target);

    CHECK(!ReplayJournal(target, tape, size - 3), "a truncated tape is refused");
    CHECK(m2World_Hash(target) == before, "and the world is untouched");
    CHECK(ReplayJournal(target, tape, size), "the whole tape replays");
    CHECK(m2World_Hash(target) == recordedHash, "onto the recorded bits");

    m2DestroyWorld(target);
    free(tape);
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
    int32_t size = m2World_SnapshotSize(world);
    uint8_t* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot taken");
    m2World_Step(world, 1.0f / 60.0f, 4);
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
static void ExpectByteRefused(m2World* world, const uint8_t* tape, int32_t bytes,
                              int32_t recordStart, size_t payloadOffset, const char* what)
{
    uint8_t* edited = (uint8_t*)malloc((size_t)bytes);
    CHECK(edited != NULL, "a copy of the tape");
    if (edited == NULL)
    {
        return;
    }
    memcpy(edited, tape, (size_t)bytes);
    edited[recordStart + 1 + (int32_t)payloadOffset] = 29;
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuse;
    CHECK(m2World_ReplayJournal(world, edited, bytes) == m2_errorInvalid, what);
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuse == misuse,
          "a refused tape changes nothing and counts nothing");
    free(edited);
}

// A record's bool flags hold 0 or 1; any other byte is refused before the
// payload is read, since reading it as a bool is undefined. fuzz_journal
// found a body def whose isEnabled byte was 29.
static void TestTapeBoolsAreZeroOrOne(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    m2World* world = NewWorld(&def);
    int32_t capacity = m2World_GetJournalBaseSize(world) + 4096;
    uint8_t* tape = (uint8_t*)malloc((size_t)capacity);
    CHECK(tape != NULL && StartJournal(world, tape, capacity), "the journal starts");
    int32_t recordStart = StopJournal(world); // the header and snapshot alone

    m2BodyDef bodyDef = m2DefaultBodyDef();
    bodyDef.type = m2_dynamicBody;
    CHECK(StartJournal(world, tape, capacity), "the journal starts again");
    m2BodyId body = NewBody(world, &bodyDef);
    int32_t bytes = StopJournal(world);
    const size_t bodyBools[] = {offsetof(m2BodyDef, enableSleep), offsetof(m2BodyDef, isEnabled),
                                offsetof(m2BodyDef, isBullet),
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
    to->isEnabled = from->isEnabled;
    to->dominance = from->dominance;
    to->isBullet = from->isBullet;
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

static void FilledFluidVolumeDef(m2FluidVolumeDef* to, const m2FluidVolumeDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->regionLower = from->regionLower;
    to->regionUpper = from->regionUpper;
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
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->enableSpring = from->enableSpring;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->springHertz = from->springHertz;
    to->springDampingRatio = from->springDampingRatio;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorTorque = from->maxMotorTorque;
    to->enableLimit = from->enableLimit;
    to->lowerAngle = from->lowerAngle;
    to->upperAngle = from->upperAngle;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorForce = from->maxMotorForce;
    to->enableLimit = from->enableLimit;
    to->lowerTranslation = from->lowerTranslation;
    to->upperTranslation = from->upperTranslation;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
    to->internalValue = from->internalValue;
}

static void FilledWeldJointDef(m2WeldJointDef* to, const m2WeldJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->localAnchorA = from->localAnchorA;
    to->localAnchorB = from->localAnchorB;
    to->linearHertz = from->linearHertz;
    to->linearDampingRatio = from->linearDampingRatio;
    to->angularHertz = from->angularHertz;
    to->angularDampingRatio = from->angularDampingRatio;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->enableMotor = from->enableMotor;
    to->motorSpeed = from->motorSpeed;
    to->maxMotorTorque = from->maxMotorTorque;
    to->enableLimit = from->enableLimit;
    to->lowerTranslation = from->lowerTranslation;
    to->upperTranslation = from->upperTranslation;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
    to->internalValue = from->internalValue;
}

static void FilledMouseJointDef(m2MouseJointDef* to, const m2MouseJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->target = from->target;
    to->hertz = from->hertz;
    to->dampingRatio = from->dampingRatio;
    to->maxForce = from->maxForce;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
    to->internalValue = from->internalValue;
}

static void FilledGearJointDef(m2GearJointDef* to, const m2GearJointDef* from, uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->ratio = from->ratio;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    to->collideConnected = from->collideConnected;
    to->internalValue = from->internalValue;
}

static void FilledRatchetJointDef(m2RatchetJointDef* to, const m2RatchetJointDef* from,
                                  uint8_t fill)
{
    memset(to, fill, sizeof(*to));
    to->bodyIdA = from->bodyIdA;
    to->bodyIdB = from->bodyIdB;
    to->ratchet = from->ratchet;
    to->phase = from->phase;
    to->userData = from->userData;
    to->collideConnected = from->collideConnected;
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
    worldDef.fluidVolumeCapacity = 2;
    return NewWorld(&worldDef);
}

// Records a create from every def the journal records into a fresh
// world's tape, each def's bytes starting as fill, and gives the world's
// hash at the end.
static uint8_t* TapeOfFilledDefs(uint8_t fill, int32_t* bytesOut, uint64_t* hashOut)
{
    m2World* world = NewDefWorld();
    m2BodyDef groundValues = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &groundValues);
    int32_t capacity = m2World_GetJournalBaseSize(world) + 16384;
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
    m2FluidVolumeDef volumeValues = m2DefaultFluidVolumeDef();
    m2FluidVolumeDef volumeDef;
    FilledFluidVolumeDef(&volumeDef, &volumeValues, fill);
    CHECK(m2FluidVolume_IsValid(world, NewFluidVolume(world, &volumeDef)),
          "a fluid volume creates");
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

    CHECK(m2World_GetCounters(world).misuse == 0,
          "every call passes its checks, the explosion too");
    *bytesOut = StopJournal(world);
    *hashOut = m2World_Hash(world);
    m2DestroyWorld(world);
    return tape;
}

// The bytes between a def's fields are not part of it: two runs of the
// same calls record the same tape whatever every recorded def's padding
// holds, and the tape replays onto the world that recorded it.
static void TestDefPaddingStaysOffTheTape(void)
{
    int32_t bytesA = 0;
    int32_t bytesB = 0;
    uint64_t hashA = 0;
    uint64_t hashB = 0;
    uint8_t* tapeA = TapeOfFilledDefs(0xA5, &bytesA, &hashA);
    uint8_t* tapeB = TapeOfFilledDefs(0x5A, &bytesB, &hashB);
    CHECK(bytesA > 0 && bytesA == bytesB && memcmp(tapeA, tapeB, (size_t)bytesA) == 0,
          "no def's padding reaches the tape");
    m2World* replayed = NewDefWorld();
    CHECK(ReplayJournal(replayed, tapeA, bytesA) && m2World_Hash(replayed) == hashA &&
              hashA == hashB,
          "and the tape replays onto the recorded world");
    m2DestroyWorld(replayed);
    free(tapeA);
    free(tapeB);
}

int main(void)
{
    TestTapeBoolsAreZeroOrOne();
    TestDefPaddingStaysOffTheTape();
    TestReplayIsAtomic();
    TestRestoreRefusesBadCounters();
    TestRollbackWhileRecording();
    TestRecordReplay();
    TestOverflowIsLoud();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
