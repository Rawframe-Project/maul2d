// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Hostile input and edge cases across the API: malformed defs, snapshot
// words, journal tapes, queries and fills are refused with a reason and
// leave the world as it was; and the corner cases of stale particle ids,
// resurrected machinery, chains, sleep and type changes behave.

#include "helpers.h"
#include "test_harness.h"
#include "test_task_pool.h"

#include "maul2d/base.h"
#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static m2BodyId AddBox(m2World* world, double x, double y)
{
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){x, y};
    m2BodyId body = NewBody(world, &bd);
    if (body.index1 != 0)
    {
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon box = m2MakeBox(0.4f, 0.4f);
        NewPolygonShape(world, body, &sd, &box);
    }
    return body;
}

// ---- Fluids, machinery, jelly, decompose -----------------------------------

// Stale particle ids across slot rebirth: the generation discipline
// holds for water exactly as it does for bodies.
static void TestParticleStaleIds(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = 8;
    m2World* world = NewWorld(&def);
    m2ParticleId ids[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        ids[i] = EmitParticle(world, (m2Pos2){(double)i, 0.0}, (m2Vec2){0.0f, 0.0f}, 0);
    }
    for (int32_t i = 0; i < 8; ++i)
    {
        DestroyParticle(world, ids[i]);
    }
    // Drain the FIFO so every original slot is reborn once.
    m2ParticleId reborn[8];
    for (int32_t i = 0; i < 8; ++i)
    {
        reborn[i] = EmitParticle(world, (m2Pos2){0.0, (double)i}, (m2Vec2){0.0f, 0.0f}, 0);
        CHECK(m2Particle_IsValid(world, reborn[i]), "reborn slots are live");
    }
    for (int32_t i = 0; i < 8; ++i)
    {
        CHECK(!m2Particle_IsValid(world, ids[i]), "dead ids stay dead after rebirth");
        m2Pos2 p = m2Particle_GetPosition(world, ids[i]);
        CHECK(p.x == 0.0 && p.y == 0.0, "stale reads return zero, never the tenant's data");
        CHECK(m2Particle_GetFlags(world, ids[i]) == 0, "stale flags read zero");
    }
    m2DestroyWorld(world);
}

// A jelly blob under a rollback storm: snapshot mid-flight, mutilate
// the future (kill net particles, shove the rest), restore, and the
// timeline replays bit for bit including the nets.
static void TestJellyRollbackStorm(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = 128;
    m2World* world = NewWorld(&def);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.2};
    m2Polygon slab = m2MakeBox(6.0f, 0.2f);
    NewPolygonShape(world, NewBody(world, &fd), &sd, &slab);
    m2Polygon blob = m2MakeBox(0.25f, 0.25f);
    FillParticles(world, &blob, (m2Pos2){0.0, 0.8}, (m2Vec2){0.0f, 0.0f},
                  m2_springParticle | m2_elasticParticle);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    Snapshot(world, snap, size);

    // The doomed future: tear holes in the net and shove survivors.
    m2ParticleId ids[128];
    int32_t n = m2World_GetParticles(world, ids, 128);
    for (int32_t i = 0; i < n; i += 5)
    {
        DestroyParticle(world, ids[i]);
    }
    n = m2World_GetParticles(world, ids, 128);
    for (int32_t i = 0; i < n; i += 3)
    {
        m2Particle_SetVelocity(world, ids[i], (m2Vec2){4.0f, 4.0f});
    }
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    // Roll back and run the canonical future twice: identical bits.
    CHECK(Restore(world, snap, size), "the storm snapshot restores");
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t first = m2World_Hash(world);
    CHECK(Restore(world, snap, size), "the second restore lands too");
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(world) == first, "the jelly timeline replays bit for bit");
    free(snap);
    m2DestroyWorld(world);
}

// The same storm through the tape: fills, tears and shoves recorded,
// then replayed onto a fresh world, landing on the recorded hash.
static void TestJellyJournalStorm(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = 128;
    m2World* world = NewWorld(&def);
    int32_t cap = 1 << 20;
    void* tape = malloc((size_t)cap);
    CHECK(StartJournal(world, tape, cap), "the storm tape starts");
    m2Polygon blob = m2MakeBox(0.2f, 0.2f);
    FillParticles(world, &blob, (m2Pos2){0.0, 1.0}, (m2Vec2){0.0f, 0.0f}, m2_springParticle);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2ParticleId ids[128];
    int32_t n = m2World_GetParticles(world, ids, 128);
    for (int32_t i = 0; i < n; i += 4)
    {
        DestroyParticle(world, ids[i]);
    }
    FillParticles(world, &blob, (m2Pos2){1.0, 1.0}, (m2Vec2){0.0f, -1.0f}, m2_elasticParticle);
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t used = StopJournal(world);
    CHECK(used > 0, "the storm tape closed");
    uint64_t recorded = m2World_Hash(world);
    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, used), "the storm tape replays");
    CHECK(m2World_Hash(fresh) == recorded, "torn nets replay onto the same bits");
    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(tape);
}

// Hostile fills: polygons smaller than a stride, pools that fill
// mid-lattice, and double fills that must never cross-link.
static void TestHostileFills(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    def.particleCapacity = 32;
    def.gravity = (m2Vec2){0.0f, 0.0f};
    m2World* world = NewWorld(&def);

    // Smaller than one stride: zero particles, zero nets, no crash.
    m2Polygon speck = m2MakeBox(0.02f, 0.02f);
    int32_t speckMade =
        FillParticles(world, &speck, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, m2_springParticle);
    CHECK(speckMade <= 1, "a speck holds at most one droplet");

    // A pool that fills mid-lattice: nets only reference what exists.
    m2Polygon big = m2MakeBox(0.5f, 0.5f);
    int32_t made = FillParticles(world, &big, (m2Pos2){5.0, 0.0}, (m2Vec2){0.0f, 0.0f},
                                 m2_springParticle | m2_elasticParticle);
    CHECK(made == 32 - speckMade, "the pool grants exactly its remaining capacity");
    CHECK(m2World_GetParticleCount(world) == 32, "the pool is exactly full");
    // m2World_Validate checks that every spring end and triad corner is
    // a live particle.
    CHECK(m2World_Validate(world), "the nets reference only live particles");
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    m2ParticleId ids[32];
    int32_t n = m2World_GetParticles(world, ids, 32);
    for (int32_t i = 0; i < n; ++i)
    {
        m2Pos2 p = m2Particle_GetPosition(world, ids[i]);
        CHECK(p.x == p.x && p.y == p.y, "a truncated lattice stays finite");
    }
    m2DestroyWorld(world);
}

// Machinery under the knife: destroy gears, pulleys and ratchets
// mid-sim, kill a shared body, and resurrect the whole workshop
// through a snapshot.
static void TestMachineryResurrection(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 16;
    m2World* world = NewWorld(&def);
    m2Circle disc = {{0.0f, 0.0f}, 0.4f};
    m2BodyId wheels[3];
    m2JointId pins[3];
    for (int32_t i = 0; i < 3; ++i)
    {
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){(double)i * 2.0, 2.0};
        m2BodyId post = NewBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){(double)i * 2.0, 2.0};
        wheels[i] = NewBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        NewCircleShape(world, wheels[i], &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheels[i];
        if (i == 0)
        {
            rj.enableMotor = true;
            rj.motorSpeed = 2.0f;
            rj.maxMotorTorque = 50.0f;
        }
        pins[i] = NewRevoluteJoint(world, &rj);
    }
    (void)pins;
    m2GearJointDef gd = m2DefaultGearJointDef();
    gd.bodyIdA = wheels[0];
    gd.bodyIdB = wheels[1];
    gd.ratio = 1.5f;
    m2JointId gear = NewGearJoint(world, &gd);
    m2RatchetJointDef rd = m2DefaultRatchetJointDef();
    rd.bodyIdA = wheels[1];
    rd.bodyIdB = wheels[2];
    rd.ratchet = 0.4f;
    m2JointId pawl = NewRatchetJoint(world, &rd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    Snapshot(world, snap, size);

    // The knife: cut the gear, kill the ratchet's far wheel (its
    // joints must die with it), keep stepping.
    DestroyJoint(world, gear);
    DestroyBody(world, wheels[2]);
    CHECK(!m2Joint_IsValid(world, pawl), "the ratchet dies with its wheel");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_GetParticleCount(world) == 0, "no stowaways");

    // Resurrection: the workshop returns, gear ratio, ratchet tooth
    // and all, and the future is deterministic twice over.
    CHECK(Restore(world, snap, size), "the workshop restores");
    CHECK(m2Joint_IsValid(world, gear) && m2Joint_IsValid(world, pawl), "the machines came back");
    CHECK(m2GearJoint_GetRatio(world, gear) == 1.5f, "the gear remembers its ratio");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    uint64_t first = m2World_Hash(world);
    Restore(world, snap, size);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(world) == first, "the resurrected workshop is deterministic");
    free(snap);
    m2DestroyWorld(world);
}

// A ratchet fighting a reversing motor: twins agree forever.
static void TestRatchetMotorWar(void)
{
    uint64_t hashes[2];
    for (int32_t run = 0; run < 2; ++run)
    {
        m2WorldDef def = m2DefaultWorldDef();
        def.bodyCapacity = 8;
        def.shapeCapacity = 8;
        def.jointCapacity = 8;
        def.gravity = (m2Vec2){0.0f, 0.0f};
        m2World* world = NewWorld(&def);
        m2BodyDef pd = m2DefaultBodyDef();
        pd.position = (m2Pos2){0.0, 2.0};
        m2BodyId post = NewBody(world, &pd);
        m2BodyDef wd = m2DefaultBodyDef();
        wd.type = m2_dynamicBody;
        wd.position = (m2Pos2){0.0, 2.0};
        m2BodyId wheel = NewBody(world, &wd);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Circle disc = {{0.0f, 0.0f}, 0.4f};
        NewCircleShape(world, wheel, &sd, &disc);
        m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
        rj.bodyIdA = post;
        rj.bodyIdB = wheel;
        rj.enableMotor = true;
        rj.motorSpeed = 3.0f;
        rj.maxMotorTorque = 40.0f;
        m2JointId pin = NewRevoluteJoint(world, &rj);
        m2RatchetJointDef rd = m2DefaultRatchetJointDef();
        rd.bodyIdA = post;
        rd.bodyIdB = wheel;
        rd.ratchet = 0.3f;
        NewRatchetJoint(world, &rd);
        for (int32_t i = 0; i < 240; ++i)
        {
            if (i % 30 == 0)
            {
                m2Joint_SetMotorSpeed(world, pin, i % 60 == 0 ? 3.0f : -3.0f);
            }
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        m2Vec2 v = {m2Body_GetAngularVelocity(world, wheel), 0.0f};
        CHECK(v.x == v.x, "the war stays finite");
        hashes[run] = m2World_Hash(world);
        m2DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the ratchet-motor war is twin deterministic");
}

// Decompose under adversarial outlines: collinear runs, needle-thin
// notches, and the full 64-point budget all keep the area or bounce.
static void TestDecomposeAdversaries(void)
{
    // A comb: deep thin notches, many reflex corners.
    m2Vec2 comb[12] = {{0.0f, 0.0f}, {3.0f, 0.0f}, {3.0f, 2.0f}, {2.4f, 2.0f},
                       {2.4f, 0.5f}, {1.8f, 0.5f}, {1.8f, 2.0f}, {1.2f, 2.0f},
                       {1.2f, 0.5f}, {0.6f, 0.5f}, {0.6f, 2.0f}, {0.0f, 2.0f}};
    m2Polygon pieces[24];
    int32_t n = DecomposeOutline(comb, 12, pieces, 24);
    CHECK(n >= 3, "the comb splits");
    float area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        float a2 = 0.0f;
        for (int32_t k = 0; k < pieces[i].count; ++k)
        {
            m2Vec2 p = pieces[i].vertices[k];
            m2Vec2 q = pieces[i].vertices[(k + 1) % pieces[i].count];
            a2 += p.x * q.y - q.x * p.y;
        }
        area += 0.5f * a2;
    }
    // Comb area: 3x2 minus two 0.6x1.5 notches = 6 - 1.8 = 4.2.
    CHECK(area > 4.19f && area < 4.21f, "the comb keeps its area");

    // Collinear runs on the hull: straight vertices clip for free.
    m2Vec2 ruler[8] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {3.0f, 0.0f},
                       {3.0f, 1.0f}, {2.0f, 1.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};
    n = DecomposeOutline(ruler, 8, pieces, 24);
    CHECK(n >= 1, "the ruler decomposes");
    area = 0.0f;
    for (int32_t i = 0; i < n; ++i)
    {
        float a2 = 0.0f;
        for (int32_t k = 0; k < pieces[i].count; ++k)
        {
            m2Vec2 p = pieces[i].vertices[k];
            m2Vec2 q = pieces[i].vertices[(k + 1) % pieces[i].count];
            a2 += p.x * q.y - q.x * p.y;
        }
        area += 0.5f * a2;
    }
    CHECK(area > 2.99f && area < 3.01f, "collinear runs keep the area");

    // The full budget: a 64-point saw blade.
    m2Vec2 saw[64];
    for (int32_t i = 0; i < 32; ++i)
    {
        saw[2 * i] = (m2Vec2){(float)i * 0.2f, 0.0f};
        saw[2 * i + 1] = (m2Vec2){(float)i * 0.2f + 0.1f, i == 31 ? 1.0f : 0.3f};
    }
    // Close the top edge by walking back: rebuild as a proper ring.
    m2Vec2 blade[64];
    for (int32_t i = 0; i < 62; ++i)
    {
        blade[i] = saw[i];
    }
    blade[62] = (m2Vec2){6.3f, 1.2f};
    blade[63] = (m2Vec2){0.0f, 1.2f};
    n = DecomposeOutline(blade, 64, pieces, 24);
    CHECK(n > 0, "the full 64-point budget decomposes");
    int32_t total = DecomposeOutline(blade, 64, NULL, 0);
    CHECK(total == n || total > 24, "count queries agree with fills");

    // Outlines that touch themselves without crossing, found by
    // fuzz_geometry: their pieces could cover what the outline cuts out.
    // The second point lies on the fourth edge, p3 to p4.
    const m2Vec2 onEdge[5] = {{10.125f, -13.5f},
                              {0.125f, 0.375f},
                              {0.375f, 10.125f},
                              {0.375f, 0.375f},
                              {-15.875f, 0.375f}};
    const m2Vec2 repeated[6] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {1.0f, 1.0f},
                                {2.0f, 2.0f}, {0.0f, 2.0f}, {1.0f, 1.0f}};
    const m2Vec2 folded[5] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 2.0f}, {2.0f, 1.0f}, {0.0f, 2.0f}};
    const m2Vec2 overshot[6] = {{0.0f, 0.0f}, {3.0f, 0.0f}, {3.0f, 1.0f},
                                {3.0f, 3.0f}, {3.0f, 0.5f}, {0.0f, 3.0f}}; // folds back past (3, 1)
    int32_t untouched = 7;
    CHECK(m2DecomposeOutline(onEdge, 5, pieces, 24, &untouched) == m2_errorInvalid &&
              m2DecomposeOutline(repeated, 6, pieces, 24, &untouched) == m2_errorInvalid &&
              m2DecomposeOutline(folded, 5, pieces, 24, &untouched) == m2_errorInvalid &&
              m2DecomposeOutline(overshot, 6, pieces, 24, &untouched) == m2_errorInvalid,
          "an outline that touches itself is refused");
    CHECK(untouched == 7, "and leaves the count as it was");
}

static void TestJournalDefenses(void)
{
    // Header attacks bounce before the world is touched; a healthy
    // journal replays twice with identical end bits.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    uint8_t buffer[1 << 16];
    StartJournal(world, buffer, (int32_t)sizeof(buffer));

    m2BodyId box = AddBox(world, 0.0, 4.0);
    m2Body_SetLinearVelocity(world, box, (m2Vec2){1.5f, 0.0f});
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t bytes = StopJournal(world);
    CHECK(bytes > 0, "journal captured");
    uint64_t recordedHash = m2World_Hash(world);
    m2DestroyWorld(world);

    m2World* fresh = NewWorld(&def);

    uint8_t corrupt[1 << 16];
    memcpy(corrupt, buffer, (size_t)bytes);
    corrupt[0] ^= 0xFF; // magic
    CHECK(!ReplayJournal(fresh, corrupt, bytes), "corrupt magic rejected");

    CHECK(!ReplayJournal(fresh, buffer, 16), "truncated header rejected");

    m2WorldDef small = m2DefaultWorldDef();
    small.bodyCapacity = 4;
    small.shapeCapacity = 4;
    m2World* wrongCapacity = NewWorld(&small);
    CHECK(!ReplayJournal(wrongCapacity, buffer, bytes), "capacity mismatch rejected");
    m2DestroyWorld(wrongCapacity);

    CHECK(ReplayJournal(fresh, buffer, bytes), "healthy replay works");
    CHECK(m2World_Hash(fresh) == recordedHash, "replay lands on the recorded bits");
    CHECK(ReplayJournal(fresh, buffer, bytes), "second replay works");
    CHECK(m2World_Hash(fresh) == recordedHash, "second replay lands on the same bits");

    m2DestroyWorld(fresh);
}

// The 16-byte prefix every state format starts with: magic, ABI version,
// format number, header size (docs/internals/snapshots-and-journal.md).
enum
{
    PREFIX_ABI = 4,
    PREFIX_FORMAT = 8,
    PREFIX_HEADER_BYTES = 12,
    SNAPSHOT_CONFIG_HASH = 16
};

static uint32_t ReadWord(const uint8_t* bytes, int32_t offset)
{
    uint32_t word = 0;
    memcpy(&word, bytes + offset, sizeof(word));
    return word;
}

static uint32_t Reversed(uint32_t word)
{
    return word >> 24 | (word >> 8 & 0xFF00u) | (word << 8 & 0xFF0000u) | word << 24;
}

// Offers state bytes: a snapshot to m2World_Restore, a tape to
// m2World_ReplayJournal. The refusal must be the expected status, change
// nothing and count nothing, since the bytes are untrusted.
static void ExpectRefused(m2World* world, const uint8_t* bytes, int32_t size, bool tape,
                          m2Result expected, const char* what)
{
    uint64_t hash = m2World_Hash(world);
    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Result result =
        tape ? m2World_ReplayJournal(world, bytes, size) : m2World_Restore(world, bytes, size);
    CHECK(result == expected, what);
    CHECK(m2World_Hash(world) == hash && m2World_GetCounters(world).misuse == misuse,
          "refused state bytes change nothing and count nothing");
}

// Offers state bytes with one 32-bit word changed and every other byte
// equal.
static void ExpectWordRefused(m2World* world, const uint8_t* bytes, int32_t size, bool tape,
                              int32_t offset, uint32_t word, m2Result expected, const char* what)
{
    uint8_t* edited = (uint8_t*)malloc((size_t)size);
    CHECK(edited != NULL, "a copy of the state bytes");
    if (edited == NULL)
    {
        return;
    }
    memcpy(edited, bytes, (size_t)size);
    memcpy(edited + offset, &word, sizeof(word));
    ExpectRefused(world, edited, size, tape, expected, what);
    free(edited);
}

// Offers the first `keep` bytes of state bytes, in an allocation of
// exactly that size, so AddressSanitizer catches a reader that looks past
// the end. Each cut is refused as m2_errorFormat.
static void ExpectCutRefused(m2World* world, const uint8_t* bytes, int32_t keep, bool tape,
                             const char* what)
{
    uint8_t* cut = (uint8_t*)malloc((size_t)keep);
    CHECK(cut != NULL, "a copy of the state bytes");
    if (cut == NULL)
    {
        return;
    }
    memcpy(cut, bytes, (size_t)keep);
    ExpectRefused(world, cut, keep, tape, m2_errorFormat, what);
    free(cut);
}

// Only the release that wrote a snapshot or a tape reads it: another ABI
// version, format number or build hash is m2_errorVersion, while another
// magic or header size is not this format at all and is m2_errorFormat.
// The tape's embedded snapshot is checked the same way and its status
// passes through. Each magic's bytes spell its name in order, and the
// same name stored in the other byte order is another magic. Bytes cut
// short behind a valid prefix are m2_errorFormat.
static void TestOtherReleasesAreRefused(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    uint8_t tape[1 << 16];
    StartJournal(world, tape, (int32_t)sizeof(tape));
    m2BodyId box = AddBox(world, 0.0, 4.0);
    m2Body_SetLinearVelocity(world, box, (m2Vec2){1.5f, 0.0f});
    for (int32_t i = 0; i < 10; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t tapeBytes = StopJournal(world);
    CHECK(tapeBytes > 0, "the tape is recorded");
    int32_t size = m2World_SnapshotSize(world);
    uint8_t* snap = (uint8_t*)malloc((size_t)size);
    CHECK(snap != NULL && Snapshot(world, snap, size) == size, "the snapshot is taken");
    if (snap == NULL || tapeBytes <= 0)
    {
        free(snap);
        m2DestroyWorld(world);
        return;
    }
    m2World_Step(world, 1.0f / 60.0f, 4); // a state the refusals must keep

    CHECK(ReadWord(snap, PREFIX_ABI) == M2_ABI_VERSION &&
              ReadWord(tape, PREFIX_ABI) == M2_ABI_VERSION,
          "both headers carry this release's ABI version");
    uint32_t tapeHeader = ReadWord(tape, PREFIX_HEADER_BYTES);
    int32_t embedded = (int32_t)tapeHeader;
    CHECK(memcmp(snap, "M2SN", 4) == 0 && memcmp(tape, "M2JN", 4) == 0 &&
              memcmp(tape + embedded, "M2SN", 4) == 0,
          "each magic's bytes spell its name");
    const uint32_t otherRelease = M2_ABI_VERSION ^ 0x100u; // the next minor before 1.0
    uint32_t snapFormat = ReadWord(snap, PREFIX_FORMAT);
    uint32_t snapHeader = ReadWord(snap, PREFIX_HEADER_BYTES);
    ExpectWordRefused(world, snap, size, false, PREFIX_ABI, otherRelease, m2_errorVersion,
                      "a snapshot from another release is m2_errorVersion");
    ExpectWordRefused(world, snap, size, false, PREFIX_FORMAT, snapFormat + 1u, m2_errorVersion,
                      "a snapshot of another format number is m2_errorVersion");
    ExpectWordRefused(world, snap, size, false, SNAPSHOT_CONFIG_HASH,
                      ReadWord(snap, SNAPSHOT_CONFIG_HASH) ^ 1u, m2_errorVersion,
                      "a snapshot from another build is m2_errorVersion");
    ExpectWordRefused(world, snap, size, false, 0, ReadWord(snap, 0) ^ 0xFFu, m2_errorFormat,
                      "bytes with another magic are m2_errorFormat");
    ExpectWordRefused(world, snap, size, false, 0, Reversed(ReadWord(snap, 0)), m2_errorFormat,
                      "a snapshot magic in the other byte order is m2_errorFormat");
    ExpectWordRefused(world, snap, size, false, PREFIX_HEADER_BYTES, snapHeader + 8u,
                      m2_errorFormat, "a snapshot with another header size is m2_errorFormat");

    ExpectWordRefused(world, tape, tapeBytes, true, 0, Reversed(ReadWord(tape, 0)), m2_errorFormat,
                      "a tape magic in the other byte order is m2_errorFormat");
    ExpectWordRefused(world, tape, tapeBytes, true, PREFIX_ABI, otherRelease, m2_errorVersion,
                      "a tape from another release is m2_errorVersion");
    ExpectWordRefused(world, tape, tapeBytes, true, PREFIX_FORMAT,
                      ReadWord(tape, PREFIX_FORMAT) + 1u, m2_errorVersion,
                      "a tape of another format number is m2_errorVersion");
    ExpectWordRefused(world, tape, tapeBytes, true, PREFIX_HEADER_BYTES, tapeHeader + 8u,
                      m2_errorFormat, "a tape with another header size is m2_errorFormat");
    ExpectWordRefused(world, tape, tapeBytes, true, embedded + PREFIX_ABI, otherRelease,
                      m2_errorVersion,
                      "a tape embedding another release's snapshot is m2_errorVersion");

    // Cut behind a valid prefix: at the prefix, one byte short of the
    // header, inside the tape's embedded snapshot header and one byte
    // short of the end.
    int32_t snapHeaderBytes = (int32_t)snapHeader;
    ExpectCutRefused(world, snap, 16, false, "a snapshot of its prefix alone is m2_errorFormat");
    ExpectCutRefused(world, snap, snapHeaderBytes - 1, false,
                     "a snapshot cut inside its header is m2_errorFormat");
    ExpectCutRefused(world, snap, size - 1, false, "a snapshot cut short is m2_errorFormat");
    ExpectCutRefused(world, tape, 16, true, "a tape of its prefix alone is m2_errorFormat");
    ExpectCutRefused(world, tape, embedded - 1, true,
                     "a tape cut inside its header is m2_errorFormat");
    ExpectCutRefused(world, tape, embedded + 16, true,
                     "a tape cut after its snapshot's prefix is m2_errorFormat");
    ExpectCutRefused(world, tape, embedded + snapHeaderBytes - 1, true,
                     "a tape cut inside its snapshot's header is m2_errorFormat");
    ExpectCutRefused(world, tape, tapeBytes - 1, true, "a tape cut short is m2_errorFormat");

    // The bytes as written still load, so only the changed word was refused.
    CHECK(Restore(world, snap, size), "the snapshot as written restores");
    CHECK(ReplayJournal(world, tape, tapeBytes), "the tape as written replays");

    free(snap);
    m2DestroyWorld(world);
}

// Parameter channels validate once, for live setters and replay alike:
// a hostile value is refused and counted, and a tape whose channel byte
// was tampered with is rejected instead of asserting or writing a
// random field.
static void TestParameterChannels(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2BodyId box = AddBox(world, 0.0, 4.0);
    m2ShapeId shape;
    m2Body_GetShapes(world, box, &shape, 1);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = box;
    m2JointId pin = NewRevoluteJoint(world, &rj);

    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Joint_SetMotorSpeed(world, pin, NAN);
    m2Joint_SetMaxMotor(world, pin, -1.0f);
    m2Joint_SetLimits(world, pin, 1.0f, -1.0f);
    m2DistanceJoint_SetLength(world, pin, 2.0f); // a revolute has no length
    m2Shape_SetFriction(world, shape, INFINITY);
    m2Body_SetGravityScale(world, box, NAN);
    CHECK(m2World_GetCounters(world).misuse == misuse + 6, "each hostile parameter refuses once");

    m2Joint_SetLimits(world, pin, -1.0f, 1.0f);
    m2Shape_SetFriction(world, shape, 0.3f);
    m2Body_SetGravityScale(world, box, 0.5f);
    CHECK(m2World_GetCounters(world).misuse == misuse + 6, "honest parameters pass");

    m2Body_SetLinearDamping(world, box, 0.5f);
    m2Body_SetAngularDamping(world, box, 0.5f);
    misuse = m2World_GetCounters(world).misuse;
    m2Body_SetLinearDamping(world, box, -1.0f);
    m2Body_SetLinearDamping(world, box, -INFINITY);
    m2Body_SetAngularDamping(world, box, -1.0f);
    m2Body_SetAngularDamping(world, box, -INFINITY);
    CHECK(m2World_GetCounters(world).misuse == misuse + 4, "each negative damping counts once");
    CHECK(m2Body_GetLinearDamping(world, box) == 0.5f &&
              m2Body_GetAngularDamping(world, box) == 0.5f,
          "a negative damping changes nothing");

    uint8_t tape[1 << 16];
    int32_t bytes = 0;
    StartJournal(world, tape, (int32_t)sizeof(tape));
    m2Body_SetLinearDamping(world, box, 0.25f);
    bytes = StopJournal(world);
    // The last record is op 25 (body param) and a 16-byte payload whose
    // channel byte sits at offset 12.
    CHECK(bytes > 17 && tape[bytes - 17] == 25, "the tape ends in a body-param record");
    tape[bytes - 4] = 200;
    CHECK(m2World_ReplayJournal(world, tape, bytes) == m2_errorFormat,
          "an unknown body channel is a malformed tape");

    StartJournal(world, tape, (int32_t)sizeof(tape));
    m2Shape_SetRestitution(world, shape, 0.5f);
    bytes = StopJournal(world);
    CHECK(bytes > 17 && tape[bytes - 17] == 22, "the tape ends in a shape-param record");
    tape[bytes - 4] = 7;
    CHECK(!ReplayJournal(world, tape, bytes), "an unknown shape channel is rejected");

    m2WorldDef other = def;
    other.bodyCapacity = 4;
    m2World* elsewhere = NewWorld(&other);
    CHECK(m2World_ReplayJournal(elsewhere, tape, bytes) == m2_errorFormat,
          "a tape for another world shape is a malformed tape");
    m2DestroyWorld(elsewhere);

    misuse = m2World_GetCounters(world).misuse;
    DestroyJoint(world, pin);
    DestroyJoint(world, pin);
    m2Joint_SetMotorSpeed(world, pin, 1.0f);
    CHECK(m2World_GetCounters(world).misuse == misuse + 2, "stale joint calls refuse and count");
    m2DestroyWorld(world);
}

// Query inputs are caller data: an oversized polygon count, a NULL
// shape, a NaN pose or a sheared rotation is refused before anything
// is read, never copied past the proxy or walked through the tree.
static void TestHostileQueries(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.particleCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyId box = AddBox(world, 0.0, 0.0);
    m2QueryFilter filter = m2DefaultQueryFilter();
    m2Transform pose = {{0.0, 3.0}, {1.0f, 0.0f}};
    m2Vec2 down = {0.0f, -6.0f};
    m2ShapeId ids[4];
    m2CastHit hits[4];

    uint64_t misuse = m2World_GetCounters(world).misuse;
    m2Polygon huge = m2MakeBox(0.5f, 0.5f);
    huge.count = 1000;
    CHECK(!PolygonClosest(world, &huge, pose, down, filter).hit,
          "an oversized polygon count casts nothing");
    CHECK(m2World_OverlapPolygon(world, &huge, pose, ids, 4, filter) == 0, "and overlaps nothing");
    CHECK(m2World_CastPolygonAll(world, &huge, pose, down, hits, 4, filter) == 0,
          "and sweeps nothing");
    m2ParticleDef particleDef = m2DefaultParticleDef();
    int32_t count = -1;
    CHECK(m2CreateParticlesInPolygon(world, &particleDef, &huge, &count) == m2_errorLimit &&
              count == 0,
          "and fills nothing, past M2_MAX_POLYGON_VERTICES");
    CHECK(!CircleClosest(world, NULL, pose, down, filter).hit, "a NULL circle casts nothing");
    m2Circle ball = {{0.0f, 0.0f}, 0.25f};
    m2Transform nanPose = {{(double)NAN, 3.0}, {1.0f, 0.0f}};
    CHECK(m2World_OverlapCircle(world, &ball, nanPose, ids, 4, filter) == 0,
          "a NaN pose overlaps nothing");
    m2Transform sheared = {{0.0, 3.0}, {2.0f, 0.0f}};
    CHECK(!CircleClosest(world, &ball, sheared, down, filter).hit,
          "a non-unit rotation casts nothing");
    CHECK(!RayClosest(world, (m2Pos2){0.0, 3.0}, (m2Vec2){NAN, -6.0f}, filter).hit,
          "a NaN ray hits nothing");
    CHECK(m2World_OverlapAabb(world, (m2WorldAabb){{(double)NAN, 0.0}, {1.0, 1.0}}, ids, 4,
                              filter) == 0,
          "a NaN box overlaps nothing");
    CHECK(m2World_GetCounters(world).misuse == misuse + 9, "every hostile query refuses once");

    CHECK(CircleClosest(world, &ball, pose, down, filter).hit, "an honest cast still hits");
    uint64_t beforeTeleport = m2World_GetCounters(world).misuse;
    m2Body_SetTransform(world, box, (m2Pos2){1.0, 0.0}, (m2Rot){2.0f, 0.0f});
    CHECK(m2World_GetCounters(world).misuse == beforeTeleport + 1,
          "a teleport with a sheared rotation is refused");
    m2Body_SetLinearVelocity(world, box, (m2Vec2){INFINITY, 0.0f});
    CHECK(m2Body_GetLinearVelocity(world, box).x == 0.0f, "an infinite velocity never lands");
    m2World_SetGravity(world, (m2Vec2){0.0f, NAN});
    CHECK(m2World_GetGravity(world).y == def.gravity.y, "a NaN gravity never lands");
    m2DestroyWorld(world);
}

// Joint defs are caller data: a NaN anchor, a negative stiffness, an
// inverted range or a correction factor past one is refused at create
// instead of poisoning the solver.
static void TestHostileJointDefs(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2BodyId box = AddBox(world, 0.0, 2.0);
    uint64_t misuse = m2World_GetCounters(world).misuse;

    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = box;
    rj.localAnchorA = (m2Vec2){NAN, 0.0f};
    CHECK(NewRevoluteJoint(world, &rj).index1 == 0, "a NaN anchor is refused");
    rj.localAnchorA = (m2Vec2){0.0f, 0.0f};
    rj.lowerAngle = 1.0f;
    rj.upperAngle = -1.0f;
    CHECK(NewRevoluteJoint(world, &rj).index1 == 0, "an inverted angle range is refused");

    m2DistanceJointDef dj = m2DefaultDistanceJointDef();
    dj.bodyIdA = ground;
    dj.bodyIdB = box;
    dj.springHertz = -2.0f;
    CHECK(NewDistanceJoint(world, &dj).index1 == 0, "a negative stiffness is refused");

    m2MotorJointDef mj = m2DefaultMotorJointDef();
    mj.bodyIdA = ground;
    mj.bodyIdB = box;
    mj.correctionFactor = 2.0f;
    CHECK(NewMotorJoint(world, &mj).index1 == 0, "a correction factor past one is refused");

    m2MouseJointDef sj = m2DefaultMouseJointDef();
    sj.bodyIdA = ground;
    sj.bodyIdB = box;
    sj.target = (m2Pos2){(double)INFINITY, 0.0};
    CHECK(NewMouseJoint(world, &sj).index1 == 0, "an infinite target is refused");
    CHECK(m2World_GetCounters(world).misuse == misuse + 5, "each refusal counts once");

    rj.lowerAngle = -1.0f;
    rj.upperAngle = 1.0f;
    CHECK(NewRevoluteJoint(world, &rj).index1 != 0, "an honest def still creates");
    m2DestroyWorld(world);
}

// Restore checks every incoming block before any of it lands: poison
// each 32-bit word of a snapshot in turn with a huge value. Every index,
// count, flag and float block must refuse it, and a refused restore
// leaves the world untouched. (Words the checks accept, like user data
// and generations, carry no index.)
static void TestHostileSnapshotWords(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    def.jointCapacity = 2;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon floor = m2MakeBox(5.0f, 0.5f);
    NewPolygonShape(world, ground, &sd, &floor);
    m2BodyId box = AddBox(world, 0.0, 1.2);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = ground;
    rj.bodyIdB = box;
    NewRevoluteJoint(world, &rj);
    m2World_Step(world, 1.0f / 60.0f, 4);

    int32_t size = m2World_SnapshotSize(world);
    uint8_t* clean = (uint8_t*)malloc((size_t)size);
    uint8_t* poisoned = (uint8_t*)malloc((size_t)size);
    CHECK(clean != NULL && poisoned != NULL, "snapshot buffers");
    if (clean == NULL || poisoned == NULL)
    {
        free(clean);
        free(poisoned);
        return;
    }
    Snapshot(world, clean, size);
    uint64_t cleanHash = m2World_Hash(world);
    int32_t refused = 0;
    int32_t untouched = 0;
    for (int32_t at = 0; at + 4 <= size; at += 4)
    {
        memcpy(poisoned, clean, (size_t)size);
        int32_t huge = 0x7FFFFFFF;
        memcpy(poisoned + at, &huge, sizeof(huge));
        if (!Restore(world, poisoned, size))
        {
            refused += 1;
            untouched += m2World_Hash(world) == cleanHash ? 1 : 0;
            continue;
        }
        Restore(world, clean, size);
    }
    CHECK(refused > 0, "hostile words are refused");
    CHECK(untouched == refused, "a refused restore leaves the world as it was");
    free(clean);
    free(poisoned);
    m2DestroyWorld(world);
}

static void TestQueryEdges(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);

    // Empty world: everything misses quietly.
    CastResult miss =
        RayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){5.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(!miss.hit, "empty world misses");
    m2ShapeId results[4];
    CHECK(m2World_OverlapAabb(world, (m2WorldAabb){{-1.0, -1.0}, {1.0, 1.0}}, results, 4,
                              m2DefaultQueryFilter()) == 0,
          "empty world overlaps nothing");

    m2BodyId box = AddBox(world, 0.0, 0.0);

    // Zero-length ray inside a shape: initial-overlap convention.
    CastResult inside =
        RayClosest(world, (m2Pos2){0.0, 0.0}, (m2Vec2){0.0f, 0.0f}, m2DefaultQueryFilter());
    CHECK(inside.hit && inside.fraction == 0.0f, "zero ray inside reports initial overlap");
    CHECK(inside.normal.x == 0.0f && inside.normal.y == 0.0f, "initial overlap has no normal");

    // Destroyed shapes drop out of query results after the flush.
    DestroyBody(world, box);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2World_OverlapAabb(world, (m2WorldAabb){{-1.0, -1.0}, {1.0, 1.0}}, results, 4,
                              m2DefaultQueryFilter()) == 0,
          "destroyed shape leaves the query space");

    m2DestroyWorld(world);
}

static void TestMultiWorldIsolation(void)
{
    // Two identical worlds march in lockstep; killing one must not
    // move a single bit in the other.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* w1 = NewWorld(&def);
    m2World* w2 = NewWorld(&def);
    for (int32_t i = 0; i < 5; ++i)
    {
        AddBox(w1, (double)i, 6.0 + 0.7 * (double)i);
        AddBox(w2, (double)i, 6.0 + 0.7 * (double)i);
    }
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(w1, 1.0f / 60.0f, 4);
        m2World_Step(w2, 1.0f / 60.0f, 4);
    }
    CHECK(m2World_Hash(w1) == m2World_Hash(w2), "twin worlds march in lockstep");

    uint64_t expected[20];
    {
        // Reference trajectory from a third twin, no destruction around.
        m2World* w3 = NewWorld(&def);
        for (int32_t i = 0; i < 5; ++i)
        {
            AddBox(w3, (double)i, 6.0 + 0.7 * (double)i);
        }
        for (int32_t i = 0; i < 40; ++i)
        {
            m2World_Step(w3, 1.0f / 60.0f, 4);
        }
        for (int32_t i = 0; i < 20; ++i)
        {
            m2World_Step(w3, 1.0f / 60.0f, 4);
            expected[i] = m2World_Hash(w3);
        }
        m2DestroyWorld(w3);
    }

    m2DestroyWorld(w1); // kill the twin mid-flight
    for (int32_t i = 0; i < 20; ++i)
    {
        m2World_Step(w2, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(w2) == expected[i], "survivor unaffected by the neighbor's death");
    }
    m2DestroyWorld(w2);
}

static void TestWakeAfterFrozenEra(void)
{
    // No stale-manifold ghosts: a body that slept through many frozen
    // steps must move immediately when shoved, then earn sleep again.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyId box = AddBox(world, 0.0, 0.5);
    for (int32_t i = 0; i < 150; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, box), "asleep deep in the frozen era");
    double x0 = m2Body_GetPosition(world, box).x;

    m2Body_ApplyLinearImpulseAtPoint(world, box, (m2Vec2){2.0f, 0.0f},
                                     m2Body_GetPosition(world, box));
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, box).x > x0 + 0.5, "the shoved sleeper actually moves");

    bool sleptAgain = false;
    for (int32_t i = 0; i < 300 && !sleptAgain; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        sleptAgain = !m2Body_IsAwake(world, box);
    }
    CHECK(sleptAgain, "and earns its sleep back");
    m2DestroyWorld(world);
}

static void TestTeleportOntoSleepers(void)
{
    // Teleport a heavy box INTO a sleeping stack: overlap must resolve
    // through the ordinary pushout without an explosion, and the stack
    // must wake.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);
    m2BodyId a = AddBox(world, 0.0, 0.5);
    m2BodyId b = AddBox(world, 0.0, 1.42);
    m2BodyId intruder = AddBox(world, 8.0, 0.5);
    for (int32_t i = 0; i < 200; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, a) && !m2Body_IsAwake(world, b), "stack asleep");

    m2Body_SetTransform(world, intruder, (m2Pos2){0.1, 0.9}, (m2Rot){1.0f, 0.0f}); // into the stack
    bool exploded = false;
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2Vec2 v = m2Body_GetLinearVelocity(world, b);
        exploded = exploded || v.x * v.x + v.y * v.y > 100.0f;
    }
    CHECK(!exploded, "overlap resolves without an explosion");
    CHECK(m2Body_GetPosition(world, b).y > 0.3, "the stack sorts itself out");
    m2DestroyWorld(world);
}

static void TestSetTypeEdges(void)
{
    // Convert a sleeping body, a jointed body, and a bullet; flip-flop
    // types under twin-world scrutiny.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, -0.5};
    m2BodyId floor = NewBody(world, &fd);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(15.0f, 0.5f);
    NewPolygonShape(world, floor, &fs, &slab);

    // Sleeping conversion round trip.
    m2BodyId sleeper = AddBox(world, 0.0, 0.5);
    for (int32_t i = 0; i < 150; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, sleeper), "asleep before conversion");
    m2Body_SetType(world, sleeper, m2_staticBody);
    m2Body_SetType(world, sleeper, m2_dynamicBody);
    m2World_Step(world, 1.0f / 60.0f, 4);
    CHECK(m2Body_IsAwake(world, sleeper), "round-trip conversion wakes it");
    bool resleeps = false;
    for (int32_t i = 0; i < 200 && !resleeps; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        resleeps = !m2Body_IsAwake(world, sleeper);
    }
    CHECK(resleeps, "and it earns sleep again with real mass");

    // A pendulum whose pivot end turns static mid-swing.
    m2BodyDef ad = m2DefaultBodyDef();
    ad.type = m2_dynamicBody;
    ad.position = (m2Pos2){5.0, 4.0};
    m2BodyId pivot = NewBody(world, &ad);
    m2ShapeDef ps = m2DefaultShapeDef();
    m2Circle pc = {{0.0f, 0.0f}, 0.2f};
    NewCircleShape(world, pivot, &ps, &pc);
    m2BodyId bob = AddBox(world, 6.5, 4.0);
    m2RevoluteJointDef jd = m2DefaultRevoluteJointDef();
    jd.bodyIdA = pivot;
    jd.bodyIdB = bob;
    jd.localAnchorB = (m2Vec2){-1.5f, 0.0f};
    m2JointId hinge = NewRevoluteJoint(world, &jd);
    m2Body_SetType(world, pivot, m2_staticBody); // becomes a wall anchor
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Joint_IsValid(world, hinge), "the joint survives its anchor turning static");
    m2Pos2 pp = m2Body_GetPosition(world, pivot);
    CHECK(pp.x == 5.0 && pp.y == 4.0, "the static anchor holds its ground");
    double dx = m2Body_GetPosition(world, bob).x - pp.x;
    double dy = m2Body_GetPosition(world, bob).y - pp.y;
    double rod = dx * dx + dy * dy;
    CHECK(rod > 1.9 && rod < 2.6, "the bob still hangs from it");

    // A bullet that turns static mid-flight simply stops mattering.
    m2BodyDef bulletDef = m2DefaultBodyDef();
    bulletDef.type = m2_dynamicBody;
    bulletDef.isBullet = true;
    bulletDef.position = (m2Pos2){-8.0, 2.0};
    bulletDef.linearVelocity = (m2Vec2){60.0f, 0.0f};
    m2BodyId bullet = NewBody(world, &bulletDef);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle bc = {{0.0f, 0.0f}, 0.1f};
    NewCircleShape(world, bullet, &bs, &bc);
    m2World_Step(world, 1.0f / 60.0f, 4);
    m2Body_SetType(world, bullet, m2_staticBody);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4); // must not crash the sweep
    }
    CHECK(m2Body_GetLinearVelocity(world, bullet).x == 0.0f, "a static bullet is just a pebble");

    m2DestroyWorld(world);
}

static void TestLoopChainBowl(void)
{
    // A closed loop chain forming a diamond bowl: a ball dropped in
    // settles inside; degenerate chain defs bounce loudly.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, 0.0};
    m2BodyId frame = NewBody(world, &gd);
    // Diamond wound clockwise so the solid side faces INWARD.
    m2Vec2 diamond[4] = {{0.0f, 3.0f}, {3.0f, 0.0f}, {0.0f, -3.0f}, {-3.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = diamond;
    chain.count = 4;
    chain.isLoop = true;
    m2ChainId bowlChain = NewChain(world, frame, &chain);
    CHECK(m2Chain_IsLoop(world, bowlChain), "a loop reads back as one");
    CHECK(m2Chain_GetSegmentCount(world, bowlChain) == 4,
          "a loop of four points makes four segments");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.3, 1.5};
    m2BodyId ball = NewBody(world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle c = {{0.0f, 0.0f}, 0.3f};
    NewCircleShape(world, ball, &sd, &c);
    bool rested = false;
    for (int32_t i = 0; i < 600 && !rested; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        rested = !m2Body_IsAwake(world, ball);
    }
    CHECK(rested, "the ball settles inside the bowl");
    CHECK(m2Body_GetPosition(world, ball).y > -3.0, "and did not leak out the bottom");

    m2DestroyWorld(world);
}

static void TestBreakStorm(void)
{
    // Every rope in a curtain breaks on the same overloaded step: all
    // events arrive that step, in canonical joint order, and a
    // pre-break snapshot resurrects the whole curtain.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);
    m2JointId ropes[4];
    for (int32_t i = 0; i < 4; ++i)
    {
        m2BodyDef ad = m2DefaultBodyDef();
        ad.position = (m2Pos2){(double)i * 3.0, 6.0};
        m2BodyId hook = NewBody(world, &ad);
        m2BodyDef cd = m2DefaultBodyDef();
        cd.type = m2_dynamicBody;
        cd.position = (m2Pos2){(double)i * 3.0, 4.0};
        m2BodyId crate = NewBody(world, &cd);
        m2ShapeDef sd = m2DefaultShapeDef();
        sd.density = 5.0f;
        m2Polygon box = m2MakeBox(0.5f, 0.5f);
        NewPolygonShape(world, crate, &sd, &box);
        m2DistanceJointDef jd = m2DefaultDistanceJointDef();
        jd.bodyIdA = hook;
        jd.bodyIdB = crate;
        ropes[i] = NewDistanceJoint(world, &jd);
        m2Joint_SetBreakLimits(world, ropes[i], 30.0f, 0.0f);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot");

    int32_t stormStep = -1;
    for (int32_t i = 0; i < 60 && stormStep < 0; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2JointEvents events = m2World_GetJointEvents(world);
        if (events.breakCount > 0)
        {
            stormStep = i;
            CHECK(events.breakCount == 4, "the whole curtain goes at once");
            for (int32_t k = 1; k < events.breakCount; ++k)
            {
                CHECK(events.breakEvents[k - 1].jointId.index1 <
                          events.breakEvents[k].jointId.index1,
                      "break events arrive in canonical joint order");
            }
        }
    }
    CHECK(stormStep >= 0, "the storm came");

    CHECK(Restore(world, snap, size), "rollback before the storm");
    for (int32_t i = 0; i < 4; ++i)
    {
        CHECK(m2Joint_IsValid(world, ropes[i]), "restore resurrects the curtain");
    }
    int32_t stormAgain = -1;
    for (int32_t i = 0; i < 60 && stormAgain < 0; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        if (m2World_GetJointEvents(world).breakCount > 0)
        {
            stormAgain = i;
        }
    }
    CHECK(stormAgain == stormStep, "and the storm returns on the identical step");

    free(snap);
    m2DestroyWorld(world);
}

static void TestKinematicSensorHibernation(void)
{
    // A kinematic at rest inside a static sensor must not hold the
    // world out of hibernation; when it moves again, the exit is
    // witnessed on the sensor stream.
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
    m2BodyId sleeperBox = AddBox(world, 3.0, 0.5);

    m2BodyDef zd = m2DefaultBodyDef();
    zd.position = (m2Pos2){-3.0, 1.0};
    m2BodyId zoneBody = NewBody(world, &zd);
    m2ShapeDef zs = m2DefaultShapeDef();
    zs.isSensor = true;
    m2Polygon zone = m2MakeBox(1.5f, 1.5f);
    m2ShapeId zoneId = NewPolygonShape(world, zoneBody, &zs, &zone);

    m2BodyDef kd = m2DefaultBodyDef();
    kd.type = m2_kinematicBody;
    kd.position = (m2Pos2){-3.0, 1.0};
    m2BodyId guard = NewBody(world, &kd);
    m2ShapeDef gs = m2DefaultShapeDef();
    m2Circle body = {{0.0f, 0.0f}, 0.3f};
    NewCircleShape(world, guard, &gs, &body);

    for (int32_t i = 0; i < 250; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, sleeperBox), "the dynamic box sleeps");
    m2ShapeId inside[2];
    CHECK(m2Shape_GetSensorOverlaps(world, zoneId, inside, 2) == 1,
          "the resting guard still shows in the roll call");
    m2Profile profile = m2World_GetProfile(world);
    CHECK(profile.contactsMs == 0.0f && profile.solveMs == 0.0f,
          "a resting guard does not hold the world out of hibernation");

    m2Body_SetLinearVelocity(world, guard, (m2Vec2){4.0f, 0.0f});
    bool sawExit = false;
    for (int32_t i = 0; i < 60 && !sawExit; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        sawExit = m2World_GetSensorEvents(world).endCount > 0;
    }
    CHECK(sawExit, "the guard's exit is witnessed");

    m2DestroyWorld(world);
}

static void TestBulletThroughOneWayChain(void)
{
    // A one-way platform must be one-way for bullets too: from the
    // ghost side the bullet keeps its speed; from the solid side it
    // stops on the platform.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    gd.position = (m2Pos2){0.0, 2.0};
    m2BodyId platform = NewBody(world, &gd);
    m2Vec2 pts[5] = {{6.0f, 0.0f}, {3.0f, 0.0f}, {0.0f, 0.0f}, {-3.0f, 0.0f}, {-6.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 5; // right-to-left: solid side up
    m2ChainId shelfChain = NewChain(world, platform, &chain);
    CHECK(m2Chain_GetSegmentCount(world, shelfChain) == 2, "two live segments");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.isBullet = true;
    bd.position = (m2Pos2){0.5, -6.0};
    bd.linearVelocity = (m2Vec2){0.0f, 80.0f}; // straight up through the ghost side
    m2BodyId riser = NewBody(world, &bd);
    m2ShapeDef bs = m2DefaultShapeDef();
    m2Circle ball = {{0.0f, 0.0f}, 0.1f};
    NewCircleShape(world, riser, &bs, &ball);

    for (int32_t i = 0; i < 12; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, riser).y > 4.0,
          "the rising bullet passed the one-way platform");
    CHECK(m2Body_GetLinearVelocity(world, riser).y > 60.0f,
          "without losing its speed to the ghost");

    bd.position = (m2Pos2){-0.5, 10.0};
    bd.linearVelocity = (m2Vec2){0.0f, -80.0f};
    m2BodyId diver = NewBody(world, &bd);
    NewCircleShape(world, diver, &bs, &ball);
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, diver).y > 1.9, "the diving bullet stopped on the solid side");

    m2DestroyWorld(world);
}

// ROUND 7: the post-1.0 surface. Chain ids, the destroy-path wake
// law, and the reaction getters each get attacked where they would
// hurt most: precision, rollback, and slot reuse.

// The wake law must be surgical: destroying a chain wakes ITS riders
// and nobody else, even when the survivor sleeps on the same body.
static void TestChainSurgicalWake(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 pts[6] = {{6.0f, 0.0f}, {4.0f, 0.0f},  {2.0f, 0.0f},
                     {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId chainId = NewChain(world, ground, &chain);
    m2ShapeDef fs = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(2.0f, 0.5f);
    m2Polygon shifted = slab;
    for (int32_t v = 0; v < shifted.count; ++v)
    {
        shifted.vertices[v].x += 12.0f;
        shifted.vertices[v].y -= 0.5f;
    }
    NewPolygonShape(world, ground, &fs, &shifted);

    m2BodyId onChain = AddBox(world, 1.0, 0.45);
    m2BodyId onSlab = AddBox(world, 12.0, 0.45);
    for (int32_t i = 0; i < 240; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, onChain) && !m2Body_IsAwake(world, onSlab), "both sleep");

    DestroyChain(world, chainId);
    DestroyChain(world, chainId); // double destroy: generation miss, silent
    for (int32_t i = 0; i < 90; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Body_GetPosition(world, onChain).y < -2.0, "the chain rider wakes and falls");
    CHECK(!m2Body_IsAwake(world, onSlab), "the slab sleeper never stirs");
    CHECK(m2Body_GetPosition(world, onSlab).y > 0.0, "and stays put");
    m2DestroyWorld(world);
}

// Rollback must resurrect a destroyed chain: registry state is
// snapshot state, so the old id becomes valid again and the re-run
// destruction lands on identical bits.
static void TestChainRollbackResurrection(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 32;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 pts[6] = {{6.0f, 0.0f}, {4.0f, 0.0f},  {2.0f, 0.0f},
                     {0.0f, 0.0f}, {-2.0f, 0.0f}, {-4.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 6;
    m2ChainId chainId = NewChain(world, ground, &chain);
    AddBox(world, 1.0, 0.45);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot before the demolition");

    DestroyChain(world, chainId);
    uint64_t hashes[40];
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        hashes[i] = m2World_Hash(world);
    }
    CHECK(!m2Chain_IsValid(world, chainId), "gone after the demolition");

    CHECK(Restore(world, snap, size), "restore");
    CHECK(m2Chain_IsValid(world, chainId), "the rolled-back chain id lives again");
    CHECK(m2Chain_GetSegmentCount(world, chainId) == 3, "with all its segments");
    CastResult ray =
        RayClosest(world, (m2Pos2){1.0, 2.0}, (m2Vec2){0.0f, -4.0f}, m2DefaultQueryFilter());
    CHECK(ray.hit, "segments answer rays again");

    DestroyChain(world, chainId);
    for (int32_t i = 0; i < 40; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == hashes[i], "re-run demolition is bit-exact");
    }
    free(snap);
    m2DestroyWorld(world);
}

// Reaction readings are simulation-grade state: after a rollback the
// getter must return the restored step's load to the last bit.
static void TestReactionRollbackReading(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    def.jointCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef ad = m2DefaultBodyDef();
    ad.position = (m2Pos2){0.0, 6.0};
    m2BodyId hook = NewBody(world, &ad);
    m2BodyId bob = AddBox(world, 2.0, 6.0); // horizontal start: it swings
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = hook;
    jd.bodyIdB = bob;
    m2JointId rope = NewDistanceJoint(world, &jd);
    for (int32_t i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    float midSwing = m2Joint_GetReactionForce(world, rope);
    CHECK(midSwing > 0.0f, "a swinging rope carries load");

    int32_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    CHECK(Snapshot(world, snap, size) == size, "snapshot mid-swing");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(Restore(world, snap, size), "restore");
    CHECK(m2Joint_GetReactionForce(world, rope) == midSwing,
          "the restored reading matches to the bit");
    free(snap);
    m2DestroyWorld(world);
}

// Every joint type's reaction path answers under a real load: the
// prismatic limit lock, the wheel spring, the revolute holding motor.
static void TestReactionAllTypes(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.jointCapacity = 8;
    m2World* world = NewWorld(&def);

    // Prismatic elevator: vertical axis, limits locked at zero, so the
    // limit rows carry the cabin's weight.
    m2BodyDef pd = m2DefaultBodyDef();
    pd.position = (m2Pos2){0.0, 4.0};
    m2BodyId post = NewBody(world, &pd);
    m2BodyId cabin = AddBox(world, 0.0, 4.0);
    m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
    pj.bodyIdA = post;
    pj.bodyIdB = cabin;
    pj.localAxisA = (m2Vec2){0.0f, 1.0f};
    pj.enableLimit = true;
    pj.lowerTranslation = 0.0f;
    pj.upperTranslation = 0.0f;
    m2JointId lift = NewPrismaticJoint(world, &pj);

    // Wheel on a spring: the suspension row carries the hanging wheel.
    m2BodyDef cd = m2DefaultBodyDef();
    cd.position = (m2Pos2){6.0, 4.0};
    m2BodyId strut = NewBody(world, &cd);
    m2BodyId hub = AddBox(world, 6.0, 4.0);
    m2WheelJointDef wj = m2DefaultWheelJointDef();
    wj.bodyIdA = strut;
    wj.bodyIdB = hub;
    wj.localAxisA = (m2Vec2){0.0f, 1.0f};
    wj.enableSpring = true;
    m2JointId spring = NewWheelJoint(world, &wj);

    // Revolute holding motor: a level arm pinned at its end, the motor
    // (speed zero, big budget) holds it against gravity.
    m2BodyDef hd = m2DefaultBodyDef();
    hd.position = (m2Pos2){12.0, 4.0};
    m2BodyId pin = NewBody(world, &hd);
    m2BodyDef armDef = m2DefaultBodyDef();
    armDef.type = m2_dynamicBody;
    armDef.position = (m2Pos2){13.0, 4.0};
    m2BodyId arm = NewBody(world, &armDef);
    m2ShapeDef as = m2DefaultShapeDef();
    as.density = 2.0f;
    m2Polygon bar = m2MakeBox(1.0f, 0.1f);
    NewPolygonShape(world, arm, &as, &bar);
    m2RevoluteJointDef rj = m2DefaultRevoluteJointDef();
    rj.bodyIdA = pin;
    rj.bodyIdB = arm;
    rj.localAnchorB = (m2Vec2){-1.0f, 0.0f};
    rj.enableMotor = true;
    rj.motorSpeed = 0.0f;
    rj.maxMotorTorque = 100.0f;
    m2JointId shoulder = NewRevoluteJoint(world, &rj);

    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(m2Joint_GetReactionForce(world, lift) > 1.0f, "the prismatic lock carries the cabin");
    CHECK(m2Joint_GetReactionForce(world, spring) > 1.0f, "the wheel spring carries the hub");
    CHECK(m2Joint_GetReactionTorque(world, shoulder) > 1.0f, "the holding motor reads torque");
    CHECK(m2Joint_GetReactionForce(world, shoulder) > 1.0f, "and the pin carries the arm");

    m2JointId never = {5, 999};
    CHECK(m2Joint_GetReactionForce(world, never) == 0.0f, "a bogus id reads zero force");
    CHECK(m2Joint_GetReactionTorque(world, never) == 0.0f, "and zero torque");
    m2DestroyWorld(world);
}

// ---- The velocity cap, the belt wake, the SIMD guard, the centroid
// inertia and the fresh prismatic mass each get an adversary. Pass/fail
// attacks; they do not feed the gated chaos hash. ----

static void TestVelocityCapUnderRollback(void)
{
    // The velocity cap bounds an absurd speed, and does it
    // bit-for-bit under rollback and across worker counts.
    uint64_t caps[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef def = m2DefaultWorldDef();
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&def, &s_twinWorkers[wc]);
        m2World* world = NewWorld(&def);
        m2BodyId body = AddBox(world, 0.0, 100.0);
        m2Body_SetLinearVelocity(world, body, (m2Vec2){70000.0f, 4000.0f});
        m2Body_SetAngularVelocity(world, body, 9000.0f);
        int32_t size = m2World_SnapshotSize(world);
        void* snap = malloc((size_t)size);
        Snapshot(world, snap, size);
        m2World_Step(world, 1.0f / 60.0f, 4);
        m2Vec2 v = m2Body_GetLinearVelocity(world, body);
        float av = m2Body_GetAngularVelocity(world, body);
        CHECK(v.x * v.x + v.y * v.y < 460.0f * 460.0f, "the cap bounds an absurd linear speed");
        CHECK(av < 200.0f && av > -200.0f, "the cap bounds an absurd spin");
        CHECK(v.x == v.x && v.y == v.y && av == av, "the capped step leaves no NaN");
        uint64_t after = m2World_Hash(world);
        CHECK(Restore(world, snap, size), "the capped world restores");
        m2World_Step(world, 1.0f / 60.0f, 4);
        CHECK(m2World_Hash(world) == after, "the capped step replays bit for bit");
        caps[wc] = after;
        free(snap);
        m2DestroyWorld(world);
    }
    CHECK(caps[0] == caps[1], "the cap is worker-count deterministic");
}

static void TestBeltWakeReplayStorm(void)
{
    // A tangent-speed change wakes riders, and that wake now rides the
    // journaled channel so a replay reproduces it. A rider left asleep on
    // replay would diverge.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 8;
    m2World* world = NewWorld(&def);
    m2BodyDef fd = m2DefaultBodyDef();
    fd.position = (m2Pos2){0.0, 0.0};
    m2BodyId belt = NewBody(world, &fd);
    m2ShapeDef bsd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(4.0f, 0.25f);
    m2ShapeId beltShape = NewPolygonShape(world, belt, &bsd, &slab);
    m2Shape_SetTangentSpeed(world, beltShape, 0.0f);
    m2BodyId rider = AddBox(world, 0.0, 0.65);
    for (int32_t i = 0; i < 400 && m2Body_IsAwake(world, rider); ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    CHECK(!m2Body_IsAwake(world, rider), "the rider sleeps on the stopped belt");
    int32_t cap = 1 << 18;
    void* tape = malloc((size_t)cap);
    CHECK(StartJournal(world, tape, cap), "the belt tape starts");
    m2Shape_SetTangentSpeed(world, beltShape, 3.0f);
    CHECK(m2Body_IsAwake(world, rider), "starting the belt wakes the sleeping rider");
    for (int32_t i = 0; i < 30; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int32_t used = StopJournal(world);
    CHECK(used > 0, "the belt tape closed");
    uint64_t recorded = m2World_Hash(world);
    m2World* fresh = NewWorld(&def);
    CHECK(ReplayJournal(fresh, tape, used), "the belt tape replays");
    CHECK(m2World_Hash(fresh) == recorded, "the belt-wake side effect replays onto the same bits");
    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(tape);
}

static void TestOffCenterInertiaPrismatic(void)
{
    // An off-center-COM body (its shape sits well off the body origin, so
    // the centroid inertia is what governs its spin) driven
    // on a stressed prismatic (with the per-substep mass refresh):
    // finite, rollback bit-exact, and worker-count deterministic.
    uint64_t hashes[2];
    for (int32_t wc = 0; wc < 2; ++wc)
    {
        m2WorldDef def = m2DefaultWorldDef();
        static int32_t s_twinWorkers[2] = {1, 4};
        TpUseWorkers(&def, &s_twinWorkers[wc]);
        def.bodyCapacity = 8;
        def.jointCapacity = 4;
        m2World* world = NewWorld(&def);
        m2ShapeDef sd = m2DefaultShapeDef();
        m2BodyDef kd = m2DefaultBodyDef();
        kd.type = m2_kinematicBody;
        kd.angularVelocity = 6.0f;
        m2BodyId base = NewBody(world, &kd);
        m2Polygon bx = m2MakeBox(0.3f, 0.3f);
        NewPolygonShape(world, base, &sd, &bx);
        m2BodyDef dd = m2DefaultBodyDef();
        dd.type = m2_dynamicBody;
        dd.position = (m2Pos2){1.5, 0.0};
        m2BodyId body = NewBody(world, &dd);
        m2Circle off = {{3.0f, 0.0f}, 0.4f};
        NewCircleShape(world, body, &sd, &off);
        m2PrismaticJointDef pj = m2DefaultPrismaticJointDef();
        pj.bodyIdA = base;
        pj.bodyIdB = body;
        pj.localAxisA = (m2Vec2){1.0f, 0.0f};
        pj.enableLimit = true;
        pj.lowerTranslation = -4.0f;
        pj.upperTranslation = 4.0f;
        NewPrismaticJoint(world, &pj);
        m2PrismaticJointDef pj2 = pj;
        pj2.localAxisA = (m2Vec2){0.0f, 1.0f};
        NewPrismaticJoint(world, &pj2);
        int32_t size = 0;
        void* snap = NULL;
        for (int32_t i = 0; i < 120; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
            m2Vec2 v = m2Body_GetLinearVelocity(world, body);
            float av = m2Body_GetAngularVelocity(world, body);
            CHECK(v.x == v.x && v.y == v.y && av == av,
                  "the off-center stressed body stays finite");
            if (i == 60)
            {
                size = m2World_SnapshotSize(world);
                snap = malloc((size_t)size);
                Snapshot(world, snap, size);
            }
        }
        uint64_t after = m2World_Hash(world);
        CHECK(Restore(world, snap, size), "the off-center stressed body restores");
        for (int32_t i = 61; i < 120; ++i)
        {
            m2World_Step(world, 1.0f / 60.0f, 4);
        }
        CHECK(m2World_Hash(world) == after, "the off-center stressed body replays bit for bit");
        hashes[wc] = after;
        free(snap);
        m2DestroyWorld(world);
    }
    CHECK(hashes[0] == hashes[1], "the off-center stressed body is worker-count deterministic");
}

// A chain's points are checked before anything is built: a NaN point
// or a degenerate segment is invalid input, and a full shape pool is a
// capacity refusal that leaves no trace in the chain ids.
static void TestChainRefusals(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 4;
    def.shapeCapacity = 4;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2Vec2 pts[4] = {{3.0f, 0.0f}, {1.0f, 0.0f}, {-1.0f, 0.0f}, {-3.0f, 0.0f}};
    m2ChainDef chain = m2DefaultChainDef();
    chain.points = pts;
    chain.count = 4;
    m2ChainId made = m2_nullChainId;
    pts[1].x = NAN;
    CHECK(m2CreateChain(world, ground, &chain, &made) == m2_errorInvalid,
          "a NaN chain point refuses as invalid input");
    CHECK(made.index1 == 0, "and leaves the null id");
    pts[1] = pts[2];
    CHECK(m2CreateChain(world, ground, &chain, &made) == m2_errorInvalid,
          "a zero-length segment refuses as invalid input");
    pts[1] = (m2Vec2){1.0f, 0.0f};
    m2Circle ball = {{0.0f, 0.0f}, 0.5f};
    m2ShapeDef sd = m2DefaultShapeDef();
    for (int32_t i = 0; i < 4; ++i)
    {
        NewCircleShape(world, ground, &sd, &ball);
    }
    CHECK(m2CreateChain(world, ground, &chain, &made) == m2_errorCapacity,
          "a full shape pool refuses a chain as a capacity limit");
    sd.friction = INFINITY;
    m2DestroyWorld(world);

    world = NewWorld(&def);
    ground = NewBody(world, &gd);
    m2ShapeId shape = m2_nullShapeId;
    CHECK(m2CreateCircleShape(world, ground, &sd, &ball, &shape) == m2_errorLimit,
          "infinite friction is past M2_COEFFICIENT_LIMIT");
    sd = m2DefaultShapeDef();
    sd.tangentSpeed = NAN;
    CHECK(m2CreateCircleShape(world, ground, &sd, &ball, &shape) == m2_errorInvalid,
          "a NaN belt speed refuses as invalid input");
    m2DestroyWorld(world);
}

int main(void)
{
    TestChainRefusals();
    TestJournalDefenses();
    TestOtherReleasesAreRefused();
    TestParameterChannels();
    TestHostileQueries();
    TestHostileJointDefs();
    TestHostileSnapshotWords();
    TestQueryEdges();
    TestMultiWorldIsolation();
    TestWakeAfterFrozenEra();
    TestTeleportOntoSleepers();
    TestSetTypeEdges();
    TestLoopChainBowl();
    TestBreakStorm();
    TestKinematicSensorHibernation();
    TestBulletThroughOneWayChain();
    TestChainSurgicalWake();
    TestChainRollbackResurrection();
    TestReactionRollbackReading();
    TestReactionAllTypes();

    TestParticleStaleIds();
    TestJellyRollbackStorm();
    TestJellyJournalStorm();
    TestHostileFills();
    TestMachineryResurrection();
    TestRatchetMotorWar();
    TestDecomposeAdversaries();

    // Velocity cap, belt wake, SIMD guard, centroid inertia, prismatic mass.
    TestVelocityCapUnderRollback();
    TestBeltWakeReplayStorm();
    TestOffCenterInertiaPrismatic();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
