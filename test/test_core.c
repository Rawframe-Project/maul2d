// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core: the elementary math against libm, the pinned min, max and abs
// semantics, the SIMD backend report, the version and result names, the
// world def checks and the world's allocator.

#include "helpers.h"
#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- 1. Accuracy against libm ------------------------------------------------

static void TestAccuracy(void)
{
    double maxSinErr = 0.0;
    double maxCosErr = 0.0;
    double maxAtanErr = 0.0;

    for (int i = 0; i <= 200000; ++i)
    {
        float angle = -10.0f + 20.0f * (float)i / 200000.0f;
        m2Rot q = m2MakeRot(angle);
        double c = cos((double)angle);
        double s = sin((double)angle);
        double cosErr = fabs((double)q.c - c);
        double sinErr = fabs((double)q.s - s);
        if (cosErr > maxCosErr)
        {
            maxCosErr = cosErr;
        }
        if (sinErr > maxSinErr)
        {
            maxSinErr = sinErr;
        }
        CHECK(m2IsNormalizedRot(q), "m2MakeRot must return a unit rotation");
    }

    for (int iy = -50; iy <= 50; ++iy)
    {
        for (int ix = -50; ix <= 50; ++ix)
        {
            if (ix == 0 && iy == 0)
            {
                continue;
            }
            float y = 0.13f * (float)iy;
            float x = 0.17f * (float)ix;
            double err = fabs((double)m2Atan2(y, x) - atan2((double)y, (double)x));
            if (err > maxAtanErr)
            {
                maxAtanErr = err;
            }
        }
    }

    // Angles past [-pi, pi] are unwound first: the rotation still points
    // where the angle does.
    for (int i = 0; i <= 40000; ++i)
    {
        float angle = -200.0f + 400.0f * (float)i / 40000.0f;
        m2Rot q = m2MakeRot(angle);
        CHECK(fabs((double)q.c - cos((double)angle)) < 1.0e-3 &&
                  fabs((double)q.s - sin((double)angle)) < 1.0e-3,
              "m2MakeRot unwinds a wide angle to the same direction");
    }
    // Out-of-range input: the deterministic fallback is finite and unit.
    m2Rot big = m2MakeRot(1.0e30f);
    CHECK(big.c == big.c && big.s == big.s, "m2MakeRot must never return NaN");
    CHECK(m2IsNormalizedRot(big), "m2MakeRot fallback must be a unit rotation");
    m2Rot degenerate = m2NormalizeRot((m2Rot){0.0f, 0.0f});
    CHECK(degenerate.c == 1.0f && degenerate.s == 0.0f, "degenerate normalize must be identity");

    // The series are exact to below float resolution; what remains is the
    // rounding of the angle and of the result (about 4e-7 here).
    CHECK(maxSinErr < 1.0e-6, "m2MakeRot sine outside error bound");
    CHECK(maxCosErr < 1.0e-6, "m2MakeRot cosine outside error bound");
    CHECK(maxAtanErr < 1.0e-6, "m2Atan2 outside error bound");
    CHECK(m2Atan2(0.0f, 0.0f) == 0.0f, "m2Atan2(0,0) must be 0, not NaN");

    printf("accuracy: sin %.2e cos %.2e atan2 %.2e\n", maxSinErr, maxCosErr, maxAtanErr);
}

// A finite nonzero rotation keeps its direction however far from unit
// length it is, and one near unit length normalizes with the plain
// arithmetic, bit for bit, so no golden value depends on the rescue.
static void TestNormalizeKeepsDirection(void)
{
    const float half = 0.70710678f;
    const m2Rot cases[][2] = {
        {{1.0e20f, 1.0e20f}, {half, half}},     {{0.0f, 3.0e19f}, {0.0f, 1.0f}},
        {{FLT_MAX, -FLT_MAX}, {half, -half}},   {{3.0e30f, 4.0e30f}, {0.6f, 0.8f}},
        {{0.0f, 1.0e-30f}, {0.0f, 1.0f}},       {{-1.0e-25f, 1.0e-25f}, {-half, half}},
        {{3.0e-30f, -4.0e-30f}, {0.6f, -0.8f}}, {{-1.0e-45f, 0.0f}, {-1.0f, 0.0f}},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        m2Rot q = m2NormalizeRot(cases[i][0]);
        CHECK(fabsf(q.c - cases[i][1].c) < 1.0e-6f && fabsf(q.s - cases[i][1].s) < 1.0e-6f,
              "a rotation far from unit length keeps its direction");
        CHECK(m2IsNormalizedRot(q), "and comes back unit length");
    }

    const m2Rot degenerate[] = {
        {0.0f, 0.0f}, {-0.0f, -0.0f}, {INFINITY, 0.0f}, {1.0f, -INFINITY}, {NAN, 1.0f}};
    for (size_t i = 0; i < sizeof(degenerate) / sizeof(degenerate[0]); ++i)
    {
        m2Rot q = m2NormalizeRot(degenerate[i]);
        CHECK(q.c == 1.0f && q.s == 0.0f, "a zero or non-finite rotation becomes the identity");
    }

    // Lengths from 2^-40 to 2^40 in every direction: the plain formula.
    uint32_t seed = 12345u;
    int32_t differ = 0;
    for (int32_t i = 0; i < 100000; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        float angle = ((float)(seed >> 8) / 16777216.0f) * 2.0f * M2_PI - M2_PI;
        seed = seed * 1664525u + 1013904223u;
        float scale = ldexpf(1.0f + (float)(seed >> 9) / 8388608.0f, (int)(seed % 81u) - 40);
        m2Rot unit = m2MakeRot(angle);
        m2Rot q = i % 2 == 0 ? unit : (m2Rot){unit.c * scale, unit.s * scale};
        float mag = sqrtf(q.c * q.c + q.s * q.s);
        float invMag = 1.0f / mag;
        m2Rot plain = {q.c * invMag, q.s * invMag};
        m2Rot got = m2NormalizeRot(q);
        differ += memcmp(&plain, &got, sizeof(got)) != 0 ? 1 : 0;
    }
    CHECK(differ == 0, "lengths near unit normalize bit for bit as before");
}

// Composing two rotations adds their angles and comes back unit length,
// whatever the inputs' lengths.
static void TestMulRotAddsAngles(void)
{
    uint32_t seed = 54321u;
    float worst = 0.0f;
    for (int32_t i = 0; i < 10000; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        float a = ((float)(seed >> 8) / 16777216.0f) * 2.0f * M2_PI - M2_PI;
        seed = seed * 1664525u + 1013904223u;
        float b = ((float)(seed >> 8) / 16777216.0f) * 2.0f * M2_PI - M2_PI;
        m2Rot q = m2MulRot(m2MakeRot(a), m2MakeRot(b));
        CHECK(m2IsNormalizedRot(q), "a composition is unit length");
        double angle = atan2((double)q.s, (double)q.c);
        double turn = 2.0 * (double)M2_PI;
        double want = remainder((double)a + (double)b, turn);
        worst = fmaxf(worst, (float)fabs(remainder(angle - want, turn)));
    }
    CHECK(worst < 2.0e-6f, "the angles add");
    m2Rot long3 = m2MulRot((m2Rot){3.0f, 4.0f}, (m2Rot){1.0f, 0.0f});
    CHECK(fabsf(long3.c - 0.6f) < 1.0e-6f && fabsf(long3.s - 0.8f) < 1.0e-6f,
          "a long input keeps its direction");
}

// --- The world's allocator ---------------------------------------------------

typedef struct Ledger
{
    int32_t live;
    int32_t total;
    int64_t liveBytes;
    int32_t failAfter; // refuse every allocation past this many; -1 never
    int32_t badAlignment;
    int32_t asked; // every call, refused or not
} Ledger;

static void* LedgerAlloc(size_t size, size_t alignment, void* context)
{
    Ledger* ledger = (Ledger*)context;
    ledger->asked += 1;
    if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
        ledger->badAlignment += 1;
    }
    if (ledger->failAfter >= 0 && ledger->total >= ledger->failAfter)
    {
        return NULL;
    }
    ledger->live += 1;
    ledger->total += 1;
    ledger->liveBytes += (int64_t)size;
    return malloc(size);
}

static void LedgerFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    Ledger* ledger = (Ledger*)context;
    ledger->live -= 1;
    ledger->liveBytes -= (int64_t)size;
    free(memory);
}

static m2WorldDef LedgerDef(Ledger* ledger)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 16;
    def.shapeCapacity = 16;
    def.allocator.alloc = LedgerAlloc;
    def.allocator.free = LedgerFree;
    def.allocator.context = ledger;
    return def;
}

static void TestWorldAllocator(void)
{
    // Every allocation of a world goes through its own allocator, each free
    // hands back the size it was given, and a world's whole life balances
    // to zero. Two worlds keep two allocators: there is no process-wide one.
    Ledger first = {0, 0, 0, -1, 0, 0};
    Ledger second = {0, 0, 0, -1, 0, 0};
    m2WorldDef firstDef = LedgerDef(&first);
    m2WorldDef secondDef = LedgerDef(&second);
    m2World* a = NewWorld(&firstDef);
    m2World* b = NewWorld(&secondDef);
    CHECK(a != NULL && b != NULL, "both worlds create");
    CHECK(first.total > 10 && second.total == first.total,
          "each world allocates through its own allocator");

    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 2.0};
    m2BodyId body = NewBody(a, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.4f};
    NewCircleShape(a, body, &sd, &circle);
    int32_t afterCreate = first.total;
    for (int32_t i = 0; i < 30; ++i)
    {
        Step(a, 1.0f / 60.0f, 4);
    }
    CHECK(first.total == afterCreate, "steps allocate nothing");

    m2DestroyWorld(a);
    CHECK(first.live == 0 && first.liveBytes == 0, "destroying the world frees every block");
    CHECK(second.live > 0, "and leaves the other world's memory alone");
    m2DestroyWorld(b);
    CHECK(second.live == 0 && second.liveBytes == 0, "the other world balances too");
    CHECK(first.badAlignment == 0 && second.badAlignment == 0, "every alignment is a power of two");

    // An allocator with one function set is refused before it is used.
    m2WorldDef half = LedgerDef(&first);
    half.allocator.free = NULL;
    m2World* refused = (m2World*)&first;
    CHECK(m2CreateWorld(&half, &refused) == m2_errorInvalid, "a half allocator refuses");
    CHECK(refused == NULL, "and leaves the null world");
}

// A world being destroyed, read back by the free hook at one of its frees.
typedef struct Reader
{
    m2World* world;
    int32_t frees;
    int32_t readAt; // the free that reads the world; 0 never
    int32_t wrong;  // reads that did not find an empty world
    m2BodyId body;
    m2ShapeId shape;
    m2JointId joint;
    m2ChainId chain;
    m2ParticleId particle;
    m2WaterVolumeId volume;
    int32_t drawn;
} Reader;

static void ReaderPolygon(const m2Vec2* localVertices, int32_t count, m2Pos2 origin, m2Rot rotation,
                          uint32_t color, void* context)
{
    (void)localVertices, (void)count, (void)origin, (void)rotation, (void)color;
    ((Reader*)context)->drawn += 1;
}

static void ReaderCircle(m2Pos2 center, float radius, m2Rot rotation, uint32_t color, void* context)
{
    (void)center, (void)radius, (void)rotation, (void)color;
    ((Reader*)context)->drawn += 1;
}

static void ReaderCapsule(m2Pos2 p1, m2Pos2 p2, float radius, uint32_t color, void* context)
{
    (void)p1, (void)p2, (void)radius, (void)color;
    ((Reader*)context)->drawn += 1;
}

static void ReaderSegment(m2Pos2 p1, m2Pos2 p2, uint32_t color, void* context)
{
    (void)p1, (void)p2, (void)color;
    ((Reader*)context)->drawn += 1;
}

static void ReaderPoint(m2Pos2 p, float size, uint32_t color, void* context)
{
    (void)p, (void)size, (void)color;
    ((Reader*)context)->drawn += 1;
}

// Every kind of reader on the world the hook is freeing: each id is stale
// and each list empty.
static void ReadBack(Reader* r)
{
    const m2World* w = r->world;
    bool stale =
        !m2Body_IsValid(w, r->body) && m2Body_GetMass(w, r->body) == 0.0f &&
        !m2Shape_IsValid(w, r->shape) && m2Shape_GetAabb(w, r->shape).upperBound.x == 0.0 &&
        !m2Joint_IsValid(w, r->joint) && m2Joint_GetReactionForce(w, r->joint) == 0.0f &&
        m2DistanceJoint_GetLength(w, r->joint) == 0.0f && !m2Chain_IsValid(w, r->chain) &&
        !m2Particle_IsValid(w, r->particle) && m2Particle_GetPosition(w, r->particle).x == 0.0 &&
        !m2WaterVolume_IsValid(w, r->volume) && m2WaterVolume_GetSurface(w, r->volume) == 0.0;
    m2Counters counters = m2World_GetCounters(w);
    bool empty = counters.bodyCount == 0 && counters.shapeCount == 0 && counters.jointCount == 0 &&
                 counters.contactCount == 0 && m2World_GetParticleCount(w) == 0 &&
                 m2World_GetKineticEnergy(w) == 0.0 &&
                 m2World_GetContactEvents(w).beginCount == 0 &&
                 m2World_GetContactData(w, NULL, 0) == 0;
    m2QueryFilter filter = m2DefaultQueryFilter();
    m2WorldAabb box = {{-100.0, -100.0}, {100.0, 100.0}};
    m2CastHit hit;
    m2CastHit hits[4];
    m2ShapeId ids[4];
    m2ParticleId particles[4];
    m2MoverPlane planes[4];
    m2Capsule mover = {{0.0f, -0.5f}, {0.0f, 0.5f}, 0.25f};
    m2Transform origin = {{0.0, 1.0}, {1.0f, 0.0f}};
    empty =
        empty && m2World_OverlapAabb(w, box, filter, ids, 4) == 0 &&
        !m2World_CastRayClosest(w, (m2Pos2){-50.0, 1.0}, (m2Vec2){100.0f, 0.0f}, filter, &hit) &&
        m2World_CastRayAll(w, (m2Pos2){-50.0, 1.0}, (m2Vec2){100.0f, 0.0f}, filter, hits, 4) == 0 &&
        m2World_OverlapParticlesAabb(w, box, particles, 4) == 0 &&
        m2World_CollideMover(w, &mover, origin, filter, planes, 4) == 0;
    // Readers that answer about the whole world: none may touch a pool.
    (void)m2World_Hash(w);
    (void)m2World_Validate(w);
    (void)m2World_GetSnapshotSize(w);
    (void)m2World_GetJournalBaseSize(w);
    (void)m2World_GetMemoryUsage(w);
    m2DebugDraw draw = {ReaderPolygon, ReaderCircle, ReaderCapsule, ReaderSegment,
                        ReaderPoint,   true,         true,          true,
                        true,          true,         1.0f,          r};
    r->drawn = 0;
    m2World_Draw(w, &draw);
    empty = empty && r->drawn == 0;
    r->wrong += stale && empty ? 0 : 1;
}

static void ReaderFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size, (void)alignment;
    Reader* r = (Reader*)context;
    r->frees += 1;
    if (r->frees == r->readAt)
    {
        ReadBack(r);
    }
    free(memory);
}

static void* ReaderAlloc(size_t size, size_t alignment, void* context)
{
    (void)alignment, (void)context;
    return malloc(size);
}

// One object of every kind, stepped once, in a world whose frees r counts.
static void BuildReadWorld(Reader* r)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 8;
    def.shapeCapacity = 16;
    def.particleCapacity = 8;
    def.waterVolumeCapacity = 1;
    def.allocator.alloc = ReaderAlloc;
    def.allocator.free = ReaderFree;
    def.allocator.context = r;
    r->world = NewWorld(&def);
    m2BodyDef bd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(r->world, &bd);
    m2Vec2 points[4] = {{-5.0f, 0.0f}, {-2.0f, 0.0f}, {2.0f, 0.0f}, {5.0f, 0.0f}};
    m2ChainDef cd = m2DefaultChainDef();
    cd.points = points;
    cd.count = 4;
    r->chain = NewChain(r->world, ground, &cd);
    bd.type = m2_dynamicBody;
    bd.position = (m2Pos2){0.0, 0.5};
    r->body = NewBody(r->world, &bd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Circle circle = {{0.0f, 0.0f}, 0.5f};
    r->shape = NewCircleShape(r->world, r->body, &sd, &circle);
    m2DistanceJointDef jd = m2DefaultDistanceJointDef();
    jd.bodyIdA = ground;
    jd.bodyIdB = r->body;
    CHECK(m2CreateDistanceJoint(r->world, &jd, &r->joint) == m2_success, "the joint creates");
    r->particle = EmitParticle(r->world, (m2Pos2){1.0, 2.0}, (m2Vec2){0.0f, 0.0f}, 0);
    m2WaterVolumeDef vd = m2DefaultWaterVolumeDef();
    vd.bounds = (m2WorldAabb){{-5.0, -1.0}, {5.0, 1.0}};
    vd.surface = 0.5;
    r->volume = NewWaterVolume(r->world, &vd);
    Step(r->world, 1.0f / 60.0f, 4);
}

static void TestDestroyedWorldReadsEmpty(void)
{
    // A free hook is host code the world runs, and it may read the world
    // back while m2DestroyWorld frees it: at every free, every reader
    // finds an empty world, every id stale, and touches no freed pool.
    Reader count = {0};
    BuildReadWorld(&count);
    m2DestroyWorld(count.world);
    CHECK(count.frees > 10, "the world frees its pools one by one");
    int32_t wrong = 0;
    for (int32_t at = 1; at <= count.frees; ++at)
    {
        Reader r = {0};
        r.readAt = at;
        BuildReadWorld(&r);
        m2DestroyWorld(r.world);
        wrong += r.wrong;
        CHECK(r.frees == count.frees, "every world frees the same blocks");
    }
    CHECK(wrong == 0, "at every free the world reads empty");
}

// The snapshot bytes of a world of shapeCapacity shapes and one body and
// one joint.
static int64_t ShapeWorldSnapshot(int32_t shapeCapacity)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 1;
    def.shapeCapacity = shapeCapacity;
    def.jointCapacity = 1;
    m2World* world = NewWorld(&def);
    int64_t size = m2World_GetSnapshotSize(world);
    m2DestroyWorld(world);
    return size;
}

static void TestWorldSnapshotBound(void)
{
    // A world's snapshot grows by the same bytes per shape slot, so the
    // shape capacity that takes it past a given size follows from two
    // small worlds.
    int64_t perShape = (ShapeWorldSnapshot(2000) - ShapeWorldSnapshot(1000)) / 1000;
    int64_t fixed = ShapeWorldSnapshot(1000) - 1000 * perShape;
    CHECK(perShape > 0 && ShapeWorldSnapshot(3000) == fixed + 3000 * perShape,
          "a world's snapshot grows by the same bytes per shape slot");

    // An allocator that refuses everything shows how far creation got: a
    // def whose snapshot passes what an int32_t counts gets past the size
    // check and is refused by the allocator.
    Ledger ledger = {0, 0, 0, 0, 0, 0};
    m2WorldDef def = LedgerDef(&ledger);
    def.bodyCapacity = 1;
    def.jointCapacity = 1;
    def.shapeCapacity = (int32_t)((INT32_MAX - fixed) / perShape) + 1;
    m2World* world = (m2World*)&ledger;
    CHECK(m2CreateWorld(&def, &world) == m2_errorCapacity && world == NULL && ledger.asked > 0,
          "a world whose snapshot passes INT32_MAX bytes goes on to allocate");
    CHECK(ledger.live == 0, "and the refused allocations leave nothing behind");
#if SIZE_MAX < INT64_MAX
    // Where a size_t is 32 bits, a snapshot it cannot hold is refused
    // before anything is asked for.
    ledger.asked = 0;
    def.shapeCapacity = (int32_t)(((int64_t)SIZE_MAX - fixed) / perShape) + 1;
    CHECK(m2CreateWorld(&def, &world) == m2_errorCapacity && world == NULL && ledger.asked == 0,
          "a world whose snapshot a size_t cannot hold is refused before it allocates");
#endif
}

// --- World defs, versions and results ------------------------------------------

static void TestWorldDefChecks(void)
{
    m2World* world = NULL;
    CHECK(m2CreateWorld(NULL, &world) == m2_errorInvalid, "a null def refuses");
    m2WorldDef def = m2DefaultWorldDef();
    CHECK(def.version == M2_ABI_VERSION, "the default def carries the headers' version");
    CHECK((M2_ABI_VERSION & 0x80000000u) != 0, "the version's top bit is set");
    CHECK(m2CreateWorld(&def, NULL) == m2_errorInvalid, "a null out pointer refuses");

    m2WorldDef stale = def;
    stale.version = M2_ABI_VERSION ^ 1u;
    world = (m2World*)&def;
    CHECK(m2CreateWorld(&stale, &world) == m2_errorVersion, "another ABI version refuses");
    CHECK(world == NULL, "and leaves the null world");
    stale.version = 0;
    CHECK(m2CreateWorld(&stale, &world) == m2_errorVersion, "a zeroed version refuses");

    // The cookie is read before the version.
    m2WorldDef zeroed;
    memset(&zeroed, 0, sizeof(zeroed));
    CHECK(m2CreateWorld(&zeroed, &world) == m2_errorInvalid, "a zeroed def refuses as invalid");
    m2WorldDef values = m2DefaultWorldDefValues();
    CHECK(values.version == 0, "the library's defaults leave the version to the program");
    CHECK(m2CreateWorld(&values, &world) == m2_errorVersion, "and are refused unstamped");

    m2WorldDef empty = def;
    empty.bodyCapacity = 0;
    CHECK(m2CreateWorld(&empty, &world) == m2_errorInvalid, "a zero capacity refuses");
    CHECK(m2CreateWorld(&def, &world) == m2_success && world != NULL, "the default def creates");
    CHECK(m2World_GetMemoryUsage(world).persistentBytes > (int64_t)(32 * sizeof(void*)),
          "a world weighs something");
    m2DestroyWorld(world);
}

static void TestVersionAndResultNames(void)
{
    m2Version version = m2GetVersion();
    CHECK(version.major == M2_VERSION_MAJOR && version.minor == M2_VERSION_MINOR &&
              version.patch == M2_VERSION_PATCH,
          "the linked version is the headers' version");
    CHECK(strcmp(m2GetResultName(m2_success), "m2_success") == 0, "success has its name");
    CHECK(strcmp(m2GetResultName(m2_errorInvalid), "m2_errorInvalid") == 0, "invalid has its name");
    CHECK(strcmp(m2GetResultName(m2_errorCapacity), "m2_errorCapacity") == 0,
          "capacity has its name");
    CHECK(strcmp(m2GetResultName(m2_errorStale), "m2_errorStale") == 0, "stale has its name");
    CHECK(strcmp(m2GetResultName(m2_errorUnsupported), "m2_errorUnsupported") == 0,
          "unsupported has its name");
    CHECK(strcmp(m2GetResultName(m2_errorState), "m2_errorState") == 0, "state has its name");
    CHECK(strcmp(m2GetResultName(m2_errorVersion), "m2_errorVersion") == 0, "version has its name");
    CHECK(strcmp(m2GetResultName(m2_errorFormat), "m2_errorFormat") == 0, "format has its name");
    CHECK(strcmp(m2GetResultName(m2_errorLimit), "m2_errorLimit") == 0, "limit has its name");
    CHECK(strcmp(m2GetResultName(m2_errorBusy), "m2_errorBusy") == 0, "busy has its name");
    CHECK(m2_errorFormat == -8 && m2_errorLimit == -9 && m2_errorBusy == -10,
          "the family's shared values");
    CHECK(strcmp(m2GetResultName(-5), "unknown m2Result") == 0, "an unused value is named unknown");
}

static void TestSimdBackend(void)
{
    // The backend name is one of the three known strings, and this test
    // process is running, so the guard did not abort: the CPU supports
    // whatever backend the binary carries.
    const char* backend = m2GetSimdBackend();
    CHECK(backend != NULL, "m2GetSimdBackend must not return NULL");
    int known = strcmp(backend, "avx2") == 0 || strcmp(backend, "neon") == 0 ||
                strcmp(backend, "scalar") == 0;
    CHECK(known, "m2GetSimdBackend must name a known backend");
    CHECK(m2IsCpuBackendSupported(), "this CPU must support the backend it is running");
    // A world was creatable in the other suites, which means the create-time
    // guard passed; confirm it is idempotent and does not disturb creation.
    m2WorldDef def = m2DefaultWorldDef();
    m2World* world = NewWorld(&def);
    CHECK(m2IsCpuBackendSupported(), "the backend check is stable across calls");
    m2DestroyWorld(world);
}

int main(void)
{
    m2Version version = m2GetVersion();
    printf("maul2d %d.%d.%d (%s backend)\n", version.major, version.minor, version.patch,
           m2GetSimdBackend());
    TestWorldAllocator();
    TestDestroyedWorldReadsEmpty();
    TestWorldSnapshotBound();
    TestWorldDefChecks();
    TestVersionAndResultNames();

    TestAccuracy();
    TestNormalizeKeepsDirection();
    TestMulRotAddsAngles();
    TestSimdBackend();

    if (s_failures > 0)
    {
        printf("%d failure(s)\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
