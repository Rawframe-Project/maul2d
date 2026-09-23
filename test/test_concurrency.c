// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The multi-world concurrency proof (integration audit B2): two
// DISTINCT worlds stepped on two host threads land bit-identical
// to their serial twins. One writer per world stays the law;
// create/destroy stay host-serialized. POSIX-only referee; the
// contract holds everywhere.
#include "maul2d/maul2d.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

static m2WorldId BuildWorld(int32_t seed)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2WorldId world = m2CreateWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = m2CreateBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(8.0f, 0.3f);
    m2CreatePolygonShape(ground, &sd, &slab);
    for (int32_t i = 0; i < 80; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.8 * (double)(i % 10) - 3.6 + 0.01 * (double)seed,
                               0.8 + 0.7 * (double)(i / 10)};
        m2BodyId b = m2CreateBody(world, &bd);
        if (i % 2 == 0)
        {
            m2Polygon box = m2MakeBox(0.3f, 0.3f);
            m2CreatePolygonShape(b, &sd, &box);
        }
        else
        {
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            m2CreateCircleShape(b, &sd, &c);
        }
    }
    return world;
}

typedef struct StepJob
{
    m2WorldId world;
    int32_t steps;
} StepJob;

static void* StepMain(void* arg)
{
    StepJob* job = (StepJob*)arg;
    for (int32_t i = 0; i < job->steps; ++i)
    {
        m2World_Step(job->world, 1.0f / 60.0f, 4);
    }
    return NULL;
}

static void TestParallelWorldsMatchSerial(void)
{
    m2WorldId serialA = BuildWorld(1);
    m2WorldId serialB = BuildWorld(2);
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(serialA, 1.0f / 60.0f, 4);
        m2World_Step(serialB, 1.0f / 60.0f, 4);
    }
    uint64_t wantA = m2World_Hash(serialA);
    uint64_t wantB = m2World_Hash(serialB);
    m2DestroyWorld(serialA);
    m2DestroyWorld(serialB);

    m2WorldId parA = BuildWorld(1);
    m2WorldId parB = BuildWorld(2);
    StepJob jobA = {parA, 300};
    StepJob jobB = {parB, 300};
    pthread_t tA;
    pthread_t tB;
    pthread_create(&tA, NULL, StepMain, &jobA);
    pthread_create(&tB, NULL, StepMain, &jobB);
    pthread_join(tA, NULL);
    pthread_join(tB, NULL);
    CHECK(m2World_Hash(parA) == wantA, "world A is thread-placement blind");
    CHECK(m2World_Hash(parB) == wantB, "world B is thread-placement blind");
    m2DestroyWorld(parA);
    m2DestroyWorld(parB);
}

// Readers share a world: many threads query one world at once and
// every answer equals the serial answer.
enum
{
    READER_THREADS = 4,
    READER_ROUNDS = 400,
    READER_CAPACITY = 16
};

typedef struct ReaderJob
{
    m2WorldId world;
    int32_t mismatches;
    int32_t expectedCount[READER_ROUNDS];
    m2ShapeId expected[READER_ROUNDS][READER_CAPACITY];
} ReaderJob;

typedef struct ReaderBox
{
    m2Pos2 lower;
    m2Pos2 upper;
} ReaderBox;

static ReaderBox MakeReaderBox(int32_t round)
{
    double x = -4.0 + 0.02 * (double)round;
    ReaderBox box = {{x, 0.0}, {x + 1.5, 6.0}};
    return box;
}

static void* ReaderMain(void* arg)
{
    ReaderJob* job = (ReaderJob*)arg;
    for (int32_t r = 0; r < READER_ROUNDS; ++r)
    {
        ReaderBox box = MakeReaderBox(r);
        m2ShapeId got[READER_CAPACITY];
        int32_t n = m2World_OverlapAABB(job->world, box.lower, box.upper, got, READER_CAPACITY,
                                        m2DefaultQueryFilter());
        int32_t stored = n < READER_CAPACITY ? n : READER_CAPACITY;
        if (n != job->expectedCount[r] ||
            memcmp(got, job->expected[r], (size_t)stored * sizeof(m2ShapeId)) != 0)
        {
            job->mismatches += 1;
        }
    }
    return NULL;
}

static void TestConcurrentReadersMatchSerial(void)
{
    m2WorldId world = BuildWorld(3);
    for (int32_t i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    static ReaderJob jobs[READER_THREADS];
    memset(jobs, 0, sizeof(jobs));
    for (int32_t r = 0; r < READER_ROUNDS; ++r)
    {
        ReaderBox box = MakeReaderBox(r);
        jobs[0].expectedCount[r] =
            m2World_OverlapAABB(world, box.lower, box.upper, jobs[0].expected[r], READER_CAPACITY,
                                m2DefaultQueryFilter());
    }
    CHECK(jobs[0].expectedCount[READER_ROUNDS / 2] > READER_CAPACITY,
          "some queries overflow the result capacity");
    pthread_t threads[READER_THREADS];
    for (int32_t t = 0; t < READER_THREADS; ++t)
    {
        jobs[t].world = world;
        if (t > 0)
        {
            memcpy(jobs[t].expectedCount, jobs[0].expectedCount, sizeof(jobs[0].expectedCount));
            memcpy(jobs[t].expected, jobs[0].expected, sizeof(jobs[0].expected));
        }
        pthread_create(&threads[t], NULL, ReaderMain, &jobs[t]);
    }
    int32_t mismatches = 0;
    for (int32_t t = 0; t < READER_THREADS; ++t)
    {
        pthread_join(threads[t], NULL);
        mismatches += jobs[t].mismatches;
    }
    CHECK(mismatches == 0, "concurrent readers see exactly the serial answers");
    m2DestroyWorld(world);
}

int main(void)
{
    TestParallelWorldsMatchSerial();
    TestConcurrentReadersMatchSerial();
    if (s_failures == 0)
    {
        printf("test_concurrency: all green\n");
        return 0;
    }
    return 1;
}
