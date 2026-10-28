// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The multi-world concurrency proof: two distinct worlds, each created,
// stepped and destroyed on its own host thread, land bit-identical to
// their serial twins, and many readers share one world. Worlds share
// nothing, so creation needs no host lock. POSIX-only referee; the
// contract holds everywhere.
#include "helpers.h"
#include "test_harness.h"

#include "maul2d/maul2d.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static m2World* BuildWorld(int32_t seed)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 128;
    def.shapeCapacity = 128;
    m2World* world = NewWorld(&def);
    m2BodyDef gd = m2DefaultBodyDef();
    m2BodyId ground = NewBody(world, &gd);
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(8.0f, 0.3f);
    NewPolygonShape(world, ground, &sd, &slab);
    for (int32_t i = 0; i < 80; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.8 * (double)(i % 10) - 3.6 + 0.01 * (double)seed,
                               0.8 + 0.7 * (double)(i / 10)};
        m2BodyId b = NewBody(world, &bd);
        if (i % 2 == 0)
        {
            m2Polygon box = m2MakeBox(0.3f, 0.3f);
            NewPolygonShape(world, b, &sd, &box);
        }
        else
        {
            m2Circle c = {{0.0f, 0.0f}, 0.3f};
            NewCircleShape(world, b, &sd, &c);
        }
    }
    return world;
}

typedef struct WorldJob
{
    int32_t seed;
    int32_t steps;
    uint64_t hash;
} WorldJob;

// A whole world's life on one thread: create, step, hash, destroy.
static void* WorldMain(void* arg)
{
    WorldJob* job = (WorldJob*)arg;
    m2World* world = BuildWorld(job->seed);
    for (int32_t i = 0; i < job->steps; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    job->hash = m2World_Hash(world);
    m2DestroyWorld(world);
    return NULL;
}

static void TestParallelWorldsMatchSerial(void)
{
    m2World* serialA = BuildWorld(1);
    m2World* serialB = BuildWorld(2);
    for (int32_t i = 0; i < 300; ++i)
    {
        m2World_Step(serialA, 1.0f / 60.0f, 4);
        m2World_Step(serialB, 1.0f / 60.0f, 4);
    }
    uint64_t wantA = m2World_Hash(serialA);
    uint64_t wantB = m2World_Hash(serialB);
    m2DestroyWorld(serialA);
    m2DestroyWorld(serialB);

    WorldJob jobA = {1, 300, 0};
    WorldJob jobB = {2, 300, 0};
    pthread_t tA;
    pthread_t tB;
    pthread_create(&tA, NULL, WorldMain, &jobA);
    pthread_create(&tB, NULL, WorldMain, &jobB);
    pthread_join(tA, NULL);
    pthread_join(tB, NULL);
    CHECK(jobA.hash == wantA, "world A is thread-placement blind");
    CHECK(jobB.hash == wantB, "world B is thread-placement blind");
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
    m2World* world;
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
        int32_t n = m2World_OverlapAabb(job->world, (m2WorldAabb){box.lower, box.upper},
                                        m2DefaultQueryFilter(), got, READER_CAPACITY);
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
    m2World* world = BuildWorld(3);
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
            m2World_OverlapAabb(world, (m2WorldAabb){box.lower, box.upper}, m2DefaultQueryFilter(),
                                jobs[0].expected[r], READER_CAPACITY);
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
