// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The particle neighbor pass: every pair of particles closer than one
// diameter, rebuilt from the positions each step (derived state, never
// snapshotted).
//
// Space is cut into square cells one diameter wide, so a particle's
// neighbors sit in its own cell or the eight around it. The particles
// are sorted by cell, row-major, and the sorted list is cut into runs,
// one per occupied cell. Each cell then pairs with itself and with the
// four cells ahead of it in key order (east, and the row above from
// west to east); the other four see it from their side, so every pair
// comes out exactly once. The order is fixed by the sort: cells by key,
// particles by index within a cell.
//
// Cell coordinates are biased by 2^31 and packed as (row << 32) | column,
// so key order is row-major and the seam where a coordinate wraps lies
// some 10^8 diameters out. Coincident particles pair with full weight
// along +y, never a NaN. The pair list has a fixed capacity: pairs past
// it are dropped in pair order and counted.

#include "particle.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

// A sorted particle, or, after the sort, a run of them in one cell.
typedef struct Entry
{
    uint64_t key;
    int32_t first; // particle index; for a run, its first sorted entry
    int32_t count; // for a run, how many particles it holds
} Entry;

static uint64_t CellKey(m2Pos2 p, double inverseCell)
{
    uint32_t column = (uint32_t)((int64_t)floor(p.x * inverseCell) + 2147483648LL);
    uint32_t row = (uint32_t)((int64_t)floor(p.y * inverseCell) + 2147483648LL);
    return ((uint64_t)row << 32) | (uint64_t)column;
}

static uint64_t Neighbor(uint64_t key, int32_t dColumn, int32_t dRow)
{
    uint32_t column = (uint32_t)key + (uint32_t)dColumn;
    uint32_t row = (uint32_t)(key >> 32) + (uint32_t)dRow;
    return ((uint64_t)row << 32) | (uint64_t)column;
}

// Stable least-significant-digit radix sort on the 64-bit key, a byte
// per pass. Entries go in by ascending particle index, and stability
// keeps that order inside a cell. Eight passes end where they started.
static void SortByCell(Entry* entries, Entry* scratch, int32_t count)
{
    Entry* from = entries;
    Entry* to = scratch;
    for (int32_t shift = 0; shift < 64; shift += 8)
    {
        int32_t offsets[256];
        memset(offsets, 0, sizeof(offsets));
        for (int32_t i = 0; i < count; ++i)
        {
            offsets[(from[i].key >> shift) & 0xFFu] += 1;
        }
        int32_t sum = 0;
        for (int32_t d = 0; d < 256; ++d)
        {
            int32_t n = offsets[d];
            offsets[d] = sum;
            sum += n;
        }
        for (int32_t i = 0; i < count; ++i)
        {
            to[offsets[(from[i].key >> shift) & 0xFFu]++] = from[i];
        }
        Entry* swap = from;
        from = to;
        to = swap;
    }
}

// Cuts the sorted entries into one run per occupied cell. Returns the
// run count.
static int32_t CutRuns(const Entry* sorted, int32_t count, Entry* runs)
{
    int32_t runCount = 0;
    for (int32_t i = 0; i < count; ++i)
    {
        if (runCount == 0 || runs[runCount - 1].key != sorted[i].key)
        {
            runs[runCount] = (Entry){sorted[i].key, i, 0};
            runCount += 1;
        }
        runs[runCount - 1].count += 1;
    }
    return runCount;
}

// The first run at or past key, searching [lo, runCount).
static int32_t FindRun(const Entry* runs, int32_t lo, int32_t runCount, uint64_t key)
{
    int32_t hi = runCount;
    while (lo < hi)
    {
        int32_t mid = lo + (hi - lo) / 2;
        if (runs[mid].key < key)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

static void AddPair(m2World* world, int32_t a, int32_t b, float diameter)
{
    m2Particles* p = &world->particles;
    float dx = (float)(p->particlePositions[b].x - p->particlePositions[a].x);
    float dy = (float)(p->particlePositions[b].y - p->particlePositions[a].y);
    float d2 = dx * dx + dy * dy;
    if (d2 >= diameter * diameter)
    {
        return;
    }
    if (p->particlePairCount >= p->particlePairCapacity)
    {
        p->particlePairOverflow += 1;
        return;
    }
    int32_t n = p->particlePairCount;
    p->particlePairCount = n + 1;
    p->particlePairA[n] = a;
    p->particlePairB[n] = b;
    p->particlePairFlags[n] = p->particleFlags[a] | p->particleFlags[b];
    p->particleFlagsUnion |= p->particlePairFlags[n];
    if (d2 > 1.0e-12f)
    {
        float d = sqrtf(d2);
        p->particlePairWeight[n] = 1.0f - d / diameter;
        p->particlePairNormal[n] = (m2Vec2){dx / d, dy / d};
    }
    else
    {
        p->particlePairWeight[n] = 1.0f;
        p->particlePairNormal[n] = (m2Vec2){0.0f, 1.0f};
    }
}

// Every particle of one run against every particle of another.
static void PairRuns(m2World* world, const Entry* sorted, const Entry* a, const Entry* b,
                     float diameter)
{
    for (int32_t i = a->first; i < a->first + a->count; ++i)
    {
        for (int32_t j = b->first; j < b->first + b->count; ++j)
        {
            AddPair(world, sorted[i].first, sorted[j].first, diameter);
        }
    }
}

// A run's own pairs, each once.
static void PairWithin(m2World* world, const Entry* sorted, const Entry* run, float diameter)
{
    for (int32_t i = run->first; i < run->first + run->count; ++i)
    {
        for (int32_t j = i + 1; j < run->first + run->count; ++j)
        {
            AddPair(world, sorted[i].first, sorted[j].first, diameter);
        }
    }
}

void m2UpdateParticlePairs(m2World* world)
{
    m2Particles* p = &world->particles;
    p->particlePairCount = 0;
    p->particlePairOverflow = 0;
    p->particleFlagsUnion = 0;
    if (p->particleCount == 0)
    {
        return;
    }
    float diameter = 2.0f * p->particleRadius;
    double inverseCell = 1.0 / (2.0 * (double)p->particleRadius);
    Entry* sorted = (Entry*)p->particleProxies;
    Entry* runs = (Entry*)p->particleProxiesTmp;
    int32_t count = 0;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        if (p->particleAlive[i] != 0)
        {
            sorted[count] = (Entry){CellKey(p->particlePositions[i], inverseCell), i, 0};
            count += 1;
        }
    }
    SortByCell(sorted, runs, count);
    int32_t runCount = CutRuns(sorted, count, runs);
    for (int32_t r = 0; r < runCount; ++r)
    {
        const Entry* run = &runs[r];
        PairWithin(world, sorted, run, diameter);
        if (r + 1 < runCount && runs[r + 1].key == Neighbor(run->key, 1, 0))
        {
            PairRuns(world, sorted, run, &runs[r + 1], diameter);
        }
        uint64_t last = Neighbor(run->key, 1, 1);
        for (int32_t s = FindRun(runs, r + 1, runCount, Neighbor(run->key, -1, 1));
             s < runCount && runs[s].key <= last; ++s)
        {
            PairRuns(world, sorted, run, &runs[s], diameter);
        }
    }
}
