// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Fluids, the neighbor structure: a uniform grid
// with cell size = particle diameter, rebuilt from positions every
// step (history-free, the island precedent: derived structures
// never enter the snapshot). The pair list is canonical by
// construction: proxies sorted by (cell key, particle index), one
// sweep visiting the east neighbor on the same row and the
// west/center/east cells on the row above, every pair emitted
// exactly once in sweep order.
//
// Adapted from LiquidFun's b2ParticleSystem contact pass (see
// THIRD_PARTY.md) with three argued deviations:
//   1. Cell keys are 64-bit with BIASED coordinates
//      ((int64)floor(x/d) + 2^31 packed as (y << 32) | x). The
//      reference's 32-bit offset tag wraps close to the origin
//      axes; a wrap between adjacent columns would silently drop
//      seam pairs. With the bias the seam sits ~2*10^8 m out, and
//      the distance test guards even that.
//   2. Coincident particles (distance exactly zero) get the
//      canonical fallback normal (0, 1) and full weight instead of
//      the reference's NaN (0 * inf); the engine never produces NaN.
//   3. The pair buffer is fixed (12 per particle of capacity);
//      overflow truncates DETERMINISTICALLY in sweep order and
//      counts loudly in particlePairOverflow. LiquidFun grows
//      buffers; Maul's fixed-capacity law does not.

#include "body.h"
#include "distance.h"
#include "journal.h"
#include "particle.h"
#include "query.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

typedef struct m2ParticleProxy
{
    uint64_t key;
    int32_t index;
    int32_t pad; // explicit, never read
} m2ParticleProxy;

static uint64_t ParticleCellKey(m2Pos2 position, double inverseDiameter)
{
    uint32_t cx = (uint32_t)((int64_t)floor(position.x * inverseDiameter) + 2147483648LL);
    uint32_t cy = (uint32_t)((int64_t)floor(position.y * inverseDiameter) + 2147483648LL);
    return ((uint64_t)cy << 32) | (uint64_t)cx;
}

static uint64_t ShiftKey(uint64_t key, int32_t dx, int32_t dy)
{
    uint32_t cx = (uint32_t)key + (uint32_t)dx;
    uint32_t cy = (uint32_t)(key >> 32) + (uint32_t)dy;
    return ((uint64_t)cy << 32) | (uint64_t)cx;
}

// Stable LSD radix over the 64-bit cell key, eight 8-bit passes.
// The proxy array is built in ascending particle index order and
// stability preserves it, so the result is exactly the (key, index)
// order the old comparison sort produced: the gated pair digest is
// the proof, byte for byte. O(n) instead of n log n, and the pair
// pass stops being sort-bound as pools grow.
static void SortProxies(m2ParticleProxy* proxies, m2ParticleProxy* scratch, int32_t count)
{
    m2ParticleProxy* src = proxies;
    m2ParticleProxy* dst = scratch;
    for (int32_t pass = 0; pass < 8; ++pass)
    {
        int32_t shift = pass * 8;
        int32_t histogram[256];
        memset(histogram, 0, sizeof(histogram));
        for (int32_t i = 0; i < count; ++i)
        {
            histogram[(src[i].key >> shift) & 0xFF] += 1;
        }
        int32_t running = 0;
        for (int32_t b = 0; b < 256; ++b)
        {
            int32_t n = histogram[b];
            histogram[b] = running;
            running += n;
        }
        for (int32_t i = 0; i < count; ++i)
        {
            dst[histogram[(src[i].key >> shift) & 0xFF]++] = src[i];
        }
        m2ParticleProxy* swap = src;
        src = dst;
        dst = swap;
    }
    // Eight passes: the data ends where it started.
}

// First proxy at or after key, searching [lo, count).
static int32_t LowerBound(const m2ParticleProxy* proxies, int32_t lo, int32_t count, uint64_t key)
{
    int32_t hi = count;
    while (lo < hi)
    {
        int32_t mid = lo + (hi - lo) / 2;
        if (proxies[mid].key < key)
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

static void TryPair(m2World* world, int32_t a, int32_t b, float diameter)
{
    float dx =
        (float)(world->particles.particlePositions[b].x - world->particles.particlePositions[a].x);
    float dy =
        (float)(world->particles.particlePositions[b].y - world->particles.particlePositions[a].y);
    float distSq = dx * dx + dy * dy;
    if (distSq >= diameter * diameter)
    {
        return;
    }
    if (world->particles.particlePairCount >= world->particles.particlePairCapacity)
    {
        world->particles.particlePairOverflow += 1; // deterministic truncation, counted
        return;
    }
    int32_t n = world->particles.particlePairCount;
    world->particles.particlePairA[n] = a;
    world->particles.particlePairB[n] = b;
    world->particles.particlePairFlags[n] =
        world->particles.particleFlags[a] | world->particles.particleFlags[b];
    world->particles.particleFlagsUnion |= world->particles.particlePairFlags[n];
    if (distSq > 1.0e-12f)
    {
        float dist = sqrtf(distSq);
        float inv = 1.0f / dist;
        world->particles.particlePairWeight[n] = 1.0f - dist / diameter;
        world->particles.particlePairNormal[n] = (m2Vec2){dx * inv, dy * inv};
    }
    else
    {
        // Coincident: canonical fallback, never NaN (deviation 2).
        world->particles.particlePairWeight[n] = 1.0f;
        world->particles.particlePairNormal[n] = (m2Vec2){0.0f, 1.0f};
    }
    world->particles.particlePairCount = n + 1;
}

void m2UpdateParticlePairs(m2World* world)
{
    world->particles.particlePairCount = 0;
    world->particles.particlePairOverflow = 0;
    world->particles.particleFlagsUnion = 0;
    if (world->particles.particleCount == 0)
    {
        return;
    }
    float diameter = 2.0f * world->particles.particleRadius;
    double inverseDiameter = 1.0 / (2.0 * (double)world->particles.particleRadius);

    m2ParticleProxy* proxies = (m2ParticleProxy*)world->particles.particleProxies;
    int32_t proxyCount = 0;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        proxies[proxyCount].key =
            ParticleCellKey(world->particles.particlePositions[i], inverseDiameter);
        proxies[proxyCount].index = i;
        proxies[proxyCount].pad = 0;
        proxyCount += 1;
    }
    SortProxies(proxies, (m2ParticleProxy*)world->particles.particleProxiesTmp, proxyCount);

    for (int32_t a = 0; a < proxyCount; ++a)
    {
        uint64_t key = proxies[a].key;
        // Same row: this cell's remainder plus the east neighbor.
        uint64_t eastKey = ShiftKey(key, 1, 0);
        for (int32_t b = a + 1; b < proxyCount && proxies[b].key <= eastKey; ++b)
        {
            TryPair(world, proxies[a].index, proxies[b].index, diameter);
        }
        // Row above: west, center and east cells as one key range.
        uint64_t rowLo = ShiftKey(key, -1, 1);
        uint64_t rowHi = ShiftKey(key, 1, 1);
        for (int32_t b = LowerBound(proxies, a + 1, proxyCount, rowLo);
             b < proxyCount && proxies[b].key <= rowHi; ++b)
        {
            TryPair(world, proxies[a].index, proxies[b].index, diameter);
        }
    }
}

// Particle-vs-body contacts: for every particle,
// every shape whose surface sits within one diameter contributes a
// contact carrying the reference fields (weight, outward normal,
// pair-effective mass). Candidates come from the three trees like
// the CCD sweep does: sensors never touch water, one-way chain
// links ignore particles on their ghost side, order is canonical
// (particles in index order, candidate shapes ascending). One-way
// for now: bodies push water, the water pushes back in the next
// slice.
#define M2_PARTICLE_CANDIDATES 32

static void StageBodyContactsRange(int32_t begin, int32_t end, void* ctx)
{
    m2World* world = (m2World*)ctx;
    float diameter = 2.0f * world->particles.particleRadius;
    float stride = 0.75f * diameter;
    float particleMass = world->particles.particleDensity * stride * stride;
    float invAm = 1.0f / particleMass;

    for (int32_t i = begin; i < end; ++i)
    {
        world->particles.particleBodyStageDrops[i] = 0;
        for (int32_t k = 0; k < 4; ++k)
        {
            world->particles.particleBodyStageBody[i * 4 + k] = -1;
        }
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 pp = world->particles.particlePositions[i];
        m2AABB box;
        box.lowerBound.x = pp.x - (double)diameter;
        box.lowerBound.y = pp.y - (double)diameter;
        box.upperBound.x = pp.x + (double)diameter;
        box.upperBound.y = pp.y + (double)diameter;

        int32_t candidates[M2_PARTICLE_CANDIDATES];
        int32_t candidateCount = 0;
        for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
        {
            int32_t results[M2_PARTICLE_CANDIDATES];
            int32_t hits = m2TreeQuery(&world->broadphase.trees[t], world->broadphase.treeNodes[t],
                                       box, results, M2_PARTICLE_CANDIDATES);
            hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
            for (int32_t h = 0; h < hits && candidateCount < M2_PARTICLE_CANDIDATES; ++h)
            {
                int32_t shape = results[h];
                if (world->shapes.shapeSensor[shape] != 0)
                {
                    continue;
                }
                candidates[candidateCount++] = shape;
            }
        }
        for (int32_t a = 1; a < candidateCount; ++a)
        {
            int32_t key = candidates[a];
            int32_t j = a - 1;
            while (j >= 0 && candidates[j] > key)
            {
                candidates[j + 1] = candidates[j];
                j -= 1;
            }
            candidates[j + 1] = key;
        }

        int32_t kept = 0;
        for (int32_t c = 0; c < candidateCount; ++c)
        {
            int32_t shape = candidates[c];
            int32_t body = world->shapes.shapeBody[shape];
            m2Transform xf = world->bodies.transforms[body];
            float dx = (float)(pp.x - xf.p.x);
            float dy = (float)(pp.y - xf.p.y);
            m2Vec2 local = {xf.q.c * dx + xf.q.s * dy, -xf.q.s * dx + xf.q.c * dy};

            const m2ShapeGeometry* g = &world->shapes.shapeGeometry[shape];
            if (g->type == m2_chainSegmentShape)
            {
                const m2ChainSegment* link = &g->chainSegment;
                m2Vec2 e = {link->segment.point2.x - link->segment.point1.x,
                            link->segment.point2.y - link->segment.point1.y};
                float offset = (local.x - link->segment.point1.x) * e.y -
                               (local.y - link->segment.point1.y) * e.x;
                if (offset < 0.0f)
                {
                    continue;
                }
            }

            m2DistanceProxy shapeProxy = m2GeometryProxy(g);
            m2DistanceProxy pointProxy;
            memset(&pointProxy, 0, sizeof(pointProxy));
            pointProxy.points[0] = local;
            pointProxy.count = 1;
            pointProxy.radius = 0.0f;
            m2DistanceResult dist = m2ShapeDistance(&shapeProxy, &pointProxy);
            float separation = dist.distance - shapeProxy.radius;
            if (separation >= diameter)
            {
                continue;
            }
            if (kept >= 4)
            {
                world->particles.particleBodyStageDrops[i] += 1;
                continue;
            }
            m2Vec2 n;
            if (dist.normal.x != 0.0f || dist.normal.y != 0.0f)
            {
                n = Rotate(xf.q, dist.normal);
            }
            else
            {
                float len = sqrtf(dx * dx + dy * dy);
                n = len > 1.0e-6f ? (m2Vec2){dx / len, dy / len} : (m2Vec2){0.0f, 1.0f};
            }
            m2Vec2 lc = world->bodies.localCenters[body];
            m2Vec2 comArm = Rotate(xf.q, lc);
            float rpx = dx - comArm.x;
            float rpy = dy - comArm.y;
            float rpn = rpx * n.y - rpy * n.x;
            float invM =
                invAm + world->bodies.invMass[body] + world->bodies.invInertia[body] * rpn * rpn;
            int32_t slot = i * 4 + kept;
            world->particles.particleBodyStageBody[slot] = body;
            world->particles.particleBodyStageWeight[slot] = 1.0f - separation / diameter;
            world->particles.particleBodyStageNormal[slot] = n;
            world->particles.particleBodyStageMass[slot] = invM > 0.0f ? 1.0f / invM : 0.0f;
            kept += 1;
        }
    }
}

// The barrier only pays above a few thousand particles (measured
// crossover between 2048 and 8192); below it the pool stays home so
// small pools never eat the fork-join tax. Either path writes the
// same bytes, so the choice is pure performance, never physics.
#define M2_FLUID_THREAD_MIN 4096

static void UpdateParticleBodyContacts(m2World* world)
{
    world->particles.particleBodyCount = 0;
    world->particles.particleBodyOverflow = 0;
    if (world->particles.particleCount >= M2_FLUID_THREAD_MIN)
    {
        m2RunParallel(world, StageBodyContactsRange, world, world->particles.maxParticleIndex, 64);
    }
    else
    {
        StageBodyContactsRange(0, world->particles.maxParticleIndex, world);
    }
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        world->particles.particleBodyOverflow += world->particles.particleBodyStageDrops[i];
        for (int32_t k = 0; k < 4; ++k)
        {
            int32_t body = world->particles.particleBodyStageBody[i * 4 + k];
            if (body < 0)
            {
                break;
            }
            if (world->particles.particleBodyCount >= world->particles.particleBodyCapacity)
            {
                world->particles.particleBodyOverflow += 1;
                continue;
            }
            int32_t out = world->particles.particleBodyCount;
            world->particles.particleBodyParticle[out] = i;
            world->particles.particleBodyBody[out] = body;
            world->particles.particleBodyWeight[out] =
                world->particles.particleBodyStageWeight[i * 4 + k];
            world->particles.particleBodyNormal[out] =
                world->particles.particleBodyStageNormal[i * 4 + k];
            world->particles.particleBodyMass[out] =
                world->particles.particleBodyStageMass[i * 4 + k];
            world->particles.particleBodyCount = out + 1;
            m2WakeIfDynamic(world, body);
        }
    }
}

// The water pass, the reference relaxation solver
// on the frozen pair list, once per step before the rigid solve:
// weight (dimensionless density) -> viscosity (system-level strength;
// zero means plain water) -> gravity -> pressure -> damping ->
// velocity limit -> advance. All scalars f32 through the pinned op
// whitelist (sqrtf + divide, never a fast inverse sqrt), min/max as
// compare+select, every loop in fixed index or pair order.
void m2SolveParticles(m2World* world, float dt)
{
    m2UpdateParticlePairs(world);
    UpdateParticleBodyContacts(world);
    if (dt <= 0.0f)
    {
        return;
    }
    float stride = 0.75f * 2.0f * world->particles.particleRadius;
    float particleInvMass = 1.0f / (world->particles.particleDensity * stride * stride);
    int32_t bodyContactCount = world->particles.particleBodyCount;
    float invDt = 1.0f / dt;
    float diameter = 2.0f * world->particles.particleRadius;
    int32_t pairCount = world->particles.particlePairCount;

    // Dimensionless density: the sum of contact weights.
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        world->particles.particleWeights[i] = 0.0f;
    }
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        world->particles.particleWeights[world->particles.particleBodyParticle[k]] +=
            world->particles.particleBodyWeight[k];
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        float w = world->particles.particlePairWeight[k];
        world->particles.particleWeights[world->particles.particlePairA[k]] += w;
        world->particles.particleWeights[world->particles.particlePairB[k]] += w;
    }

    // Viscosity drags flagged particles toward their neighbors'
    // velocities (the reference's per-particle gate, adopted now
    // that behavior flags exist).
    float viscous = world->particles.particleViscousStrength;
    if (viscous > 0.0f)
    {
        for (int32_t k = 0; k < bodyContactCount; ++k)
        {
            int32_t a = world->particles.particleBodyParticle[k];
            if ((world->particles.particleFlags[a] & m2_viscousParticle) == 0)
            {
                continue;
            }
            int32_t body = world->particles.particleBodyBody[k];
            float w = world->particles.particleBodyWeight[k];
            float m = world->particles.particleBodyMass[k];
            m2Pos2 pp = world->particles.particlePositions[a];
            m2Vec2 lc = world->bodies.localCenters[body];
            m2Vec2 comArm = Rotate(world->bodies.transforms[body].q, lc);
            float rx = (float)(pp.x - world->bodies.transforms[body].p.x) - comArm.x;
            float ry = (float)(pp.y - world->bodies.transforms[body].p.y) - comArm.y;
            float wb = world->bodies.angularVelocities[body];
            float bvx = world->bodies.linearVelocities[body].x - wb * ry;
            float bvy = world->bodies.linearVelocities[body].y + wb * rx;
            float fx = viscous * m * w * (bvx - world->particles.particleVelocities[a].x);
            float fy = viscous * m * w * (bvy - world->particles.particleVelocities[a].y);
            world->particles.particleVelocities[a].x += particleInvMass * fx;
            world->particles.particleVelocities[a].y += particleInvMass * fy;
            world->bodies.linearVelocities[body].x -= world->bodies.invMass[body] * fx;
            world->bodies.linearVelocities[body].y -= world->bodies.invMass[body] * fy;
            world->bodies.angularVelocities[body] -=
                world->bodies.invInertia[body] * (rx * fy - ry * fx);
        }
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particles.particlePairFlags[k] & m2_viscousParticle) == 0)
            {
                continue;
            }
            int32_t a = world->particles.particlePairA[k];
            int32_t b = world->particles.particlePairB[k];
            float w = world->particles.particlePairWeight[k];
            float fx = viscous * w *
                       (world->particles.particleVelocities[b].x -
                        world->particles.particleVelocities[a].x);
            float fy = viscous * w *
                       (world->particles.particleVelocities[b].y -
                        world->particles.particleVelocities[a].y);
            world->particles.particleVelocities[a].x += fx;
            world->particles.particleVelocities[a].y += fy;
            world->particles.particleVelocities[b].x -= fx;
            world->particles.particleVelocities[b].y -= fy;
        }
    }

    // Powder (reference SolvePowder): grains packed tighter than the
    // rest stride push apart and never cohere; that is the whole law
    // of sand.
    if ((world->particles.particleFlagsUnion & m2_powderParticle) != 0)
    {
        float powder = world->particles.particlePowderStrength * (diameter * invDt);
        float minWeight = 1.0f - 0.75f; // one minus the particle stride
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particles.particlePairFlags[k] & m2_powderParticle) == 0)
            {
                continue;
            }
            float w = world->particles.particlePairWeight[k];
            if (w <= minWeight)
            {
                continue;
            }
            int32_t a = world->particles.particlePairA[k];
            int32_t b = world->particles.particlePairB[k];
            m2Vec2 n = world->particles.particlePairNormal[k];
            float f = powder * (w - minWeight);
            world->particles.particleVelocities[a].x -= f * n.x;
            world->particles.particleVelocities[a].y -= f * n.y;
            world->particles.particleVelocities[b].x += f * n.x;
            world->particles.particleVelocities[b].y += f * n.y;
        }
    }

    // Surface tension (reference SolveTensile): each tensile pair
    // first votes a weighted surface normal, then pairs attract by
    // local density above 2 plus the normal disagreement, capped at
    // half the critical velocity. Droplets bead up and cling.
    if ((world->particles.particleFlagsUnion & m2_tensileParticle) != 0)
    {
        for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
        {
            world->particles.particleAccumulation2[i] = (m2Vec2){0.0f, 0.0f};
        }
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particles.particlePairFlags[k] & m2_tensileParticle) == 0)
            {
                continue;
            }
            int32_t a = world->particles.particlePairA[k];
            int32_t b = world->particles.particlePairB[k];
            float w = world->particles.particlePairWeight[k];
            m2Vec2 n = world->particles.particlePairNormal[k];
            float wn = (1.0f - w) * w;
            world->particles.particleAccumulation2[a].x -= wn * n.x;
            world->particles.particleAccumulation2[a].y -= wn * n.y;
            world->particles.particleAccumulation2[b].x += wn * n.x;
            world->particles.particleAccumulation2[b].y += wn * n.y;
        }
        float tensilePressure = world->particles.particleTensilePressure * (diameter * invDt);
        float tensileNormal = world->particles.particleTensileNormal * (diameter * invDt);
        float maxVariation = 0.5f * (diameter * invDt);
        for (int32_t k = 0; k < pairCount; ++k)
        {
            if ((world->particles.particlePairFlags[k] & m2_tensileParticle) == 0)
            {
                continue;
            }
            int32_t a = world->particles.particlePairA[k];
            int32_t b = world->particles.particlePairB[k];
            float w = world->particles.particlePairWeight[k];
            m2Vec2 n = world->particles.particlePairNormal[k];
            float h = world->particles.particleWeights[a] + world->particles.particleWeights[b];
            float sx = world->particles.particleAccumulation2[b].x -
                       world->particles.particleAccumulation2[a].x;
            float sy = world->particles.particleAccumulation2[b].y -
                       world->particles.particleAccumulation2[a].y;
            float fn = m2MinF(tensilePressure * (h - 2.0f) + tensileNormal * (sx * n.x + sy * n.y),
                              maxVariation) *
                       w;
            world->particles.particleVelocities[a].x -= fn * n.x;
            world->particles.particleVelocities[a].y -= fn * n.y;
            world->particles.particleVelocities[b].x += fn * n.x;
            world->particles.particleVelocities[b].y += fn * n.y;
        }
    }

    // Gravity, full step, fixed index order.
    float gx = dt * world->particles.particleGravityScale * world->gravity.x;
    float gy = dt * world->particles.particleGravityScale * world->gravity.y;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] != 0)
        {
            world->particles.particleVelocities[i].x += gx;
            world->particles.particleVelocities[i].y += gy;
        }
    }

    // Pressure as a linear function of density above the rest weight
    // (reference constants: min weight 1, pressure cap 0.25 of the
    // critical pressure; the critical velocity is one diameter per
    // step, the scale that keeps neighbors discoverable).
    float criticalVelocity = diameter * invDt;
    float criticalPressure = world->particles.particleDensity * criticalVelocity * criticalVelocity;
    float pressurePerWeight = world->particles.particlePressureStrength * criticalPressure;
    float maxPressure = 0.25f * criticalPressure;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        float w = world->particles.particleWeights[i];
        float h = pressurePerWeight * m2MaxF(0.0f, w - 1.0f);
        world->particles.particleAccumulation[i] = m2MinF(h, maxPressure);
        // Powder and tensile particles produce no dynamic pressure
        // (the reference's k_noPressureFlags): they carry their own
        // repulsion and cohesion laws instead.
        if ((world->particles.particleFlags[i] & (m2_powderParticle | m2_tensileParticle)) != 0)
        {
            world->particles.particleAccumulation[i] = 0.0f;
        }
    }
    float velocityPerPressure = dt / (world->particles.particleDensity * diameter);
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        int32_t a = world->particles.particleBodyParticle[k];
        int32_t body = world->particles.particleBodyBody[k];
        float w = world->particles.particleBodyWeight[k];
        float m = world->particles.particleBodyMass[k];
        m2Vec2 n = world->particles.particleBodyNormal[k];
        float h = world->particles.particleAccumulation[a] + pressurePerWeight * w;
        float f = velocityPerPressure * w * m * h;
        // n points from the shape toward the particle: push out, and
        // push the body the other way (buoyancy is exactly this).
        world->particles.particleVelocities[a].x += particleInvMass * f * n.x;
        world->particles.particleVelocities[a].y += particleInvMass * f * n.y;
        m2Pos2 pp = world->particles.particlePositions[a];
        m2Vec2 lc = world->bodies.localCenters[body];
        m2Vec2 comArm = Rotate(world->bodies.transforms[body].q, lc);
        float rx = (float)(pp.x - world->bodies.transforms[body].p.x) - comArm.x;
        float ry = (float)(pp.y - world->bodies.transforms[body].p.y) - comArm.y;
        world->bodies.linearVelocities[body].x -= world->bodies.invMass[body] * f * n.x;
        world->bodies.linearVelocities[body].y -= world->bodies.invMass[body] * f * n.y;
        world->bodies.angularVelocities[body] -=
            world->bodies.invInertia[body] * (rx * f * n.y - ry * f * n.x);
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        int32_t a = world->particles.particlePairA[k];
        int32_t b = world->particles.particlePairB[k];
        float w = world->particles.particlePairWeight[k];
        m2Vec2 n = world->particles.particlePairNormal[k];
        float h =
            world->particles.particleAccumulation[a] + world->particles.particleAccumulation[b];
        float f = velocityPerPressure * w * h;
        world->particles.particleVelocities[a].x -= f * n.x;
        world->particles.particleVelocities[a].y -= f * n.y;
        world->particles.particleVelocities[b].x += f * n.x;
        world->particles.particleVelocities[b].y += f * n.y;
    }

    // Damping eats approach speed only (vn < 0), linear plus
    // quadratic, capped at half the approach per pass.
    float linearDamping = world->particles.particleDampingStrength;
    float quadraticDamping = 1.0f / criticalVelocity;
    for (int32_t k = 0; k < bodyContactCount; ++k)
    {
        int32_t a = world->particles.particleBodyParticle[k];
        int32_t body = world->particles.particleBodyBody[k];
        float w = world->particles.particleBodyWeight[k];
        float m = world->particles.particleBodyMass[k];
        m2Vec2 n = world->particles.particleBodyNormal[k];
        m2Pos2 pp = world->particles.particlePositions[a];
        m2Vec2 lc = world->bodies.localCenters[body];
        m2Vec2 comArm = Rotate(world->bodies.transforms[body].q, lc);
        float rx = (float)(pp.x - world->bodies.transforms[body].p.x) - comArm.x;
        float ry = (float)(pp.y - world->bodies.transforms[body].p.y) - comArm.y;
        float wb = world->bodies.angularVelocities[body];
        float bvx = world->bodies.linearVelocities[body].x - wb * ry;
        float bvy = world->bodies.linearVelocities[body].y + wb * rx;
        float relx = bvx - world->particles.particleVelocities[a].x;
        float rely = bvy - world->particles.particleVelocities[a].y;
        float vn = relx * n.x + rely * n.y;
        if (vn > 0.0f)
        {
            // Approaching along the outward normal from the particle's
            // side; eat it on both ends like the pair pass does.
            float damping = m2MaxF(linearDamping * w, m2MinF(quadraticDamping * vn, 0.5f));
            float f = damping * m * vn;
            world->particles.particleVelocities[a].x += particleInvMass * f * n.x;
            world->particles.particleVelocities[a].y += particleInvMass * f * n.y;
            world->bodies.linearVelocities[body].x -= world->bodies.invMass[body] * f * n.x;
            world->bodies.linearVelocities[body].y -= world->bodies.invMass[body] * f * n.y;
            world->bodies.angularVelocities[body] -=
                world->bodies.invInertia[body] * (rx * f * n.y - ry * f * n.x);
        }
    }
    for (int32_t k = 0; k < pairCount; ++k)
    {
        int32_t a = world->particles.particlePairA[k];
        int32_t b = world->particles.particlePairB[k];
        float w = world->particles.particlePairWeight[k];
        m2Vec2 n = world->particles.particlePairNormal[k];
        float vx =
            world->particles.particleVelocities[b].x - world->particles.particleVelocities[a].x;
        float vy =
            world->particles.particleVelocities[b].y - world->particles.particleVelocities[a].y;
        float vn = vx * n.x + vy * n.y;
        if (vn < 0.0f)
        {
            float damping = m2MaxF(linearDamping * w, m2MinF(-quadraticDamping * vn, 0.5f));
            float f = damping * vn;
            world->particles.particleVelocities[a].x += f * n.x;
            world->particles.particleVelocities[a].y += f * n.y;
            world->particles.particleVelocities[b].x -= f * n.x;
            world->particles.particleVelocities[b].y -= f * n.y;
        }
    }

    // Jelly, late for stability like the reference: elastic triads
    // pull toward their spawn shape through a best-fit rotation, then
    // springs pull toward their spawn lengths. Both read predicted
    // positions (p + dt*v) and correct velocities.
    if (world->particles.particleTriadCount > 0)
    {
        float strength = invDt * world->particles.particleElasticStrength;
        for (int32_t k = 0; k < world->particles.particleTriadCount; ++k)
        {
            int32_t a = world->particles.particleTriadA[k];
            int32_t b = world->particles.particleTriadB[k];
            int32_t c = world->particles.particleTriadC[k];
            m2Vec2 oa = world->particles.particleTriadPA[k];
            m2Vec2 ob = world->particles.particleTriadPB[k];
            m2Vec2 oc = world->particles.particleTriadPC[k];
            double pax = world->particles.particlePositions[a].x +
                         (double)(dt * world->particles.particleVelocities[a].x);
            double pay = world->particles.particlePositions[a].y +
                         (double)(dt * world->particles.particleVelocities[a].y);
            double pbx = world->particles.particlePositions[b].x +
                         (double)(dt * world->particles.particleVelocities[b].x);
            double pby = world->particles.particlePositions[b].y +
                         (double)(dt * world->particles.particleVelocities[b].y);
            double pcx = world->particles.particlePositions[c].x +
                         (double)(dt * world->particles.particleVelocities[c].x);
            double pcy = world->particles.particlePositions[c].y +
                         (double)(dt * world->particles.particleVelocities[c].y);
            double midx = (pax + pbx + pcx) / 3.0;
            double midy = (pay + pby + pcy) / 3.0;
            float ax = (float)(pax - midx);
            float ay = (float)(pay - midy);
            float bx = (float)(pbx - midx);
            float by = (float)(pby - midy);
            float cx = (float)(pcx - midx);
            float cy = (float)(pcy - midy);
            float rs = oa.x * ay - oa.y * ax + (ob.x * by - ob.y * bx) + (oc.x * cy - oc.y * cx);
            float rc = oa.x * ax + oa.y * ay + (ob.x * bx + ob.y * by) + (oc.x * cx + oc.y * cy);
            float r2 = rs * rs + rc * rc;
            if (r2 <= 1.0e-12f)
            {
                continue; // degenerate triad this step: never a NaN
            }
            float inv = 1.0f / sqrtf(r2);
            rs *= inv;
            rc *= inv;
            world->particles.particleVelocities[a].x += strength * (rc * oa.x - rs * oa.y - ax);
            world->particles.particleVelocities[a].y += strength * (rs * oa.x + rc * oa.y - ay);
            world->particles.particleVelocities[b].x += strength * (rc * ob.x - rs * ob.y - bx);
            world->particles.particleVelocities[b].y += strength * (rs * ob.x + rc * ob.y - by);
            world->particles.particleVelocities[c].x += strength * (rc * oc.x - rs * oc.y - cx);
            world->particles.particleVelocities[c].y += strength * (rs * oc.x + rc * oc.y - cy);
        }
    }
    if (world->particles.particleSpringCount > 0)
    {
        float strength = invDt * world->particles.particleSpringStrength;
        for (int32_t k = 0; k < world->particles.particleSpringCount; ++k)
        {
            int32_t a = world->particles.particleSpringA[k];
            int32_t b = world->particles.particleSpringB[k];
            double pax = world->particles.particlePositions[a].x +
                         (double)(dt * world->particles.particleVelocities[a].x);
            double pay = world->particles.particlePositions[a].y +
                         (double)(dt * world->particles.particleVelocities[a].y);
            double pbx = world->particles.particlePositions[b].x +
                         (double)(dt * world->particles.particleVelocities[b].x);
            double pby = world->particles.particlePositions[b].y +
                         (double)(dt * world->particles.particleVelocities[b].y);
            float dx = (float)(pbx - pax);
            float dy = (float)(pby - pay);
            float r1 = sqrtf(dx * dx + dy * dy);
            if (r1 <= 1.0e-6f)
            {
                continue;
            }
            float f = strength * (world->particles.particleSpringRest[k] - r1) / r1;
            world->particles.particleVelocities[a].x -= f * dx;
            world->particles.particleVelocities[a].y -= f * dy;
            world->particles.particleVelocities[b].x += f * dx;
            world->particles.particleVelocities[b].y += f * dy;
        }
    }

    // The velocity limit is the stability law: nothing crosses more
    // than one diameter per step, so neighbors stay discoverable and
    // dense clusters cannot explode.
    float criticalSq = criticalVelocity * criticalVelocity;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        float vx = world->particles.particleVelocities[i].x;
        float vy = world->particles.particleVelocities[i].y;
        float v2 = vx * vx + vy * vy;
        if (v2 > criticalSq)
        {
            float scale = sqrtf(criticalSq / v2);
            world->particles.particleVelocities[i].x = vx * scale;
            world->particles.particleVelocities[i].y = vy * scale;
        }
    }

    // The projection pass (reference SolveCollision): whoever would
    // land inside a body this step gets its velocity redirected to a
    // point one slop above the surface instead. Anti-tunneling for
    // water; the chain one-sided law rides inside the ray kernel.
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 p1 = world->particles.particlePositions[i];
        m2Vec2 move = {world->particles.particleVelocities[i].x * dt,
                       world->particles.particleVelocities[i].y * dt};
        m2AABB box;
        box.lowerBound.x = p1.x + (move.x < 0.0f ? (double)move.x : 0.0);
        box.lowerBound.y = p1.y + (move.y < 0.0f ? (double)move.y : 0.0);
        box.upperBound.x = p1.x + (move.x > 0.0f ? (double)move.x : 0.0);
        box.upperBound.y = p1.y + (move.y > 0.0f ? (double)move.y : 0.0);

        float bestFraction = 1.0f;
        int32_t bestShape = -1;
        m2Vec2 bestNormal = {0.0f, 0.0f};
        for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
        {
            int32_t results[M2_PARTICLE_CANDIDATES];
            int32_t hits = m2TreeQuery(&world->broadphase.trees[t], world->broadphase.treeNodes[t],
                                       box, results, M2_PARTICLE_CANDIDATES);
            hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
            for (int32_t h = 0; h < hits; ++h)
            {
                int32_t shape = results[h];
                if (world->shapes.shapeSensor[shape] != 0)
                {
                    continue;
                }
                struct m2CastHitInternal hit = m2RayCastShapeIndex(world, shape, p1, move, 1.0f);
                if (!hit.hit || (hit.normal.x == 0.0f && hit.normal.y == 0.0f))
                {
                    continue; // misses and initial overlaps (pressure owns those)
                }
                if (hit.fraction < bestFraction ||
                    (hit.fraction == bestFraction && shape < bestShape))
                {
                    bestFraction = hit.fraction;
                    bestShape = shape;
                    bestNormal = hit.normal;
                }
            }
        }
        if (bestShape >= 0)
        {
            double tx = p1.x + (double)(bestFraction * move.x) + (double)(0.005f * bestNormal.x);
            double ty = p1.y + (double)(bestFraction * move.y) + (double)(0.005f * bestNormal.y);
            world->particles.particleVelocities[i].x = (float)(tx - p1.x) * invDt;
            world->particles.particleVelocities[i].y = (float)(ty - p1.y) * invDt;
            // The projection stays a pure clamp on the particle, like
            // the reference's SolveCollision (its body push arrives
            // through the contact loops above); the reference's
            // lost-momentum re-add force is deliberately omitted.
        }
    }

    // Advance, single f64 crossing per axis.
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        world->particles.particlePositions[i].x +=
            (double)(world->particles.particleVelocities[i].x * dt);
        world->particles.particlePositions[i].y +=
            (double)(world->particles.particleVelocities[i].y * dt);
    }
}
