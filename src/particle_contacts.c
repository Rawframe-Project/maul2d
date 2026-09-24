// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Particle contacts with bodies: every shape whose surface lies within
// one diameter of a particle, at most four per particle, with the
// contact's weight q = 1 - separation / h, the surface normal toward the
// particle, and the mass the particle and the body show each other along
// that normal. Sensors never touch particles, and a one-way chain link
// ignores particles on its ghost side.
//
// Each particle's candidates come from the broadphase trees and are
// taken in ascending shape order, so the contacts are canonical: by
// particle, then by shape. Large pools stage the particles in parallel
// ranges; the gather that follows runs serially in particle order, so
// the worker count never changes a bit.

#include "body.h"
#include "distance.h"
#include "dynamic_tree.h"
#include "particle.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

#define M2_PARTICLE_CANDIDATES   32
#define M2_CONTACTS_PER_PARTICLE 4

// Staging a pool this large pays for the fork and join.
#define M2_FLUID_THREAD_MIN 4096

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// The non-sensor shapes whose boxes overlap box, ascending. Returns the
// count.
static int32_t Candidates(const m2World* world, m2AABB box, int32_t* out)
{
    int32_t count = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t found[M2_PARTICLE_CANDIDATES];
        int32_t hits = m2TreeQuery(&world->broadphase.trees[t], world->broadphase.treeNodes[t], box,
                                   found, M2_PARTICLE_CANDIDATES);
        hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
        for (int32_t h = 0; h < hits && count < M2_PARTICLE_CANDIDATES; ++h)
        {
            if (world->shapes.shapeSensor[found[h]] == 0)
            {
                out[count++] = found[h];
            }
        }
    }
    for (int32_t a = 1; a < count; ++a)
    {
        int32_t key = out[a];
        int32_t b = a - 1;
        for (; b >= 0 && out[b] > key; --b)
        {
            out[b + 1] = out[b];
        }
        out[b + 1] = key;
    }
    return count;
}

// A staged contact slot.
typedef struct Touch
{
    float weight;
    m2Vec2 normal;
    float mass;
} Touch;

// The contact between particle p and shape, if any.
static bool TouchShape(const m2World* world, m2Pos2 p, int32_t shape, float h,
                       float particleInvMass, Touch* out)
{
    int32_t body = world->shapes.shapeBody[shape];
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 offset = {(float)(p.x - xf.p.x), (float)(p.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * offset.x + xf.q.s * offset.y, -xf.q.s * offset.x + xf.q.c * offset.y};
    const m2ShapeGeometry* g = &world->shapes.shapeGeometry[shape];
    if (g->type == m2_chainSegmentShape)
    {
        m2Vec2 a = g->chainSegment.segment.point1;
        m2Vec2 b = g->chainSegment.segment.point2;
        if ((local.x - a.x) * (b.y - a.y) - (local.y - a.y) * (b.x - a.x) < 0.0f)
        {
            return false; // the ghost side
        }
    }
    m2DistanceProxy surface = m2GeometryProxy(g);
    m2DistanceProxy point;
    memset(&point, 0, sizeof(point));
    point.points[0] = local;
    point.count = 1;
    m2DistanceResult d = m2ShapeDistance(&surface, &point);
    float separation = d.distance - surface.radius;
    if (separation >= h)
    {
        return false;
    }
    m2Vec2 n;
    if (d.normal.x != 0.0f || d.normal.y != 0.0f)
    {
        n = Rotate(xf.q, d.normal);
    }
    else
    {
        // Inside the core: away from the body origin, or up.
        float length = sqrtf(offset.x * offset.x + offset.y * offset.y);
        n = length > 1.0e-6f ? (m2Vec2){offset.x / length, offset.y / length}
                             : (m2Vec2){0.0f, 1.0f};
    }
    m2Vec2 center = Rotate(xf.q, world->bodies.localCenters[body]);
    float turn = (offset.x - center.x) * n.y - (offset.y - center.y) * n.x;
    float k = particleInvMass + world->bodies.invMass[body] +
              world->bodies.invInertia[body] * turn * turn;
    *out = (Touch){1.0f - separation / h, n, k > 0.0f ? 1.0f / k : 0.0f};
    return true;
}

static void StageRange(int32_t begin, int32_t end, void* context)
{
    m2World* world = (m2World*)context;
    m2Particles* p = &world->particles;
    float h = 2.0f * p->particleRadius;
    float stride = 0.75f * h;
    float particleInvMass = 1.0f / (p->particleDensity * stride * stride);
    for (int32_t i = begin; i < end; ++i)
    {
        p->particleBodyStageDrops[i] = 0;
        for (int32_t k = 0; k < M2_CONTACTS_PER_PARTICLE; ++k)
        {
            p->particleBodyStageBody[i * M2_CONTACTS_PER_PARTICLE + k] = -1;
        }
        if (p->particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 at = p->particlePositions[i];
        m2AABB box = {{at.x - (double)h, at.y - (double)h}, {at.x + (double)h, at.y + (double)h}};
        int32_t shapes[M2_PARTICLE_CANDIDATES];
        int32_t count = Candidates(world, box, shapes);
        int32_t kept = 0;
        for (int32_t c = 0; c < count; ++c)
        {
            Touch touch;
            if (!TouchShape(world, at, shapes[c], h, particleInvMass, &touch))
            {
                continue;
            }
            if (kept == M2_CONTACTS_PER_PARTICLE)
            {
                p->particleBodyStageDrops[i] += 1;
                continue;
            }
            int32_t slot = i * M2_CONTACTS_PER_PARTICLE + kept;
            p->particleBodyStageBody[slot] = world->shapes.shapeBody[shapes[c]];
            p->particleBodyStageWeight[slot] = touch.weight;
            p->particleBodyStageNormal[slot] = touch.normal;
            p->particleBodyStageMass[slot] = touch.mass;
            kept += 1;
        }
    }
}

// Appends the staged contacts in particle order; a touched body wakes.
static void Gather(m2World* world)
{
    m2Particles* p = &world->particles;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        p->particleBodyOverflow += p->particleBodyStageDrops[i];
        for (int32_t k = 0; k < M2_CONTACTS_PER_PARTICLE; ++k)
        {
            int32_t slot = i * M2_CONTACTS_PER_PARTICLE + k;
            int32_t body = p->particleBodyStageBody[slot];
            if (body < 0)
            {
                break;
            }
            if (p->particleBodyCount >= p->particleBodyCapacity)
            {
                p->particleBodyOverflow += 1;
                continue;
            }
            int32_t out = p->particleBodyCount;
            p->particleBodyCount = out + 1;
            p->particleBodyParticle[out] = i;
            p->particleBodyBody[out] = body;
            p->particleBodyWeight[out] = p->particleBodyStageWeight[slot];
            p->particleBodyNormal[out] = p->particleBodyStageNormal[slot];
            p->particleBodyMass[out] = p->particleBodyStageMass[slot];
            m2WakeIfDynamic(world, body);
        }
    }
}

void m2UpdateParticleContacts(m2World* world)
{
    world->particles.particleBodyCount = 0;
    world->particles.particleBodyOverflow = 0;
    if (world->particles.particleCount >= M2_FLUID_THREAD_MIN)
    {
        m2RunParallel(world, StageRange, world, world->particles.maxParticleIndex, 64);
    }
    else
    {
        StageRange(0, world->particles.maxParticleIndex, world);
    }
    Gather(world);
}
