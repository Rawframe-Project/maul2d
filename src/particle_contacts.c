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
// Each particle tests every non-sensor shape whose box overlaps its own,
// as the broadphase trees yield them, and keeps the touching shapes with
// the lowest slots; every touch past them counts as a drop. What it keeps
// depends only on which shapes touch, not on the order the trees yield
// them, so the contacts are canonical: by particle, then by shape. Large
// pools stage the particles in parallel ranges; the gather that follows
// runs serially in particle order, so the worker count never changes a
// bit.

#include "body.h"
#include "distance.h"
#include "dynamic_tree.h"
#include "island.h"
#include "parallel.h"
#include "particle.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

#define M2_CONTACTS_PER_PARTICLE 4

// Staging a pool this large pays for the fork and join.
#define M2_FLUID_THREAD_MIN 4096

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// A staged contact slot.
typedef struct Touch
{
    float weight;
    m2Vec2 normal;
    float mass;
} Touch;

// The contact between particle p and shape, if any.
M2_NODISCARD static bool TouchShape(const m2World* world, m2Pos2 p, int32_t shape, float h,
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

// A particle's kept touches, ascending by shape slot.
typedef struct Kept
{
    int32_t count;
    int32_t drops;
    int32_t shapes[M2_CONTACTS_PER_PARTICLE];
    Touch touches[M2_CONTACTS_PER_PARTICLE];
} Kept;

// Keeps a touch if its shape is among the lowest slots so far; the touch
// left out, this one or the highest kept, counts as a drop.
static void KeepTouch(Kept* kept, int32_t shape, Touch touch)
{
    int32_t n = kept->count;
    if (n == M2_CONTACTS_PER_PARTICLE)
    {
        kept->drops += 1;
        if (shape > kept->shapes[n - 1])
        {
            return;
        }
        n -= 1; // the highest makes room
    }
    int32_t i = n;
    for (; i > 0 && kept->shapes[i - 1] > shape; --i)
    {
        kept->shapes[i] = kept->shapes[i - 1];
        kept->touches[i] = kept->touches[i - 1];
    }
    kept->shapes[i] = shape;
    kept->touches[i] = touch;
    kept->count = n + 1;
}

// Every non-sensor shape in the box that touches the particle at.
static void TouchShapes(const m2World* world, m2Pos2 at, float h, float particleInvMass, Kept* kept)
{
    m2Aabb box = {{at.x - (double)h, at.y - (double)h}, {at.x + (double)h, at.y + (double)h}};
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t], box);
        int32_t shape;
        while (m2TreeNextQuery(&cursor, &shape))
        {
            Touch touch;
            if (world->shapes.shapeSensor[shape] == 0 &&
                TouchShape(world, at, shape, h, particleInvMass, &touch))
            {
                KeepTouch(kept, shape, touch);
            }
        }
    }
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
        Kept kept = {0};
        TouchShapes(world, p->particlePositions[i], h, particleInvMass, &kept);
        p->particleBodyStageDrops[i] = kept.drops;
        for (int32_t k = 0; k < kept.count; ++k)
        {
            int32_t slot = i * M2_CONTACTS_PER_PARTICLE + k;
            p->particleBodyStageBody[slot] = world->shapes.shapeBody[kept.shapes[k]];
            p->particleBodyStageWeight[slot] = kept.touches[k].weight;
            p->particleBodyStageNormal[slot] = kept.touches[k].normal;
            p->particleBodyStageMass[slot] = kept.touches[k].mass;
        }
    }
}

// Appends the staged contacts in particle order; a touched body wakes.
static void Gather(m2World* world)
{
    m2Particles* p = &world->particles;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        m2CountDrops(&p->particleBodyContactsDropped, p->particleBodyStageDrops[i]);
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
                m2CountDrops(&p->particleBodyContactsDropped, 1);
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
    world->particles.particleBodyContactsDropped = 0;
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
