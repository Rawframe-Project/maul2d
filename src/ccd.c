// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bullet continuous collision: conservative advancement
// per substep for bullet-flagged bodies. Deterministic by construction:
// bullets advance in body-index order, candidates come from tree queries
// filtered and processed in canonical shape order, iteration counts are
// capped, and hitting the cap is documented behavior (slight overlap
// next substep, which the speculative solver then resolves). Bullets do
// not sweep against other bullets.
//
// Simplifications: the bullet sweeps as its bounding circle (exact for
// circle shapes, conservative for the rest), and rotation during the
// sweep is ignored.

#include "ccd.h"

#include "world_internal.h"

#include "maul2d/base.h"

#define M2_TOI_ITERATIONS 20
#define M2_TOI_TARGET     0.005f // stop one slop short of the surface
#define M2_TOI_CANDIDATES 64

static float BulletBoundingRadius(const m2World* world, int32_t body)
{
    float radius = 0.0f;
    for (int32_t s = world->bodies.bodyShapeHead[body]; s != -1; s = world->shapes.shapeNext[s])
    {
        const m2ShapeGeometry* g = &world->shapes.shapeGeometry[s];
        switch (g->type)
        {
        case m2_circleShape:
        {
            float r = sqrtf(g->circle.center.x * g->circle.center.x +
                            g->circle.center.y * g->circle.center.y) +
                      g->circle.radius;
            radius = m2MaxF(radius, r);
            break;
        }
        case m2_capsuleShape:
        {
            float r1 = sqrtf(g->capsule.point1.x * g->capsule.point1.x +
                             g->capsule.point1.y * g->capsule.point1.y);
            float r2 = sqrtf(g->capsule.point2.x * g->capsule.point2.x +
                             g->capsule.point2.y * g->capsule.point2.y);
            radius = m2MaxF(radius, m2MaxF(r1, r2) + g->capsule.radius);
            break;
        }
        case m2_segmentShape:
        case m2_chainSegmentShape:
        {
            const m2Segment* seg =
                g->type == m2_segmentShape ? &g->segment : &g->chainSegment.segment;
            float r1 = sqrtf(seg->point1.x * seg->point1.x + seg->point1.y * seg->point1.y);
            float r2 = sqrtf(seg->point2.x * seg->point2.x + seg->point2.y * seg->point2.y);
            radius = m2MaxF(radius, m2MaxF(r1, r2));
            break;
        }
        default:
        {
            for (int32_t v = 0; v < g->polygon.count; ++v)
            {
                float r = sqrtf(g->polygon.vertices[v].x * g->polygon.vertices[v].x +
                                g->polygon.vertices[v].y * g->polygon.vertices[v].y);
                radius = m2MaxF(radius, r + g->polygon.radius);
            }
            break;
        }
        }
    }
    return radius;
}

// Distance from the bullet's bounding circle (center at world point wp)
// to a target shape, in the target body's frame (single f64 crossing).
static float DistanceToShape(const m2World* world, int32_t shape, m2Pos2 wp, float bulletRadius)
{
    int32_t body = world->shapes.shapeBody[shape];
    m2Transform xf = world->bodies.transforms[body];
    float dx = (float)(wp.x - xf.p.x);
    float dy = (float)(wp.y - xf.p.y);
    m2Vec2 local = {xf.q.c * dx + xf.q.s * dy, -xf.q.s * dx + xf.q.c * dy};
    return m2PointShapeDistance(&world->shapes.shapeGeometry[shape], local) - bulletRadius;
}

// Whether a shape the sweep overlaps may stop the bullet body: not a
// sensor, not on the ghost side of a one-way chain, not the bullet itself
// or another bullet, and allowed by the collision filters.
static bool BulletMayHit(const m2World* world, int32_t body, int32_t shape)
{
    if (world->shapes.shapeSensor[shape] != 0)
    {
        return false; // sensors never block a bullet
    }
    if (world->shapes.shapeGeometry[shape].type == m2_chainSegmentShape)
    {
        // One-way platforms are one-way for bullets too: a
        // sweep that begins on the ghost side never blocks.
        // Same sign law as the contact and ray paths.
        const m2ChainSegment* link = &world->shapes.shapeGeometry[shape].chainSegment;
        int32_t chainBody = world->shapes.shapeBody[shape];
        m2Transform cxf = world->bodies.transforms[chainBody];
        m2Pos2 start = world->solver.ccdPrevPositions[body];
        m2Vec2 rel = {(float)(start.x - cxf.p.x), (float)(start.y - cxf.p.y)};
        m2Vec2 local = {cxf.q.c * rel.x + cxf.q.s * rel.y, -cxf.q.s * rel.x + cxf.q.c * rel.y};
        m2Vec2 e = {link->segment.point2.x - link->segment.point1.x,
                    link->segment.point2.y - link->segment.point1.y};
        float offset =
            (local.x - link->segment.point1.x) * e.y - (local.y - link->segment.point1.y) * e.x;
        if (offset < 0.0f)
        {
            return false;
        }
    }
    int32_t other = world->shapes.shapeBody[shape];
    if (other == body || world->bodies.bullets[other] != 0)
    {
        return false; // self, or bullet-vs-bullet (excluded)
    }
    // Collision filters apply to bullets too: keep the
    // candidate only if some bullet shape may hit it.
    bool mayCollide = false;
    for (int32_t bs = world->bodies.bodyShapeHead[body]; bs != -1; bs = world->shapes.shapeNext[bs])
    {
        int32_t group = world->shapes.shapeGroup[bs];
        if (group != 0 && group == world->shapes.shapeGroup[shape])
        {
            mayCollide = mayCollide || group > 0;
            continue;
        }
        mayCollide = mayCollide ||
                     ((world->shapes.shapeCategory[bs] & world->shapes.shapeMask[shape]) != 0 &&
                      (world->shapes.shapeCategory[shape] & world->shapes.shapeMask[bs]) != 0);
    }
    return mayCollide;
}

// The lowest M2_TOI_CANDIDATES shape slots in the sweep that may stop the
// bullet, in ascending order whatever the tree's shape. Returns the count.
static int32_t SweepCandidates(const m2World* world, int32_t body, m2Aabb sweep,
                               int32_t* candidates)
{
    int32_t count = 0;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        m2TreeCursor cursor;
        m2TreeBeginQuery(&cursor, &world->broadphase.trees[t], world->broadphase.treeNodes[t],
                         sweep);
        int32_t shape;
        while (m2TreeNextQuery(&cursor, &shape))
        {
            if (BulletMayHit(world, body, shape))
            {
                m2TreeKeepLowest(candidates, &count, M2_TOI_CANDIDATES, shape);
            }
        }
    }
    return count;
}

// Sweep one bullet from p0 toward its current position; clamp on impact.
static void SweepBullet(m2World* world, int32_t body, m2Pos2 p0)
{
    m2Pos2 p1 = world->bodies.transforms[body].p;
    double mx = p1.x - p0.x;
    double my = p1.y - p0.y;
    float moveLen = sqrtf((float)(mx * mx + my * my));
    float bulletRadius = BulletBoundingRadius(world, body);
    if (moveLen < 0.5f * bulletRadius || bulletRadius == 0.0f)
    {
        return; // slow enough for the speculative pipeline
    }

    // Swept AABB over the whole motion, fattened by the bounding circle.
    m2Aabb sweep;
    sweep.lowerBound.x = (p0.x < p1.x ? p0.x : p1.x) - (double)bulletRadius;
    sweep.lowerBound.y = (p0.y < p1.y ? p0.y : p1.y) - (double)bulletRadius;
    sweep.upperBound.x = (p0.x > p1.x ? p0.x : p1.x) + (double)bulletRadius;
    sweep.upperBound.y = (p0.y > p1.y ? p0.y : p1.y) + (double)bulletRadius;

    int32_t candidates[M2_TOI_CANDIDATES];
    int32_t candidateCount = SweepCandidates(world, body, sweep, candidates);
    if (candidateCount == 0)
    {
        return;
    }

    // Conservative advancement to the earliest impact across candidates.
    float minT = 1.0f;
    for (int32_t c = 0; c < candidateCount; ++c)
    {
        float t = 0.0f;
        for (int32_t iter = 0; iter < M2_TOI_ITERATIONS; ++iter)
        {
            m2Pos2 wp = {p0.x + (double)t * mx, p0.y + (double)t * my};
            float distance = DistanceToShape(world, candidates[c], wp, bulletRadius);
            if (distance < M2_TOI_TARGET)
            {
                if (t < minT)
                {
                    minT = t;
                }
                break;
            }
            float advance = (distance - 0.5f * M2_TOI_TARGET) / moveLen;
            t += advance;
            if (t >= minT || t >= 1.0f)
            {
                break; // no earlier impact on this candidate
            }
        }
    }

    if (minT < 1.0f)
    {
        // Clamp position to the impact time. Velocity is left alone: the
        // next substep's speculative contact resolves the collision.
        world->bodies.transforms[body].p =
            (m2Pos2){p0.x + (double)minT * mx, p0.y + (double)minT * my};
        world->solver.deltaPositions[body].x -= (1.0f - minT) * (float)mx;
        world->solver.deltaPositions[body].y -= (1.0f - minT) * (float)my;
    }
}

// Called after each substep's position integration: the last pass of the
// substep that moves transforms.
void m2SolveContinuous(m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.bullets[i] == 0 ||
            world->bodies.asleep[i] != 0 || world->bodies.disabled[i] != 0 ||
            world->bodies.types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        SweepBullet(world, i, world->solver.ccdPrevPositions[i]);
    }
}
