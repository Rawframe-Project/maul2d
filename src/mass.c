// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mass properties of a body, summed from its shapes.

#include "mass.h"

#include "geometry.h"

// Deterministic: walks the body's shape list in stored (insertion) order,
// which the snapshot preserves. Dynamic bodies get the minimum-mass floor
// (a zero-density dynamic body must not get an infinite inverse mass).
void m2RecomputeMass(m2World* world, int32_t bodyIndex)
{
    if (world->bodies.types[bodyIndex] != (uint8_t)m2_dynamicBody)
    {
        world->bodies.invMass[bodyIndex] = 0.0f;
        world->bodies.invInertia[bodyIndex] = 0.0f;
        world->bodies.localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    // First pass: total mass and the mass-weighted center. Shape mass now
    // reports inertia about the SHAPE centroid, so the shift to the body
    // COM below is a sum of non-negative parallel-axis terms with no
    // big-minus-big cancellation.
    float mass = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        m2MassData data =
            m2ComputeShapeMass(&world->shapes.shapeGeometry[s], world->shapes.shapeDensity[s]);
        mass += data.mass;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        world->bodies.invMass[bodyIndex] = 1.0f;
        world->bodies.invInertia[bodyIndex] = 0.0f;
        world->bodies.localCenters[bodyIndex] = (m2Vec2){0.0f, 0.0f};
        return;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;

    // Second pass: accumulate inertia about the body COM. Each shape's
    // centroid inertia plus mass times the (small) offset from the COM,
    // in the same canonical shape-list order.
    float inertiaCenter = 0.0f;
    for (int32_t s = world->bodies.bodyShapeHead[bodyIndex]; s != -1;
         s = world->shapes.shapeNext[s])
    {
        m2MassData data =
            m2ComputeShapeMass(&world->shapes.shapeGeometry[s], world->shapes.shapeDensity[s]);
        float ox = center.x - data.center.x;
        float oy = center.y - data.center.y;
        inertiaCenter += data.rotationalInertia + data.mass * (ox * ox + oy * oy);
    }
    world->bodies.invMass[bodyIndex] = invMass;
    world->bodies.invInertia[bodyIndex] =
        world->bodies.fixedRotations[bodyIndex] == 0 && inertiaCenter > 0.0f ? 1.0f / inertiaCenter
                                                                             : 0.0f;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->bodies.localCenters[bodyIndex] = center;
}
