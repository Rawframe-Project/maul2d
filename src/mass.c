// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mass properties of a body, summed from its shapes.

#include "mass.h"

#include "geometry.h"

// Deterministic: walks the body's shape list in stored (insertion) order,
// which the snapshot preserves. Dynamic bodies get the minimum-mass floor
// (a zero-density dynamic body must not get an infinite inverse mass).
m2BodyMass m2BodyMassFromShapes(uint8_t type, bool fixedRotation, const m2MassShapes* shapes)
{
    m2BodyMass result = {0.0f, 0.0f, {0.0f, 0.0f}};
    if (type != (uint8_t)m2_dynamicBody)
    {
        return result;
    }

    // First pass: total mass and the mass-weighted center. Shape mass now
    // reports inertia about the SHAPE centroid, so the shift to the body
    // COM below is a sum of non-negative parallel-axis terms with no
    // big-minus-big cancellation.
    float mass = 0.0f;
    m2Vec2 center = {0.0f, 0.0f};
    for (int32_t s = shapes->head; s != -1; s = shapes->next(shapes->context, s))
    {
        m2MassData data = shapes->mass(shapes->context, s);
        mass += data.mass;
        center.x += data.mass * data.center.x;
        center.y += data.mass * data.center.y;
    }

    if (!(mass > 0.0f))
    {
        // Floor: shapeless or zero-density dynamic bodies weigh 1 kg and
        // do not rotate from impulses (deterministic fallback, no NaN).
        result.invMass = 1.0f;
        return result;
    }

    float invMass = 1.0f / mass;
    center.x *= invMass;
    center.y *= invMass;
    // The mass the body keeps is held to [M2_MIN_MASS, M2_MAX_MASS] and its
    // inertia to [M2_MIN_INERTIA, M2_MAX_INERTIA], the ranges the solver is
    // budgeted for; the center stays the mass-weighted mean of the shapes.
    // Within the ranges the clamps change no bit, and m2Body_GetMassData
    // reads back what the body kept.
    float keptInvMass = 1.0f / m2ClampF(mass, M2_MIN_MASS, M2_MAX_MASS);
    // Every shape lies within M2_HUGE, so the true center does too; the
    // rounding of the weighted mean can still carry it an ulp past, and
    // the snapshot holds centers to the bound.
    center.x = m2ClampF(center.x, -M2_HUGE, M2_HUGE);
    center.y = m2ClampF(center.y, -M2_HUGE, M2_HUGE);

    // Second pass: accumulate inertia about the body COM. Each shape's
    // centroid inertia plus mass times the (small) offset from the COM,
    // in the same canonical shape-list order.
    float inertiaCenter = 0.0f;
    for (int32_t s = shapes->head; s != -1; s = shapes->next(shapes->context, s))
    {
        m2MassData data = shapes->mass(shapes->context, s);
        float ox = center.x - data.center.x;
        float oy = center.y - data.center.y;
        inertiaCenter += data.centralInertia + data.mass * (ox * ox + oy * oy);
    }
    result.invMass = keptInvMass;
    result.invInertia = !fixedRotation && inertiaCenter > 0.0f
                            ? 1.0f / m2ClampF(inertiaCenter, M2_MIN_INERTIA, M2_MAX_INERTIA)
                            : 0.0f;
    result.center = center;
    return result;
}

bool m2MassInversesValid(float invMass, float invInertia)
{
    // Rounding to nearest is monotonic, so a mass within its range always
    // has an inverse within these bounds.
    return m2WithinF(invMass, 1.0f / M2_MAX_MASS, 1.0f / M2_MIN_MASS) &&
           (invInertia == 0.0f ||
            m2WithinF(invInertia, 1.0f / M2_MAX_INERTIA, 1.0f / M2_MIN_INERTIA));
}

static int32_t WorldNext(const void* context, int32_t shape)
{
    return ((const m2World*)context)->shapes.shapeNext[shape];
}

static m2MassData WorldShapeMass(const void* context, int32_t shape)
{
    const m2World* world = context;
    return m2ComputeShapeMass(&world->shapes.shapeGeometry[shape],
                              world->shapes.shapeDensity[shape]);
}

void m2RecomputeMass(m2World* world, int32_t bodyIndex)
{
    m2MassShapes shapes = {world, world->bodies.bodyShapeHead[bodyIndex], WorldNext,
                           WorldShapeMass};
    m2BodyMass mass = m2BodyMassFromShapes(world->bodies.types[bodyIndex],
                                           world->bodies.fixedRotations[bodyIndex] != 0, &shapes);
    world->bodies.invMass[bodyIndex] = mass.invMass;
    world->bodies.invInertia[bodyIndex] = mass.invInertia;
    // The body now rotates about this point: velocities, solver
    // anchors and integration all live in the COM frame.
    world->bodies.localCenters[bodyIndex] = mass.center;
    world->bodies.massOverrides[bodyIndex] = 0;
}
