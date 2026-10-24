// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mass properties of a body, summed from its shapes.

#ifndef MAUL2D_SRC_MASS_H
#define MAUL2D_SRC_MASS_H

#include "world_internal.h"

// What a body keeps of its mass: the inverses and the center of mass.
typedef struct m2BodyMass
{
    float invMass;
    float invInertia;
    m2Vec2 center;
} m2BodyMass;

// A body's shapes as the mass sum walks them: the list from head, each
// shape's geometry and density read through the callbacks, so the same
// arithmetic runs on a world and on a snapshot's bytes.
typedef struct m2MassShapes
{
    const void* context;
    int32_t head;
    int32_t (*next)(const void* context, int32_t shape);
    m2MassData (*mass)(const void* context, int32_t shape);
} m2MassShapes;

// The mass a body of this type takes from its shapes, in list order:
// zero for static and kinematic bodies, a 1 kg floor without rotation for
// a dynamic body whose shapes weigh nothing, and otherwise the sum with
// its mass and inertia clamped into the ranges of base.h.
m2BodyMass m2BodyMassFromShapes(uint8_t type, bool fixedRotation, const m2MassShapes* shapes);

// Whether a body may keep these inverses as mass data of its own: the
// inverse of a mass within [M2_MIN_MASS, M2_MAX_MASS], and 0 or the
// inverse of an inertia within [M2_MIN_INERTIA, M2_MAX_INERTIA].
// m2Body_SetMassData and the snapshot check share it, so what one accepts
// the other does.
bool m2MassInversesValid(float invMass, float invInertia);

// Sets a body's inverse mass, inverse inertia and center of mass from its
// shapes, in shape-list order, and drops any override.
void m2RecomputeMass(m2World* world, int32_t bodyIndex);

#endif // MAUL2D_SRC_MASS_H
