// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A contact constraint as the solver stages see it, and the stages.

#ifndef MAUL2D_SRC_CONTACT_CONSTRAINT_H
#define MAUL2D_SRC_CONTACT_CONSTRAINT_H

#include "solver_base.h"

typedef struct m2ContactPoint
{
    m2Vec2 armA; // from each center of mass, world-rotated at prepare
    m2Vec2 armB;
    float gap;      // separation minus the arms' normal gap at prepare
    float approach; // normal speed at prepare, negative when closing
    float normalMass;
    float tangentMass;
    float normalImpulse;
    float tangentImpulse;
} m2ContactPoint;

typedef struct m2ContactConstraint
{
    int32_t pairIndex;
    int32_t bodyA;
    int32_t bodyB;
    // Masses as this pair sees them: dominance zeroes the stronger side.
    float invMassA;
    float invIA;
    float invMassB;
    float invIB;
    m2Vec2 normal; // world frame, A toward B
    float friction;
    float restitution;
    float beltSpeed; // tangent speed of both surfaces together
    m2Softness softness;
    int32_t pointCount;
    m2ContactPoint points[2];
} m2ContactConstraint;

typedef enum m2ContactStage
{
    m2_stageWarmStart,
    m2_stageSolve,       // soft rows push overlap out
    m2_stageRelax,       // rigid rows, no push: removes the push's speed
    m2_stageRestitution, // bounce, once per step
    m2_stageStore,       // impulses back to the manifolds
} m2ContactStage;

#endif // MAUL2D_SRC_CONTACT_CONSTRAINT_H
