// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact constraints: building them from the manifolds, grouping them
// into lane blocks by graph color, and running the solver stages.

#ifndef MAUL2D_SRC_CONTACT_SOLVER_H
#define MAUL2D_SRC_CONTACT_SOLVER_H

#include "solver.h"

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

// A step's contact work: the constraints, their colors and blocks.
typedef struct m2ContactPlan
{
    m2ContactConstraint* constraints;
    int32_t count;
    int32_t colorStart[M2_GRAPH_COLORS + 2]; // the last range is the overflow
    int32_t blockStart[M2_GRAPH_COLORS + 1];
    float invH;
    bool reversed; // points in reverse order this substep
} m2ContactPlan;

int32_t m2PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h);

// Colors the prepared constraints and packs the colored ones into blocks.
void m2PlanContacts(m2World* world, m2ContactPlan* plan);

// Colors run block-parallel in color order; the overflow runs serially.
void m2RunContactStage(m2World* world, const m2ContactPlan* plan, m2ContactStage stage);

// sizeof(m2ContactConstraint): the scratch block is sized by its owner.
int32_t m2ContactConstraintSize(void);

// Bytes of the block scratch for a pair capacity.
int32_t m2ContactBlockScratchBytes(int32_t pairCapacity);

#endif // MAUL2D_SRC_CONTACT_SOLVER_H
