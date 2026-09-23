// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact kernel: lane blocks of contact constraints and the one
// kernel that runs every contact stage on them.

#ifndef MAUL2D_SRC_CONTACT_KERNEL_H
#define MAUL2D_SRC_CONTACT_KERNEL_H

#include "contact_solver.h"

// Lane width is a layout constant, never a semantics knob: every lane
// runs the same IEEE sequence, so how constraints are packed into lanes
// cannot move a bit.
#define M2_LANES 8

// One contact point of every lane.
typedef struct m2LanePoint
{
    float armAX[M2_LANES];
    float armAY[M2_LANES];
    float armBX[M2_LANES];
    float armBY[M2_LANES];
    float gap[M2_LANES];
    float approach[M2_LANES];
    float normalMass[M2_LANES];
    float tangentMass[M2_LANES];
    float normalImpulse[M2_LANES];
    float tangentImpulse[M2_LANES];
} m2LanePoint;

// Up to M2_LANES constraints with the same point count that share no
// dynamic body. Unused lanes are inert: they point at the static dummy
// body slot (index bodyCapacity), carry no mass and stay separated.
typedef struct m2ContactBlock
{
    int32_t lanes;
    int32_t pointCount;
    int32_t bodyA[M2_LANES];
    int32_t bodyB[M2_LANES];
    int32_t pairIndex[M2_LANES];
    float invMassA[M2_LANES];
    float invIA[M2_LANES];
    float invMassB[M2_LANES];
    float invIB[M2_LANES];
    float normalX[M2_LANES];
    float normalY[M2_LANES];
    float friction[M2_LANES];
    float restitution[M2_LANES];
    float beltSpeed[M2_LANES];
    float pushRate[M2_LANES];
    float massScale[M2_LANES];
    float impulseScale[M2_LANES];
    m2LanePoint points[2];
} m2ContactBlock;

void m2PackContactLane(m2ContactBlock* block, int32_t lane, const m2ContactConstraint* c);
void m2PadContactLane(m2ContactBlock* block, int32_t lane, int32_t dummyBody);
void m2UnpackContactLane(const m2ContactBlock* block, int32_t lane, m2ContactConstraint* c);

// Runs one stage on one block. The block's constraints share no dynamic
// body, so blocks of one color may run at once.
void m2RunContactBlock(m2World* world, m2ContactBlock* block, m2ContactStage stage, float invH,
                       bool reversed);

#endif // MAUL2D_SRC_CONTACT_KERNEL_H
