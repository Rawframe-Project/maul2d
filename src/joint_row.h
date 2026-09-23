// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows. Every joint is a list of scalar rows, each a linear
// function J of the two bodies' velocities that the solver drives
// toward a target:
//   J v = linA . vA + angA wA + linB . vB + angB wB.
// An impulse lambda on a row changes the velocities by M^-1 J^T lambda.
// The kinds build their rows from the current pose every pass; this
// module solves them, alone or as a coupled pair.

#ifndef MAUL2D_SRC_JOINT_ROW_H
#define MAUL2D_SRC_JOINT_ROW_H

#include "solver.h"

// The two bodies of a joint as one pass sees them.
typedef struct m2JointBodies
{
    m2Vec2 vA;
    float wA;
    m2Vec2 vB;
    float wB;
    float mA;
    float iA;
    float mB;
    float iB;
} m2JointBodies;

typedef struct m2JointRow
{
    m2Vec2 linA;
    float angA;
    m2Vec2 linB;
    float angB;
} m2JointRow;

// How a row is driven in one pass. The impulse added is
//   -mass (scale J v + bias) - leak accumulated.
typedef struct m2RowDrive
{
    float bias;
    float scale;
    float leak;
} m2RowDrive;

// No bound on an accumulated impulse.
#define M2_ROW_FREE 3.0e38f

// B's point minus A's point moving along u, where u is fixed in the world
// (leverA = A's arm) or turns with A (leverA = A's arm plus the gap).
static inline m2JointRow m2LineRow(m2Vec2 leverA, m2Vec2 armB, m2Vec2 u)
{
    m2JointRow row = {{-u.x, -u.y}, -m2Cross2(leverA, u), u, m2Cross2(armB, u)};
    return row;
}

// B's spin minus A's.
static inline m2JointRow m2TurnRow(void)
{
    m2JointRow row = {{0.0f, 0.0f}, -1.0f, {0.0f, 0.0f}, 1.0f};
    return row;
}

static inline m2JointRow m2ScaleRow(m2JointRow row, float s)
{
    m2JointRow scaled = {{s * row.linA.x, s * row.linA.y},
                         s * row.angA,
                         {s * row.linB.x, s * row.linB.y},
                         s * row.angB};
    return scaled;
}

static inline float m2RowSpeed(const m2JointRow* row, const m2JointBodies* b)
{
    return row->linA.x * b->vA.x + row->linA.y * b->vA.y + row->angA * b->wA +
           row->linB.x * b->vB.x + row->linB.y * b->vB.y + row->angB * b->wB;
}

// The row's effective mass, zero when the row cannot move.
float m2RowMass(const m2JointRow* row, const m2JointBodies* b);

static inline void m2PushRow(const m2JointRow* row, m2JointBodies* b, float impulse)
{
    b->vA.x += b->mA * impulse * row->linA.x;
    b->vA.y += b->mA * impulse * row->linA.y;
    b->wA += b->iA * impulse * row->angA;
    b->vB.x += b->mB * impulse * row->linB.x;
    b->vB.y += b->mB * impulse * row->linB.y;
    b->wB += b->iB * impulse * row->angB;
}

// Solves one row with its accumulated impulse held in [lo, hi]. Returns
// the impulse applied.
float m2SolveRow(const m2JointRow* row, m2JointBodies* b, m2RowDrive drive, float* accumulated,
                 float lo, float hi);

// Solves two coupled rows, both driven with the same scale and leak and
// each with its own bias. A singular pair (a body with fixed rotation
// under an angle row, say) falls back to the rows one at a time.
void m2SolveRowPair(const m2JointRow rows[2], m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                    m2Vec2* accumulated);

// The two rows that pin B's anchor to A's (along x and y), solved as one
// pair in closed form with the accumulated impulse held within a circle
// of radius budget (M2_ROW_FREE for none). A singular pair does nothing.
void m2SolvePointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                      m2Vec2* accumulated, float budget);
void m2PushPointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 impulse);

// Drives. A rigid row meets bias exactly; a held row pulls its error C
// out through the softness while biased and is rigid while relaxing; a
// spring pulls in every pass; a limit is speculative while open
// (C > 0), held while violated.
static inline m2RowDrive m2RigidDrive(float bias)
{
    m2RowDrive drive = {bias, 1.0f, 0.0f};
    return drive;
}

static inline m2RowDrive m2SpringDrive(m2Softness soft, float C)
{
    m2RowDrive drive = {soft.massScale * soft.biasRate * C, soft.massScale, soft.impulseScale};
    return drive;
}

static inline m2RowDrive m2HeldDrive(m2Softness soft, float C, bool biased)
{
    return biased ? m2SpringDrive(soft, C) : m2RigidDrive(0.0f);
}

static inline m2RowDrive m2LimitDrive(m2Softness soft, float C, float invH, bool biased)
{
    return C > 0.0f ? m2RigidDrive(C * invH) : m2HeldDrive(soft, C, biased);
}

#endif // MAUL2D_SRC_JOINT_ROW_H
