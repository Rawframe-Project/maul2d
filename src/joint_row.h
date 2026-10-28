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

#include "solver_base.h"

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

// A safety bound, like the solver's speed caps: no accumulated impulse
// may change its row's speed by more than this, nor pass M2_IMPULSE_LIMIT.
// Rows that contradict each other (two joints that cannot both hold)
// fight through the softness leak and grow their impulses without limit;
// this holds them finite instead of letting them overflow into a NaN.
// Ordinary scenes never come near it.
#define M2_ROW_SPEED_BOUND 1.0e8f

// The largest coupling a 2x2 solve multiplies by another: a row whose
// effective mass is below 1e-19 is solved alone instead, so k11 k22 stays
// within a float.
#define M2_PAIR_COUPLING_MAX 1.0e19f

// A limit row's gap is held within this before it becomes a speed: every
// real gap is within the reach plus a lever, and an open-ended limit
// (the distance joint's unbounded length) stays finite.
#define M2_LIMIT_GAP_MAX (4.0f * (float)M2_REACH_LIMIT)

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

// Holds both bodies' velocities within M2_SPEED_LIMIT, after every push.
static inline void m2SaturateBodies(m2JointBodies* b)
{
    b->vA.x = m2SaturateF(b->vA.x, M2_SPEED_LIMIT);
    b->vA.y = m2SaturateF(b->vA.y, M2_SPEED_LIMIT);
    b->wA = m2SaturateF(b->wA, M2_SPEED_LIMIT);
    b->vB.x = m2SaturateF(b->vB.x, M2_SPEED_LIMIT);
    b->vB.y = m2SaturateF(b->vB.y, M2_SPEED_LIMIT);
    b->wB = m2SaturateF(b->wB, M2_SPEED_LIMIT);
}

static inline void m2PushRow(const m2JointRow* row, m2JointBodies* b, float impulse)
{
    b->vA.x += b->mA * impulse * row->linA.x;
    b->vA.y += b->mA * impulse * row->linA.y;
    b->wA += b->iA * impulse * row->angA;
    b->vB.x += b->mB * impulse * row->linB.x;
    b->vB.y += b->mB * impulse * row->linB.y;
    b->wB += b->iB * impulse * row->angB;
    m2SaturateBodies(b);
}

// Solves one row with its accumulated impulse held in [lo, hi] and
// within M2_ROW_SPEED_BOUND. Returns the impulse applied.
float m2SolveRow(const m2JointRow* row, m2JointBodies* b, m2RowDrive drive, float* accumulated,
                 float lo, float hi);

// Solves two coupled rows, both driven with the same scale and leak and
// each with its own bias. A singular pair (a body with fixed rotation
// under an angle row, say) falls back to the rows one at a time.
void m2SolveRowPair(const m2JointRow rows[2], m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                    m2Vec2* accumulated);

// Solves the point pair (the two rows that pin B's anchor to A's, along x
// and y) at the given arms, the ones the bodies have turned to, with the
// accumulated impulse held within a circle of radius budget (M2_ROW_FREE
// for none). A singular pair does nothing.
void m2SolvePointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                      m2Vec2* accumulated, float budget);
void m2PushPointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 impulse);

// Pulls B's anchor toward A's with a spring of the same stiffness in every
// direction, k = mass w^2 with damping c = 2 mass zeta w, the mass being
// B's own. Integrated implicitly over the substep h, the pair's impulse P
// meets Cdot + biasRate C + gamma P = 0, with gamma = 1 / (h (c + h k)):
// each pass adds -(K + gamma I)^-1 (Cdot + biasRate C + gamma P). The
// caller gives gamma per inverse mass, 1 / (h w (2 zeta + h w)), as
// compliance, and the m2MakeSoft bias rate; a compliance of 0 is no
// spring. The softness rows scale the pull by the pair's own mass instead,
// which is stiff along a long arm and slack across it; when the arm turns
// fast that stiffness turns with it and can pump the swing.
void m2SolveSpringPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 gap, float biasRate,
                       float compliance, m2Vec2* accumulated, float budget);

// Solves a rigid weld's three rows as one block: the point pair and the
// angle row (B's spin minus A's), all driven with the same scale and leak.
// Solved in turn, the two converge slowly when B's arm is long against
// its inertia, since they are then coupled almost one to one. Returns
// false, changing nothing, when the block is singular (neither body
// turns); the caller then solves the rows apart.
bool m2SolveWeldBlock(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, float angleBias,
                      m2RowDrive drive, m2Vec2* accumulated, float* accumulatedAngle);

// Pushes a weld's point impulse and angle impulse together, each body's
// spin changed once by their summed torque.
void m2PushWeldBlock(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 impulse, float angular);

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
    C = m2MinF(C, M2_LIMIT_GAP_MAX);
    return C > 0.0f ? m2RigidDrive(C * invH) : m2HeldDrive(soft, C, biased);
}

// Relative angle of B against A from their rotations, through the
// engine's own deterministic atan2.
float m2RelativeJointAngle(m2Rot qA, m2Rot qB);

#endif // MAUL2D_SRC_JOINT_ROW_H
