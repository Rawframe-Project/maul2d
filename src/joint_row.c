// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows: building, measuring, applying and solving them.

#include "joint_row.h"

#include "math_functions.h"

#include "maul2d/base.h"

#include <math.h>

// J M^-1 J'^T for two rows.
static float Coupling(const m2JointRow* r, const m2JointRow* s, const m2JointBodies* b)
{
    return b->mA * (r->linA.x * s->linA.x + r->linA.y * s->linA.y) + b->iA * r->angA * s->angA +
           b->mB * (r->linB.x * s->linB.x + r->linB.y * s->linB.y) + b->iB * r->angB * s->angB;
}

float m2RowMass(const m2JointRow* row, const m2JointBodies* b)
{
    float k = Coupling(row, row, b);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

float m2SolveRow(const m2JointRow* row, m2JointBodies* b, m2RowDrive drive, float* accumulated,
                 float lo, float hi)
{
    float mass = m2RowMass(row, b);
    if (mass == 0.0f)
    {
        return 0.0f;
    }
    float old = *accumulated;
    float speed = m2RowSpeed(row, b);
    float next = old - mass * (drive.scale * speed + drive.bias) - drive.leak * old;
    float bound = m2MinF(mass * M2_ROW_SPEED_BOUND, M2_IMPULSE_LIMIT);
    next = m2ClampF(next, m2MaxF(lo, -bound), m2MinF(hi, bound));
    *accumulated = next;
    m2PushRow(row, b, next - old);
    return next - old;
}

// The unbounded impulse that meets both rows at once; solved is false
// when the pair is singular.
static m2Vec2 PairImpulse(const m2JointRow rows[2], const m2JointBodies* b, m2Vec2 bias,
                          m2RowDrive drive, m2Vec2 accumulated, bool* solved)
{
    float k11 = Coupling(&rows[0], &rows[0], b);
    float k22 = Coupling(&rows[1], &rows[1], b);
    // A coupling too large to multiply falls back to the rows one at a
    // time: k12 squared is at most k11 k22, so the det stays finite.
    *solved = k11 <= M2_PAIR_COUPLING_MAX && k22 <= M2_PAIR_COUPLING_MAX;
    float k12 = *solved ? Coupling(&rows[0], &rows[1], b) : 0.0f;
    float det = *solved ? k11 * k22 - k12 * k12 : 0.0f;
    *solved = *solved && det > 0.0f;
    if (!*solved)
    {
        return (m2Vec2){0.0f, 0.0f};
    }
    float x = drive.scale * m2RowSpeed(&rows[0], b) + bias.x;
    float y = drive.scale * m2RowSpeed(&rows[1], b) + bias.y;
    float inv = 1.0f / det;
    m2Vec2 impulse = {-inv * (k22 * x - k12 * y) - drive.leak * accumulated.x,
                      -inv * (k11 * y - k12 * x) - drive.leak * accumulated.y};
    // Each row's share held within the speed bound at the row's own mass.
    float boundX = m2MinF(M2_ROW_SPEED_BOUND / k11, M2_IMPULSE_LIMIT);
    float boundY = m2MinF(M2_ROW_SPEED_BOUND / k22, M2_IMPULSE_LIMIT);
    float nextX = accumulated.x + impulse.x;
    float nextY = accumulated.y + impulse.y;
    if (m2AbsF(nextX) > boundX)
    {
        impulse.x = m2ClampF(nextX, -boundX, boundX) - accumulated.x;
    }
    if (m2AbsF(nextY) > boundY)
    {
        impulse.y = m2ClampF(nextY, -boundY, boundY) - accumulated.y;
    }
    return impulse;
}

void m2SolveRowPair(const m2JointRow rows[2], m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                    m2Vec2* accumulated)
{
    bool solved;
    m2Vec2 impulse = PairImpulse(rows, b, bias, drive, *accumulated, &solved);
    if (solved)
    {
        // The sum can round a float past the bound the impulse was clamped
        // to; what is stored is held to M2_IMPULSE_LIMIT, which a restore
        // enforces.
        accumulated->x = m2SaturateF(accumulated->x + impulse.x, M2_IMPULSE_LIMIT);
        accumulated->y = m2SaturateF(accumulated->y + impulse.y, M2_IMPULSE_LIMIT);
        m2PushRow(&rows[0], b, impulse.x);
        m2PushRow(&rows[1], b, impulse.y);
        return;
    }
    m2RowDrive first = {bias.x, drive.scale, drive.leak};
    m2RowDrive second = {bias.y, drive.scale, drive.leak};
    m2SolveRow(&rows[0], b, first, &accumulated->x, -M2_ROW_FREE, M2_ROW_FREE);
    m2SolveRow(&rows[1], b, second, &accumulated->y, -M2_ROW_FREE, M2_ROW_FREE);
}

void m2PushPointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 impulse)
{
    b->vA.x -= b->mA * impulse.x;
    b->vA.y -= b->mA * impulse.y;
    b->wA -= b->iA * m2Cross2(armA, impulse);
    b->vB.x += b->mB * impulse.x;
    b->vB.y += b->mB * impulse.y;
    b->wB += b->iB * m2Cross2(armB, impulse);
    m2SaturateBodies(b);
}

// The inverse of the 2x2 mass the point pair shows at the given arms:
// {xx, xy, yy}, all zero when the pair is singular.
typedef struct m2PointMass
{
    double xx;
    double xy;
    double yy;
} m2PointMass;

// The mass is K = m I + iA a' a'^T + iB b' b'^T, with m the summed inverse
// masses and a', b' the arms turned a quarter. When an arm is long against
// its body's inertia, K's small eigenvalue m lies along that arm and turns
// with it, so K is built from the arms the solve is at, never kept from an
// earlier pose. It is built and inverted in double: in float, m falls
// below the rounding of iB |b|^2 once the ratio passes about 1e7, and the
// inverse loses the arm's direction. Its determinant is not k11 k22 -
// k12^2, which subtracts two near-equal squares: expanded, it is m^2 +
// m (iA |a|^2 + iB |b|^2) + iA iB (a x b)^2, a sum of terms that are never
// negative.
static inline m2PointMass PointMass(m2Vec2 armA, m2Vec2 armB, const m2JointBodies* b)
{
    double ax = (double)armA.x;
    double ay = (double)armA.y;
    double bx = (double)armB.x;
    double by = (double)armB.y;
    double iA = (double)b->iA;
    double iB = (double)b->iB;
    double m = (double)b->mA + (double)b->mB;
    double k11 = m + iA * ay * ay + iB * by * by;
    double k12 = -iA * ax * ay - iB * bx * by;
    double k22 = m + iA * ax * ax + iB * bx * bx;
    if (!(k11 <= (double)M2_PAIR_COUPLING_MAX && k22 <= (double)M2_PAIR_COUPLING_MAX))
    {
        return (m2PointMass){0.0, 0.0, 0.0}; // too stiff to multiply: skipped
    }
    double cross = ax * by - ay * bx;
    double det =
        m * m + m * (iA * (ax * ax + ay * ay) + iB * (bx * bx + by * by)) + iA * iB * cross * cross;
    if (!(det > 0.0))
    {
        return (m2PointMass){0.0, 0.0, 0.0};
    }
    double inv = 1.0 / det;
    return (m2PointMass){k22 * inv, -k12 * inv, k11 * inv};
}

// Holds an accumulated point pair impulse within a circle: the budget,
// the speed bound at the pair's largest axis mass (xx or yy of its
// inverse) and M2_IMPULSE_LIMIT, the smallest of them.
static m2Vec2 BoundPair(m2Vec2 next, double xx, double yy, float budget)
{
    float bound = M2_ROW_SPEED_BOUND * (float)(xx > yy ? xx : yy);
    float limit = m2MinF(m2MinF(budget, bound), M2_IMPULSE_LIMIT);
    float length2 = next.x * next.x + next.y * next.y;
    if (length2 > limit * limit)
    {
        // Scaled to the limit, a component can round a float past it; each
        // is held to M2_IMPULSE_LIMIT, which a restore enforces.
        float scale = limit / sqrtf(length2);
        next = (m2Vec2){m2SaturateF(scale * next.x, M2_IMPULSE_LIMIT),
                        m2SaturateF(scale * next.y, M2_IMPULSE_LIMIT)};
    }
    return next;
}

void m2SolvePointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                      m2Vec2* accumulated, float budget)
{
    m2PointMass mass = PointMass(armA, armB, b);
    if (mass.xx == 0.0 && mass.yy == 0.0)
    {
        return;
    }
    m2Vec2 speed = {b->vB.x - b->wB * armB.y - b->vA.x + b->wA * armA.y,
                    b->vB.y + b->wB * armB.x - b->vA.y - b->wA * armA.x};
    double x = (double)(drive.scale * speed.x + bias.x);
    double y = (double)(drive.scale * speed.y + bias.y);
    m2Vec2 old = *accumulated;
    // The impulse in double too: across a long arm it is far smaller than
    // along it, and a float inverse would round it away.
    m2Vec2 next = {(float)((double)old.x - (mass.xx * x + mass.xy * y)) - drive.leak * old.x,
                   (float)((double)old.y - (mass.xy * x + mass.yy * y)) - drive.leak * old.y};
    next = BoundPair(next, mass.xx, mass.yy, budget);
    *accumulated = next;
    m2PushPointPair(armA, armB, b, (m2Vec2){next.x - old.x, next.y - old.y});
}

void m2SolveSpringPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 gap, float biasRate,
                       float compliance, m2Vec2* accumulated, float budget)
{
    m2Vec2 old = *accumulated;
    // K + gamma I is the pair's mass with gamma added to the summed
    // inverse masses m, which keeps its determinant's terms non-negative.
    // A gamma past the coupling bound is a spring too weak to pull.
    float gamma = b->mB * compliance;
    m2JointBodies softened = *b;
    softened.mA += gamma;
    m2PointMass mass = PointMass(armA, armB, &softened);
    if (!(compliance > 0.0f) || (mass.xx == 0.0 && mass.yy == 0.0))
    {
        // No spring: what the warm start pushed is taken back.
        *accumulated = (m2Vec2){0.0f, 0.0f};
        m2PushPointPair(armA, armB, b, (m2Vec2){-old.x, -old.y});
        return;
    }
    m2Vec2 speed = {b->vB.x - b->wB * armB.y - b->vA.x + b->wA * armA.y,
                    b->vB.y + b->wB * armB.x - b->vA.y - b->wA * armA.x};
    double x = (double)speed.x + (double)(biasRate * gap.x) + (double)gamma * (double)old.x;
    double y = (double)speed.y + (double)(biasRate * gap.y) + (double)gamma * (double)old.y;
    m2Vec2 next = {(float)((double)old.x - (mass.xx * x + mass.xy * y)),
                   (float)((double)old.y - (mass.xy * x + mass.yy * y))};
    next = BoundPair(next, mass.xx, mass.yy, budget);
    *accumulated = next;
    m2PushPointPair(armA, armB, b, (m2Vec2){next.x - old.x, next.y - old.y});
}

// The inverse of the 3x3 mass a rigid weld shows at the given arms, its
// point pair and its angle row together; symmetric, so six entries.
typedef struct m2WeldMass
{
    double xx;
    double xy;
    double xa;
    double yy;
    double ya;
    double aa;
} m2WeldMass;

// The mass is K = m P + iA uA uA^T + iB uB uB^T, with P = diag(1, 1, 0)
// and u = (-r.y, r.x, 1) for each arm r. Like the pair's, it is built at
// the arms the solve is at and in double, and nothing in it subtracts
// near-equal products: det K = m^2 (iA + iB) + m iA iB |a - b|^2, and each
// cofactor is expanded the same way. Returns false when the block is
// singular (neither body turns) or too stiff to multiply.
static bool WeldMass(m2Vec2 armA, m2Vec2 armB, const m2JointBodies* b, m2WeldMass* out)
{
    double ax = (double)armA.x;
    double ay = (double)armA.y;
    double bx = (double)armB.x;
    double by = (double)armB.y;
    double iA = (double)b->iA;
    double iB = (double)b->iB;
    double m = (double)b->mA + (double)b->mB;
    double turn = iA + iB;
    double k11 = m + iA * ay * ay + iB * by * by;
    double k22 = m + iA * ax * ax + iB * bx * bx;
    double limit = (double)M2_PAIR_COUPLING_MAX;
    if (!(k11 <= limit && k22 <= limit && turn <= limit))
    {
        return false;
    }
    double dx = ax - bx;
    double dy = ay - by;
    double det = m * m * turn + m * iA * iB * (dx * dx + dy * dy);
    if (!(det > 0.0))
    {
        return false;
    }
    double cross = ax * by - ay * bx;
    double both = iA * iB;
    double inv = 1.0 / det;
    out->xx = (m * turn + both * dx * dx) * inv;
    out->xy = both * dx * dy * inv;
    out->xa = (m * (iA * ay + iB * by) + both * dx * cross) * inv;
    out->yy = (m * turn + both * dy * dy) * inv;
    out->ya = (both * dy * cross - m * (iA * ax + iB * bx)) * inv;
    out->aa =
        (m * m + m * (iA * (ax * ax + ay * ay) + iB * (bx * bx + by * by)) + both * cross * cross) *
        inv;
    return true;
}

// Pushes a weld's point impulse (x, y) and angle impulse a in one go. On a
// long arm against a small inertia the angle impulse and the point
// impulse's torque nearly cancel on B: pushed apart, B's spin would pass
// M2_MAX_SPEED in between and be clipped, and in float their sum would
// lose the rounding times B's inverse inertia. So each spin changes once,
// by the sum taken in double.
static void PushWeldBlock(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, double x, double y, double a)
{
    double torqueA = (double)armA.x * y - (double)armA.y * x + a;
    double torqueB = (double)armB.x * y - (double)armB.y * x + a;
    b->vA.x -= (float)((double)b->mA * x);
    b->vA.y -= (float)((double)b->mA * y);
    b->wA -= (float)((double)b->iA * torqueA);
    b->vB.x += (float)((double)b->mB * x);
    b->vB.y += (float)((double)b->mB * y);
    b->wB += (float)((double)b->iB * torqueB);
    m2SaturateBodies(b);
}

void m2PushWeldBlock(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 impulse, float angular)
{
    PushWeldBlock(armA, armB, b, (double)impulse.x, (double)impulse.y, (double)angular);
}

bool m2SolveWeldBlock(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, float angleBias,
                      m2RowDrive drive, m2Vec2* accumulated, float* accumulatedAngle)
{
    m2WeldMass mass;
    if (!WeldMass(armA, armB, b, &mass))
    {
        return false;
    }
    m2Vec2 speed = {b->vB.x - b->wB * armB.y - b->vA.x + b->wA * armA.y,
                    b->vB.y + b->wB * armB.x - b->vA.y - b->wA * armA.x};
    double x = (double)(drive.scale * speed.x + bias.x);
    double y = (double)(drive.scale * speed.y + bias.y);
    double a = (double)(drive.scale * (b->wB - b->wA) + angleBias);
    m2Vec2 old = *accumulated;
    float oldAngle = *accumulatedAngle;
    // The change is pushed as computed, in double: across a long arm the
    // angle row carries an impulse far larger than the change a substep
    // needs, and the difference of two rounded totals would leave B
    // spinning by the rounding times its inverse inertia.
    double cx = -(mass.xx * x + mass.xy * y + mass.xa * a) - (double)(drive.leak * old.x);
    double cy = -(mass.xy * x + mass.yy * y + mass.ya * a) - (double)(drive.leak * old.y);
    double ca = -(mass.xa * x + mass.ya * y + mass.aa * a) - (double)(drive.leak * oldAngle);
    m2Vec2 next = {old.x + (float)cx, old.y + (float)cy};
    m2Vec2 bounded = BoundPair(next, mass.xx, mass.yy, M2_ROW_FREE);
    float nextAngle = oldAngle + (float)ca;
    float angleBound = m2MinF(M2_ROW_SPEED_BOUND * (float)mass.aa, M2_IMPULSE_LIMIT);
    float boundedAngle = m2ClampF(nextAngle, -angleBound, angleBound);
    if (bounded.x != next.x || bounded.y != next.y || boundedAngle != nextAngle)
    {
        cx = (double)(bounded.x - old.x);
        cy = (double)(bounded.y - old.y);
        ca = (double)(boundedAngle - oldAngle);
    }
    *accumulated = bounded;
    *accumulatedAngle = boundedAngle;
    PushWeldBlock(armA, armB, b, cx, cy, ca);
    return true;
}

// Relative angle of B vs A from their rotations, through the engine's own
// deterministic atan2.
float m2RelativeJointAngle(m2Rot qA, m2Rot qB)
{
    float sin = qA.c * qB.s - qA.s * qB.c;
    float cos = qA.c * qB.c + qA.s * qB.s;
    return m2Atan2Work(sin, cos);
}
