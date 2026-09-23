// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint rows: building, measuring, applying and solving them.

#include "joint_row.h"

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
    next = m2ClampF(next, lo, hi);
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
    float k12 = Coupling(&rows[0], &rows[1], b);
    float k22 = Coupling(&rows[1], &rows[1], b);
    float det = k11 * k22 - k12 * k12;
    *solved = det > 0.0f;
    if (!*solved)
    {
        return (m2Vec2){0.0f, 0.0f};
    }
    float x = drive.scale * m2RowSpeed(&rows[0], b) + bias.x;
    float y = drive.scale * m2RowSpeed(&rows[1], b) + bias.y;
    float inv = 1.0f / det;
    return (m2Vec2){-inv * (k22 * x - k12 * y) - drive.leak * accumulated.x,
                    -inv * (k11 * y - k12 * x) - drive.leak * accumulated.y};
}

void m2SolveRowPair(const m2JointRow rows[2], m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                    m2Vec2* accumulated)
{
    bool solved;
    m2Vec2 impulse = PairImpulse(rows, b, bias, drive, *accumulated, &solved);
    if (solved)
    {
        accumulated->x += impulse.x;
        accumulated->y += impulse.y;
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
}

void m2SolvePointPair(m2Vec2 armA, m2Vec2 armB, m2JointBodies* b, m2Vec2 bias, m2RowDrive drive,
                      m2Vec2* accumulated, float budget)
{
    float m = b->mA + b->mB;
    float k11 = m + b->iA * armA.y * armA.y + b->iB * armB.y * armB.y;
    float k12 = -b->iA * armA.x * armA.y - b->iB * armB.x * armB.y;
    float k22 = m + b->iA * armA.x * armA.x + b->iB * armB.x * armB.x;
    float det = k11 * k22 - k12 * k12;
    if (!(det > 0.0f))
    {
        return;
    }
    m2Vec2 speed = {b->vB.x - b->wB * armB.y - b->vA.x + b->wA * armA.y,
                    b->vB.y + b->wB * armB.x - b->vA.y - b->wA * armA.x};
    float x = drive.scale * speed.x + bias.x;
    float y = drive.scale * speed.y + bias.y;
    float inv = 1.0f / det;
    m2Vec2 old = *accumulated;
    m2Vec2 next = {old.x - inv * (k22 * x - k12 * y) - drive.leak * old.x,
                   old.y - inv * (k11 * y - k12 * x) - drive.leak * old.y};
    if (budget < M2_ROW_FREE)
    {
        float length2 = next.x * next.x + next.y * next.y;
        if (length2 > budget * budget)
        {
            float scale = budget / sqrtf(length2);
            next = (m2Vec2){scale * next.x, scale * next.y};
        }
    }
    *accumulated = next;
    m2PushPointPair(armA, armB, b, (m2Vec2){next.x - old.x, next.y - old.y});
}
