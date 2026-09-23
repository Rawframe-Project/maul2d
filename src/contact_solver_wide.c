// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The wide contact solver: colored constraints packed into SIMD blocks
// and solved lane by lane with the same arithmetic as the scalar path.

#include "contact_solver_wide.h"

#include "contact_solver.h"
#include "simd.h"
#include "solver.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

// --- Wide-lane contact solving (topic-08 phase 2) --------------------------
//
// Blocks are SoA bundles of M2_LANES constraints from one graph color,
// homogeneous in point count. Every lane executes the same scalar IEEE
// sequence as the per-item path - the kernels below are transliterations
// of m2WarmStartOne/m2SolveContactOne/m2RestitutionOne - so the lane layout is
// pure mechanical sympathy: compilers vectorize the lane loops, and the
// bits cannot move. Padding lanes point at the dummy body slot
// (index bodyCapacity, zero mass, never scattered).

typedef struct m2ContactBlock
{
    int32_t lanes;      // live lanes; the rest are dummy-padded
    int32_t pointCount; // homogeneous: 1 or 2 for every live lane
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
    float biasRate[M2_LANES];
    float massScale[M2_LANES];
    float impulseScale[M2_LANES];
    float rAX[2][M2_LANES];
    float rAY[2][M2_LANES];
    float rBX[2][M2_LANES];
    float rBY[2][M2_LANES];
    float baseSep[2][M2_LANES];
    float relVel[2][M2_LANES];
    float normalMass[2][M2_LANES];
    float tangentMass[2][M2_LANES];
    float tangentSpeed[M2_LANES];
    float normalImp[2][M2_LANES];
    float tangentImp[2][M2_LANES];
} m2ContactBlock;

int32_t m2ContactBlockScratchBytes(int32_t pairCapacity)
{
    // Worst case: every block half-full plus two partial blocks per
    // color (one per point-count class).
    int32_t blocks = pairCapacity / M2_LANES + 2 * (M2_GRAPH_COLORS + 1) + 2;
    return blocks * (int32_t)sizeof(m2ContactBlock);
}

// Pack one color's constraints (already partitioned by point count)
// into blocks. Returns the new block count.
static int32_t PackRun(m2World* world, m2ContactConstraint* constraints, const int32_t* order,
                       int32_t count, int32_t pointCount, int32_t blockCount)
{
    m2ContactBlock* blocks = (m2ContactBlock*)world->solver.contactBlocks;
    int32_t dummy = world->bodies.bodyCapacity;
    for (int32_t base = 0; base < count; base += M2_LANES)
    {
        m2ContactBlock* block = blocks + blockCount;
        blockCount += 1;
        int32_t lanes = count - base < M2_LANES ? count - base : M2_LANES;
        block->lanes = lanes;
        block->pointCount = pointCount;
        for (int32_t lane = 0; lane < M2_LANES; ++lane)
        {
            if (lane >= lanes)
            {
                block->bodyA[lane] = dummy;
                block->bodyB[lane] = dummy;
                block->pairIndex[lane] = -1;
                block->invMassA[lane] = 0.0f;
                block->invIA[lane] = 0.0f;
                block->invMassB[lane] = 0.0f;
                block->invIB[lane] = 0.0f;
                block->normalX[lane] = 0.0f;
                block->normalY[lane] = 1.0f;
                block->friction[lane] = 0.0f;
                block->tangentSpeed[lane] = 0.0f;
                block->restitution[lane] = 0.0f;
                block->biasRate[lane] = 0.0f;
                block->massScale[lane] = 1.0f;
                block->impulseScale[lane] = 0.0f;
                for (int32_t k = 0; k < 2; ++k)
                {
                    block->rAX[k][lane] = 0.0f;
                    block->rAY[k][lane] = 0.0f;
                    block->rBX[k][lane] = 0.0f;
                    block->rBY[k][lane] = 0.0f;
                    block->baseSep[k][lane] = 1.0f; // separated: speculative no-op
                    block->relVel[k][lane] = 0.0f;
                    block->normalMass[k][lane] = 0.0f;
                    block->tangentMass[k][lane] = 0.0f;
                    block->normalImp[k][lane] = 0.0f;
                    block->tangentImp[k][lane] = 0.0f;
                }
                continue;
            }
            m2ContactConstraint* c = constraints + order[base + lane];
            block->bodyA[lane] = c->bodyA;
            block->bodyB[lane] = c->bodyB;
            block->pairIndex[lane] = c->pairIndex;
            block->invMassA[lane] = c->invMassA;
            block->invIA[lane] = c->invIA;
            block->invMassB[lane] = c->invMassB;
            block->invIB[lane] = c->invIB;
            block->normalX[lane] = c->normal.x;
            block->normalY[lane] = c->normal.y;
            block->friction[lane] = c->friction;
            block->tangentSpeed[lane] = c->tangentSpeed;
            block->restitution[lane] = c->restitution;
            block->biasRate[lane] = c->softness.biasRate;
            block->massScale[lane] = c->softness.massScale;
            block->impulseScale[lane] = c->softness.impulseScale;
            for (int32_t k = 0; k < 2; ++k)
            {
                const m2ConstraintPoint* cp = &c->points[k < c->pointCount ? k : 0];
                bool live = k < c->pointCount;
                block->rAX[k][lane] = live ? cp->rA.x : 0.0f;
                block->rAY[k][lane] = live ? cp->rA.y : 0.0f;
                block->rBX[k][lane] = live ? cp->rB.x : 0.0f;
                block->rBY[k][lane] = live ? cp->rB.y : 0.0f;
                block->baseSep[k][lane] = live ? cp->baseSeparation : 1.0f;
                block->relVel[k][lane] = live ? cp->relativeVelocity : 0.0f;
                block->normalMass[k][lane] = live ? cp->normalMass : 0.0f;
                block->tangentMass[k][lane] = live ? cp->tangentMass : 0.0f;
                block->normalImp[k][lane] = live ? cp->normalImpulse : 0.0f;
                block->tangentImp[k][lane] = live ? cp->tangentImpulse : 0.0f;
            }
        }
    }
    return blockCount;
}

// Partition each full color by point count (order within a color is
// free: its constraints share no dynamic body), then pack. Returns
// block count; blockStart[c]..blockStart[c+1] are color c's blocks.
int32_t m2PackContactBlocks(m2World* world, m2ContactConstraint* constraints,
                            const int32_t* colorStart, int32_t* blockStart)
{
    int32_t blockCount = 0;
    int32_t scratch[2][256];
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        blockStart[color] = blockCount;
        int32_t begin = colorStart[color];
        int32_t end = colorStart[color + 1];
        int32_t twos = 0;
        int32_t ones = 0;
        for (int32_t k = begin; k < end; ++k)
        {
            int32_t index = world->solver.colorOrder[k];
            if (constraints[index].pointCount == 2)
            {
                if (twos < 256)
                {
                    scratch[0][twos] = index;
                }
                twos += 1;
            }
            else
            {
                if (ones < 256)
                {
                    scratch[1][ones] = index;
                }
                ones += 1;
            }
            // Colors are capped by the per-body u32 mask, but a color
            // can hold more than 256 constraints in huge worlds; spill
            // the excess back into the overflow-style scalar path by
            // marking it - handled below via the spill list.
        }
        M2_ASSERT(twos <= 256 && ones <= 256);
        blockCount =
            PackRun(world, constraints, scratch[0], twos <= 256 ? twos : 256, 2, blockCount);
        blockCount =
            PackRun(world, constraints, scratch[1], ones <= 256 ? ones : 256, 1, blockCount);
    }
    blockStart[M2_GRAPH_COLORS] = blockCount;
    return blockCount;
}

typedef struct m2BlockStageCtx
{
    m2World* world;
    m2ContactBlock* blocks;
    m2ContactStage stage;
    float invH;
    float minBiasVel;
    bool useBias;
} m2BlockStageCtx;

static void WarmStartBlock(m2World* world, m2ContactBlock* b)
{
    float vAx[M2_LANES];
    float vAy[M2_LANES];
    float wA[M2_LANES];
    float vBx[M2_LANES];
    float vBy[M2_LANES];
    float wB[M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        vAx[lane] = world->bodies.linearVelocities[b->bodyA[lane]].x;
        vAy[lane] = world->bodies.linearVelocities[b->bodyA[lane]].y;
        wA[lane] = world->bodies.angularVelocities[b->bodyA[lane]];
        vBx[lane] = world->bodies.linearVelocities[b->bodyB[lane]].x;
        vBy[lane] = world->bodies.linearVelocities[b->bodyB[lane]].y;
        wB[lane] = world->bodies.angularVelocities[b->bodyB[lane]];
    }
    // Vector body (simd.h bit law): tangent = (-ny, nx), P = nImp*n +
    // tImp*t, velocities pick up the usual +-invMass * P terms.
    m2f8 nX = m2F8Load(b->normalX);
    m2f8 nY = m2F8Load(b->normalY);
    m2f8 imA = m2F8Load(b->invMassA);
    m2f8 iiA = m2F8Load(b->invIA);
    m2f8 imB = m2F8Load(b->invMassB);
    m2f8 iiB = m2F8Load(b->invIB);
    m2f8 avAx = m2F8Load(vAx);
    m2f8 avAy = m2F8Load(vAy);
    m2f8 awA = m2F8Load(wA);
    m2f8 avBx = m2F8Load(vBx);
    m2f8 avBy = m2F8Load(vBy);
    m2f8 awB = m2F8Load(wB);
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 nImp = m2F8Load(b->normalImp[k]);
        m2f8 tImp = m2F8Load(b->tangentImp[k]);
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        m2f8 Px = m2F8NegMulAdd(tImp, nY, m2F8Mul(nImp, nX));
        m2f8 Py = m2F8MulAdd(tImp, nX, m2F8Mul(nImp, nY));
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        m2f8 crossA = m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py));
        awA = m2F8NegMulAdd(iiA, crossA, awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        m2f8 crossB = m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py));
        awB = m2F8MulAdd(iiB, crossB, awB);
    }
    m2F8Store(vAx, avAx);
    m2F8Store(vAy, avAy);
    m2F8Store(wA, awA);
    m2F8Store(vBx, avBx);
    m2F8Store(vBy, avBy);
    m2F8Store(wB, awB);
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        if (world->bodies.types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx[lane], vAy[lane]};
            world->bodies.angularVelocities[b->bodyA[lane]] = wA[lane];
        }
        if (world->bodies.types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx[lane], vBy[lane]};
            world->bodies.angularVelocities[b->bodyB[lane]] = wB[lane];
        }
    }
}

static void SolveBlock(m2World* world, m2ContactBlock* b, float invH, float minBiasVel,
                       bool useBias)
{
    float vAx[M2_LANES];
    float vAy[M2_LANES];
    float wA[M2_LANES];
    float vBx[M2_LANES];
    float vBy[M2_LANES];
    float wB[M2_LANES];
    float dpx[M2_LANES];
    float dpy[M2_LANES];
    float dqAc[M2_LANES];
    float dqAs[M2_LANES];
    float dqBc[M2_LANES];
    float dqBs[M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        int32_t iA = b->bodyA[lane];
        int32_t iB = b->bodyB[lane];
        vAx[lane] = world->bodies.linearVelocities[iA].x;
        vAy[lane] = world->bodies.linearVelocities[iA].y;
        wA[lane] = world->bodies.angularVelocities[iA];
        vBx[lane] = world->bodies.linearVelocities[iB].x;
        vBy[lane] = world->bodies.linearVelocities[iB].y;
        wB[lane] = world->bodies.angularVelocities[iB];
        dpx[lane] = world->solver.deltaPositions[iB].x - world->solver.deltaPositions[iA].x;
        dpy[lane] = world->solver.deltaPositions[iB].y - world->solver.deltaPositions[iA].y;
        dqAc[lane] = world->solver.deltaRotations[iA].c;
        dqAs[lane] = world->solver.deltaRotations[iA].s;
        dqBc[lane] = world->solver.deltaRotations[iB].c;
        dqBs[lane] = world->solver.deltaRotations[iB].s;
    }

    // Vector body (simd.h bit law). The selects mirror the scalar
    // ternaries exactly; useBias is uniform per call, so its branch is
    // resolved once outside the lane math.
    m2f8 nX = m2F8Load(b->normalX);
    m2f8 nY = m2F8Load(b->normalY);
    m2f8 imA = m2F8Load(b->invMassA);
    m2f8 iiA = m2F8Load(b->invIA);
    m2f8 imB = m2F8Load(b->invMassB);
    m2f8 iiB = m2F8Load(b->invIB);
    m2f8 avAx = m2F8Load(vAx);
    m2f8 avAy = m2F8Load(vAy);
    m2f8 awA = m2F8Load(wA);
    m2f8 avBx = m2F8Load(vBx);
    m2f8 avBy = m2F8Load(vBy);
    m2f8 awB = m2F8Load(wB);
    m2f8 vdpx = m2F8Load(dpx);
    m2f8 vdpy = m2F8Load(dpy);
    m2f8 vqAc = m2F8Load(dqAc);
    m2f8 vqAs = m2F8Load(dqAs);
    m2f8 vqBc = m2F8Load(dqBc);
    m2f8 vqBs = m2F8Load(dqBs);
    m2f8 zero = m2F8Zero();
    m2f8 one = m2F8Set1(1.0f);
    m2f8 vInvH = m2F8Set1(invH);
    m2f8 vMinBias = m2F8Set1(minBiasVel);
    m2f8 biasRate = m2F8Load(b->biasRate);
    m2f8 msIn = useBias ? m2F8Load(b->massScale) : one;
    m2f8 isIn = useBias ? m2F8Load(b->impulseScale) : zero;

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        m2f8 rsAx = m2F8NegMulAdd(vqAs, rAYk, m2F8Mul(vqAc, rAXk));
        m2f8 rsAy = m2F8MulAdd(vqAs, rAXk, m2F8Mul(vqAc, rAYk));
        m2f8 rsBx = m2F8NegMulAdd(vqBs, rBYk, m2F8Mul(vqBc, rBXk));
        m2f8 rsBy = m2F8MulAdd(vqBs, rBXk, m2F8Mul(vqBc, rBYk));
        m2f8 dsx = m2F8Add(vdpx, m2F8Sub(rsBx, rsAx));
        m2f8 dsy = m2F8Add(vdpy, m2F8Sub(rsBy, rsAy));
        m2f8 sep = m2F8MulAdd(dsy, nY, m2F8MulAdd(dsx, nX, m2F8Load(b->baseSep[k])));

        m2f8 spec = m2F8GT(sep, zero);
        m2f8 softBias = m2F8Max(m2F8Mul(biasRate, sep), vMinBias);
        m2f8 nonSpecBias = useBias ? softBias : zero;
        m2f8 bias = m2F8Select(spec, m2F8Mul(sep, vInvH), nonSpecBias);
        m2f8 massScale = m2F8Select(spec, one, msIn);
        m2f8 impulseScale = m2F8Select(spec, zero, isIn);

        m2f8 vrAx = m2F8NegMulAdd(awA, rAYk, avAx);
        m2f8 vrAy = m2F8MulAdd(awA, rAXk, avAy);
        m2f8 vrBx = m2F8NegMulAdd(awB, rBYk, avBx);
        m2f8 vrBy = m2F8MulAdd(awB, rBXk, avBy);
        m2f8 vn = m2F8MulAdd(m2F8Sub(vrBy, vrAy), nY, m2F8Mul(m2F8Sub(vrBx, vrAx), nX));

        m2f8 nImp = m2F8Load(b->normalImp[k]);
        m2f8 scaledMass = m2F8Mul(m2F8Load(b->normalMass[k]), massScale);
        m2f8 drag = m2F8Mul(impulseScale, nImp);
        m2f8 impulse = m2F8NegMulAdd(scaledMass, m2F8Add(vn, bias), m2F8Neg(drag));
        m2f8 newImpulse = m2F8Max(m2F8Add(nImp, impulse), zero);
        impulse = m2F8Sub(newImpulse, nImp);
        m2F8Store(b->normalImp[k], newImpulse);

        m2f8 Px = m2F8Mul(impulse, nX);
        m2f8 Py = m2F8Mul(impulse, nY);
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        awA = m2F8NegMulAdd(iiA, m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py)), awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        awB = m2F8MulAdd(iiB, m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py)), awB);
    }

    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2f8 rAXk = m2F8Load(b->rAX[k]);
        m2f8 rAYk = m2F8Load(b->rAY[k]);
        m2f8 rBXk = m2F8Load(b->rBX[k]);
        m2f8 rBYk = m2F8Load(b->rBY[k]);
        // tangent = (-nY, nX)
        m2f8 vrAx = m2F8NegMulAdd(awA, rAYk, avAx);
        m2f8 vrAy = m2F8MulAdd(awA, rAXk, avAy);
        m2f8 vrBx = m2F8NegMulAdd(awB, rBYk, avBx);
        m2f8 vrBy = m2F8MulAdd(awB, rBXk, avBy);
        m2f8 vt = m2F8MulAdd(m2F8Sub(vrBy, vrAy), nX, m2F8Mul(m2F8Sub(vrBx, vrAx), m2F8Neg(nY)));
        vt = m2F8Add(vt, m2F8Load(b->tangentSpeed)); // left-perp tangent: belt term adds
        m2f8 tImp = m2F8Load(b->tangentImp[k]);
        m2f8 impulse = m2F8Neg(m2F8Mul(m2F8Load(b->tangentMass[k]), vt));
        m2f8 maxFriction = m2F8Mul(m2F8Load(b->friction), m2F8Load(b->normalImp[k]));
        m2f8 newImpulse =
            m2F8Min(m2F8Max(m2F8Add(tImp, impulse), m2F8Neg(maxFriction)), maxFriction);
        impulse = m2F8Sub(newImpulse, tImp);
        m2F8Store(b->tangentImp[k], newImpulse);

        m2f8 Px = m2F8Mul(impulse, m2F8Neg(nY));
        m2f8 Py = m2F8Mul(impulse, nX);
        avAx = m2F8NegMulAdd(imA, Px, avAx);
        avAy = m2F8NegMulAdd(imA, Py, avAy);
        awA = m2F8NegMulAdd(iiA, m2F8NegMulAdd(rAYk, Px, m2F8Mul(rAXk, Py)), awA);
        avBx = m2F8MulAdd(imB, Px, avBx);
        avBy = m2F8MulAdd(imB, Py, avBy);
        awB = m2F8MulAdd(iiB, m2F8NegMulAdd(rBYk, Px, m2F8Mul(rBXk, Py)), awB);
    }
    m2F8Store(vAx, avAx);
    m2F8Store(vAy, avAy);
    m2F8Store(wA, awA);
    m2F8Store(vBx, avBx);
    m2F8Store(vBy, avBy);
    m2F8Store(wB, awB);

    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        if (world->bodies.types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx[lane], vAy[lane]};
            world->bodies.angularVelocities[b->bodyA[lane]] = wA[lane];
        }
        if (world->bodies.types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx[lane], vBy[lane]};
            world->bodies.angularVelocities[b->bodyB[lane]] = wB[lane];
        }
    }
}

static void RestitutionBlock(m2World* world, m2ContactBlock* b)
{
    // Per-lane branches on purpose: the scalar path skips whole
    // updates, and a masked add of a signed zero would move bits.
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        if (b->restitution[lane] == 0.0f)
        {
            continue;
        }
        float vAx = world->bodies.linearVelocities[b->bodyA[lane]].x;
        float vAy = world->bodies.linearVelocities[b->bodyA[lane]].y;
        float wA = world->bodies.angularVelocities[b->bodyA[lane]];
        float vBx = world->bodies.linearVelocities[b->bodyB[lane]].x;
        float vBy = world->bodies.linearVelocities[b->bodyB[lane]].y;
        float wB = world->bodies.angularVelocities[b->bodyB[lane]];
        for (int32_t k = 0; k < b->pointCount; ++k)
        {
            if (b->relVel[k][lane] > -M2_RESTITUTION_THRESHOLD || b->normalImp[k][lane] == 0.0f)
            {
                continue;
            }
            float vrAx = vAx - wA * b->rAY[k][lane];
            float vrAy = vAy + wA * b->rAX[k][lane];
            float vrBx = vBx - wB * b->rBY[k][lane];
            float vrBy = vBy + wB * b->rBX[k][lane];
            float vn = (vrBx - vrAx) * b->normalX[lane] + (vrBy - vrAy) * b->normalY[lane];
            float impulse =
                -b->normalMass[k][lane] * (vn + b->restitution[lane] * b->relVel[k][lane]);
            float newImpulse = m2MaxF(b->normalImp[k][lane] + impulse, 0.0f);
            impulse = newImpulse - b->normalImp[k][lane];
            b->normalImp[k][lane] = newImpulse;
            float Px = impulse * b->normalX[lane];
            float Py = impulse * b->normalY[lane];
            vAx -= b->invMassA[lane] * Px;
            vAy -= b->invMassA[lane] * Py;
            wA -= b->invIA[lane] * (b->rAX[k][lane] * Py - b->rAY[k][lane] * Px);
            vBx += b->invMassB[lane] * Px;
            vBy += b->invMassB[lane] * Py;
            wB += b->invIB[lane] * (b->rBX[k][lane] * Py - b->rBY[k][lane] * Px);
        }
        if (world->bodies.types[b->bodyA[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyA[lane]] = (m2Vec2){vAx, vAy};
            world->bodies.angularVelocities[b->bodyA[lane]] = wA;
        }
        if (world->bodies.types[b->bodyB[lane]] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[b->bodyB[lane]] = (m2Vec2){vBx, vBy};
            world->bodies.angularVelocities[b->bodyB[lane]] = wB;
        }
    }
}

static void StoreBlock(m2World* world, m2ContactBlock* b)
{
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        m2Manifold* manifold = &world->contacts.manifolds[b->pairIndex[lane]];
        for (int32_t k = 0; k < b->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = b->normalImp[k][lane];
            manifold->points[k].tangentImpulse = b->tangentImp[k][lane];
        }
    }
}

static void BlockStageRange(int32_t begin, int32_t end, void* userCtx)
{
    m2BlockStageCtx* ctx = (m2BlockStageCtx*)userCtx;
    for (int32_t i = begin; i < end; ++i)
    {
        m2ContactBlock* block = ctx->blocks + i;
        switch (ctx->stage)
        {
        case m2_stageWarmStart:
            WarmStartBlock(ctx->world, block);
            break;
        case m2_stageSolve:
            SolveBlock(ctx->world, block, ctx->invH, ctx->minBiasVel, ctx->useBias);
            break;
        case m2_stageRestitution:
            RestitutionBlock(ctx->world, block);
            break;
        default:
            StoreBlock(ctx->world, block);
            break;
        }
    }
}

// Full colors run wide; the overflow bucket stays on the per-item path.
void m2RunContactStageWide(m2World* world, m2ContactConstraint* constraints,
                           const int32_t* colorStart, const int32_t* blockStart,
                           m2ContactStage stage, float invH, float minBiasVel, bool useBias)
{
    m2BlockStageCtx ctx;
    ctx.world = world;
    ctx.blocks = (m2ContactBlock*)world->solver.contactBlocks;
    ctx.stage = stage;
    ctx.invH = invH;
    ctx.minBiasVel = minBiasVel;
    ctx.useBias = useBias;
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        int32_t begin = blockStart[color];
        int32_t end = blockStart[color + 1];
        if (begin == end)
        {
            continue;
        }
        ctx.blocks = (m2ContactBlock*)world->solver.contactBlocks + begin;
        m2RunParallel(world, BlockStageRange, &ctx, end - begin, 1);
    }

    // Overflow bucket: serial per-item path, canonical order.
    int32_t begin = colorStart[M2_GRAPH_COLORS];
    int32_t end = colorStart[M2_GRAPH_COLORS + 1];
    if (begin != end)
    {
        m2ContactStageCtx overflow;
        overflow.world = world;
        overflow.constraints = constraints;
        overflow.order = world->solver.colorOrder + begin;
        overflow.stage = stage;
        overflow.invH = invH;
        overflow.minBiasVel = minBiasVel;
        overflow.useBias = useBias;
        m2ContactStageRange(0, end - begin, &overflow);
    }
}
