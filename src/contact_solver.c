// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Contact constraints: one per touching manifold that can move, built
// fresh every step, then colored and packed into lane blocks. Every
// stage runs the one kernel in contact_kernel.c: colored blocks in
// parallel, and the overflow, whose constraints may share bodies, one
// constraint at a time through a single-lane block.

#include "contact_solver.h"

#include "contact_kernel.h"
#include "graph_color.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>

int32_t m2ContactConstraintSize(void)
{
    // Joint constraints ride in the tail of the same scratch block.
    return (int32_t)sizeof(m2ContactConstraint) + (int32_t)sizeof(m2JointConstraint);
}

int32_t m2ContactBlockScratchBytes(int32_t pairCapacity)
{
    // Full blocks, plus one partial block per color and point count.
    int32_t blocks = pairCapacity / M2_LANES + 2 * M2_GRAPH_COLORS;
    return blocks * (int32_t)sizeof(m2ContactBlock);
}

static bool IsDynamic(const m2World* world, int32_t body)
{
    return world->bodies.types[body] == (uint8_t)m2_dynamicBody;
}

// The pair's masses. Between dynamic bodies of different dominance the
// stronger side acts as if it were static; statics outrank everything.
static void PairMasses(const m2World* world, m2ContactConstraint* c)
{
    int32_t rankA = IsDynamic(world, c->bodyA) ? (int32_t)world->bodies.dominances[c->bodyA] : 128;
    int32_t rankB = IsDynamic(world, c->bodyB) ? (int32_t)world->bodies.dominances[c->bodyB] : 128;
    bool keepA = rankA <= rankB;
    bool keepB = rankB <= rankA;
    c->invMassA = keepA ? world->bodies.invMass[c->bodyA] : 0.0f;
    c->invIA = keepA ? world->bodies.invInertia[c->bodyA] : 0.0f;
    c->invMassB = keepB ? world->bodies.invMass[c->bodyB] : 0.0f;
    c->invIB = keepB ? world->bodies.invInertia[c->bodyB] : 0.0f;
}

// Inverse effective mass of a row along dir through the arms; zero mass
// marks a row that cannot move.
static float RowMass(const m2ContactConstraint* c, m2Vec2 armA, m2Vec2 armB, m2Vec2 dir)
{
    float turnA = m2Cross2(armA, dir);
    float turnB = m2Cross2(armB, dir);
    float k = c->invMassA + c->invMassB + c->invIA * turnA * turnA + c->invIB * turnB * turnB;
    return k > 0.0f ? 1.0f / k : 0.0f;
}

static m2Vec2 PointVelocity(const m2World* world, int32_t body, m2Vec2 arm)
{
    m2Vec2 v = world->bodies.linearVelocities[body];
    float w = world->bodies.angularVelocities[body];
    return (m2Vec2){v.x - w * arm.y, v.y + w * arm.x};
}

static void PreparePoint(const m2World* world, m2ContactConstraint* c, const m2ManifoldPoint* mp,
                         m2ContactPoint* cp)
{
    m2Rot qA = world->bodies.transforms[c->bodyA].q;
    m2Rot qB = world->bodies.transforms[c->bodyB].q;
    m2Vec2 centerA = world->bodies.localCenters[c->bodyA];
    m2Vec2 centerB = world->bodies.localCenters[c->bodyB];
    cp->armA = m2RotateVec2(qA, (m2Vec2){mp->anchorA.x - centerA.x, mp->anchorA.y - centerA.y});
    cp->armB = m2RotateVec2(qB, (m2Vec2){mp->anchorB.x - centerB.x, mp->anchorB.y - centerB.y});
    // The solver measures separation as gap + n.(dB + armB' - dA - armA')
    // with the arms turned by the rotation since the step began, so the
    // prepare-time arm offset is taken out of the gap here once.
    m2Vec2 n = c->normal;
    m2Vec2 armGap = {cp->armB.x - cp->armA.x, cp->armB.y - cp->armA.y};
    cp->gap = mp->separation - (armGap.x * n.x + armGap.y * n.y);
    cp->normalImpulse = mp->normalImpulse;
    cp->tangentImpulse = mp->tangentImpulse;
    cp->normalMass = RowMass(c, cp->armA, cp->armB, n);
    cp->tangentMass = RowMass(c, cp->armA, cp->armB, (m2Vec2){-n.y, n.x});
    m2Vec2 vA = PointVelocity(world, c->bodyA, cp->armA);
    m2Vec2 vB = PointVelocity(world, c->bodyB, cp->armB);
    cp->approach = (vB.x - vA.x) * n.x + (vB.y - vA.y) * n.y;
}

// Surface mixing: friction by geometric mean, restitution by maximum,
// belt speeds add. Contacts with a static or kinematic body use the
// stiffer softness: a soft ground row stores energy under a tall stack.
static void PrepareMaterial(const m2World* world, m2ContactConstraint* c, int32_t shapeA,
                            int32_t shapeB, const m2Softness soft[2])
{
    c->friction = sqrtf(world->shapes.shapeFriction[shapeA] * world->shapes.shapeFriction[shapeB]);
    c->restitution =
        m2MaxF(world->shapes.shapeRestitution[shapeA], world->shapes.shapeRestitution[shapeB]);
    c->beltSpeed =
        world->shapes.shapeTangentSpeed[shapeA] + world->shapes.shapeTangentSpeed[shapeB];
    bool anchored = !IsDynamic(world, c->bodyA) || !IsDynamic(world, c->bodyB);
    c->softness = soft[anchored ? 1 : 0];
}

// True when the pair is solved this step: no sensor, something can move,
// and at least one side is awake.
static bool PairSolves(const m2World* world, int32_t shapeA, int32_t shapeB)
{
    if (world->shapes.shapeSensor[shapeA] != 0 || world->shapes.shapeSensor[shapeB] != 0)
    {
        return false;
    }
    int32_t bodyA = world->shapes.shapeBody[shapeA];
    int32_t bodyB = world->shapes.shapeBody[shapeB];
    bool awakeA = IsDynamic(world, bodyA) && world->bodies.asleep[bodyA] == 0;
    bool awakeB = IsDynamic(world, bodyB) && world->bodies.asleep[bodyB] == 0;
    return awakeA || awakeB;
}

int32_t m2PrepareContacts(m2World* world, m2ContactConstraint* constraints, float h)
{
    m2Softness soft[2] = {
        m2MakeSoft(M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h),
        m2MakeSoft(2.0f * M2_CONTACT_HERTZ, M2_CONTACT_DAMPING_RATIO, h),
    };
    int32_t count = 0;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        const m2Manifold* manifold = &world->contacts.manifolds[i];
        int32_t shapeA = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t shapeB = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (manifold->pointCount == 0 || !PairSolves(world, shapeA, shapeB))
        {
            continue;
        }
        m2ContactConstraint* c = constraints + count;
        c->pairIndex = i;
        c->bodyA = world->shapes.shapeBody[shapeA];
        c->bodyB = world->shapes.shapeBody[shapeB];
        PairMasses(world, c);
        if (c->invMassA + c->invMassB == 0.0f && c->invIA + c->invIB == 0.0f)
        {
            continue; // dominance left nothing to move
        }
        count += 1;
        PrepareMaterial(world, c, shapeA, shapeB, soft);
        c->normal = m2RotateVec2(world->bodies.transforms[c->bodyA].q, manifold->normal);
        c->pointCount = manifold->pointCount;
        for (int32_t k = 0; k < manifold->pointCount; ++k)
        {
            PreparePoint(world, c, &manifold->points[k], &c->points[k]);
        }
    }
    return count;
}

// Packs the constraints of one color with the given point count into
// full blocks and one padded tail. Returns the new block count.
static int32_t PackColor(m2World* world, const m2ContactPlan* plan, int32_t color,
                         int32_t pointCount, int32_t blockCount)
{
    m2ContactBlock* blocks = (m2ContactBlock*)world->solver.contactBlocks;
    m2ContactBlock* block = NULL;
    for (int32_t k = plan->colorStart[color]; k < plan->colorStart[color + 1]; ++k)
    {
        const m2ContactConstraint* c = plan->constraints + world->solver.colorOrder[k];
        if (c->pointCount != pointCount)
        {
            continue;
        }
        if (block == NULL || block->lanes == M2_LANES)
        {
            block = blocks + blockCount;
            blockCount += 1;
            block->lanes = 0;
            block->pointCount = pointCount;
        }
        m2PackContactLane(block, block->lanes, c);
        block->lanes += 1;
    }
    if (block != NULL)
    {
        for (int32_t lane = block->lanes; lane < M2_LANES; ++lane)
        {
            m2PadContactLane(block, lane, world->bodies.bodyCapacity);
        }
    }
    return blockCount;
}

void m2PlanContacts(m2World* world, m2ContactPlan* plan)
{
    m2ColorConstraints(world, plan->constraints, plan->count, plan->colorStart);
    int32_t blockCount = 0;
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        plan->blockStart[color] = blockCount;
        blockCount = PackColor(world, plan, color, 2, blockCount);
        blockCount = PackColor(world, plan, color, 1, blockCount);
    }
    plan->blockStart[M2_GRAPH_COLORS] = blockCount;
}

typedef struct StageTask
{
    m2World* world;
    m2ContactBlock* blocks;
    m2ContactStage stage;
    float invH;
    bool reversed;
} StageTask;

static void StageRange(int32_t begin, int32_t end, void* context)
{
    StageTask* task = (StageTask*)context;
    for (int32_t i = begin; i < end; ++i)
    {
        m2RunContactBlock(task->world, task->blocks + i, task->stage, task->invH, task->reversed);
    }
}

// The overflow in canonical order, each constraint alone in lane 0 of a
// block whose other lanes stay inert.
static void RunOverflow(m2World* world, const m2ContactPlan* plan, m2ContactStage stage)
{
    int32_t begin = plan->colorStart[M2_GRAPH_COLORS];
    int32_t end = plan->colorStart[M2_GRAPH_COLORS + 1];
    if (begin == end)
    {
        return;
    }
    m2ContactBlock block;
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        m2PadContactLane(&block, lane, world->bodies.bodyCapacity);
    }
    block.lanes = 1;
    for (int32_t k = begin; k < end; ++k)
    {
        m2ContactConstraint* c = plan->constraints + world->solver.colorOrder[k];
        block.pointCount = c->pointCount;
        m2PackContactLane(&block, 0, c);
        m2RunContactBlock(world, &block, stage, plan->invH, plan->reversed);
        m2UnpackContactLane(&block, 0, c);
    }
}

void m2RunContactStage(m2World* world, const m2ContactPlan* plan, m2ContactStage stage)
{
    StageTask task = {world, NULL, stage, plan->invH, plan->reversed};
    for (int32_t color = 0; color < M2_GRAPH_COLORS; ++color)
    {
        int32_t begin = plan->blockStart[color];
        int32_t count = plan->blockStart[color + 1] - begin;
        if (count > 0)
        {
            task.blocks = (m2ContactBlock*)world->solver.contactBlocks + begin;
            m2RunParallel(world, StageRange, &task, count, 1);
        }
    }
    RunOverflow(world, plan, stage);
}
