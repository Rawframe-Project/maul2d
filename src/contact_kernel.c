// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The contact kernel. Each stage gathers the velocities of a block's
// bodies into lane vectors, works through the contact rows of every lane
// at once, and scatters the velocities back.
//
// A contact point has two rows. The normal row keeps the bodies from
// closing: a separated point only lets them close the gap in this
// substep (speculative), an overlapping point is pushed apart through a
// soft constraint while solving and held rigid while relaxing. The
// friction row resists sliding, bounded by the friction cone on the
// point's normal impulse. Impulses accumulate across substeps and warm
// start the next step.

#include "contact_kernel.h"

#include "simd.h"
#include "world_internal.h"

// The arms of one contact point, every lane.
typedef struct Arms
{
    m2f8 ax;
    m2f8 ay;
    m2f8 bx;
    m2f8 by;
} Arms;

// The velocities of both bodies and the pair's masses, every lane.
typedef struct Pair
{
    m2f8 vAx;
    m2f8 vAy;
    m2f8 wA;
    m2f8 vBx;
    m2f8 vBy;
    m2f8 wB;
    m2f8 mA;
    m2f8 iA;
    m2f8 mB;
    m2f8 iB;
    m2f8 nx;
    m2f8 ny;
} Pair;

// How far each body moved and turned since the step began, every lane.
typedef struct Drift
{
    m2f8 dx; // B's translation minus A's
    m2f8 dy;
    m2f8 qAc;
    m2f8 qAs;
    m2f8 qBc;
    m2f8 qBs;
} Drift;

static Arms LoadArms(const m2LanePoint* p)
{
    Arms r = {m2F8Load(p->armAX), m2F8Load(p->armAY), m2F8Load(p->armBX), m2F8Load(p->armBY)};
    return r;
}

static Pair GatherPair(const m2World* world, const m2ContactBlock* b)
{
    float lanes[6][M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        m2Vec2 vA = world->bodies.linearVelocities[b->bodyA[lane]];
        m2Vec2 vB = world->bodies.linearVelocities[b->bodyB[lane]];
        lanes[0][lane] = vA.x;
        lanes[1][lane] = vA.y;
        lanes[2][lane] = world->bodies.angularVelocities[b->bodyA[lane]];
        lanes[3][lane] = vB.x;
        lanes[4][lane] = vB.y;
        lanes[5][lane] = world->bodies.angularVelocities[b->bodyB[lane]];
    }
    Pair p;
    p.vAx = m2F8Load(lanes[0]);
    p.vAy = m2F8Load(lanes[1]);
    p.wA = m2F8Load(lanes[2]);
    p.vBx = m2F8Load(lanes[3]);
    p.vBy = m2F8Load(lanes[4]);
    p.wB = m2F8Load(lanes[5]);
    p.mA = m2F8Load(b->invMassA);
    p.iA = m2F8Load(b->invIA);
    p.mB = m2F8Load(b->invMassB);
    p.iB = m2F8Load(b->invIB);
    p.nx = m2F8Load(b->normalX);
    p.ny = m2F8Load(b->normalY);
    return p;
}

// Only dynamic bodies are written: a static or kinematic body appears in
// many blocks of one color, and even an unchanged write would race.
static void ScatterPair(m2World* world, const m2ContactBlock* b, const Pair* p)
{
    float lanes[6][M2_LANES];
    m2F8Store(lanes[0], p->vAx);
    m2F8Store(lanes[1], p->vAy);
    m2F8Store(lanes[2], p->wA);
    m2F8Store(lanes[3], p->vBx);
    m2F8Store(lanes[4], p->vBy);
    m2F8Store(lanes[5], p->wB);
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        int32_t bodyA = b->bodyA[lane];
        int32_t bodyB = b->bodyB[lane];
        if (world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[bodyA] = (m2Vec2){lanes[0][lane], lanes[1][lane]};
            world->bodies.angularVelocities[bodyA] = lanes[2][lane];
        }
        if (world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody)
        {
            world->bodies.linearVelocities[bodyB] = (m2Vec2){lanes[3][lane], lanes[4][lane]};
            world->bodies.angularVelocities[bodyB] = lanes[5][lane];
        }
    }
}

static Drift GatherDrift(const m2World* world, const m2ContactBlock* b)
{
    float lanes[6][M2_LANES];
    for (int32_t lane = 0; lane < M2_LANES; ++lane)
    {
        int32_t bodyA = b->bodyA[lane];
        int32_t bodyB = b->bodyB[lane];
        m2Vec2 dA = world->solver.deltaPositions[bodyA];
        m2Vec2 dB = world->solver.deltaPositions[bodyB];
        lanes[0][lane] = dB.x - dA.x;
        lanes[1][lane] = dB.y - dA.y;
        lanes[2][lane] = world->solver.deltaRotations[bodyA].c;
        lanes[3][lane] = world->solver.deltaRotations[bodyA].s;
        lanes[4][lane] = world->solver.deltaRotations[bodyB].c;
        lanes[5][lane] = world->solver.deltaRotations[bodyB].s;
    }
    Drift d = {m2F8Load(lanes[0]), m2F8Load(lanes[1]), m2F8Load(lanes[2]),
               m2F8Load(lanes[3]), m2F8Load(lanes[4]), m2F8Load(lanes[5])};
    return d;
}

// Speed of B's contact point relative to A's along (dx, dy).
static m2f8 RelativeSpeed(const Pair* p, const Arms* r, m2f8 dx, m2f8 dy)
{
    m2f8 x = m2F8Sub(m2F8NegMulAdd(p->wB, r->by, p->vBx), m2F8NegMulAdd(p->wA, r->ay, p->vAx));
    m2f8 y = m2F8Sub(m2F8MulAdd(p->wB, r->bx, p->vBy), m2F8MulAdd(p->wA, r->ax, p->vAy));
    return m2F8MulAdd(y, dy, m2F8Mul(x, dx));
}

// Applies the impulse (px, py) to B and its opposite to A.
static void ApplyImpulse(Pair* p, const Arms* r, m2f8 px, m2f8 py)
{
    m2f8 torqueA = m2F8NegMulAdd(r->ay, px, m2F8Mul(r->ax, py));
    m2f8 torqueB = m2F8NegMulAdd(r->by, px, m2F8Mul(r->bx, py));
    p->vAx = m2F8NegMulAdd(p->mA, px, p->vAx);
    p->vAy = m2F8NegMulAdd(p->mA, py, p->vAy);
    p->wA = m2F8NegMulAdd(p->iA, torqueA, p->wA);
    p->vBx = m2F8MulAdd(p->mB, px, p->vBx);
    p->vBy = m2F8MulAdd(p->mB, py, p->vBy);
    p->wB = m2F8MulAdd(p->iB, torqueB, p->wB);
}

// The separation now. The rows keep their prepare-time arms; only the
// gap follows the bodies, through the arms turned by each body's
// rotation since the step began.
static m2f8 Separation(const m2LanePoint* point, const Arms* r, const Drift* d, const Pair* p)
{
    m2f8 ax = m2F8NegMulAdd(d->qAs, r->ay, m2F8Mul(d->qAc, r->ax));
    m2f8 ay = m2F8MulAdd(d->qAs, r->ax, m2F8Mul(d->qAc, r->ay));
    m2f8 bx = m2F8NegMulAdd(d->qBs, r->by, m2F8Mul(d->qBc, r->bx));
    m2f8 by = m2F8MulAdd(d->qBs, r->bx, m2F8Mul(d->qBc, r->by));
    m2f8 sx = m2F8Add(d->dx, m2F8Sub(bx, ax));
    m2f8 sy = m2F8Add(d->dy, m2F8Sub(by, ay));
    return m2F8MulAdd(sy, p->ny, m2F8MulAdd(sx, p->nx, m2F8Load(point->gap)));
}

// The normal rows. The row's impulse is
//   -normalMass (scale vn + target) - decay accumulated
// with, per point: separated, target = separation / h and a rigid row;
// overlapping while solving, target = the soft push rate times the
// separation, never faster than M2_CONTACT_PUSH_MAX_SPEED, on a soft
// row; overlapping while relaxing, no target on a rigid row.
static void NormalRows(Pair* p, m2ContactBlock* b, const Drift* d, bool soft, float invH,
                       bool reversed)
{
    m2f8 zero = m2F8Zero();
    m2f8 one = m2F8Set1(1.0f);
    m2f8 maxPush = m2F8Set1(-M2_CONTACT_PUSH_MAX_SPEED);
    m2f8 pushRate = m2F8Load(b->pushRate);
    m2f8 softScale = soft ? m2F8Load(b->massScale) : one;
    m2f8 softDecay = soft ? m2F8Load(b->impulseScale) : zero;
    for (int32_t i = 0; i < b->pointCount; ++i)
    {
        m2LanePoint* point = &b->points[reversed ? b->pointCount - 1 - i : i];
        Arms r = LoadArms(point);
        m2f8 s = Separation(point, &r, d, p);
        m2f8 open = m2F8GT(s, zero);
        m2f8 push = soft ? m2F8Max(m2F8Mul(pushRate, s), maxPush) : zero;
        m2f8 target = m2F8Select(open, m2F8Mul(s, m2F8Set1(invH)), push);
        m2f8 scale = m2F8Select(open, one, softScale);
        m2f8 decay = m2F8Select(open, zero, softDecay);

        m2f8 vn = RelativeSpeed(p, &r, p->nx, p->ny);
        m2f8 old = m2F8Load(point->normalImpulse);
        m2f8 impulse = m2F8NegMulAdd(m2F8Load(point->normalMass), m2F8MulAdd(scale, vn, target),
                                     m2F8Neg(m2F8Mul(decay, old)));
        m2f8 total = m2F8Max(m2F8Add(old, impulse), zero);
        m2F8Store(point->normalImpulse, total);
        impulse = m2F8Sub(total, old);
        ApplyImpulse(p, &r, m2F8Mul(impulse, p->nx), m2F8Mul(impulse, p->ny));
    }
}

// The friction rows. The tangent is the normal turned a quarter left; a
// positive belt speed carries a body resting on an upward-facing surface
// toward +x.
static void FrictionRows(Pair* p, m2ContactBlock* b, bool reversed)
{
    m2f8 tx = m2F8Neg(p->ny);
    m2f8 ty = p->nx;
    m2f8 friction = m2F8Load(b->friction);
    m2f8 belt = m2F8Load(b->beltSpeed);
    for (int32_t i = 0; i < b->pointCount; ++i)
    {
        m2LanePoint* point = &b->points[reversed ? b->pointCount - 1 - i : i];
        Arms r = LoadArms(point);
        m2f8 vt = m2F8Add(RelativeSpeed(p, &r, tx, ty), belt);
        m2f8 old = m2F8Load(point->tangentImpulse);
        m2f8 bound = m2F8Mul(friction, m2F8Load(point->normalImpulse));
        m2f8 total = m2F8NegMulAdd(m2F8Load(point->tangentMass), vt, old);
        total = m2F8Min(m2F8Max(total, m2F8Neg(bound)), bound);
        m2F8Store(point->tangentImpulse, total);
        m2f8 impulse = m2F8Sub(total, old);
        ApplyImpulse(p, &r, m2F8Mul(impulse, tx), m2F8Mul(impulse, ty));
    }
}

// Re-applies last step's impulses.
static void WarmStart(Pair* p, const m2ContactBlock* b)
{
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        const m2LanePoint* point = &b->points[k];
        Arms r = LoadArms(point);
        m2f8 n = m2F8Load(point->normalImpulse);
        m2f8 t = m2F8Load(point->tangentImpulse);
        m2f8 px = m2F8NegMulAdd(t, p->ny, m2F8Mul(n, p->nx));
        m2f8 py = m2F8MulAdd(t, p->nx, m2F8Mul(n, p->ny));
        ApplyImpulse(p, &r, px, py);
    }
}

// Bounce: a point that closed faster than M2_RESTITUTION_THRESHOLD and
// carried load is driven to separate at restitution times its approach.
static void Restitution(Pair* p, m2ContactBlock* b)
{
    m2f8 zero = m2F8Zero();
    m2f8 slow = m2F8Set1(-M2_RESTITUTION_THRESHOLD);
    m2f8 restitution = m2F8Load(b->restitution);
    m2f8 bouncy = m2F8GT(restitution, zero);
    for (int32_t k = 0; k < b->pointCount; ++k)
    {
        m2LanePoint* point = &b->points[k];
        Arms r = LoadArms(point);
        m2f8 approach = m2F8Load(point->approach);
        m2f8 old = m2F8Load(point->normalImpulse);
        m2f8 vn = RelativeSpeed(p, &r, p->nx, p->ny);
        m2f8 impulse =
            m2F8Neg(m2F8Mul(m2F8Load(point->normalMass), m2F8MulAdd(restitution, approach, vn)));
        m2f8 total = m2F8Max(m2F8Add(old, impulse), zero);
        total = m2F8Select(m2F8GT(old, zero), total, old);
        total = m2F8Select(m2F8GT(approach, slow), old, total);
        total = m2F8Select(bouncy, total, old);
        m2F8Store(point->normalImpulse, total);
        impulse = m2F8Sub(total, old);
        ApplyImpulse(p, &r, m2F8Mul(impulse, p->nx), m2F8Mul(impulse, p->ny));
    }
}

static void Store(m2World* world, const m2ContactBlock* b)
{
    for (int32_t lane = 0; lane < b->lanes; ++lane)
    {
        m2Manifold* manifold = &world->contacts.manifolds[b->pairIndex[lane]];
        for (int32_t k = 0; k < b->pointCount; ++k)
        {
            manifold->points[k].normalImpulse = b->points[k].normalImpulse[lane];
            manifold->points[k].tangentImpulse = b->points[k].tangentImpulse[lane];
        }
    }
}

void m2RunContactBlock(m2World* world, m2ContactBlock* block, m2ContactStage stage, float invH,
                       bool reversed)
{
    if (stage == m2_stageStore)
    {
        Store(world, block);
        return;
    }
    Pair p = GatherPair(world, block);
    if (stage == m2_stageWarmStart)
    {
        WarmStart(&p, block);
    }
    else if (stage == m2_stageRestitution)
    {
        Restitution(&p, block);
    }
    else
    {
        Drift d = GatherDrift(world, block);
        NormalRows(&p, block, &d, stage == m2_stageSolve, invH, reversed);
        FrictionRows(&p, block, reversed);
    }
    ScatterPair(world, block, &p);
}

void m2PackContactLane(m2ContactBlock* block, int32_t lane, const m2ContactConstraint* c)
{
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
    block->restitution[lane] = c->restitution;
    block->beltSpeed[lane] = c->beltSpeed;
    block->pushRate[lane] = c->softness.massScale * c->softness.biasRate;
    block->massScale[lane] = c->softness.massScale;
    block->impulseScale[lane] = c->softness.impulseScale;
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        const m2ContactPoint* cp = &c->points[k];
        m2LanePoint* point = &block->points[k];
        point->armAX[lane] = cp->armA.x;
        point->armAY[lane] = cp->armA.y;
        point->armBX[lane] = cp->armB.x;
        point->armBY[lane] = cp->armB.y;
        point->gap[lane] = cp->gap;
        point->approach[lane] = cp->approach;
        point->normalMass[lane] = cp->normalMass;
        point->tangentMass[lane] = cp->tangentMass;
        point->normalImpulse[lane] = cp->normalImpulse;
        point->tangentImpulse[lane] = cp->tangentImpulse;
    }
}

void m2PadContactLane(m2ContactBlock* block, int32_t lane, int32_t dummyBody)
{
    m2ContactConstraint inert = {0};
    inert.pairIndex = -1;
    inert.bodyA = dummyBody;
    inert.bodyB = dummyBody;
    inert.normal = (m2Vec2){0.0f, 1.0f};
    inert.pointCount = 2;
    inert.points[0].gap = 1.0f; // separated: the row never acts
    inert.points[1].gap = 1.0f;
    m2PackContactLane(block, lane, &inert);
}

void m2UnpackContactLane(const m2ContactBlock* block, int32_t lane, m2ContactConstraint* c)
{
    for (int32_t k = 0; k < c->pointCount; ++k)
    {
        c->points[k].normalImpulse = block->points[k].normalImpulse[lane];
        c->points[k].tangentImpulse = block->points[k].tangentImpulse[lane];
    }
}
