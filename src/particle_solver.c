// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The particle pass, once per step before the rigid solve. The model is
// double density relaxation (Clavet, Beaudoin and Poulin, 2005) moved
// from positions to velocities.
//
// Every pair closer than one diameter h carries the weight
// q = 1 - r / h. A particle's density is the sum of q^2 over its pairs
// and over the bodies it touches (a wall counts as a neighbor), its near
// density the sum of q^3. The rest density is that of the square
// packing the particle fill lays out at the rest stride of three
// quarters of a diameter: four neighbors at q = 1/4. Above rest density a particle has pressure; a
// pair's two pressures push it apart along its normal, weighted by q. Tensile particles also pull
// below rest density (cohesion) and carry a near pressure weighted by q^2, which stops a droplet
// from collapsing onto itself; plain water feels nothing below rest density, so sparse water never
// moves.
//
// Then, in order: powder grains packed tighter than the rest stride
// push apart; spring nets and elastic triads pull toward their spawn
// shape (shape matching, Mueller et al. 2005); viscous particles lose
// relative speed to their neighbors (XSPH); every approaching pair and
// contact loses approach speed, linearly and quadratically in it, at
// most all of it; the speed of any particle is held to one diameter
// per step, which keeps neighbors discoverable; a particle that would
// cross into a body this step is stopped just outside it; positions
// advance.
//
// Speeds are measured against the critical speed h / dt: pressure is a
// speed, cohesion a negative one, and nothing moves faster than the
// critical speed. Bodies take the equal and opposite impulse of every
// contact push, which is what floats a crate.

#include "body.h"
#include "dynamic_tree.h"
#include "particle.h"
#include "query.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

// Four neighbors at q = 1/4: the density of the rest packing.
#define M2_FLUID_REST_DENSITY 0.25f
// The weight of a pair at the rest stride.
#define M2_FLUID_REST_WEIGHT 0.25f
// No particle's pressure exceeds this share of the critical speed.
#define M2_FLUID_MAX_PRESSURE 0.25f
// How far outside a surface a crossing particle stops.
#define M2_FLUID_SURFACE_GAP 0.005f

#define M2_PARTICLE_CANDIDATES 32

// The step's scales.
typedef struct Fluid
{
    float dt;
    float invDt;
    float h;        // interaction radius: one diameter
    float critical; // h / dt
    float invMass;  // of one particle
} Fluid;

static Fluid MakeFluid(const m2World* world, float dt)
{
    Fluid f;
    f.dt = dt;
    f.invDt = 1.0f / dt;
    f.h = 2.0f * world->particles.particleRadius;
    f.critical = f.h * f.invDt;
    float stride = 0.75f * f.h;
    f.invMass = 1.0f / (world->particles.particleDensity * stride * stride);
    return f;
}

static m2Vec2 Rotate(m2Rot q, m2Vec2 v)
{
    return (m2Vec2){q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

// From a body's center of mass to a particle.
static m2Vec2 ArmTo(const m2World* world, int32_t body, m2Pos2 p)
{
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 center = Rotate(xf.q, world->bodies.localCenters[body]);
    return (m2Vec2){(float)(p.x - xf.p.x) - center.x, (float)(p.y - xf.p.y) - center.y};
}

// Pushes two particles apart along n by dv, half each.
static void Separate(m2Particles* p, int32_t a, int32_t b, m2Vec2 n, float dv)
{
    float half = 0.5f * dv;
    p->particleVelocities[a].x -= half * n.x;
    p->particleVelocities[a].y -= half * n.y;
    p->particleVelocities[b].x += half * n.x;
    p->particleVelocities[b].y += half * n.y;
}

// An impulse j on a contact's particle and its opposite on the body, at
// the particle.
static void PushContact(m2World* world, int32_t k, m2Vec2 j, float particleInvMass)
{
    m2Particles* p = &world->particles;
    int32_t i = p->particleBodyParticle[k];
    int32_t body = p->particleBodyBody[k];
    p->particleVelocities[i].x += particleInvMass * j.x;
    p->particleVelocities[i].y += particleInvMass * j.y;
    m2Vec2 arm = ArmTo(world, body, p->particlePositions[i]);
    world->bodies.linearVelocities[body].x -= world->bodies.invMass[body] * j.x;
    world->bodies.linearVelocities[body].y -= world->bodies.invMass[body] * j.y;
    world->bodies.angularVelocities[body] -=
        world->bodies.invInertia[body] * (arm.x * j.y - arm.y * j.x);
}

// The velocity of a contact's body at its particle, relative to the
// particle.
static m2Vec2 ContactSlip(const m2World* world, int32_t k)
{
    const m2Particles* p = &world->particles;
    int32_t i = p->particleBodyParticle[k];
    int32_t body = p->particleBodyBody[k];
    m2Vec2 arm = ArmTo(world, body, p->particlePositions[i]);
    float w = world->bodies.angularVelocities[body];
    m2Vec2 v = world->bodies.linearVelocities[body];
    return (m2Vec2){v.x - w * arm.y - p->particleVelocities[i].x,
                    v.y + w * arm.x - p->particleVelocities[i].y};
}

// --- Densities and pressures -------------------------------------------------

// Density in particleWeights, near density in particleAccumulation2.x.
static void Densities(m2Particles* p)
{
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        p->particleWeights[i] = 0.0f;
        p->particleAccumulation2[i] = (m2Vec2){0.0f, 0.0f};
    }
    for (int32_t k = 0; k < p->particlePairCount; ++k)
    {
        float q = p->particlePairWeight[k];
        float q2 = q * q;
        p->particleWeights[p->particlePairA[k]] += q2;
        p->particleWeights[p->particlePairB[k]] += q2;
        p->particleAccumulation2[p->particlePairA[k]].x += q2 * q;
        p->particleAccumulation2[p->particlePairB[k]].x += q2 * q;
    }
    for (int32_t k = 0; k < p->particleBodyCount; ++k)
    {
        float q = p->particleBodyWeight[k];
        p->particleWeights[p->particleBodyParticle[k]] += q * q;
        p->particleAccumulation2[p->particleBodyParticle[k]].x += q * q * q;
    }
}

// Pressure in particleAccumulation, near pressure in
// particleAccumulation2.y. Powder has neither.
static void Pressures(m2Particles* p, const Fluid* f)
{
    float cap = M2_FLUID_MAX_PRESSURE * f->critical;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        uint32_t flags = p->particleFlags[i];
        float excess = p->particleWeights[i] - M2_FLUID_REST_DENSITY;
        float pressure = excess > 0.0f ? p->particlePressureStrength * excess * f->critical : 0.0f;
        float near = 0.0f;
        if ((flags & m2_tensileParticle) != 0)
        {
            pressure = excess > 0.0f ? pressure : p->particleCohesion * excess * f->critical;
            near = p->particleNearPressure * p->particleAccumulation2[i].x * f->critical;
        }
        if ((flags & m2_powderParticle) != 0)
        {
            pressure = 0.0f;
        }
        p->particleAccumulation[i] = m2MinF(pressure, cap);
        p->particleAccumulation2[i].y = near;
    }
}

static void PressurePush(m2World* world, const Fluid* f)
{
    m2Particles* p = &world->particles;
    for (int32_t k = 0; k < p->particlePairCount; ++k)
    {
        int32_t a = p->particlePairA[k];
        int32_t b = p->particlePairB[k];
        float q = p->particlePairWeight[k];
        float dv = (p->particleAccumulation[a] + p->particleAccumulation[b]) * q +
                   (p->particleAccumulation2[a].y + p->particleAccumulation2[b].y) * q * q;
        Separate(p, a, b, p->particlePairNormal[k], dv);
    }
    // A contact pushes its particle out along the body's normal with the
    // particle's own pressure; the body takes the opposite impulse.
    for (int32_t k = 0; k < p->particleBodyCount; ++k)
    {
        int32_t i = p->particleBodyParticle[k];
        float q = p->particleBodyWeight[k];
        float dv = p->particleAccumulation[i] * q + p->particleAccumulation2[i].y * q * q;
        float j = p->particleBodyMass[k] * dv;
        m2Vec2 n = p->particleBodyNormal[k];
        PushContact(world, k, (m2Vec2){j * n.x, j * n.y}, f->invMass);
    }
}

// Grains packed tighter than the rest stride push apart; nothing pulls
// them together.
static void PowderPush(m2Particles* p, const Fluid* f)
{
    if ((p->particleFlagsUnion & m2_powderParticle) == 0)
    {
        return;
    }
    float strength = p->particlePowderStrength * f->critical;
    for (int32_t k = 0; k < p->particlePairCount; ++k)
    {
        float q = p->particlePairWeight[k];
        if ((p->particlePairFlags[k] & m2_powderParticle) != 0 && q > M2_FLUID_REST_WEIGHT)
        {
            Separate(p, p->particlePairA[k], p->particlePairB[k], p->particlePairNormal[k],
                     strength * (q - M2_FLUID_REST_WEIGHT));
        }
    }
}

// --- Spring nets and elastic triads ------------------------------------------

// A particle's position one step ahead at its current velocity.
static m2Pos2 Ahead(const m2Particles* p, int32_t i, float dt)
{
    return (m2Pos2){p->particlePositions[i].x + (double)(dt * p->particleVelocities[i].x),
                    p->particlePositions[i].y + (double)(dt * p->particleVelocities[i].y)};
}

// Each triad matches its spawn shape to where its corners are heading:
// the best-fit rotation of the spawn offsets onto the current ones about
// their mean, and a velocity toward each rotated spawn corner.
static void ElasticTriads(m2Particles* p, const Fluid* f)
{
    float strength = f->invDt * p->particleElasticStrength;
    for (int32_t k = 0; k < p->particleTriadCount; ++k)
    {
        int32_t id[3] = {p->particleTriadA[k], p->particleTriadB[k], p->particleTriadC[k]};
        m2Vec2 spawn[3] = {p->particleTriadPA[k], p->particleTriadPB[k], p->particleTriadPC[k]};
        m2Pos2 at[3] = {Ahead(p, id[0], f->dt), Ahead(p, id[1], f->dt), Ahead(p, id[2], f->dt)};
        double mx = (at[0].x + at[1].x + at[2].x) / 3.0;
        double my = (at[0].y + at[1].y + at[2].y) / 3.0;
        m2Vec2 now[3];
        float sine = 0.0f;
        float cosine = 0.0f;
        for (int32_t c = 0; c < 3; ++c)
        {
            now[c] = (m2Vec2){(float)(at[c].x - mx), (float)(at[c].y - my)};
            sine += spawn[c].x * now[c].y - spawn[c].y * now[c].x;
            cosine += spawn[c].x * now[c].x + spawn[c].y * now[c].y;
        }
        float length2 = sine * sine + cosine * cosine;
        if (length2 <= 1.0e-12f)
        {
            continue; // a triad folded flat this step: skip it
        }
        float inv = 1.0f / sqrtf(length2);
        m2Rot best = {cosine * inv, sine * inv};
        for (int32_t c = 0; c < 3; ++c)
        {
            m2Vec2 goal = Rotate(best, spawn[c]);
            p->particleVelocities[id[c]].x += strength * (goal.x - now[c].x);
            p->particleVelocities[id[c]].y += strength * (goal.y - now[c].y);
        }
    }
}

// Each spring pulls its two ends toward its spawn length, measured where
// they are heading.
static void SpringNets(m2Particles* p, const Fluid* f)
{
    float strength = f->invDt * p->particleSpringStrength;
    for (int32_t k = 0; k < p->particleSpringCount; ++k)
    {
        int32_t a = p->particleSpringA[k];
        int32_t b = p->particleSpringB[k];
        m2Pos2 pa = Ahead(p, a, f->dt);
        m2Pos2 pb = Ahead(p, b, f->dt);
        m2Vec2 d = {(float)(pb.x - pa.x), (float)(pb.y - pa.y)};
        float length = sqrtf(d.x * d.x + d.y * d.y);
        if (length > 1.0e-6f)
        {
            float pull = strength * (length - p->particleSpringRest[k]) / length;
            p->particleVelocities[a].x += pull * d.x;
            p->particleVelocities[a].y += pull * d.y;
            p->particleVelocities[b].x -= pull * d.x;
            p->particleVelocities[b].y -= pull * d.y;
        }
    }
}

// --- Viscosity ----------------------------------------------------------------

// Viscous particles lose a share c q of their relative velocity to each
// neighbor and to the bodies they touch, at most half of it.
static void ShearViscosity(m2World* world, const Fluid* f)
{
    m2Particles* p = &world->particles;
    float c = p->particleViscousStrength;
    if (!(c > 0.0f))
    {
        return;
    }
    for (int32_t k = 0; k < p->particlePairCount; ++k)
    {
        if ((p->particlePairFlags[k] & m2_viscousParticle) == 0)
        {
            continue;
        }
        int32_t a = p->particlePairA[k];
        int32_t b = p->particlePairB[k];
        float share = m2MinF(c * p->particlePairWeight[k], 0.5f);
        m2Vec2 dv = {share * (p->particleVelocities[b].x - p->particleVelocities[a].x),
                     share * (p->particleVelocities[b].y - p->particleVelocities[a].y)};
        p->particleVelocities[a].x += dv.x;
        p->particleVelocities[a].y += dv.y;
        p->particleVelocities[b].x -= dv.x;
        p->particleVelocities[b].y -= dv.y;
    }
    for (int32_t k = 0; k < p->particleBodyCount; ++k)
    {
        if ((p->particleFlags[p->particleBodyParticle[k]] & m2_viscousParticle) != 0)
        {
            float share = m2MinF(c * p->particleBodyWeight[k], 0.5f) * p->particleBodyMass[k];
            m2Vec2 slip = ContactSlip(world, k);
            PushContact(world, k, (m2Vec2){share * slip.x, share * slip.y}, f->invMass);
        }
    }
}

// The share of an approach speed u that a pair or contact of weight q
// loses: q (damping + u / critical), linear then quadratic in u.
static float ApproachShare(const m2Particles* p, const Fluid* f, float q, float u, float cap)
{
    return m2MinF(q * (p->particleDampingStrength + u / f->critical), cap);
}

// Approach damping: never a pull, and never more than the whole approach.
static void ApproachViscosity(m2World* world, const Fluid* f)
{
    m2Particles* p = &world->particles;
    for (int32_t k = 0; k < p->particlePairCount; ++k)
    {
        int32_t a = p->particlePairA[k];
        int32_t b = p->particlePairB[k];
        m2Vec2 n = p->particlePairNormal[k];
        float u = (p->particleVelocities[a].x - p->particleVelocities[b].x) * n.x +
                  (p->particleVelocities[a].y - p->particleVelocities[b].y) * n.y;
        if (u > 0.0f)
        {
            // Each end takes half of the removed approach.
            float share = ApproachShare(p, f, p->particlePairWeight[k], u, 0.5f);
            Separate(p, a, b, n, 2.0f * share * u);
        }
    }
    for (int32_t k = 0; k < p->particleBodyCount; ++k)
    {
        m2Vec2 n = p->particleBodyNormal[k];
        m2Vec2 slip = ContactSlip(world, k);
        float u = slip.x * n.x + slip.y * n.y; // the body closing on the particle
        if (u > 0.0f)
        {
            float share = ApproachShare(p, f, p->particleBodyWeight[k], u, 1.0f);
            float j = share * u * p->particleBodyMass[k];
            PushContact(world, k, (m2Vec2){j * n.x, j * n.y}, f->invMass);
        }
    }
}

// --- Speed, bodies and the advance --------------------------------------------

static void LimitSpeed(m2Particles* p, const Fluid* f)
{
    float cap2 = f->critical * f->critical;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        m2Vec2 v = p->particleVelocities[i];
        float v2 = v.x * v.x + v.y * v.y;
        if (p->particleAlive[i] != 0 && v2 > cap2)
        {
            float scale = sqrtf(cap2 / v2);
            p->particleVelocities[i] = (m2Vec2){v.x * scale, v.y * scale};
        }
    }
}

// The first body surface a particle's move for this step would cross:
// the fraction of the move and the surface normal; -1 if none. A
// particle already inside a body is left to the pressure.
static int32_t FirstCrossing(const m2World* world, m2Pos2 from, m2Vec2 move, float* fraction,
                             m2Vec2* normal)
{
    m2Aabb box = {{from.x + (move.x < 0.0f ? (double)move.x : 0.0),
                   from.y + (move.y < 0.0f ? (double)move.y : 0.0)},
                  {from.x + (move.x > 0.0f ? (double)move.x : 0.0),
                   from.y + (move.y > 0.0f ? (double)move.y : 0.0)}};
    int32_t best = -1;
    *fraction = 1.0f;
    for (int32_t t = 0; t < M2_TREE_COUNT; ++t)
    {
        int32_t shapes[M2_PARTICLE_CANDIDATES];
        int32_t hits = m2TreeQuery(&world->broadphase.trees[t], world->broadphase.treeNodes[t], box,
                                   shapes, M2_PARTICLE_CANDIDATES);
        hits = hits <= M2_PARTICLE_CANDIDATES ? hits : M2_PARTICLE_CANDIDATES;
        for (int32_t h = 0; h < hits; ++h)
        {
            int32_t shape = shapes[h];
            if (world->shapes.shapeSensor[shape] != 0)
            {
                continue;
            }
            struct m2CastHitInternal hit = m2RayCastShapeIndex(world, shape, from, move, 1.0f);
            bool inside = hit.normal.x == 0.0f && hit.normal.y == 0.0f;
            if (hit.hit && !inside &&
                (hit.fraction < *fraction || (hit.fraction == *fraction && shape < best)))
            {
                *fraction = hit.fraction;
                *normal = hit.normal;
                best = shape;
            }
        }
    }
    return best;
}

// A particle that would cross into a body stops just outside it. The
// body feels the particle through the contacts, not here.
static void KeepOutOfBodies(m2World* world, const Fluid* f)
{
    m2Particles* p = &world->particles;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        if (p->particleAlive[i] == 0)
        {
            continue;
        }
        m2Pos2 from = p->particlePositions[i];
        m2Vec2 move = {p->particleVelocities[i].x * f->dt, p->particleVelocities[i].y * f->dt};
        float fraction;
        m2Vec2 n = {0.0f, 0.0f};
        if (FirstCrossing(world, from, move, &fraction, &n) >= 0)
        {
            double x = from.x + (double)(fraction * move.x) + (double)(M2_FLUID_SURFACE_GAP * n.x);
            double y = from.y + (double)(fraction * move.y) + (double)(M2_FLUID_SURFACE_GAP * n.y);
            p->particleVelocities[i] =
                (m2Vec2){(float)(x - from.x) * f->invDt, (float)(y - from.y) * f->invDt};
        }
    }
}

static void Advance(m2Particles* p, float dt)
{
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        if (p->particleAlive[i] != 0)
        {
            p->particlePositions[i].x += (double)(p->particleVelocities[i].x * dt);
            p->particlePositions[i].y += (double)(p->particleVelocities[i].y * dt);
        }
    }
}

static void Gravity(m2World* world, float dt)
{
    m2Particles* p = &world->particles;
    float gx = dt * p->particleGravityScale * world->gravity.x;
    float gy = dt * p->particleGravityScale * world->gravity.y;
    for (int32_t i = 0; i < p->maxParticleIndex; ++i)
    {
        if (p->particleAlive[i] != 0)
        {
            p->particleVelocities[i].x += gx;
            p->particleVelocities[i].y += gy;
        }
    }
}

void m2SolveParticles(m2World* world, float dt)
{
    m2UpdateParticlePairs(world);
    m2UpdateParticleContacts(world);
    if (dt <= 0.0f)
    {
        return;
    }
    Fluid f = MakeFluid(world, dt);
    m2Particles* p = &world->particles;
    Gravity(world, dt);
    Densities(p);
    Pressures(p, &f);
    PressurePush(world, &f);
    PowderPush(p, &f);
    ElasticTriads(p, &f);
    SpringNets(p, &f);
    ShearViscosity(world, &f);
    ApproachViscosity(world, &f);
    LimitSpeed(p, &f);
    KeepOutOfBodies(world, &f);
    Advance(p, dt);
}
