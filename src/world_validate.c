// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The world's own consistency check behind m2World_Validate: every
// live body, joint and particle and the links between them.

#include "fpenv.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

// Fails the walk on the first broken invariant; m2World_Validate never
// stops the program, and the validate build asserts its result instead.
#define M2_CHECK_INVARIANT(cond)                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            return false;                                                                          \
        }                                                                                          \
    } while (0)

static bool ValidBody(const m2World* world, int32_t i)
{
    m2Transform xf = world->bodies.transforms[i];
    M2_CHECK_INVARIANT(m2FinitePos2(xf.p));
    M2_CHECK_INVARIANT(m2UnitRot(xf.q));
    m2Vec2 v = world->bodies.linearVelocities[i];
    M2_CHECK_INVARIANT(m2FiniteVec2(v));
    M2_CHECK_INVARIANT(m2FiniteF(world->bodies.angularVelocities[i]));
    M2_CHECK_INVARIANT(world->bodies.types[i] <= 2);
    return true;
}

static bool ValidBodies(const m2World* world)
{
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        M2_CHECK_INVARIANT(world->bodies.alive[i] == 0 || ValidBody(world, i));
    }
    return true;
}

static bool LiveBody(const m2World* world, int32_t body)
{
    return body >= 0 && body < world->bodies.bodyCapacity && world->bodies.alive[body] != 0;
}

static bool ValidJoint(const m2World* world, int32_t i)
{
    M2_CHECK_INVARIANT(world->joints.jointType[i] <= 10);
    M2_CHECK_INVARIANT(LiveBody(world, world->joints.jointBodyA[i]));
    M2_CHECK_INVARIANT(LiveBody(world, world->joints.jointBodyB[i]));
    return true;
}

// Shapes and joints point at live bodies; contact pairs stay in key order.
static bool ValidLinks(const m2World* world)
{
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        M2_CHECK_INVARIANT(world->shapes.shapeAlive[i] == 0 ||
                           LiveBody(world, world->shapes.shapeBody[i]));
    }
    for (int32_t i = 0; i < world->joints.maxJointIndex; ++i)
    {
        M2_CHECK_INVARIANT(world->joints.jointAlive[i] == 0 || ValidJoint(world, i));
    }
    for (int32_t i = 1; i < world->contacts.pairCount; ++i)
    {
        // The canonical ordering law, checked where it lives.
        M2_CHECK_INVARIANT(world->contacts.pairKeys[i - 1] < world->contacts.pairKeys[i]);
    }
    return true;
}

// Springs and triads join live particles only.
static bool ValidParticleLinks(const m2World* world)
{
    const uint8_t* alive = world->particles.particleAlive;
    for (int32_t k = 0; k < world->particles.particleSpringCount; ++k)
    {
        M2_CHECK_INVARIANT(alive[world->particles.particleSpringA[k]] != 0 &&
                           alive[world->particles.particleSpringB[k]] != 0);
    }
    for (int32_t k = 0; k < world->particles.particleTriadCount; ++k)
    {
        M2_CHECK_INVARIANT(alive[world->particles.particleTriadA[k]] != 0 &&
                           alive[world->particles.particleTriadB[k]] != 0 &&
                           alive[world->particles.particleTriadC[k]] != 0);
    }
    return true;
}

static bool ValidParticles(const m2World* world)
{
    if (world->particles.particleCapacity == 0)
    {
        return true;
    }
    int32_t alive = 0;
    for (int32_t i = 0; i < world->particles.maxParticleIndex; ++i)
    {
        if (world->particles.particleAlive[i] == 0)
        {
            continue;
        }
        alive += 1;
        M2_CHECK_INVARIANT(m2FinitePos2(world->particles.particlePositions[i]));
        M2_CHECK_INVARIANT(m2FiniteVec2(world->particles.particleVelocities[i]));
    }
    M2_CHECK_INVARIANT(alive == world->particles.particleCount);
    return ValidParticleLinks(world);
}

#undef M2_CHECK_INVARIANT

// The invariant walk: everything a healthy world must be able to
// say about itself, checked loudly. Pure reader.
static M2_NOINLINE bool WorldValidateWorker(const m2World* world)
{
    if (world == NULL)
    {
        m2Refuse(world, m2_errorInvalid);
        return false;
    }
    return ValidBodies(world) && ValidLinks(world) && ValidParticles(world);
}

bool m2World_Validate(const m2World* world)
{
    uint64_t env = m2FloatEnvEnter();
    bool result = WorldValidateWorker(world);
    m2FloatEnvLeave(env);
    return result;
}
