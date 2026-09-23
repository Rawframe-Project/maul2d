// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Islands and sleeping. Islands are rebuilt each step with a
// deterministic union-find over touching contacts in canonical pair
// order. Rebuilding from canonical inputs is history-free, so island
// structure needs no snapshot blocks. What DOES persist - and is
// snapshot state and hashed - is per-body sleep: the asleep flag and
// the sleep timer.
//
// Wake rules: any awake member wakes its whole island; a moving
// kinematic touching the island disturbs it; API setters wake
// their body directly, and the island coupling spreads it next step.

#include "island.h"
#include "world_internal.h"

#include "maul2d/base.h"

#define M2_SLEEP_LINEAR_TOLERANCE  0.05f // m/s
#define M2_SLEEP_ANGULAR_TOLERANCE 0.12f // rad/s
#define M2_TIME_TO_SLEEP           0.5f  // seconds under tolerance

static int32_t Find(int32_t* parent, int32_t i)
{
    while (parent[i] != i)
    {
        parent[i] = parent[parent[i]]; // halving; deterministic
        i = parent[i];
    }
    return i;
}

static void Union(int32_t* parent, int32_t a, int32_t b)
{
    int32_t ra = Find(parent, a);
    int32_t rb = Find(parent, b);
    // Min-root union: canonical regardless of processing order.
    if (ra < rb)
    {
        parent[rb] = ra;
    }
    else if (rb < ra)
    {
        parent[ra] = rb;
    }
}

static bool BodySlow(const m2World* world, int32_t i)
{
    m2Vec2 v = world->bodies.linearVelocities[i];
    float w = world->bodies.angularVelocities[i];
    return v.x * v.x + v.y * v.y < M2_SLEEP_LINEAR_TOLERANCE * M2_SLEEP_LINEAR_TOLERANCE &&
           w > -M2_SLEEP_ANGULAR_TOLERANCE && w < M2_SLEEP_ANGULAR_TOLERANCE;
}

// Pre-solve: build islands over touching contacts, wake islands that
// have any awake member or a moving-kinematic toucher.
void m2UpdateIslandsAndWake(m2World* world)
{
    int32_t* parent = world->solver.islandParent;
    uint8_t* disturbed = world->solver.islandDisturbed;
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        parent[i] = i;
        disturbed[i] = 0;
    }

    // Union dynamic bodies over touching contacts (canonical order);
    // note moving-kinematic touches as disturbances.
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        if (world->contacts.manifolds[i].pointCount == 0)
        {
            continue;
        }
        int32_t sa = (int32_t)(world->contacts.pairKeys[i] >> 32);
        int32_t sb = (int32_t)(world->contacts.pairKeys[i] & 0xFFFFFFFFu);
        if (world->shapes.shapeSensor[sa] != 0 || world->shapes.shapeSensor[sb] != 0)
        {
            continue; // sensors never couple islands or disturb sleep
        }
        int32_t bodyA = world->shapes.shapeBody[sa];
        int32_t bodyB = world->shapes.shapeBody[sb];
        bool dynA = world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody;
        bool dynB = world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody;
        if (dynA && dynB)
        {
            Union(parent, bodyA, bodyB);
        }
        else if (dynA || dynB)
        {
            int32_t dynamic = dynA ? bodyA : bodyB;
            int32_t other = dynA ? bodyB : bodyA;
            if (world->bodies.types[other] == (uint8_t)m2_kinematicBody && !BodySlow(world, other))
            {
                disturbed[dynamic] = 1;
            }
        }
    }

    // Joints connect islands exactly like touching contacts do.
    for (int32_t j = 0; j < world->joints.maxJointIndex; ++j)
    {
        if (world->joints.jointAlive[j] == 0 ||
            world->bodies.disabled[world->joints.jointBodyA[j]] != 0 ||
            world->bodies.disabled[world->joints.jointBodyB[j]] != 0)
        {
            continue;
        }
        int32_t bodyA = world->joints.jointBodyA[j];
        int32_t bodyB = world->joints.jointBodyB[j];
        if (world->bodies.types[bodyA] == (uint8_t)m2_dynamicBody &&
            world->bodies.types[bodyB] == (uint8_t)m2_dynamicBody)
        {
            Union(parent, bodyA, bodyB);
        }
    }

#ifndef NDEBUG
    // Min-root union: every path must lead downward in index space.
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        M2_ASSERT(Find(parent, i) <= i);
    }
#endif

    // Fold member-awake and disturbance flags up to the roots.
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        int32_t root = Find(parent, i);
        if (world->bodies.asleep[i] == 0 || disturbed[i] != 0)
        {
            disturbed[root] = 2; // root marker: island must be awake
        }
    }

    // Wake every member of awake islands (fixed body order).
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody)
        {
            continue;
        }
        if (disturbed[Find(parent, i)] == 2 && world->bodies.asleep[i] != 0)
        {
            world->bodies.asleep[i] = 0;
            world->bodies.sleepTimes[i] = 0.0f;
        }
    }
}

// Post-solve: island-coupled sleep accounting. An island sleeps only
// when every member has stayed under tolerance for the full window; one
// fast member resets the whole island.
void m2UpdateSleep(m2World* world, float dt)
{
    int32_t* parent = world->solver.islandParent;
    uint8_t* islandFast = world->solver.islandDisturbed; // reuse scratch

    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        islandFast[i] = 0;
    }
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[i] != 0)
        {
            continue;
        }
        if (!BodySlow(world, i))
        {
            islandFast[Find(parent, i)] = 1;
        }
    }

    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[i] != 0)
        {
            continue;
        }
        if (islandFast[Find(parent, i)] != 0)
        {
            world->bodies.sleepTimes[i] = 0.0f;
            continue;
        }
        if (world->sleepEnabled == 0 || world->bodies.sleepEnables[i] == 0 ||
            world->bodies.disabled[i] != 0)
        {
            world->bodies.sleepTimes[i] = 0.0f; // this body is not allowed to drowse
            continue;
        }
        world->bodies.sleepTimes[i] += dt;
    }

    // Island sleeps when its minimum timer crosses the window. Two
    // passes keep it order-canonical: find sleepy roots, then apply.
    uint8_t* rootSleeps = islandFast; // reuse again: 1 = candidate
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        rootSleeps[i] = 0;
    }
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[i] != 0)
        {
            continue;
        }
        int32_t root = Find(parent, i);
        if (rootSleeps[root] == 0)
        {
            rootSleeps[root] = 1; // assume sleepy until a member objects
        }
        if (world->bodies.sleepTimes[i] < M2_TIME_TO_SLEEP)
        {
            rootSleeps[root] = 2; // objection: someone is not ready
        }
    }
    for (int32_t i = 0; i < world->bodies.maxBodyIndex; ++i)
    {
        if (world->bodies.alive[i] == 0 || world->bodies.types[i] != (uint8_t)m2_dynamicBody ||
            world->bodies.asleep[i] != 0)
        {
            continue;
        }
        if (rootSleeps[Find(parent, i)] == 1)
        {
            world->bodies.asleep[i] = 1;
            world->bodies.linearVelocities[i] = (m2Vec2){0.0f, 0.0f};
            world->bodies.angularVelocities[i] = 0.0f;
        }
    }
}
