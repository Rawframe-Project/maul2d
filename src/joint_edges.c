// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The joint edges of each body: the lists that tell which joints touch a
// body and which jointed pairs may not collide.

#include "joint_edges.h"

// Inserts edge (2 * joint + side) into body's list, keeping the list in
// ascending joint order so walks visit joints in slot order.
static void LinkJointEdge(m2World* world, int32_t body, int32_t edge)
{
    int32_t joint = edge >> 1;
    int32_t* link = &world->joints.bodyJointHead[body];
    while (*link != -1 && (*link >> 1) < joint)
    {
        link = &world->joints.jointEdgeNext[*link];
    }
    world->joints.jointEdgeNext[edge] = *link;
    *link = edge;
}

static void UnlinkJointEdge(m2World* world, int32_t body, int32_t edge)
{
    int32_t* link = &world->joints.bodyJointHead[body];
    while (*link != -1 && *link != edge)
    {
        link = &world->joints.jointEdgeNext[*link];
    }
    M2_ASSERT(*link == edge);
    if (*link == edge)
    {
        *link = world->joints.jointEdgeNext[edge];
        world->joints.jointEdgeNext[edge] = -1;
    }
}

void m2LinkJoint(m2World* world, int32_t joint)
{
    LinkJointEdge(world, world->joints.jointBodyA[joint], 2 * joint);
    LinkJointEdge(world, world->joints.jointBodyB[joint], 2 * joint + 1);
}

void m2UnlinkJoint(m2World* world, int32_t joint)
{
    UnlinkJointEdge(world, world->joints.jointBodyA[joint], 2 * joint);
    UnlinkJointEdge(world, world->joints.jointBodyB[joint], 2 * joint + 1);
}

// Rebuilds every adjacency list from the joint arrays, after a restore
// has replaced them wholesale.
void m2RebuildJointEdges(m2World* world)
{
    for (int32_t b = 0; b < world->bodies.bodyCapacity; ++b)
    {
        world->joints.bodyJointHead[b] = -1;
    }
    for (int32_t e = 0; e < 2 * world->joints.jointCapacity; ++e)
    {
        world->joints.jointEdgeNext[e] = -1;
    }
    for (int32_t j = 0; j < world->joints.maxJointIndex; ++j)
    {
        if (world->joints.jointAlive[j] != 0)
        {
            m2LinkJoint(world, j);
        }
    }
}

// Jointed bodies do not collide unless the joint allows it. Walks only
// body A's joints, so the cost is its joint count, not the world's.
bool m2JointsForbidPair(const m2World* world, int32_t bodyA, int32_t bodyB)
{
    for (int32_t e = world->joints.bodyJointHead[bodyA]; e != -1;
         e = world->joints.jointEdgeNext[e])
    {
        int32_t j = e >> 1;
        int32_t other = (e & 1) != 0 ? world->joints.jointBodyA[j] : world->joints.jointBodyB[j];
        if (other == bodyB && world->joints.jointCollide[j] == 0)
        {
            return true;
        }
    }
    return false;
}
