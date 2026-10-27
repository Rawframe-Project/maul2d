// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ten-minute Maul2D: a ground, a tower, one snapshot, one
// rollback. Build against an installed maul2d (see README.md here).
#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

// Stops the program when a call fails; a sample has nothing to fall back
// to.
static void Require(m2Result result, const char* what)
{
    if (result != m2_success)
    {
        fprintf(stderr, "%s: %s\n", what, m2ResultName(result));
        exit(1);
    }
}

// Adds a box body; the world is sized for every body this program makes.
static m2BodyId AddBox(m2World* world, const m2BodyDef* def, float halfWidth, float halfHeight)
{
    m2BodyId body = m2_nullBodyId;
    Require(m2CreateBody(world, def, &body), "create a body");
    m2ShapeDef sd = m2DefaultShapeDef();
    m2Polygon box = m2MakeBox(halfWidth, halfHeight);
    m2ShapeId shape = m2_nullShapeId;
    Require(m2CreatePolygonShape(world, body, &sd, &box, &shape), "shape a body");
    return body;
}

int main(void)
{
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 8;
    m2World* world = NULL;
    Require(m2CreateWorld(&def, &world), "create the world");

    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -0.5};
    AddBox(world, &groundDef, 10.0f, 0.5f);

    m2BodyId top = m2_nullBodyId;
    for (int i = 0; i < 5; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){0.0, 0.5 + (double)i * 1.02};
        top = AddBox(world, &bd, 0.5f, 0.5f);
    }

    // Simulate one second, snapshot, wreck the tower, then roll back:
    // the wreck never happened, bit for bit.
    for (int i = 0; i < 60; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    int64_t size = m2World_SnapshotSize(world);
    void* snap = malloc((size_t)size);
    if (snap == NULL)
    {
        return 1;
    }
    int64_t written = 0;
    Require(m2World_Snapshot(world, snap, size, &written), "take a snapshot");
    unsigned long long before = (unsigned long long)m2World_Hash(world);

    m2Body_ApplyLinearImpulse(world, top, (m2Vec2){8.0f, 4.0f});
    for (int i = 0; i < 120; ++i)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    printf("wrecked hash %016llx\n", (unsigned long long)m2World_Hash(world));

    Require(m2World_Restore(world, snap, written), "restore the snapshot");
    int matches = m2World_Hash(world) == before;
    printf("restored hash %016llx (matches the snapshot moment: %s)\n",
           (unsigned long long)m2World_Hash(world), matches ? "yes" : "NO");

    m2Pos2 p = m2Body_GetPosition(world, top);
    printf("tower top after rollback: (%.2f, %.2f)\n", p.x, p.y);
    free(snap);
    m2DestroyWorld(world);
    return matches ? 0 : 1;
}
