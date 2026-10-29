// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Record a session into the command journal, then replay it into a
// completely fresh world and compare the bits. This is the primitive
// that deterministic lockstep, replays and bug reports are built on.

#include "maul2d/maul2d.h"

#include <stdio.h>
#include <stdlib.h>

// Stops the program when a call fails; a sample has nothing to fall back
// to.
static void Require(m2Result result, const char* what)
{
    if (result != m2_success)
    {
        fprintf(stderr, "%s: %s\n", what, m2GetResultName(result));
        exit(1);
    }
}

static void Shove(m2World* world, m2BodyId body, float impulseX)
{
    m2Body_ApplyLinearImpulseAtPoint(world, body, (m2Vec2){impulseX, 0.0f},
                                     m2Body_GetPosition(world, body));
}

int main(void)
{
    // The journal embeds a full snapshot up front, so size the tape
    // from m2World_GetJournalBaseSize plus room for the ops you expect.
    m2WorldDef def = m2DefaultWorldDef();
    def.bodyCapacity = 64;
    def.shapeCapacity = 64;
    def.jointCapacity = 8;
    m2World* world = NULL;
    Require(m2CreateWorld(&def, &world), "create the world");

    int64_t tapeCapacity = m2World_GetJournalBaseSize(world) + (1 << 16);
    unsigned char* journal = malloc((size_t)tapeCapacity);
    if (journal == NULL)
    {
        return 1;
    }
    Require(m2World_StartJournal(world, journal, tapeCapacity), "start the journal");

    // Everything from here on is being recorded: creation, physics
    // steps, impulses, even destruction.
    m2BodyDef groundDef = m2DefaultBodyDef();
    groundDef.position = (m2Pos2){0.0, -0.5};
    m2BodyId ground = m2_nullBodyId;
    Require(m2CreateBody(world, &groundDef, &ground), "create the ground");
    m2ShapeDef groundShape = m2DefaultShapeDef();
    m2Polygon slab = m2MakeBox(20.0f, 0.5f);
    m2ShapeId shape = m2_nullShapeId;
    Require(m2CreatePolygonShape(world, ground, &groundShape, &slab, &shape), "shape the ground");

    m2BodyId boxes[5];
    for (int i = 0; i < 5; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = (m2Pos2){-2.0 + (double)i, 2.0 + 0.5 * (double)i};
        Require(m2CreateBody(world, &bd, &boxes[i]), "create a box");
        m2ShapeDef sd = m2DefaultShapeDef();
        m2Polygon unit = m2MakeBox(0.4f, 0.4f);
        Require(m2CreatePolygonShape(world, boxes[i], &sd, &unit, &shape), "shape a box");
    }

    for (int step = 0; step < 60; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }
    Shove(world, boxes[2], 3.0f);                             // meddle mid-session
    Require(m2DestroyBody(world, boxes[4]), "destroy a box"); // and destroy something
    for (int step = 0; step < 60; ++step)
    {
        m2World_Step(world, 1.0f / 60.0f, 4);
    }

    int64_t bytes = 0;
    Require(m2World_StopJournal(world, &bytes), "stop the journal");
    unsigned long long recorded = (unsigned long long)m2World_Hash(world);
    printf("recorded %lld bytes of session, end hash %016llx\n", (long long)bytes, recorded);

    // A brand-new world, same capacities, replay the tape.
    m2World* fresh = NULL;
    Require(m2CreateWorld(&def, &fresh), "create the fresh world");
    Require(m2World_ReplayJournal(fresh, journal, bytes), "replay the tape");
    unsigned long long replayed = (unsigned long long)m2World_Hash(fresh);
    printf("replayed into a fresh world,  end hash %016llx\n", replayed);
    printf(recorded == replayed ? "bit-exact match.\n" : "MISMATCH - file a bug!\n");

    m2DestroyWorld(fresh);
    m2DestroyWorld(world);
    free(journal);
    return recorded == replayed ? 0 : 1;
}
