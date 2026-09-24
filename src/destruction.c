// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Destruction: explosions and shattering bodies into pieces.

#include "destruction.h"

#include "body.h"
#include "distance.h"
#include "geometry.h"
#include "journal.h"
#include "world.h"
#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <string.h>

m2ExplosionDef m2DefaultExplosionDef(void)
{
    m2ExplosionDef def;
    memset(&def, 0, sizeof(def));
    def.radius = 1.0f;
    def.falloff = 0.5f;
    def.impulse = 1.0f;
    def.maskBits = 0xFFFFFFFFu;
    def.internalValue = M2_EXPLODE_COOKIE;
    return def;
}

// Pushes one shape's body away from the blast center. The impulse lands
// at the shape's closest point, found in the body frame (one f64
// crossing).
static void BlastShape(m2World* world, const m2ExplosionDef* def, int32_t s)
{
    int32_t body = world->shapes.shapeBody[s];
    m2Transform xf = world->bodies.transforms[body];
    m2Vec2 rel = {(float)(def->position.x - xf.p.x), (float)(def->position.y - xf.p.y)};
    m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
    m2DistanceProxy target = m2GeometryProxy(&world->shapes.shapeGeometry[s]);
    m2DistanceProxy point;
    point.points[0] = local;
    point.count = 1;
    point.radius = 0.0f;
    m2DistanceResult d = m2ShapeDistance(&target, &point);
    float dist = d.distance - target.radius;
    if (dist > def->radius + def->falloff)
    {
        return;
    }
    // Away from the center; deep overlap falls back to the body-center
    // direction, and a dead-centered blast on a centered body skips.
    m2Vec2 lc = world->bodies.localCenters[body];
    m2Vec2 dir = {-d.normal.x, -d.normal.y}; // the normal points shape to center
    if (!(dist > 0.0f))
    {
        dir = (m2Vec2){lc.x - local.x, lc.y - local.y};
        float len = sqrtf(dir.x * dir.x + dir.y * dir.y);
        if (!(len > 0.0f))
        {
            return;
        }
        dir = (m2Vec2){dir.x / len, dir.y / len};
        dist = 0.0f;
    }
    float scale = dist <= def->radius ? 1.0f : 1.0f - (dist - def->radius) / def->falloff;
    float mag = def->impulse * scale;
    m2Vec2 hitLocal = {d.pointA.x + target.radius * d.normal.x,
                       d.pointA.y + target.radius * d.normal.y};
    m2Vec2 arm = {hitLocal.x - lc.x, hitLocal.y - lc.y};
    m2Vec2 impulseLocal = {mag * dir.x, mag * dir.y};
    // The linear update needs world axes; the angular one uses the local
    // cross, identical either way.
    m2Vec2 impulseWorld = {xf.q.c * impulseLocal.x - xf.q.s * impulseLocal.y,
                           xf.q.s * impulseLocal.x + xf.q.c * impulseLocal.y};
    world->bodies.linearVelocities[body].x += world->bodies.invMass[body] * impulseWorld.x;
    world->bodies.linearVelocities[body].y += world->bodies.invMass[body] * impulseWorld.y;
    world->bodies.angularVelocities[body] +=
        world->bodies.invInertia[body] * (arm.x * impulseLocal.y - arm.y * impulseLocal.x);
    world->bodies.asleep[body] = 0;
    world->bodies.sleepTimes[body] = 0.0f;
}

void m2World_Explode(m2WorldId worldId, const m2ExplosionDef* def)
{
    m2World* world = m2GetWorld(worldId);
    if (world == NULL || def == NULL || def->internalValue != M2_EXPLODE_COOKIE ||
        !(def->radius >= 0.0f) || !(def->falloff > 0.0f))
    {
        m2Refuse(world, m2_errorInvalid);
        return;
    }
    m2JournalRecord(world, m2_opExplode, def, (int32_t)sizeof(*def));
    for (int32_t s = 0; s < world->shapes.maxShapeIndex; ++s)
    {
        int32_t body = world->shapes.shapeBody[s];
        if (world->shapes.shapeAlive[s] != 0 && world->shapes.shapeSensor[s] == 0 &&
            (world->shapes.shapeCategory[s] & def->maskBits) != 0 &&
            world->bodies.types[body] == (uint8_t)m2_dynamicBody &&
            world->bodies.disabled[body] == 0)
        {
            BlastShape(world, def, s);
        }
    }
}

// --- Shatter: the destruction road ------------------------------------------------

// Materials and filter ride from the parent's first shape; a shapeless
// parent hands out defaults.
static m2ShapeDef PieceShapeDef(const m2World* world, int32_t parent)
{
    m2ShapeDef def = m2DefaultShapeDef();
    int32_t first = world->bodies.bodyShapeHead[parent];
    if (first != -1)
    {
        def.density = world->shapes.shapeDensity[first];
        def.friction = world->shapes.shapeFriction[first];
        def.restitution = world->shapes.shapeRestitution[first];
        def.tangentSpeed = world->shapes.shapeTangentSpeed[first];
        def.categoryBits = world->shapes.shapeCategory[first];
        def.maskBits = world->shapes.shapeMask[first];
        def.groupIndex = world->shapes.shapeGroup[first];
    }
    return def;
}

// Creates one piece at the parent's pose, moving with the parent's rigid
// field sampled at the piece's own center of mass.
static m2BodyId CreatePiece(m2World* world, int32_t parent, const m2ShapeDef* shapeDef,
                            const m2Polygon* polygon)
{
    m2Transform xf = world->bodies.transforms[parent];
    m2Vec2 vParent = world->bodies.linearVelocities[parent];
    float wParent = world->bodies.angularVelocities[parent];
    m2Vec2 lcParent = world->bodies.localCenters[parent];
    m2Vec2 comArm = {xf.q.c * lcParent.x - xf.q.s * lcParent.y,
                     xf.q.s * lcParent.x + xf.q.c * lcParent.y};
    m2BodyDef bd = m2DefaultBodyDef();
    bd.type = m2_dynamicBody;
    bd.position = xf.p;
    bd.rotation = xf.q;
    bd.gravityScale = world->bodies.gravityScales[parent];
    bd.linearDamping = world->bodies.linearDampings[parent];
    bd.angularDamping = world->bodies.angularDampings[parent];
    m2BodyId piece =
        m2CreateBody((m2WorldId){(uint16_t)(world->slot + 1), world->worldGeneration}, &bd);
    m2CreatePolygonShape(piece, shapeDef, polygon);
    int32_t index = piece.index1 - 1;
    m2Vec2 lc = world->bodies.localCenters[index];
    m2Vec2 arm = {xf.q.c * lc.x - xf.q.s * lc.y, xf.q.s * lc.x + xf.q.c * lc.y};
    float rx = arm.x - comArm.x;
    float ry = arm.y - comArm.y;
    world->bodies.linearVelocities[index] =
        (m2Vec2){vParent.x - wParent * ry, vParent.y + wParent * rx};
    world->bodies.angularVelocities[index] = wParent;
    return piece;
}

int32_t m2World_ShatterBody(m2BodyId bodyId, const m2Polygon* pieces, int32_t pieceCount,
                            m2BodyId* outBodies, int32_t capacity)
{
    m2World* world = m2WorldFromTag(bodyId.world);
    int32_t parent = world != NULL ? m2BodySlot(world, bodyId) : -1;
    bool valid = parent >= 0 && pieces != NULL && pieceCount >= 1 && pieceCount <= 64 &&
                 world->bodies.types[parent] == (uint8_t)m2_dynamicBody;
    for (int32_t i = 0; valid && i < pieceCount; ++i)
    {
        valid = m2ValidatePolygon(&pieces[i]);
    }
    if (!valid)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    if (world->bodies.freeCount < pieceCount || world->shapes.shapeFreeCount < pieceCount)
    {
        m2Refuse(world, m2_errorCapacity); // all pieces or none
        return 0;
    }
    m2ShapeDef shapeDef = PieceShapeDef(world, parent);
    uint8_t journalWasActive = world->recorder.journalActive;
    world->recorder.journalActive = 0;
    int32_t firstIndex1 = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2BodyId piece = CreatePiece(world, parent, &shapeDef, &pieces[i]);
        firstIndex1 = i == 0 ? piece.index1 : firstIndex1;
        if (outBodies != NULL && i < capacity)
        {
            outBodies[i] = piece;
        }
    }
    m2DestroyBody(bodyId); // joints die with it, touching sleepers wake
    world->recorder.journalActive = journalWasActive;
    m2JournalRecordShatter(world, bodyId, pieces, pieceCount, firstIndex1);
    return pieceCount;
}
