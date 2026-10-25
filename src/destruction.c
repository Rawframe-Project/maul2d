// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Destruction: explosions and shattering bodies into pieces.

#include "destruction.h"

#include "body.h"
#include "distance.h"
#include "fpenv.h"
#include "geometry.h"
#include "journal.h"
#include "object_ids.h"
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
    def.maskBits = UINT64_MAX;
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
    m2Vec2 rel = {m2WorldOffset(def->position.x - xf.p.x), m2WorldOffset(def->position.y - xf.p.y)};
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
    m2SaturateBodyVelocity(world, body);
    world->bodies.asleep[body] = 0;
    world->bodies.sleepTimes[body] = 0.0f;
}

static M2_NOINLINE void WorldExplodeWorker(m2World* world, const m2ExplosionDef* def)
{
    if (m2CheckWriter(world) != m2_success)
    {
        return;
    }
    if (world == NULL || def == NULL || def->internalValue != M2_EXPLODE_COOKIE ||
        !m2LocalLength(def->radius) || !m2LocalLength(def->falloff) || !(def->falloff > 0.0f) ||
        !m2ValidImpulse(def->impulse) || !m2FinitePos2(def->position))
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

void m2World_Explode(m2World* world, const m2ExplosionDef* def)
{
    uint64_t env = m2FloatEnvEnter();
    WorldExplodeWorker(world, def);
    m2FloatEnvLeave(env);
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
    // The caller checked both pools, and the trees hold two nodes per
    // shape slot, so neither create can fail.
    m2BodyId piece = m2_nullBodyId;
    m2Result made = m2CreateBody(world, &bd, &piece);
    M2_ASSERT(made == m2_success);
    m2ShapeId shape = m2_nullShapeId;
    made = m2CreatePolygonShape(world, piece, shapeDef, polygon, &shape);
    M2_ASSERT(made == m2_success);
    (void)made;
    int32_t index = (int32_t)piece.index1 - 1;
    m2Vec2 lc = world->bodies.localCenters[index];
    m2Vec2 arm = {xf.q.c * lc.x - xf.q.s * lc.y, xf.q.s * lc.x + xf.q.c * lc.y};
    float rx = arm.x - comArm.x;
    float ry = arm.y - comArm.y;
    world->bodies.linearVelocities[index] =
        (m2Vec2){vParent.x - wParent * ry, vParent.y + wParent * rx};
    world->bodies.angularVelocities[index] = wParent;
    m2SaturateBodyVelocity(world, index);
    return piece;
}

static M2_NOINLINE m2Result WorldShatterBodyWorker(m2World* world, m2BodyId bodyId,
                                                   const m2Polygon* pieces, int32_t pieceCount,
                                                   m2BodyId* bodiesOut, int32_t capacity,
                                                   int32_t* countOut)
{
    if (countOut != NULL)
    {
        *countOut = 0;
    }
    if (countOut == NULL || capacity < 0 || (bodiesOut == NULL && capacity > 0))
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    for (int32_t i = 0; i < capacity; ++i)
    {
        bodiesOut[i] = m2_nullBodyId;
    }
    m2Result writer = m2CheckWriter(world);
    if (writer != m2_success)
    {
        return writer;
    }
    int32_t parent = m2BodySlot(world, bodyId);
    if (parent < 0)
    {
        return m2Refuse(world, m2_errorStale);
    }
    bool valid = pieces != NULL && pieceCount >= 1 && pieceCount <= 64 &&
                 world->bodies.types[parent] == (uint8_t)m2_dynamicBody;
    for (int32_t i = 0; valid && i < pieceCount; ++i)
    {
        valid = m2ValidatePolygon(&pieces[i]);
    }
    if (!valid)
    {
        return m2Refuse(world, m2_errorInvalid);
    }
    if (world->bodies.freeCount < pieceCount || world->shapes.shapeFreeCount < pieceCount)
    {
        return m2Refuse(world, m2_errorCapacity); // all pieces or none
    }
    m2ShapeDef shapeDef = PieceShapeDef(world, parent);
    uint8_t journalWasActive = world->recorder.journalActive;
    world->recorder.journalActive = 0;
    int32_t firstIndex1 = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2BodyId piece = CreatePiece(world, parent, &shapeDef, &pieces[i]);
        firstIndex1 = i == 0 ? (int32_t)piece.index1 : firstIndex1;
        if (bodiesOut != NULL && i < capacity)
        {
            bodiesOut[i] = piece;
        }
    }
    m2Result destroyed = m2DestroyBody(world, bodyId); // joints die with it, sleepers wake
    M2_ASSERT(destroyed == m2_success);
    (void)destroyed;
    world->recorder.journalActive = journalWasActive;
    m2JournalRecordShatter(world, bodyId, pieces, pieceCount, firstIndex1);
    *countOut = pieceCount;
    return m2_success;
}

m2Result m2World_ShatterBody(m2World* world, m2BodyId bodyId, const m2Polygon* pieces,
                             int32_t pieceCount, m2BodyId* bodiesOut, int32_t capacity,
                             int32_t* countOut)
{
    uint64_t env = m2FloatEnvEnter();
    m2Result result =
        WorldShatterBodyWorker(world, bodyId, pieces, pieceCount, bodiesOut, capacity, countOut);
    m2FloatEnvLeave(env);
    return result;
}
