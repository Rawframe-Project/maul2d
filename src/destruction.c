// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Destruction: explosions and shattering bodies into pieces.

#include "destruction.h"

#include "body.h"
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
    // The blast applies raw impulses below; they must not be recorded
    // twice (the chain-create suppression pattern).
    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    float reach = def->radius + def->falloff;
    for (int32_t s = 0; s < world->maxShapeIndex; ++s)
    {
        if (world->shapeAlive[s] == 0 || world->shapeSensor[s] != 0 ||
            (world->shapeCategory[s] & def->maskBits) == 0)
        {
            continue;
        }
        int32_t body = world->shapeBody[s];
        if (world->types[body] != (uint8_t)m2_dynamicBody || world->disabled[body] != 0)
        {
            continue;
        }
        // Closest point on the shape to the blast center, in the
        // shape's body frame (one f64 crossing).
        m2Transform xf = world->transforms[body];
        m2Vec2 rel = {(float)(def->position.x - xf.p.x), (float)(def->position.y - xf.p.y)};
        m2Vec2 local = {xf.q.c * rel.x + xf.q.s * rel.y, -xf.q.s * rel.x + xf.q.c * rel.y};
        m2DistanceProxy target = m2GeometryProxy(&world->shapeGeometry[s]);
        m2DistanceProxy point;
        point.points[0] = local;
        point.count = 1;
        point.radius = 0.0f;
        m2DistanceResult d = m2ShapeDistance(&target, &point);
        float dist = d.distance - target.radius;
        if (dist > reach)
        {
            continue;
        }
        // Blast direction: away from the center. Deep overlap falls
        // back to the body-center direction; a dead-centered blast on
        // a centered body has no direction and skips, loudly fair.
        m2Vec2 dir;
        if (dist > 0.0f)
        {
            dir = (m2Vec2){-d.normal.x, -d.normal.y}; // normal points shape->center
        }
        else
        {
            m2Vec2 lc = world->localCenters[body];
            dir = (m2Vec2){lc.x - local.x, lc.y - local.y};
            float len = sqrtf(dir.x * dir.x + dir.y * dir.y);
            if (!(len > 0.0f))
            {
                continue;
            }
            dir = (m2Vec2){dir.x / len, dir.y / len};
            dist = 0.0f;
        }
        float scale = dist <= def->radius ? 1.0f : 1.0f - (dist - def->radius) / def->falloff;
        float mag = def->impulse * scale;
        m2Vec2 hitLocal = {d.pointA.x + target.radius * d.normal.x,
                           d.pointA.y + target.radius * d.normal.y};
        m2Vec2 lc = world->localCenters[body];
        m2Vec2 arm = {hitLocal.x - lc.x, hitLocal.y - lc.y};
        m2Vec2 impulseLocal = {mag * dir.x, mag * dir.y};
        // Rotate impulse and arm out to world axes for the velocity
        // update (angular uses the local cross, identical either way).
        m2Vec2 impulseWorld = {xf.q.c * impulseLocal.x - xf.q.s * impulseLocal.y,
                               xf.q.s * impulseLocal.x + xf.q.c * impulseLocal.y};
        world->linearVelocities[body].x += world->invMass[body] * impulseWorld.x;
        world->linearVelocities[body].y += world->invMass[body] * impulseWorld.y;
        world->angularVelocities[body] +=
            world->invInertia[body] * (arm.x * impulseLocal.y - arm.y * impulseLocal.x);
        world->asleep[body] = 0;
        world->sleepTimes[body] = 0.0f;
    }
    world->journalActive = journalWasActive;
}

// --- Shatter: the destruction road ------------------------------------------------

int32_t m2World_ShatterBody(m2BodyId bodyId, const m2Polygon* pieces, int32_t pieceCount,
                            m2BodyId* outBodies, int32_t capacity)
{
    m2World* world = m2WorldFromIndex(bodyId.world0);
    int32_t parent = world != NULL ? m2BodySlot(world, bodyId) : -1;
    if (parent < 0 || pieces == NULL || pieceCount < 1 || pieceCount > 64 ||
        world->types[parent] != (uint8_t)m2_dynamicBody)
    {
        m2Refuse(world, m2_errorInvalid);
        return 0;
    }
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        if (pieces[i].count < 3)
        {
            m2Refuse(world, m2_errorInvalid);
            return 0;
        }
    }
    if (world->freeCount < pieceCount || world->shapeFreeCount < pieceCount)
    {
        return 0; // cannot seat every piece: a runtime fact, all or nothing
    }

    // The parent's rigid field, sampled before anything moves.
    m2Transform xf = world->transforms[parent];
    m2Vec2 vParent = world->linearVelocities[parent];
    float wParent = world->angularVelocities[parent];
    m2Vec2 lcParent = world->localCenters[parent];
    m2Vec2 comArm = {xf.q.c * lcParent.x - xf.q.s * lcParent.y,
                     xf.q.s * lcParent.x + xf.q.c * lcParent.y};

    // Materials and filter ride from the parent's first shape; a
    // shapeless parent hands out defaults (documented).
    m2ShapeDef pieceShape = m2DefaultShapeDef();
    int32_t firstShape = world->bodyShapeHead[parent];
    if (firstShape != -1)
    {
        pieceShape.density = world->shapeDensity[firstShape];
        pieceShape.friction = world->shapeFriction[firstShape];
        pieceShape.restitution = world->shapeRestitution[firstShape];
        pieceShape.tangentSpeed = world->shapeTangentSpeed[firstShape];
        pieceShape.categoryBits = world->shapeCategory[firstShape];
        pieceShape.maskBits = world->shapeMask[firstShape];
        pieceShape.groupIndex = world->shapeGroup[firstShape];
    }

    // Ids carry the 1-based world index; the world keeps its generation.
    m2WorldId worldId = {bodyId.world0, world->worldGeneration};

    uint8_t journalWasActive = world->journalActive;
    world->journalActive = 0;

    int32_t firstIndex1 = 0;
    for (int32_t i = 0; i < pieceCount; ++i)
    {
        m2BodyDef bd = m2DefaultBodyDef();
        bd.type = m2_dynamicBody;
        bd.position = xf.p;
        bd.rotation = xf.q;
        bd.gravityScale = world->gravityScales[parent];
        bd.linearDamping = world->linearDampings[parent];
        bd.angularDamping = world->angularDampings[parent];
        m2BodyId piece = m2CreateBody(worldId, &bd);
        m2CreatePolygonShape(piece, &pieceShape, &pieces[i]);
        int32_t pieceIndex = piece.index1 - 1;
        // The rigid field at this piece's own center of mass.
        m2Vec2 lc = world->localCenters[pieceIndex];
        m2Vec2 arm = {xf.q.c * lc.x - xf.q.s * lc.y, xf.q.s * lc.x + xf.q.c * lc.y};
        float rx = arm.x - comArm.x;
        float ry = arm.y - comArm.y;
        world->linearVelocities[pieceIndex] =
            (m2Vec2){vParent.x - wParent * ry, vParent.y + wParent * rx};
        world->angularVelocities[pieceIndex] = wParent;
        if (i == 0)
        {
            firstIndex1 = piece.index1;
        }
        if (outBodies != NULL && i < capacity)
        {
            outBodies[i] = piece;
        }
    }
    m2DestroyBody(bodyId); // joints die with it, touching sleepers wake

    world->journalActive = journalWasActive;
    m2JournalRecordShatter(world, bodyId, pieces, pieceCount, firstIndex1);
    return pieceCount;
}
