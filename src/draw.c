// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Debug drawing: a read-only walk over the world that hands your
// renderer everything it needs. Colors encode state - static gray,
// awake tan, sleeping dim, sensors green, bullets orange - and every
// world position leaves here as f64 so the renderer can subtract the
// camera before dropping to floats.

#include "fpenv.h"
#include "world_internal.h"

#include "maul2d/base.h"

enum
{
    m2_colorStatic = 0x7f7f7f,
    m2_colorKinematic = 0x5f9fd8,
    m2_colorAwake = 0xd8b25f,
    m2_colorSleeping = 0x6f6a55,
    m2_colorSensor = 0x5fd88a,
    m2_colorBullet = 0xd8755f,
    m2_colorJoint = 0x9f5fd8,
    m2_colorContact = 0xd85f5f,
    m2_colorFriction = 0x5fd8d8,
    m2_colorAabb = 0x3f4f3f,
};

// The callbacks are host code: each runs in the host's floating-point
// environment, through a helper whose body is only that call.
static M2_NOINLINE void DrawPolygon(const m2DebugDraw* draw, const m2Vec2* vertices, int32_t count,
                                    m2Pos2 origin, m2Rot rotation, uint32_t color)
{
    m2FloatHost host = m2FloatHostEnter();
    draw->drawPolygon(vertices, count, origin, rotation, color, draw->context);
    m2FloatHostLeave(host);
}

static M2_NOINLINE void DrawCircle(const m2DebugDraw* draw, m2Pos2 center, float radius,
                                   m2Rot rotation, uint32_t color)
{
    m2FloatHost host = m2FloatHostEnter();
    draw->drawCircle(center, radius, rotation, color, draw->context);
    m2FloatHostLeave(host);
}

static M2_NOINLINE void DrawCapsule(const m2DebugDraw* draw, m2Pos2 p1, m2Pos2 p2, float radius,
                                    uint32_t color)
{
    m2FloatHost host = m2FloatHostEnter();
    draw->drawCapsule(p1, p2, radius, color, draw->context);
    m2FloatHostLeave(host);
}

static M2_NOINLINE void DrawSegment(const m2DebugDraw* draw, m2Pos2 p1, m2Pos2 p2, uint32_t color)
{
    m2FloatHost host = m2FloatHostEnter();
    draw->drawSegment(p1, p2, color, draw->context);
    m2FloatHostLeave(host);
}

static M2_NOINLINE void DrawPoint(const m2DebugDraw* draw, m2Pos2 p, float size, uint32_t color)
{
    m2FloatHost host = m2FloatHostEnter();
    draw->drawPoint(p, size, color, draw->context);
    m2FloatHostLeave(host);
}

static uint32_t BodyColor(const m2World* world, int32_t body)
{
    if (world->bodies.types[body] == (uint8_t)m2_staticBody)
    {
        return m2_colorStatic;
    }
    if (world->bodies.types[body] == (uint8_t)m2_kinematicBody)
    {
        return m2_colorKinematic;
    }
    if (world->bodies.asleep[body] != 0)
    {
        return m2_colorSleeping;
    }
    return world->bodies.bullets[body] != 0 ? m2_colorBullet : m2_colorAwake;
}

static m2Pos2 LocalToWorld(m2Transform xf, m2Vec2 local)
{
    m2Pos2 p;
    p.x = xf.p.x + (double)(xf.q.c * local.x - xf.q.s * local.y);
    p.y = xf.p.y + (double)(xf.q.s * local.x + xf.q.c * local.y);
    return p;
}

static void DrawShape(const m2World* world, const m2DebugDraw* draw, int32_t shape)
{
    int32_t body = world->shapes.shapeBody[shape];
    m2Transform xf = world->bodies.transforms[body];
    uint32_t color =
        world->shapes.shapeSensor[shape] != 0 ? m2_colorSensor : BodyColor(world, body);
    const m2ShapeGeometry* geometry = &world->shapes.shapeGeometry[shape];

    switch (geometry->type)
    {
    case m2_circleShape:
        if (draw->drawCircle != NULL)
        {
            DrawCircle(draw, LocalToWorld(xf, geometry->circle.center), geometry->circle.radius,
                       xf.q, color);
        }
        break;
    case m2_capsuleShape:
        if (draw->drawCapsule != NULL)
        {
            DrawCapsule(draw, LocalToWorld(xf, geometry->capsule.point1),
                        LocalToWorld(xf, geometry->capsule.point2), geometry->capsule.radius,
                        color);
        }
        break;
    case m2_segmentShape:
        if (draw->drawSegment != NULL)
        {
            DrawSegment(draw, LocalToWorld(xf, geometry->segment.point1),
                        LocalToWorld(xf, geometry->segment.point2), color);
        }
        break;
    case m2_chainSegmentShape:
        if (draw->drawSegment != NULL)
        {
            DrawSegment(draw, LocalToWorld(xf, geometry->chainSegment.segment.point1),
                        LocalToWorld(xf, geometry->chainSegment.segment.point2), color);
        }
        break;
    default:
        if (draw->drawPolygon != NULL)
        {
            DrawPolygon(draw, geometry->polygon.vertices, geometry->polygon.count, xf.p, xf.q,
                        color);
        }
        break;
    }
}

static void DrawJoint(const m2World* world, const m2DebugDraw* draw, int32_t j)
{
    uint8_t type = world->joints.jointType[j];
    m2Transform xfA = world->bodies.transforms[world->joints.jointBodyA[j]];
    m2Transform xfB = world->bodies.transforms[world->joints.jointBodyB[j]];
    if (type == (uint8_t)m2_filterJoint)
    {
        return; // a filter joint is the absence of contact: nothing to draw
    }
    if (type == (uint8_t)m2_gearJoint || type == (uint8_t)m2_ratchetJoint)
    {
        // The anchor slots carry phase-tracking rotation state, not
        // anchors; draw the coupling hub to hub.
        DrawSegment(draw, xfA.p, xfB.p, m2_colorJoint);
        return;
    }
    m2Pos2 a = LocalToWorld(xfA, world->joints.jointLocalAnchorA[j]);
    m2Pos2 b = LocalToWorld(xfB, world->joints.jointLocalAnchorB[j]);
    if (type == (uint8_t)m2_pulleyJoint)
    {
        // Two ropes up to the ground anchors and the crossbar between them.
        m2Pos2 ga = world->joints.jointTargets[j];
        m2Pos2 gb = world->joints.jointTargetsB[j];
        DrawSegment(draw, a, ga, m2_colorJoint);
        DrawSegment(draw, b, gb, m2_colorJoint);
        DrawSegment(draw, ga, gb, m2_colorJoint);
        a = ga;
        b = gb;
    }
    else
    {
        if (type == (uint8_t)m2_mouseJoint)
        {
            a = world->joints.jointTargets[j]; // the spring runs from the target
        }
        DrawSegment(draw, a, b, m2_colorJoint);
    }
    if (draw->drawPoint != NULL)
    {
        DrawPoint(draw, a, 4.0f, m2_colorJoint);
        DrawPoint(draw, b, 4.0f, m2_colorJoint);
    }
}

// The manifold of a touching, non-sensor pair and the transform of its
// first shape's body, or NULL.
static const m2Manifold* DrawnManifold(const m2World* world, int32_t pair, m2Transform* xfA)
{
    if (world->contacts.pairTouching[pair] == 0)
    {
        return NULL;
    }
    int32_t shapeA = (int32_t)(world->contacts.pairKeys[pair] >> 32);
    int32_t shapeB = (int32_t)(world->contacts.pairKeys[pair] & 0xFFFFFFFFu);
    if (world->shapes.shapeSensor[shapeA] != 0 || world->shapes.shapeSensor[shapeB] != 0)
    {
        return NULL;
    }
    *xfA = world->bodies.transforms[world->shapes.shapeBody[shapeA]];
    return &world->contacts.manifolds[pair];
}

static void DrawContactPoints(const m2World* world, const m2DebugDraw* draw)
{
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        m2Transform xfA;
        const m2Manifold* manifold = DrawnManifold(world, i, &xfA);
        for (int32_t k = 0; manifold != NULL && k < manifold->pointCount; ++k)
        {
            DrawPoint(draw, LocalToWorld(xfA, manifold->points[k].anchorA), 5.0f, m2_colorContact);
        }
    }
}

// An arrow per contact point: the normal impulse along the world normal,
// the friction impulse along the tangent. The stored impulses are the
// warm-start payload the last solve settled on.
static void DrawContactForces(const m2World* world, const m2DebugDraw* draw)
{
    float scale = draw->forceScale > 0.0f ? draw->forceScale : 1.0f;
    for (int32_t i = 0; i < world->contacts.pairCount; ++i)
    {
        m2Transform xfA;
        const m2Manifold* manifold = DrawnManifold(world, i, &xfA);
        if (manifold == NULL)
        {
            continue;
        }
        m2Vec2 n = {xfA.q.c * manifold->normal.x - xfA.q.s * manifold->normal.y,
                    xfA.q.s * manifold->normal.x + xfA.q.c * manifold->normal.y};
        m2Vec2 t = {-n.y, n.x};
        for (int32_t k = 0; k < manifold->pointCount; ++k)
        {
            m2Pos2 p = LocalToWorld(xfA, manifold->points[k].anchorA);
            float ni = manifold->points[k].normalImpulse * scale;
            m2Pos2 nEnd = {p.x + (double)(n.x * ni), p.y + (double)(n.y * ni)};
            DrawSegment(draw, p, nEnd, m2_colorContact);
            float ti = manifold->points[k].tangentImpulse * scale;
            m2Pos2 tEnd = {p.x + (double)(t.x * ti), p.y + (double)(t.y * ti)};
            DrawSegment(draw, p, tEnd, m2_colorFriction);
        }
    }
}

static void DrawAabbs(const m2World* world, const m2DebugDraw* draw)
{
    for (int32_t i = 0; i < world->shapes.maxShapeIndex; ++i)
    {
        if (world->shapes.shapeAlive[i] == 0 || world->broadphase.proxyIds[i] == M2_NULL_NODE)
        {
            continue;
        }
        const m2TreeNode* node =
            &world->broadphase.treeNodes[world->bodies.types[world->shapes.shapeBody[i]]]
                                        [world->broadphase.proxyIds[i]];
        m2Aabb box = node->aabb;
        m2Pos2 c1 = {box.lowerBound.x, box.lowerBound.y};
        m2Pos2 c2 = {box.upperBound.x, box.lowerBound.y};
        m2Pos2 c3 = {box.upperBound.x, box.upperBound.y};
        m2Pos2 c4 = {box.lowerBound.x, box.upperBound.y};
        DrawSegment(draw, c1, c2, m2_colorAabb);
        DrawSegment(draw, c2, c3, m2_colorAabb);
        DrawSegment(draw, c3, c4, m2_colorAabb);
        DrawSegment(draw, c4, c1, m2_colorAabb);
    }
}

static void DrawWorld(const m2World* world, const m2DebugDraw* draw);

static M2_NOINLINE void WorldDrawWorker(const m2World* world, const m2DebugDraw* draw)
{
    if (world == NULL || draw == NULL)
    {
        return;
    }
    // The callbacks are host code: while they run, the world's writers
    // refuse.
    m2EnterHostCode(world);
    DrawWorld(world, draw);
    m2LeaveHostCode(world);
}

void m2World_Draw(const m2World* world, const m2DebugDraw* draw)
{
    uint64_t env = m2FloatEnvEnter();
    WorldDrawWorker(world, draw);
    m2FloatEnvLeave(env);
}

// The walk itself: shapes, joints, contacts and proxies.
static void DrawWorld(const m2World* world, const m2DebugDraw* draw)
{
    for (int32_t i = 0; draw->drawShapes && i < world->shapes.maxShapeIndex; ++i)
    {
        if (world->shapes.shapeAlive[i] != 0)
        {
            DrawShape(world, draw, i);
        }
    }
    for (int32_t j = 0;
         draw->drawJoints && draw->drawSegment != NULL && j < world->joints.maxJointIndex; ++j)
    {
        if (world->joints.jointAlive[j] != 0)
        {
            DrawJoint(world, draw, j);
        }
    }
    if (draw->drawContacts && draw->drawPoint != NULL)
    {
        DrawContactPoints(world, draw);
    }
    if (draw->drawContactForces && draw->drawSegment != NULL)
    {
        DrawContactForces(world, draw);
    }
    if (draw->drawAabbs && draw->drawSegment != NULL)
    {
        DrawAabbs(world, draw);
    }
}
